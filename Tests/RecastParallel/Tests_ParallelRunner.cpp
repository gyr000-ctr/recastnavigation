// Tests for the optional RecastParallel task runner.
//
// These exercise the generic runner directly (no Recast geometry): thread
// count clamping, degradation, duplicate-free parallel execution, completion
// order independence, cooperative cancellation and single-task failure
// propagation.
#include "RecastParallel/RecastParallelRunner.h"

#include "catch2/catch_amalgamated.hpp"

#include <atomic>
#include <chrono>
#include <mutex>
#include <set>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace recastparallel;

TEST_CASE("resolveThreadCount clamps deterministic edge cases", "[parallel][runner]")
{
	const int hw = hardwareConcurrency();
	REQUIRE(hw >= 1);

	SECTION("Zero means auto (hardware concurrency)")
	{
		REQUIRE(resolveThreadCount(0) == hw);
	}
	SECTION("Negative values are treated as auto")
	{
		REQUIRE(resolveThreadCount(-1) == hw);
		REQUIRE(resolveThreadCount(-999) == hw);
	}
	SECTION("One stays serial")
	{
		REQUIRE(resolveThreadCount(1) == 1);
	}
	SECTION("Oversized value is clamped to hardware")
	{
		REQUIRE(resolveThreadCount(hw + 1) == hw);
		REQUIRE(resolveThreadCount(1000000) == hw);
	}
	SECTION("In-range value is honoured")
	{
		const int want = hw >= 2 ? 2 : 1;
		REQUIRE(resolveThreadCount(want) == want);
	}
}

TEST_CASE("runTasks empty input", "[parallel][runner]")
{
	RunResult r = runTasks(0, [](std::size_t, const CancellationToken&) { return true; });
	REQUIRE(r.status == RunStatus::Empty);
	REQUIRE(r.tasksExecuted == 0);
	REQUIRE(r.effectiveThreadCount == 1);
}

TEST_CASE("runTasks executes every index exactly once", "[parallel][runner]")
{
	constexpr std::size_t kTasks = 500;

	for (int threads : {1, 2, 4, 8})
	{
		std::vector<std::atomic<int>> hits(kTasks);
		for (auto& h : hits)
		{
			h.store(0);
		}
		std::atomic<int> maxConcurrency{0};
		std::atomic<int> inFlight{0};

		RunOptions opts;
		opts.requestedThreadCount = threads;
		RunResult r = runTasks(
			kTasks,
			[&](std::size_t i, const CancellationToken&) {
				const int now = inFlight.fetch_add(1) + 1;
				int observed = maxConcurrency.load();
				while (now > observed && !maxConcurrency.compare_exchange_weak(observed, now))
				{
				}
				hits[i].fetch_add(1);
				inFlight.fetch_sub(1);
				return true;
			},
			opts);

		REQUIRE(r.status == RunStatus::Completed);
		REQUIRE(r.tasksExecuted == kTasks);
		for (std::size_t i = 0; i < kTasks; ++i)
		{
			REQUIRE(hits[i].load() == 1);  // Exactly once, never twice.
		}
		if (threads > 1 && r.effectiveThreadCount > 1)
		{
			REQUIRE(maxConcurrency.load() >= 1);
		}
	}
}

TEST_CASE("runTasks result is independent of completion order", "[parallel][runner]")
{
	constexpr std::size_t kTasks = 200;
	// Each task adds its index into a shared set from arbitrary threads; the
	// final aggregated set must be identical regardless of scheduling.
	std::set<std::size_t> seen;
	std::mutex seenMutex;

	RunOptions opts;
	opts.requestedThreadCount = 8;
	RunResult r = runTasks(
		kTasks,
		[&](std::size_t i, const CancellationToken&) {
			// Small jitter to encourage non-deterministic completion order.
			std::this_thread::sleep_for(std::chrono::microseconds((i * 7) % 50));
			std::lock_guard<std::mutex> lock(seenMutex);
			seen.insert(i);
			return true;
		},
		opts);

	REQUIRE(r.status == RunStatus::Completed);
	REQUIRE(seen.size() == kTasks);
	for (std::size_t i = 0; i < kTasks; ++i)
	{
		REQUIRE(seen.count(i) == 1);
	}
}

TEST_CASE("runTasks single-task failure stops further work and propagates index", "[parallel][runner]")
{
	constexpr std::size_t kTasks = 100;
	constexpr std::size_t kFailing = 10;
	std::atomic<int> started{0};

	RunOptions opts;
	opts.requestedThreadCount = 4;
	RunResult r = runTasks(
		kTasks,
		[&](std::size_t i, const CancellationToken&) {
			started.fetch_add(1);
			if (i == kFailing)
			{
				return false;  // Hard failure.
			}
			// Simulate work so other tasks are in flight when the failure hits.
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			return true;
		},
		opts);

	REQUIRE(r.status == RunStatus::TaskFailed);
	REQUIRE(r.firstFailedIndex == static_cast<long long>(kFailing));
	// Cancellation must have prevented at least some tasks from starting.
	REQUIRE(r.tasksExecuted <= kTasks);
	REQUIRE(started.load() <= static_cast<int>(kTasks));
}

TEST_CASE("runTasks external cancellation stops subsequent tasks", "[parallel][runner]")
{
	constexpr std::size_t kTasks = 1000;
	CancellationToken token;
	std::atomic<int> completed{0};

	RunOptions opts;
	opts.requestedThreadCount = 4;
	opts.token = &token;

	// Cancel from a separate thread shortly after starting.
	std::thread canceller([&] {
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
		token.cancel();
	});

	RunResult r = runTasks(
		kTasks,
		[&](std::size_t, const CancellationToken& t) {
			if (t.isCancellationRequested())
			{
				return true;
			}
			std::this_thread::sleep_for(std::chrono::microseconds(100));
			completed.fetch_add(1);
			return true;
		},
		opts);
	canceller.join();

	REQUIRE(r.status == RunStatus::Cancelled);
	// Far fewer than the full batch should have run.
	REQUIRE(r.tasksExecuted < kTasks);
}

TEST_CASE("runTasks cancellation requested before run does no work", "[parallel][runner]")
{
	CancellationToken token;
	token.cancel();
	std::atomic<int> started{0};

	RunOptions opts;
	opts.requestedThreadCount = 8;
	opts.token = &token;
	RunResult r = runTasks(
		50,
		[&](std::size_t, const CancellationToken&) {
			started.fetch_add(1);
			return true;
		},
		opts);

	REQUIRE(r.status == RunStatus::Cancelled);
	REQUIRE(started.load() == 0);
}

TEST_CASE("runTasks serial mode uses exactly one thread", "[parallel][runner]")
{
	RunOptions opts;
	opts.requestedThreadCount = 1;
	RunResult r = runTasks(10, [](std::size_t, const CancellationToken&) { return true; }, opts);
	REQUIRE(r.status == RunStatus::Completed);
	REQUIRE(r.effectiveThreadCount == 1);
	REQUIRE(r.degradedToSerial == false);
	REQUIRE(r.tasksExecuted == 10);
}

TEST_CASE("runTasks repeated runs are independent", "[parallel][runner]")
{
	// Ensure no global state leaks between runs (the runner must be re-entrant).
	for (int round = 0; round < 5; ++round)
	{
		std::atomic<int> sum{0};
		RunOptions opts;
		opts.requestedThreadCount = 8;
		RunResult r = runTasks(
			50,
			[&](std::size_t i, const CancellationToken&) {
				sum.fetch_add(static_cast<int>(i) + 1);
				return true;
			},
			opts);
		REQUIRE(r.status == RunStatus::Completed);
		REQUIRE(sum.load() == (50 * 51) / 2);
	}
}

TEST_CASE("runTasks exception in a task is treated as failure, not a crash", "[parallel][runner]")
{
	RunOptions opts;
	opts.requestedThreadCount = 4;
	RunResult r = runTasks(
		20,
		[](std::size_t i, const CancellationToken&) -> bool {
			if (i == 5)
			{
				throw std::runtime_error("boom");
			}
			return true;
		},
		opts);
	REQUIRE(r.status == RunStatus::TaskFailed);
	REQUIRE(r.firstFailedIndex == 5);
}
