//
// RecastParallel
//
// Implementation of the optional parallel task runner. See
// RecastParallelRunner.h for the contract and guarantees.
//
#include "RecastParallel/RecastParallelRunner.h"

#include <thread>
#include <vector>

namespace recastparallel
{

int hardwareConcurrency()
{
	const unsigned int n = std::thread::hardware_concurrency();
	// hardware_concurrency() is allowed to return 0 when the value is not
	// computable; treat that as a single available thread.
	return n == 0 ? 1 : static_cast<int>(n);
}

int resolveThreadCount(int requestedThreadCount)
{
	const int hw = hardwareConcurrency();

	// 0 (auto) or any negative (invalid) value: use all available hardware
	// threads. This is the deterministic correction for 0 / bad inputs.
	if (requestedThreadCount <= 0)
	{
		return hw;
	}

	// A positive value larger than the machine is clamped down; we never try
	// to oversubscribe beyond hardware concurrency.
	if (requestedThreadCount > hw)
	{
		return hw;
	}

	return requestedThreadCount;
}

namespace
{

// Shared, lock-free coordination state for one run. Everything here is
// stack/object local to a single runTasks() call, so distinct runs never
// interfere and there is nothing global to leak or contend on.
struct RunState
{
	std::size_t taskCount;
	const Task* task;
	CancellationToken* token;

	// Next not-yet-claimed submission index. Workers atomically fetch-add, so
	// tasks are pulled in a data-race-free way regardless of thread count.
	std::atomic<std::size_t> nextIndex;
	// Number of tasks that have started executing.
	std::atomic<std::size_t> executed;
	// First failing index (submission order), -1 while nobody has failed.
	std::atomic<long long> firstFailure;

	RunState(std::size_t count, const Task& t, CancellationToken* tok)
		: taskCount(count), task(&t), token(tok), nextIndex(0), executed(0), firstFailure(-1)
	{}

	void requestCancel()
	{
		if (token)
		{
			token->cancel();
		}
	}

	// Worker loop shared by spawned threads and the calling thread.
	void workerLoop()
	{
		while (true)
		{
			// Stop pulling new work once cancel/failure has been observed.
			if (token && token->isCancellationRequested())
			{
				return;
			}

			const std::size_t index = nextIndex.fetch_add(1, std::memory_order_relaxed);
			if (index >= taskCount)
			{
				return;  // Every task has been claimed.
			}

			executed.fetch_add(1, std::memory_order_relaxed);

			bool ok = false;
			try
			{
				ok = (*task)(index, *token);
			}
			catch (...)
			{
				ok = false;  // Core libs do not throw, but stay robust if a caller does.
			}

			if (!ok)
			{
				// Record only the first failure and trip cancellation so no
				// further tasks start. compare_exchange keeps the winner
				// independent of thread scheduling.
				long long expected = -1;
				firstFailure.compare_exchange_strong(
					expected, static_cast<long long>(index), std::memory_order_acq_rel);
				requestCancel();
				// Continue the loop: we may still claim already-released work
				// only if not cancelled, but cancel is now set so the top
				// guard will return. In-flight work on other threads finishes.
			}
		}
	}
};

}  // namespace

RunResult runTasks(std::size_t taskCount, const Task& task, const RunOptions& options)
{
	RunResult result;

	if (taskCount == 0)
	{
		result.status = RunStatus::Empty;
		result.effectiveThreadCount = 1;
		result.degradedToSerial = false;
		return result;
	}

	// Resolve the caller-supplied thread count deterministically.
	const int resolved = resolveThreadCount(options.requestedThreadCount);

	// Use the caller's token when supplied, otherwise own an internal one for
	// the lifetime of this run. A caller-owned token must never be reset here.
	const bool externalToken = (options.token != nullptr);
	CancellationToken internalToken;
	RunState state(taskCount, task, options.token ? options.token : &internalToken);

	// Serial fast path: one thread means no spawn, no join, no pool. This is
	// also the guaranteed-degradation target if threading cannot be used.
	if (resolved <= 1)
	{
		state.workerLoop();
		result.effectiveThreadCount = 1;
		result.degradedToSerial = (options.requestedThreadCount > 1);
	}
	else
	{
		// Spawn resolved-1 helpers; the calling thread participates as the
		// final worker so a run with N threads uses at most N OS threads.
		std::vector<std::thread> workers;
		workers.reserve(static_cast<std::size_t>(resolved - 1));

		bool spawnFailed = false;
		try
		{
			for (int i = 0; i < resolved - 1; ++i)
			{
				workers.emplace_back([&state] { state.workerLoop(); });
			}
		}
		catch (...)
		{
			// Thread pool unavailable (resource limit / disabled threads).
			// Degrade deterministically: ask whatever spawned to finish, then
			// run the remainder on the calling thread.
			spawnFailed = true;
			state.requestCancel();
			for (std::thread& t : workers)
			{
				if (t.joinable())
				{
					t.join();
				}
			}
			// Clear the internal cancellation (used only to stop the partially
			// spawned workers) so the calling thread drains all remaining tasks
			// serially; the partial work already done is valid and need not be
			// redone. A caller-owned token is never reset: if the caller asked
			// to cancel, that request is honoured and the run ends cancelled.
			if (!externalToken)
			{
				internalToken.reset();
			}
			state.workerLoop();
		}

		if (!spawnFailed)
		{
			// Calling thread joins the pool, then waits for every helper.
			state.workerLoop();
			for (std::thread& t : workers)
			{
				if (t.joinable())
				{
					t.join();
				}
			}
		}

		result.effectiveThreadCount = spawnFailed ? 1 : resolved;
		result.degradedToSerial = spawnFailed;
	}

	result.tasksExecuted = state.executed.load(std::memory_order_relaxed);
	result.firstFailedIndex = state.firstFailure.load(std::memory_order_acquire);

	const bool cancelled = state.token && state.token->isCancellationRequested();
	if (result.firstFailedIndex >= 0)
	{
		result.status = RunStatus::TaskFailed;
	}
	else if (cancelled)
	{
		result.status = RunStatus::Cancelled;
	}
	else
	{
		result.status = RunStatus::Completed;
	}

	return result;
}

}  // namespace recastparallel
