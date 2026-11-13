//
// RecastParallel
//
// Optional, explicitly opt-in parallel task runner for Recast/Detour.
//
// This header does NOT change any existing rcBuild*/dt* public API. The core
// Recast and Detour libraries remain single-threaded, C++98 and byte-for-byte
// unchanged. Parallel execution is only available to callers that explicitly
// include this header and link the RecastParallel target.
//
// Design goals (see RecastParallelTileBuild.h for the navmesh tile wrapper):
//  * Deterministic: results are identical for any thread count and any
//    completion order. The runner only schedules; ordering of the produced
//    data is the caller's responsibility (the tile helper commits in input
//    order).
//  * Isolated: tasks never share mutable state inside the runner.
//  * Cooperative cancellation: requesting cancel stops further tasks from
//    starting; in-flight tasks run to completion. No deadlock, no leak.
//  * Deterministic degradation: thread counts of 0, negatives or values that
//    exceed the available hardware clamp to a supported value; if no worker
//    thread can be created the work runs serially on the calling thread.
//
#ifndef RECASTPARALLEL_RUNNER_H
#define RECASTPARALLEL_RUNNER_H

#include <atomic>
#include <cstddef>
#include <functional>
#include <memory>

namespace recastparallel
{

/// Reason a parallel run finished. The enum value alone fully describes the
/// outcome; it does not depend on how many threads were used or which task
/// failed first.
enum class RunStatus
{
	/// Every task completed successfully.
	Completed,
	/// A task reported failure. The index of the first failing task (in
	/// submission order) is recorded separately.
	TaskFailed,
	/// Cancellation was requested before all tasks could run.
	Cancelled,
	/// The input contained no work.
	Empty,
};

/// Cooperative cancellation flag. A task receives a const reference to this
/// object and may poll `isCancellationRequested()`. The flag is fully
/// thread-safe and can be tripped from any thread, including from inside a
/// task.
class CancellationToken
{
public:
	CancellationToken() : m_flag(false) {}

	/// Request cancellation. Safe to call from any thread, including from a
	/// worker task. Idempotent.
	void cancel() { m_flag.store(true, std::memory_order_release); }

	/// @return True once cancellation has been requested.
	bool isCancellationRequested() const { return m_flag.load(std::memory_order_acquire); }

	/// Reset the token so it can be reused for a new run.
	void reset() { m_flag.store(false, std::memory_order_release); }

private:
	std::atomic<bool> m_flag;
};

/// A unit of work. It receives the task's submission index and the active
/// cancellation token. Returning false marks the task failed and requests
/// cancellation of the remaining tasks. Throwing is not required (the core
/// libraries are built without exceptions); the runner also defensively
/// catches exceptions if a caller's task uses them.
using Task = std::function<bool(std::size_t index, const CancellationToken& token)>;

/// Configuration for a parallel run.
struct RunOptions
{
	/// Requested number of worker threads.
	///   0  -> auto: use hardware concurrency (falls back to 1 if unknown).
	///   1  -> serial execution on the calling thread (no worker threads).
	///  <0  -> treated as auto (deterministic correction of a bad value).
	///  >hardware concurrency -> clamped to the hardware concurrency.
	/// An oversized value can never create more threads than the machine has.
	int requestedThreadCount = 0;

	/// Optional cancellation token. If null, an internal token is created for
	/// the duration of the run.
	CancellationToken* token = nullptr;
};

/// Outcome of a parallel run.
struct RunResult
{
	RunStatus status = RunStatus::Empty;

	/// Number of tasks that actually executed (regardless of success).
	std::size_t tasksExecuted = 0;

	/// Submission-order index of the first task that failed, or -1.
	long long firstFailedIndex = -1;

	/// Number of worker threads actually used. Always >= 1. A value of 1 means
	/// the work ran serially (either requested, or after degradation).
	int effectiveThreadCount = 1;

	/// True when the requested configuration could not be honoured and the
	/// runner degraded to serial execution. Callers can surface this to detect
	/// a "thread pool unavailable" environment.
	bool degradedToSerial = false;
};

/// @return The number of hardware threads available, or 1 if it cannot be
/// determined. Never returns 0 or a negative value.
int hardwareConcurrency();

/// Resolve a caller-supplied thread count to the number of threads the runner
/// will actually attempt to use, applying the deterministic clamping rules
/// documented on RunOptions. Exposed for testing and for callers that want to
/// report the resolved value before running.
/// @param requestedThreadCount  Raw caller value (0 = auto, negatives = auto).
int resolveThreadCount(int requestedThreadCount);

/// Run @p taskCount copies of @p task concurrently.
///
/// The @p task callback is invoked once per index in [0, taskCount). It must
/// be self-contained: any mutable data it touches has to be either local to
/// the invocation or externally synchronised by the caller. The runner itself
/// shares no mutable state between invocations.
///
/// On failure or cancellation no further tasks are started, but already
/// running tasks are allowed to finish. The function blocks until every
/// worker has quiesced, so it can never return while a task still touches
/// caller memory (no use-after-join, no deadlock).
RunResult runTasks(std::size_t taskCount, const Task& task, const RunOptions& options = RunOptions());

}  // namespace recastparallel

#endif  // RECASTPARALLEL_RUNNER_H
