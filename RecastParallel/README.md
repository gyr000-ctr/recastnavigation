# RecastParallel — optional parallel tile builder

RecastParallel is an **opt-in** add-on that builds tiled Detour navigation
meshes across multiple worker threads. It does not modify any existing
`rcBuild*` / `dt*` public API, the default serial behaviour, or single-threaded
output. The core Recast and Detour libraries remain C++98, single-threaded and
byte-for-byte unchanged; all threading lives in this separate C++17 static
library.

## Enabling it

The component is **off by default**. Turn it on at configure time:

```sh
cmake -B build -DRECASTNAVIGATION_PARALLEL=ON
```

Then link the `RecastNavigation::RecastParallel` target (it pulls in Recast,
Detour and the platform thread library automatically).

## Why the output is deterministic

A single tile's build is a pure function of its `rcParallelTileInput`. The
builder restructures the serial `Sample_TileMesh::buildTileMesh()` pipeline so
that **every intermediate object is function-local**:

* `rcHeightfield`, `rcCompactHeightfield`, `rcContourSet`, `rcPolyMesh` and
  `rcPolyMeshDetail` are allocated per tile and released (RAII) before the
  build returns — including on every failure path.
* Each tile gets its **own `rcContext`** (the base `rcContext` is not
  thread-safe), via a caller-supplied `ContextFactory` or a built-in silent
  context.
* Tiles share only read-only input (vertex/index buffers and the config).

Consequently a tile's Detour blob depends only on its input geometry and
configuration — never on the thread count, scheduling or completion order.
`buildTiles()` writes each tile into its own pre-sized output slot (no shared
mutable state), and `commitTilesInOrder()` adds the blobs to a `dtNavMesh` on
the calling thread in strict submission order. A serial build and a parallel
build therefore produce **identical tile blobs and an identical navmesh**.

## Typical use

```cpp
#include "RecastParallel/RecastParallelTileBuild.h"

std::vector<recastparallel::rcParallelTileInput> inputs = ...; // one per tile
std::vector<recastparallel::rcParallelTileOutput> outputs;

recastparallel::rcParallelTileBuildOptions opts;
opts.requestedThreadCount = 8;                 // 0 = auto, 1 = serial
recastparallel::RunResult r =
    recastparallel::buildTiles(inputs, outputs, opts);

if (r.status == recastparallel::RunStatus::TaskFailed) {
    // r.firstFailedIndex identifies the failing tile; later tiles did not run.
}
recastparallel::commitTilesInOrder(*navMesh, inputs, outputs, firstFailed);
```

The generic `RecastParallelRunner.h` exposes the same runner
(`runTasks`, `CancellationToken`, `RunOptions`/`RunResult`) for any
embarrassingly-parallel workload, independent of navmesh code.

## Edge-case and degradation rules (all deterministic)

| Situation | Behaviour |
|-----------|-----------|
| `requestedThreadCount == 0` or negative | Treated as **auto**: use hardware concurrency. |
| `requestedThreadCount == 1` | Serial execution on the calling thread; no worker threads. |
| Value larger than hardware concurrency | Clamped to hardware concurrency (never oversubscribes). |
| Worker thread creation fails (`std::thread` throws) | Already-spawned workers join, remaining tasks run on the calling thread; `degradedToSerial == true`, `effectiveThreadCount == 1`. No task is lost or run twice. |
| Empty task/tile list | Returns `RunStatus::Empty`, no work performed. |
| Duplicate `(tileX, tileY, tileLayer)` | Rejected up-front (`TaskFailed`, `tasksExecuted == 0`) before any worker starts. |
| One tile/task fails | First failure index is recorded, cancellation is requested, no further tasks start; in-flight tasks finish. No deadlock (the calling thread also joins the pool). |
| Cancellation requested (external or mid-run) | Pending tasks do not start; result is `Cancelled`. Already-queued results stay valid. |
| A task throws | Caught and reported as a failure of that index; the process does not terminate. |

Cancellation is **cooperative**: requesting it stops new tasks from starting
but lets already-running tiles finish, so the function never returns while a
worker still touches caller memory. Every produced blob is owned by its output
slot; `freeTileOutputs()` releases them, and `commitTilesInOrder()` transfers
ownership to the navmesh (`DT_TILE_FREE_DATA`) without double-free.

## Measured performance

Synthetic flat world, 20×20 = 400 tiles (169 non-empty), release build on a
16-core x86-64 Linux host. Wall time is the median of 7 runs; peak live
allocation is the high-water mark of bytes outstanding through the Recast/Detour
allocators; peak RSS is from `/usr/bin/time -v`.

| Threads | Wall time (ms) | Speedup | Peak live alloc (KiB) | Peak RSS (KiB) |
|--------:|---------------:|--------:|----------------------:|---------------:|
| 1 (serial) | 48.0 | 1.0× | ~153 | ~5,018 |
| 4 | 12.9 | **3.7×** | ~360 | ~5,556 |
| 8 | 9.6 | **5.0×** | ~560 | ~5,748 |

Peak live memory scales roughly with the number of tiles being built
concurrently (each worker holds one tile's intermediates); total tile output
size is independent of thread count. All runs report zero outstanding bytes
after cleanup (no leaks), and the parallel blobs are byte-identical to the
serial blobs.

## Correctness verification

* Serial-vs-parallel byte-for-byte comparison across 1/2/4/8/16 threads.
* Repeated-run / high-thread-count concurrency stress.
* Duplicate-tile rejection, single-tile failure propagation, mid-run and
  pre-run cancellation, empty batch and empty-geometry handling.
* Deterministic in-order commit producing identical navmeshes.
* AddressSanitizer/UBSan (clean, no leaks) and ThreadSanitizer with the Recast,
  Detour and RecastParallel libraries all instrumented (zero data races).
* Forced thread-creation failure (`ulimit -u`) confirming serial degradation.
