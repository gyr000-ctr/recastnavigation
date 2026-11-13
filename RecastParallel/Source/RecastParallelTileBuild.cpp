//
// RecastParallel
//
// Implementation of the optional parallel tiled-navmesh builder.
//
// The per-tile pipeline below is the same sequence of public Recast and
// Detour calls used by the serial Sample_TileMesh::buildTileMesh(). It is
// restructured so that every intermediate object is a function-local RAII
// value rather than shared member state. Combined with a per-tile rcContext,
// this makes concurrent invocations independent and their output identical
// regardless of thread count or scheduling.
//
#include "RecastParallel/RecastParallelTileBuild.h"

#include "DetourAlloc.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"

#include <cstring>
#include <new>

namespace recastparallel
{

namespace
{

// Minimal context that discards log/timer output. Each tile gets its own
// instance, so there is no shared mutable context between threads.
class SilentContext : public rcContext
{
public:
	SilentContext() : rcContext(false) {}
};
// RAII guards for the Recast intermediate objects. Keeping them local and
// automatically freed guarantees no leak on any early-return failure path and
// full isolation between tiles.
template <typename T, void (*Free)(T*)>
struct RcGuard
{
	T* ptr;
	explicit RcGuard(T* p = nullptr) : ptr(p) {}
	~RcGuard() { reset(); }
	void reset(T* p = nullptr)
	{
		if (ptr)
		{
			Free(ptr);
		}
		ptr = p;
	}
	RcGuard(const RcGuard&) = delete;
	RcGuard& operator=(const RcGuard&) = delete;
};

using HeightfieldGuard = RcGuard<rcHeightfield, rcFreeHeightField>;
using CompactHeightfieldGuard = RcGuard<rcCompactHeightfield, rcFreeCompactHeightfield>;
using ContourSetGuard = RcGuard<rcContourSet, rcFreeContourSet>;
using PolyMeshGuard = RcGuard<rcPolyMesh, rcFreePolyMesh>;
using PolyMeshDetailGuard = RcGuard<rcPolyMeshDetail, rcFreePolyMeshDetail>;

struct TriAreaGuard
{
	unsigned char* p;
	explicit TriAreaGuard(unsigned char* q = nullptr) : p(q) {}
	~TriAreaGuard() { delete[] p; }
	TriAreaGuard(const TriAreaGuard&) = delete;
	TriAreaGuard& operator=(const TriAreaGuard&) = delete;
};

}  // namespace

bool buildTileImpl(const rcParallelTileInput& in, rcParallelTileOutput& out, const ContextFactory& factory,
                   std::size_t tileIndex)
{
	out = rcParallelTileOutput();
	out.tileX = in.tileX;
	out.tileY = in.tileY;
	out.tileLayer = in.tileLayer;

	if (!in.verts || in.vertCount <= 0 || !in.tris || in.triCount <= 0)
	{
		// Nothing to rasterize: a well-defined empty result, not an error.
		out.empty = true;
		return true;
	}

	// Every tile gets an independent context. The factory (if provided) runs on
	// this thread and returns a context nobody else can reach.
	std::unique_ptr<rcContext> owned;
	rcContext* ctx = nullptr;
	if (factory)
	{
		owned = factory(tileIndex);
	}
	if (!owned)
	{
		owned.reset(new (std::nothrow) SilentContext());
	}
	if (!owned)
	{
		return false;
	}
	ctx = owned.get();

	const rcConfig& cfg = in.config;

	// --- 1. Heightfield (local) ---
	HeightfieldGuard solid(rcAllocHeightfield());
	if (!solid.ptr)
	{
		return false;
	}
	if (!rcCreateHeightfield(ctx, *solid.ptr, cfg.width, cfg.height, cfg.bmin, cfg.bmax, cfg.cs, cfg.ch))
	{
		return false;
	}

	TriAreaGuard triareas(new (std::nothrow) unsigned char[static_cast<std::size_t>(in.triCount)]);
	if (!triareas.p)
	{
		return false;
	}
	std::memset(triareas.p, 0, static_cast<std::size_t>(in.triCount));

	rcMarkWalkableTriangles(
		ctx, cfg.walkableSlopeAngle, in.verts, in.vertCount, in.tris, in.triCount, triareas.p);

	if (!rcRasterizeTriangles(
			ctx, in.verts, in.vertCount, in.tris, triareas.p, in.triCount, *solid.ptr, cfg.walkableClimb))
	{
		return false;
	}

	// --- 2. Filters (operate only on the local heightfield) ---
	rcFilterLowHangingWalkableObstacles(ctx, cfg.walkableClimb, *solid.ptr);
	rcFilterLedgeSpans(ctx, cfg.walkableHeight, cfg.walkableClimb, *solid.ptr);
	rcFilterWalkableLowHeightSpans(ctx, cfg.walkableHeight, *solid.ptr);

	// --- 3. Compact heightfield (local) ---
	CompactHeightfieldGuard chf(rcAllocCompactHeightfield());
	if (!chf.ptr)
	{
		return false;
	}
	if (!rcBuildCompactHeightfield(ctx, cfg.walkableHeight, cfg.walkableClimb, *solid.ptr, *chf.ptr))
	{
		return false;
	}
	if (!rcErodeWalkableArea(ctx, cfg.walkableRadius, *chf.ptr))
	{
		return false;
	}

	// Monotone partitioning is used because it is fully deterministic and has
	// no distance-field randomness, making byte-identical cross-thread output
	// straightforward to reason about. It needs only the local compact field.
	if (!rcBuildRegionsMonotone(ctx, *chf.ptr, cfg.borderSize, cfg.minRegionArea, cfg.mergeRegionArea))
	{
		return false;
	}

	// --- 4. Contours (local) ---
	ContourSetGuard cset(rcAllocContourSet());
	if (!cset.ptr)
	{
		return false;
	}
	if (!rcBuildContours(ctx, *chf.ptr, cfg.maxSimplificationError, cfg.maxEdgeLen, *cset.ptr))
	{
		return false;
	}
	if (cset.ptr->nconts == 0)
	{
		// No walkable surface in this tile: valid empty result.
		out.empty = true;
		return true;
	}

	// --- 5. Polygon mesh (local) ---
	PolyMeshGuard pmesh(rcAllocPolyMesh());
	if (!pmesh.ptr)
	{
		return false;
	}
	if (!rcBuildPolyMesh(ctx, *cset.ptr, in.maxVertsPerPoly, *pmesh.ptr))
	{
		return false;
	}

	// --- 6. Detail mesh (local) ---
	PolyMeshDetailGuard dmesh(rcAllocPolyMeshDetail());
	if (!dmesh.ptr)
	{
		return false;
	}
	if (!rcBuildPolyMeshDetail(ctx, *pmesh.ptr, *chf.ptr, cfg.detailSampleDist, cfg.detailSampleMaxError, *dmesh.ptr))
	{
		return false;
	}

	if (pmesh.ptr->nverts >= 0xffff)
	{
		// Detour stores vertex indices in ushorts.
		return false;
	}
	if (in.maxVertsPerPoly > DT_VERTS_PER_POLYGON)
	{
		return false;
	}

	// --- 7. Detour tile blob (the only object that outlives this call) ---
	dtNavMeshCreateParams params;
	std::memset(&params, 0, sizeof(params));
	params.verts = pmesh.ptr->verts;
	params.vertCount = pmesh.ptr->nverts;
	params.polys = pmesh.ptr->polys;
	params.polyAreas = pmesh.ptr->areas;
	params.polyFlags = pmesh.ptr->flags;
	params.polyCount = pmesh.ptr->npolys;
	params.nvp = pmesh.ptr->nvp;
	params.detailMeshes = dmesh.ptr->meshes;
	params.detailVerts = dmesh.ptr->verts;
	params.detailVertsCount = dmesh.ptr->nverts;
	params.detailTris = dmesh.ptr->tris;
	params.detailTriCount = dmesh.ptr->ntris;
	params.userId = in.userId;
	params.tileX = in.tileX;
	params.tileY = in.tileY;
	params.tileLayer = in.tileLayer;
	rcVcopy(params.bmin, pmesh.ptr->bmin);
	rcVcopy(params.bmax, pmesh.ptr->bmax);
	params.walkableHeight = in.agentHeight;
	params.walkableRadius = in.agentRadius;
	params.walkableClimb = in.agentMaxClimb;
	params.cs = cfg.cs;
	params.ch = cfg.ch;
	params.buildBvTree = in.buildBvTree;

	unsigned char* navData = nullptr;
	int navDataSize = 0;
	if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
	{
		return false;
	}

	out.data = navData;
	out.dataSize = navDataSize;
	out.empty = false;
	return true;
}

bool buildTile(const rcParallelTileInput& in, rcParallelTileOutput& out, const ContextFactory& factory)
{
	return buildTileImpl(in, out, factory, 0);
}

bool hasDuplicateTiles(const std::vector<rcParallelTileInput>& inputs)
{
	// O(n^2) is fine: tile counts are modest and this keeps the check free of
	// any global map state. Compares full identity including layer.
	for (std::size_t i = 0; i < inputs.size(); ++i)
	{
		for (std::size_t j = i + 1; j < inputs.size(); ++j)
		{
			if (inputs[i].tileX == inputs[j].tileX && inputs[i].tileY == inputs[j].tileY &&
			    inputs[i].tileLayer == inputs[j].tileLayer)
			{
				return true;
			}
		}
	}
	return false;
}

RunResult buildTiles(const std::vector<rcParallelTileInput>& inputs,
                     std::vector<rcParallelTileOutput>& outputs,
                     const rcParallelTileBuildOptions& options)
{
	outputs.clear();
	outputs.resize(inputs.size());

	if (inputs.empty())
	{
		RunResult r;
		r.status = RunStatus::Empty;
		r.effectiveThreadCount = 1;
		return r;
	}

	// Reject duplicates deterministically before touching any worker. Building
	// the same coordinate twice would make the final addTile order ambiguous.
	if (hasDuplicateTiles(inputs))
	{
		RunResult r;
		r.status = RunStatus::TaskFailed;
		r.tasksExecuted = 0;
		r.firstFailedIndex = -1;
		r.effectiveThreadCount = 1;
		r.degradedToSerial = false;
		return r;
	}

	RunOptions runOptions;
	runOptions.requestedThreadCount = options.requestedThreadCount;
	runOptions.token = options.token;

	// Each task writes only its own output slot and owns all intermediates;
	// the shared input vector and geometry buffers are read-only.
	const Task task = [&](std::size_t index, const CancellationToken& token) -> bool {
		// Honor a cancellation that raced ahead of this task being scheduled.
		if (token.isCancellationRequested())
		{
			return true;  // Not a failure; run will report Cancelled.
		}
		return buildTileImpl(inputs[index], outputs[index], options.contextFactory, index);
	};

	return runTasks(inputs.size(), task, runOptions);
}

void freeTileOutput(rcParallelTileOutput& output)
{
	if (output.data)
	{
		dtFree(output.data);
		output.data = nullptr;
		output.dataSize = 0;
	}
	output.empty = false;
}

void freeTileOutputs(std::vector<rcParallelTileOutput>& outputs)
{
	for (rcParallelTileOutput& o : outputs)
	{
		freeTileOutput(o);
	}
}

bool commitTilesInOrder(dtNavMesh& navMesh,
                        const std::vector<rcParallelTileInput>& inputs,
                        std::vector<rcParallelTileOutput>& outputs,
                        long long& firstFailed)
{
	firstFailed = -1;
	const std::size_t count = inputs.size() < outputs.size() ? inputs.size() : outputs.size();

	for (std::size_t i = 0; i < count; ++i)
	{
		rcParallelTileOutput& o = outputs[i];
		if (o.empty || !o.data || o.dataSize <= 0)
		{
			continue;
		}

		unsigned char* blob = o.data;
		// Clear the slot's ownership first so a later freeTileOutputs() call
		// cannot double-free: on success the navmesh owns the blob (it will
		// release it via DT_TILE_FREE_DATA), on failure we free it below.
		o.data = nullptr;

		const dtStatus status = navMesh.addTile(blob, o.dataSize, DT_TILE_FREE_DATA, 0, nullptr);
		if (dtStatusFailed(status))
		{
			dtFree(blob);
			o.dataSize = 0;
			if (firstFailed < 0)
			{
				firstFailed = static_cast<long long>(i);
			}
			return false;
		}
		o.dataSize = 0;
	}
	return true;
}

}  // namespace recastparallel
