//
// RecastParallel
//
// Optional parallel tiled-navmesh builder.
//
// This is an additive, opt-in convenience layer. It calls only the existing,
// unchanged public Recast (rcBuild*) and Detour (dtCreateNavMeshData)
// functions. No existing API, default serial behaviour or single-threaded
// output is modified.
//
// Determinism guarantee
// ---------------------
// The per-tile build is a pure function of its rcParallelTileInput. Every
// intermediate Recast object (heightfield, compact heightfield, contour set,
// poly mesh, detail mesh) and every rcContext is local to a single tile
// invocation, so two tiles never share mutable state. The produced Detour
// tile blob therefore depends only on the input geometry/config, never on the
// thread count, scheduling or completion order. Tiles are committed to the
// dtNavMesh on the calling thread in strict submission order, so the final
// assembled navmesh (and every tile blob) is byte-for-byte identical to a
// serial build.
//
#ifndef RECASTPARALLEL_TILE_BUILD_H
#define RECASTPARALLEL_TILE_BUILD_H

#include "RecastParallel/RecastParallelRunner.h"

#include "Recast.h"

#include <functional>
#include <memory>
#include <vector>

class dtNavMesh;

namespace recastparallel
{

/// Input describing one independent tile build. All pointers must remain
/// valid for the duration of the build; the data is read-only and may be
/// shared concurrently between tiles (e.g. a shared vertex buffer).
struct rcParallelTileInput
{
	// --- Source geometry (read-only, shareable across threads) ---
	const float* verts = nullptr;      // [(x,y,z) * vertCount]
	int vertCount = 0;
	const int* tris = nullptr;         // [(a,b,c) * triCount]
	int triCount = 0;

	// --- Recast configuration for this tile ---
	// A fully populated rcConfig (width/height/cs/ch/bmin/bmax/borderSize/...).
	// Typically the same template for every tile with bmin/bmax adjusted per
	// tile, exactly as the serial Sample_TileMesh pipeline does.
	rcConfig config;

	// --- Detour tile placement ---
	int tileX = 0;
	int tileY = 0;
	int tileLayer = 0;
	unsigned int userId = 0;

	// Agent dimensions copied into dtNavMeshCreateParams.
	float agentHeight = 0.0f;
	float agentRadius = 0.0f;
	float agentMaxClimb = 0.0f;

	/// Build a bounding-volume tree inside the tile data. Matches the tiled
	/// sample, which sets buildBvTree = true.
	bool buildBvTree = true;

	/// Maximum vertices per polygon; must be <= DT_VERTS_PER_POLYGON.
	int maxVertsPerPoly = 6;
};

/// Output of one tile build.
struct rcParallelTileOutput
{
	int tileX = 0;
	int tileY = 0;
	int tileLayer = 0;

	/// Detour tile data allocated with dtAlloc(), or null if the tile produced
	/// no walkable polygons (a normal, non-error empty result). Owned by this
	/// struct until released or handed to dtNavMesh::addTile.
	unsigned char* data = nullptr;
	int dataSize = 0;

	/// True when this tile genuinely had no geometry to build. This is not a
	/// failure.
	bool empty = false;
};

/// Factory that produces a fresh rcContext for a tile. Called once per tile on
/// the thread that builds that tile, so each tile gets its own context (the
/// base rcContext is not thread-safe). Returning nullptr selects a built-in
/// silent context. The returned context is owned and deleted by the builder.
using ContextFactory = std::function<std::unique_ptr<rcContext>(std::size_t tileIndex)>;

/// Build a single tile synchronously. All intermediate objects are local and
/// freed before returning, even on failure. This is the exact same sequence of
/// public rcBuild*/dt calls the serial sample performs; running it on multiple
/// threads is safe because nothing here touches shared mutable state.
///
/// @param[in]  input   Tile geometry and configuration.
/// @param[out] output  Result; on success output.data is non-null unless the
///                     tile is empty (output.empty == true).
/// @param[in]  factory Optional per-tile context factory.
/// @return True on success (including an empty tile); false on a hard failure.
bool buildTile(const rcParallelTileInput& input, rcParallelTileOutput& output, const ContextFactory& factory = nullptr);

/// Options controlling a parallel batch of tile builds. Mirrors RunOptions.
struct rcParallelTileBuildOptions
{
	/// See RunOptions::requestedThreadCount. 0 = auto, 1 = serial.
	int requestedThreadCount = 0;

	/// Optional externally controlled cancellation.
	CancellationToken* token = nullptr;

	/// Optional per-tile context factory.
	ContextFactory contextFactory = nullptr;
};

/// Build many tiles in parallel. Each tile writes only to its own slot in
/// @p outputs (pre-sized to inputs.size()), so worker tasks share no mutable
/// state. Nothing is added to a dtNavMesh here; the caller commits the blobs
/// deterministically afterwards (see commitTilesInOrder).
///
/// Duplicate tile coordinates are rejected up-front without running any work.
/// On the first tile failure or cancellation, no further tiles are started and
/// the error is returned. Every blob that was produced is owned by its output
/// slot; buildTiles never leaks (use freeTileOutput / the bulk free helper).
///
/// @param[in]  inputs   One entry per tile, in the desired commit order.
/// @param[out] outputs  Resized to inputs.size(); slot i corresponds to input i.
/// @param[in]  options  Thread count, cancellation and context factory.
/// @return A RunResult describing the deterministic outcome.
RunResult buildTiles(const std::vector<rcParallelTileInput>& inputs,
                     std::vector<rcParallelTileOutput>& outputs,
                     const rcParallelTileBuildOptions& options = rcParallelTileBuildOptions());

/// Free the Detour buffer held by an output slot and reset it. Safe on empty
/// slots (no-op).
void freeTileOutput(rcParallelTileOutput& output);

/// Free every non-empty output slot.
void freeTileOutputs(std::vector<rcParallelTileOutput>& outputs);

/// Commit already-built tile blobs to a navmesh in submission order. This runs
/// entirely on the calling thread and is therefore deterministic: identical
/// blobs committed in the same order always yield an identical navmesh,
/// independent of how the blobs were produced.
///
/// On success each blob's ownership is transferred to the navmesh with
/// DT_TILE_FREE_DATA. If addTile fails the blob is freed here so nothing leaks.
///
/// @param[out] navMesh      Initialised navmesh to receive the tiles.
/// @param[in]  inputs       The same inputs passed to buildTiles (for coords).
/// @param[in]  outputs      The blobs produced by buildTiles.
/// @param[out] firstFailed  Submission index of the first failed addTile, -1 if all ok.
/// @return True if every non-empty tile was added successfully.
bool commitTilesInOrder(dtNavMesh& navMesh,
                        const std::vector<rcParallelTileInput>& inputs,
                        std::vector<rcParallelTileOutput>& outputs,
                        long long& firstFailed);

/// @return True if two tile coordinate lists contain a repeated (x,y,layer).
bool hasDuplicateTiles(const std::vector<rcParallelTileInput>& inputs);

}  // namespace recastparallel

#endif  // RECASTPARALLEL_TILE_BUILD_H
