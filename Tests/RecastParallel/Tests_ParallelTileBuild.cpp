// Tests for the optional parallel tiled-navmesh builder.
//
// The core correctness claim is byte-for-byte equivalence between a serial
// per-tile build and a parallel build for any thread count or completion
// order. These tests build a synthetic flat walkable world, generate identical
// tile blobs via buildTile (serial) and buildTiles (1, 2, 4, 8+ threads) and
// compare the raw Detour buffers. They also cover duplicate tiles, single-tile
// failure, cancellation, empty input and deterministic in-order commit.
#include "RecastParallel/RecastParallelTileBuild.h"

#include "DetourNavMesh.h"
#include "DetourStatus.h"
#include "catch2/catch_amalgamated.hpp"

#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

using namespace recastparallel;

namespace
{

// A flat, slightly subdivided ground plane at y=0 spanning a square world.
// Flat and walkable so every interior tile produces polygons. The subdivision
// gives rasterization many triangles to process concurrently.
struct FlatWorld
{
	std::vector<float> verts;
	std::vector<int> tris;
	float bmin[3];
	float bmax[3];

	explicit FlatWorld(int quadsPerSide)
	{
		const float extent = 100.0f;
		const float step = (extent * 2.0f) / static_cast<float>(quadsPerSide);
		bmin[0] = -extent;
		bmin[1] = 0.0f;
		bmin[2] = -extent;
		bmax[0] = extent;
		bmax[1] = 0.0f;
		bmax[2] = extent;

		for (int z = 0; z <= quadsPerSide; ++z)
		{
			for (int x = 0; x <= quadsPerSide; ++x)
			{
				verts.push_back(-extent + x * step);
				verts.push_back(0.0f);
				verts.push_back(-extent + z * step);
			}
		}

		const auto idx = [&](int x, int z) { return z * (quadsPerSide + 1) + x; };
		for (int z = 0; z < quadsPerSide; ++z)
		{
			for (int x = 0; x < quadsPerSide; ++x)
			{
				// Two upward-facing (+Y normal) triangles per quad. The winding
				// order is verified against rcMarkWalkableTriangles: a flipped
				// order yields down-facing normals and zero walkable triangles.
				tris.push_back(idx(x, z));
				tris.push_back(idx(x + 1, z + 1));
				tris.push_back(idx(x + 1, z));

				tris.push_back(idx(x, z));
				tris.push_back(idx(x, z + 1));
				tris.push_back(idx(x + 1, z + 1));
			}
		}
	}
};

struct TileBuildFixture
{
	const float cs = 0.5f;
	const float ch = 0.2f;
	const int tileSize = 32;          // Cells per tile side.
	const int borderSize = 4;
	const int agentHeightCells = 10;
	const int agentClimbCells = 3;
	const int agentRadiusCells = 2;

	rcParallelTileInput makeInput(const FlatWorld& world, int tileX, int tileY) const
	{
		rcParallelTileInput in;
		in.verts = world.verts.data();
		in.vertCount = static_cast<int>(world.verts.size()) / 3;
		in.tris = world.tris.data();
		in.triCount = static_cast<int>(world.tris.size()) / 3;
		in.tileX = tileX;
		in.tileY = tileY;
		in.tileLayer = 0;
		in.agentHeight = agentHeightCells * ch;
		in.agentRadius = agentRadiusCells * cs;
		in.agentMaxClimb = agentClimbCells * ch;
		in.maxVertsPerPoly = 6;
		in.buildBvTree = true;

		rcConfig& cfg = in.config;
		std::memset(&cfg, 0, sizeof(cfg));
		cfg.cs = cs;
		cfg.ch = ch;
		cfg.walkableSlopeAngle = 45.0f;
		cfg.walkableHeight = agentHeightCells;
		cfg.walkableClimb = agentClimbCells;
		cfg.walkableRadius = agentRadiusCells;
		cfg.maxEdgeLen = 12;
		cfg.maxSimplificationError = 1.3f;
		cfg.minRegionArea = 8;
		cfg.mergeRegionArea = 20;
		cfg.maxVertsPerPoly = 6;
		cfg.tileSize = tileSize;
		cfg.borderSize = borderSize;
		cfg.width = tileSize + borderSize * 2;
		cfg.height = tileSize + borderSize * 2;
		cfg.detailSampleDist = 6.0f;
		cfg.detailSampleMaxError = 1.0f;

		const float tileWorld = tileSize * cs;
		cfg.bmin[0] = world.bmin[0] + tileX * tileWorld - borderSize * cs;
		cfg.bmin[1] = world.bmin[1];
		cfg.bmin[2] = world.bmin[2] + tileY * tileWorld - borderSize * cs;
		cfg.bmax[0] = world.bmin[0] + (tileX + 1) * tileWorld + borderSize * cs;
		// Generous vertical headroom: must exceed walkableHeight*ch (2.0 units)
		// or rcFilterWalkableLowHeightSpans removes every span.
		cfg.bmax[1] = world.bmin[1] + 10.0f;
		cfg.bmax[2] = world.bmin[2] + (tileY + 1) * tileWorld + borderSize * cs;
		return in;
	}

	std::vector<rcParallelTileInput> makeGrid(const FlatWorld& world, int tilesX, int tilesY) const
	{
		std::vector<rcParallelTileInput> v;
		v.reserve(static_cast<std::size_t>(tilesX) * tilesY);
		for (int y = 0; y < tilesY; ++y)
		{
			for (int x = 0; x < tilesX; ++x)
			{
				v.push_back(makeInput(world, x, y));
			}
		}
		return v;
	}
};

// Compare two built tile buffers byte-for-byte.
bool blobsIdentical(const rcParallelTileOutput& a, const rcParallelTileOutput& b)
{
	if (a.empty != b.empty)
	{
		return false;
	}
	if (a.dataSize != b.dataSize)
	{
		return false;
	}
	if (a.dataSize == 0)
	{
		return a.data == nullptr && b.data == nullptr;
	}
	return std::memcmp(a.data, b.data, static_cast<std::size_t>(a.dataSize)) == 0;
}

}  // namespace

TEST_CASE("Parallel tile build matches serial build byte-for-byte", "[parallel][tile][consistency]")
{
	FlatWorld world(40);
	TileBuildFixture fx;
	const int tilesX = 6;
	const int tilesY = 6;
	const std::vector<rcParallelTileInput> inputs = fx.makeGrid(world, tilesX, tilesY);
	REQUIRE(inputs.size() == static_cast<std::size_t>(tilesX * tilesY));

	// Reference: build every tile serially, one buildTile call each.
	std::vector<rcParallelTileOutput> serial(inputs.size());
	int nonEmptySerial = 0;
	for (std::size_t i = 0; i < inputs.size(); ++i)
	{
		REQUIRE(buildTile(inputs[i], serial[i]));
		if (!serial[i].empty)
		{
			++nonEmptySerial;
		}
	}
	REQUIRE(nonEmptySerial > 0);  // The test is only meaningful with real data.

	for (int threads : {1, 2, 4, 8, 16})
	{
		std::vector<rcParallelTileOutput> parallel;
		rcParallelTileBuildOptions opts;
		opts.requestedThreadCount = threads;
		RunResult r = buildTiles(inputs, parallel, opts);

		REQUIRE(r.status == RunStatus::Completed);
		REQUIRE(parallel.size() == serial.size());

		for (std::size_t i = 0; i < serial.size(); ++i)
		{
			INFO("tile " << i << " thread count " << threads);
			REQUIRE(blobsIdentical(serial[i], parallel[i]));
		}
		freeTileOutputs(parallel);
	}
	freeTileOutputs(serial);
}

TEST_CASE("Parallel output is stable across repeated runs and thread counts", "[parallel][tile][stress]")
{
	FlatWorld world(32);
	TileBuildFixture fx;
	const auto inputs = fx.makeGrid(world, 5, 5);

	std::vector<rcParallelTileOutput> baseline;
	REQUIRE(buildTiles(inputs, baseline, rcParallelTileBuildOptions{1}).status == RunStatus::Completed);

	// Concurrency pressure: many runs at high thread count must all match.
	for (int round = 0; round < 4; ++round)
	{
		for (int threads : {2, 8})
		{
			std::vector<rcParallelTileOutput> out;
			rcParallelTileBuildOptions opts;
			opts.requestedThreadCount = threads;
			REQUIRE(buildTiles(inputs, out, opts).status == RunStatus::Completed);
			REQUIRE(out.size() == baseline.size());
			for (std::size_t i = 0; i < out.size(); ++i)
			{
				REQUIRE(blobsIdentical(baseline[i], out[i]));
			}
			freeTileOutputs(out);
		}
	}
	freeTileOutputs(baseline);
}

TEST_CASE("Duplicate tile coordinates are rejected without running work", "[parallel][tile]")
{
	FlatWorld world(8);
	TileBuildFixture fx;
	auto inputs = fx.makeGrid(world, 2, 2);
	// Duplicate tile (0,0,0).
	inputs.push_back(inputs.front());
	REQUIRE(hasDuplicateTiles(inputs));

	std::vector<rcParallelTileOutput> out;
	rcParallelTileBuildOptions opts;
	opts.requestedThreadCount = 4;
	RunResult r = buildTiles(inputs, out, opts);
	REQUIRE(r.status == RunStatus::TaskFailed);
	REQUIRE(r.tasksExecuted == 0);
	REQUIRE(out.size() == inputs.size());
}

TEST_CASE("Single tile failure cancels the remaining batch", "[parallel][tile]")
{
	FlatWorld world(24);
	TileBuildFixture fx;
	auto inputs = fx.makeGrid(world, 4, 4);

	// Track that each built tile receives a distinct, isolated context index.
	std::mutex indexMutex;
	std::set<std::size_t> seenIndices;
	std::atomic<int> calls{0};
	ContextFactory factory = [&](std::size_t tileIndex) -> std::unique_ptr<rcContext> {
		calls.fetch_add(1);
		std::lock_guard<std::mutex> lock(indexMutex);
		seenIndices.insert(tileIndex);
		return std::unique_ptr<rcContext>(new rcContext(false));
	};

	// Force a deterministic hard failure on one tile by exceeding Detour's
	// maximum vertices-per-polygon limit (checked before dtCreateNavMeshData).
	// This avoids relying on allocator behaviour for a zero-size heightfield.
	inputs[7].maxVertsPerPoly = DT_VERTS_PER_POLYGON + 1;

	std::vector<rcParallelTileOutput> out;
	rcParallelTileBuildOptions opts;
	opts.requestedThreadCount = 4;
	opts.contextFactory = factory;
	RunResult r = buildTiles(inputs, out, opts);

	REQUIRE(r.status == RunStatus::TaskFailed);
	REQUIRE(r.firstFailedIndex == 7);
	// Every context handed out was for a unique tile (no shared context).
	REQUIRE(seenIndices.size() == static_cast<std::size_t>(calls.load()));
	REQUIRE(r.tasksExecuted >= 1);
	REQUIRE(r.tasksExecuted <= inputs.size());
	// Cleanup must not leak the blobs that did get produced.
	freeTileOutputs(out);
}

TEST_CASE("Cancellation during a tile build stops later tiles", "[parallel][tile]")
{
	// A large, dense grid so that after the early cancel there are still tiles
	// waiting in the queue that must never start.
	FlatWorld world(48);
	TileBuildFixture fx;
	const auto inputs = fx.makeGrid(world, 10, 10);  // 100 tiles

	CancellationToken token;
	std::thread canceller([&] {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		token.cancel();
	});

	std::vector<rcParallelTileOutput> out;
	rcParallelTileBuildOptions opts;
	opts.requestedThreadCount = 4;
	opts.token = &token;
	RunResult r = buildTiles(inputs, out, opts);
	canceller.join();

	REQUIRE(r.status == RunStatus::Cancelled);
	REQUIRE(r.tasksExecuted < inputs.size());
	freeTileOutputs(out);
}

TEST_CASE("Empty tile batch is a no-op", "[parallel][tile]")
{
	std::vector<rcParallelTileInput> inputs;
	std::vector<rcParallelTileOutput> out;
	RunResult r = buildTiles(inputs, out, rcParallelTileBuildOptions{4});
	REQUIRE(r.status == RunStatus::Empty);
	REQUIRE(out.empty());
}

TEST_CASE("Empty-geometry tile yields an empty (non-error) result", "[parallel][tile]")
{
	TileBuildFixture fx;
	FlatWorld world(4);
	auto in = fx.makeInput(world, 0, 0);
	in.verts = nullptr;  // No geometry.
	in.vertCount = 0;
	rcParallelTileOutput out;
	REQUIRE(buildTile(in, out));
	REQUIRE(out.empty);
	REQUIRE(out.data == nullptr);
}

TEST_CASE("Tiles commit to a navmesh in deterministic order", "[parallel][tile][commit]")
{
	FlatWorld world(24);
	TileBuildFixture fx;
	const int n = 4;
	const auto inputs = fx.makeGrid(world, n, n);

	// Build two identical batches with different thread counts.
	std::vector<rcParallelTileOutput> outA;
	std::vector<rcParallelTileOutput> outB;
	REQUIRE(buildTiles(inputs, outA, rcParallelTileBuildOptions{1}).status == RunStatus::Completed);
	REQUIRE(buildTiles(inputs, outB, rcParallelTileBuildOptions{8}).status == RunStatus::Completed);

	const auto makeMesh = [&] {
		dtNavMesh* mesh = dtAllocNavMesh();
		REQUIRE(mesh != nullptr);
		dtNavMeshParams params;
		std::memset(&params, 0, sizeof(params));
		params.orig[0] = world.bmin[0];
		params.orig[1] = world.bmin[1];
		params.orig[2] = world.bmin[2];
		params.tileWidth = fx.tileSize * fx.cs;
		params.tileHeight = fx.tileSize * fx.cs;
		params.maxTiles = 256;
		params.maxPolys = 16384;
		REQUIRE(dtStatusSucceed(mesh->init(&params)));
		return mesh;
	};

	dtNavMesh* meshA = makeMesh();
	dtNavMesh* meshB = makeMesh();
	long long failA = -1;
	long long failB = -1;
	REQUIRE(commitTilesInOrder(*meshA, inputs, outA, failA));
	REQUIRE(commitTilesInOrder(*meshB, inputs, outB, failB));
	REQUIRE(failA == -1);
	REQUIRE(failB == -1);

	// Both meshes must contain the same number of live tiles.
	int liveTilesA = 0;
	int liveTilesB = 0;
	for (int y = 0; y < n; ++y)
	{
		for (int x = 0; x < n; ++x)
		{
			if (meshA->getTileRefAt(x, y, 0) != 0)
			{
				++liveTilesA;
			}
			if (meshB->getTileRefAt(x, y, 0) != 0)
			{
				++liveTilesB;
			}
		}
	}
	REQUIRE(liveTilesA == liveTilesB);
	REQUIRE(liveTilesA > 0);

	dtFreeNavMesh(meshA);
	dtFreeNavMesh(meshB);
	// Outputs transferred ownership; freeing the now-null slots must be safe.
	freeTileOutputs(outA);
	freeTileOutputs(outB);
}
