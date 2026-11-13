#include "Recast.h"
#include "RecastVolume.h"
#include "catch2/catch_amalgamated.hpp"

#include "Tests_RecastVolumeHelpers.h"

#include <string.h>

long long CountingAllocator::liveAllocations = 0;
long long CountingAllocator::totalAllocations = 0;

namespace
{
	constexpr int VOXEL_H = 20;
}

TEST_CASE("rvlBuildNavVolume: basic free space and solid spans",
           "[recast][volume]")
{
	rcContext ctx(false);
	rcHeightfield hf;
	REQUIRE(makeHeightfield(ctx, hf, 4, 4, VOXEL_H));

	// One solid block in a single column.
	REQUIRE(addSolid(ctx, hf, 1, 1, 5, 8));

	rvlConfig cfg;
	rvlNavVolume* vol = 0;
	rvlStatus st = rvlBuildNavVolume(&ctx, hf, cfg, &vol);
	REQUIRE(rvlStatusSucceed(st));
	REQUIRE(vol != nullptr);

	// 15 empty columns have one interval each; the blocked column is split
	// into two intervals (below and above the solid span).
	REQUIRE(vol->width == 4);
	REQUIRE(vol->height == 4);
	REQUIRE(vol->maxVoxelY == VOXEL_H);
	REQUIRE(vol->intervalCount == 15 * 1 + 2);

	const rvlCell& blocked = vol->cells[1 + 1 * 4];
	REQUIRE(blocked.count == 2);
	int first = blocked.first;
	REQUIRE(vol->intervals[first].ymin == 0);
	REQUIRE(vol->intervals[first].ymax == 5);
	REQUIRE(vol->intervals[first + 1].ymin == 8);
	REQUIRE(vol->intervals[first + 1].ymax == VOXEL_H);

	rvlFreeNavVolume(&vol);
}

TEST_CASE("rvlBuildNavVolume: layered platforms with vertical connections",
           "[recast][volume][layered]")
{
	rcContext ctx(false);
	rcHeightfield hf;
	REQUIRE(makeHeightfield(ctx, hf, 6, 6, VOXEL_H));

	// Two solid platform slabs at different heights, each covering part of
	// the world. Free space exists below, between and above them.
	for (int x = 0; x < 3; ++x)
		for (int z = 0; z < 6; ++z)
			REQUIRE(addSolid(ctx, hf, x, z, 10, 12));
	for (int x = 3; x < 6; ++x)
		for (int z = 0; z < 6; ++z)
			REQUIRE(addSolid(ctx, hf, x, z, 4, 6));

	rvlConfig cfg;
	rvlNavVolume* vol = 0;
	REQUIRE(rvlStatusSucceed(rvlBuildNavVolume(&ctx, hf, cfg, &vol)));

	// Every column contains two free intervals: below the slab and above it.
	for (int c = 0; c < 36; ++c)
		REQUIRE(vol->cells[c].count == 2);

	// Vertical links connect the lower and upper interval inside each column.
	int verticalLinks = 0;
	for (int i = 0; i < vol->intervalCount; ++i)
		if (vol->intervals[i].links[5] != RVOL_NULL) ++verticalLinks;
	REQUIRE(verticalLinks == 36);

	// Horizontal links bridge the height step across x=2 -> x=3 where the
	// upper intervals overlap in the 10..12 band region of free voxels.
	int crossLinks = 0;
	for (int z = 0; z < 6; ++z)
	{
		const rvlCell& left = vol->cells[2 + z * 6];
		const rvlCell& right = vol->cells[3 + z * 6];
		// Upper interval of the left column links toward +x.
		int leftUpper = left.first + 1;
		if (vol->intervals[leftUpper].links[1] != RVOL_NULL) ++crossLinks;
		(void)right;
	}
	REQUIRE(crossLinks == 6);

	// A 3D path can climb from the low side's lower layer to the high side.
	rvlVolumeQuery* q = 0;
	REQUIRE(rvlStatusSucceed(rvlAllocVolumeQuery(&q)));
	REQUIRE(rvlStatusSucceed(rvlInitVolumeQuery(*q, *vol, vol->intervalCount)));

	int lowStart = vol->cells[4 + 4 * 6].first;       // lower interval, high slab side
	int highGoal = vol->cells[1 + 1 * 6].first + 1;   // upper interval, low slab side
	int path[64];
	int pathCount = 64;
	REQUIRE(rvlStatusSucceed(
		rvlFindPath(*q, lowStart, highGoal, path, &pathCount, 64)));
	REQUIRE(pathCount >= 2);
	REQUIRE(path[0] == lowStart);
	REQUIRE(path[pathCount - 1] == highGoal);

	rvlFreeVolumeQuery(&q);
	rvlFreeNavVolume(&vol);
}

TEST_CASE("rvlBuildNavVolume: floating corridor connects stacked open layers",
           "[recast][volume][layered]")
{
	rcContext ctx(false);
	rcHeightfield hf;
	REQUIRE(makeHeightfield(ctx, hf, 5, 5, VOXEL_H));

	// A slab at y=8..10 across most of the world splits it into a ground
	// layer and an upper layer. A one-cell-wide shaft stays open so the two
	// layers remain connected through a vertical "floating corridor".
	for (int x = 0; x < 5; ++x)
	{
		for (int z = 0; z < 5; ++z)
		{
			if (x == 2 && z == 2) continue; // open shaft
			REQUIRE(addSolid(ctx, hf, x, z, 8, 10));
		}
	}

	rvlConfig cfg;
	rvlNavVolume* vol = 0;
	REQUIRE(rvlStatusSucceed(rvlBuildNavVolume(&ctx, hf, cfg, &vol)));

	// The shaft column keeps a single continuous interval; others have two.
	REQUIRE(vol->cells[2 + 2 * 5].count == 1);
	int twoLayerCols = 0;
	for (int c = 0; c < 25; ++c)
		if (vol->cells[c].count == 2) ++twoLayerCols;
	REQUIRE(twoLayerCols == 24);

	rvlVolumeQuery* q = 0;
	REQUIRE(rvlStatusSucceed(rvlAllocVolumeQuery(&q)));
	REQUIRE(rvlStatusSucceed(rvlInitVolumeQuery(*q, *vol, vol->intervalCount)));

	// From a ground-layer interval to an upper-layer interval: the route must
	// pass through the shaft's continuous interval.
	int ground = vol->cells[0].first;
	int top = vol->cells[4 + 4 * 5].first + 1;
	int path[64];
	int pathCount = 64;
	REQUIRE(rvlStatusSucceed(rvlFindPath(*q, ground, top, path, &pathCount, 64)));

	bool passedShaft = false;
	int shaftInterval = vol->cells[2 + 2 * 5].first;
	for (int i = 0; i < pathCount; ++i)
		if (path[i] == shaftInterval) passedShaft = true;
	REQUIRE(passedShaft);

	rvlFreeVolumeQuery(&q);
	rvlFreeNavVolume(&vol);
}

TEST_CASE("rvlRaycast: thin one-voxel wall blocks the ray",
           "[recast][volume][thinwall]")
{
	rcContext ctx(false);
	rcHeightfield hf;
	REQUIRE(makeHeightfield(ctx, hf, 8, 3, VOXEL_H));

	// A full-height single-column wall at x=4 (every z column). One voxel
	// thick: rays must still stop exactly on its face.
	for (int z = 0; z < 3; ++z)
		REQUIRE(addSolid(ctx, hf, 4, z, 0, VOXEL_H));

	rvlConfig cfg;
	rvlNavVolume* vol = 0;
	REQUIRE(rvlStatusSucceed(rvlBuildNavVolume(&ctx, hf, cfg, &vol)));

	rvlVolumeQuery* q = 0;
	REQUIRE(rvlStatusSucceed(rvlAllocVolumeQuery(&q)));
	REQUIRE(rvlStatusSucceed(rvlInitVolumeQuery(*q, *vol, vol->intervalCount)));

	float start[3] = {1.0f, 5.0f, 1.0f};
	float end[3] = {7.0f, 5.0f, 1.0f};
	float t = 0;
	float hit[3];
	float normal[3];
	REQUIRE(rvlStatusSucceed(
		rvlRaycast(*q, start, end, t, hit, normal)));

	// Hit lies on the wall's near face x=4 and never crosses it.
	REQUIRE(hit[0] == Catch::Approx(4.0f).margin(0.001f));
	REQUIRE(hit[0] <= 4.0001f);
	REQUIRE(t < 1.0f);
	REQUIRE(normal[0] == Catch::Approx(-1.0f));
	REQUIRE(normal[1] == Catch::Approx(0.0f));
	REQUIRE(normal[2] == Catch::Approx(0.0f));

	// A ray parallel to and clear of the wall reaches its destination.
	float sideStart[3] = {1.0f, 5.0f, 1.0f};
	float sideEnd[3] = {3.5f, 5.0f, 1.0f};
	REQUIRE(rvlStatusSucceed(
		rvlRaycast(*q, sideStart, sideEnd, t, hit, normal)));
	REQUIRE(t == Catch::Approx(1.0f));

	rvlFreeVolumeQuery(&q);
	rvlFreeNavVolume(&vol);
}

TEST_CASE("rvlRaycast: open world boundary stops the ray",
           "[recast][volume][openboundary]")
{
	rcContext ctx(false);
	rcHeightfield hf;
	REQUIRE(makeHeightfield(ctx, hf, 4, 4, VOXEL_H));
	// Entirely empty world: every ray eventually leaves the volume.

	rvlConfig cfg;
	rvlNavVolume* vol = 0;
	REQUIRE(rvlStatusSucceed(rvlBuildNavVolume(&ctx, hf, cfg, &vol)));

	rvlVolumeQuery* q = 0;
	REQUIRE(rvlStatusSucceed(rvlAllocVolumeQuery(&q)));
	REQUIRE(rvlStatusSucceed(rvlInitVolumeQuery(*q, *vol, vol->intervalCount)));

	float start[3] = {2.0f, 5.0f, 2.0f};
	float end[3] = {20.0f, 5.0f, 2.0f};
	float t = 0;
	float hit[3];
	float normal[3];
	REQUIRE(rvlStatusSucceed(
		rvlRaycast(*q, start, end, t, hit, normal)));
	// Boundary face is x=4.
	REQUIRE(hit[0] == Catch::Approx(4.0f).margin(0.05f));
	REQUIRE(t < 1.0f);
	REQUIRE(normal[0] == Catch::Approx(-1.0f));

	// A ray starting outside the volume fails rather than crashing.
	float outside[3] = {-5.0f, 5.0f, 2.0f};
	REQUIRE(rvlStatusFailed(rvlRaycast(*q, outside, end, t, hit, normal)));

	rvlFreeVolumeQuery(&q);
	rvlFreeNavVolume(&vol);
}

TEST_CASE("rvlFindPath: no solution across a sealed full-height wall",
           "[recast][volume][nosolution]")
{
	rcContext ctx(false);
	rcHeightfield hf;
	REQUIRE(makeHeightfield(ctx, hf, 6, 6, VOXEL_H));

	// Seal the world into two halves with a full-height wall along x=3.
	for (int z = 0; z < 6; ++z)
		REQUIRE(addSolid(ctx, hf, 3, z, 0, VOXEL_H));

	rvlConfig cfg;
	rvlNavVolume* vol = 0;
	REQUIRE(rvlStatusSucceed(rvlBuildNavVolume(&ctx, hf, cfg, &vol)));

	rvlVolumeQuery* q = 0;
	REQUIRE(rvlStatusSucceed(rvlAllocVolumeQuery(&q)));
	REQUIRE(rvlStatusSucceed(rvlInitVolumeQuery(*q, *vol, vol->intervalCount)));

	int left = vol->cells[0].first;
	int right = vol->cells[5].first;
	int path[64];
	int pathCount = 64;
	rvlStatus st = rvlFindPath(*q, left, right, path, &pathCount, 64);
	REQUIRE(rvlStatusFailed(st));
	REQUIRE(rvlStatusDetail(st, RVOL_NO_PATH));

	// Nearest query outside any search window also fails cleanly.
	int idx = RVOL_NULL;
	float farCenter[3] = {100.0f, 5.0f, 100.0f};
	float extents[3] = {0.5f, 0.5f, 0.5f};
	REQUIRE(rvlStatusFailed(
		rvlFindNearest(*q, farCenter, extents, idx, 0)));
	REQUIRE(idx == RVOL_NULL);

	rvlFreeVolumeQuery(&q);
	rvlFreeNavVolume(&vol);
}

TEST_CASE("rvlBuildNavVolume: oversized and malformed worlds fail gracefully",
           "[recast][volume][hugeworld]")
{
	rcContext ctx(false);

	SECTION("Cell cap exceeded")
	{
		rcHeightfield hf;
		REQUIRE(makeHeightfield(ctx, hf, 8, 8, VOXEL_H));
		rvlConfig cfg;
		cfg.maxCells = 10; // 64 columns > cap
		rvlNavVolume* vol = 0;
		rvlStatus st = rvlBuildNavVolume(&ctx, hf, cfg, &vol);
		REQUIRE(rvlStatusFailed(st));
		REQUIRE(vol == nullptr);
	}

	SECTION("Interval cap exceeded")
	{
		rcHeightfield hf;
		REQUIRE(makeHeightfield(ctx, hf, 4, 4, VOXEL_H));
		// Alternating thin spans produce many free gaps.
		for (int x = 0; x < 4; ++x)
			for (int z = 0; z < 4; ++z)
				for (int y = 1; y < VOXEL_H; y += 2)
					REQUIRE(addSolid(ctx, hf, x, z, y, y + 1));
		rvlConfig cfg;
		cfg.maxIntervals = 4;
		rvlNavVolume* vol = 0;
		REQUIRE(rvlStatusFailed(rvlBuildNavVolume(&ctx, hf, cfg, &vol)));
		REQUIRE(vol == nullptr);
	}

	SECTION("Invalid arguments")
	{
		rcHeightfield hf;
		REQUIRE(makeHeightfield(ctx, hf, 2, 2, VOXEL_H));
		rvlConfig cfg;
		rvlNavVolume* vol = 0;
		cfg.minFreeHeight = 0;
		REQUIRE(rvlStatusFailed(rvlBuildNavVolume(&ctx, hf, cfg, &vol)));
		REQUIRE(rvlStatusFailed(rvlBuildNavVolume(&ctx, hf, cfg, 0)));
	}
}

TEST_CASE("rvlVolumeQuery: per-frame queries allocate nothing",
           "[recast][volume][noalloc]")
{
	rcContext ctx(false);
	rcHeightfield hf;
	REQUIRE(makeHeightfield(ctx, hf, 6, 6, VOXEL_H));
	for (int z = 0; z < 6; ++z)
		REQUIRE(addSolid(ctx, hf, 3, z, 0, VOXEL_H / 2));

	rvlNavVolume* vol = 0;
	rvlVolumeQuery* q = 0;

	CountingAllocator::install();
	{
		rvlConfig cfg;
		REQUIRE(rvlStatusSucceed(rvlBuildNavVolume(&ctx, hf, cfg, &vol)));
		REQUIRE(rvlStatusSucceed(rvlAllocVolumeQuery(&q)));
		REQUIRE(rvlStatusSucceed(rvlInitVolumeQuery(*q, *vol, vol->intervalCount)));

		const long long baseline = CountingAllocator::totalAllocations;

		// Repeated queries of every kind must not touch the allocator.
		for (int iter = 0; iter < 10; ++iter)
		{
			int idx = RVOL_NULL;
			float center[3] = {1.5f, 5.0f, 1.5f};
			float extents[3] = {2.0f, 2.0f, 2.0f};
			REQUIRE(rvlStatusSucceed(rvlFindNearest(*q, center, extents, idx, 0)));

			float s[3] = {1.0f, 5.0f, 1.0f};
			float e[3] = {2.0f, 8.0f, 2.0f};
			float t;
			REQUIRE(rvlStatusSucceed(rvlRaycast(*q, s, e, t, 0, 0)));

			int path[64];
			int pathCount = 64;
			int a = vol->cells[0].first;
			int b = vol->cells[5 + 5 * 6].first;
			rvlFindPath(*q, a, b, path, &pathCount, 64);
		}

		REQUIRE(CountingAllocator::totalAllocations == baseline);
	}
	CountingAllocator::uninstall();

	rvlFreeVolumeQuery(&q);
	rvlFreeNavVolume(&vol);
}

TEST_CASE("rvlSerialization: round trip preserves the volume and rejects garbage",
           "[recast][volume][serialization]")
{
	rcContext ctx(false);
	rcHeightfield hf;
	REQUIRE(makeHeightfield(ctx, hf, 5, 5, VOXEL_H));
	for (int x = 0; x < 3; ++x)
		for (int z = 0; z < 5; ++z)
			REQUIRE(addSolid(ctx, hf, x, z, 6, 9));

	rvlConfig cfg;
	rvlNavVolume* vol = 0;
	REQUIRE(rvlStatusSucceed(rvlBuildNavVolume(&ctx, hf, cfg, &vol)));

	const int size = rvlSerializedSize(*vol);
	REQUIRE(size > 64); // header is a fixed packed block larger than 64 bytes
	std::vector<unsigned char> buffer(size);
	REQUIRE(rvlStatusSucceed(
		rvlSerializeNavVolume(*vol, buffer.data(), (int)buffer.size())));

	// Undersized buffer is rejected.
	REQUIRE(rvlStatusDetail(
		rvlSerializeNavVolume(*vol, buffer.data(), size - 1),
		RVOL_BUFFER_TOO_SMALL));

	rvlNavVolume* loaded = 0;
	REQUIRE(rvlStatusSucceed(
		rvlDeserializeNavVolume(buffer.data(), (int)buffer.size(), &loaded)));

	REQUIRE(loaded->width == vol->width);
	REQUIRE(loaded->height == vol->height);
	REQUIRE(loaded->maxVoxelY == vol->maxVoxelY);
	REQUIRE(loaded->intervalCount == vol->intervalCount);
	REQUIRE(loaded->cs == Catch::Approx(vol->cs));
	REQUIRE(loaded->ch == Catch::Approx(vol->ch));
	for (int i = 0; i < 3; ++i)
	{
		REQUIRE(loaded->bmin[i] == Catch::Approx(vol->bmin[i]));
		REQUIRE(loaded->bmax[i] == Catch::Approx(vol->bmax[i]));
	}
	REQUIRE(memcmp(loaded->cells, vol->cells,
	               sizeof(rvlCell) * vol->width * vol->height) == 0);
	REQUIRE(memcmp(loaded->intervals, vol->intervals,
	               sizeof(rvlInterval) * vol->intervalCount) == 0);

	// Corrupt a payload byte: checksum validation must reject it.
	std::vector<unsigned char> corrupted = buffer;
	corrupted[size - 1] ^= 0xff;
	rvlNavVolume* bad = 0;
	REQUIRE(rvlStatusFailed(
		rvlDeserializeNavVolume(corrupted.data(), (int)corrupted.size(), &bad)));
	REQUIRE(bad == nullptr);

	// Bad magic and truncated streams are rejected.
	std::vector<unsigned char> wrongMagic = buffer;
	wrongMagic[0] = 'X';
	REQUIRE(rvlStatusDetail(
		rvlDeserializeNavVolume(wrongMagic.data(), (int)wrongMagic.size(), &bad),
		RVOL_WRONG_MAGIC));
	REQUIRE(rvlStatusFailed(
		rvlDeserializeNavVolume(buffer.data(), 8, &bad)));

	rvlFreeNavVolume(&loaded);
	rvlFreeNavVolume(&vol);
}

TEST_CASE("rvlFindNearest: clamps into the closest free interval",
           "[recast][volume]")
{
	rcContext ctx(false);
	rcHeightfield hf;
	REQUIRE(makeHeightfield(ctx, hf, 4, 4, VOXEL_H));
	REQUIRE(addSolid(ctx, hf, 0, 0, 0, VOXEL_H)); // fully solid column

	rvlConfig cfg;
	rvlNavVolume* vol = 0;
	REQUIRE(rvlStatusSucceed(rvlBuildNavVolume(&ctx, hf, cfg, &vol)));

	rvlVolumeQuery* q = 0;
	REQUIRE(rvlStatusSucceed(rvlAllocVolumeQuery(&q)));
	REQUIRE(rvlStatusSucceed(rvlInitVolumeQuery(*q, *vol, vol->intervalCount)));

	// A point over the solid column clamps to the neighbouring free column.
	float center[3] = {0.5f, 5.0f, 0.5f};
	float extents[3] = {2.0f, 2.0f, 2.0f};
	int idx = RVOL_NULL;
	float nearest[3];
	REQUIRE(rvlStatusSucceed(
		rvlFindNearest(*q, center, extents, idx, nearest)));
	REQUIRE(idx != RVOL_NULL);
	REQUIRE(nearest[0] >= 1.0f); // clamped out of the solid x=0 column

	rvlFreeVolumeQuery(&q);
	rvlFreeNavVolume(&vol);
}

TEST_CASE("rvlFindPath: long corridor beyond a fixed stack buffer",
           "[recast][volume][longpath]")
{
	// A 300-cell long 1D corridor produces a path longer than the 256-entry
	// fixed buffer the reconstruction used to rely on. Reconstruction now
	// reuses query scratch, so the full path must come back.
	constexpr int W = 300;
	rcContext ctx(false);
	rcHeightfield hf;
	REQUIRE(makeHeightfield(ctx, hf, W, 1, VOXEL_H));

	rvlConfig cfg;
	rvlNavVolume* vol = 0;
	REQUIRE(rvlStatusSucceed(rvlBuildNavVolume(&ctx, hf, cfg, &vol)));
	REQUIRE(vol->intervalCount == W); // one open interval per column

	rvlVolumeQuery* q = 0;
	REQUIRE(rvlStatusSucceed(rvlAllocVolumeQuery(&q)));
	REQUIRE(rvlStatusSucceed(rvlInitVolumeQuery(*q, *vol, vol->intervalCount)));

	std::vector<int> path(W);
	int pathCount = W;
	int start = vol->cells[0].first;
	int goal = vol->cells[W - 1].first;
	rvlStatus st = rvlFindPath(*q, start, goal,
	                            path.data(), &pathCount, W);
	REQUIRE(rvlStatusSucceed(st));
	REQUIRE(!rvlStatusDetail(st, RVOL_BUFFER_TOO_SMALL));
	REQUIRE(pathCount == W);
	REQUIRE(path.front() == start);
	REQUIRE(path.back() == goal);

	// Every consecutive pair must be linked neighbours.
	for (int i = 0; i + 1 < pathCount; ++i)
	{
		const rvlInterval& iv = vol->intervals[path[i]];
		bool linked = false;
		for (int d = 0; d < 6; ++d)
			if (iv.links[d] == path[i + 1]) linked = true;
		REQUIRE(linked);
	}

	rvlFreeVolumeQuery(&q);
	rvlFreeNavVolume(&vol);
}
