#include "Recast.h"
#include "RecastVolume.h"

#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"

#include "catch2/catch_amalgamated.hpp"

#include "Tests_RecastVolumeHelpers.h"

namespace
{
	constexpr int VOXEL_H = 20;

	// Build a minimal single-quad 2D Detour navmesh covering the 8x8 ground.
	// Ownership of the tile data transfers to the navmesh.
	bool buildFlat2DNavMesh(dtNavMesh& nav, dtNavMeshQuery& query)
	{
		// Four quad corners in Detour vertex coordinates (plain floats here via
		// a 1:1 cs/ch mapping below).
		static const int nvp = 4;
		unsigned short verts[4 * 3] = {
			0, 0, 0,
			8, 0, 0,
			8, 0, 8,
			0, 0, 8,
		};
		// One polygon referencing the four verts, no neighbours.
		unsigned short polys[2 * nvp] = {0, 1, 2, 3, 0, 0, 0, 0};
		unsigned short flags[1] = {0xffff};
		unsigned char areas[1] = {RC_WALKABLE_AREA};

		dtNavMeshCreateParams params;
		memset(&params, 0, sizeof(params));
		params.verts = verts;
		params.vertCount = 4;
		params.polys = polys;
		params.polyFlags = flags;
		params.polyAreas = areas;
		params.polyCount = 1;
		params.nvp = nvp;
		params.walkableHeight = 2.0f;
		params.walkableRadius = 0.5f;
		params.walkableClimb = 1.0f;
		params.cs = 1.0f;
		params.ch = 1.0f;
		params.bmin[0] = 0; params.bmin[1] = 0; params.bmin[2] = 0;
		params.bmax[0] = 8; params.bmax[1] = 1; params.bmax[2] = 8;
		params.buildBvTree = true;

		unsigned char* data = 0;
		int dataSize = 0;
		if (!dtCreateNavMeshData(&params, &data, &dataSize))
			return false;

		if (dtStatusFailed(nav.init(data, dataSize, DT_TILE_FREE_DATA)))
		{
			dtFree(data);
			return false;
		}
		return dtStatusSucceed(query.init(&nav, 2048));
	}
}

TEST_CASE("rvlFindNearestFallback: uses 2D mesh when the volume has no answer",
           "[recast][volume][fallback]")
{
	rcContext ctx(false);
	rcHeightfield hf;
	REQUIRE(makeHeightfield(ctx, hf, 8, 8, VOXEL_H));
	// A fully solid block leaves no free interval near (0.5, 0.5, 0.5).
	for (int x = 0; x < 4; ++x)
		for (int z = 0; z < 4; ++z)
			REQUIRE(addSolid(ctx, hf, x, z, 0, VOXEL_H));

	rvlConfig cfg;
	rvlNavVolume* vol = 0;
	REQUIRE(rvlStatusSucceed(rvlBuildNavVolume(&ctx, hf, cfg, &vol)));

	rvlVolumeQuery* q = 0;
	REQUIRE(rvlStatusSucceed(rvlAllocVolumeQuery(&q)));
	REQUIRE(rvlStatusSucceed(rvlInitVolumeQuery(*q, *vol, vol->intervalCount)));

	dtNavMesh nav;
	dtNavMeshQuery navQuery;
	REQUIRE(buildFlat2DNavMesh(nav, navQuery));
	dtQueryFilter filter;

	float center[3] = {0.5f, 0.5f, 0.5f};
	float tinyExtents[3] = {0.1f, 0.1f, 0.1f};
	int idx = RVOL_NULL;
	float nearest[3] = {0,0,0};

	SECTION("Without a 2D fallback the query simply fails, never crashes")
	{
		rvlStatus st = rvlFindNearestFallback(*q, 0, 0, center, tinyExtents,
		                                       idx, nearest);
		REQUIRE(rvlStatusFailed(st));
	}

	SECTION("With a 2D fallback the Detour result is returned and flagged")
	{
		rvlStatus st = rvlFindNearestFallback(*q, &navQuery, &filter,
		                                       center, tinyExtents, idx, nearest);
		REQUIRE(rvlStatusSucceed(st));
		REQUIRE(rvlStatusDetail(st, RVOL_USED_2D_FALLBACK));
		REQUIRE(idx == RVOL_NULL); // not a volume interval
		REQUIRE(nearest[0] == Catch::Approx(0.5f).margin(0.01f));
		REQUIRE(nearest[2] == Catch::Approx(0.5f).margin(0.01f));
	}

	rvlFreeVolumeQuery(&q);
	rvlFreeNavVolume(&vol);
}

TEST_CASE("rvlFindPathFallback: degrades to a 2D straight path",
           "[recast][volume][fallback]")
{
	rcContext ctx(false);
	rcHeightfield hf;
	REQUIRE(makeHeightfield(ctx, hf, 8, 8, VOXEL_H));
	// Wall sealing the volume into two disconnected halves.
	for (int z = 0; z < 8; ++z)
		REQUIRE(addSolid(ctx, hf, 4, z, 0, VOXEL_H));

	rvlConfig cfg;
	rvlNavVolume* vol = 0;
	REQUIRE(rvlStatusSucceed(rvlBuildNavVolume(&ctx, hf, cfg, &vol)));

	rvlVolumeQuery* q = 0;
	REQUIRE(rvlStatusSucceed(rvlAllocVolumeQuery(&q)));
	REQUIRE(rvlStatusSucceed(rvlInitVolumeQuery(*q, *vol, vol->intervalCount)));

	dtNavMesh nav;
	dtNavMeshQuery navQuery;
	REQUIRE(buildFlat2DNavMesh(nav, navQuery));
	dtQueryFilter filter;

	float start[3] = {1.0f, 0.0f, 1.0f};
	float end[3] = {7.0f, 0.0f, 7.0f};
	float extents[3] = {2.0f, 2.0f, 2.0f};
	float straight[64 * 3];
	int straightCount = 0;
	dtPolyRef scratch[64];

	// 3D path is impossible (sealed wall), so the 2D mesh supplies the route.
	rvlStatus st = rvlFindPathFallback(*q, &navQuery, &filter,
	                                    start, end, extents,
	                                    straight, &straightCount, 64,
	                                    scratch, 64);
	REQUIRE(rvlStatusSucceed(st));
	REQUIRE(rvlStatusDetail(st, RVOL_USED_2D_FALLBACK));
	REQUIRE(straightCount >= 2);
	REQUIRE(straight[0] == Catch::Approx(1.0f));
	REQUIRE(straight[(straightCount - 1) * 3 + 0] == Catch::Approx(7.0f));
	REQUIRE(straight[(straightCount - 1) * 3 + 2] == Catch::Approx(7.0f));

	// Without the 2D query the same request reports failure instead of crashing.
	rvlStatus stNoFallback = rvlFindPathFallback(*q, 0, 0,
	                                              start, end, extents,
	                                              straight, &straightCount, 64,
	                                              scratch, 64);
	REQUIRE(rvlStatusFailed(stNoFallback));

	rvlFreeVolumeQuery(&q);
	rvlFreeNavVolume(&vol);
}
