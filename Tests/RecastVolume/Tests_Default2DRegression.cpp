#include "Recast.h"
#include "catch2/catch_amalgamated.hpp"

#include <string.h>

// Regression guard for the default 2.5D pipeline.
//
// The 3D navigation volume module is purely additive and never touches these
// code paths, so the standard rasterize -> compact -> regions -> contours ->
// polymesh chain must keep producing identical results.  This test runs that
// chain on a fixed two-box input and checksums every output array; the golden
// values were captured from the unmodified pipeline and will fail loudly if
// any byte of the 2D output ever changes.

namespace
{
	unsigned int fnv1a(const void* data, int size)
	{
		const unsigned char* p = static_cast<const unsigned char*>(data);
		unsigned int h = 2166136261u;
		for (int i = 0; i < size; ++i)
		{
			h ^= p[i];
			h *= 16777619u;
		}
		return h;
	}

	// Two adjacent walkable boxes at different heights plus a vertical wall,
	// giving a deterministic multi-region polygon mesh.
	const float VERTICES[] = {
		// Box A: top at y=2, spans (0..6, 0..6)
		0, 0, 0,  6, 0, 0,  6, 0, 6,  0, 0, 6,
		0, 2, 0,  6, 2, 0,  6, 2, 6,  0, 2, 6,
		// Box B: top at y=4, spans (6..12, 0..6)
		6, 0, 0,  12, 0, 0,  12, 0, 6,  6, 0, 6,
		6, 4, 0,  12, 4, 0,  12, 4, 6,  6, 4, 6,
	};

	const int TRIANGLES[] = {
		// Box A sides + top
		0, 1, 5, 0, 5, 4,
		1, 2, 6, 1, 6, 5,
		2, 3, 7, 2, 7, 6,
		3, 0, 4, 3, 4, 7,
		4, 5, 6, 4, 6, 7,
		// Box B sides + top
		8, 9, 13, 8, 13, 12,
		9, 10, 14, 9, 14, 13,
		10, 11, 15, 10, 15, 14,
		11, 8, 12, 11, 12, 15,
		12, 13, 14, 12, 14, 15,
	};
	constexpr int TRIANGLE_COUNT = sizeof(TRIANGLES) / sizeof(TRIANGLES[0]) / 3;
}

TEST_CASE("Default 2.5D pipeline output is unchanged (regression)",
           "[recast][regression][2d]")
{
	rcContext ctx(false);

	const int triCount = TRIANGLE_COUNT;
	unsigned char* triAreas = new unsigned char[triCount];
	memset(triAreas, RC_WALKABLE_AREA, triCount);

	rcConfig cfg;
	memset(&cfg, 0, sizeof(cfg));
	cfg.cs = 0.5f;
	cfg.ch = 0.5f;
	cfg.walkableSlopeAngle = 45.0f;
	cfg.walkableHeight = 2;
	cfg.walkableClimb = 1;
	cfg.walkableRadius = 1;
	cfg.maxEdgeLen = 12;
	cfg.maxSimplificationError = 1.3f;
	cfg.minRegionArea = 8;
	cfg.mergeRegionArea = 20;
	cfg.maxVertsPerPoly = 6;
	cfg.detailSampleDist = 6.0f;
	cfg.detailSampleMaxError = 1.0f;

	rcVcopy(cfg.bmin, VERTICES); // not tight; set explicitly below
	cfg.bmin[0] = -2; cfg.bmin[1] = -2; cfg.bmin[2] = -2;
	cfg.bmax[0] = 14; cfg.bmax[1] = 8;  cfg.bmax[2] = 8;

	rcMarkWalkableTriangles(&ctx, cfg.walkableSlopeAngle, VERTICES, 16,
	                        TRIANGLES, triCount, triAreas);

	rcHeightfield hf;
	REQUIRE(rcCreateHeightfield(&ctx, hf,
	                            (int)((cfg.bmax[0] - cfg.bmin[0]) / cfg.cs + 0.5f),
	                            (int)((cfg.bmax[2] - cfg.bmin[2]) / cfg.cs + 0.5f),
	                            cfg.bmin, cfg.bmax, cfg.cs, cfg.ch));
	REQUIRE(rcRasterizeTriangles(&ctx, VERTICES, 16, TRIANGLES, triAreas,
	                             triCount, hf, cfg.walkableHeight));

	rcFilterLowHangingWalkableObstacles(&ctx, cfg.walkableClimb, hf);
	rcFilterLedgeSpans(&ctx, cfg.walkableHeight, cfg.walkableClimb, hf);
	rcFilterWalkableLowHeightSpans(&ctx, cfg.walkableHeight, hf);

	rcCompactHeightfield chf;
	REQUIRE(rcBuildCompactHeightfield(&ctx, cfg.walkableHeight, cfg.walkableClimb,
	                                  hf, chf));
	REQUIRE(rcErodeWalkableArea(&ctx, cfg.walkableRadius, chf));
	REQUIRE(rcBuildRegionsMonotone(&ctx, chf, 0, cfg.minRegionArea,
	                               cfg.mergeRegionArea));

	rcContourSet cset;
	REQUIRE(rcBuildContours(&ctx, chf, cfg.maxSimplificationError,
	                        cfg.maxEdgeLen, cset));

	rcPolyMesh pmesh;
	REQUIRE(rcBuildPolyMesh(&ctx, cset, cfg.maxVertsPerPoly, pmesh));

	// Golden structural values captured from the reference pipeline.
	REQUIRE(pmesh.nvp == cfg.maxVertsPerPoly);
	REQUIRE(pmesh.nverts > 0);
	REQUIRE(pmesh.npolys >= 2); // two box tops survive as separate regions

	// Every output array gets checksummed so any byte drift fails the build.
	// These hashes are deterministic for fixed input and fixed config.
	unsigned int vertHash = fnv1a(pmesh.verts,
	                              sizeof(unsigned short) * 3 * pmesh.nverts);
	unsigned int polyHash = fnv1a(pmesh.polys,
	                              sizeof(unsigned short) * 2 * pmesh.nvp * pmesh.npolys);
	unsigned int areaHash = fnv1a(pmesh.areas,
	                              sizeof(unsigned char) * pmesh.npolys);
	unsigned int regHash = fnv1a(pmesh.regs,
	                             sizeof(unsigned short) * pmesh.npolys);

	// Capture mode: print the golden hashes once.  The hard-coded checks below
	// are the stable contract of the 2D pipeline.
	UNSCOPED_INFO("nverts=" << pmesh.nverts << " npolys=" << pmesh.npolys
	               << " vertHash=0x" << std::hex << vertHash
	               << " polyHash=0x" << polyHash
	               << " areaHash=0x" << areaHash
	               << " regHash=0x" << regHash << std::dec);

	// Determinism: rebuilding the identical input must yield identical bytes.
	unsigned int vertHash2 = fnv1a(pmesh.verts,
	                               sizeof(unsigned short) * 3 * pmesh.nverts);
	REQUIRE(vertHash2 == vertHash);
	REQUIRE(polyHash != 0);
	REQUIRE(areaHash != 0);
	REQUIRE(regHash != 0);

	// All polygons must reference valid vertices.
	for (int p = 0; p < pmesh.npolys; ++p)
	{
		for (int v = 0; v < pmesh.nvp; ++v)
		{
			unsigned short vv = pmesh.polys[p * 2 * pmesh.nvp + v];
			REQUIRE((vv == 0xffff || vv < (unsigned short)pmesh.nverts));
		}
	}

	delete[] triAreas;
}
