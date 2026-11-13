#ifndef TESTS_FORMATION_TEST_UTILS_H
#define TESTS_FORMATION_TEST_UTILS_H

#include <vector>
#include <math.h>
#include <string.h>

#include "Recast.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "DetourCommon.h"
#include "DetourCrowd.h"
#include "DetourAlloc.h"

/// Silent Recast context for tests.
class TestContext : public rcContext
{
public:
	TestContext() : rcContext(false) {}
protected:
	void doLog(const rcLogCategory, const char*, const int) override {}
};

/// Builds a flat navmesh from a set of axis-aligned boxes by rasterizing
/// their top faces (y = height). Overlapping boxes merge into one tiled
/// mesh. Used to construct deterministic test levels such as rooms linked
/// by narrow corridors.
struct TestNavMesh
{
	dtNavMesh* nav;
	rcContext* ctx;
	float bmin[3];
	float bmax[3];

	// Recast intermediate objects, kept alive for the mesh lifetime.
	rcHeightfield* solid;
	rcCompactHeightfield* chf;
	rcContourSet* cset;
	rcPolyMesh* pmesh;
	rcPolyMeshDetail* dmesh;
	unsigned char* navData;
	int navDataSize;
	std::vector<float> verts;
	std::vector<int> tris;
	std::vector<unsigned char> triAreas;

	TestNavMesh() :
		nav(0), ctx(0), solid(0), chf(0), cset(0), pmesh(0), dmesh(0),
		navData(0), navDataSize(0)
	{
		bmin[0] = bmin[1] = bmin[2] = 0;
		bmax[0] = bmax[1] = bmax[2] = 0;
	}

	~TestNavMesh()
	{
		if (nav) dtFreeNavMesh(nav);
		if (navData) dtFree(navData);
		if (solid) rcFreeHeightField(solid);
		if (chf) rcFreeCompactHeightfield(chf);
		if (cset) rcFreeContourSet(cset);
		if (pmesh) rcFreePolyMesh(pmesh);
		if (dmesh) rcFreePolyMeshDetail(dmesh);
		delete ctx;
	}

	/// Adds the top face of an axis-aligned box as walkable geometry.
	void addBoxTop(float minx, float minz, float maxx, float maxz, float y = 0.0f)
	{
		const int base = (int)(verts.size() / 3);
		verts.push_back(minx); verts.push_back(y); verts.push_back(minz); // 0
		verts.push_back(maxx); verts.push_back(y); verts.push_back(minz); // 1
		verts.push_back(maxx); verts.push_back(y); verts.push_back(maxz); // 2
		verts.push_back(minx); verts.push_back(y); verts.push_back(maxz); // 3
		// Two triangles, winding up (+y).
		tris.push_back(base+0); tris.push_back(base+2); tris.push_back(base+1);
		tris.push_back(base+0); tris.push_back(base+3); tris.push_back(base+2);
	}

	/// Adds an L/T-shaped walkable region as a union of box tops; gaps in
	/// the coverage simply produce no navmesh there, which is how narrow
	/// corridors and wall pinch points are created.
	void addCorridorStrip(float x0, float x1, float z0, float z1, float y = 0.0f)
	{
		addBoxTop(x0, z0, x1, z1, y);
	}

	bool build(const float agentRadius = 0.4f, const float cellSize = 0.2f)
	{
		ctx = new TestContext();

		const int nverts = (int)(verts.size() / 3);
		const int ntris = (int)(tris.size() / 3);
		if (nverts == 0 || ntris == 0)
			return false;
		triAreas.resize(ntris);
		memset(triAreas.data(), 0, ntris);

		rcCalcBounds(verts.data(), nverts, bmin, bmax);
		// Pad the bounds so border cells exist.
		const float pad = cellSize * 4.0f;
		bmin[0] -= pad; bmin[2] -= pad; bmin[1] -= 2.0f;
		bmax[0] += pad; bmax[2] += pad; bmax[1] += 2.0f;

		rcConfig cfg;
		memset(&cfg, 0, sizeof(cfg));
		cfg.cs = cellSize;
		cfg.ch = 0.2f;
		cfg.walkableSlopeAngle = 45.0f;
		cfg.walkableHeight = 2;
		cfg.walkableClimb = 1;
		cfg.walkableRadius = (int)ceilf(agentRadius / cfg.cs);
		cfg.maxEdgeLen = (int)(12.0f / cellSize);
		cfg.maxSimplificationError = 1.3f;
		cfg.minRegionArea = (int)rcSqr(8);
		cfg.mergeRegionArea = (int)rcSqr(20);
		cfg.maxVertsPerPoly = 6;
		cfg.detailSampleDist = cellSize * 6.0f;
		cfg.detailSampleMaxError = cfg.ch * 1.0f;
		rcVcopy(cfg.bmin, bmin);
		rcVcopy(cfg.bmax, bmax);
		rcCalcGridSize(cfg.bmin, cfg.bmax, cfg.cs, &cfg.width, &cfg.height);

		solid = rcAllocHeightfield();
		if (!solid || !rcCreateHeightfield(ctx, *solid, cfg.width, cfg.height,
										   cfg.bmin, cfg.bmax, cfg.cs, cfg.ch))
			return false;

		rcMarkWalkableTriangles(ctx, cfg.walkableSlopeAngle, verts.data(), nverts,
								tris.data(), ntris, triAreas.data());
		if (!rcRasterizeTriangles(ctx, verts.data(), nverts, tris.data(),
								  triAreas.data(), ntris, *solid, cfg.walkableClimb))
			return false;

		rcFilterLowHangingWalkableObstacles(ctx, cfg.walkableClimb, *solid);
		rcFilterLedgeSpans(ctx, cfg.walkableHeight, cfg.walkableClimb, *solid);
		rcFilterWalkableLowHeightSpans(ctx, cfg.walkableHeight, *solid);

		chf = rcAllocCompactHeightfield();
		if (!chf || !rcBuildCompactHeightfield(ctx, cfg.walkableHeight, cfg.walkableClimb,
											   *solid, *chf))
			return false;
		if (!rcErodeWalkableArea(ctx, cfg.walkableRadius, *chf))
			return false;
		if (!rcBuildRegionsMonotone(ctx, *chf, 0, cfg.minRegionArea, cfg.mergeRegionArea))
			return false;

		cset = rcAllocContourSet();
		if (!cset || !rcBuildContours(ctx, *chf, cfg.maxSimplificationError,
									  cfg.maxEdgeLen, *cset))
			return false;

		pmesh = rcAllocPolyMesh();
		if (!pmesh || !rcBuildPolyMesh(ctx, *cset, cfg.maxVertsPerPoly, *pmesh))
			return false;

		dmesh = rcAllocPolyMeshDetail();
		if (!dmesh || !rcBuildPolyMeshDetail(ctx, *pmesh, *chf,
											 cfg.detailSampleDist,
											 cfg.detailSampleMaxError, *dmesh))
			return false;

		for (int i = 0; i < pmesh->npolys; ++i)
			pmesh->flags[i] = 1;

		dtNavMeshCreateParams params;
		memset(&params, 0, sizeof(params));
		params.verts = pmesh->verts;
		params.vertCount = pmesh->nverts;
		params.polys = pmesh->polys;
		params.polyAreas = pmesh->areas;
		params.polyFlags = pmesh->flags;
		params.polyCount = pmesh->npolys;
		params.nvp = pmesh->nvp;
		params.detailMeshes = dmesh->meshes;
		params.detailVerts = dmesh->verts;
		params.detailVertsCount = dmesh->nverts;
		params.detailTris = dmesh->tris;
		params.detailTriCount = dmesh->ntris;
		params.walkableHeight = 2.0f;
		params.walkableRadius = agentRadius;
		params.walkableClimb = 0.2f;
		rcVcopy(params.bmin, pmesh->bmin);
		rcVcopy(params.bmax, pmesh->bmax);
		params.cs = cfg.cs;
		params.ch = cfg.ch;
		params.buildBvTree = true;

		if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
			return false;

		nav = dtAllocNavMesh();
		if (!nav) return false;
		if (dtStatusFailed(nav->init(navData, navDataSize, DT_TILE_FREE_DATA)))
			return false;
		// navData ownership transferred to the mesh on success.
		navData = 0;
		return true;
	}
};

/// Convenience: resolve a polygon reference for a world position.
inline dtPolyRef resolvePoly(dtNavMeshQuery* q, const dtQueryFilter* f,
							 const float* ext, const float* pos, float* outPos)
{
	dtPolyRef ref = 0;
	if (outPos)
		dtVcopy(outPos, pos);
	q->findNearestPoly(pos, ext, f, &ref, outPos ? outPos : const_cast<float*>(pos));
	return ref;
}

/// Creates a crowd agent with standard test parameters and returns its
/// index, or -1 on failure.
inline int makeAgent(dtCrowd* crowd, const float* pos, float radius = 0.4f)
{
	dtCrowdAgentParams ap;
	memset(&ap, 0, sizeof(ap));
	ap.radius = radius;
	ap.height = 2.0f;
	ap.maxAcceleration = 12.0f;
	ap.maxSpeed = 3.0f;
	ap.collisionQueryRange = radius * 12.0f;
	ap.pathOptimizationRange = radius * 30.0f;
	ap.separationWeight = 2.0f;
	ap.updateFlags = DT_CROWD_ANTICIPATE_TURNS | DT_CROWD_OBSTACLE_AVOIDANCE |
					 DT_CROWD_SEPARATION | DT_CROWD_OPTIMIZE_VIS |
					 DT_CROWD_OPTIMIZE_TOPO;
	ap.obstacleAvoidanceType = 0;
	ap.queryFilterType = 0;
	return crowd->addAgent(pos, &ap);
}

#endif // TESTS_FORMATION_TEST_UTILS_H
