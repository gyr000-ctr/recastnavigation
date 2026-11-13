//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#include "RecastVolume.h"

#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"

// This translation unit is the only place Detour is pulled in, keeping the
// core volume build/query code independent of the 2D mesh.

rvlStatus rvlFindNearestFallback(const rvlVolumeQuery& q,
                                 const dtNavMeshQuery* navQuery,
                                 const dtQueryFilter* filter,
                                 const float* center, const float* halfExtents,
                                 int& nearestInterval, float* nearestPt)
{
	rvlStatus st = rvlFindNearest(q, center, halfExtents, nearestInterval, nearestPt);
	if (rvlStatusSucceed(st))
		return st;

	// 3D query failed: fall back to the 2D Detour mesh instead of failing.
	if (!navQuery)
		return rvlFail(RVOL_NO_PATH);

	dtPolyRef ref = 0;
	float pt[3];
	dtStatus dts = navQuery->findNearestPoly(center, halfExtents, filter, &ref, pt);
	if (dtStatusFailed(dts) || ref == 0)
		return rvlFail(RVOL_NO_PATH);

	nearestInterval = RVOL_NULL; // not a volume interval: caller sees the flag
	if (nearestPt)
	{
		nearestPt[0] = pt[0];
		nearestPt[1] = pt[1];
		nearestPt[2] = pt[2];
	}
	return RVOL_SUCCESS | RVOL_USED_2D_FALLBACK;
}

rvlStatus rvlFindPathFallback(const rvlVolumeQuery& q,
                              const dtNavMeshQuery* navQuery,
                              const dtQueryFilter* filter,
                              const float* startPos, const float* endPos,
                              const float* halfExtents,
                              float* straightPath, int* straightPathCount,
                              int maxPath, dtPolyRef* scratchPolys,
                              int maxScratchPolys)
{
	if (!straightPath || !straightPathCount || maxPath <= 0 ||
	    !scratchPolys || maxScratchPolys <= 0)
		return rvlFail(RVOL_INVALID_PARAM);

	// Try the full 3D route first.  A fixed 128-interval scratch is enough for
	// ordinary in-level routes; if a route needs more (BUFFER_TOO_SMALL) it is
	// treated as a 3D miss and the request falls through to the 2D mesh below.
	int startInterval = RVOL_NULL, endInterval = RVOL_NULL;
	float startNearest[3], endNearest[3];
	rvlStatus stStart = rvlFindNearest(q, startPos, halfExtents, startInterval, startNearest);
	rvlStatus stEnd = rvlFindNearest(q, endPos, halfExtents, endInterval, endNearest);

	if (rvlStatusSucceed(stStart) && rvlStatusSucceed(stEnd))
	{
		int intervalPath[128];
		int intervalCount = 128;
		rvlStatus stPath = rvlFindPath(q, startInterval, endInterval,
		                                intervalPath, &intervalCount, 128);
		// Require a complete, untruncated 3D corridor.
		if (rvlStatusSucceed(stPath) &&
		    !rvlStatusDetail(stPath, RVOL_BUFFER_TOO_SMALL))
		{
			int straightCount = maxPath;
			rvlStatus stStraight = rvlFindStraightPath(q, intervalPath, intervalCount,
			                                            startPos, endPos,
			                                            straightPath, &straightCount, maxPath);
			if (rvlStatusSucceed(stStraight) &&
			    !rvlStatusDetail(stStraight, RVOL_BUFFER_TOO_SMALL))
			{
				*straightPathCount = straightCount;
				return stStraight; // pure 3D result
			}
		}
	}

	// Any 3D failure: degrade to the 2D mesh. No crash, no assertion.
	if (!navQuery)
		return rvlFail(RVOL_NO_PATH);

	dtPolyRef startRef = 0, endRef = 0;
	float startPt[3], endPt[3];
	dtStatus dts = navQuery->findNearestPoly(startPos, halfExtents, filter, &startRef, startPt);
	if (dtStatusFailed(dts) || startRef == 0)
		return rvlFail(RVOL_NO_PATH);
	dts = navQuery->findNearestPoly(endPos, halfExtents, filter, &endRef, endPt);
	if (dtStatusFailed(dts) || endRef == 0)
		return rvlFail(RVOL_NO_PATH);

	int polyCount = maxScratchPolys;
	dts = navQuery->findPath(startRef, endRef, startPt, endPt, filter,
	                         scratchPolys, &polyCount, maxScratchPolys);
	if (dtStatusFailed(dts) || polyCount == 0)
		return rvlFail(RVOL_NO_PATH);

	int straightCount = maxPath;
	dts = navQuery->findStraightPath(startPos, endPos, scratchPolys, polyCount,
	                                  straightPath, 0, 0, &straightCount, straightCount);
	if (dtStatusFailed(dts) || straightCount == 0)
		return rvlFail(RVOL_NO_PATH);

	*straightPathCount = straightCount;
	return RVOL_SUCCESS | RVOL_USED_2D_FALLBACK;
}
