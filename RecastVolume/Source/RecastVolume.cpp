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
#include "RecastAlloc.h"

#include <float.h>
#include <math.h>
#include <new>
#include <string.h>
#include <stdlib.h>

// ---------------------------------------------------------------------------
// Status helpers and small utilities
// ---------------------------------------------------------------------------

static inline int rvlClampi(int v, int lo, int hi)
{
	return v < lo ? lo : (v > hi ? hi : v);
}

// FNV-1a 32 bit, used only as an integrity checksum of the serialized stream.
static unsigned int rvlFnv1a(const unsigned char* data, int size)
{
	unsigned int h = 2166136261u;
	for (int i = 0; i < size; ++i)
	{
		h ^= (unsigned int)data[i];
		h *= 16777619u;
	}
	return h;
}

// ---------------------------------------------------------------------------
// rvlNavVolume / rvlVolumeQuery lifecycle
// ---------------------------------------------------------------------------

rvlNavVolume::rvlNavVolume()
	: width(0), height(0), maxVoxelY(0),
	  intervalCount(0), intervalCapacity(0),
	  cells(0), intervals(0)
{
	bmin[0] = bmin[1] = bmin[2] = 0;
	bmax[0] = bmax[1] = bmax[2] = 0;
	cs = 0; ch = 0;
}

rvlNavVolume::~rvlNavVolume()
{
	rcFree(cells);
	rcFree(intervals);
}

rvlStatus rvlAllocNavVolume(rvlNavVolume** vol)
{
	if (!vol) return rvlFail(RVOL_INVALID_PARAM);
	*vol = static_cast<rvlNavVolume*>(rcAlloc(sizeof(rvlNavVolume), RC_ALLOC_PERM));
	if (!*vol) return rvlFail(RVOL_OUT_OF_MEMORY);
	new (*vol) rvlNavVolume();
	return RVOL_SUCCESS;
}

void rvlFreeNavVolume(rvlNavVolume** vol)
{
	if (!vol || !*vol) return;
	(*vol)->~rvlNavVolume();
	rcFree(*vol);
	*vol = 0;
}

rvlConfig::rvlConfig()
	: minFreeHeight(1), maxIntervals(2000000), maxCells(4096 * 4096)
{
}

rvlVolumeQuery::rvlVolumeQuery()
	: vol(0), nodeParent(0), nodeG(0), nodeF(0), nodeState(0),
	  openHeap(0), heapPos(0), maxNodes(0)
{
}

rvlVolumeQuery::~rvlVolumeQuery()
{
	rcFree(nodeParent);
	rcFree(nodeG);
	rcFree(nodeF);
	rcFree(nodeState);
	rcFree(openHeap);
	rcFree(heapPos);
}

rvlStatus rvlAllocVolumeQuery(rvlVolumeQuery** q)
{
	if (!q) return rvlFail(RVOL_INVALID_PARAM);
	*q = static_cast<rvlVolumeQuery*>(rcAlloc(sizeof(rvlVolumeQuery), RC_ALLOC_PERM));
	if (!*q) return rvlFail(RVOL_OUT_OF_MEMORY);
	new (*q) rvlVolumeQuery();
	return RVOL_SUCCESS;
}

void rvlFreeVolumeQuery(rvlVolumeQuery** q)
{
	if (!q || !*q) return;
	(*q)->~rvlVolumeQuery();
	rcFree(*q);
	*q = 0;
}

rvlStatus rvlInitVolumeQuery(rvlVolumeQuery& q, const rvlNavVolume& vol, int maxNodes)
{
	if (maxNodes <= 0) return rvlFail(RVOL_INVALID_PARAM);
	// The interval count is the largest node set a search can need.
	int n = maxNodes;
	if (vol.intervalCount > 0 && n > vol.intervalCount)
		n = vol.intervalCount;

	int* parent = static_cast<int*>(rcAlloc(sizeof(int) * n, RC_ALLOC_PERM));
	float* g = static_cast<float*>(rcAlloc(sizeof(float) * n, RC_ALLOC_PERM));
	float* f = static_cast<float*>(rcAlloc(sizeof(float) * n, RC_ALLOC_PERM));
	unsigned char* state = static_cast<unsigned char*>(rcAlloc(sizeof(unsigned char) * n, RC_ALLOC_PERM));
	int* heap = static_cast<int*>(rcAlloc(sizeof(int) * n, RC_ALLOC_PERM));
	int* hpos = static_cast<int*>(rcAlloc(sizeof(int) * n, RC_ALLOC_PERM));
	if (!parent || !g || !f || !state || !heap || !hpos)
	{
		rcFree(parent); rcFree(g); rcFree(f); rcFree(state);
		rcFree(heap); rcFree(hpos);
		return rvlFail(RVOL_OUT_OF_MEMORY);
	}
	rcFree(q.nodeParent); rcFree(q.nodeG); rcFree(q.nodeF);
	rcFree(q.nodeState); rcFree(q.openHeap); rcFree(q.heapPos);
	q.nodeParent = parent;
	q.nodeG = g;
	q.nodeF = f;
	q.nodeState = state;
	q.openHeap = heap;
	q.heapPos = hpos;
	q.maxNodes = n;
	q.vol = &vol;
	for (int i = 0; i < n; ++i)
	{
		q.nodeState[i] = 0;
		q.heapPos[i] = -1;
	}
	return RVOL_SUCCESS;
}

// ---------------------------------------------------------------------------
// Build
// ---------------------------------------------------------------------------

// Collect the free gaps of one column as [lo,hi) voxel ranges.
// Returns the number of gaps; ranges are appended to the supplied arrays.
static int rvlColumnFreeIntervals(const rcSpan* span, int maxY, int minFree,
                                  int* loOut, int* hiOut, unsigned char* areaOut,
                                  int cap, int& written)
{
	int cur = 0; // next voxel known to be free
	int count = 0;
	while (span)
	{
		int lo = (int)span->smin;
		int hi = (int)span->smax;
		if (lo < 0) lo = 0;
		if (hi > maxY) hi = maxY;
		if (lo > cur)
		{
			if (lo - cur >= minFree)
			{
				if (written >= cap) return -1;
				loOut[written] = cur;
				hiOut[written] = lo;
				// Inherit the area of the solid span bounding the gap below.
				areaOut[written] = span->area;
				++written;
				++count;
			}
		}
		if (hi > cur) cur = hi;
		span = span->next;
	}
	if (cur < maxY)
	{
		if (maxY - cur >= minFree)
		{
			if (written >= cap) return -1;
			loOut[written] = cur;
			hiOut[written] = maxY;
			areaOut[written] = RC_NULL_AREA; // open sky above the highest solid
			++written;
			++count;
		}
	}
	return count;
}

rvlStatus rvlBuildNavVolume(rcContext* ctx, const rcHeightfield& hf,
                            const rvlConfig& cfg, rvlNavVolume** out)
{
	if (!out || hf.width <= 0 || hf.height <= 0 || hf.cs <= 0 || hf.ch <= 0)
		return rvlFail(RVOL_INVALID_PARAM);
	if (cfg.minFreeHeight <= 0)
		return rvlFail(RVOL_INVALID_PARAM);

	const int w = hf.width;
	const int h = hf.height;

	// Overflow / huge-world guards.
	if (w > 0x7fff || h > 0x7fff)
	{
		if (ctx) ctx->log(RC_LOG_ERROR, "rvlBuildNavVolume: dimension too large (%d x %d)", w, h);
		return rvlFail(RVOL_INVALID_PARAM);
	}
	const long long cellCountLL = (long long)w * (long long)h;
	if (cellCountLL > (long long)cfg.maxCells)
	{
		if (ctx) ctx->log(RC_LOG_ERROR, "rvlBuildNavVolume: world exceeds cell cap (%lld > %d)",
		                  cellCountLL, cfg.maxCells);
		return rvlFail(RVOL_INVALID_PARAM);
	}
	const int cellCount = (int)cellCountLL;

	const float heightRange = hf.bmax[1] - hf.bmin[1];
	if (!(heightRange > 0))
	{
		if (ctx) ctx->log(RC_LOG_ERROR, "rvlBuildNavVolume: empty height range");
		return rvlFail(RVOL_INVALID_PARAM);
	}
	int maxY = (int)(heightRange / hf.ch + 0.5f);
	if (maxY <= 0) maxY = 1;
	// rcSpan stores 13-bit heights.
	if (maxY > RC_SPAN_MAX_HEIGHT) maxY = RC_SPAN_MAX_HEIGHT;

	// Upper bound on intervals: every solid span can open at most one gap,
	// and every column can contribute one sky gap.  Count solid spans first so
	// the estimate stays a constant multiple of the input.
	int solidSpanCount = 0;
	for (int c = 0; c < cellCount; ++c)
	{
		for (const rcSpan* s = hf.spans[c]; s; s = s->next)
			++solidSpanCount;
	}
	// A column with k spans has at most k+1 free gaps.
	long long capLL = (long long)solidSpanCount + (long long)cellCount;
	if (capLL > (long long)cfg.maxIntervals)
	{
		if (ctx) ctx->log(RC_LOG_ERROR, "rvlBuildNavVolume: estimated interval count %lld exceeds cap %d",
		                  capLL, cfg.maxIntervals);
		return rvlFail(RVOL_FAILURE);
	}
	const int capacity = (int)capLL;

	rvlNavVolume* vol = 0;
	rvlStatus st = rvlAllocNavVolume(&vol);
	if (rvlStatusFailed(st)) return st;

	vol->width = w;
	vol->height = h;
	vol->maxVoxelY = maxY;
	vol->cs = hf.cs;
	vol->ch = hf.ch;
	rcVcopy(vol->bmin, hf.bmin);
	rcVcopy(vol->bmax, hf.bmax);

	vol->cells = static_cast<rvlCell*>(rcAlloc(sizeof(rvlCell) * cellCount, RC_ALLOC_PERM));
	vol->intervals = capacity > 0
		? static_cast<rvlInterval*>(rcAlloc(sizeof(rvlInterval) * capacity, RC_ALLOC_PERM))
		: 0;
	if (!vol->cells || (capacity > 0 && !vol->intervals))
	{
		rvlFreeNavVolume(&vol);
		return rvlFail(RVOL_OUT_OF_MEMORY);
	}

	// Scratch for a single column's gaps. A column cannot contain more spans
	// than fit in 13 bits of height, so this is small and bounded.
	int* tmpLo = static_cast<int*>(rcAlloc(sizeof(int) * (maxY + 1), RC_ALLOC_TEMP));
	int* tmpHi = static_cast<int*>(rcAlloc(sizeof(int) * (maxY + 1), RC_ALLOC_TEMP));
	unsigned char* tmpArea = static_cast<unsigned char*>(rcAlloc(sizeof(unsigned char) * (maxY + 1), RC_ALLOC_TEMP));
	if (!tmpLo || !tmpHi || !tmpArea)
	{
		rcFree(tmpLo); rcFree(tmpHi); rcFree(tmpArea);
		rvlFreeNavVolume(&vol);
		return rvlFail(RVOL_OUT_OF_MEMORY);
	}

	int total = 0;
	for (int c = 0; c < cellCount; ++c)
	{
		vol->cells[c].first = total;
		vol->cells[c].count = 0;
		int written = 0;
		int got = rvlColumnFreeIntervals(hf.spans[c], maxY, cfg.minFreeHeight,
		                                  tmpLo, tmpHi, tmpArea, maxY + 1, written);
		if (got < 0 || total + written > capacity)
		{
			rcFree(tmpLo); rcFree(tmpHi); rcFree(tmpArea);
			rvlFreeNavVolume(&vol);
			return rvlFail(RVOL_FAILURE);
		}
		for (int i = 0; i < written; ++i)
		{
			rvlInterval& iv = vol->intervals[total + i];
			iv.ymin = tmpLo[i];
			iv.ymax = tmpHi[i];
			iv.cell = c;
			iv.area = tmpArea[i];
			for (int d = 0; d < 6; ++d) iv.links[d] = RVOL_NULL;
		}
		vol->cells[c].count = written;
		total += written;
	}
	vol->intervalCount = total;
	vol->intervalCapacity = capacity;

	rcFree(tmpLo); rcFree(tmpHi); rcFree(tmpArea);

	// Link intervals: vertical links inside each column, horizontal links to
	// the overlapping interval in each of the four neighbour columns.
	for (int z = 0; z < h; ++z)
	{
		for (int x = 0; x < w; ++x)
		{
			const int c = x + z * w;
			const rvlCell& cell = vol->cells[c];

			// Vertical: consecutive intervals of the same column.
			for (int i = 0; i + 1 < cell.count; ++i)
			{
				int a = cell.first + i;
				int b = cell.first + i + 1;
				vol->intervals[a].links[5] = b; // up
				vol->intervals[b].links[4] = a; // down
			}

			// Horizontal neighbours.
			const int dirs[4][2] = { {-1,0},{1,0},{0,-1},{0,1} };
			for (int d = 0; d < 4; ++d)
			{
				int nx = x + dirs[d][0];
				int nz = z + dirs[d][1];
				if (nx < 0 || nx >= w || nz < 0 || nz >= h)
					continue; // open boundary: no link
				const rvlCell& ncell = vol->cells[nx + nz * w];
				for (int i = 0; i < cell.count; ++i)
				{
					const rvlInterval& iv = vol->intervals[cell.first + i];
					// Pick the neighbour interval with the largest height overlap.
					int best = RVOL_NULL;
					int bestOverlap = 0;
					for (int j = 0; j < ncell.count; ++j)
					{
						const rvlInterval& nv = vol->intervals[ncell.first + j];
						int lo = iv.ymin > nv.ymin ? iv.ymin : nv.ymin;
						int hi = iv.ymax < nv.ymax ? iv.ymax : nv.ymax;
						int overlap = hi - lo;
						if (overlap > bestOverlap)
						{
							bestOverlap = overlap;
							best = ncell.first + j;
						}
					}
					vol->intervals[cell.first + i].links[d] = best;
				}
			}
		}
	}

	*out = vol;
	return RVOL_SUCCESS;
}

// ---------------------------------------------------------------------------
// Coordinate helpers and interval lookup
// ---------------------------------------------------------------------------

void rvlWorldToVoxel(const rvlNavVolume& vol, const float* pos,
                     int& cx, int& cy, int& cz)
{
	cx = (int)floorf((pos[0] - vol.bmin[0]) / vol.cs);
	cy = (int)floorf((pos[1] - vol.bmin[1]) / vol.ch);
	cz = (int)floorf((pos[2] - vol.bmin[2]) / vol.cs);
}

static bool rvlInsideVolume(const rvlNavVolume& vol, int cx, int cz)
{
	return cx >= 0 && cx < vol.width && cz >= 0 && cz < vol.height;
}

// Interval in column (cx,cz) whose [ymin,ymax) contains voxel y. RVOL_NULL if none.
static int rvlIntervalAt(const rvlNavVolume& vol, int cx, int cy, int cz)
{
	if (!rvlInsideVolume(vol, cx, cz)) return RVOL_NULL;
	const rvlCell& cell = vol.cells[cx + cz * vol.width];
	// Intervals are stored bottom-to-top; binary search on ymin.
	int lo = 0, hi = cell.count - 1;
	while (lo <= hi)
	{
		int mid = (lo + hi) >> 1;
		const rvlInterval& iv = vol.intervals[cell.first + mid];
		if (cy < iv.ymin) hi = mid - 1;
		else if (cy >= iv.ymax) lo = mid + 1;
		else return cell.first + mid;
	}
	return RVOL_NULL;
}

static void rvlIntervalBox(const rvlNavVolume& vol, const rvlInterval& iv,
                           float* bminOut, float* bmaxOut)
{
	const int cx = iv.cell % vol.width;
	const int cz = iv.cell / vol.width;
	bminOut[0] = vol.bmin[0] + cx * vol.cs;
	bminOut[1] = vol.bmin[1] + iv.ymin * vol.ch;
	bminOut[2] = vol.bmin[2] + cz * vol.cs;
	bmaxOut[0] = vol.bmin[0] + (cx + 1) * vol.cs;
	bmaxOut[1] = vol.bmin[1] + iv.ymax * vol.ch;
	bmaxOut[2] = vol.bmin[2] + (cz + 1) * vol.cs;
}

bool rvlClampToInterval(const rvlNavVolume& vol, int intervalIdx,
                        const float* pos, float* nearest)
{
	if (intervalIdx < 0 || intervalIdx >= vol.intervalCount) return false;
	float lo[3], hi[3];
	rvlIntervalBox(vol, vol.intervals[intervalIdx], lo, hi);
	for (int i = 0; i < 3; ++i)
		nearest[i] = pos[i] < lo[i] ? lo[i] : (pos[i] > hi[i] ? hi[i] : pos[i]);
	return true;
}

// ---------------------------------------------------------------------------
// Nearest-point query
// ---------------------------------------------------------------------------

rvlStatus rvlFindNearest(const rvlVolumeQuery& q, const float* center,
                         const float* halfExtents,
                         int& nearestInterval, float* nearestPt)
{
	if (!q.vol || !center || !halfExtents) return rvlFail(RVOL_INVALID_PARAM);
	const rvlNavVolume& vol = *q.vol;

	int cx0, cy0, cz0;
	rvlWorldToVoxel(vol, center, cx0, cy0, cz0);
	int rx = (int)ceilf(halfExtents[0] / vol.cs);
	int rz = (int)ceilf(halfExtents[2] / vol.cs);
	int ry = (int)ceilf(halfExtents[1] / vol.ch);

	// The search box (in world space) must actually intersect the volume AABB;
	// otherwise a point far outside the world would incorrectly snap onto it.
	float searchMin[3] = { center[0] - halfExtents[0],
	                       center[1] - halfExtents[1],
	                       center[2] - halfExtents[2] };
	float searchMax[3] = { center[0] + halfExtents[0],
	                       center[1] + halfExtents[1],
	                       center[2] + halfExtents[2] };
	if (searchMax[0] <= vol.bmin[0] || searchMin[0] >= vol.bmax[0] ||
	    searchMax[1] <= vol.bmin[1] || searchMin[1] >= vol.bmax[1] ||
	    searchMax[2] <= vol.bmin[2] || searchMin[2] >= vol.bmax[2])
	{
		nearestInterval = RVOL_NULL;
		return rvlFail(RVOL_FAILURE);
	}

	int x0 = rvlClampi(cx0 - rx, 0, vol.width - 1);
	int x1 = rvlClampi(cx0 + rx, 0, vol.width - 1);
	int z0 = rvlClampi(cz0 - rz, 0, vol.height - 1);
	int z1 = rvlClampi(cz0 + rz, 0, vol.height - 1);

	int best = RVOL_NULL;
	float bestDistSqr = FLT_MAX;
	float bestPt[3] = {0,0,0};

	for (int z = z0; z <= z1; ++z)
	{
		for (int x = x0; x <= x1; ++x)
		{
			const rvlCell& cell = vol.cells[x + z * vol.width];
			for (int i = 0; i < cell.count; ++i)
			{
				int idx = cell.first + i;
				const rvlInterval& iv = vol.intervals[idx];
				// Reject intervals fully outside the vertical search window.
				if (iv.ymax < cy0 - ry || iv.ymin > cy0 + ry + 1)
					continue;
				float pt[3];
				rvlClampToInterval(vol, idx, center, pt);
				float dx = pt[0] - center[0];
				float dy = pt[1] - center[1];
				float dz = pt[2] - center[2];
				float d2 = dx*dx + dy*dy + dz*dz;
				if (d2 < bestDistSqr)
				{
					bestDistSqr = d2;
					best = idx;
					rcVcopy(bestPt, pt);
				}
			}
		}
	}

	if (best == RVOL_NULL)
	{
		nearestInterval = RVOL_NULL;
		return rvlFail(RVOL_FAILURE);
	}
	nearestInterval = best;
	if (nearestPt) rcVcopy(nearestPt, bestPt);
	return RVOL_SUCCESS;
}

// ---------------------------------------------------------------------------
// Raycast: 3D voxel DDA over the interval grid
// ---------------------------------------------------------------------------

// Test whether the segment p0->p1 crosses any solid voxel between them.
// Both endpoints are integer voxel coordinates; the walk advances one axis at
// a time, so hitting a voxel with no containing interval means a wall hit.
rvlStatus rvlRaycast(const rvlVolumeQuery& q,
                     const float* startPos, const float* endPos,
                     float& hitT, float* hitPoint, float* hitNormal)
{
	if (!q.vol || !startPos || !endPos) return rvlFail(RVOL_INVALID_PARAM);
	const rvlNavVolume& vol = *q.vol;

	int sx, sy, sz;
	rvlWorldToVoxel(vol, startPos, sx, sy, sz);
	int ex, ey, ez;
	rvlWorldToVoxel(vol, endPos, ex, ey, ez);

	// Reject rays that begin outside the world bounds: open boundary.
	if (!rvlInsideVolume(vol, sx, sz) || sy < 0 || sy >= vol.maxVoxelY)
		return rvlFail(RVOL_FAILURE);
	int cur = rvlIntervalAt(vol, sx, sy, sz);
	if (cur == RVOL_NULL)
		return rvlFail(RVOL_FAILURE); // started inside solid geometry

	int dx = ex - sx, dy = ey - sy, dz = ez - sz;
	const int stepX = dx > 0 ? 1 : (dx < 0 ? -1 : 0);
	const int stepY = dy > 0 ? 1 : (dy < 0 ? -1 : 0);
	const int stepZ = dz > 0 ? 1 : (dz < 0 ? -1 : 0);

	// tMax = distance (in ray parameter units scaled by |delta|) to the next
	// boundary on each axis; tDelta = parameter span of one voxel on an axis.
	float tMaxX = FLT_MAX, tMaxY = FLT_MAX, tMaxZ = FLT_MAX;
	float tDeltaX = FLT_MAX, tDeltaY = FLT_MAX, tDeltaZ = FLT_MAX;
	if (dx != 0)
	{
		tDeltaX = 1.0f / (float)rcAbs(dx);
		// First boundary measured from the starting fractional voxel position.
		float startFracX = (startPos[0] - vol.bmin[0]) / vol.cs - sx;
		tMaxX = tDeltaX * (dx > 0 ? (1.0f - startFracX) : startFracX);
	}
	if (dy != 0)
	{
		tDeltaY = 1.0f / (float)rcAbs(dy);
		float startFracY = (startPos[1] - vol.bmin[1]) / vol.ch - sy;
		tMaxY = tDeltaY * (dy > 0 ? (1.0f - startFracY) : startFracY);
	}
	if (dz != 0)
	{
		tDeltaZ = 1.0f / (float)rcAbs(dz);
		float startFracZ = (startPos[2] - vol.bmin[2]) / vol.cs - sz;
		tMaxZ = tDeltaZ * (dz > 0 ? (1.0f - startFracZ) : startFracZ);
	}

	float t = 0.0f;
	int nx = sx, ny = sy, nz = sz;
	int normal[3] = {0,0,0};
	const int maxSteps = rcAbs(dx) + rcAbs(dy) + rcAbs(dz) + 1;
	for (int step = 0; step < maxSteps; ++step)
	{
		if (nx == ex && ny == ey && nz == ez)
		{
			// End voxel must itself be free.
			if (rvlIntervalAt(vol, nx, ny, nz) == RVOL_NULL)
				break; // wall at destination
			hitT = 1.0f;
			if (hitPoint) rcVcopy(hitPoint, endPos);
			if (hitNormal) { hitNormal[0]=0; hitNormal[1]=0; hitNormal[2]=0; }
			return RVOL_SUCCESS;
		}

		// Advance along the axis whose boundary is nearest.
		if (tMaxX < tMaxY && tMaxX < tMaxZ)
		{
			t = tMaxX; tMaxX += tDeltaX; nx += stepX; normal[0]=-stepX; normal[1]=0; normal[2]=0;
		}
		else if (tMaxY < tMaxZ)
		{
			t = tMaxY; tMaxY += tDeltaY; ny += stepY; normal[0]=0; normal[1]=-stepY; normal[2]=0;
		}
		else
		{
			t = tMaxZ; tMaxZ += tDeltaZ; nz += stepZ; normal[0]=0; normal[1]=0; normal[2]=-stepZ;
		}

		// Open world edge: ray leaves the volume, stop at the boundary.
		if (!rvlInsideVolume(vol, nx, nz) || ny < 0 || ny >= vol.maxVoxelY)
		{
			hitT = t > 1.0f ? 1.0f : t;
			if (hitPoint)
			{
				hitPoint[0] = startPos[0] + (endPos[0]-startPos[0]) * hitT;
				hitPoint[1] = startPos[1] + (endPos[1]-startPos[1]) * hitT;
				hitPoint[2] = startPos[2] + (endPos[2]-startPos[2]) * hitT;
			}
			if (hitNormal)
			{
				hitNormal[0] = (float)normal[0];
				hitNormal[1] = (float)normal[1];
				hitNormal[2] = (float)normal[2];
			}
			return RVOL_SUCCESS;
		}

		int next = rvlIntervalAt(vol, nx, ny, nz);
		if (next == RVOL_NULL)
		{
			// Solid voxel: wall hit at parameter t.
			hitT = t > 1.0f ? 1.0f : t;
			if (hitPoint)
			{
				hitPoint[0] = startPos[0] + (endPos[0]-startPos[0]) * hitT;
				hitPoint[1] = startPos[1] + (endPos[1]-startPos[1]) * hitT;
				hitPoint[2] = startPos[2] + (endPos[2]-startPos[2]) * hitT;
			}
			if (hitNormal)
			{
				hitNormal[0] = (float)normal[0];
				hitNormal[1] = (float)normal[1];
				hitNormal[2] = (float)normal[2];
			}
			return RVOL_SUCCESS;
		}
		cur = next;
	}

	// Loop exhausted without reaching the goal: treat the last point as a hit.
	hitT = t > 1.0f ? 1.0f : t;
	if (hitPoint)
	{
		hitPoint[0] = startPos[0] + (endPos[0]-startPos[0]) * hitT;
		hitPoint[1] = startPos[1] + (endPos[1]-startPos[1]) * hitT;
		hitPoint[2] = startPos[2] + (endPos[2]-startPos[2]) * hitT;
	}
	if (hitNormal)
	{
		hitNormal[0] = (float)normal[0];
		hitNormal[1] = (float)normal[1];
		hitNormal[2] = (float)normal[2];
	}
	return RVOL_SUCCESS;
}

// ---------------------------------------------------------------------------
// A* pathfinding over the interval link graph
// ---------------------------------------------------------------------------

namespace {

struct rvlHeap
{
	int* heap;
	int* pos;
	float* keys;
	int size;
};

}

static void rvlHeapSwap(rvlHeap& h, int a, int b)
{
	int tmp = h.heap[a];
	h.heap[a] = h.heap[b];
	h.heap[b] = tmp;
	h.pos[h.heap[a]] = a;
	h.pos[h.heap[b]] = b;
}

static void rvlHeapUp(rvlHeap& h, int i)
{
	while (i > 0)
	{
		int parent = (i - 1) >> 1;
		if (h.keys[h.heap[parent]] <= h.keys[h.heap[i]]) break;
		rvlHeapSwap(h, parent, i);
		i = parent;
	}
}

static void rvlHeapDown(rvlHeap& h, int i)
{
	for (;;)
	{
		int smallest = i;
		int l = 2 * i + 1, r = 2 * i + 2;
		if (l < h.size && h.keys[h.heap[l]] < h.keys[h.heap[smallest]]) smallest = l;
		if (r < h.size && h.keys[h.heap[r]] < h.keys[h.heap[smallest]]) smallest = r;
		if (smallest == i) break;
		rvlHeapSwap(h, smallest, i);
		i = smallest;
	}
}

static float rvlIntervalDistance(const rvlNavVolume& vol, int aIdx, int bIdx)
{
	const rvlInterval& a = vol.intervals[aIdx];
	const rvlInterval& b = vol.intervals[bIdx];
	int ax = a.cell % vol.width, az = a.cell / vol.width;
	int bx = b.cell % vol.width, bz = b.cell / vol.width;
	// Movement cost uses voxel units; vertical moves count like horizontal ones.
	int ay = (a.ymin + a.ymax) >> 1;
	int by = (b.ymin + b.ymax) >> 1;
	float dx = (float)(ax - bx);
	float dy = (float)(ay - by);
	float dz = (float)(az - bz);
	return rcSqrt(dx*dx + dy*dy + dz*dz);
}

static float rvlHeuristic(const rvlNavVolume& vol, int aIdx, int bIdx)
{
	return rvlIntervalDistance(vol, aIdx, bIdx);
}

rvlStatus rvlFindPath(const rvlVolumeQuery& q,
                      int startInterval, int endInterval,
                      int* path, int* pathCount, int maxPath)
{
	if (!q.vol || !path || !pathCount || maxPath <= 0)
		return rvlFail(RVOL_INVALID_PARAM);
	const rvlNavVolume& vol = *q.vol;
	if (startInterval < 0 || startInterval >= vol.intervalCount ||
	    endInterval < 0 || endInterval >= vol.intervalCount)
		return rvlFail(RVOL_INVALID_PARAM);

	if (startInterval == endInterval)
	{
		path[0] = startInterval;
		*pathCount = 1;
		return RVOL_SUCCESS;
	}

	const int n = q.maxNodes;
	// Node storage is indexed by interval index when the volume fits, otherwise
	// node indices and interval indices would need a remap. The query init
	// clamps maxNodes to intervalCount, so indexing by interval is valid.
	if (vol.intervalCount > n)
		return rvlFail(RVOL_OUT_OF_NODES);

	for (int i = 0; i < n; ++i)
	{
		q.nodeState[i] = 0;
		q.heapPos[i] = -1;
	}

	rvlHeap heap;
	heap.heap = q.openHeap;
	heap.pos = q.heapPos;
	heap.keys = q.nodeF;
	heap.size = 0;

	q.nodeG[startInterval] = 0;
	q.nodeF[startInterval] = rvlHeuristic(vol, startInterval, endInterval);
	q.nodeParent[startInterval] = RVOL_NULL;
	q.nodeState[startInterval] = 1;
	heap.heap[0] = startInterval;
	heap.pos[startInterval] = 0;
	heap.size = 1;

	int goalReached = 0;
	while (heap.size > 0)
	{
		int cur = heap.heap[0];
		if (cur == endInterval) { goalReached = 1; break; }

		// Pop.
		heap.heap[0] = heap.heap[heap.size - 1];
		heap.pos[heap.heap[0]] = 0;
		--heap.size;
		if (heap.size > 0) rvlHeapDown(heap, 0);
		q.nodeState[cur] = 2;

		const rvlInterval& iv = vol.intervals[cur];
		for (int d = 0; d < 6; ++d)
		{
			int nb = iv.links[d];
			if (nb == RVOL_NULL) continue;
			if (q.nodeState[nb] == 2) continue;

			float cost = rvlIntervalDistance(vol, cur, nb);
			float ng = q.nodeG[cur] + cost;
			if (q.nodeState[nb] != 1 || ng < q.nodeG[nb])
			{
				q.nodeG[nb] = ng;
				q.nodeF[nb] = ng + rvlHeuristic(vol, nb, endInterval);
				q.nodeParent[nb] = cur;
				if (q.nodeState[nb] == 0)
				{
					q.nodeState[nb] = 1;
					heap.pos[nb] = heap.size;
					heap.heap[heap.size] = nb;
					++heap.size;
				}
				rvlHeapUp(heap, heap.pos[nb]);
			}
		}
	}

	if (!goalReached)
		return rvlFail(RVOL_NO_PATH);

	// Reconstruct goal -> start into the (now idle) openHeap scratch, so the
	// path can be arbitrarily long without a fixed stack buffer or allocation.
	int* reversed = q.openHeap;
	int count = 0;
	int cur = endInterval;
	while (cur != RVOL_NULL && count < n)
	{
		reversed[count++] = cur;
		cur = q.nodeParent[cur];
	}
	if (count > maxPath)
	{
		// Keep the start-anchored prefix and signal the truncation.
		for (int i = 0; i < maxPath; ++i)
			path[i] = reversed[count - 1 - i];
		*pathCount = maxPath;
		return RVOL_SUCCESS | RVOL_BUFFER_TOO_SMALL;
	}
	for (int i = 0; i < count; ++i)
		path[i] = reversed[count - 1 - i];
	*pathCount = count;
	return RVOL_SUCCESS;
}

// ---------------------------------------------------------------------------
// Straight path: string-pull via voxel DDA visibility
// ---------------------------------------------------------------------------

// True if the segment between two world points crosses only free voxels.
static bool rvlSegmentVisible(const rvlNavVolume& vol,
                              const float* p0, const float* p1)
{
	int x0, y0, z0;
	rvlWorldToVoxel(vol, p0, x0, y0, z0);
	int x1, y1, z1;
	rvlWorldToVoxel(vol, p1, x1, y1, z1);
	int dx = x1 - x0, dy = y1 - y0, dz = z1 - z0;
	const int stepX = dx > 0 ? 1 : (dx < 0 ? -1 : 0);
	const int stepY = dy > 0 ? 1 : (dy < 0 ? -1 : 0);
	const int stepZ = dz > 0 ? 1 : (dz < 0 ? -1 : 0);
	float tMaxX = FLT_MAX, tMaxY = FLT_MAX, tMaxZ = FLT_MAX;
	float tDeltaX = FLT_MAX, tDeltaY = FLT_MAX, tDeltaZ = FLT_MAX;
	if (dx != 0) { tDeltaX = 1.0f / (float)rcAbs(dx); tMaxX = tDeltaX; }
	if (dy != 0) { tDeltaY = 1.0f / (float)rcAbs(dy); tMaxY = tDeltaY; }
	if (dz != 0) { tDeltaZ = 1.0f / (float)rcAbs(dz); tMaxZ = tDeltaZ; }

	int x = x0, y = y0, z = z0;
	if (rvlIntervalAt(vol, x, y, z) == RVOL_NULL) return false;
	const int maxSteps = rcAbs(dx) + rcAbs(dy) + rcAbs(dz) + 1;
	for (int i = 0; i < maxSteps; ++i)
	{
		if (x == x1 && y == y1 && z == z1)
			return rvlIntervalAt(vol, x, y, z) != RVOL_NULL;
		if (tMaxX < tMaxY && tMaxX < tMaxZ)
		{
			tMaxX += tDeltaX; x += stepX;
		}
		else if (tMaxY < tMaxZ)
		{
			tMaxY += tDeltaY; y += stepY;
		}
		else
		{
			tMaxZ += tDeltaZ; z += stepZ;
		}
		if (!rvlInsideVolume(vol, x, z) || y < 0 || y >= vol.maxVoxelY)
			return false;
		if (rvlIntervalAt(vol, x, y, z) == RVOL_NULL) return false;
	}
	return true;
}

rvlStatus rvlFindStraightPath(const rvlVolumeQuery& q,
                              const int* intervalPath, int intervalPathCount,
                              const float* startPos, const float* endPos,
                              float* straightPath, int* straightPathCount,
                              int maxPath)
{
	if (!q.vol || !intervalPath || !straightPath || !straightPathCount || maxPath <= 0)
		return rvlFail(RVOL_INVALID_PARAM);
	const rvlNavVolume& vol = *q.vol;

	int out = 0;
	straightPath[out*3+0] = startPos[0];
	straightPath[out*3+1] = startPos[1];
	straightPath[out*3+2] = startPos[2];
	++out;

	int anchor = 0;
	for (int i = 1; i < intervalPathCount; ++i)
	{
		// Visibility from the current anchor point to the center of interval i.
		float center[3];
		float lo[3], hi[3];
		rvlIntervalBox(vol, vol.intervals[intervalPath[i]], lo, hi);
		center[0] = (lo[0] + hi[0]) * 0.5f;
		center[1] = (lo[1] + hi[1]) * 0.5f;
		center[2] = (lo[2] + hi[2]) * 0.5f;

		float anchorPt[3];
		rcVcopy(anchorPt, &straightPath[anchor * 3]);
		if (!rvlSegmentVisible(vol, anchorPt, center))
		{
			if (out >= maxPath)
			{
				*straightPathCount = out;
				return RVOL_SUCCESS | RVOL_BUFFER_TOO_SMALL;
			}
			// Emit the center of the previous interval as the bend point.
			float prevLo[3], prevHi[3], prevCenter[3];
			rvlIntervalBox(vol, vol.intervals[intervalPath[i-1]], prevLo, prevHi);
			prevCenter[0] = (prevLo[0]+prevHi[0])*0.5f;
			prevCenter[1] = (prevLo[1]+prevHi[1])*0.5f;
			prevCenter[2] = (prevLo[2]+prevHi[2])*0.5f;
			rcVcopy(&straightPath[out*3], prevCenter);
			++out;
			anchor = out - 1;
		}
	}

	if (out >= maxPath)
	{
		*straightPathCount = out;
		return RVOL_SUCCESS | RVOL_BUFFER_TOO_SMALL;
	}
	rcVcopy(&straightPath[out*3], endPos);
	++out;
	*straightPathCount = out;
	return RVOL_SUCCESS;
}

// ---------------------------------------------------------------------------
// Serialization
// ---------------------------------------------------------------------------

// The wire header is written field-by-field in little endian; its size is a
// fixed constant rather than sizeof(struct), so the stream layout never
// depends on compiler struct padding.  Layout (all 4-byte words):
// magic, version, width, height, maxVoxelY, intervalCount, intervalCapacity
// (7), bmin[3] (10), bmax[3] (13), cs (14), ch (15), checksum (16).
static const int RVOL_HEADER_SIZE = 16 * 4;

int rvlSerializedSize(const rvlNavVolume& vol)
{
	return RVOL_HEADER_SIZE
	     + (int)sizeof(rvlCell) * vol.width * vol.height
	     + (int)sizeof(rvlInterval) * vol.intervalCount;
}

static void rvlWriteLE32(unsigned char* p, unsigned int v)
{
	p[0] = (unsigned char)(v & 0xff);
	p[1] = (unsigned char)((v >> 8) & 0xff);
	p[2] = (unsigned char)((v >> 16) & 0xff);
	p[3] = (unsigned char)((v >> 24) & 0xff);
}

static unsigned int rvlReadLE32(const unsigned char* p)
{
	return (unsigned int)p[0]
	     | ((unsigned int)p[1] << 8)
	     | ((unsigned int)p[2] << 16)
	     | ((unsigned int)p[3] << 24);
}

rvlStatus rvlSerializeNavVolume(const rvlNavVolume& vol,
                                unsigned char* buffer, int bufferSize)
{
	if (!buffer) return rvlFail(RVOL_INVALID_PARAM);
	const int needed = rvlSerializedSize(vol);
	if (bufferSize < needed) return rvlFail(RVOL_BUFFER_TOO_SMALL);

	const int cellBytes = (int)sizeof(rvlCell) * vol.width * vol.height;
	const int intervalBytes = (int)sizeof(rvlInterval) * vol.intervalCount;

	// Header values kept in plain locals; nothing is written as a raw struct.
	const int width = vol.width, height = vol.height, maxVoxelY = vol.maxVoxelY;
	const int intervalCount = vol.intervalCount, intervalCapacity = vol.intervalCapacity;

	unsigned char* p = buffer;
	union { float f; unsigned int u; } cv;
	rvlWriteLE32(p, RVOL_MAGIC); p += 4;
	rvlWriteLE32(p, RVOL_VERSION); p += 4;
	rvlWriteLE32(p, (unsigned int)width); p += 4;
	rvlWriteLE32(p, (unsigned int)height); p += 4;
	rvlWriteLE32(p, (unsigned int)maxVoxelY); p += 4;
	rvlWriteLE32(p, (unsigned int)intervalCount); p += 4;
	rvlWriteLE32(p, (unsigned int)intervalCapacity); p += 4;
	for (int i = 0; i < 3; ++i) { cv.f = vol.bmin[i]; rvlWriteLE32(p, cv.u); p += 4; }
	for (int i = 0; i < 3; ++i) { cv.f = vol.bmax[i]; rvlWriteLE32(p, cv.u); p += 4; }
	cv.f = vol.cs; rvlWriteLE32(p, cv.u); p += 4;
	cv.f = vol.ch; rvlWriteLE32(p, cv.u); p += 4;
	rvlWriteLE32(p, 0); p += 4; // checksum placeholder

	// Payload is only the plain cell and interval arrays; the checksum covers
	// exactly these bytes.
	unsigned char* payload = p;
	memcpy(payload, vol.cells, cellBytes);
	memcpy(payload + cellBytes, vol.intervals, intervalBytes);

	const int payloadSize = cellBytes + intervalBytes;
	unsigned int checksum = rvlFnv1a(payload, payloadSize);
	// Patch the checksum field (last word of the fixed-size header).
	rvlWriteLE32(buffer + RVOL_HEADER_SIZE - 4, checksum);
	return RVOL_SUCCESS;
}

static bool rvlValidateVolume(const rvlNavVolume& vol)
{
	if (vol.width <= 0 || vol.height <= 0 || vol.maxVoxelY <= 0) return false;
	if (vol.intervalCount < 0 || vol.intervalCapacity < vol.intervalCount) return false;
	if (!(vol.cs > 0) || !(vol.ch > 0)) return false;
	const int cells = vol.width * vol.height;
	int expected = 0;
	for (int c = 0; c < cells; ++c)
	{
		const rvlCell& cell = vol.cells[c];
		if (cell.count < 0 || cell.first != expected) return false;
		expected += cell.count;
	}
	if (expected != vol.intervalCount) return false;
	for (int i = 0; i < vol.intervalCount; ++i)
	{
		const rvlInterval& iv = vol.intervals[i];
		if (iv.ymin < 0 || iv.ymax > vol.maxVoxelY || iv.ymin >= iv.ymax) return false;
		if (iv.cell < 0 || iv.cell >= cells) return false;
		for (int d = 0; d < 6; ++d)
			if (iv.links[d] != RVOL_NULL &&
			    (iv.links[d] < 0 || iv.links[d] >= vol.intervalCount))
				return false;
	}
	return true;
}
rvlStatus rvlDeserializeNavVolume(const unsigned char* buffer, int bufferSize,
                                  rvlNavVolume** out)
{
	if (!buffer || !out) return rvlFail(RVOL_INVALID_PARAM);
	if (bufferSize < RVOL_HEADER_SIZE) return rvlFail(RVOL_CORRUPT);

	const unsigned char* p = buffer;
	unsigned int magic = rvlReadLE32(p); p += 4;
	if (magic != RVOL_MAGIC) return rvlFail(RVOL_WRONG_MAGIC);
	unsigned int version = rvlReadLE32(p); p += 4;
	if (version != RVOL_VERSION) return rvlFail(RVOL_WRONG_VERSION);

	const int width = (int)rvlReadLE32(p); p += 4;
	const int height = (int)rvlReadLE32(p); p += 4;
	const int maxVoxelY = (int)rvlReadLE32(p); p += 4;
	const int intervalCount = (int)rvlReadLE32(p); p += 4;
	const int intervalCapacity = (int)rvlReadLE32(p); p += 4;
	float bmin[3], bmax[3];
	union { unsigned int u; float f; } cv;
	for (int i = 0; i < 3; ++i) { cv.u = rvlReadLE32(p); bmin[i] = cv.f; p += 4; }
	for (int i = 0; i < 3; ++i) { cv.u = rvlReadLE32(p); bmax[i] = cv.f; p += 4; }
	cv.u = rvlReadLE32(p); float cs = cv.f; p += 4;
	cv.u = rvlReadLE32(p); float ch = cv.f; p += 4;
	unsigned int checksum = rvlReadLE32(p); p += 4;

	if (width <= 0 || height <= 0 || intervalCount < 0 ||
	    intervalCapacity < intervalCount || maxVoxelY <= 0)
		return rvlFail(RVOL_CORRUPT);

	const long long cellsLL = (long long)width * (long long)height;
	if (cellsLL > 0x7fffffffLL) return rvlFail(RVOL_CORRUPT);
	const int cells = (int)cellsLL;
	const int cellBytes = (int)sizeof(rvlCell) * cells;
	const int intervalBytes = (int)sizeof(rvlInterval) * intervalCount;
	const int needed = RVOL_HEADER_SIZE + cellBytes + intervalBytes;
	if (bufferSize < needed) return rvlFail(RVOL_CORRUPT);

	const unsigned char* payload = buffer + RVOL_HEADER_SIZE;
	const int payloadSize = cellBytes + intervalBytes;
	if (rvlFnv1a(payload, payloadSize) != checksum)
		return rvlFail(RVOL_CORRUPT);

	rvlNavVolume* vol = 0;
	rvlStatus st = rvlAllocNavVolume(&vol);
	if (rvlStatusFailed(st)) return st;
	vol->width = width;
	vol->height = height;
	vol->maxVoxelY = maxVoxelY;
	vol->intervalCount = intervalCount;
	vol->intervalCapacity = intervalCapacity;
	rcVcopy(vol->bmin, bmin);
	rcVcopy(vol->bmax, bmax);
	vol->cs = cs;
	vol->ch = ch;

	vol->cells = static_cast<rvlCell*>(rcAlloc(sizeof(rvlCell) * (cells > 0 ? cells : 1), RC_ALLOC_PERM));
	vol->intervals = intervalCount > 0
		? static_cast<rvlInterval*>(rcAlloc(sizeof(rvlInterval) * intervalCount, RC_ALLOC_PERM))
		: 0;
	if (!vol->cells || (intervalCount > 0 && !vol->intervals))
	{
		rvlFreeNavVolume(&vol);
		return rvlFail(RVOL_OUT_OF_MEMORY);
	}
	memcpy(vol->cells, payload, cellBytes);
	memcpy(vol->intervals, payload + cellBytes, intervalBytes);

	if (!rvlValidateVolume(*vol))
	{
		rvlFreeNavVolume(&vol);
		return rvlFail(RVOL_CORRUPT);
	}

	*out = vol;
	return RVOL_SUCCESS;
}
