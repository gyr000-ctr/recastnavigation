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

#ifndef RECASTVOLUME_H
#define RECASTVOLUME_H

/// @file
/// Optional 3D navigation volumes ("Nav Volumes").
///
/// The standard Recast pipeline produces a 2.5D walkable surface: every (x,z)
/// column keeps only the open span that leads to a walkable polygon.  Flying,
/// swimming and vertically layered environments cannot be expressed that way.
///
/// This module is fully additive.  It consumes an #rcHeightfield as produced by
/// the unchanged rasterization stage (it therefore preserves non-walkable and
/// obstructed voxels) and turns the *unobstructed* space of every column into a
/// set of height intervals, a "navigation volume".  Intervals overlapping in
/// height are linked across the four horizontal neighbours; neighbouring
/// intervals inside the same column are linked vertically.  The resulting graph
/// supports nearest-point, raycast and path queries in full 3D, including
/// layered connections.
///
/// Design guarantees:
///  * No existing Recast/Detour type or code path is modified, so the default
///    2D build output is byte-for-byte identical when this module is not used.
///  * Build storage is a constant multiple of the voxel data: O(solid spans +
///    columns).  See rvlBuildNavVolume().
///  * Query objects hold all scratch memory they need up front; the per-frame
///    query functions never call the allocator.
///  * Every query reports failure through a status flag instead of asserting or
///    crashing, so callers can fall back to a 2D #dtNavMeshQuery.  The
///    rvlQuery*Fallback helpers implement that fallback.
///
/// @ingroup recast

#include "Recast.h"

// Forward declarations to keep the core header independent of Detour.
class dtNavMeshQuery;
class dtQueryFilter;
typedef unsigned int dtPolyRef;

/// Serialization magic number ("RVOL" little-endian).
static const unsigned int RVOL_MAGIC = 0x4c4f5652;
/// On-disk format version.
static const unsigned int RVOL_VERSION = 1;

/// Status flags returned by the volume functions.  Mirrors the style of
/// Detour's dtStatus so the two are easy to use together.
typedef unsigned int rvlStatus;

static const rvlStatus RVOL_FAILURE       = 1u << 31; ///< Operation failed.
static const rvlStatus RVOL_SUCCESS       = 1u << 30; ///< Operation succeeded.
static const rvlStatus RVOL_WRONG_MAGIC   = 1 << 0;   ///< Stream magic mismatch.
static const rvlStatus RVOL_WRONG_VERSION = 1 << 1;   ///< Stream version mismatch.
static const rvlStatus RVOL_OUT_OF_MEMORY = 1 << 2;   ///< Allocation failed.
static const rvlStatus RVOL_INVALID_PARAM = 1 << 3;   ///< Bad argument.
static const rvlStatus RVOL_BUFFER_TOO_SMALL = 1 << 4; ///< Output buffer too small.
static const rvlStatus RVOL_CORRUPT       = 1 << 5;   ///< Stream failed validation.
static const rvlStatus RVOL_NO_PATH       = 1 << 6;   ///< No route exists.
static const rvlStatus RVOL_OUT_OF_NODES  = 1 << 8;   ///< Search exhausted its node budget.
/// The result was produced by the 2D fallback rather than the volume.
static const rvlStatus RVOL_USED_2D_FALLBACK = 1 << 7;

inline bool rvlStatusSucceed(rvlStatus s) { return (s & RVOL_SUCCESS) != 0; }
inline bool rvlStatusFailed(rvlStatus s) { return (s & RVOL_FAILURE) != 0; }
inline bool rvlStatusDetail(rvlStatus s, rvlStatus d) { return (s & d) != 0; }

/// Build a failure status carrying a detail flag.
/// @ingroup recast
inline rvlStatus rvlFail(rvlStatus detail) { return RVOL_FAILURE | detail; }

/// Invalid index sentinel.
static const int RVOL_NULL = -1;

/// A vertically contiguous range of unobstructed (free) voxels in one column.
/// Height values use the same integer voxel units as #rcSpan::smin/#rcSpan::smax.
/// @ingroup recast
struct rvlInterval
{
	int ymin;          ///< Lower voxel bound, inclusive.
	int ymax;          ///< Upper voxel bound, exclusive.
	int cell;          ///< Column index into #rvlNavVolume::cells, x + z*width.
	unsigned char area; ///< Area id inherited from the surrounding geometry.
	int links[6];      ///< Neighbour interval indices, RVOL_NULL when absent.
	                   ///< Order: -x,+x,-z,+z, down (same column), up (same column).
};

/// One (x,z) column: the run of free intervals it contains.
/// @ingroup recast
struct rvlCell
{
	int first; ///< Index of the first interval in #rvlNavVolume::intervals.
	int count; ///< Number of intervals in this column.
};

/// A static 3D navigation volume built from an #rcHeightfield.
///
/// All arrays are plain, trivially copyable blocks so the object can be
/// serialized directly (see rvlSerializeNavVolume()).
/// @ingroup recast
struct rvlNavVolume
{
	rvlNavVolume();
	~rvlNavVolume();

	int width;        ///< Number of columns along x.
	int height;       ///< Number of columns along z.
	int maxVoxelY;    ///< Number of voxel layers along y (bmax-bmin)/ch.
	int intervalCount;///< Number of live intervals (<= intervalCapacity).
	int intervalCapacity; ///< Allocated size of #intervals.
	float bmin[3];    ///< World-space minimum bounds.
	float bmax[3];    ///< World-space maximum bounds.
	float cs;         ///< xz cell size (copied from the heightfield).
	float ch;         ///< voxel height (copied from the heightfield).
	rvlCell* cells;       ///< Columns, [width*height].
	rvlInterval* intervals; ///< Intervals, grouped by column, [intervalCapacity].

private:
	rvlNavVolume(const rvlNavVolume&);
	rvlNavVolume& operator=(const rvlNavVolume&);
};

/// Build parameters for rvlBuildNavVolume().
/// @ingroup recast
struct rvlConfig
{
	rvlConfig();

	/// Minimum interval height in voxels.  Free gaps smaller than this are
	/// treated as solid (they cannot fit an agent). Default 1.
	int minFreeHeight;
	/// Hard cap on the number of intervals produced; the build fails with
	/// RVOL_FAILURE instead of exhausting memory on pathological input.
	int maxIntervals;
	/// Hard cap on width*height columns (guards overflow on huge worlds).
	int maxCells;
};

/// Allocate an empty navigation volume.
/// @ingroup recast
rvlStatus rvlAllocNavVolume(rvlNavVolume** vol);

/// Free a navigation volume and all its arrays.
/// @ingroup recast
void rvlFreeNavVolume(rvlNavVolume** vol);

/// Build a navigation volume from a rasterized heightfield.
///
/// The heightfield retains every solid voxel (walkable or not); the free gaps
/// between its spans become #rvlInterval objects.  The 2.5D Recast pipeline is
/// never touched.  Fails gracefully (returns an error status, leaves @p out
/// untouched) on empty input, integer overflow or oversized worlds.
///
/// Memory usage is bounded by a constant multiple of the heightfield:
/// one #rvlCell per column and at most (solid span count + column count)
/// intervals, plus six ints of link storage per interval.
///
/// @param[in]  ctx   Recast context for warnings (may be 0).
/// @param[in]  hf    Rasterized heightfield.
/// @param[in]  cfg   Build parameters.
/// @param[out] out   Receives a newly allocated volume on success.
/// @ingroup recast
rvlStatus rvlBuildNavVolume(rcContext* ctx, const rcHeightfield& hf,
                            const rvlConfig& cfg, rvlNavVolume** out);

/// @name Coordinate helpers
/// @{

/// Convert a world position to integer column/voxel coordinates.
/// @ingroup recast
void rvlWorldToVoxel(const rvlNavVolume& vol, const float* pos,
                     int& cx, int& cy, int& cz);

/// Clamp a world point into the axis-aligned box of an interval and return the
/// result in @p nearest.  Returns false if @p intervalIdx is invalid.
/// @ingroup recast
bool rvlClampToInterval(const rvlNavVolume& vol, int intervalIdx,
                        const float* pos, float* nearest);

/// @}

/// @name Queries
///
/// Query objects own fixed-size scratch arrays.  All query functions are const
/// after rvlInitVolumeQuery() and perform zero heap allocation, so they are
/// safe to call every frame.
/// @{

/// Persistent query object for one #rvlNavVolume.
/// @ingroup recast
struct rvlVolumeQuery
{
	rvlVolumeQuery();
	~rvlVolumeQuery();

	const rvlNavVolume* vol; ///< Volume queried against (set by init).
	int* nodeParent;   ///< Parent interval per node, [maxNodes].
	float* nodeG;      ///< Best g-cost per node, [maxNodes].
	float* nodeF;      ///< f-cost per node, [maxNodes].
	unsigned char* nodeState; ///< 0 unseen, 1 open, 2 closed, [maxNodes].
	int* openHeap;     ///< Binary heap of interval indices, [maxNodes].
	int* heapPos;      ///< Heap position per interval or -1, [maxNodes].
	int maxNodes;      ///< Capacity of the scratch arrays.

private:
	rvlVolumeQuery(const rvlVolumeQuery&);
	rvlVolumeQuery& operator=(const rvlVolumeQuery&);
};

/// Allocate a query object.
/// @ingroup recast
rvlStatus rvlAllocVolumeQuery(rvlVolumeQuery** q);

/// Free a query object.
/// @ingroup recast
void rvlFreeVolumeQuery(rvlVolumeQuery** q);

/// Initialize scratch storage.  Allocates once; later queries never allocate.
/// @param maxNodes Maximum simultaneously expanded nodes.  Clamped internally to
///                 the volume's interval count, which is always sufficient.
/// @ingroup recast
rvlStatus rvlInitVolumeQuery(rvlVolumeQuery& q, const rvlNavVolume& vol,
                             int maxNodes);

/// Find the free interval closest to @p center.
///
/// Searches an expanding box of @p halfExtents (world units) around the point.
/// @param[out] nearestInterval  Index of the best interval or RVOL_NULL.
/// @param[out] nearestPt        Closest point, clamped to the interval box. [opt]
/// @ingroup recast
rvlStatus rvlFindNearest(const rvlVolumeQuery& q, const float* center,
                         const float* halfExtents,
                         int& nearestInterval, float* nearestPt);

/// Cast a ray through free space.
///
/// Uses a 3D voxel DDA over the interval grid.  Stops at the first solid
/// boundary or open world edge.
/// @param[out] hitT     Fraction in [0,1] of the hit; 1.0 means end reached.
/// @param[out] hitPoint World position of the hit. [opt]
/// @param[out] hitNormal Axis-aligned normal of the hit surface. [opt]
/// @ingroup recast
rvlStatus rvlRaycast(const rvlVolumeQuery& q,
                     const float* startPos, const float* endPos,
                     float& hitT, float* hitPoint, float* hitNormal);

/// Find an interval path between two intervals with A*.
///
/// Costs are Euclidean voxel distances; horizontal and vertical links are
/// traversed uniformly so flight/swim paths can move on all three axes.
/// @param[out] path     Interval indices from start to goal.
/// @param[in,out] pathCount  Capacity in, number of intervals out.
/// @ingroup recast
rvlStatus rvlFindPath(const rvlVolumeQuery& q,
                      int startInterval, int endInterval,
                      int* path, int* pathCount, int maxPath);

/// Simplify an interval path into straight world-space waypoints using a
/// 3D voxel DDA visibility test.  Never allocates.
/// @ingroup recast
rvlStatus rvlFindStraightPath(const rvlVolumeQuery& q,
                              const int* intervalPath, int intervalPathCount,
                              const float* startPos, const float* endPos,
                              float* straightPath, int* straightPathCount,
                              int maxPath);

/// @}

/// @name 2D fallback wrappers
///
/// Each helper tries the 3D volume query first and, on any failure, runs the
/// equivalent 2D Detour query.  A successful fallback returns RVOL_SUCCESS with
/// the RVOL_USED_2D_FALLBACK detail flag set; total failure returns
/// RVOL_FAILURE.  The Detour objects are optional - passing a null
/// #dtNavMeshQuery disables the fallback.
/// @{

/// Nearest point with 2D fallback.  @p filter may be null for Detour defaults
/// only when a query object is supplied.
/// @ingroup recast
rvlStatus rvlFindNearestFallback(const rvlVolumeQuery& q,
                                 const dtNavMeshQuery* navQuery,
                                 const dtQueryFilter* filter,
                                 const float* center, const float* halfExtents,
                                 int& nearestInterval, float* nearestPt);

/// Path with 2D fallback.  The 2D corridor is converted to straight points.
/// @ingroup recast
rvlStatus rvlFindPathFallback(const rvlVolumeQuery& q,
                              const dtNavMeshQuery* navQuery,
                              const dtQueryFilter* filter,
                              const float* startPos, const float* endPos,
                              const float* halfExtents,
                              float* straightPath, int* straightPathCount,
                              int maxPath, dtPolyRef* scratchPolys,
                              int maxScratchPolys);

/// @}

/// @name Serialization
///
/// The format is a fixed little-endian header followed by the raw cell and
/// interval arrays.  All fields are integers or IEEE-754 floats of known size,
/// so the stream is platform independent.  A checksum detects truncation and
/// corruption; every field is re-validated on load.
/// @{

/// Return the number of bytes rvlSerializeNavVolume() would write.
/// @ingroup recast
int rvlSerializedSize(const rvlNavVolume& vol);

/// Serialize into @p buffer (at least rvlSerializedSize() bytes).
/// @returns RVOL_SUCCESS or RVOL_INVALID_PARAM / RVOL_BUFFER_TOO_SMALL.
/// @ingroup recast
rvlStatus rvlSerializeNavVolume(const rvlNavVolume& vol,
                                unsigned char* buffer, int bufferSize);

/// Deserialize and validate a stream into a new volume.
/// @ingroup recast
rvlStatus rvlDeserializeNavVolume(const unsigned char* buffer, int bufferSize,
                                  rvlNavVolume** out);

/// @}

#endif // RECASTVOLUME_H
