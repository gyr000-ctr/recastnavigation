// Test helpers for the optional 3D navigation volume module.
#ifndef TESTS_RECASTVOLUME_HELPERS_H
#define TESTS_RECASTVOLUME_HELPERS_H

#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastVolume.h"

#include <vector>

// Build a heightfield over a w x h x voxelH voxel grid with cs=ch=1.
inline bool makeHeightfield(rcContext& ctx, rcHeightfield& hf,
                            int w, int h, int voxelH)
{
	const float bmin[3] = {0.0f, 0.0f, 0.0f};
	const float bmax[3] = {(float)w, (float)voxelH, (float)h};
	return rcCreateHeightfield(&ctx, hf, w, h, bmin, bmax, 1.0f, 1.0f);
}

// Add a solid span [smin,smax) to column (x,z).
inline bool addSolid(rcContext& ctx, rcHeightfield& hf,
                     int x, int z, int smin, int smax,
                     unsigned char area = RC_WALKABLE_AREA)
{
	return rcAddSpan(&ctx, hf, x, z, smin, smax, area, 1);
}

// Count the live free intervals of a built volume.
inline int countIntervals(const rvlNavVolume& vol)
{
	return vol.intervalCount;
}

// Total number of non-null links of a volume (used for overlap/layer checks).
inline int countLinks(const rvlNavVolume& vol)
{
	int n = 0;
	for (int i = 0; i < vol.intervalCount; ++i)
		for (int d = 0; d < 6; ++d)
			if (vol.intervals[i].links[d] != RVOL_NULL) ++n;
	return n;
}

// A counting allocator: increments a counter on every allocation so tests can
// prove the per-frame query path performs zero heap allocations.
struct CountingAllocator
{
	static long long liveAllocations;
	static long long totalAllocations;

	static void* alloc(size_t size, rcAllocHint)
	{
		++liveAllocations;
		++totalAllocations;
		return malloc(size);
	}
	static void freeFunc(void* p)
	{
		if (p) --liveAllocations;
		free(p);
	}
	static void install()
	{
		liveAllocations = 0;
		totalAllocations = 0;
		rcAllocSetCustom(&CountingAllocator::alloc, &CountingAllocator::freeFunc);
	}
	static void uninstall()
	{
		rcAllocSetCustom(0, 0);
	}
};

#endif // TESTS_RECASTVOLUME_HELPERS_H
