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

#include <float.h>
#include <new>
#include <string.h>

#include "DetourFormation.h"
#include "DetourCrowd.h"
#include "DetourCommon.h"
#include "DetourMath.h"
#include "DetourAssert.h"
#include "DetourAlloc.h"

static const int FORMATION_QUERY_NODES = 1024;
static const int FORMATION_MAX_VISITED = 32;
static const float FORMATION_PI = 3.14159265358979323846f;

dtCrowdFormation* dtAllocCrowdFormation()
{
	void* mem = dtAlloc(sizeof(dtCrowdFormation), DT_ALLOC_PERM);
	if (!mem) return 0;
	return new(mem) dtCrowdFormation;
}

void dtFreeCrowdFormation(dtCrowdFormation* ptr)
{
	if (!ptr) return;
	ptr->~dtCrowdFormation();
	dtFree(ptr);
}

dtCrowdFormation::dtCrowdFormation() :
	m_crowd(0),
	m_nav(0),
	m_query(0),
	m_memberCount(0),
	m_state(DT_FORMATION_STATE_IDLE),
	m_pathCount(0),
	m_anchorSpeed(0.0f),
	m_replanTimer(0.0f),
	m_probeWidth(FLT_MAX),
	m_goalRef(0),
	m_hasGoal(false),
	m_goalUnreachable(false),
	m_forceRefresh(true)
{
	memset(m_members, 0, sizeof(m_members));
	memset(m_path, 0, sizeof(m_path));
	dtVset(m_pathTarget, 0,0,0);
	dtVset(m_anchorPos, 0,0,0);
	dtVset(m_facing, 0,0,1);
	dtVset(m_goalPos, 0,0,0);
	m_anchorRef = 0;
}

dtCrowdFormation::~dtCrowdFormation()
{
	purge();
}

void dtCrowdFormation::purge()
{
	if (m_query)
	{
		dtFreeNavMeshQuery(m_query);
		m_query = 0;
	}
	m_nav = 0;
	m_crowd = 0;
	m_memberCount = 0;
	m_state = DT_FORMATION_STATE_IDLE;
	m_pathCount = 0;
	m_goalRef = 0;
	m_hasGoal = false;
	m_goalUnreachable = false;
}

bool dtCrowdFormation::init(dtCrowd* crowd, dtNavMesh* nav)
{
	purge();
	if (!crowd || !nav)
		return false;

	m_crowd = crowd;
	m_nav = nav;

	m_query = dtAllocNavMeshQuery();
	if (!m_query)
		return false;
	if (dtStatusFailed(m_query->init(nav, FORMATION_QUERY_NODES)))
		return false;

	return true;
}

dtCrowdFormation::Member* dtCrowdFormation::findMember(int agentIdx)
{
	for (int i = 0; i < m_memberCount; ++i)
		if (m_members[i].agentIdx == agentIdx)
			return &m_members[i];
	return 0;
}

const dtCrowdFormation::Member* dtCrowdFormation::findMember(int agentIdx) const
{
	for (int i = 0; i < m_memberCount; ++i)
		if (m_members[i].agentIdx == agentIdx)
			return &m_members[i];
	return 0;
}

int dtCrowdFormation::getSlotOfAgent(int agentIdx) const
{
	const Member* m = findMember(agentIdx);
	return m ? m->slotIdx : -1;
}

void dtCrowdFormation::assignSlotsDefault()
{
	for (int i = 0; i < m_memberCount; ++i)
		m_members[i].slotIdx = i;
}

bool dtCrowdFormation::start(const int* agentIndices, int agentCount, const dtFormationConfig& config)
{
	if (!agentIndices || agentCount <= 0 || agentCount > DT_FORMATION_MAX_MEMBERS)
		return false;
	if (!m_query)
		return false;
	if (config.spacing <= 0.0f || config.depth <= 0.0f || config.memberRadius <= 0.0f)
		return false;
	if (config.type == DT_FORMATION_CUSTOM &&
		(!config.customSlots || config.customSlotCount <= 0))
		return false;

	m_config = config;

	// Normalize facing; keep a sane default when the caller passes a zero vector.
	const float flen = dtMathSqrtf(dtVlenSqr(m_config.facing));
	if (flen > 1e-6f)
	{
		dtVscale(m_facing, m_config.facing, 1.0f / flen);
		m_facing[1] = 0.0f;
	}
	else
	{
		dtVset(m_facing, 0,0,1);
	}

	dtVcopy(m_anchorPos, m_config.anchorPos);

	m_memberCount = 0;
	for (int i = 0; i < agentCount; ++i)
	{
		// Reject duplicate and out-of-range indices defensively.
		if (agentIndices[i] < 0 || agentIndices[i] >= m_crowd->getAgentCount())
			continue;
		if (findMember(agentIndices[i]))
			continue;
		Member* m = &m_members[m_memberCount++];
		m->agentIdx = agentIndices[i];
		m->slotIdx = m_memberCount - 1;
		dtVset(m->slotGoal, 0,0,0);
		dtVset(m->lastGoal, 0,0,0);
		m->flags = 0;
	}

	if (!m_memberCount)
		return false;

	assignSlotsDefault();

	m_pathCount = 0;
	m_anchorRef = 0;
	m_anchorSpeed = 0.0f;
	m_replanTimer = 0.0f;
	m_probeWidth = FLT_MAX;
	m_state = DT_FORMATION_STATE_NORMAL;
	m_forceRefresh = true;
	m_goalUnreachable = false;

	// Resolve the anchor to the navmesh immediately.
	const dtQueryFilter* filter = m_crowd->getFilter(0);
	float nearest[3];
	dtVcopy(nearest, m_anchorPos);
	m_query->findNearestPoly(m_anchorPos, m_crowd->getQueryHalfExtents(), filter,
							&m_anchorRef, nearest);
	if (m_anchorRef)
		dtVcopy(m_anchorPos, nearest);

	if (m_hasGoal)
		updateAnchorPath();

	return true;
}

bool dtCrowdFormation::updateConfig(const dtFormationConfig& config)
{
	if (config.spacing <= 0.0f || config.depth <= 0.0f || config.memberRadius <= 0.0f)
		return false;
	if (config.type == DT_FORMATION_CUSTOM &&
		(!config.customSlots || config.customSlotCount <= 0))
		return false;

	const int oldType = m_config.type;
	m_config = config;

	const float flen = dtMathSqrtf(dtVlenSqr(m_config.facing));
	if (flen > 1e-6f)
	{
		dtVscale(m_facing, m_config.facing, 1.0f / flen);
		m_facing[1] = 0.0f;
	}
	// A zero facing keeps the previous one.

	if (oldType != config.type)
	{
		assignSlotsDefault();
		m_forceRefresh = true;
	}
	return true;
}

bool dtCrowdFormation::ensureGoalRef()
{
	if (!m_hasGoal)
		return false;
	if (m_goalRef && m_query->isValidPolyRef(m_goalRef, m_crowd->getFilter(0)))
		return true;

	dtPolyRef ref = 0;
	float nearest[3];
	dtVcopy(nearest, m_goalPos);
	m_query->findNearestPoly(m_goalPos, m_crowd->getQueryHalfExtents(),
							m_crowd->getFilter(0), &ref, nearest);
	if (ref)
	{
		m_goalRef = ref;
		dtVcopy(m_goalPos, nearest);
		return true;
	}
	return false;
}

bool dtCrowdFormation::setGroupGoal(dtPolyRef ref, const float* pos)
{
	if (!m_query || !pos)
		return false;

	dtVcopy(m_goalPos, pos);
	m_hasGoal = true;
	m_goalUnreachable = false;

	if (ref)
	{
		m_goalRef = ref;
	}
	else
	{
		ensureGoalRef();
	}

	if (m_state != DT_FORMATION_STATE_IDLE)
	{
		updateAnchorPath();
		m_forceRefresh = true;
	}
	return true;
}

void dtCrowdFormation::teleportAnchor(const float* pos)
{
	if (!pos)
		return;
	dtVcopy(m_anchorPos, pos);
	m_anchorRef = 0;
	m_pathCount = 0;
	m_anchorSpeed = 0.0f;
	m_replanTimer = 0.0f;
	m_forceRefresh = true;
	m_goalUnreachable = false;

	const dtQueryFilter* filter = m_crowd->getFilter(0);
	float nearest[3];
	dtVcopy(nearest, m_anchorPos);
	m_query->findNearestPoly(m_anchorPos, m_crowd->getQueryHalfExtents(), filter,
							&m_anchorRef, nearest);
	if (m_anchorRef)
		dtVcopy(m_anchorPos, nearest);

	if (m_state != DT_FORMATION_STATE_IDLE && m_hasGoal)
		updateAnchorPath();
}

bool dtCrowdFormation::addMember(int agentIdx)
{
	if (m_state == DT_FORMATION_STATE_IDLE)
		return false;
	if (agentIdx < 0 || agentIdx >= m_crowd->getAgentCount())
		return false;
	if (m_memberCount >= DT_FORMATION_MAX_MEMBERS)
		return false;
	if (findMember(agentIdx))
		return false;

	// Lowest unused slot.
	int used[DT_FORMATION_MAX_MEMBERS];
	for (int i = 0; i < m_memberCount; ++i)
		used[i] = m_members[i].slotIdx;
	int slot = 0;
	bool taken = true;
	while (taken)
	{
		taken = false;
		for (int i = 0; i < m_memberCount; ++i)
			if (used[i] == slot) { taken = true; ++slot; break; }
	}

	Member* m = &m_members[m_memberCount++];
	m->agentIdx = agentIdx;
	m->slotIdx = slot;
	dtVset(m->slotGoal, 0,0,0);
	dtVset(m->lastGoal, 0,0,0);
	m->flags = 0;
	m_forceRefresh = true;
	return true;
}

bool dtCrowdFormation::removeMember(int agentIdx)
{
	for (int i = 0; i < m_memberCount; ++i)
	{
		if (m_members[i].agentIdx == agentIdx)
		{
			// Keep the remaining slots stable; only compact the member array.
			m_members[i] = m_members[m_memberCount - 1];
			--m_memberCount;
			m_forceRefresh = true;
			return true;
		}
	}
	return false;
}

void dtCrowdFormation::reassignSlots()
{
	if (!m_memberCount)
		return;

	// Order members by distance to the anchor so agents already close to a
	// slot keep a nearby slot; ties break on agent index for determinism.
	int order[DT_FORMATION_MAX_MEMBERS];
	for (int i = 0; i < m_memberCount; ++i)
		order[i] = i;
	for (int i = 1; i < m_memberCount; ++i)
	{
		const int tmp = order[i];
		const dtCrowdAgent* agI = m_crowd->getAgent(m_members[tmp].agentIdx);
		const float dI = agI ? dtVdist2DSqr(agI->npos, m_anchorPos) : FLT_MAX;
		int j = i;
		while (j > 0)
		{
			const dtCrowdAgent* agJ = m_crowd->getAgent(m_members[order[j-1]].agentIdx);
			const float dJ = agJ ? dtVdist2DSqr(agJ->npos, m_anchorPos) : FLT_MAX;
			if (dJ <= dI)
				break;
			order[j] = order[j-1];
			--j;
		}
		order[j] = tmp;
	}
	for (int i = 0; i < m_memberCount; ++i)
		m_members[order[i]].slotIdx = i;

	m_forceRefresh = true;
}

void dtCrowdFormation::stop()
{
	m_state = DT_FORMATION_STATE_IDLE;
	m_memberCount = 0;
	m_pathCount = 0;
	m_anchorSpeed = 0.0f;
	m_goalUnreachable = false;
	// Agents keep their current move targets: nothing is reset on dtCrowd.
}

void dtCrowdFormation::updateAnchorPath()
{
	m_pathCount = 0;
	m_goalUnreachable = false;

	if (!m_anchorRef || !ensureGoalRef())
	{
		m_goalUnreachable = true;
		return;
	}

	dtPolyRef path[DT_FORMATION_MAX_PATH];
	int pathCount = 0;
	const dtStatus status = m_query->findPath(m_anchorRef, m_goalRef, m_anchorPos, m_goalPos,
											   m_crowd->getFilter(0), path, &pathCount,
											   DT_FORMATION_MAX_PATH);
	if (dtStatusFailed(status) || pathCount == 0)
	{
		m_goalUnreachable = true;
		return;
	}

	if (path[pathCount - 1] != m_goalRef)
	{
		// Partial result: the goal is not reachable from the anchor.
		m_goalUnreachable = true;
	}

	memcpy(m_path, path, sizeof(dtPolyRef) * pathCount);
	m_pathCount = pathCount;

	// Constrain the corridor target to the last reachable polygon.
	if (m_goalUnreachable)
	{
		float closest[3];
		if (dtStatusSucceed(m_query->closestPointOnPoly(path[pathCount - 1],
														m_goalPos, closest, 0)))
			dtVcopy(m_pathTarget, closest);
		else
			dtVcopy(m_pathTarget, m_anchorPos);
	}
	else
	{
		dtVcopy(m_pathTarget, m_goalPos);
	}
}

void dtCrowdFormation::worldFromLocal(const float* local, float* world) const
{
	// local: x = right, z = forward relative to facing.
	const float rightX = m_facing[2];
	const float rightZ = -m_facing[0];
	world[0] = m_anchorPos[0] + rightX * local[0] + m_facing[0] * local[2];
	world[1] = m_anchorPos[1] + local[1];
	world[2] = m_anchorPos[2] + rightZ * local[0] + m_facing[2] * local[2];
}

void dtCrowdFormation::computeSlotLocal(int slotIdx, float* x, float* z) const
{
	const float spacing = m_config.spacing;
	const float depth = m_config.depth;

	switch (m_config.type)
	{
	case DT_FORMATION_WEDGE:
	{
		// Slot 0 at the tip; pairs spread back and out in V shape.
		const int row = (slotIdx + 1) / 2;
		const float side = (slotIdx % 2 == 0) ? 1.0f : -1.0f;
		*x = side * row * spacing;
		*z = -((float)row) * depth;
		break;
	}
	case DT_FORMATION_RING:
	{
		// Slot 0 at the center, remaining members distributed over rings.
		if (slotIdx == 0)
		{
			*x = 0.0f;
			*z = 0.0f;
			break;
		}
		int ring = 1;
		int base = 1;
		int perRing = 6;
		while (slotIdx >= base + perRing)
		{
			base += perRing;
			++ring;
			perRing += 6;
		}
		const int idxInRing = slotIdx - base;
		const float angle = (FORMATION_PI * 2.0f * idxInRing) / (float)perRing;
		const float radius = ring * depth;
		*x = dtMathCosf(angle) * radius;
		*z = dtMathSinf(angle) * radius;
		break;
	}
	case DT_FORMATION_CUSTOM:
	{
		const int idx = slotIdx < m_config.customSlotCount ? slotIdx : (slotIdx % m_config.customSlotCount);
		const float* s = &m_config.customSlots[idx * 3];
		*x = s[0];
		*z = s[2];
		break;
	}
	case DT_FORMATION_LINE:
	default:
	{
		// One centered row perpendicular to the facing.
		const int n = m_memberCount;
		*x = ((float)slotIdx - ((float)n - 1.0f) * 0.5f) * spacing;
		*z = 0.0f;
		break;
	}
	}
}

void dtCrowdFormation::probeWidth(float& width) const
{
	width = FLT_MAX;
	if (!m_anchorRef)
		return;

	// Place a probe point ahead of the anchor along the facing. Aiming the
	// full probe distance lets findNearestPoly snap onto a room or corridor
	// mouth within range instead of pinning the probe to the near wall.
	float probePos[3];
	dtVmad(probePos, m_anchorPos, m_facing, m_config.probeDistance);
	probePos[1] = m_anchorPos[1];

	dtPolyRef probeRef = 0;
	float nearest[3];
	dtVcopy(nearest, probePos);
	m_query->findNearestPoly(probePos, m_crowd->getQueryHalfExtents(),
							 m_crowd->getFilter(0), &probeRef, nearest);
	if (!probeRef)
		return;
	dtVcopy(probePos, nearest);

	// Measure lateral clearance by sampling findNearestPoly on both sides.
	// Raycast is unsuitable here: on a tessellated navmesh it reports the
	// internal polygon edges of a wide room as immediate "hits". Sampling
	// instead tests whether a real navigable surface exists at each offset,
	// which is exactly the property the column degradation needs.
	const float right[3] = { m_facing[2], 0.0f, -m_facing[0] };
	const float maxRange = m_config.spacing * (float)DT_FORMATION_MAX_MEMBERS;
	const float step = m_config.memberRadius * 2.0f;
	const int maxSteps = step > 0.0f ? (int)(maxRange / step) : 1;

	float clearRight = 0.0f;
	float clearLeft = 0.0f;
	for (int side = 0; side < 2; ++side)
	{
		const float sign = (side == 0) ? 1.0f : -1.0f;
		float clearance = 0.0f;
		for (int k = 1; k <= maxSteps; ++k)
		{
			float sample[3];
			sample[0] = probePos[0] + right[0] * sign * step * (float)k;
			sample[1] = probePos[1];
			sample[2] = probePos[2] + right[2] * sign * step * (float)k;

			dtPolyRef sampleRef = 0;
			float sampleNear[3];
			dtVcopy(sampleNear, sample);
			m_query->findNearestPoly(sample, m_crowd->getQueryHalfExtents(),
									 m_crowd->getFilter(0), &sampleRef, sampleNear);
			if (!sampleRef)
				break;

			// The sample only counts as clear space if the mesh point is
			// close to the requested offset; otherwise findNearestPoly would
			// have snapped a wall-side request back onto the walkable area.
			const float drift = dtVdist2D(sample, sampleNear);
			if (drift > m_config.memberRadius)
				break;

			clearance = step * (float)k;
		}
		if (side == 0)
			clearRight = clearance;
		else
			clearLeft = clearance;
	}

	width = clearRight + clearLeft;
}

void dtCrowdFormation::updateFacing()
{
	// Face along the anchor path: direction from the anchor toward the
	// corridor target. When stationary at the goal, keep the last facing.
	float dir[3];
	dtVsub(dir, m_pathTarget, m_anchorPos);
	dir[1] = 0.0f;
	const float len = dtVlen(dir);
	if (len > 0.05f)
	{
		dtVscale(dir, dir, 1.0f / len);
		dtVcopy(m_facing, dir);
	}
	// Explicitly configured facing is only applied as an initial value;
	// during movement the path direction is authoritative.
}

void dtCrowdFormation::requestMemberTarget(Member* m, dtPolyRef ref, const float* pos)
{
	// Only re-submit when the goal moved enough, to avoid flooding the
	// crowd path queue with identical requests every frame. A forced
	// refresh (teleport, reconfiguration, membership change) bypasses the
	// distance threshold once.
	const float movedSq = dtVdist2DSqr(pos, m->lastGoal);
	const bool first = (m->flags & DT_FORMATION_MEMBER_SLOT_VALID) == 0;
	if (first || m_forceRefresh || movedSq >= dtSqr(m_config.slotGoalRefreshDistance))
	{
		if (m_crowd->requestMoveTarget(m->agentIdx, ref, pos))
			dtVcopy(m->lastGoal, pos);
	}
}

void dtCrowdFormation::update(float dt)
{
	if (m_state == DT_FORMATION_STATE_IDLE || !m_memberCount || !m_query)
		return;

	// Drop members whose crowd agent was deactivated externally.
	for (int i = m_memberCount - 1; i >= 0; --i)
	{
		const dtCrowdAgent* ag = m_crowd->getAgent(m_members[i].agentIdx);
		if (!ag || !ag->active)
		{
			m_members[i] = m_members[m_memberCount - 1];
			--m_memberCount;
			m_forceRefresh = true;
		}
	}
	if (!m_memberCount)
	{
		m_state = DT_FORMATION_STATE_IDLE;
		return;
	}

	// --- Anchor localization -------------------------------------------------
	if (!m_anchorRef || !m_query->isValidPolyRef(m_anchorRef, m_crowd->getFilter(0)))
	{
		float nearest[3];
		dtVcopy(nearest, m_anchorPos);
		m_query->findNearestPoly(m_anchorPos, m_crowd->getQueryHalfExtents(),
								 m_crowd->getFilter(0), &m_anchorRef, nearest);
		if (m_anchorRef)
			dtVcopy(m_anchorPos, nearest);
	}

	// --- Group planning (throttled, member-independent) ----------------------
	m_replanTimer -= dt;
	if (m_replanTimer <= 0.0f)
	{
		m_replanTimer = m_config.replanInterval > 0.0f ? m_config.replanInterval : 0.5f;
		if (m_hasGoal)
			updateAnchorPath();
		probeWidth(m_probeWidth);
	}

	// --- Anchor advance along the corridor (stays on the navmesh) ------------
	float maxMemberSpeed = FLT_MAX;
	for (int i = 0; i < m_memberCount; ++i)
	{
		const dtCrowdAgent* ag = m_crowd->getAgent(m_members[i].agentIdx);
		if (ag && ag->params.maxSpeed < maxMemberSpeed)
			maxMemberSpeed = ag->params.maxSpeed;
	}
	if (maxMemberSpeed == FLT_MAX)
		maxMemberSpeed = 0.0f;

	// Straggler check: members well behind their slot slow the group down.
	bool anyStraggler = false;
	for (int i = 0; i < m_memberCount; ++i)
	{
		Member* mem = &m_members[i];
		const dtCrowdAgent* ag = m_crowd->getAgent(mem->agentIdx);
		mem->flags &= (unsigned char)~DT_FORMATION_MEMBER_STRAGGLER;
		if (ag && (mem->flags & DT_FORMATION_MEMBER_SLOT_VALID))
		{
			if (dtVdist2D(ag->npos, mem->slotGoal) > m_config.stragglerDistance)
			{
				mem->flags |= DT_FORMATION_MEMBER_STRAGGLER;
				anyStraggler = true;
			}
		}
	}

	float groupSpeed = maxMemberSpeed;
	if (anyStraggler && m_config.catchUpSpeedScale > 0.0f)
		groupSpeed *= m_config.catchUpSpeedScale;

	// Follow clamp: the anchor must not get more than stragglerDistance
	// ahead of the rearmost member along the movement axis. This keeps the
	// group together through long corridors and makes the anchor wait at
	// the goal for stragglers instead of outrunning them.
	float anchorStepBudget = groupSpeed * dt;
	{
		const float anchorProj = m_anchorPos[0] * m_facing[0] +
								 m_anchorPos[2] * m_facing[2];
		float rearmProj = FLT_MAX;
		for (int i = 0; i < m_memberCount; ++i)
		{
			const dtCrowdAgent* ag = m_crowd->getAgent(m_members[i].agentIdx);
			if (!ag)
				continue;
			const float p = ag->npos[0] * m_facing[0] +
							ag->npos[2] * m_facing[2];
			if (p < rearmProj)
				rearmProj = p;
		}
		if (rearmProj != FLT_MAX)
		{
			// In a column the rearmost slot trails the anchor by roughly the
			// whole column length, so allow at least that much slack plus one
			// straggler margin before halting the anchor.
			float columnLength = 0.0f;
			if (m_state == DT_FORMATION_STATE_COLUMN1)
				columnLength = (float)m_memberCount * m_config.spacing;
			else if (m_state == DT_FORMATION_STATE_COLUMN2)
				columnLength = ((float)(m_memberCount + 1) * 0.5f) * m_config.depth;
			const float allowedAhead = dtMax(m_config.stragglerDistance,
											 columnLength + m_config.spacing);
			const float maxAhead = (rearmProj + allowedAhead) - anchorProj;
			if (maxAhead < anchorStepBudget)
				anchorStepBudget = dtMax(0.0f, maxAhead);
		}
	}

	if (m_pathCount && m_anchorRef)
	{
		const float distToTarget = dtVdist2D(m_anchorPos, m_pathTarget);
		if (distToTarget > 0.01f)
		{
			// moveAlongSurface keeps the anchor on the mesh through gaps.
			float step[3];
			float dirToTarget[3];
			dtVsub(dirToTarget, m_pathTarget, m_anchorPos);
			dirToTarget[1] = 0.0f;
			dtVnormalize(dirToTarget);
			const float stepLen = dtMin(anchorStepBudget, distToTarget);
			dtVscale(step, dirToTarget, stepLen);

			// moveAlongSurface takes an absolute end position, not a delta.
			float endPos[3];
			dtVadd(endPos, m_anchorPos, step);

			float resultPos[3];
			dtPolyRef visited[FORMATION_MAX_VISITED];
			int visitedCount = 0;
			dtStatus st = m_query->moveAlongSurface(m_anchorRef, m_anchorPos,
													 endPos, m_crowd->getFilter(0),
													 resultPos, visited, &visitedCount,
													 FORMATION_MAX_VISITED);
			if (dtStatusSucceed(st) && visitedCount > 0)
			{
				// Re-acquire ground height.
				float groundY = resultPos[1];
				m_query->getPolyHeight(visited[visitedCount - 1], resultPos, &groundY);
				resultPos[1] = groundY;
				dtVcopy(m_anchorPos, resultPos);
				m_anchorRef = visited[visitedCount - 1];
				m_anchorSpeed = stepLen / dt;
			}
		}
		else
		{
			m_anchorSpeed = 0.0f;
		}
	}
	else
	{
		m_anchorSpeed = 0.0f;
	}

	updateFacing();

	// --- Narrow-passage degradation (with hysteresis) ------------------------
	const float diameter1 = m_config.memberRadius * 2.0f * 1.25f;
	const float col2Width = m_config.spacing + diameter1;

	// Width needed by the widest row of the base shape.
	float widestLocal = 0.0f;
	for (int i = 0; i < m_memberCount; ++i)
	{
		float lx, lz;
		computeSlotLocal(m_members[i].slotIdx, &lx, &lz);
		if (dtMathFabsf(lx) > widestLocal)
			widestLocal = dtMathFabsf(lx);
	}
	const float fullWidth = widestLocal * 2.0f + diameter1;

	// The restore threshold adds one agent diameter of margin so the
	// formation does not oscillate at a boundary width.
	const float restoreMargin = diameter1;
	int desiredState = m_state;
	if (m_probeWidth == FLT_MAX)
	{
		// No wall detected ahead: plenty of room.
		desiredState = DT_FORMATION_STATE_NORMAL;
	}
	else if (m_state == DT_FORMATION_STATE_COLUMN1)
	{
		if (m_probeWidth >= col2Width + restoreMargin)
			desiredState = (m_probeWidth >= fullWidth + restoreMargin)
				? DT_FORMATION_STATE_NORMAL : DT_FORMATION_STATE_COLUMN2;
	}
	else if (m_state == DT_FORMATION_STATE_COLUMN2)
	{
		if (m_probeWidth < col2Width)
			desiredState = DT_FORMATION_STATE_COLUMN1;
		else if (m_probeWidth >= fullWidth + restoreMargin)
			desiredState = DT_FORMATION_STATE_NORMAL;
	}
	else
	{
		if (m_probeWidth < col2Width)
			desiredState = DT_FORMATION_STATE_COLUMN1;
		else if (m_probeWidth < fullWidth)
			desiredState = DT_FORMATION_STATE_COLUMN2;
	}

	if (desiredState != m_state)
	{
		m_state = desiredState;
		m_forceRefresh = true;
	}

	// --- Slot layout and per-member goals (linear in member count) -----------
	// In a column, members keep a stable front-to-back order established by a
	// single linear selection of the foremost member each frame. Selection is
	// O(n): we remember the current front member and only swap when another
	// member is measurably further forward, avoiding per-frame target crosses
	// inside a narrow passage without a nested sort.
	int columnOrder[DT_FORMATION_MAX_MEMBERS];
	if (m_state == DT_FORMATION_STATE_COLUMN1 || m_state == DT_FORMATION_STATE_COLUMN2)
	{
		// Start from the stored member order and bubble just the foremost
		// member to the front in one linear pass. Repeated over frames this
		// converges to a sorted column while remaining O(n) per frame.
		for (int i = 0; i < m_memberCount; ++i)
			columnOrder[i] = i;
		int front = 0;
		for (int i = 1; i < m_memberCount; ++i)
		{
			const dtCrowdAgent* agFront = m_crowd->getAgent(m_members[columnOrder[front]].agentIdx);
			const dtCrowdAgent* agI = m_crowd->getAgent(m_members[i].agentIdx);
			const float projFront = agFront
				? (agFront->npos[0] * m_facing[0] + agFront->npos[2] * m_facing[2])
				: -FLT_MAX;
			const float projI = agI
				? (agI->npos[0] * m_facing[0] + agI->npos[2] * m_facing[2])
				: -FLT_MAX;
			if (projI > projFront)
				front = i;
		}
		// Place the foremost member first; the rest retain their relative
		// order, which keeps every other slot stable frame to frame.
		if (front != 0)
		{
			const int tmp = columnOrder[front];
			for (int i = front; i > 0; --i)
				columnOrder[i] = columnOrder[i - 1];
			columnOrder[0] = tmp;
		}
	}

	bool anyFallback = false;
	for (int i = 0; i < m_memberCount; ++i)
	{
		// In column mode i is the front-to-back column position; otherwise it
		// is the plain member index.
		const int memberIdx = (m_state == DT_FORMATION_STATE_COLUMN1 ||
							   m_state == DT_FORMATION_STATE_COLUMN2)
			? columnOrder[i] : i;
		Member* mem = &m_members[memberIdx];
		float lx, lz;

		if (m_state == DT_FORMATION_STATE_COLUMN1)
		{
			// Single file: everyone lines up behind the anchor.
			lx = 0.0f;
			lz = -((float)(i + 1)) * m_config.spacing;
		}
		else if (m_state == DT_FORMATION_STATE_COLUMN2)
		{
			// Two files, staggered longitudinally.
			const int row = i / 2;
			const float side = (i % 2 == 0) ? -0.5f : 0.5f;
			lx = side * m_config.spacing;
			lz = -((float)row) * m_config.depth;
		}
		else
		{
			computeSlotLocal(mem->slotIdx, &lx, &lz);
		}

		const float local[3] = { lx, 0.0f, lz };
		float desiredWorld[3];
		worldFromLocal(local, desiredWorld);

		// Single navmesh placement query for the slot goal.
		dtPolyRef slotRef = 0;
		float slotNearest[3];
		dtVcopy(slotNearest, desiredWorld);
		m_query->findNearestPoly(desiredWorld, m_crowd->getQueryHalfExtents(),
								 m_crowd->getFilter(0), &slotRef, slotNearest);

		if (slotRef)
		{
			dtVcopy(mem->slotGoal, slotNearest);
			mem->flags |= DT_FORMATION_MEMBER_SLOT_VALID;
			mem->flags &= (unsigned char)~DT_FORMATION_MEMBER_FALLBACK;
			requestMemberTarget(mem, slotRef, slotNearest);
		}
		else
		{
			// Slot is off the navmesh: steer the member toward the anchor
			// (or group goal) using ordinary individual pathfinding.
			mem->flags &= (unsigned char)~DT_FORMATION_MEMBER_SLOT_VALID;
			mem->flags |= DT_FORMATION_MEMBER_FALLBACK;
			anyFallback = true;

			dtPolyRef safeRef = m_anchorRef;
			const float* safePos = m_anchorPos;
			if (!safeRef && m_hasGoal)
			{
				safeRef = m_goalRef;
				safePos = m_goalPos;
			}
			if (safeRef)
				m_crowd->requestMoveTarget(mem->agentIdx, safeRef, safePos);
		}
	}

	if (anyFallback && m_state == DT_FORMATION_STATE_NORMAL)
		m_state = DT_FORMATION_STATE_DEGRADED;
	else if (!anyFallback && m_state == DT_FORMATION_STATE_DEGRADED)
		m_state = DT_FORMATION_STATE_NORMAL;

	m_forceRefresh = false;
}

int dtCrowdFormation::getMemberStates(dtFormationMemberState* out, int maxMembers) const
{
	if (!out || maxMembers <= 0)
		return 0;
	const int n = dtMin(m_memberCount, maxMembers);
	for (int i = 0; i < n; ++i)
	{
		const Member* m = &m_members[i];
		out[i].agentIdx = m->agentIdx;
		out[i].slotIdx = m->slotIdx;
		dtVcopy(out[i].slotGoal, m->slotGoal);
		out[i].flags = m->flags;
	}
	return n;
}

bool dtCrowdFormation::getMemberState(int agentIdx, dtFormationMemberState* out) const
{
	const Member* m = findMember(agentIdx);
	if (!m || !out)
		return false;
	out->agentIdx = m->agentIdx;
	out->slotIdx = m->slotIdx;
	dtVcopy(out->slotGoal, m->slotGoal);
	out->flags = m->flags;
	return true;
}

void dtCrowdFormation::getDebugInfo(dtFormationDebugInfo* out) const
{
	if (!out)
		return;
	out->state = m_state;
	dtVcopy(out->anchorPos, m_anchorPos);
	dtVcopy(out->facing, m_facing);
	out->probeWidth = m_probeWidth;
}
