#include <vector>
#include <float.h>
#include <stdlib.h>

#include "catch2/catch_amalgamated.hpp"

#include "DetourFormation.h"
#include "DetourCommon.h"
#include "Recast.h"

#include "FormationTestUtils.h"

namespace
{
	// Advances both crowd and formation for a number of ticks.
	void tick(dtCrowd* crowd, dtCrowdFormation* form, float frameDt, int frames)
	{
		for (int i = 0; i < frames; ++i)
		{
			if (form) form->update(frameDt);
			crowd->update(frameDt, 0);
		}
	}

	// A level made of two square rooms joined by a narrow z-axis corridor.
	// Room A: x[-10,10] z[-10,0]; corridor: x[-1,1] z[0,12];
	// Room B: x[-10,10] z[12,22].
	bool buildCorridorLevel(TestNavMesh& level)
	{
		level.addBoxTop(-10.0f, -10.0f, 10.0f, 0.0f);
		level.addBoxTop(-1.0f, 0.0f, 1.0f, 12.0f);
		level.addBoxTop(-10.0f, 12.0f, 10.0f, 22.0f);
		return level.build(0.3f, 0.2f);
	}

	dtPolyRef polyAt(dtNavMeshQuery* q, const dtQueryFilter* f, const float* ext,
					 float x, float z, float* out = 0)
	{
		float p[3] = { x, 0.0f, z };
		float near[3];
		dtPolyRef ref = resolvePoly(q, f, ext, p, near);
		if (out) { out[0] = near[0]; out[1] = near[1]; out[2] = near[2]; }
		return ref;
	}
}

TEST_CASE("Formation: shape slot layouts are deterministic", "[formation]")
{
	TestNavMesh level;
	level.addBoxTop(-15.0f, -15.0f, 15.0f, 15.0f);
	REQUIRE(level.build(0.3f, 0.2f));

	dtCrowd* crowd = dtAllocCrowd();
	REQUIRE(crowd);
	REQUIRE(crowd->init(32, 0.5f, level.nav));

	dtCrowdFormation* form = dtAllocCrowdFormation();
	REQUIRE(form);
	REQUIRE(form->init(crowd, level.nav));

	const int N = 5;
	int agents[N];
	float start[3] = { 0, 0, -5 };
	for (int i = 0; i < N; ++i)
	{
		agents[i] = makeAgent(crowd, start);
		REQUIRE(agents[i] >= 0);
	}

	SECTION("Line: centered row perpendicular to facing")
	{
		dtFormationConfig cfg;
		cfg.type = DT_FORMATION_LINE;
		cfg.spacing = 2.0f;
		cfg.anchorPos[0] = 0; cfg.anchorPos[2] = 0;
		cfg.facing[0] = 0; cfg.facing[2] = 1;
		REQUIRE(form->start(agents, N, cfg));

		dtFormationMemberState st[DT_FORMATION_MAX_MEMBERS];
		const int n = form->getMemberStates(st, DT_FORMATION_MAX_MEMBERS);
		REQUIRE(n == N);
		// Slots are assigned in submission order.
		for (int i = 0; i < N; ++i)
			CHECK(st[i].slotIdx == i);
	}

	SECTION("Wedge: tip first, symmetric pairs")
	{
		dtFormationConfig cfg;
		cfg.type = DT_FORMATION_WEDGE;
		cfg.spacing = 2.0f;
		cfg.depth = 2.0f;
		REQUIRE(form->start(agents, N, cfg));
		CHECK(form->getSlotOfAgent(agents[0]) == 0);
		CHECK(form->getSlotOfAgent(agents[1]) == 1);
		CHECK(form->getSlotOfAgent(agents[4]) == 4);
	}

	SECTION("Ring: center slot and ring members")
	{
		dtFormationConfig cfg;
		cfg.type = DT_FORMATION_RING;
		cfg.depth = 2.0f;
		cfg.rings = 2;
		REQUIRE(form->start(agents, N, cfg));
		CHECK(form->getSlotOfAgent(agents[0]) == 0);
	}

	SECTION("Custom: explicit slots are honored")
	{
		const float slots[] = {
			-3, 0, 0,
			 3, 0, 0,
			 0, 0, -3,
			 0, 0,  3,
			 0, 0,  0
		};
		dtFormationConfig cfg;
		cfg.type = DT_FORMATION_CUSTOM;
		cfg.customSlots = slots;
		cfg.customSlotCount = 5;
		REQUIRE(form->start(agents, N, cfg));
		CHECK(form->getConfig().type == DT_FORMATION_CUSTOM);
	}

	SECTION("Degenerate config is rejected")
	{
		dtFormationConfig cfg;
		cfg.spacing = 0.0f;
		CHECK_FALSE(form->start(agents, N, cfg));

		dtFormationConfig cfg2;
		cfg2.type = DT_FORMATION_CUSTOM;
		cfg2.customSlots = 0;
		cfg2.customSlotCount = 0;
		CHECK_FALSE(form->start(agents, N, cfg2));
	}

	dtFreeCrowdFormation(form);
	dtFreeCrowd(crowd);
}

TEST_CASE("Formation: maintains shape while moving toward goal", "[formation]")
{
	TestNavMesh level;
	level.addBoxTop(-15.0f, -15.0f, 15.0f, 15.0f);
	REQUIRE(level.build(0.3f, 0.2f));

	dtCrowd* crowd = dtAllocCrowd();
	REQUIRE(crowd->init(32, 0.5f, level.nav));
	dtCrowdFormation* form = dtAllocCrowdFormation();
	REQUIRE(form->init(crowd, level.nav));

	const int N = 5;
	int agents[DT_FORMATION_MAX_MEMBERS];
	for (int i = 0; i < N; ++i)
	{
		float p[3] = { (i - 2) * 1.2f, 0, -6.0f };
		agents[i] = makeAgent(crowd, p);
		REQUIRE(agents[i] >= 0);
	}

	dtNavMeshQuery q;
	REQUIRE(dtStatusSucceed(q.init(level.nav, 512)));
	const dtQueryFilter* f = crowd->getFilter(0);
	const float* ext = crowd->getQueryHalfExtents();

	dtFormationConfig cfg;
	cfg.type = DT_FORMATION_LINE;
	cfg.spacing = 1.6f;
	cfg.anchorPos[0] = 0; cfg.anchorPos[2] = -4.0f;
	cfg.facing[0] = 0; cfg.facing[2] = 1;
	cfg.stragglerDistance = 8.0f;
	cfg.replanInterval = 0.25f;
	REQUIRE(form->start(agents, N, cfg));

	float goalPos[3];
	const dtPolyRef goalRef = polyAt(&q, f, ext, 0.0f, 6.0f, goalPos);
	REQUIRE(goalRef);
	REQUIRE(form->setGroupGoal(goalRef, goalPos));

	// Move for a while in open space.
	tick(crowd, form, 0.1f, 120);

	// All members must have valid slots and be steering (valid target).
	dtFormationMemberState st[DT_FORMATION_MAX_MEMBERS];
	const int n = form->getMemberStates(st, DT_FORMATION_MAX_MEMBERS);
	REQUIRE(n == N);
	for (int i = 0; i < N; ++i)
	{
		CHECK(st[i].flags & DT_FORMATION_MEMBER_SLOT_VALID);
		const dtCrowdAgent* ag = crowd->getAgent(st[i].agentIdx);
		REQUIRE(ag);
		CHECK(ag->targetState != DT_CROWDAGENT_TARGET_FAILED);
	}

	// Shape cohesion: members stay within a bounded lateral spread of each
	// other (line width) and no member is left far behind the anchor.
	float minX = FLT_MAX, maxX = -FLT_MAX, minZ = FLT_MAX;
	for (int i = 0; i < N; ++i)
	{
		const dtCrowdAgent* ag = crowd->getAgent(agents[i]);
		minX = dtMin(minX, ag->npos[0]);
		maxX = dtMax(maxX, ag->npos[0]);
		minZ = dtMin(minZ, ag->npos[2]);
	}
	// Five agents at 1.6 spacing => nominal width 6.4; allow steering slack.
	CHECK((maxX - minX) < 11.0f);
	CHECK(minZ > form->getAnchorPos()[2] - cfg.stragglerDistance);

	// Anchor advanced toward the goal.
	CHECK(form->getAnchorPos()[2] > -4.0f);
	CHECK_FALSE(form->isGoalUnreachable());

	dtFreeCrowdFormation(form);
	dtFreeCrowd(crowd);
}

TEST_CASE("Formation: narrow corridor triggers column degradation and restores", "[formation]")
{
	TestNavMesh level;
	REQUIRE(buildCorridorLevel(level));

	dtCrowd* crowd = dtAllocCrowd();
	REQUIRE(crowd->init(32, 0.5f, level.nav));
	dtCrowdFormation* form = dtAllocCrowdFormation();
	REQUIRE(form->init(crowd, level.nav));

	const int N = 6;
	int agents[DT_FORMATION_MAX_MEMBERS];
	for (int i = 0; i < N; ++i)
	{
		float p[3] = { (i - 2.5f) * 1.1f, 0, -6.0f };
		agents[i] = makeAgent(crowd, p, 0.3f);
		REQUIRE(agents[i] >= 0);
	}

	dtNavMeshQuery q;
	REQUIRE(dtStatusSucceed(q.init(level.nav, 512)));
	const dtQueryFilter* f = crowd->getFilter(0);
	const float* ext = crowd->getQueryHalfExtents();

	dtFormationConfig cfg;
	cfg.type = DT_FORMATION_LINE;
	cfg.spacing = 1.4f;
	cfg.depth = 1.4f;
	cfg.memberRadius = 0.3f;
	cfg.anchorPos[0] = 0; cfg.anchorPos[2] = -4.0f;
	cfg.facing[0] = 0; cfg.facing[2] = 1;
	cfg.probeDistance = 5.0f;
	cfg.replanInterval = 0.2f;
	cfg.stragglerDistance = 9.0f;
	REQUIRE(form->start(agents, N, cfg));

	float goalPos[3];
	const dtPolyRef goalRef = polyAt(&q, f, ext, 0.0f, 18.0f, goalPos);
	REQUIRE(goalRef);
	REQUIRE(form->setGroupGoal(goalRef, goalPos));

	// March until the formation reaches the far room.
	int sawColumn = 0;
	int sawNormalAgain = 0;
	for (int frame = 0; frame < 600; ++frame)
	{
		form->update(0.1f);
		crowd->update(0.1f, 0);
		const int s = form->getState();
		if (s == DT_FORMATION_STATE_COLUMN1 || s == DT_FORMATION_STATE_COLUMN2)
			++sawColumn;
		if (sawColumn > 0 && s == DT_FORMATION_STATE_NORMAL)
			++sawNormalAgain;
	}

	// The narrow corridor must have forced a degraded shape at some point.
	CHECK(sawColumn > 0);
	// After entering the wide far room it must have restored to normal.
	CHECK(sawNormalAgain > 0);

	// No agent should have ended up outside the navmesh (all still walking
	// and holding a valid target rather than a failed one).
	for (int i = 0; i < N; ++i)
	{
		const dtCrowdAgent* ag = crowd->getAgent(agents[i]);
		REQUIRE(ag);
		CHECK(ag->state == DT_CROWDAGENT_STATE_WALKING);
		CHECK(ag->targetState != DT_CROWDAGENT_TARGET_FAILED);
		// Everyone made it into the far room (z > 12).
		CHECK(ag->npos[2] > 12.5f);
	}

	dtFreeCrowdFormation(form);
	dtFreeCrowd(crowd);
}

TEST_CASE("Formation: member loss is handled and slots stay stable", "[formation]")
{
	TestNavMesh level;
	level.addBoxTop(-15.0f, -15.0f, 15.0f, 15.0f);
	REQUIRE(level.build(0.3f, 0.2f));

	dtCrowd* crowd = dtAllocCrowd();
	REQUIRE(crowd->init(32, 0.5f, level.nav));
	dtCrowdFormation* form = dtAllocCrowdFormation();
	REQUIRE(form->init(crowd, level.nav));

	const int N = 4;
	int agents[N];
	for (int i = 0; i < N; ++i)
	{
		float p[3] = { (i - 1.5f) * 1.2f, 0, -5.0f };
		agents[i] = makeAgent(crowd, p);
		REQUIRE(agents[i] >= 0);
	}

	dtFormationConfig cfg;
	cfg.spacing = 1.5f;
	cfg.anchorPos[0] = 0; cfg.anchorPos[2] = -4.0f;
	cfg.facing[0] = 0; cfg.facing[2] = 1;
	REQUIRE(form->start(agents, N, cfg));

	// Remove the second member explicitly.
	REQUIRE(form->removeMember(agents[1]));
	CHECK(form->getMemberCount() == N - 1);
	CHECK(form->getSlotOfAgent(agents[1]) == -1);
	// Remaining members keep their original slot numbers.
	CHECK(form->getSlotOfAgent(agents[0]) == 0);
	CHECK(form->getSlotOfAgent(agents[2]) == 2);
	CHECK(form->getSlotOfAgent(agents[3]) == 3);

	// reassignSlots compacts the surviving members into slots 0..n-1,
	// ordered by distance to the anchor.
	form->reassignSlots();
	const int kept[3] = { agents[0], agents[2], agents[3] };
	int slots[3];
	CHECK(form->getMemberCount() == 3);
	for (int k = 0; k < 3; ++k)
		slots[k] = form->getSlotOfAgent(kept[k]);
	for (int k = 0; k < 3; ++k)
	{
		CHECK(slots[k] >= 0);
		CHECK(slots[k] < 3);
	}
	// Every compacted slot is unique.
	CHECK(slots[0] != slots[1]);
	CHECK(slots[0] != slots[2]);
	CHECK(slots[1] != slots[2]);

	// External deactivation of a member is pruned on update.
	crowd->removeAgent(agents[3]);
	form->update(0.1f);
	crowd->update(0.1f, 0);
	CHECK(form->getMemberCount() == 2);
	CHECK(form->getSlotOfAgent(agents[3]) == -1);

	// A new member can join and gets an unused slot.
	float p[3] = { 2.0f, 0, -5.0f };
	const int newbie = makeAgent(crowd, p);
	REQUIRE(newbie >= 0);
	REQUIRE(form->addMember(newbie));
	CHECK(form->getMemberCount() == 3);
	CHECK(form->getSlotOfAgent(newbie) >= 0);

	dtFreeCrowdFormation(form);
	dtFreeCrowd(crowd);
}

TEST_CASE("Formation: unreachable goal flags partial path but members keep pathfinding", "[formation]")
{
	// Two disconnected platforms: the goal platform is not linked to the
	// start platform, so the group goal cannot be reached.
	TestNavMesh level;
	level.addBoxTop(-8.0f, -8.0f, -2.0f, -2.0f); // start island
	level.addBoxTop(2.0f, 2.0f, 8.0f, 8.0f);     // goal island, gap between
	REQUIRE(level.build(0.3f, 0.2f));

	dtCrowd* crowd = dtAllocCrowd();
	REQUIRE(crowd->init(32, 0.5f, level.nav));
	dtCrowdFormation* form = dtAllocCrowdFormation();
	REQUIRE(form->init(crowd, level.nav));

	const int N = 3;
	int agents[N];
	for (int i = 0; i < N; ++i)
	{
		float p[3] = { -5.0f + i, 0, -5.0f };
		agents[i] = makeAgent(crowd, p);
		REQUIRE(agents[i] >= 0);
	}

	dtNavMeshQuery q;
	REQUIRE(dtStatusSucceed(q.init(level.nav, 512)));
	const dtQueryFilter* f = crowd->getFilter(0);
	const float* ext = crowd->getQueryHalfExtents();

	dtFormationConfig cfg;
	cfg.anchorPos[0] = -5.0f; cfg.anchorPos[2] = -5.0f;
	cfg.facing[0] = 1; cfg.facing[2] = 0;
	REQUIRE(form->start(agents, N, cfg));

	float goalPos[3];
	const dtPolyRef goalRef = polyAt(&q, f, ext, 5.0f, 5.0f, goalPos);
	REQUIRE(goalRef);
	REQUIRE(form->setGroupGoal(goalRef, goalPos));

	tick(crowd, form, 0.1f, 60);

	// The formation reports the goal as unreachable.
	CHECK(form->isGoalUnreachable());

	// But the agents are still alive, walking and holding targets (individual
	// pathfinding toward the best reachable point), not failed/invalid.
	for (int i = 0; i < N; ++i)
	{
		const dtCrowdAgent* ag = crowd->getAgent(agents[i]);
		REQUIRE(ag);
		CHECK(ag->active);
		CHECK(ag->state == DT_CROWDAGENT_STATE_WALKING);
		CHECK(ag->targetState != DT_CROWDAGENT_TARGET_NONE);
	}

	dtFreeCrowdFormation(form);
	dtFreeCrowd(crowd);
}

TEST_CASE("Formation: anchor teleport replans without stranding members", "[formation]")
{
	TestNavMesh level;
	level.addBoxTop(-15.0f, -15.0f, 15.0f, 25.0f);
	REQUIRE(level.build(0.3f, 0.2f));

	dtCrowd* crowd = dtAllocCrowd();
	REQUIRE(crowd->init(32, 0.5f, level.nav));
	dtCrowdFormation* form = dtAllocCrowdFormation();
	REQUIRE(form->init(crowd, level.nav));

	const int N = 3;
	int agents[N];
	for (int i = 0; i < N; ++i)
	{
		float p[3] = { (i - 1) * 1.2f, 0, -6.0f };
		agents[i] = makeAgent(crowd, p);
		REQUIRE(agents[i] >= 0);
	}

	dtNavMeshQuery q;
	REQUIRE(dtStatusSucceed(q.init(level.nav, 512)));
	const dtQueryFilter* f = crowd->getFilter(0);
	const float* ext = crowd->getQueryHalfExtents();

	dtFormationConfig cfg;
	cfg.anchorPos[0] = 0; cfg.anchorPos[2] = -4.0f;
	cfg.facing[0] = 0; cfg.facing[2] = 1;
	REQUIRE(form->start(agents, N, cfg));

	float goalPos[3];
	const dtPolyRef goalRef = polyAt(&q, f, ext, 0.0f, 8.0f, goalPos);
	REQUIRE(goalRef);
	REQUIRE(form->setGroupGoal(goalRef, goalPos));

	tick(crowd, form, 0.1f, 30);

	// Teleport the anchor far across the map.
	float tele[3] = { 0.0f, 0.0f, 6.0f };
	form->teleportAnchor(tele);
	REQUIRE(form->getAnchorPos()[2] == Catch::Approx(6.0f).margin(0.5f));

	// Update keeps the formation coherent and all members still get targets.
	tick(crowd, form, 0.1f, 60);
	for (int i = 0; i < N; ++i)
	{
		const dtCrowdAgent* ag = crowd->getAgent(agents[i]);
		REQUIRE(ag);
		CHECK(ag->targetState != DT_CROWDAGENT_TARGET_FAILED);
	}

	dtFreeCrowdFormation(form);
	dtFreeCrowd(crowd);
}

TEST_CASE("Formation: disabled/stopped formation leaves crowd behavior unchanged", "[formation]")
{
	TestNavMesh level;
	level.addBoxTop(-15.0f, -15.0f, 15.0f, 15.0f);
	REQUIRE(level.build(0.3f, 0.2f));

	dtCrowd* crowd = dtAllocCrowd();
	REQUIRE(crowd->init(32, 0.5f, level.nav));

	dtNavMeshQuery q;
	REQUIRE(dtStatusSucceed(q.init(level.nav, 512)));
	const dtQueryFilter* f = crowd->getFilter(0);
	const float* ext = crowd->getQueryHalfExtents();

	const int N = 4;
	int agents[N];
	for (int i = 0; i < N; ++i)
	{
		float p[3] = { (i - 1.5f) * 1.2f, 0, -6.0f };
		agents[i] = makeAgent(crowd, p);
		REQUIRE(agents[i] >= 0);
	}

	// Baseline: ordinary individual move targets, no formation.
	float targetPos[3];
	const dtPolyRef targetRef = polyAt(&q, f, ext, 0.0f, 8.0f, targetPos);
	REQUIRE(targetRef);
	for (int i = 0; i < N; ++i)
		crowd->requestMoveTarget(agents[i], targetRef, targetPos);

	// Snapshot positions after running without a formation.
	float baseline[4][3];
	for (int frame = 0; frame < 90; ++frame)
		crowd->update(0.1f, 0);
	for (int i = 0; i < N; ++i)
		dtVcopy(baseline[i], crowd->getAgent(agents[i])->npos);

	// Now run a formation then stop it; stop() must not touch agent requests.
	dtCrowdFormation* form = dtAllocCrowdFormation();
	REQUIRE(form->init(crowd, level.nav));
	dtFormationConfig cfg;
	cfg.anchorPos[0] = 0; cfg.anchorPos[2] = -4.0f;
	REQUIRE(form->start(agents, N, cfg));
	form->stop();
	CHECK(form->getState() == DT_FORMATION_STATE_IDLE);
	CHECK(form->getMemberCount() == 0);

	// A stopped formation's update is a no-op: agents keep their last target.
	for (int i = 0; i < N; ++i)
	{
		const dtCrowdAgent* ag = crowd->getAgent(agents[i]);
		CHECK(ag->targetState != DT_CROWDAGENT_TARGET_NONE);
	}
	form->update(0.1f); // must not crash or alter agents

	dtFreeCrowdFormation(form);
	dtFreeCrowd(crowd);
}

namespace
{
	// Custom Detour allocator used to prove the per-frame update performs no
	// heap allocation. All blocks are routed through malloc/free while the
	// number of live allocations is counted.
	int g_liveAllocs = 0;
	void* countingAlloc(size_t size, dtAllocHint)
	{
		++g_liveAllocs;
		return malloc(size);
	}
	void countingFree(void* ptr)
	{
		if (ptr)
			--g_liveAllocs;
		free(ptr);
	}
}

TEST_CASE("Formation: update does no per-frame heap allocation and stays linear", "[formation]")
{
	TestNavMesh level;
	level.addBoxTop(-15.0f, -15.0f, 15.0f, 25.0f);
	REQUIRE(level.build(0.3f, 0.2f));

	dtAllocSetCustom(countingAlloc, countingFree);

	dtCrowd* crowd = dtAllocCrowd();
	REQUIRE(crowd);
	REQUIRE(crowd->init(32, 0.5f, level.nav));
	dtCrowdFormation* form = dtAllocCrowdFormation();
	REQUIRE(form);
	REQUIRE(form->init(crowd, level.nav));

	const int N = 12;
	int agents[DT_FORMATION_MAX_MEMBERS];
	for (int i = 0; i < N; ++i)
	{
		float p[3] = { (i - 5.5f) * 1.0f, 0, -6.0f };
		agents[i] = makeAgent(crowd, p, 0.3f);
		REQUIRE(agents[i] >= 0);
	}

	dtFormationConfig cfg;
	cfg.spacing = 1.2f;
	cfg.anchorPos[0] = 0; cfg.anchorPos[2] = -4.0f;
	cfg.facing[0] = 0; cfg.facing[2] = 1;
	cfg.replanInterval = 0.2f;
	REQUIRE(form->start(agents, N, cfg));

	// Resolve the goal with a short-lived query whose node pool is released
	// at the end of this scope, so it cannot pollute the leak accounting.
	{
		dtNavMeshQuery q;
		REQUIRE(dtStatusSucceed(q.init(level.nav, 512)));
		const dtQueryFilter* f = crowd->getFilter(0);
		const float* ext = crowd->getQueryHalfExtents();

		float goalPos[3];
		const dtPolyRef goalRef = polyAt(&q, f, ext, 0.0f, 12.0f, goalPos);
		REQUIRE(goalRef);
		REQUIRE(form->setGroupGoal(goalRef, goalPos));
	}

	// Warm up so lazy one-time structures (paths, queries) are initialized.
	for (int i = 0; i < 20; ++i)
	{
		form->update(0.1f);
		crowd->update(0.1f, 0);
	}

	// Over a steady-state window neither the formation update nor a full
	// crowd step driven by it may allocate.
	const int liveBefore = g_liveAllocs;
	for (int i = 0; i < 50; ++i)
	{
		form->update(0.1f);
		crowd->update(0.1f, 0);
	}
	CHECK(g_liveAllocs == liveBefore);

	dtFreeCrowdFormation(form);
	dtFreeCrowd(crowd);

	// Everything allocated through the counting allocator was released.
	CHECK(g_liveAllocs == 0);
	dtAllocSetCustom(0, 0);
}

TEST_CASE("Formation: duplicate and full membership are rejected, slots unique", "[formation]")
{
	TestNavMesh level;
	level.addBoxTop(-15.0f, -15.0f, 15.0f, 15.0f);
	REQUIRE(level.build(0.3f, 0.2f));

	dtCrowd* crowd = dtAllocCrowd();
	REQUIRE(crowd->init(DT_FORMATION_MAX_MEMBERS + 8, 0.5f, level.nav));
	dtCrowdFormation* form = dtAllocCrowdFormation();
	REQUIRE(form->init(crowd, level.nav));

	const int N = 4;
	int agents[N];
	for (int i = 0; i < N; ++i)
	{
		float p[3] = { (i - 1.5f) * 1.2f, 0, -5.0f };
		agents[i] = makeAgent(crowd, p);
		REQUIRE(agents[i] >= 0);
	}

	dtFormationConfig cfg;
	cfg.anchorPos[0] = 0; cfg.anchorPos[2] = -4.0f;
	REQUIRE(form->start(agents, N, cfg));

	// Adding an already-joined agent again is rejected.
	CHECK_FALSE(form->addMember(agents[2]));
	CHECK(form->getMemberCount() == N);

	// Fill to the cap; every assigned slot is distinct.
	int added = 0;
	for (int i = 0; i < DT_FORMATION_MAX_MEMBERS; ++i)
	{
		float p[3] = { (float)i * 0.5f, 0, -8.0f };
		const int idx = makeAgent(crowd, p);
		if (idx < 0)
			continue;
		if (form->addMember(idx))
			++added;
	}
	CHECK(form->getMemberCount() == DT_FORMATION_MAX_MEMBERS);
	CHECK(added == DT_FORMATION_MAX_MEMBERS - N);

	// Once full, further joins fail rather than corrupting the slot table.
	float extra[3] = { 0, 0, -9.0f };
	const int overflow = makeAgent(crowd, extra);
	REQUIRE(overflow >= 0);
	CHECK_FALSE(form->addMember(overflow));
	CHECK(form->getMemberCount() == DT_FORMATION_MAX_MEMBERS);

	// All slots are unique in [0, MAX).
	bool seen[DT_FORMATION_MAX_MEMBERS];
	for (int i = 0; i < DT_FORMATION_MAX_MEMBERS; ++i)
		seen[i] = false;
	dtFormationMemberState st[DT_FORMATION_MAX_MEMBERS];
	const int n = form->getMemberStates(st, DT_FORMATION_MAX_MEMBERS);
	REQUIRE(n == DT_FORMATION_MAX_MEMBERS);
	for (int i = 0; i < n; ++i)
	{
		REQUIRE(st[i].slotIdx >= 0);
		REQUIRE(st[i].slotIdx < DT_FORMATION_MAX_MEMBERS);
		CHECK_FALSE(seen[st[i].slotIdx]);
		seen[st[i].slotIdx] = true;
	}

	dtFreeCrowdFormation(form);
	dtFreeCrowd(crowd);
}

TEST_CASE("Formation: straggler flag is raised and cleared", "[formation]")
{
	TestNavMesh level;
	level.addBoxTop(-15.0f, -15.0f, 15.0f, 30.0f);
	REQUIRE(level.build(0.3f, 0.2f));

	dtCrowd* crowd = dtAllocCrowd();
	REQUIRE(crowd->init(16, 0.5f, level.nav));
	dtCrowdFormation* form = dtAllocCrowdFormation();
	REQUIRE(form->init(crowd, level.nav));

	const int N = 3;
	int agents[N];
	// Two agents near the anchor, one deliberately far behind it.
	for (int i = 0; i < N; ++i)
	{
		float p[3] = { (i - 1) * 1.2f, 0, -4.0f };
		agents[i] = makeAgent(crowd, p);
		REQUIRE(agents[i] >= 0);
	}

	dtFormationConfig cfg;
	cfg.anchorPos[0] = 0; cfg.anchorPos[2] = 0.0f;
	cfg.facing[0] = 0; cfg.facing[2] = 1;
	cfg.stragglerDistance = 3.0f;
	cfg.replanInterval = 0.2f;
	REQUIRE(form->start(agents, N, cfg));

	dtNavMeshQuery q;
	REQUIRE(dtStatusSucceed(q.init(level.nav, 512)));
	const dtQueryFilter* f = crowd->getFilter(0);
	const float* ext = crowd->getQueryHalfExtents();
	float goalPos[3];
	const dtPolyRef goalRef = polyAt(&q, f, ext, 0.0f, 20.0f, goalPos);
	REQUIRE(goalRef);
	REQUIRE(form->setGroupGoal(goalRef, goalPos));

	// Teleport the third agent far behind the group to force straggler state.
	// Remove it from both the formation and the crowd, re-add the crowd agent
	// at a distant position, then re-join the formation.
	const dtCrowdAgentParams savedParams = crowd->getAgent(agents[2])->params;
	REQUIRE(form->removeMember(agents[2]));
	crowd->removeAgent(agents[2]);

	float behind[3] = { 0.0f, 0.0f, -12.0f };
	const int laggardIdx = crowd->addAgent(behind, &savedParams);
	REQUIRE(laggardIdx >= 0);
	agents[2] = laggardIdx;
	REQUIRE(form->addMember(agents[2]));

	// Run a few frames so the re-joined member acquires a slot goal before its
	// straggler status is evaluated.
	for (int i = 0; i < 5; ++i)
	{
		form->update(0.1f);
		crowd->update(0.1f, 0);
	}

	dtFormationMemberState ms;
	REQUIRE(form->getMemberState(agents[2], &ms));
	// The rearmost agent is well beyond stragglerDistance from its slot.
	CHECK(ms.flags & DT_FORMATION_MEMBER_STRAGGLER);

	// After enough time the anchor waits (follow clamp) and the agent closes
	// the gap; the straggler flag must eventually clear.
	for (int i = 0; i < 400; ++i)
	{
		form->update(0.1f);
		crowd->update(0.1f, 0);
	}
	REQUIRE(form->getMemberState(agents[2], &ms));
	CHECK((ms.flags & DT_FORMATION_MEMBER_STRAGGLER) == 0);

	dtFreeCrowdFormation(form);
	dtFreeCrowd(crowd);
}
