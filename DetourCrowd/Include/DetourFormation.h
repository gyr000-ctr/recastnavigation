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

#ifndef DETOURFORMATION_H
#define DETOURFORMATION_H

#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"

/// The maximum number of members a single #dtCrowdFormation can manage.
/// All per-member storage is pre-allocated, so no per-frame heap allocations
/// are needed regardless of how many members are present.
/// @ingroup formation
static const int DT_FORMATION_MAX_MEMBERS = 64;

/// The maximum number of polygon references in the formation anchor path.
/// @ingroup formation
static const int DT_FORMATION_MAX_PATH = 256;

/// Formation shape templates.
/// @ingroup formation
enum dtFormationType
{
	/// Members are arranged in a line perpendicular to the movement direction.
	DT_FORMATION_LINE = 0,

	/// Members are arranged in a V/wedge shape pointing along the movement
	/// direction.
	DT_FORMATION_WEDGE = 1,

	/// Members are arranged in concentric rings around the anchor.
	DT_FORMATION_RING = 2,

	/// The caller provides explicit slot positions (in formation-local
	/// coordinates, relative to the anchor, x = right, z = forward).
	DT_FORMATION_CUSTOM = 3
};

/// Coarse formation state. The formation automatically degrades into a
/// narrower shape when the navigable space ahead is constrained and
/// restores itself once there is room again.
/// @ingroup formation
enum dtFormationState
{
	/// The formation is not active or has no valid target.
	DT_FORMATION_STATE_IDLE = 0,

	/// Using the requested shape at full spacing.
	DT_FORMATION_STATE_NORMAL = 1,

	/// Space ahead is tight: slots are compressed toward the movement axis.
	DT_FORMATION_STATE_COLUMN2 = 2,

	/// Space ahead is very tight: members follow in a single file column.
	DT_FORMATION_STATE_COLUMN1 = 3,

	/// One or more slot goals could not be placed on the navmesh. Members
	/// keep their last valid goals and fall back to individual pathfinding
	/// where necessary.
	DT_FORMATION_STATE_DEGRADED = 4
};

/// Per-member runtime status flags. Combine with bitwise OR.
/// @ingroup formation
enum dtFormationMemberFlags
{
	/// The member currently has a formation assigned slot goal.
	DT_FORMATION_MEMBER_SLOT_VALID = 0x01,

	/// The member is lagging behind its slot and has been flagged as
	/// straggling. The formation slows down while it catches up.
	DT_FORMATION_MEMBER_STRAGGLER = 0x02,

	/// No reachable navmesh position could be found for the member's slot;
	/// the member is steering toward the group goal with individual
	/// pathfinding instead.
	DT_FORMATION_MEMBER_FALLBACK = 0x04
};

/// Configuration of a formation. A copy of this structure is kept inside
/// #dtCrowdFormation; mutate it through #dtCrowdFormation::updateConfig().
/// @ingroup formation
struct dtFormationConfig
{
	dtFormationConfig() :
		type(DT_FORMATION_LINE),
		spacing(2.0f),
		depth(2.0f),
		rings(1),
		customSlots(0),
		customSlotCount(0),
		slotGoalRefreshDistance(1.0f),
		replanInterval(0.5f),
		stragglerDistance(6.0f),
		catchUpSpeedScale(0.6f),
		probeDistance(6.0f),
		memberRadius(0.5f)
	{
		anchorPos[0] = anchorPos[1] = anchorPos[2] = 0.0f;
		facing[0] = 0.0f; facing[1] = 0.0f; facing[2] = 1.0f;
	}

	/// Requested formation shape. (See: #dtFormationType)
	int type;

	/// Center of the formation in world space. The anchor is the frame of
	/// reference for every slot and is moved along the group path.
	float anchorPos[3];

	/// Facing direction of the formation in world space (xz plane). The
	/// vector is normalized internally; a zero vector keeps the previous
	/// facing.
	float facing[3];

	/// Distance between adjacent slots. Must be > 0.
	float spacing;

	/// Longitudinal distance between rows (wedge) / rings (ring). Must be > 0.
	float depth;

	/// Number of rings requested for #DT_FORMATION_RING. Ring 0 is the
	/// anchor; members are packed on hexagonal rings (6, 12, 18, ... slots),
	/// so the actual ring count grows with the member count and this value
	/// only documents the intended ring depth used to scale the radius.
	int rings;

	/// For #DT_FORMATION_CUSTOM: slot offsets in formation-local space,
	/// x = right of the facing, z = forward of the facing, packed as
	/// (x, y, z) triples. Pointer is borrowed and only read while the
	/// custom type is active; pass 0 to clear.
	const float* customSlots;

	/// Number of triples in #customSlots.
	int customSlotCount;

	/// A member's move target is re-submitted once its slot goal has moved
	/// more than this distance (world units). Keeps per-frame work and path
	/// requests low.
	float slotGoalRefreshDistance;

	/// How often (seconds) the anchor path and local width probes are
	/// re-evaluated. Slot goal placement runs every frame regardless.
	float replanInterval;

	/// A member further than this distance (world units) behind its slot is
	/// considered a straggler.
	float stragglerDistance;

	/// When at least one member is straggling, the anchor advance speed is
	/// scaled by this factor (0..1) so the group can close up.
	float catchUpSpeedScale;

	/// How far ahead of the anchor the width probes are cast.
	float probeDistance;

	/// Radius used when snapping slot goals to the navmesh and when
	/// estimating whether a column fits through a gap.
	float memberRadius;
};

/// Read-only per-member snapshot returned by
/// #dtCrowdFormation::getMemberState().
/// @ingroup formation
struct dtFormationMemberState
{
	/// Crowd agent index, or -1 when the slot is unused.
	int agentIdx;

	/// Stable slot index within the shape template.
	int slotIdx;

	/// Current slot goal in world space (valid when the status contains
	/// #DT_FORMATION_MEMBER_SLOT_VALID).
	float slotGoal[3];

	/// Bitfield of #dtFormationMemberFlags.
	unsigned char flags;
};

/// Optional debug snapshot of the formation internals.
/// @ingroup formation
struct dtFormationDebugInfo
{
	/// Current coarse formation state. (See: #dtFormationState)
	int state;

	/// Anchor position after this update.
	float anchorPos[3];

	/// Current facing direction (normalized, xz plane).
	float facing[3];

	/// Estimated free width at the probe point ahead of the anchor.
	float probeWidth;
};

class dtNavMeshQuery;
class dtCrowd;

/// Manages a single formation: slot assignment, an anchor path, local
/// width probing for narrow-passage degradation, and per-member slot goal
/// generation. The class never moves agents directly; on each update it
/// submits ordinary move targets through the owning #dtCrowd, so the
/// existing per-agent steering, avoidance and collision resolution are
/// reused unchanged.
///
/// All storage is pre-allocated. #update() does not perform any heap
/// allocation and does work proportional to the member count plus a small,
/// member-independent number of navigation queries.
/// @ingroup formation
class dtCrowdFormation
{
public:
	dtCrowdFormation();
	~dtCrowdFormation();

	/// Allocates the formation. Must be called before any other method.
	///  @param[in]		crowd	The crowd that owns the agents.
	///  @param[in]		nav		The navigation mesh used for group planning.
	/// @return True if the initialization succeeded.
	bool init(dtCrowd* crowd, dtNavMesh* nav);

	/// Purges all members and internal state. Safe to call repeatedly.
	void purge();

	/// Starts (or restarts) the formation. Members are assigned shape slots
	/// in the order they appear in @p agentIndices.
	///  @param[in]		agentIndices	Crowd agent indices to include.
	///  @param[in]		agentCount		Number of indices
	///									[Limit: 1 .. #DT_FORMATION_MAX_MEMBERS].
	///  @param[in]		config			Formation configuration.
	/// @return True if the formation was started.
	bool start(const int* agentIndices, int agentCount, const dtFormationConfig& config);

	/// Replaces the configuration while keeping the current member/slot
	/// assignment. Degenerate values (zero/negative spacing, zero facing)
	/// are rejected and the previous configuration is kept.
	///  @param[in]		config	New configuration.
	/// @return True if the configuration was accepted.
	bool updateConfig(const dtFormationConfig& config);

	/// Sets the group goal. A goal on an invalid or unreachable polygon is
	/// reported through #getState()/members falling back to individual
	/// pathfinding and does not stop the update.
	///  @param[in]		ref		Polygon reference of the goal (0 to resolve
	///							the nearest poly to @p pos).
	///  @param[in]		pos		Goal position. [(x, y, z)]
	/// @return True if the request was accepted.
	bool setGroupGoal(dtPolyRef ref, const float* pos);

	/// Teleports the anchor to a new position, e.g. when the group logic
	/// driving the formation jumps. The anchor path is replanned and slot
	/// goals are refreshed on the next update.
	///  @param[in]		pos		New anchor position. [(x, y, z)]
	void teleportAnchor(const float* pos);

	/// Adds a member to the formation. The member receives the lowest
	/// unused slot; if every slot is occupied the call fails.
	///  @param[in]		agentIdx	Crowd agent index.
	/// @return True if the member was added.
	bool addMember(int agentIdx);

	/// Removes a member. Remaining members keep their slot assignment, so
	/// removing a member never causes a mass reshuffle; use
	/// #reassignSlots() to compact the slots again.
	///  @param[in]		agentIdx	Crowd agent index.
	/// @return True if the member was part of the formation.
	bool removeMember(int agentIdx);

	/// Recomputes slot assignments for the current membership, ordered by
	/// distance to the anchor so that members already in position keep
	/// nearby slots.
	void reassignSlots();

	/// Stops the formation. Member agents keep their current move targets
	/// and continue as ordinary crowd agents; the formation returns to
	/// #DT_FORMATION_STATE_IDLE.
	void stop();

	/// Per-frame update. Submits ordinary crowd move targets for the
	/// member agents. Does no heap allocation.
	///  @param[in]		dt		Elapsed time in seconds. [Limit: > 0]
	void update(float dt);

	/// Current coarse formation state. (See: #dtFormationState)
	int getState() const { return m_state; }

	/// Number of currently tracked members.
	int getMemberCount() const { return m_memberCount; }

	/// Returns the active configuration.
	const dtFormationConfig& getConfig() const { return m_config; }

	/// Returns the current anchor position.
	const float* getAnchorPos() const { return m_anchorPos; }

	/// Returns the anchor's current speed along its path (world units/sec).
	float getAnchorSpeed() const { return m_anchorSpeed; }

	/// Returns the current (normalized) facing direction.
	const float* getFacing() const { return m_facing; }

	/// Returns the group goal position, or 0 when no goal is set.
	const float* getGoalPos() const { return m_hasGoal ? m_goalPos : 0; }

	/// Returns the group goal polygon reference.
	dtPolyRef getGoalRef() const { return m_goalRef; }

	/// True when the goal polygon could not be reached from the anchor
	/// (partial/empty anchor path). Individual members still path toward
	/// the best reachable point.
	bool isGoalUnreachable() const { return m_goalUnreachable; }

	/// Fills a per-member status snapshot.
	///  @param[out]	out			Output array.
	///  @param[in]		maxMembers	Capacity of @p out.
	/// @return Number of entries written.
	int getMemberStates(dtFormationMemberState* out, int maxMembers) const;

	/// Returns the state of a single member, or false if the agent is not
	/// part of the formation.
	bool getMemberState(int agentIdx, dtFormationMemberState* out) const;

	/// Returns the slot index assigned to an agent, or -1.
	int getSlotOfAgent(int agentIdx) const;

	/// Fills the optional debug snapshot.
	void getDebugInfo(dtFormationDebugInfo* out) const;

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtCrowdFormation(const dtCrowdFormation&);
	dtCrowdFormation& operator=(const dtCrowdFormation&);

	struct Member
	{
		int agentIdx;
		int slotIdx;
		float slotGoal[3];
		float lastGoal[3];
		unsigned char flags;
	};

	void assignSlotsDefault();
	void computeSlotLocal(int slotIdx, float* x, float* z) const;
	void worldFromLocal(const float* local, float* world) const;
	void updateAnchorPath();
	void probeWidth(float& width) const;
	void updateFacing();
	bool ensureGoalRef();
	void requestMemberTarget(Member* m, dtPolyRef ref, const float* pos);
	Member* findMember(int agentIdx);
	const Member* findMember(int agentIdx) const;

	dtCrowd* m_crowd;
	dtNavMesh* m_nav;
	dtNavMeshQuery* m_query;

	dtFormationConfig m_config;
	Member m_members[DT_FORMATION_MAX_MEMBERS];
	int m_memberCount;
	int m_state;

	// Anchor path (polygon corridor following the group goal).
	dtPolyRef m_path[DT_FORMATION_MAX_PATH];
	int m_pathCount;
	float m_pathTarget[3];
	dtPolyRef m_anchorRef;

	float m_anchorPos[3];
	float m_facing[3];
	float m_anchorSpeed;
	float m_replanTimer;
	float m_probeWidth;

	dtPolyRef m_goalRef;
	float m_goalPos[3];
	bool m_hasGoal;
	bool m_goalUnreachable;
	bool m_forceRefresh;
};

/// Allocates a formation object using the Detour allocator.
/// @return A formation object ready for #dtCrowdFormation::init(), or null.
///  @ingroup formation
dtCrowdFormation* dtAllocCrowdFormation();

/// Frees a formation object allocated with #dtAllocCrowdFormation().
///  @ingroup formation
void dtFreeCrowdFormation(dtCrowdFormation* ptr);

#endif // DETOURFORMATION_H
