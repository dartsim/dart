# Contact Patch Cache Design (01)

## Naming

Alternatives considered:

- PersistentContactPatch
- ContactPatchCache
- PersistentContactSet
- ContactPointCache
- ContactCluster
- ContactPatchSet

Chosen: `ContactPatchCache`

Rationale: it emphasizes persistence and caching at the DART layer without
reusing the term "manifold", and it describes a small patch of representative
points rather than all raw contacts.

## Data Model

### Pair Key

- Keyed by collision object pairs with a stable order
- `PairKey = (min(collisionObject1, collisionObject2), max(...))`
- Ordering is only used for deterministic storage and lookup within a run

### Patch Storage

Goal: contiguous storage with stable iteration and minimal pointer chasing.

Proposed representation:

- `std::vector<ContactPatch>` where each patch holds up to 4 points
- `ContactPatch` contains:
  - `PairKey pair`
  - `std::array<ContactPoint, kMaxPoints>`
  - `std::uint8_t count`
  - `std::uint16_t framesSinceSeen`
  - `std::uint32_t lastUpdateFrame`
- `ContactPoint` contains:
  - `Eigen::Vector3d point`
  - `Eigen::Vector3d normal`
  - `double penetrationDepth`
  - `std::uint8_t age`

Output contacts for constraints are stored in a separate contiguous
`std::vector<collision::Contact>` per frame to provide stable references during
constraint construction.

### Configuration

A `ContactPatchCacheOptions` struct owned by `ConstraintSolver`:

- `bool enabled` (default false)
- `std::size_t maxPointsPerPair` (default 4)
- `std::size_t maxPairs` (optional cap)
- `std::size_t maxSeparationFrames` (TTL before drop)
- `double positionThreshold` (uniqueness distance)
- `double normalThreshold` (uniqueness by angle or dot)
- `double depthEpsilon` (tie-breaking for penetration depth)

## Invariants

- `count <= maxPointsPerPair`
- normals are non-zero (skip invalid raw contacts)
- penetration depth is non-negative unless the global option allows otherwise
- points within a patch are unique by position and normal thresholds
- deterministic ordering of points inside each patch

## Update Algorithm Overview

For each simulation step:

1. Collect raw contacts from `CollisionResult` and group by pair key.
2. For each pair:
   - If the patch exists, match raw contacts to existing points.
   - Otherwise, initialize a new patch.
3. Update points in place, add new points if capacity allows, or replace based
   on coverage and depth rules.
4. Increment `framesSinceSeen` for pairs with no raw contacts. Drop patches that
   exceed TTL.
5. Emit a contiguous list of `collision::Contact` for constraint construction
   in a stable order.

### Matching and Replacement

- Match existing points to raw contacts if:
  - distance < `positionThreshold` AND
  - normal dot > `normalThreshold`
- Update matched points (point, normal, depth, age reset)
- For unmatched raw contacts:
  - If patch has capacity: insert
  - If full: replace a point using the selection policy below

### Selection Policy (<= 4 points)

Heuristic to balance stability and coverage:

- Always keep the deepest penetration contact from the current raw set
- For remaining points, use greedy farthest-point sampling:
  - at each step, pick the point with the largest minimum distance to already
    selected points
- Apply uniqueness thresholds before inserting to reduce churn

### When There Are More Than 4 Candidates

- Start with the deepest contact
- Sort remaining candidates by a coverage score (distance and area proxy)
- Insert in deterministic order, stopping at 4

## Pair Lifetime and Sleeping

- If a pair has no raw contacts in a step, increment `framesSinceSeen`
- Continue emitting the cached contacts for up to `maxSeparationFrames` while
  the pair is unseen to avoid flicker
- Drop the patch when `framesSinceSeen > maxSeparationFrames`
- If a pair reappears before TTL expires, reuse and update the patch

## Determinism Considerations

- Avoid iteration over unordered containers in output ordering
- For each step, sort pair keys and emit patches in that order
- Within each patch, keep a stable ordering of points using tie-breakers:
  - deeper penetration first
  - then lexicographic position (x,y,z)
  - then normal direction
- Ensure all thresholds are fixed constants or user-configurable values
