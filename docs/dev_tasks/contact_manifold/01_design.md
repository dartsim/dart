# Contact Manifold Cache Design (01)

## Naming

Alternatives considered:

- ContactPairCache
- ContactPair
- ContactManifold
- ContactManifoldCache
- PersistentContactSet
- ContactPointCache
- ContactCluster
- PersistentContactPatch
- ContactPatchSet

Chosen: `ContactManifoldCache`

Rationale: "contact manifold" is a common term across physics engines, and
adding "cache" makes the persistence intent explicit while keeping the name
backend-agnostic.

## Data Model

### Pair Key

- Keyed by collision object pairs with a stable order
- `PairKey = (min(collisionObject1, collisionObject2), max(...))`
- Ordering is only used for deterministic storage and lookup within a run

### Manifold Storage

Goal: contiguous storage with stable iteration and minimal pointer chasing.

Proposed representation:

- `std::vector<ContactManifold>` where each manifold holds up to 4 points
- `ContactManifold` contains:
  - `PairKey pair`
  - `std::array<ContactManifoldPoint, kMaxManifoldPoints>`
  - `std::uint8_t count`
  - `std::uint16_t framesSinceSeen`
  - `std::uint32_t lastUpdateFrame`
- `ContactManifoldPoint` contains:
  - `collision::Contact contact`
  - `std::uint8_t age`

Output contacts for constraints are stored in a separate contiguous
`std::vector<collision::Contact>` per frame to provide stable references during
constraint construction.

### Configuration

A `ContactManifoldCacheOptions` struct owned by `ConstraintSolver`:

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
- points within a manifold are unique by position and normal thresholds
- deterministic ordering of points inside each manifold

## Update Algorithm Overview

For each simulation step:

1. Collect raw contacts from `CollisionResult` and group by pair key.
2. For each pair:
   - If the manifold exists, match raw contacts to existing points.
   - Otherwise, initialize a new manifold.
3. Update points in place, add new points if capacity allows, or replace based
   on coverage and depth rules.
4. Increment `framesSinceSeen` for pairs with no raw contacts. Drop manifolds that
   exceed TTL.
5. Emit a contiguous list of `collision::Contact` for constraint construction
   in a stable order.

### Matching and Replacement

- Match existing points to raw contacts if:
  - distance < `positionThreshold` AND
  - normal dot > `normalThreshold`
- Update matched points (point, normal, depth, age reset)
- For unmatched raw contacts:
  - If manifold has capacity: insert
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
- Drop the manifold when `framesSinceSeen > maxSeparationFrames`
- If a pair reappears before TTL expires, reuse and update the manifold

## Determinism Considerations

- Avoid iteration over unordered containers in output ordering
- For each step, sort pair keys and emit manifolds in that order
- Within each manifold, keep a stable ordering of points using tie-breakers:
  - deeper penetration first
  - then lexicographic position (x,y,z)
  - then normal direction
- Ensure all thresholds are fixed constants or user-configurable values
