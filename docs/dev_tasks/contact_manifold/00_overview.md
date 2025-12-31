# Persistent Contact Manifolds Overview (00)

## Status

- Stage 0 recon complete
- Stage 1 docs complete
- Stage 2 skeleton implementation complete
- Stage 3 wiring complete
- Stage 4 tests complete
- Stage 4.5 GUI demo complete
- Stage 5 benchmarks/doc polish ongoing

## Motivation

Current contact constraints are built from raw per-step detector contacts. This
creates instability for resting and stacking scenarios due to:

- Too few contacts for some shape pairs and backends, causing flickering support
- Large frame-to-frame variability in contact points, causing jitter
- Contact churn that makes constraint stabilization less effective

## Goals

- Maintain a persistent per-pair contact set with up to 4 representative points
- Update the set each step to reduce jitter while still tracking changes
- Keep the logic backend-agnostic at the DART layer
- Favor contiguous, cache-friendly storage and stable iteration
- Gate the new behavior behind a runtime feature flag (default OFF)

## Non-goals

- No change to raw contact generation inside backends
- No change to public collision APIs or the meaning of CollisionResult
- No multiple contact manifolds per pair in the initial version
- No behavioral changes for soft-body contacts in the first pass

## Current Pipeline (Stage 0)

- Collision detection runs in `ConstraintSolver::updateConstraints` and fills
  `mCollisionResult` using `CollisionGroup::collide`.
- Contact constraints are recreated every step from
  `mCollisionResult.getContacts()`.
- Per-pair contact counts are computed on the fly and passed into
  `ContactSurfaceHandler` for compliance scaling.
- `World::getLastCollisionResult()` exposes the raw contacts, and
  `World::bake()` records contact points and forces from that result.
- The only persistent contact history today is backend-specific in
  `dart/collision/ode/OdeCollisionDetector.cpp`.

## Proposed Integration Points

- Add a DART-level contact manifold cache owned by `ConstraintSolver`.
- Update it immediately after `mCollisionGroup->collide(...)` and before
  creating contact constraints.
- Use the persistent set for constraint construction when the feature flag is
  enabled, while leaving `mCollisionResult` unchanged.
- Sync constraint forces back into raw collision contacts for rigid bodies when
  the cache is enabled.
- Reset or prune the cache when collision detectors change or skeletons are
  added/removed.
- Keep soft contacts on the legacy path initially to reduce risk.
- Disable backend contact history during cached contact use to avoid double
  persistence (currently implemented for ODE).
- Recording (`World::bake`) uses contacts that fed constraints so forces and
  points remain consistent when the cache is enabled.
