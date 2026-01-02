# World Split: Classic and ECS Worlds (Design Note)

## Status

- Draft: replace the multi-solver approach with two dedicated worlds.

## Goals

- Keep `dart::simulation::World` stable for classic `Skeleton`/`Frame` workflows.
- Provide a new ECS-first world for new object types without exposing EnTT in
  classic headers.
- Avoid mixing solver backends inside a single world.

## Non-Goals

- No multi-solver orchestration inside a single world.
- No automatic interop between classic and ECS worlds.
- No public EnTT exposure in classic `dart/` headers.

## Proposed Shape

- **Classic world**
  - `dart::simulation::World` stays as-is (Skeleton/Frame + ConstraintSolver).
  - Classic solver behavior remains the default and only Skeleton path.

- **ECS world**
  - New `dart::ecs::World` (preferred) owns the ECS registry and ECS object APIs.
  - Object handles are opaque IDs + typed wrappers (no `entt::registry` in
    public headers).

- **Shared internals**
  - Optional internal `WorldBase` for name/time/step bookkeeping to avoid code
    duplication, not part of the public API.

## Namespace / Directory Choice

- **Recommendation**: `dart/ecs` with `namespace dart::ecs`.
  - Stable and descriptive (not versioned).
  - Allows `dart8` to become a compatibility alias during migration.
- **Alternative**: `dart/simulation/ecs` if we want to keep all worlds under
  `simulation`, but it blurs the clean split with classic APIs.

## Python Binding Direction

- Add `dartpy.ecs` for ECS world bindings.
- Keep classic bindings as-is; no default mixing of classic and ECS types.
- Provide `dartpy.dart8` as a thin compatibility alias during migration.

## Compatibility Notes

- `dart8` should remain a compatibility layer during transition, pointing at
  `dart::ecs` types without new features.
- Classic world APIs remain unchanged to protect downstream users.
