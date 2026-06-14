# DART 7 Handle Opaque-Storage Design (PLAN-041 WS5)

## Status

Accepted and implemented for the current promoted-header surface. This is the
implementation design for de-ECS-ing the DART 7 simulation handle classes
so the promoted DART 7 public API does not expose ECS storage. It is the WS5
detail behind the plan in
[`dart7_promotion_readiness_audit.md`](dart7_promotion_readiness_audit.md) and is
gated by the promotion-surface audit so future headers cannot reintroduce the
same storage leak.

Operating state: `PLAN-041` in [`../plans/dashboard.md`](../plans/dashboard.md).

## Problem

The handle classes (`Frame`, `FreeFrame`, `FixedFrame`, `RigidBody`,
`Multibody`, `Link`, `Joint`, `LoopClosure`, ...) leak ECS storage through their
**public headers** in three ways:

1. **`entt::entity` in signatures/members** — constructors and `getEntity()`
   expose the raw ECS entity type; `world.hpp` additionally holds an
   `entt::registry` member.
2. **`comps::` types in the class declaration** — every handle publicly inherits
   `EntityObjectWith<TagComps<comps::...>, ReadOnlyComps<...>, WriteOnlyComps<...>,
ReadWriteComps<comps::...>>`, naming component types in the public base.
3. **`comps`/`ecs` includes** pulled in to make the above compile.

The `audit-dart7-promotion-surface` check reports the current leak set (run it
for the live count). The current promoted-header surface is storage-neutral:
`Entity` is an opaque token with no public raw integer field, handle comments no
longer point users at `detail::toRegistryEntity(...)`, and `DeformableBody` uses
the same opaque token as the rest of the handle family.

## Constraints

- **Preserve ECS storage.** This is an API-surface change, not a storage change.
  Default-build behavior and the DART 7 simulation tests must stay
  green at every step.
- **Preserve the compile-time component-access validation** that
  `EntityObjectWith` provides today — move it internal, do not delete it.
- **Do not churn the ~96 internal `getEntity()` call sites** in one breaking
  step; migrate them behind a seam.

## Options

### A. Per-handle pimpl

Each public handle stores an opaque `Impl*` and forwards every accessor. Fully
hides ECS, but is the most invasive (an impl type + forwarding for every handle)
and adds an allocation/indirection per handle unless pooled.

### B. Internal base + internal validated-access helper (recommended)

- Public handles inherit only a **minimal non-template public base** that stores
  the opaque identity (`Entity` + `World*`) and exposes public identity/validity
  (`isValid()`, `operator==`, an `Entity entity()` accessor). No `comps`, no
  `entt`, no template in the public header.
- The `EntityObjectWith<...>` compile-time-validation machinery becomes an
  **internal** helper (under `ecs/` / `detail/`) used only by the handle `.cpp`
  files for validated component access. The public class no longer inherits it.
- `getEntity() -> entt::entity` becomes an **internal** accessor (e.g.,
  `detail::registryEntity(const HandleBase&)` via friendship, or a protected
  method visible only to internal TUs), so the public header stops naming `entt`.
  The ~96 internal callers switch to the internal accessor (mechanical).

This keeps the storage and the validation, removes all three leak classes from
the public headers, and avoids per-handle pimpl boilerplate.

### C. Forward-declared component tags

Forward-declare `comps::` types and move the template instantiation to `.cpp`
via explicit instantiation. Fragile: template bases need complete types at the
point of use, so this tends to collapse back into option A or B.

## Recommendation

Adopt **Option B**. It is the smallest change that fully de-ECS-es the public
handle surface while preserving storage and validation.

## Migration approach

The design was applied in dependency order so each change stayed small while ECS
storage and validated internal access were preserved throughout: introduce the
opaque `Entity` token and conversion seam; migrate public handles off raw
`entt::entity` / `entityId` storage; replace public
`EntityObjectWith<...comps...>` inheritance with storage-neutral handle classes;
move the `world.hpp` registry behind opaque storage; then enforce the result
with private EnTT/Taskflow linkage, an explicit install allowlist, and the
strict promotion-surface audit.

The per-step sequencing, status, and gates are tracked in PLAN-041
([`../plans/dashboard.md`](../plans/dashboard.md) and
[`../plans/041-official-simulation-api-promotion.md`](../plans/041-official-simulation-api-promotion.md)),
not in this design doc.

## Verification

Each implementation step keeps the DART 7 simulation gates green:
`pixi run build-simulation-tests`, `pixi run test-simulation-quick`,
`pixi run check-api-boundaries`,
`pixi run check-dart7-promotion-surface`, and `pixi run lint`.

## Risk and review notes

- Keep future promoted handles on the opaque-token pattern. A new public raw
  entity ID, `detail::toRegistryEntity(...)` comment, EnTT type, component type,
  or ECS helper base should fail the strict promotion-surface audit.
- The remaining broad promotion risk is no longer handle storage leakage; it is
  the final namespace/package transaction that replaces the classic
  `dart::simulation::World` owner and retires the staging target names.
