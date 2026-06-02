# DART 7 Handle Opaque-Storage Design (PLAN-041 WS5)

## Status

Proposal. This is the implementation design for de-ECS-ing the experimental
simulation handle classes so the promoted DART 7 public API does not expose ECS
storage. It is the WS5 detail behind the plan in
[`dart7_promotion_readiness_audit.md`](dart7_promotion_readiness_audit.md) and is
gated for maintainer review because it changes the shared handle base.

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
for the live count). `entity.hpp` (the opaque `Entity`
token) and the `detail/entity_conversion.hpp` seam already exist; the
`FreeFrame`/`FixedFrame` constructors already use `Entity`. The remaining leak is
dominated by the `EntityObjectWith<...comps...>` public base.

## Constraints

- **Preserve ECS storage.** This is an API-surface change, not a storage change.
  Default-build behavior and the 49 `simulation-experimental` tests must stay
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

The design is applied in dependency order so each change stays small while ECS
storage and the compile-time validation are preserved throughout: introduce the
opaque `Entity` token and conversion seam; migrate `getEntity()` to an internal
accessor so the internal callers move behind the seam without churn; replace the
public `EntityObjectWith<...comps...>` inheritance with the minimal identity base
(starting at the shared `Frame`, then the `body`/`multibody`/`constraint`
handles); then move the `world.hpp` `entt::registry` member behind an opaque
impl. The package-shape and boundary-enforcement follow-ups (private
EnTT/Taskflow, an explicit install allowlist, and a `--strict` audit gate) build
on a zero-leak promoted surface.

The per-step sequencing, status, and gates are tracked in PLAN-041
([`../plans/dashboard.md`](../plans/dashboard.md) and
[`../plans/041-official-simulation-api-promotion.md`](../plans/041-official-simulation-api-promotion.md)),
not in this design doc.

## Verification

Each implementation step keeps the experimental gates green:
`pixi run build-simulation-experimental-tests` + `ctest -L simulation-experimental`,
`pixi run check-api-boundaries`, `pixi run check-api-boundary-inventory`,
`pixi run audit-dart7-promotion-surface`, and `pixi run lint`.

## Risk and review notes

- Removing the public `EntityObjectWith` base changes the handle design idiom;
  the compile-time validation must be retained internally (step 2) so handle
  `.cpp` code keeps its safety. This is the item needing maintainer sign-off.
- The `getEntity()` migration (step 3) is broad but mechanical and fully behind
  the seam; do it as one reviewable commit with no behavior change.
