# DART 7 Clean-Break Strategy

Status: accepted release-topology direction.

This document records durable topology and rationale. The mutable DART 7
release gates and active implementation order are owned by
[`docs/onboarding/release-roadmap.md`](../onboarding/release-roadmap.md) and
PLAN-040 in [`docs/plans/dashboard.md`](../plans/dashboard.md#plan-040-dart-7-release-hardening).

## Decision

DART 7 is the clean-break major release for the DART 6 to DART 7 transition.
DART 7 does not serve as a long-lived compatibility bridge for the DART 6 API.
`release-6.*` branches remain the compatibility lane for the established API
and for Gazebo / gz-physics users that need it.

The DART 7 simulation stack is the public API target for the clean break, but
promotion is parity-gated. DART 7 should not ship claims for core robotics
workflows without direct evidence. DART 8 is reserved for a later post-DART-7
major release, not as the active cleanup point for DART 6 compatibility debt.

Main no longer treats gz-physics source compatibility as a required design
constraint for the DART 7 API. The main-branch Gazebo workflow is a migration
canary unless a change is explicitly scoped as Gazebo migration work.

## Rationale

Keeping DART 7 as a bridge and DART 8 as the cleanup release creates a long
dual-maintenance window. Main would need to carry the DART 6 API, the classic
Skeleton-backed simulation path, and the DART 7 world while the
robotics simulation landscape is moving quickly.

Parking gz-physics support on `release-6.*` removes the main reason to keep the
classic DART 6 surface on main. The compatibility surface is broad: classic
`World` ownership, `Skeleton` / `BodyNode` / `Joint` concepts, deprecated
collision and constraint APIs, and package-config behavior used by pinned
gz-physics builds. A broad compatibility shim inside DART 7 would recreate the
same maintenance burden under a different name.

The clean break is still evidence-gated. The DART 7 world must cover
basic robotics simulation before promotion. Research algorithms such as IPC,
VBD, differentiable simulation, and GPU backends can continue as opt-in
features; they do not all block the DART 7 release unless the promoted public
API depends on them.

## Release Topology

- **DART 6.x**: maintained compatibility line for DART 6 API users and
  Gazebo / gz-physics. Backport critical bug fixes, build fixes, security
  fixes, and gz-required compatibility fixes. Do not backport normal DART 7
  features by default.
- **DART 7.0**: clean-break release from main. Promotes the new simulation API
  only after the parity gates in the release roadmap pass. Removes DART 6
  public API shims, legacy dartpy modules, classic `World`, and gz-only
  compatibility surfaces from the DART 7 public contract.
- **DART 8**: future post-DART-7 major release. It is not the active cleanup
  target for the DART 6 to DART 7 transition.

## Compatibility-Shim Policy

Do not build a broad `dart6-compat` shim into main. A future shim is acceptable
only if it is separate from the DART 7 core, inventory-driven by a concrete
downstream migration, limited to the calls that downstream uses, and sunset
dated. The default compatibility answer for gz-physics is the maintained
`release-6.*` line.

## Open Decisions

- Final DART 6 Gazebo support window or sunset trigger for the active
  `release-6.*` branch.
- Whether yanked `dartpy` 7.0.0 artifacts can be reused safely, or whether the
  first published clean-break package should use a later patch/pre-release
  number.
- Exact `release-6.*` parity evidence packet and scene corpus for rigid dynamics.
- Whether any future gz-physics migration adapter is owned by DART or by
  downstream Gazebo work.
