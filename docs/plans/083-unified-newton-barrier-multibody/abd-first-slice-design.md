# PLAN-083 ABD First-Slice Design

This sidecar defines the first bounded implementation slice for affine body
dynamics (ABD) inside DART's unified Newton-barrier family. It depends on the
primitive promotion slice for shared distance, barrier, and tangent math, but it
does not require replacing rigid IPC or changing public APIs.

## Source Commitments

- The unified Newton-barrier paper treats rigid and soft bodies through
  Lagrangian nodal displacements, handles linear equality joints through
  change-of-variable, handles nonlinear constraints with stiff potentials, and
  handles inequality constraints, friction, and restitution through barrier
  potentials.
- The supplied ABD deck frames affine body dynamics as a compact stiff-body
  representation for robust IPC contact, with reported CPU gains over rigid IPC
  and a GPU acceleration target for collision-heavy scenes.
- PLAN-083 keeps rigid IPC as the correctness oracle until ABD proves matched
  behavior and better performance on shared fixtures.

## First-Slice Outcome

Add an internal affine-body primitive prototype that can map a body-local
surface into world coordinates, evaluate shared Newton-barrier primitive
contacts and tangent-displacement friction through affine chain rules, and
prove rigid-equivalence on small fixtures when the affine transform is
constrained to rotation plus translation.

The first slice is an internal correctness foundation. It does not expose ABD
as a user-selectable solver and does not claim paper-scale performance.

## Proposed Internal Shape

Suggested owner path:

```text
dart/simulation/experimental/detail/affine_body_dynamics.hpp
dart/simulation/experimental/detail/affine_body_dynamics.cpp
```

Suggested test path:

```text
tests/unit/simulation/experimental/contact/test_affine_body_dynamics.cpp
```

Suggested benchmark path after correctness lands:

```text
tests/benchmark/simulation/experimental/bm_affine_body_dynamics.cpp
```

The owner can move later if ABD grows into a full runtime method, but the first
slice should stay under `detail/` and out of public headers.

## Internal Types

| Type                            | Responsibility                                                                     | First-slice fields                                                                                                 |
| ------------------------------- | ---------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------ |
| `AffineBodyState`               | Compact 12-DOF body state.                                                         | Translation `a`, affine matrix `A`, linear velocity, affine velocity, mass/density proxy, stiffness parameters.    |
| `AffineSurfaceAdapter`          | Maps body-local vertices, edges, and triangles to world coordinates and Jacobians. | Rest vertices, triangle indices, `x = a + A X`, per-vertex 3x12 Jacobian, active/dynamic flag.                     |
| `AffineBarrierOptions`          | Parameters shared with Newton-barrier contact.                                     | Activation distance, barrier stiffness, Hessian PSD policy.                                                        |
| `AffineFrictionOptions`         | Parameters shared with tangent-displacement friction.                              | Coefficient, lagged normal force, static-friction displacement, Hessian PSD policy.                                |
| `AffinePrimitiveBarrierResult`  | Shared primitive result mapped into affine coordinates.                            | Value, 12-DOF gradient per body, 12x12/24x24 Hessian blocks, primitive provenance, active flag.                    |
| `AffinePrimitiveFrictionResult` | Shared friction result mapped into affine coordinates.                             | Value, tangent displacement, 12-DOF gradient per body, 24x24 Hessian, primitive provenance, active flag.           |
| `AffineOrthogonalityEnergy`     | Stiff rigid-shape prior for affine bodies.                                         | Energy/gradient/Hessian for penalizing non-rigid affine deformation; stiffness and diagnostics.                    |
| `AffineComparisonPacket`        | Correctness/performance evidence row.                                              | Scene name, body/triangle count, timestep, accuracy metric, CPU timing, optional GPU timing, rigid IPC comparison. |

## First-Slice Scope

| Step | Change                                                                  | Required invariant                                                                                                                                                                                                          |
| ---- | ----------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1    | Add `AffineBodyState` and `AffineSurfaceAdapter`.                       | World vertex positions and 3x12 Jacobians match finite differences for points, edges, and triangles.                                                                                                                        |
| 2    | Map shared Newton-barrier primitive outputs into affine coordinates.    | Affine gradient/Hessian match finite differences for point-triangle, edge-edge, point-edge, and point-point rows.                                                                                                           |
| 3    | Add orthogonality/stiffness energy.                                     | Energy is zero or near-zero for rigid transforms, positive for shear/scale, and has finite derivative checks.                                                                                                               |
| 4    | Add rigid-equivalence tests.                                            | When `A` is a rotation matrix and affine perturbations are restricted to rigid tangent directions, mapped barrier and friction derivatives agree with rigid IPC reduced-coordinate derivatives within documented tolerance. |
| 5    | Add a two-body contact micro-solve only if derivative tests are stable. | One affine dynamic body moves away from a static surface under a barrier objective without violating the activation-distance line-search contract.                                                                          |
| 6    | Add benchmark smoke after correctness.                                  | `bm_affine_body_dynamics` or a temporary benchmark row records primitive mapping cost versus rigid reduced mapping on matched tiny scenes.                                                                                  |

## Correctness Oracles

Use layered oracles rather than one broad scene claim:

| Oracle                           | Evidence                                                                                                                        |
| -------------------------------- | ------------------------------------------------------------------------------------------------------------------------------- |
| Finite differences               | Per-primitive affine gradient/Hessian checks for regular and near-degenerate cases.                                             |
| Rigid IPC equivalence            | Compare affine mapping constrained to rigid transforms against `rigidIpc*ReducedBarrier` and reduced friction rows.             |
| Deformable primitive equivalence | Shared Newton-barrier world primitive value/gradient/Hessian matches the affine mapped primitive before chain-rule application. |
| Energy behavior                  | Orthogonality energy stays inactive for rigid transforms and increases under shear/scale.                                       |
| Determinism                      | Repeated identical micro-solves produce identical state and diagnostics.                                                        |

## Performance Packet Shape

Every ABD benchmark row should be recorded in the same shape before any
paper/deck comparison:

| Field      | Requirement                                                                                                                     |
| ---------- | ------------------------------------------------------------------------------------------------------------------------------- |
| Scene      | Stable DART scene name and source row, for example `abd-vs-rigid-wreck` or `abd-chain-8`.                                       |
| Geometry   | Body count, triangle count, body-local vertex count, and contact candidate count.                                               |
| Step       | Timestep, friction coefficient, activation distance, stiffness, solver tolerances, iteration counts.                            |
| Accuracy   | Minimum distance, interpenetration count, energy diagnostic, and rigid-equivalence residual where applicable.                   |
| CPU timing | End-to-end step time plus contact candidate, barrier/friction, PSD, assembly, and linear-solve breakdown.                       |
| GPU timing | Same breakdown when a private GPU backend exists; otherwise record that the cited source has no DART GPU packet yet.            |
| Comparison | Current DART rigid contact, rigid IPC, Bullet where the source uses Bullet, and paper/deck numbers on matched scene parameters. |

## CPU/GPU Sequencing

1. CPU derivative and rigid-equivalence tests land first.
2. CPU micro-benchmark smoke lands before any runtime stage.
3. GPU work is private and benchmark-gated through PLAN-030-style rules. The
   likely first GPU candidates are primitive barrier batches, PSD projection,
   and assembly; affine state/public APIs must not expose device, stream, or
   memory-pool concepts.
4. A GPU speedup claim is valid only when CPU/GPU results match on the same
   DART scene and the packet records accuracy and timing fields above.

## py-Demos Sequencing

The first ABD slice has no py-demo requirement. Add py-demos only after there
is a runtime stepping path. The first eligible scenes are:

- `abd-chain-8` as a `Rigid IPC (sx)` or future unified stiff-body category row.
- `abd-bullet-small` as a rigid-contact comparison row.
- `abd-fem-coupling` only after mixed affine/deformable contact exists.

Each promoted demo still needs headless capture, nonblank image checks,
motion-difference evidence, and benchmark/profiling packets.

## Non-Goals

- No public ABD solver selector.
- No replacement of rigid IPC or current rigid contact.
- No mixed deformable/affine coupling in the first slice.
- No paper-scale chain nets, gears, Bullet comparisons, or GPU speedup claims.
- No py-demos scene before a runtime stage exists.
- No upstream code dependency.

## Done Means

- Internal affine state and surface adapters exist.
- Shared Newton-barrier primitive results can be mapped through affine
  Jacobians with finite-difference coverage.
- Rigid-equivalence tests compare against PLAN-082 reduced-coordinate barriers.
- The affine primitive-family friction rows share the Newton-barrier
  tangent-displacement kernel with rigid IPC and match the rigid reduced
  value/gradient/Hessian oracles under rigid tangent projection.
- Orthogonality energy has derivative and rigid-invariance tests.
- The first benchmark smoke shape is defined, even if paper-scale rows remain
  `planned`.
