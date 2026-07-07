# Paper parity matrix

This file converts the requested papers into tracked DART 6 acceptance targets.

Sources:

- Kim and Pollard project page:
  <https://www.cs.cmu.edu/~junggon/projects/fastsimuldbody/fastsimuldbody.htm>
- Jain and Liu author PDF:
  <https://sites.cc.gatech.edu/graphics/projects/Sumit/homepage/papers/sigasia11/jain_softcontacts_siga11.pdf>
- Jain and Liu ACM DOI:
  <https://dl.acm.org/doi/10.1145/2024156.2024197>

## Kim and Pollard 2011

The paper target is a skeleton-driven deformable character system with reduced
nonlinear FEM, embedded surface meshes, linear-time skeleton dynamics, explicit
or symplectic Euler integration, and one-way or two-way coupling. The project
page states that the representative characters were real-time or near
real-time, with optional GPU acceleration for complicated characters. GPU is
recorded here as reference evidence only; DART 6 implementation work remains
CPU-first.

| Representative target | Paper evidence to replicate | DART 6 status | Owner packet |
| --- | --- | --- | --- |
| Passive character jiggle | Kinematic skeleton drives deformable body response; Fatman-scale character is the flagship one-way case. | No tracked DART 6 equivalent. Existing `soft_bodies` is much smaller and not skeleton-driven at paper scale. | WP-DB.03, WP-DB.08 |
| Self-propelled starfish | Two-way coupling among skeleton, deformable body, and environment; real-time CPU target on a small character. | No volumetric FEM backend or equivalent scene. | WP-DB.04, WP-DB.08 |
| Fish jump | Internally actuated skeleton with deformable body and contact. | No equivalent scene or controller. | WP-DB.08 |
| Worm roll | Internally actuated chain with deformable body and environment interaction. | `soft_open_chain` is only a small legacy spring-mass chain; it lacks paper metrics. | WP-DB.08 |
| Obstacle escape | Contact-heavy character scene where collision checking is a material cost. | No soft-body contact benchmark with paper-scale contact metrics. | WP-DB.05, WP-DB.07 |
| CPU/GPU scaling | Optional GPU speedup increases with mesh size; CPU near-real-time remains the DART 6 target. | DART 6 has no tracked GPU deformable surface. Need CPU single-core, multi-core, and SIMD rows. | WP-DB.06, WP-DB.07 |

## Jain and Liu 2011

The paper target is closer to DART 6 `SoftBodyNode`: surface point masses
attached to rigid bodies, vertex and edge springs, adaptive active vertices near
contact, and LCP/friction soft contact.

| Representative target | Paper evidence to replicate | DART 6 status | Owner packet |
| --- | --- | --- | --- |
| Biped push recovery | Same SIMBICON-style controller with rigid contact vs soft contact; soft contact withstands larger pushes. | No tracked soft-foot locomotion regression. | WP-DB.03, WP-DB.08 |
| Noisy-floor biped | 5 cm floor tiles with random 0-2 cm offsets; compare balance and contact stability. | No tracked noisy-floor soft contact scene. | WP-DB.08 |
| Center of pressure and force variance | Soft contact gives smoother CoP and lower contact-force variance over walking frames. | No CoP/force instrumentation for soft contacts. | WP-DB.02, WP-DB.05 |
| Finger flick | 2573 total DOFs, 576 +/- 88 simulated DOFs, 39 +/- 6 contacts, 3.9 +/- 3.2 fps, 86 +/- 6% LCP time, 1.7 ms step. | No hand/finger soft-contact scene. | WP-DB.03, WP-DB.08 |
| Arm fold | 2802 total DOFs, 322 +/- 89 simulated DOFs, 33 +/- 10 contacts, 3.5 +/- 1.7 fps, 68 +/- 10% LCP time, 8.3 ms step. | No upper-body self-contact scene. | WP-DB.08 |
| Pinch-grasp | 1427 total DOFs, 258 +/- 22 simulated DOFs, 29 +/- 4 contacts, 5.2 +/- 3.2 fps, 85 +/- 6% LCP time, 1.7 ms step. | No soft-hand manipulation scene. | WP-DB.08 |
| Biped walk | 334 total DOFs, 197 +/- 43 simulated DOFs, 16 +/- 3 contacts, 18.5 +/- 4.5 fps, 63 +/- 5% LCP time, 4.0 ms step. | No soft-contact SIMBICON scene. | WP-DB.08 |
| LCP initial-point robustness | Contact-force magnitudes remain close across zero, one, and random initial guesses. | DART 6 LCP behavior is not tested for this soft-contact invariant. | WP-DB.02, WP-DB.05 |
| Flexible rigid foot comparison | Compare a deformable foot to a four-link rigid foot approximation. | No paired comparison scene. | WP-DB.08 |

## Acceptance rule

Paper parity is not satisfied by matching API names. Each row needs:

- a runnable DART 6 scene or benchmark,
- correctness/stability evidence,
- performance rows for single-core and multi-core CPU,
- an explicit SIMD-on/off result when the row touches vectorizable kernels,
- comparison against the paper number or an approved modern normalized target.
