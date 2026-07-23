# Deformable-body paper targets

This document preserves the paper-derived reference models and measurements
used to evaluate DART 6 deformable-body work. It is a reference foundation,
not a claim that DART implements every scene or reaches parity. Current
roadmap state belongs in `docs/plans/dashboard.md`; DART-specific compatibility
decisions belong in `docs/design/dart6_deformable_body.md`.

## Sources

- Junggon Kim and Nancy S. Pollard, "Fast Simulation of Skeleton-Driven
  Deformable Body Characters," ACM Transactions on Graphics 30(5), 2011:
  [project page](https://www.cs.cmu.edu/~junggon/projects/fastsimuldbody/fastsimuldbody.htm)
  and [author preprint](https://www.cs.cmu.edu/~junggon/projects/fastsimuldbody/fastsimuldbody_preprint.pdf).
- Sumit Jain and C. Karen Liu, "Controlling Physics-Based Characters Using
  Soft Contacts," ACM Transactions on Graphics 30(6), 2011:
  [author PDF](https://www.cc.gatech.edu/graphics/projects/Sumit/homepage/papers/sigasia11/jain_softcontacts_siga11.pdf).

## Kim and Pollard

Kim and Pollard couple a skeleton to a reduced nonlinear volumetric FEM body,
embed a fine surface in the coarse volume, use linear-time skeleton dynamics,
and integrate explicitly. Their one-way mode follows prescribed skeleton
motion; their two-way mode lets the skeleton, deformable body, and environment
affect one another. The published measurements used a 2.8 GHz Core 2 Quad CPU
and a GeForce GTX 280 GPU.

| Model | Coupling and motion | Step | Total / skeleton DOFs | Volumetric nodes / elements | Surface nodes | Compute for 1 s simulated |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| Fatman | One-way dancing/jiggle | 0.5 ms | 4,887 / 60 | 2,121 / 8,619 | 34,362 | 1.33 s GPU; 13.24 s CPU |
| Starfish | Two-way jump-turn | 1.0 ms | 260 / 26 | 78 / 200 | 1,162 | 0.49 s CPU |
| Fish | Two-way actuated jump and landing | 1.0 ms | 258 / 9 | 107 / 415 | 958 | 0.50 s CPU |
| Worm | Two-way actuated roll | 1.0 ms | 543 / 9 | 224 / 714 | 262 | 0.57 s CPU |

The obstacle-escape starfish took 0.94 s per simulated second; more than 40%
of that run was spent in the paper's simple point-triangle collision check.
The published timings exclude high-level controller-command computation and
offline rendering. They are therefore model and hardware reference points,
not portable wall-clock thresholds for DART.

## Jain and Liu

Jain and Liu attach a triangle surface of point masses to articulated rigid
bodies. Each active point mass contributes three translational coordinates;
vertex and edge springs model deformation, and an LCP couples normal contact
and Coulomb friction. Contact seeds a local p-ring; marked vertices keep being
simulated until they leave contact and return to equilibrium. Inactive points
at rest contribute their mass and inertia to the rigid parent.

| Example | Total DOFs | Simulated DOFs | Contacts | Frames/s | LCP time | Step | Stiffness |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Finger flick | 2,573 | 576 +/- 88 | 39 +/- 6 | 3.9 +/- 3.2 | 86 +/- 6% | 1.7 ms | 1.5e4 |
| Arm fold | 2,802 | 322 +/- 89 | 33 +/- 10 | 3.5 +/- 1.7 | 68 +/- 10% | 8.3 ms | 1e4 |
| Pinch grasp | 1,427 | 258 +/- 22 | 29 +/- 4 | 5.2 +/- 3.2 | 85 +/- 6% | 1.7 ms | 1.5e4 |
| Biped walk | 334 | 197 +/- 43 | 16 +/- 3 | 18.5 +/- 4.5 | 63 +/- 5% | 4.0 ms | 1e3 |

For biped walking, the paper solves the LCP every eight 0.5 ms SIMBICON
controller steps. Its evaluation also compares push recovery, a seeded noisy
floor, contact-force and center-of-pressure smoothness, hand manipulation, and
LCP initial points.

The flexible-foot comparison is a distinct paper target: the rigid comparator
uses four links connected by hinge joints along the foot center line. It is
more stable than a simpler rigid foot, while the deformable foot remains more
stable because its contacts change more continuously. A DART comparison must
therefore include the four-link rigid model; comparing only a simple rigid
foot with an existing soft-foot asset does not reproduce this target.
