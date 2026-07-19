# Paper Notes: exact reduced Coulomb FBF

This file is a compact source digest so future sessions do not need to re-read
the full paper for orientation. Use the original paper for derivations,
figures, and final cross-checks.

## Bibliographic Record

Title: "A Splitting Architecture for Exact Reduced Coulomb Friction"

Authors: Hongcheng Song, Ye Fan, Uri M. Ascher, Dinesh K. Pai

Venue: ACM SIGGRAPH / Eurographics Symposium on Computer Animation 2026,
Computer Graphics Forum, Volume 45, Number 8.

Project page: <https://www.cs.ubc.ca/research/fbf-friction/>

Paper: <https://www.cs.ubc.ca/research/fbf-friction/paper.pdf>

Video: <https://www.youtube.com/watch?v=5THad4PAGmI>

## What The Video Adds

The video is short, 82 seconds, and acts as a visual abstract. The description
confirms the same method and examples as the paper: exact reduced Coulomb
friction, Tseng-style FBF outer iteration, modular strongly convex cone-QP
inner stage, matrix-free contact-parallel implementation, and rigid-body
benchmarks covering stick-slip transitions, house of cards, and masonry arches.

The video description identifies
<https://github.com/matthcsong/fbf-sca-2026>. Historically, on 2026-07-12, the
official project page still displayed **Code (coming soon)** and that GitHub
URL returned 404 to an unauthenticated request. That availability observation
is superseded: on 2026-07-18 the MIT-licensed repository was public and was
pinned for this task at
`b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0`. It contains the current solver,
six runnable example/configuration sources, pinned dependencies, optional
MuJoCo/Kamino runners, and masonry-arch meshes. It does not contain the
historical renderer cameras/materials/goldens or the original Apple timing-run
attestation.

## Mathematical Core

The method starts from the reduced contact-space velocity

```text
v(lambda) = W lambda + v_free,
W = J M^-1 J^T.
```

Each contact reaction is a 3-vector with normal and two tangential components.
The Coulomb cone is

```text
K_mu = { lambda | lambda_n >= 0, ||lambda_t|| <= mu lambda_n }.
```

The De Saxce-Feng augmented velocity is

```text
v_tilde = v + mu ||v_t|| e_n.
```

The exact law is primal cone feasibility, dual cone feasibility of
`v_tilde`, and complementarity between `v_tilde` and `lambda`.

The split is:

```text
A(lambda) = W lambda + v_free + N_K(lambda)
B(lambda) = mu ||v_t(lambda)|| e_n
```

where `A` is the cone-constrained linear response and `B` is the explicit
non-associated coupling.

## Algorithm

The outer iteration:

1. Form `v_k = W lambda_k + v_free`.
2. Form `g_k = mu ||v_t,k|| e_n`.
3. Solve the cone QP with `W + gamma^-1 I` and frozen `g_k`.
4. Recompute the coupling at the intermediate point.
5. Apply the FBF correction and project back to the cone.
6. Stop when the scaled Coulomb residual is below tolerance.

The inner cone QP is strongly convex even when `W` is singular because of the
proximal `gamma^-1 I` term. The paper's implementation uses matrix-free block
Gauss-Seidel, but the architecture is meant to admit other SOCP solvers.

## Step-Size Control

The paper's safe base step is derived from a Lipschitz bound on the explicit
coupling:

```text
gamma_safe = 0.5 / (mu_max * lambda_max(W)).
```

`lambda_max(W)` is estimated by a short power iteration. The paper publishes
that the adaptive rule accepts a trial step when the local variation ratio is
at most `0.9`; otherwise it shrinks the step by `0.7` and resolves the cone QP.

The paper does **not** publish an author cross-step gamma recovery/growth rule.
DART's persisted-gamma recovery (currently a local `1.05` growth factor capped
by the freshly computed safe bound) is an implementation policy, not a paper
fact. Paper-comparison metadata must retain that difference.

## Residual

The paper avoids a unit-mismatched natural-map residual by scaling the primal
reaction and velocity sides separately. The stopping residual is the max of:

- distance from the primal Coulomb cone,
- distance from the dual Coulomb cone for `v_tilde`,
- scaled complementarity gap.

The fixed per-solve scales are:

```text
s_r = max(||lambda_0||, ||v_f|| / lambda_max(W), epsilon_0)
s_u = max(||v_tilde(lambda_0)||, ||v_f||, epsilon_0)
epsilon_0 = 1e-12
```

These are held constant over the outer loop. The DART implementation should
therefore not treat the residual tolerance as a plain unit-scaled velocity or
impulse distance.

The reported tolerance is `1e-6`. Because the implementation is single
precision, the paper treats this as close to the meaningful floor for large
contact counts.

## Implementation Notes From The Paper

- Backend: NVIDIA Warp plus Newton.
- Collision/contact frontend: Newton provides points, normals, penetration
  depths, deterministic contact frames, friction coefficients, and normal
  Baumgarte stabilization.
- `W` is matrix-free: scatter contact reactions to body wrenches, apply
  inverse mass, gather contact velocities.
- The inner block Gauss-Seidel solver needs per-contact 3x3 diagonal blocks and
  on-the-fly off-diagonal coupling for contacts sharing a body.
- Warm start reuses the previous solution when the contact configuration is
  unchanged and reportedly halves iteration count.

## Important Limitations

- The paper's method is velocity-level rigid-body contact. It does not provide
  deformable position-level non-intersection guarantees.
- The outer FBF correction can lose contractivity on globally coupled contact
  graphs such as arches; the paper diagnoses this as the main remaining
  limitation.
- Performance results are tied to the authors' Warp/Newton implementation and
  single-precision pipeline; DART needs its own baselines before claiming
  matching or better performance.
- The current public source exposes the local 3x3 block kernel and cross-step
  gamma controller. The exact historical Apple-silicon model and paper
  timer/warmup/aggregation attestation remain unavailable, and DART's x86-64
  float64 implementation is structurally different, so local results still
  cannot be apples-to-apples paper comparisons.
