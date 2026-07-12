# Decisions - DART 6 deformable body feature and performance

Records consequential decisions for this task: maintainer-confirmed choices,
and orchestrator assumptions taken to keep autonomous work unblocked. Each
assumption names the safest-reversible default chosen and what would change it.
Items marked "needs maintainer sign-off" must be confirmed before or at PR
review; they do not block implementation because the chosen default is
reversible.

## Confirmed by task brief / release-branch policy

- Target branch is `release-6.20`; DART 6 public headers, API, and ABI are
  preserved. Breaking changes require explicit maintainer acceptance
  (`docs/ai/principles.md`).
- GPU is out of scope for DART 6; CPU single-core, multi-core, and SIMD lanes
  are the performance envelope (task `README.md`).
- GitHub mutations (push, PR creation, review threads) require explicit
  maintainer approval.

## Orchestrator assumptions (2026-07-09)

1. **`SoftBodyNode` remains the public API; internals are replaced.**
   The alternative (a parallel opt-in deformable solver) adds public surface to
   a compatibility branch without evidence of need. All WP-DB.06 slices so far
   already follow the preserve-API path. Reversal trigger: a maintainer asks
   for a separate solver type.
2. **Competitive-implementation envelope for measured comparison** is the
   in-tree detector backends (FCL, Bullet, ODE vs `dart`/`native`) on identical
   scenes plus the published paper metrics normalized in
   `02-paper-parity-matrix.md`. Benchmarking external engines (e.g. MuJoCo)
   from this branch is out of scope for the completing PR. Needs maintainer
   sign-off as the formal definition of "competing implementations".
3. **Paper demos mandatory for the first release-branch PR** are a
   representative subset: one Kim/Pollard-lane skeleton-driven deformable demo
   and one Jain/Liu-lane soft-contact demo with adaptive activation, both
   headless-runnable with GUI capture evidence. The remaining matrix rows
   (Fatman-scale FEM character, starfish/fish volumetric FEM, SIMBICON
   locomotion suite, hand manipulation scenes) are recorded as evidence-backed
   deferrals: they require controller/FEM infrastructure that DART 6's
   point-mass `SoftBodyNode` model does not provide, and adding a volumetric
   FEM backend is not a compatibility-preserving release-branch change. Needs
   maintainer sign-off on the deferral list. The named list was approved on
   2026-07-11; rows outside it remain open rather than implicitly deferred.
4. **Adaptive contact activation (WP-DB.05) ships opt-in.** Default behavior
   stays all-active so existing soft bodies and downstream consumers are
   unchanged; activation is enabled per soft body or per world through
   non-breaking API. Deterministic hashes gate equivalence.
5. **Native soft collision direction**: keep the vertex-face lane plus
   adaptive/contact-neighborhood coverage as the measured path; full
   triangle-triangle mesh collision remains a follow-up unless parity evidence
   shows it is required for the representative scenes. This follows the
   Jain/Liu model that owns the DART 6 soft-body semantics. Evidence, not
   preference, decides any change.
6. **Branch history**: `wp-db-native-soft-fallback` was pushed to origin on
   2026-07-09 (before this session). History is therefore not rewritten or
   squashed without explicit maintainer approval.

7. **Stranded-branch reunification (2026-07-09).** Investigation showed the
   task's evidence base was split across two pushed branches: PR #3307 merged
   an earlier snapshot into `release-6.20`, while
   `wp-db-soft-skel-allocation-gates` kept ~20 newer commits (WP-DB.04
   equation-correctness gates plus `07-equation-correctness.md`, the
   `PointMassPhaseView` series, FCL soft-mesh read slices, World empty-solve
   fast paths, extra SKEL allocation gates) that were in neither
   `release-6.20` nor `wp-db-native-soft-fallback`, even though this task's
   README/RESUME described them as current state. Decision: merge
   `wp-db-soft-skel-allocation-gates` into `wp-db-native-soft-fallback`
   (both already pushed, so a merge preserves history without rewrites).
   Conflict policy: native-collision files and infra took the current
   branch's newer re-derived versions; dynamics/tests/docs payload came from
   the stranded branch. Validation: focused native/soft/allocation CTest
   gates passed, and 200-step checksums across
   {drop_box, soft_cubes, soft_bodies, soft_open_chain, adaptive_deformable}
   x {fcl, dart, native} x {1, 4 threads} were bit-identical to pre-merge
   HEAD except three point-mass accumulators on the dart/native contact
   scenes drifting by 1-3 ULPs (~1e-16 relative) from composed FP
   reordering; thread-count determinism and dart==native equivalence stayed
   bit-exact, so the drift is accepted within task tolerance.

8. **Public mass matrices exclude retained point-mass acceleration
   (2026-07-12).** PR review correctly identified that the first WP-DB.04
   implementation added `PointMass::State::mAccelerations` to every
   generalized-coordinate mass-matrix column. Point masses are not exposed as
   DART 6 `Skeleton` DOFs, so retained simulation acceleration is not a basis
   acceleration for this public matrix. Decision: use only the parent body
   response for mass and augmented-mass column assembly, keep the physical
   acceleration term in inverse dynamics, and gate the distinction with a
   nonzero-retained-acceleration regression. This exact correction does not
   backport to DART 7 because `main` still has point-mass mass aggregation
   disabled.
9. **Dual-PR applicability for the zero-DoF assertion fix (2026-07-12).** The
   soft point-mass `Skeleton::updateBiasImpulse` overload on live
   `origin/main` still carries the same over-strict `getNumDofs() > 0`
   assertion removed by release commit `10c6b6055e4`. That bug therefore
   requires a `main` follow-up under the dual-PR policy. Keep it separate from
   #3382 stabilization and do not use the unrelated MJCF baseline failure as a
   reason to widen this release PR.

## Deferral list (maintainer-approved 2026-07-11)

Tracked in `02-paper-parity-matrix.md` rows whose acceptance requires
infrastructure beyond the DART 6 point-mass soft-body model:

- Kim/Pollard reduced nonlinear FEM characters (Fatman jiggle, starfish and its
  obstacle-escape row, fish, worm at paper scale) — requires a volumetric FEM
  backend.
- Jain/Liu SIMBICON-driven locomotion rows (biped push recovery, noisy floor,
  biped walk) and hand scenes (finger flick, arm fold, pinch grasp) at full
  paper scale — require controller infrastructure; representative reduced
  scenes stand in for the contact/performance claims.

Each deferred row keeps its matrix entry with current evidence and the reason
it is deferred, so the release branch records the gap honestly. The
maintainer approved this list as recorded on 2026-07-11; representative
reduced scenes (soft_worm, adaptive_soft_contact) stand in for the deferred
rows' contact and performance claims.

This approval does not explicitly cover the Jain/Liu flexible-rigid-foot versus
deformable-foot comparison. Keep that row open unless a maintainer expands the
deferral list or a representative comparison lands. The approved decisions are
now also preserved in `docs/design/dart6_deformable_body.md`, while PLAN-622
owns the remaining open decision.
