# RESUME - DART 6 deformable body feature and performance

Updated: 2026-07-30

## Start here

Read, in order:

1. `docs/ai/principles.md`
2. `README.md`
3. `docs/design/dart6_deformable_body.md`
4. `docs/background/deformable_body_paper_targets.md`
5. `12-pr3a-soft-foot-simbicon.md`
6. `verification.md`

Use `docs/plans/dashboard.md` for current priority. Treat older commits and
merged PR branches as evidence only, not as resumable work.

## Verified merged state

- #3382 merged into `release-6.20` on 2026-07-29 as
  `6c88ac1d774a702b494643fb598be6b8af9385e1`.
- #3381 merged the DART-owned collision implementation into the built-in
  `dart` detector as `46719bfbd75e1f70e69b2c76fb34a3fa2b78edd5`.
- #3407 removed the volumetric-FEM subsystem from DART 6 as
  `2ffe228c14c67e120d2a946ce9d36e8a9658044f`.
- The DART 6 task therefore covers the Jain/Liu point-mass surface model only.

Re-fetch before starting: the release branch advances frequently, and the
commit above is a snapshot rather than a permanent branch tip.

## Immediate packet

Build PR-3a soft-foot SIMBICON as specified in
`12-pr3a-soft-foot-simbicon.md`.

1. Fetch `origin/release-6.20`.
2. Create a fresh non-tracking topic branch from the authorized target.
3. Confirm the existing `atlas_simbicon` controller and soft-feet Atlas asset
   still match the packet assumptions.
4. Implement the smallest complete scene and verification slice.
5. Integrate GUI access through `dart-demos`.
6. Capture text-first simulation evidence, then visual evidence tied to the
   scene claims.
7. Run the focused and downstream gates before publishing.

## Required evidence

- Finite state and deterministic final-state metrics.
- Rigid-foot versus deformable-foot comparison using the same controller and
  initial conditions.
- Contact, support, and controller-stability observations tied to explicit
  pass/fail thresholds.
- One-thread and host-capped multi-thread measurements when performance is
  claimed.
- A reproducible `dart-demos` command plus captured visual evidence.
- Gazebo/gz-physics coverage for collision, constraint, or default-policy
  changes.

Do not count interrupted benchmark directories as evidence. A paired-runner
artifact is complete only when its completion marker and all required raw rows
exist.

## Verification commands

Run the exact focused tests named by the packet, then at minimum:

```bash
pixi run lint
pixi run check-lint
pixi run build
pixi run test
pixi run -e gazebo test-gz
```

Use `dart-verify-sim` for scene behavior and visual claims. Cap local build
parallelism when the host is shared.

## Stop conditions

- Do not restart Kim/Pollard volumetric FEM on DART 6.
- Do not change public layouts, defaults, or installed compatibility surfaces
  without explicit approval and the corresponding acceptance evidence.
- Do not make `dart` the default detector from this packet.
- Keep the `main` zero-DoF soft point-mass assertion fix in a separate PR.
- GitHub mutations, pushes, review requests, and branch deletion require the
  authorization defined by the repository instructions.

## Remaining PLAN-622 work

After PR-3a, reassess the dashboard rather than assuming the next packet.
Known open items are the competitive envelope, flexible-foot comparison,
WP-DB.07 scaling, pre-default collision coverage, a complete paired benchmark
or approved disposition, and the separate `main` bug fix.
