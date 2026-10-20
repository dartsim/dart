# Resume

## Current state (2026-10-19)

- The plan is on `main` (#3848); this home is updated through follow-up PRs.
- The maintainer answered the second decision batch one question at a time
  (DD-10 to DD-34 in [decisions.md](decisions.md)); DD-33 is the standing
  approval for this plan's publishing, and DD-34 extends it to merging
  `main`, resolving conflicts and fixing CI on the plan's ready PRs.
- The published PRs and the merge order are below.
- Two local spike branches hold experiment edits only and will not be
  published: `spike/cxx20-standard` and `spike/cxx20-private`. Their worktrees
  hold the Gazebo lane builds and base variants; remove both once DEP-21's own
  evidence replaces them.
- gz-sim's `INTEGRATION_reset_sensors` is an external test defect (#3866).

## Published (2026-10-19)

- Merged: #3848 (this plan), #3849 (DEP-0), #3852 (DEP-11 PR1), #3860
  (DEP-20(d)), #3861 (DEP-31), #3862 (DEP-20(e)), #3864 (DEP-3(a)).
- Ready for review: #3856 (DEP-28), #3857 (DEP-27a, stacked on #3856),
  #3859 (DEP-32), #3863 (DEP-17 PR2), #3865 (DEP-13 PR1).
- Drafts: #3850 (DEP-12(a)), #3851 (DEP-1), #3853 and #3854 (DEP-14 and
  DEP-15, GitHub stack), #3870 (DEP-35), #3875 (DEP-37), #3876 (DEP-30),
  #3877 (DEP-36).
- Merge order: #3857 after #3856; #3854 after #3853 and #3850; #3865 after
  #3850. DD-33 covers pushes to drafts and DD-34 the merge, conflict and CI
  pushes to ready PRs; marking ready, merging and backports are asked.

## Next steps

1. Watch CI on the pushes of 2026-10-19: #3853 (macOS C++20 fix and the
   perf rationale), #3854 (Windows guard-test fix, merged with #3853),
   #3856 and #3857 (merged with `main`), #3865 (CHANGELOG conflict
   resolved). #3850 and #3851 carry earlier fixes.
2. Finish DEP-33 (a) to (c) and DEP-25; start DEP-34 on #3877 (DEP-36),
   with the `readWorld` null-world check that review found; DEP-33(d)
   needs DEP-19. DEP-36 follow-ups: geom-inferred moments and
   `<compiler inertiafromgeom>`.
3. After DEP-25 (umbrella switch and fail-fast), the tinyxml2 fetch
   prototype; DEP-18, DEP-19 and the urdfdom 6.0.1 wheel bump follow the
   decisions; the backports (DD-27, DD-32) wait for their main PRs.
4. Owed spikes: the Werror C++20 budget on every toolchain, `abidiff` between
   the C++17 and C++20 builds, and the stock-noble Tier 2 build. The distro
   FCL abort is fixed in a DART-carried patch (DD-18, #3870); nothing is
   reported upstream.
5. After M1d: DEP-21, ball-joint limits for MJCF, the libccd attribution in
   the MPR sources, the `GenericJoint` finite checks that GCC 13 stopped
   inlining after DEP-14 (`array().isFinite().all()`; see #3853's perf
   rationale), and any quiet-window hunk the PLAN-624 owner did not clear
   earlier.

## Risks and recovery

- `main` moves quickly. Re-check file and line references in
  [packets.md](packets.md) and the review on current `main` before each
  packet.
- The step-path quiet window is open until M1d merges: `World.cpp` and
  `lcpsolver` hunks wait or go through the PLAN-624 owner.
- The merging PR regenerates `pixi.lock`; never hand-merge it.
- Gazebo lanes: compare against a base variant built from the packet's base
  (`GZ_COMPAT_BASE_VARIANT`); `reset_sensors` fails on every base today.
- Never edit a lane wrapper script while a lane that runs it is still going.

## Verification commands

- `pixi run lint`, then `pixi run check-lint`.
- `pixi run build`, `pixi run test`, `pixi run test-py`, and
  `pixi run test-gui` for GUI changes.
- `pixi run -e gazebo test-gz`, `pixi run -e gazebo gz-compat-ionic`,
  `pixi run -e gazebo-harmonic gz-compat-harmonic`,
  `pixi run -e gazebo-jetty gz-compat-jetty`.
- `pixi run test-package-core`; `pixi run test-build-requirements` when floors
  change.
