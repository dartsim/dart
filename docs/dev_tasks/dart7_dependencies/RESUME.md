# Resume

## Current state (2026-10-20)

- The plan is on `main` (#3848); this home is updated through follow-up PRs.
- The maintainer answered the second decision batch one question at a time
  (DD-10 to DD-33 in [decisions.md](decisions.md)); DD-33 is the standing
  approval for this plan's publishing.
- The published PRs and the merge order are below.
- Two local spike branches hold experiment edits only and will not be
  published: `spike/cxx20-standard` and `spike/cxx20-private`. Their worktrees
  hold the Gazebo lane builds and base variants; remove both once DEP-21's own
  evidence replaces them.
- gz-sim's `INTEGRATION_reset_sensors` is an external test defect (#3866).

## Published (2026-10-20)

- Merged: #3848 (this plan), #3849 (DEP-0), #3852 (DEP-11 PR1), #3860
  (DEP-20(d)), #3862 (DEP-20(e)), #3864 (DEP-3(a)).
- Ready for review: #3856 (DEP-28), #3857 (DEP-27a, stacked on #3856),
  #3859 (DEP-32), #3861 (DEP-31), #3863 (DEP-17 PR2), #3865 (DEP-13 PR1).
- Drafts: #3850 (DEP-12(a)), #3851 (DEP-1), #3853 and #3854 (DEP-14 and
  DEP-15, GitHub stack), #3870 (DEP-35).
- Merge order: #3857 after #3856; #3854 after #3853 and #3850; #3865 after
  #3850. DD-33 covers pushes to drafts; pushes to ready PRs are asked.

## Next steps

1. Fix the CI failures on #3850, #3851, #3853 and #3854, merging the latest
   `main` into each (DD-33), and resolve the conflicts that ready PRs #3857
   and #3863 now have with `main` (ask before pushing).
2. Finish DEP-36 (MJCF limits and inertia) and DEP-37 (Bullet leak), then
   DEP-34 (MuJoCo fixtures, Fetch removal) and DEP-33 (provenance actions and
   the G1, UR5e and FR3 catalog).
3. Start DEP-25 (umbrella switch and fail-fast), then the tinyxml2 fetch
   prototype; DEP-18, DEP-19, DEP-30 and the urdfdom 6.0.1 wheel bump follow
   the decisions; the backports (DD-27, DD-32) wait for their main PRs.
4. Owed spikes: the Werror C++20 budget on every toolchain, `abidiff` between
   the C++17 and C++20 builds, and the stock-noble Tier 2 build. The distro
   FCL abort is fixed in a DART-carried patch (DD-18, #3870); nothing is
   reported upstream.
5. After M1d: DEP-21, ball-joint limits for MJCF, the libccd attribution in
   the MPR sources, and any quiet-window hunk the PLAN-624 owner did not
   clear earlier.

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
