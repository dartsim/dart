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
  #3877 (DEP-36), #3878 (DEP-33(b)), #3879 (DEP-25), #3880 (DEP-33(a)),
  #3881 (DEP-34, stacked on #3877), #3882 (DEP-33(c), stacked on #3857),
  #3883 (DEP-2), #3884 (DEP-17 PR1), #3885 (DEP-8(f)), #3886 (DEP-4(e)),
  #3887 (DEP-17 PR3, stacked on #3885), #3888 (DEP-17 PR4, stacked on
  #3884).
- CI: every PR that merged current `main` fails `test_api_surface_budget` on
  the Python jobs (`dartpy.experimental` imported earlier in the run); open
  PR #3872, from another workstream, fixes it. Merge `main` into the plan's
  PRs once it lands.
- Merge order: #3857 after #3856; #3854 after #3853 and #3850; #3865 after
  #3850. DD-33 covers pushes to drafts and DD-34 the merge, conflict and CI
  pushes to ready PRs; marking ready, merging and backports are asked.

## Next steps

1. Watch CI on the plan's PRs; after #3872 merges, merge `main` into them
   (DD-33, DD-34). Overlaps to resolve at merge time: #3876 and #3879 in
   `dart/gui/imgui/CMakeLists.txt` (drop the ImGui guard's source-directory
   argument); #3886 and DEP-18 in the optional-dependency docs (add sdformat
   to the strict switch's list); #3888 and DEP-18 in
   `dart/utils/CMakeLists.txt`.
2. Finish DEP-18 (fix round 1), DEP-7, DEP-9 and the DEP-5 remainder.
   After #3887 merges, the release-6.20 backport of its urdfdom wheel bump
   (DD-32) is asked for; after #3859 merges, its backport (DD-27).
3. Then: DEP-4 (a) to (c), DEP-10 and DEP-6 (after #3879), DEP-12(b) (after
   #3850), DEP-19's wheel half (curl and OpenSSL in the wheels, on #3885),
   the tinyxml2 fetch prototype on DEP-25, DEP-16, and DEP-33(d) after
   DEP-19.
4. Follow-ups found on the way: MJCF inertia from all geoms and
   `<compiler inertiafromgeom>`; MJCF heightfields; the root `NOTICE` entry
   for the DeepMind models once #3857 merges; `data/sdf/atlas/atlas_v3.urdf`
   fails to load (its `head` link is missing); urdfdom in `package.xml` could
   become build and exec dependencies now that shared installs link it
   privately.
5. Owed spikes: the Werror C++20 budget on every toolchain, `abidiff` between
   the C++17 and C++20 builds, and the stock-noble Tier 2 build.
6. After M1d: DEP-21, ball-joint limits for MJCF, the libccd attribution in
   the MPR sources, the `GenericJoint` finite checks that GCC 13 stopped
   inlining after DEP-14 (`array().isFinite().all()`; see #3853's perf
   rationale), and any quiet-window hunk the PLAN-624 owner did not clear
   earlier.
7. Routing: Claude's 7-day usage reached 58% on 2026-10-20, so new writer
   packets go to Codex or Muse and Claude-side reviews use Sonnet.

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
