# Resume

## Current state (2026-10-20)

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
- Muse's quota is exhausted until 2026-10-26; until then packets go to Codex
  and Claude, and each packet's two reviews come from whichever of those two
  did not write it (two passes from the same engine when needed).

## Published (2026-10-20)

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
  #3884), #3889 (DEP-18), #3890 (DEP-7), #3891 (DEP-5), #3892 (DEP-9),
  #3893 (DEP-6), #3894 (DEP-10, stacked on #3892), #3895 (DEP-4(c)), #3896
  (DEP-12(b) tinyxml2, stacked on #3850), #3897 (perf-harness link fix for
  #3889), #3898 (DEP-4(a), stacked on #3885), #3899 (DEP-16, stacked on
  #3854).
- DEP-8(a) needed no PR: #3847 already stopped packaging `tests` in wheels.
- CI: every PR that merged current `main` fails `test_api_surface_budget` on
  the Python jobs (`dartpy.experimental` imported earlier in the run); open
  PR #3872, from another workstream, fixes it. Merge `main` into the plan's
  PRs once it lands.
- Merge order: #3857 after #3856; #3854 after #3853 and #3850; #3865 after
  #3850; #3899 after #3854; #3896 after #3850; #3898 after #3885. #3889's
  perf gate passes only after #3897 lands and #3889 merges `main` (the gate
  runs the base revision's harness); #3889 then needs a
  `Rebaseline-Rationale` for `pend/dart`. #3879 carries one for `dyn` and
  `lcp`, and #3893 will need the same, because the harness hashes
  `tests/benchmark/CMakeLists.txt` into those rows. DD-33 covers pushes to drafts and DD-34 the merge, conflict and CI
  pushes to ready PRs; marking ready, merging and backports are asked.

## Next steps

1. Watch CI on the plan's PRs; after #3872 merges, merge `main` into them
   (DD-33, DD-34). Overlaps to resolve when the second PR of each pair
   merges `main`:
   - #3876 and #3879 in `dart/gui/imgui/CMakeLists.txt`: drop the ImGui
     guard's source-directory argument.
   - #3886 and #3889 in the optional-dependency docs: add sdformat to the
     strict switch's list.
   - #3888 and #3889 in `dart/utils/CMakeLists.txt`.
   - #3884 and #3889 in the shared IO CHANGELOG bullet.
   - #3891, #3892 and #3894 in `scripts/nanobind/eigen_probe/CMakeLists.txt`.
   - #3893 and #3894 against #3879's offline-guard lines.
   - #3870 and #3885 in the FCL block of the wheel image.
2. Waiting on the maintainer: whether fetch-first applies to libraries that
   DART's required dependencies or Gazebo load in the same process
   (tinyxml2, urdfdom, assimp). The tinyxml2 prototype (local branch
   `build/tinyxml2-fetch`, unpublished) and two reviews show a fetched copy
   is either redundant or a second ABI in one process; as default it would
   break the standalone configure on Ubuntu 24.04 and every non-Linux
   platform. Recommended: those libraries follow their consumers (system),
   fetch-first stays for ODE, Bullet, FCL/libccd and octomap, and the next
   fetch prototype is FCL/libccd. In progress meanwhile: DEP-4(b) (final
   review fixes; it publishes as an ODE PR on #3879 with a Bullet PR on top)
   and DEP-19's wheel half (in review, on #3885).
3. Asked when the condition holds: the release-6.20 backport of #3887's
   urdfdom wheel bump (DD-32) after it merges; #3859's backport (DD-27)
   after it merges.
4. Then: DEP-33(d) after DEP-19's wheel half, DEP-3(b), DEP-8 (b), (c) and (e), the
   remaining DEP-12(b) floors (libcurl 8.5.0 and OpenSSL 3.0.13 with
   DEP-19), DEP-22, DEP-23 and DEP-24.
5. Follow-ups found on the way: MJCF inertia from all geoms and
   `<compiler inertiafromgeom>`; MJCF heightfields; the root `NOTICE` entry
   for the DeepMind models once #3857 merges; `data/sdf/atlas/atlas_v3.urdf`
   fails to load (its `head` link is missing); urdfdom in `package.xml` could
   become build and exec dependencies now that shared installs link it
   privately; `DART_BUILD_WHEELS` is unused after #3892 (remove it once
   #3850 and #3892 merge, since #3850 edits the same test); the Korean
   translations of the build docs still list Python 3.10 and 3.11;
   `data/sdf/atlas/atlas_v3.urdf` can never load (its neck joint names a
   missing `head` link) and can go in a PR stacked on #3882; DEP-16's
   optional review nits (clear retired wrappers on an empty wrap result,
   restore the log level before the bridge's restore).
6. Owed spikes: the Werror C++20 budget on every toolchain, `abidiff` between
   the C++17 and C++20 builds, and the stock-noble Tier 2 build.
7. Sequence DEP-21, the `World.cpp` captures from DEP-11, ball-joint limits
   for MJCF, the libccd attribution in the MPR sources, the `GenericJoint`
   finite checks that GCC 13 stopped inlining after DEP-14
   (`array().isFinite().all()`; see #3853's perf rationale), and other
   step-path hunks in the PLAN-624 program-owned merge order, each
   re-qualified on current `main`.
8. Routing: Claude's 7-day usage reached 58% on 2026-10-20, so new writer
   packets go to Codex first; Claude-side reviews use Sonnet, and Opus for
   the hardest.

## Risks and recovery

- `main` moves quickly. Re-check file and line references in
  [packets.md](packets.md) and the review on current `main` before each
  packet.
- The step-path quiet window is closed: `World.cpp` and `lcpsolver` hunks
  follow the PLAN-624 program-owned merge order, each re-qualified on current
  `main`.
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
