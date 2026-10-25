# Resume

## Current state (2026-10-25)

- The plan is on `main` (#3848); this home is updated through follow-up PRs.
- The maintainer answered the decision batches and the later questions one
  at a time ([decisions.md](decisions.md), DD-10 to DD-38). DD-33 is the
  standing approval for this plan's publishing. DD-34 extends it to merging
  `main`, resolving conflicts and fixing CI on the plan's ready PRs. Under
  DD-35, drafts leave draft once they pass the PR Lifecycle gate.
- Every PR the plan published through 2026-10-25 has merged (list below).
  The FCL/libccd fetch path (DD-36) is in review and not yet published.
- The maintainer's order from 2026-10-24 stands. The in-flight work, the
  queued decisions and both backports are done, so the remaining packets
  come next, then DEP-21 after its prerequisites below.
- Two local spike branches hold experiment edits only and will not be
  published: `spike/cxx20-standard` and `spike/cxx20-private`. Their worktrees
  hold the Gazebo lane builds and base variants; remove both once DEP-21's own
  evidence replaces them. The tinyxml2 fetch prototype is parked on the local
  branch `build/tinyxml2-fetch`.
- gz-sim's `INTEGRATION_reset_sensors` is an external test defect (#3866).
- Muse's quota is exhausted until 2026-10-26; until then packets go to Codex
  and Claude, and each packet's two reviews come from whichever of those two
  did not write it (two passes from the same engine when needed).

## Published

- Merged in `main`: #3848 (this plan), #3849 (DEP-0), #3850 (DEP-12(a)),
  #3851 (DEP-1), #3852 (DEP-11 PR1), #3853 (DEP-14), #3854 (DEP-15),
  #3856 (DEP-28), #3857 (DEP-27a), #3859 (DEP-32), #3860 (DEP-20(d)),
  #3861 (DEP-31), #3862 (DEP-20(e)), #3863 (DEP-17 PR2), #3864 (DEP-3(a)),
  #3865 (DEP-13 PR1), #3870 (DEP-35), #3875 (DEP-37), #3876 (DEP-30),
  #3877 (DEP-36), #3878 (DEP-33(b)), #3879 (DEP-25), #3880 (DEP-33(a)),
  #3881 (DEP-34), #3882 (DEP-33(c)), #3883 (DEP-2), #3884 (DEP-17 PR1, with
  PR4 #3888), #3885 (DEP-8(f)), #3886 (DEP-4(e)), #3887 (DEP-17 PR3),
  #3889 (DEP-18), #3890 (DEP-7), #3891 (DEP-5), #3892 (DEP-9),
  #3893 (DEP-6), #3894 (DEP-10, through #3892), #3895 (DEP-4(c)),
  #3896 (DEP-12(b) tinyxml2), #3897 (perf-harness links), #3898 (DEP-4(a)),
  #3899 (DEP-16, through #3854), #3900 and #3901 (DEP-4(b)), #3921
  (DEP-33(e)), #3922 (libcurl 8.5.0), #3923 (OpenSSL 3.0.13), #3924
  (DEP-13(d)), #3925 (DEP-11 PR3), #3929 (installed DART finds its fetched
  Bullet), #3931 (DEP-19's Linux wheels), wave 7: #3936 (a Clang
  `-Werror` fix for #3924's test), #3937 (DEP-12(a)'s Tier 1 apt lane),
  #3938 (DEP-13(c)), #3939 (DEP-22 Tracy), #3940 (DEP-24), #3941 (DEP-22
  CMake cap), #3942 (DEP-20(b)) and #3943 (DEP-11 PR4), and wave 8: #3952
  (DEP-3(b)), #3953 (`DART_STRING_TYPE`), #3954 (DEP-23), #3955 (DEP-8(b)),
  #3957 (DEP-8(e)) and #3958 (DEP-8(c)).
- Merged in `release-6.20` for 6.20.4: #3947 (#3859's backport, DD-27) and
  #3948 (#3887's backport, DD-32).
- DEP-8(a) needed no PR: #3847 already stopped packaging `tests` in wheels.
  DEP-13(b) needed no PR: #3865 already deleted those paths.

## Next steps

1. Publish the FCL/libccd fetch path (DD-36) once its review findings are
   fixed and two reviews pass on the result.
2. DEP-20(c) under DD-38, in this order:
   - Check for Clang 20 or newer and `GL/glx.h` before the 828 MB Filament
     download, with an error that names `clang-20 libgl-dev`.
   - Declare `DART_USE_SYSTEM_FILAMENT` after the umbrella and flip its
     default where DART has a private SDK, as a labeled change with a
     changelog fragment.
   - Document the noble apt line, including `libgl-dev` and
     `fonts-dejavu-core`.
   - Fix the Tier 2 row: the CMake route needs no pip packages.
   - Add a nightly Tier 2 lane: noble apt packages plus clang-20,
     `VK_DRIVER_FILES=/usr/share/vulkan/icd.d/lvp_icd.json`, Xvfb, and
     `-E '^test_Distance$'`.
3. Make the Filament notice download honor
   `FETCHCONTENT_FULLY_DISCONNECTED` (found by DEP-23).
4. DEP-33(d): the online catalog entries DD-24 names that are not in
   `data/robot_models/` yet.
5. DEP-23's remaining handoff proposals: batch raycasts, the MSVC probe, and
   a `utils-urdf`-only package. #3954's research drafted only the
   gz_dartsim_vendor and gz-physics notes, and nothing is filed upstream.
6. DEP-19's macOS and Windows wheels with a minimal libcurl and OpenSSL
   (DD-37). Validating them needs a dispatched wheel workflow run; ask the
   maintainer before dispatching it.
7. DEP-22's rest: model CMP0200 per imported target in the Bullet precision
   probe, then lift the CMake cap; FreeBSD patches upstream; formatter
   bumps. The clang-format reformat follows the PLAN-624 order.
8. DEP-21 prerequisites, then DEP-21 itself in the PLAN-624 merge order:
   - Replace gate 3's raw `abidiff` condition with the scripted checks
     ([verification](verification.md#wave-8-2026-10-24-to-2026-10-25) and
     #3953's object comparison).
   - Add a mixed-dialect consumer to `tools/package`.
9. Follow-ups found on the way:
   - Linux wheels ship notices for libcurl and OpenSSL only, not for the
     other bundled libraries (FCL, assimp, ODE, sdformat and the gz
     libraries among them).
   - main's #3887 changelog entry says Linux wheels accept URDF 1.1 and 1.2;
     files with geometry DART lacks, such as capsules, still fail.
   - `test_compare_soft_body_detector_pairs` fails outside a git checkout
     and leaves a lock-file handle open, whose warning fails
     `test_evidence_publish`.
   - MJCF inertia from all geoms and `<compiler inertiafromgeom>`; MJCF
     heightfields.
   - urdfdom in `package.xml` could become build and exec dependencies now
     that shared installs link it privately.
   - The Korean translations of the build docs still list Python 3.10 and
     3.11.
   - DEP-16's optional review nits: clear retired wrappers on an empty wrap
     result, and restore the log level before the bridge's restore.
   - The ~30 property structs that share DEP-11's deprecated-copy pattern.
   - Clang's `-Wunused-const-variable` on `kMeshMax` in
     `tests/benchmark/simd/bm_simd.cpp`.
   - A dartpy binding for `SoftMeshShape.getTriMesh()` after #3938.
10. Sequence in the PLAN-624 merge order: ball-joint limits for MJCF, the
   libccd attribution in the MPR sources, and the `GenericJoint` finite
   checks that GCC 13 stopped inlining after DEP-14
   (`array().isFinite().all()`; see #3853's perf rationale), each
   re-qualified on current `main`.

## Risks and recovery

- `main` moves quickly. Re-check file and line references in
  [packets.md](packets.md) and the review on current `main` before each
  packet.
- `World.cpp` and `lcpsolver` hunks follow the PLAN-624 program-owned merge
  order, each re-qualified on current `main`.
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
