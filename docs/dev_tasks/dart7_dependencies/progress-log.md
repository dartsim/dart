# Progress log

- **2026-10-18**
  - The maintainer directed the effort: minimal, self-contained DART 7
    dependencies, an Ubuntu 24.04 floor, and a higher C++ standard where
    possible.
  - Ubuntu 22.04, 24.04 and 26.04 package versions recorded from containers
    and the host.
  - Spikes on `main` `17ce7eefa`: stock Ubuntu 24.04 builds core and
    `dart-io` (S3, S3c); DART builds and passes its suites as C++20 (S1);
    Harmonic and Jetty lanes pass against the C++17 base for both C++20
    options (S2, S2b).
  - The research workflow (eleven dossiers, seven studies, adversarial
    verification, synthesis, critique) and an independent Codex audit of the
    C++ standard produced the inventory and 25 packets.
  - Codex classified gz-sim's `INTEGRATION_reset_sensors` failure as a
    use-after-free in the gz-sim test; ownership handed to the DART 7 program
    session.
  - This project home and the PLAN-625 dashboard entry were written on the
    local branch `docs/dart7-dependency-plan`.
- **2026-10-19**
  - The maintainer answered the decision batch: option B for the C++
    standard, the standard library over third-party libraries (spdlog and fmt
    go), specialized libraries kept for URDF and SDF, a tradeoff study for
    asset networking, the clean long-term public surface within gz
    compatibility, and fetch-first source builds with packager switches and
    no bundled copies where avoidable.
  - #3829 merged into `main` (`548ff24ad`).
  - The follow-up study started, and Wave 1 started: DEP-0, DEP-11 PRs 1-2
    and DEP-14.
  - Wave 1 finished locally: DEP-0, DEP-1, DEP-11 PR1, DEP-12(a) and
    DEP-14 are committed on their own branches, each with two clean reviews
    from engines other than its writer. The Harmonic, Ionic and Jetty lanes
    pass on their combined tree; the Harmonic and Jetty lanes caught a
    missing `Threads::Threads` link in DEP-14 under the conda toolchain,
    fixed before the final run. DEP-11's capture commit was dropped in
    favor of open PR #3804, which makes the same change.
  - The branches were rebased onto `0d7f6d77a`; their integration branch
    passes the default gates.
  - The balance review under DD-9 re-scoped the packet plan and produced a
    second decision batch ([balance-review.md](balance-review.md)).
  - Wave 2 committed locally: DEP-28, DEP-20(e), DEP-20(d), DEP-31,
    DEP-17 PR2, DEP-32, DEP-3(a) and DEP-27a; Codex and Muse wrote every
    packet, and each passed two clean reviews from other engines.
  - DEP-13 PR1 committed (assimp 5.3 floor; raw scenes owned again, as in
    DART 6). The Gazebo lanes pass on the full Wave 2 tree; DEP-15 started.
  - DEP-15 committed on DEP-14: fmt leaves DART; a differential test
    against fmt caught a float layout bug that both reviewers missed.
  - The distro FCL assertion is an FCL GJK bug at exact ellipsoid
    tangency (aborts with Ubuntu's asserting build, wrong nearest points
    otherwise); upstream reports await approval.
  - The maintainer answered the second batch one question at a time
    (DD-10 to DD-34); the 16 finished branches were published, and DEP-35
    to DEP-37 were added (the FCL patch replaces the upstream reports).
  - CI fixes: #3853 failed on macOS because libc++ in C++20 mode reports
    the deprecated Assimp `MeshShape` constructor inside `construct_at`,
    outside the call site's suppression; 17 test and binding sites now call
    `new` directly. Its perf rows rose under GCC 13 only (an Eigen
    `allFinite()` outlined in `GenericJoint`), recorded as a rationale with
    the fix deferred past M1d. #3854's Windows failure was a test feeding
    native paths to a macOS-only guard.
  - DEP-33 split into four PRs; DEP-30, DEP-25 and DEP-33 (a) to (c)
    started. DEP-37 (#3875), DEP-30 (#3876) and DEP-36 (#3877) published
    after two clean reviews each.
  - A full local `test-py` run fails `test_api_surface_budget` whenever an
    earlier test imported `dartpy.experimental`; open PR #3872 fixes it.
  - DEP-33(b) (#3878), DEP-25 (#3879), DEP-33(a) (#3880), DEP-34 (#3881)
    and DEP-33(c) (#3882) published after two clean reviews each. DEP-34
    fixed three MJCF parser bugs the DeepMind models exposed instead of
    editing the models around them: world-level sites crashed, `<freejoint>`
    was ignored (bodies were welded), and a failed model with a weld
    dereferenced a null world. The orchestrator's capture review sent
    DEP-33(b)'s arm back for an upright zero pose and rounded meshes, and
    gave the WAM's STL visuals colors that match the removed textures.
  - DEP-2, DEP-17 PR1 and DEP-8(f) started.
