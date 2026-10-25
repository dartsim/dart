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
- **2026-10-20**
  - DEP-2 (#3883), DEP-17 PR1 (#3884), DEP-8(f) (#3885), DEP-4(e) (#3886),
    DEP-17 PR3 (#3887) and PR4 (#3888) published after two clean reviews
    each. Reviews caught a DEP-4(e) bug (a deliberate `DART_SKIP_*` still
    failed under the strict switch) and a pre-existing macro bug (the
    optional-dependency helper's version test never fired); DEP-17 PR3's
    acceptance table showed every URDF in `data/` loading the same with
    urdfdom 4.0.1 and 6.0.1.
  - DEP-18 (#3889), DEP-7 (#3890), the DEP-5 remainder (#3891), DEP-9
    (#3892), DEP-6 (#3893) and DEP-10 (#3894, stacked on #3892) published
    after two clean reviews each. Reviews sent DEP-18's SDF-off test skips
    back to per-test marks, DEP-7's unqualified tasks back to one
    environment each, and DEP-16's first design back to an unchanged
    default console (the sink changes nothing until a custom sink is set).
  - An experiment settled `DOWNLOAD_EXTRACT_TIMESTAMP`: `FALSE` extracts at
    the current time, which is what #3876 and #3893 use.
  - DEP-4 (a) to (c), DEP-8(a), DEP-12(b) (tinyxml2) and the tinyxml2 fetch
    prototype on DEP-25 started. Muse's quota ran out until 2026-10-26, so
    its packets moved to Codex and Claude, and reviews come from those two
    engines until then.
  - DEP-4(c) (#3895), DEP-12(b) (#3896), the perf-harness link fix (#3897),
    DEP-4(a) (#3898) and DEP-16 (#3899) published after two clean reviews
    each; DEP-8(a) needed no PR (#3847 covered it). Windows-only test
    failures on #3882 (CRLF license checkouts) and #3891 (file times sharing
    a clock tick) were fixed. Reviews kept the octomap shim, caught a
    DEP-4(b) uninstall bug that could delete files outside the manifest, and
    sent DEP-16 through two more rounds on sink restoration order.
  - The tinyxml2 fetch prototype showed fetch-first cannot work for a
    library that DART's required dependencies already load; the maintainer
    was asked how DD-10 applies.
- **2026-10-21 to 2026-10-23**
  - The maintainer marked the plan's PRs ready and merged them in the
    recorded order; by 2026-10-23 every PR through #3901 had merged.
  - DEP-4(b) published and merged as #3900 (ODE) and #3901 (Bullet).
- **2026-10-24**
  - Wave 6: #3921 (DEP-33(e)), #3922 (libcurl 8.5.0), #3923 (OpenSSL
    3.0.13), #3924 (DEP-13(d)) and #3925 (DEP-11 PR3) published after two
    clean Codex reviews each, and merged. The OpenSSL tests needed a fix for
    CMake 3.28 that a bot review found.
  - The nightly ODE and Bullet fetch lane failed because an installed DART
    did not find the Bullet it fetched; #3929 fixed it after three review
    rounds. DEP-19's Linux wheels published and merged as #3931.
  - Checkpoint with the maintainer: land the in-flight work, then the
    queued decisions one at a time, then the backports, then the remaining
    packets, with DEP-21 after M1d.
  - Wave 7 (Claude workers, Codex reviews): the Tier 1 apt lane, DEP-11
    PR4, DEP-13(c), DEP-22's CMake cap and Tracy, DEP-20(b) and DEP-24
    published as drafts #3937 to #3942 and #3943. Its two spikes found DEP-21's four targets
    clean on every Linux toolchain, a `DART_STRING_TYPE` initialization hazard
    between dialects to fix first, and a Clang `-Werror` failure from #3924,
    fixed in #3936.
  - DEP-24's research found gz-physics uses `Eigen::Vector6d` and
    `Eigen::Matrix6d`, so Q3's Eigen-alias deprecation skips them (dated note
    in [decisions.md](decisions.md)).
  - The maintainer asked that finished drafts leave draft once they pass
    the PR Lifecycle gate (DD-35); every open draft got an `@codex review`,
    and the maintainer merged #3936 to #3942 the same evening. #3939's merge
    settled the Tracy viewer question. The maintainer then answered the two
    queued questions: fetch-first follows the loaders (DD-36), and macOS and
    Windows wheels build a minimal libcurl and OpenSSL (DD-37). DEP-11 PR4
    published as #3943 after nine review rounds, which narrowed the test to
    top-level builds and skipped the SIMD backend fragments.
  - Routing: Muse is out of quota until 2026-10-26, so Claude workers in
    isolated worktrees wrote wave 7 (including packets the table assigns to
    Codex or Muse) and Codex reviewed every packet.
  - Later that evening the maintainer merged #3943, the project-home update
    (#3945) and both 6.20.4 backports: #3947 (#3859) and #3948 (#3887).
- **2026-10-25**
  - Wave 8 (Claude workers, Codex reviews; Codex wrote DEP-3(b)) merged as
    #3952 (DEP-3(b)), #3953 (`DART_STRING_TYPE`), #3954 (DEP-23 docs),
    #3955 (DEP-8(b)), #3957 (DEP-8(e)) and #3958 (DEP-8(c)). Each draft left
    draft through the DD-35 gate.
  - The DEP-20(a) spike built and tested the GUI and dartpy on stock Ubuntu
    24.04. The maintainer then chose fetch-first for the Filament SDK where
    DART has a private one (DD-38).
  - PR text now follows the Writing Style and PR Descriptions sections of
    `contributing.md` (#3951).
  - The FCL/libccd fetch path (DD-36) is in review.
