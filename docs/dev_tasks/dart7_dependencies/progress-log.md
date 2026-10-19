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
