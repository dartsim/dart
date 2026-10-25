# Decisions

Revisit a decision only with new evidence and maintainer approval.

## Maintainer direction (2026-10-18)

- **DD-0. Minimal, self-contained DART 7.** Optimize DART 7's dependencies
  (not DART 6's): use as few as possible so DART is self-contained, more
  feature complete and faster to evolve.
- **DD-1. Ubuntu 24.04 floor.** Raise the minimum supported Ubuntu LTS from
  22.04 to 24.04 on `main`, and raise every dependency floor to match.
  `release-6.20` keeps Ubuntu 22.04.
- **DD-2. C++ standard.** Raise the C++ standard where that is possible on
  the new floor. The evidence and options are in Q1 below.

## Answers to the decision batch (2026-10-19)

- **DD-3. C++ standard: option B.** DART's own targets build as C++20;
  exported targets keep `cxx_std_17`; installed headers stay C++17-clean and
  the `ContactSurface.hpp` include tree stays parseable at gz-physics'
  `-std=c++14` probe; the C++20 library components GCC 16 changed are banned
  in DART code until the GCC floor reaches 16. AGENTS.md's "preserve C++17"
  and the PLAN-624 constraint are reworded to say exactly this (DEP-21).
- **DD-4. Standard library over third-party libraries.** The maintainer's
  words: "Using STL is good approach over 3p libraries." Logging moves to a
  DART-owned sink and formatter built on the standard library, and spdlog and
  fmt leave DART (DEP-14 to DEP-16).
- **DD-5. Specialized libraries for robot description formats.** The
  maintainer's words: "urdf and sdf would be better to use the specialized
  libraries." urdfdom stays as the URDF parser and sdformat as the SDF
  parser; the in-house URDF reader (former DEP-17) is dropped. The follow-up
  study answered whether SDF moves into its own library: no, it stays an
  optional part of `dart-io` (DD-11).
- **DD-6. Asset networking needs a tradeoff study.** The maintainer's words:
  "Not sure about assets. Need more tradoff study for me." DEP-19 waits for it.
- **DD-7. Clean, long-term public surface.** The maintainer's words: "Prefer
  the clean/long-term/proper approach, as long as keeping backward
  compatibility for gz-physics and gz-sim." The Q3 batch is approved, and
  later packets prefer the clean fix over a compatibility shim wherever the
  gz-physics and gz-sim compatibility gates still pass.
- **DD-8. Dependency acquisition.** The maintainer's words: "Probably
  case-by-case. However, fetch first for build from source (so that users
  don't need to install deps unnecessarily), system first when building for
  package manager (but in this case package managers can explicitly set the
  DART_USE_SYSTEM_<X>). Bundled should be removed whenever possible over the
  other two options." Default source builds fetch pinned versions where that
  works, packagers opt into system packages with `DART_USE_SYSTEM_<X>`, and
  in-tree bundled copies give way to fetched or system packages. The
  per-dependency matrix is part of the follow-up study.

- **DD-9. Balance, not blind removal.** The maintainer's words: "find good
  balance between self-contained (less dependency) vs.
  simplify/productivity (more dependency)" and "not simply/blindly removing
  dependencies". Every removal, replacement or internalization must show a
  net benefit: what it saves (packaging, footprint, exposure in installed
  headers, upstream churn, security surface) against what it costs (code DART
  then owns and maintains, its bugs and security fixes, build and developer
  productivity, features the library provided). A dependency that is well
  maintained, cheap to obtain and kept out of DART's public interface may stay.
  This also governs how DD-4 and DD-8 apply case by case.

## Follow-up study (requested 2026-10-19)

- Asset networking: keep, isolate or move to Python, with security,
  footprint, packaging and maintenance tradeoffs (DD-6).
- SDF packaging with sdformat kept: its own library or not (DD-5; answered by
  DD-11).
- The per-dependency acquisition matrix under DD-8, and an inventory of
  in-tree bundled third-party code.

## Answers to the second batch (2026-10-19)

The maintainer answers these one question at a time.

- **DD-10. Acquisition by kind of build (Q4, re-affirming DD-8).** The
  maintainer's words: "Again, fetch first for dart standalone build. But in
  package managers, in pixi, system use first to be consistent." A
  standalone DART source build fetches its dependencies first, the
  libraries Gazebo also loads included, so it needs no system installs.
  Package-manager builds and pixi use the environment's libraries first,
  consistent with everything else built there, selected through the
  umbrella switch (DEP-25) or the per-dependency `DART_USE_SYSTEM_<X>`
  switches. sdformat is an explicit exception to the literal fetch-first
  rule: DD-11 applies, so a standalone build uses an installed sdformat (a
  supported major, 14 to 16) and fetches only when none is found, once the
  DEP-25 fetch path supports it; where the environment cannot provide what DART needs (Dear
  ImGui's docking release), they fetch. The balance review's system-only
  proposal for the Gazebo-shared libraries is not adopted. Its findings
  become constraints on the fetch path: FCL, urdfdom and ODE define
  unguarded `uninstall` targets and need configure-time sub-builds, a
  fetched copy must not end up next to Gazebo's own (package-manager and
  pixi builds never fetch them), and DART owns updates for every pinned
  copy. The fetch path starts with a one-library prototype (tinyxml2) to
  bound its cost.

- **DD-11. SDF is an optional part of `dart-io` (Q3(a)).** DEP-18 reuses
  the optional-dependency mechanism Bullet and ODE use, adds an `io-sdf`
  forwarding component and exports sdformat only for static builds. Pixi,
  package managers, CI, wheels and the Gazebo lanes use the environment's
  sdformat with the strict switch, so a missing sdformat fails there. A
  standalone build uses an installed sdformat if present and otherwise
  fetches sdformat with gz-cmake, gz-math and gz-utils (DD-10); until that
  fetch path lands, a standalone build without sdformat skips SDF with a
  warning. Builds without SDF ship the legacy `utils` component without
  `SdfParser`, which the maintainer approved as part of this answer.

- **DD-12. C++ and Python feature parity; the asset retriever stays
  (Q2).** The maintainer's words: "C++ and Python should be feature parity,
  where Python is binding of C++ for Python, where the Python API should be
  Pythonic." The balance review's Q2(a), deleting the unreleased C++
  `ModelResourceRetriever` for a Python-only fetcher, would have broken
  that parity and is withdrawn. `ModelResourceRetriever` stays in C++
  (`utils-assets`), with the dartpy binding as its Pythonic surface and no
  separate Python fetcher; `dart.data` builds on the binding. It keeps
  libcurl and OpenSSL (SHA-256 verification and random temporary names),
  which DEP-2 exports only for static builds; DD-10 decides how each kind
  of build obtains them. DEP-19 is re-scoped accordingly.

- **DD-13. Wheels ship the asset retriever.** dartpy wheels enable
  `utils-assets`, so pip users get `ModelResourceRetriever` like C++ and
  pixi users (DD-12). The Linux wheel image builds current libcurl and
  OpenSSL 3 from hash-pinned release archives (DEP-8(f)); macOS and Windows
  wheels bundle conda-forge's. DART rebuilds the wheels for curl and OpenSSL
  advisories, and the PR measures the wheel-size cost (estimated at about
  2 MB compressed).

  Note (2026-10-24): measured, the locked conda-forge libcurl closure adds
  about 18.2 MB (macOS) and 16.5 MB (Windows) compressed, not 2 MB, and
  Linux adds about 2.6 MB. Pending the maintainer's choice between that cost and
  a minimal source-built curl, DEP-19 enables the retriever in Linux wheels
  only, with per-platform size budgets.

- **DD-14. The Dear ImGui compatibility patch is dropped (Q5(a)).** Once
  #3861 (DEP-31) lands, DEP-30 fetches ImGui as a hash-pinned,
  TLS-verified tag archive with no patch, deletes `dart-compat.patch`, its
  apply and stamp CMake and the three regression tests, and adds a labeled
  changelog entry: third-party code that misuses DART's re-exported ImGui
  gets upstream's behavior (the 6.16.x guards for #2516, #2668 and #2671 are
  reverted), and `ImFormatString` is exported again.

- **DD-15. Replace the Roboti-derived MuJoCo models.** The 14 top-level MJCF
  models under `data/mjcf/openai/` (15 in the first count, which was wrong;
  the six Fetch robotics scenes under `robotics/` belong to DD-19's removal
  scope) that Gym derived from Roboti's MuJoCo models
  (attribution, no license grant) are replaced with Apache-2.0 models from
  DeepMind's MuJoCo and dm_control (hopper, cheetah, walker, swimmer,
  reacher, humanoid, pendulums, quadruped), and the MJCF parser tests'
  expected values are rewritten for the new fixtures. It can land any time
  before the 7.0 release (new packet DEP-34).

- **DD-16. Research the unattributed meshes, then decide per set.** A
  research packet traces Atlas, WAM, KR5, kima and the Fetch meshes to
  their upstreams and licenses. Sets with a recoverable license get NOTICE
  entries and license files (DEP-33); any set without one goes back to the
  maintainer as its own question, with replace or remove options and their
  test impact.

- **DD-17. Modernize the robot models.** The maintainer's words: "also
  consider modernize the robot models (e.g., replacing old model with
  newer/popular models with good license)". Alongside the provenance
  research (DD-16) and the MuJoCo replacement (DD-15), a research packet
  maps each role the old models play in tests, examples and demos to
  current, popular, permissively licensed models, and proposes which live
  in-tree as small fixtures and which come as pinned, hash-checked bundles
  through `ModelResourceRetriever` (DD-12). The maintainer decides per set.

- **DD-18. Fix the FCL GJK tangency bug in a DART-carried patch.** The
  maintainer's words: "Don't report, but just fix it internally, with clear
  note and patch file, so that it can eventually submitted to the
  upstream". DART carries a patch for FCL 0.7.0's GJK initial-origin case,
  with a note that records the problem, the reproducer and how to submit it
  upstream, and applies it wherever DART builds FCL itself (the Linux wheel
  image now, the standalone fetch path under DD-10 later). Nothing is
  reported upstream for now; system FCL in pixi and distributions stays
  unpatched, and the Ubuntu 24.04 apt lane lists `test_Distance` as an
  expected external failure pointing at the note (new packet DEP-35).

- **DD-19. Remove the Fetch assets.** The 26 Fetch files under
  `data/mjcf/openai/robotics/` (six MJCF scenes, 18 STL meshes, two
  textures, via OpenAI Gym) carry Fetch Robotics' noncommercial
  CC-BY-NC-ND-4.0 / CC-BY-NC-SA-4.0 terms from `fetch_ros`. They are
  removed, and `MjcfParserTest.RoboticsFetch` and Python's
  `test_parse_fetch` move to a small DART-authored BSD MJCF fixture that
  keeps their coverage (includes, meshes, hierarchy, default detector). A
  mobile-manipulator demo can come later as a PAL TIAGo (Apache-2.0)
  online bundle (part of DEP-34).

- **DD-20. WAM keeps only its BSD files.** The WAM URDF's visuals move to
  the BSD-3-Clause STLs it already includes (`wam1`-`wam7.STL`,
  `wam_base.STL`); the eight GPL-3.0-or-later COLLADA meshes and six JPEG
  textures (RE2, Inc.) are deleted; upstream `herb_description`'s
  `LICENSE.bsd` ships verbatim with a NOTICE entry (CMU, 2013). The
  analytical-IK demo and its regression keep working; replacing the WAM with
  a modern arm stays a separate catalog decision (part of DEP-33).

- **DD-21. Remove the kima meshes.** `data/skel/kima/` (three SKELs, seven
  COLLADA meshes) has no license and likely comes from a commercial human
  model catalog that restricts redistribution. It is removed, and the three
  `MeshShape` tests that load `l-foot.dae` move to a tiny DART-authored
  centimeter-unit COLLADA fixture that keeps their coverage (unit scaling,
  metadata refresh on pointer reuse, extensionless URIs) (part of DEP-33).

- **DD-22. Replace the KUKA KR5.** `data/urdf/KR5/` has only a bare
  "BSD" declaration and meshes generated from KUKA's CAD with unknown
  redistribution terms. A small DART-authored six-axis URDF with tiny meshes
  replaces it first as the offline fixture for the tests and hello-world
  (keeping `KR5MeshColor`'s mesh-color coverage); then Universal Robots
  UR5e (BSD-3-Clause, ROS-Industrial Consortium) arrives as a pinned online
  bundle for the operational-space-control and point-cloud demos and the
  dominoes tutorial (part of DEP-33).

- **DD-23. Attribute Atlas v3; feature the Unitree G1.** The in-tree
  Atlas v3 (`data/sdf/atlas`, OSRF DRCSim `atlas_description` v3,
  Apache-2.0) stays, with the Apache license, a NOTICE entry and a note on
  DART's changes (relocated meshes, SDF conversion, soft-foot variants,
  edited textures). The Unitree G1 (BSD-3-Clause, pinned online bundle)
  becomes the primary humanoid for new demos and docs. Porting the Atlas
  controllers (puppet, SIMBICON, soft-foot) and the robot benchmark to G1 is
  a separate later project; Atlas stays until G1's demos match.

- **DD-24. The 7.0 online model catalog: G1, UR5e and FR3.** Next to the
  Unitree G1 (DD-23) and UR5e (DD-22), the Franka FR3 (Apache-2.0) joins the
  pinned online catalog as the modern research arm for manipulation and IK
  demos and the WAM's eventual successor. The Unitree Go2 (BSD-3-Clause)
  follows when a quadruped demo exists; PAL TIAGo and hands only for real
  demos. Each bundle carries its expanded description, meshes, pinned
  source and generator revisions, licenses and notices, byte counts and
  SHA-256 hashes, and passes per-model qualification (known-pose kinematics,
  inertia and limit checks, inspected rendering).

- **DD-25. Fix the MJCF limit and inertia bugs first.** The MJCF parser
  never reads a joint's `limited` attribute (limits are never enforced) and
  copies only mass and center of mass from `<inertial>` (declared
  `diaginertia` and `fullinertia` are ignored). Both are fixed now in a
  separate labeled bug-fix PR (DEP-36), before DD-15's fixture swap, so the
  new fixtures are baselined once on correct dynamics. Inferring inertia
  from all geoms and the wider MuJoCo Menagerie features stay separate.

- **DD-26. Fix the Bullet detector leak now.** LeakSanitizer reported a
  pre-existing leak of about 4.4 MB (35 allocations) through an ownership
  cycle between DART's Bullet collision detector and its filter group. A
  separate bug-fix PR (DEP-37) reproduces it, breaks the cycle at its owner,
  adds a LeakSanitizer regression test and runs the Gazebo lanes.

- **DD-27. Backport the voxel-ray fix to release-6.20.** Once #3859
  merges on `main`, a `release-6.20` PR through `dart-backport-pr` carries
  the same fix and tests, a 6.20.3 changelog entry and release-6.20's
  Gazebo lanes (6.20.2 was already packaged).

- **DD-28. Retire the urdfdom-typed installed headers before F (DEP-17
  PR4).** `dart/utils/urdf/IncludeUrdf.hpp` and `urdf_world_parser.hpp`
  (`dart::utils::urdf_parsing::parseWorldURDF`) move to a non-installed
  `detail/`; urdfdom links privately in shared builds and is exported only
  for static builds; a breaking-change entry points users to
  `DartLoader::parseWorld` and `parseWorldString`. gz-physics and gz-sim do
  not use either header.

- **DD-29. libpng keeps its API-minimum floor, 1.6.0 (a DD-1
  exception).** No RHEL-based manylinux image ships Ubuntu 24.04's 1.6.43;
  DART's Linux wheels keep AlmaLinux 8's Red Hat-patched libpng, which
  carries the 2025-26 fixes that reach DART's decode path, and DART owns no
  libpng updates. libpng stays a private dependency of `dart-gui`, and #3860
  caps decode sizes.

- **DD-30. Two more shared mechanical steps.** Besides the toolchain
  floors, two PRs may each touch many dependencies: DEP-25 (the umbrella
  `DART_USE_SYSTEM_DEPENDENCIES` switch, OFF for standalone builds and ON in
  pixi and package managers; an offline fail-fast that names the switch;
  CMP0170; wired at every existing fetch site) and DEP-8(f) (the wheel
  image's sources as SHA-256-checked release archives, with no version
  changes). New fetch paths stay one dependency per PR, starting with
  tinyxml2. The plan README's constraints are updated to match.

- **DD-31. Linux wheels move to urdfdom 6.0.1 (DEP-17 PR3).** The wheel
  image builds urdfdom 6.0.1 with urdfdom_headers 3.0.x and rebuilds its
  sdformat on top. That removes urdfdom 4.0.0-5.0.4's use of URDF text as a
  printf format string from the wheels (they build an unfortified
  console_bridge) and lets them read URDF 1.1 and 1.2, a labeled additive
  change. An acceptance table compares 4.0.1 and 6.0.1 so nothing that loads
  today stops loading. Source builds keep the 4.0.0 floor, with a note that
  urdfdom below 5.1 should only load trusted URDF.

- **DD-32. Backport the urdfdom 6.0.1 wheel bump to release-6.20.** After
  main's DEP-17 PR3 merges, a `release-6.20` PR through `dart-backport-pr`
  moves 6.20's wheels to urdfdom 6.0.1 and urdfdom_headers 3.0.x with the
  same acceptance table, for 6.20.3, with a labeled additive changelog entry
  (URDF 1.1 and 1.2). 6.20's pixi pins stay as they are.

- **DD-33. Standing approval for this plan's publishing.** Without asking
  each time: open each finished packet as a draft PR after its two clean
  reviews (template, DART 7.0.0 milestone, cross-links, `gh stack` for
  stacks); push its changelog-link commit; push CI fixes, review fixes and
  project-home updates to this plan's draft PRs, each after merging the
  latest `main` and passing the local-path check, lint and the reviews.
  Still asked every time: marking a PR ready for review, merging,
  backports, anything outside this plan, and replies to human reviewers'
  comments. Replies to AI-generated review comments (bot users) are never
  sent, with or without approval, as the root `AGENTS.md` requires; approved
  local fixes for them are made silently and re-review is requested only
  after an approved push. DD-33's authorization is the 2026-10-19 plan
  approval; it adds no reply permission.

- **DD-34. The standing approval covers ready PRs for merges and fixes.**
  DD-33 extends to this plan's PRs that the maintainer marked ready, for
  merging `main`, resolving conflicts and fixing CI only, with no other
  content changes; each push follows lint, the local-path check and the
  relevant tests.

- **DD-35. Drafts leave draft as soon as they pass the gate (2026-10-24).**
  The maintainer asked that the plan's PRs be marked ready for human review
  as soon as they are ready: the agent is done updating them, every required
  CI job passes, no review comment is left to address, and the Codex and
  GitHub review bots have passed. Without asking each time: post one
  top-level `@codex review` on each finished draft, fix bot findings
  silently (pushes under DD-33), request a new `@codex review` for each new
  head, and run `gh pr ready` once the
  [PR Lifecycle](../../onboarding/ai-tools.md#pr-lifecycle) step-2 gate passes
  on the current head. Merging (which needs the `maintainer-approved` label),
  backports and replies to human reviewers are still asked.

- **DD-36. Fetch-first follows the loaders (2026-10-24).** The maintainer
  chose option 1 of the open question below: tinyxml2, urdfdom and assimp are
  fetched only when nothing installed in the build already loads them;
  otherwise DART uses the installed copy, so one process never holds two of
  their ABIs. Fetch-first stays unconditional for ODE, Bullet, FCL/libccd and
  octomap, and FCL/libccd is the next fetch prototype. The tinyxml2
  prototype stays unmerged on its local branch: DART requires urdfdom, which
  loads tinyxml2, so a tinyxml2 fetch only applies once urdfdom can be
  fetched too.

- **DD-37. macOS and Windows wheels build their own minimal libcurl
  (2026-10-24).** The maintainer chose option 1 of the DEP-19 question: those
  wheels ship `ModelResourceRetriever` with a minimal libcurl (HTTP and
  HTTPS) and OpenSSL built from pinned sources, as the Linux image does,
  instead of conda-forge's closure (about +16 to 18 MB per wheel) or no
  retriever.

- **DD-38. Standalone GUI builds use DART's private Filament SDK by default
  (2026-10-25).** The maintainer chose option 1 of the DEP-20(c) question.
  `DART_USE_SYSTEM_FILAMENT` takes its default from
  `DART_USE_SYSTEM_DEPENDENCIES` where DART has a private SDK. Linux builds
  that SDK from source; macOS arm64 and 64-bit Windows use prebuilt
  archives. Other platforms keep the installed SDK. When the selected source cannot
  work, configure stops with a message that names the fix; it never drops
  the GUI quietly. This is a labeled default change with a changelog
  fragment.

## Second decision batch (2026-10-19, answered one by one in DD-10 to DD-34)

The balance review under DD-9 ([balance-review.md](balance-review.md)) asks
five questions and lists the approvals its packet plan needs. The
recommended option comes first.

- **Q1. fmt (DD-4).** (a) DEP-15 as recorded: a standard-library formatter
  without byte parity with fmt. (b) Keep fmt hidden in `Logging.cpp` until
  `std::format` is allowed, which re-opens DD-4.
- **Q2. C++ asset fetching in the 7.0 API (DD-6).** (a) Delete the
  unreleased `utils-assets` component before F; a standard-library Python
  helper fetches sample data and later becomes `dart.data`. (b) Keep it and
  add DEP-2. Also: should D22's "a fetcher" fetch the humanoid bundles for
  wheel users?
- **Q3. SDF packaging (DEP-18).** (a) An optional part of `dart-io` that
  warns and skips SDF without sdformat. (b) The same, but a missing sdformat
  stops configure unless `DART_SKIP_SDFORMAT=ON`. (c) A separate
  `dart-io-sdf` library.
- **Q4. Fetch first for the libraries Gazebo also loads (re-opens DD-8).**
  (a) Fetch first only for ImGui, nanobind, GoogleTest, Benchmark and Tracy
  (and the prebuilt Filament SDK where the DEP-20 spike agrees); system only
  for Eigen, libccd, octomap, FCL, assimp, tinyxml2, the urdfdom group,
  sdformat, GLFW 3.3.10, libpng and libjpeg-turbo. (b) As (a), plus a
  fetched static GLFW. (c) As (a), plus find-or-fetch fallbacks in 7.x.
- **Q5. The Dear ImGui patch.** (a) Drop it in 7.0 once DEP-31 fixes DART's
  own font path. (b) Keep its guards, applied without Git.
- **Approvals:** DEP-17 PR4 (the urdfdom-typed headers leave the installed
  set before F); DEP-12(b) keeps libpng at its API-minimum floor (a DD-1
  exception); DEP-25 and DEP-8(f) as shared mechanical steps; DEP-19 PR2
  removes curl and OpenSSL together; the release-6.20 backport of DEP-17
  PR3's wheel bump; the labeled changes listed in the review's §4.

## Open questions (2026-10-24)

Asked one at a time, in this order.

- **Fetch-first for libraries that DART's own dependencies load.**
  Answered 2026-10-24: option 1 (DD-36). The
  tinyxml2 prototype ([verification](verification.md#tinyxml2-fetch-prototype-2026-10-20))
  shows that a fetched tinyxml2 is either the ABI that urdfdom and sdformat
  already load, so it is redundant, or a second ABI in one process, so it
  must be rejected; on Ubuntu 24.04, OSRF's sdformat 14 links apt's urdfdom
  4 and tinyxml2 10. Options: (1, recommended) tinyxml2, urdfdom and assimp
  are fetched only when nothing installed in the build already loads them,
  and fetch-first stays unconditional for ODE, Bullet, FCL/libccd and
  octomap, with FCL/libccd as the next fetch prototype; (2) literal
  fetch-first for tinyxml2: merge the prototype, and fail configure where an
  installed urdfdom or sdformat loads another tinyxml2; (3) fetch the whole
  chain (urdfdom with console_bridge and urdfdom_headers, and sdformat with
  its gz libraries when SDF is on), revising DD-11.
- **DEP-19 on macOS and Windows** (asked 2026-10-20, in DD-13's note).
  Answered 2026-10-24: option 1 (DD-37).
  Linux wheels ship the retriever (#3931). conda-forge's libcurl closure
  adds about 18.2 MB (macOS) and 16.5 MB (Windows) per compressed wheel,
  against DD-13's estimate of about 2 MB. Options: (1, recommended) build a
  minimal libcurl and OpenSSL from the pinned sources, as the Linux image
  does; (2) accept conda-forge's closure; (3) keep the retriever off on
  those wheels.
- **DEP-20(c): the Filament source when no SDK is installed.** Asked and
  answered 2026-10-25: option 1 (DD-38). The DEP-20(a) spike
  ([verification](verification.md#wave-8-2026-10-24-to-2026-10-25)) built
  and tested the GUI and dartpy on stock Ubuntu 24.04 with DART's fallbacks
  after the worker installed `libgl-dev`. Options:
  1. (recommended) Fetch first where DART has a private SDK.
  2. Use an installed SDK first, else build the private one.
  3. Keep failing, and improve only the message and docs.
- **The Tracy viewer in Pixi** (DEP-22). Resolved 2026-10-24: the maintainer
  merged #3939 as proposed, so the `profile` environment fetches the Tracy
  0.14.1 client and the viewer is installed separately. conda-forge's
  `tracy-profiler-client` 0.14.1 is an empty stub (its recipe does not pass
  `TRACY_ENABLE`), and `tracy-profiler-gui` 0.14.1 needs GLFW 3.5.1 against
  the GUI's 3.4 pin.

## Original questions (2026-10-18)

Asked as one batch; the answers are above.

- **Q1. C++ standard.** Recommended: DART's own targets build as C++20 while
  exported targets keep `cxx_std_17` and installed headers stay C++17-clean
  (the `ContactSurface.hpp` include tree stays parseable at gz-physics'
  `-std=c++14` probe), with the GCC-16-changed C++20 library components
  banned in DART code until the GCC floor reaches 16. Alternatives: export
  C++20 to consumers (gz-physics' plugin silently becomes C++20, gz-physics'
  Bazel and C++17 pkg-config users break, and GCC < 16's experimental C++20
  library ABI enters DART's ABI), or stay on C++17. Evidence:
  [verification.md](verification.md) S1, S3c, S2 and S2b.
- **Q2. Removals in 7.0.** Recommended: all four, before API freeze `F`:
  logging (spdlog and fmt, DEP-14 to DEP-16), URDF (the urdfdom stack,
  DEP-17), asset networking (libcurl and OpenSSL, DEP-19), and SDF in its own
  library (DEP-18). Unselected items keep today's dependency at the 24.04
  floor.
- **Q3. Small public-surface cleanups for 7.0.** Recommended: approve all.
  The items: drop the exported Boost define; drop the always-1 `ASSIMP_*`
  config macros; trim transitive includes (FCL from `VoxelGridShape.hpp`,
  assimp from `BulletCollisionDetector.hpp`, forward-declared octomap types);
  7.0 deprecation attributes for 8.0 removal (`getAssimpMesh`,
  `AssimpInputResourceAdaptor`, fcl `createMesh(aiScene)`, the namespace-Eigen
  aliases); drop the dartpy `aiScene` overloads; an assimp importer allowlist
  in Linux wheels; retired switches kept as no-ops through 7.x; Bullet and ODE
  usage requirements moved from core to their components. No gz-physics,
  gz-sim or dartpy use was found for any of them.
  Note (2026-10-24): gz-physics uses `Eigen::Vector6d` (`JointFeatures.cc`)
  and `Eigen::Matrix6d` (`AddedMassFeatures_TEST.cc`) in Harmonic, Ionic and
  Jetty; the lexical API inventory misses both. The Eigen-alias deprecation
  therefore covers only names gz does not use (`EIGEN_V_VEC3D`,
  `EIGEN_VV_VEC3D`, `Eigen::aligned_vector`, `Eigen::aligned_map`,
  `Eigen::make_aligned_shared`), and `Vector6d` and `Matrix6d` stay, as the
  [8.0 design record](../../design/dart8_dependencies.md) keeps them.
- **Q4. Fetch policy.** Recommended: keep per-dependency defaults (fetch
  first only where an exact version matters: the ImGui docking release,
  nanobind, GoogleTest, Benchmark, Tracy; system first with a fetch fallback
  elsewhere, adding GLFW 3.4), and give every fetched dependency a
  `DART_USE_SYSTEM_<X>` switch that fails fast for offline packagers.

## Defaults taken without a question

Object to change any of these.

- FCL stays a core dependency and the C++ default detector in 7.0 (PLAN-620,
  D13). assimp, Bullet and ODE stay; their removal is the 8.0 track (DEP-24).
- Compiler floors: GCC 13.2, Clang 18 (20 only for the private Filament SDK),
  AppleClang 17 with Xcode 16.3+, and MSVC 19.32 under the recommended C++
  option. CMake 3.28; the installed consumer fallback stays at 3.22.1.
- Support tiers: Tier 1 (core and `dart-io` from noble apt plus OSRF
  sdformat) is enforced by a nightly apt lane; Tier 2 (GUI and dartpy on
  noble through DART's fallbacks) follows the DEP-20 spike.
- Bullet keeps the 3.05 detection floor from #3829 (3.06 and 3.07 report
  3.05; no code needs a newer Bullet, and Fedora and EPEL ship 3.08). The
  Eigen, FCL, libccd, octomap and ODE floors do not change.
- #3829 (the maintainer's open packaging PR) is independent: if it merges
  first, DEP-12 raises the floors on top of it.
- libpng and libjpeg-turbo stay system libraries with decode caps; swapping
  them for vendored codecs is deferred.
- Dear ImGui was to stay bundled; DD-14 superseded that with a fetched,
  hash-pinned archive (DEP-30). Filament stays
  exactly pinned at 1.77.3, now enforced in CMake, and moves to 1.78.x in 7.x.
- dartpy stays one all-in-one module that requires the GUI and IO (the
  recorded 7.0 decision).
- The default pixi environment stays on Eigen 3.4.0, with a separate Eigen 5
  lane.
- Dependency packets run alongside the M1 critical path, decision-free ones
  first; step-path hunks follow the PLAN-624 program-owned merge order, each
  re-qualified on current `main`.
- gz-sim's `INTEGRATION_reset_sensors` failure is external (a use-after-free
  in the test); the DART 7 program session owns its record and the upstream
  report.
- `dart-simd` (about 15K lines of installed headers with no production
  consumer) and the C++17-era compatibility headers are left for a separate
  decision.
