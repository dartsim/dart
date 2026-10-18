# Verification

Evidence per packet. A packet is done only after its gates pass and two clean
review passes are recorded on its final state. Every claim records its
revision, command, date and scope.

## Shape/query capability draft (2026-10-13)

[The matrix draft](shape-query-capabilities.md) inventories public shapes and
separates conversion from collision, distance/nearest points and raycast at
`4ff47d438c3695f2ce63ef702949890e48258b52`. Its implementation routes and existing
test contracts are source evidence; no new runtime qualification is claimed.
Earlier fixed-binary collision diagnostics lack exact commit/build attribution
and do not qualify this pin or other query operations. The published matrix and
M1b strict rejection still gate the Python default change; C++ keeps FCL in 7.0.

## Planning (2026-10-10 to 2026-10-11)

What changed: decisions D0–D32 and D31a (see [decisions.md](decisions.md)) and
the [design doc](../../design/dart7_python_api.md).

Evidence:
- Read-only research over the DART sources, the archived DART 7/8 prototypes,
  and gz-physics usage.
- Three independent reviews of the plan: a whole-plan review, a numerics and
  physics review, and a three-lens critique covering consistency, compatibility
  and release, and feasibility. Their verified findings became the design
  contracts.
- Spot checks on 2026-10-11, by reading the named sources at DART main
  `3d31d5687` and gz-physics `gz-physics9` at `fd0b1a25`:
  - gz-physics requests `DART 6.10 CONFIG` with legacy components
    (`CMakeLists.txt`, `gz_find_package(DART ...)`). DART writes
    `SameMajorVersion` in its top-level `CMakeLists.txt`.
  - gz-physics uses whole-multibody position getters and setters plus the reset
    functions, reads velocities through per-joint accessors, and overrides
    collision-detector and contact-surface virtuals. It forward-declares only
    `dart::collision::Contact`, none of the classes planned for renaming.
  - Every fixed-size helper in `dart/utils/XmlHelpers.cpp` asserts the element
    count and then writes every parsed value. In release builds the assert is
    compiled out, so an over-long value writes past a stack object.
  - nanobind 3.1's `gil_scoped_acquire` exposes `is_valid()`, because attaching
    can fail during finalization on Python 3.15 and newer. dartpy's deleter and
    callback sites do not check it.
  - `World::clone()` keeps fresh defaults for filters, contact-surface handlers
    and manual constraints.
  - The native collision detector has no heightmap conversion; it warns once
    and skips unsupported shapes.
  - The repository's AI-policy checks hard-code the DART 6 profile and 6.x
    version patterns.

Known gaps:
- The third-party research is private by design (D25).
- gz-physics facts are re-verified by machine-checked inventories in WP-7.0.6.

## M0 packets

### WP-7.0.1 Release-line and AI-policy rollover

- **Pull request:** [#3697](https://github.com/dartsim/dart/pull/3697). It ships
  with WP-7.0.2 as one PR: the project-home commit, then the rollover commit.
  CI checks each PR's milestone against the release target that the rollover
  sets, and the rollover cites the project home.
- **Scope:**
  - `AGENTS.md` compatibility rules for the DART 7 line, plus the AI
    principles, north star and `docs/ai/branch-profile.json` profile `dart7`;
  - the `.codex/agents` profile descriptions;
  - the release-management target row `main | Development | 7.0.0`;
  - CHANGELOG: a `## DART 7` heading that holds the former 6.21.0 entries;
  - release-neutral wording across `docs/ai/`, the docs index and owner docs
    (including `docs/design/AGENTS.md` and `docs/dev_tasks/README.md`),
    onboarding, and the `.claude/` skills and commands, with regenerated
    adapters;
  - "DART 6.21" facts retargeted to 7.0: build-system components, Python
    bindings, profiling, and four tutorial introductions;
  - validators: release targets, changelog headings, release references,
    scenario neutrality and performance release tags accept any major version,
    with tests.
- **Non-goals:**
  - the GUI-backend profile flip, which belongs to M6a;
  - version bumps beyond the release-target row (WP-7.0.6);
  - reconciling the stale 6.20.0 changelog section (WP-7.0.1b);
  - dashboard plan content (WP-7.0.2).
- **Gates:**
  - `pixi run sync-ai-commands`, `pixi run check-ai-commands`;
  - `pixi run check-ai-infra`, `pixi run test-ai-infra`;
  - `pixi run exercise-agent-scenarios`;
  - the performance-harness tests
    (`pixi run python -I scripts/run_pytest.py tests/test_perf_regression.py`);
  - `pixi run lint`, `pixi run check-lint`.
- **Milestones:** done on 2026-10-11 at the maintainer's request. The archived
  milestone #18 became "DART 7.0.0 (archived 2015-2025)", and #105 ("DART
  6.21.0") became "DART 7.0.0".
- **Review passes:**
  - The first review found two issues: the rollover cited PLAN-624 before it
    existed (now stacked on WP-7.0.2), and the identity-variation test never
    varied its input (now mutation-checked). Both were fixed.
  - A review of the stacked commit was clean.
  - A focused review found that the generalized patterns rejected dependency
    versions and simulation parameters, that `docs/ai/README.md` still
    preserved OSG on main, and that the binder diagnostic named DART 6.21. The
    patterns now count bare numbers only from DART's current majors up, with
    regression tests; OSG preservation is scoped to release branches; the
    diagnostic says 7.0.
- **Evidence:** `pixi run test-ai-infra` (1688 passed),
  `pixi run exercise-agent-scenarios`, `pixi run check-lint` and
  `pixi run lint` pass on the stacked commit.
- **Status:** done; merged in [#3697](https://github.com/dartsim/dart/pull/3697).

### WP-7.0.1b Changelog reconciliation

- **Objective:** main's stale "DART 6.20.0 (Unreleased)" section matches the
  released 6.20.0 section on `release-6.20`.
- **Status:** done on 2026-10-11 by the maintainer's merge of `release-6.20`
  into main (`e029bcc17`, v6.20.0), which also moved the package version to
  6.20.0.

### WP-7.0.2 Project home and design doc

- **Pull request:** inside WP-7.0.1's [#3697](https://github.com/dartsim/dart/pull/3697).
- **Scope:**
  - this folder;
  - `docs/design/dart7_python_api.md` and its index row;
  - PLAN-624 on the dashboard, plus the PLAN-620/622/623 updates from D13 and
    D30, and the PLAN-622 owner docs retargeted to 7.0.
- **Gates:** `pixi run lint`, `pixi run check-lint`,
  `scripts/check_local_paths.py`.
- **Review pass 1:**
  - A whole-change review found 6 issues: stale 6.21 targets in the PLAN-622
    owner docs, undefined M1 sub-milestones, snapshot portability across
    Worlds, the gz usage summary, the forward-declaration claim, and the
    heightmap wording. All were fixed.
  - A plan-conformance review found 14 major and 16 minor issues, mostly
    approved contracts missing from the public docs. They were added to the
    README (target shape, M1 and M5 order, release train, default-detector
    switch, M6a interface, gates) and the design doc (mode contract, API
    surface, motor rows, determinism, concurrency, snapshots, batching,
    transforms, rendering, packaging). The XML-attribute scope moved to its
    own packet, WP-7.0.3b.
  - A commit review found an unsupported performance-gate invocation (`local
    --nightly` measures only the head; the comparison form is
    `--rows nightly`) and a shim assertion that rejected the permitted FCL
    default. Both were fixed.
- **Status:** done; merged in [#3697](https://github.com/dartsim/dart/pull/3697).

### WP-7.0.3 Fixed-size XML value parsing

- **Pull request:** [#3698](https://github.com/dartsim/dart/pull/3698).
- **Objective:** wrong element counts raise a clear error before anything is
  written, routed through one shared helper.
- **Scope:** `dart/utils/XmlHelpers.cpp`, its unit tests, and an SDF-level
  regression test.
- **Non-goals:** SDF pose attributes; see WP-7.0.3b.
- **Evidence:**
  - Before the fix, an AddressSanitizer build reported a stack buffer overflow
    in `toIsometry3d` for a seven-value pose.
  - After the fix: the focused XML-helper and parser tests pass, and
    `pixi run test` passes (171 of 171) on the final commit.
  - The changed translation units compile warning-free at every
    `DART_ACTIVE_LOG_LEVEL`.
- **Gates:**
  - `pixi run lint`;
  - `pixi run build`;
  - the focused XML-helper and parser tests;
  - `pixi run test`.
- **Review passes:**
  - A whole-change review found a build failure when logging is compiled out
    (an unused parameter at log levels FATAL and OFF) and a rejected valid
    input (an MJCF weld `solref` with one value). Both were fixed, with an
    MJCF regression test.
  - Two clean passes followed on the final commit: a commit review, and a
    focused review of caller cardinalities, exception contracts and log levels
    0–6.
- **Status:** done; merged in [#3698](https://github.com/dartsim/dart/pull/3698). The `release-6.20` backport is [#3717](https://github.com/dartsim/dart/pull/3717).

### WP-7.0.3b SDF pose attributes

- **Objective:** the SDF parser honors `rotation_format="quat_xyzw"` (stored as
  `wxyz`), `degrees="true"` and `relative_to`, or raises a clear unsupported
  error, instead of ignoring them.
- **Acceptance:** tests for five, six, seven and eight values, with and without
  each attribute.
- **Pull request:** [#3719](https://github.com/dartsim/dart/pull/3719), merged
  from head `a2e914f18` as `9ea8f6834`. It changes behavior, so it targets
  main only.
- **Evidence:** CI passed on `a2e914f18` (16 checks passed, 10 skipped).
  #3719's description reports that all 21 SDF, world and model files under
  `data/` (783 poses) load to byte-identical state before and after the
  change.
- **Historical review findings, resolved by the follow-up below** (Codex: two on the first commit; on
  `a2e914f18`, a clean pass before the merge, then two findings after it):
  - inertial poses keep only their translation, so a rotated inertial frame
    loads an unrotated inertia tensor;
  - the version gate accepts only SDF 1.4–1.6, so files that declare the newer
    SDF versions defining these attributes are rejected before any pose is
    read; the tests use SDF 1.6 documents;
  - soft-shape spheres, ellipsoids and cylinders ignore the parsed pose; only
    boxes apply it;
  - an empty `<pose rotation_format="quat_xyzw"/>` raises instead of reading
    as the identity.
- **Status:** done. [#3782](https://github.com/dartsim/dart/pull/3782) merged
  the four-gap follow-up with two clean independent reviews and the evidence
  recorded below.

### WP-7.0.4 GIL acquisition during finalization

- **Pull request:** [#3699](https://github.com/dartsim/dart/pull/3699).
- **Objective:**
  - deleters skip Python calls when no thread state can be attached;
  - callbacks raise a defined error instead of inventing results.
- **Scope:** the non-GUI binding sites and a shared helper under
  `python/dartpy/detail/`.
- **Non-goals:** GUI binding sites. The Filament effort applies the same rule.
- **Evidence:**
  - Python 3.14 (the default environment): `pixi run test-py` passes (723;
    the 3.15-only test is skipped).
  - Python 3.15.0, where attachment can fail: dartpy built against 3.15 passes
    the full suite (724). That includes a regression test that releases a
    pinned Python object inside a GIL-releasing binding during finalization.
    With the previous deleter its subprocess crashes with SIGSEGV; with the
    fix it passes.
  - An isolated native probe against Python 3.15.0, using the helper and the
    pin type, passed 25 subprocess runs. It covered a valid native-thread
    attach, a refused attach during finalization, the skipped release, and
    the exact error.
- **Follow-ups found** (pre-existing, separate packets):
  - the ODE collision callback cannot unwind exceptions, because ODE releases
    internal state after the callback returns;
  - cloning a skeleton with analytical IK dereferences a null skeleton, because
    the IK is cloned before the cloned body node is registered.
- **Gates:** `pixi run lint`, `pixi run build-py-dev`, `pixi run test-py`,
  plus a Python 3.15 build until CI has a 3.15 lane.
- **Review passes:**
  - A review found the guards correct, but the first tests could not detect
    the bug, and guards before pure overrides duplicated nanobind's own. The
    revision uses nanobind's cleanup guard for deleters, raises only where
    dartpy itself calls Python, and adds the discriminating test.
  - A later pass found that the test's skip missed Windows stable-ABI builds;
    it now uses the interpreter's version-specific extension suffix.
  - Two clean passes followed on the final commit: a commit review, and a
    focused review of the guards, the remaining owners and callbacks, and the
    skip condition.
- **Status:** done; merged in [#3699](https://github.com/dartsim/dart/pull/3699).

### WP-7.0.5 Performance baseline

- **Objective:** measure the core refactors against `dart7-base`, the last main
  revision before the first M1 PR.
- **Scope:**
  - add `--worlds K` to `tools/perf/portable_step_bench.cpp`, using 6.x C++ APIs
    so both arms run it, with detectors pinned;
  - per PR: K = 1 and 64, with the DART and ODE detectors;
  - nightly: K = 1024, plus FCL, Bullet and the closed-loop joint-friction row;
  - a dartpy loop benchmark written in 6.x spelling, which the M5 codemod later
    converts (doubling as a codemod test);
  - rows measure stepping plus reading transforms and contacts;
  - nightly Gazebo-preset sleeping rows, which cover the custom collision-filter
    replay path Gazebo's default step uses.
- **Added:** `--worlds K` steps K independent Worlds in sequence per outer
  step, with two generated scenes: a box resting on a floor (`contact`) and a
  four-bar linkage with joint friction and a ball-joint closure
  (`closed-loop-friction`). Existing rows keep their settings and measurement
  boundary, and their guards match the previous driver exactly. Editing the
  shared driver changes the `gzb` and `robot` input hashes in both arms, so the
  PR names those rows in a `Rebaseline-Rationale:` line. The dartpy loop,
  `pixi run perf-dartpy-loop`, times stepping plus transform and contact reads
  with pyperf. [Profiling](../../onboarding/profiling.md#many-world-step-loops)
  owns how to run both.
- **Rows and cost:** measured on 2026-10-11 at main `e629dd3de` on the local
  performance host. Cost is the time of two serial measurements of a row,
  which estimates its incremental two-arm CI time; the per-PR rows total 25 s.

  | Row | Tier | Ir per outer step | Cost (s) |
  | --- | --- | ---: | ---: |
  | `w1/dart` | Per PR | 76,560 | 5.3 |
  | `w1/ode` | Per PR | 76,401 | 5.3 |
  | `w64/dart` | Per PR | 4,898,253 | 7.2 |
  | `w64/ode` | Per PR | 4,893,465 | 7.0 |
  | `w1/fcl` | Nightly | 76,629 | 5.2 |
  | `w1/bullet` | Nightly | 71,858 | 5.6 |
  | `w64/fcl` | Nightly | 4,902,497 | 6.9 |
  | `w64/bullet` | Nightly | 4,597,060 | 29.8 |
  | `w1024/dart` | Nightly | 78,371,096 | 39.7 |
  | `w1024/ode` | Nightly | 78,295,213 | 83.1 |
  | `w1024/fcl` | Nightly | 78,438,333 | 35.4 |
  | `w1024/bullet` | Nightly | 73,552,338 | 499.5 |
  | `closed-loop-friction/dart` | Nightly | 72,141 | 9.0 |

  All 13 rows qualified under the seven heap-layout perturbations, with 4, 256
  and 4096 contacts for K = 1, 64 and 1024, and a loop-closure error of
  2e-12 m. The dartpy loop measured 114 microseconds per step and read. These
  are head-only smoke numbers, not a baseline.
- **Baseline procedure:** right before the M1a PR, record the last main SHA as
  `dart7-base`. It is a local git ref on the local performance host, not a
  pushed tag. This project home records its SHA, the host and the per-row
  records, measured there with the full nightly harness:
  `pixi run python scripts/perf_regression.py local --smoke --nightly --head <sha>`.
  Each milestone exit runs
  `pixi run python scripts/perf_regression.py local --base dart7-base --head HEAD --rows nightly`
  on the same host and compares each detector against its own baseline.
- **Gates:** [#3720](https://github.com/dartsim/dart/pull/3720) merged from
  head `adf9a57f6` as `fc823113d` while CI was still running; every check on
  that head passed afterwards (20 passed, 10 skipped), including Perf A/B.
- **Review passes:** Codex found no major issues on `bd4fe2625` and again on
  the merged head `adf9a57f6`.
- **Gazebo-preset rows:** nightly `S7-t1` and `S7-t4` run
  `contact_benchmark --gz-preset` on the settled `3k_shapes.sdf` with the
  DART, Bullet and ODE detectors at 1 and 4 threads. The preset installs a
  `BodyNodeCollisionFilter` subclass, as gz-physics does, so bodies sleep
  through `World::step`'s custom-filter decision replay, the step path
  gz-physics uses by default. The rows reuse existing driver flags, so no
  row's input hash changes. Measured on 2026-10-12 at main `722f28ddd` with
  the rows added, on the local performance host:

  | Detector | Hash | Resting | Cap hit | Recorder solves | Process time (s) |
  | --- | --- | --- | --- | ---: | ---: |
  | DART | `0xa6f5233706594236` | 3003/3003 | no | 4 | 2.5–2.7 |
  | Bullet | `0xb02e822fcc3c1d04` | 2724/3003 | no | 3,000 | 150–232 |
  | ODE | `0x53e83625f00d4813` | 3003/3003 | yes | 503 | 19–23 |

  Two smoke runs gave identical guards, and each detector's 4-thread guards
  equal its 1-thread guards. ODE's contact demand of 13,013 exceeds the
  10,000-contact cap that gz-physics sets; no row has starved pairs or sunk
  bodies. Recorder solves count the 1-thread steps, of 3,000, that solve
  through the constraint stage's decision recorder; the other steps take the
  all-resting fast path, so only the Bullet and ODE rows exercise the
  recorder at length. With `DART_CUSTOM_FILTER_SLEEPING=0`, which turns replay
  off, the 1-thread DART and ODE rows rest no body and their hashes change to
  `0x29469c863f538cf2` and `0x7de27186a1a62f97`. The S7 oracle now marks any
  row with zero final resting bodies broken, and marks both thread rows
  broken when a detector's guards differ within one record. Thus a head-only
  nightly smoke run rejects those replay-off results without a baseline.
  Focused regression tests cover zero resting bodies, mismatched guards and
  passing guards for all three detectors. This is a text-only harness change;
  rendered frames cannot prove the row status or guard-parity verdict.
  Changelog decision: no entry required; the change fixes internal CI guard
  enforcement without changing released library behavior or user commands.
  Process time is one whole driver run at host load 7–11; the Bullet rows
  took up to 417 s at load 60,
  within the harness's 900 s per-row timeout. Run four at a time on two
  physical cores with both SMT threads, like `--jobs 4` on the hosted
  runner's four vCPUs, and sharing those cores with other host work, the six
  rows kept their guards and the slowest, 1-thread Bullet, took 311 s. No
  hosted run has measured them yet. A row that times out makes the nightly
  verdict ERROR and skips that night's publication for every row, so confirm
  the fit with a nightly `perf.yml` dispatch on the branch, which publishes
  nothing off main, or by watching the first nightly after merge.
  Oracle follow-up validation for [#3749](https://github.com/dartsim/dart/pull/3749)
  used native revision `41d045267` after merging `origin/main`, with the
  updated harness in the worktree:
  `pixi run python scripts/perf_regression.py local --smoke --head HEAD --rows S7-t1,S7-t4`
  passed, with all six rows `ok` and identical thread guards for each detector.
  DART and ODE rested 3003/3003 bodies, and Bullet rested 2724/3003; their
  hashes match the table above. A private staging root avoided another
  worktree's nightly lock; jobs stayed at 4 and the per-row timeout at 900 s.
  `pixi run python -I scripts/run_pytest.py tests/test_perf_regression.py -q`
  passed all 878 tests. `pixi run lint` and `pixi run check-lint` passed.
  Compatibility and acceptance review passes were clean after preserving
  the existing quick `mt4` selection separately from S7 parity metadata.
- **Baseline procedure:** immediately before the first M1 PR, record the last
  main SHA as `dart7-base`, with its host and the per-row records from
  `pixi run python scripts/perf_regression.py local --smoke --nightly --head <sha>`.
  Each milestone exit runs
  `pixi run python scripts/perf_regression.py local --base dart7-base --head HEAD --rows nightly`
  and compares each detector against its own baseline.
- **Status:** harness merged in #3720/#3749; `dart7-base` is pinned and its
  full nightly smoke is complete. The record is under M1a below.

### WP-7.0.6 Compatibility harness and version plumbing

- **Scope:**
  - the gz shim test and the contact-surface header probe, as described under
    Gazebo gates in the [README](README.md#gates);
  - version plumbing: a custom version file with a 6.0 floor, and a `cmake -P`
    test that renders it for 7.0.0, 7.4.2 and 8.0.0 and asserts that 6.10 and
    6.16 are accepted while 5.x, `6.16 EXACT` and the range `6.10...<7` are
    rejected, a status
    line when a 6.x request resolves to 7.x, and a way for the Gazebo lanes to
    see a 7.x version before packaging (a version bump or a CI-only override);
  - machine-checked inventories of the gz-used accessors and mutation paths.
- **Baseline:** on 2026-10-11 at main `1b5722176`, a quick check compiled
  `dart/constraint/ContactSurface.hpp` as C++14, the dialect gz-physics probes
  with. The CTest makes this exact, with only DART and Eigen include paths.
- **WP-7.0.6a version plumbing (2026-10-11, on main `e629dd3de`):**
  - `cmake/DARTConfigVersion.cmake.in` accepts requests from 6.0 through the
    installed version and keeps `EXACT`, range bounds and the pointer-size
    check. `test_DARTConfigVersion` renders 6.20.0, 7.0.0, 7.4.2, 8.0.0 and the
    configured version, evaluates each request directly and through a real
    `find_package`, and fails when the floor or the exact rule is removed.
  - With `DART_VERSION_OVERRIDE=7.0.0`, an installed DART resolves
    `find_package(DART 6.10 CONFIG REQUIRED COMPONENTS collision-bullet
    collision-ode utils utils-urdf)` to 7.0.0 and prints the status line,
    `6.10...<7` fails, and the SONAMEs are `7.0`.
  - The Gazebo tasks take the override from main's release-target row when it
    is newer than `package.xml`. `pixi run -e gazebo test-gz-physics` passes
    with it active: 199 of 199 tests plus 4 performance tests.
  - gz-physics and gz-sim use only lower-bound version guards (6.10 and 6.13),
    which hold for 7.x. The reported engine name becomes `dartsim-7.0.0`.
- **WP-7.0.6b Gazebo gates (2026-10-11, on main `e629dd3de`):**
  - `ContactSurfaceProbe` compiles gz-physics's header-check source as C++14
    with only DART and Eigen include directories; it fails when
    `ContactSurface.hpp`'s include tree needs `std::optional`.
  - `test_GazeboShim` mirrors gz-physics main
    `4e2af8c733ac8396a9d2b4dcae1f7f6aa28c5668` and counts each override:
    detector subclasses through the factory, collision-group and ray-cast
    dispatch, `ContactSurfaceHandler` callbacks while stepping, and FCL defaults
    after every factory creator is removed. Registering the base Bullet detector
    makes it fail. Its Bullet and ODE cases run only when those components are
    built.
  - Still open: the machine-checked inventories of gz-used accessors and
    mutation paths.
- **Status:** in progress.
  [#3725](https://github.com/dartsim/dart/pull/3725) merged WP-7.0.6a and
  WP-7.0.6b; [#3729](https://github.com/dartsim/dart/pull/3729), in review,
  adds the inventories. [#3731](https://github.com/dartsim/dart/pull/3731)
  closed in favor of [#3741](https://github.com/dartsim/dart/pull/3741), merged
  as `3c4c68813`. The replacement keeps the Unix probe in C++14, accepting
  the existing compiler extensions, and runs the MSVC probe in DART's required
  C++17 dialect with its exported template-parsing options. It fixes the
  Windows failure without rewriting the installed header tree.

### M0 follow-up fixes merged on 2026-10-12

- [#3722](https://github.com/dartsim/dart/pull/3722) merged the analytical-IK
  clone and node-copy crash fixes as `13ebab5aa`. It no longer holds M0's
  exit; its `release-6.20` backport remains a next step.
- [#3750](https://github.com/dartsim/dart/pull/3750) merged the skel-parser
  detector fallback null-check fix as `039fdf497`. It no longer needs a
  parallel follow-up after M0.

#### WP-7.0.3b follow-up: pose consumers and version admission

- **Starting point (2026-10-13):** the original packet merged in #3719,
  but four review findings remain in main `e378f985f`: rotated inertial poses
  discard tensor orientation; sphere/ellipsoid/cylinder soft shapes discard
  their pose; version admission rejects the 1.9 schema defining quaternion
  and degree poses; empty poses throw instead of using the default identity.
- **Scope:** `dart/utils/sdf/SdfParser.cpp`, its integration tests, a
  reproducible model artifact and documentation. No installed-header,
  component, layout or step-path changes. Preserve old default-frame and
  identity-pose loads; corrected inertia/soft poses are labeled bug fixes.
- **Acceptance:** rotated COM-frame moments equal `R I R^T` in the link frame
  without a parallel-axis shift; every soft rest point receives its local
  pose exactly once; both loaders accept actual supported modern versions;
  empty poses default to identity after attribute validation, and invalid
  counts, attributes, quaternion norms and unsupported frames still fail.
- **Gates:** focused SDF regression, `pixi run build`, `pixi run test`,
  `pixi run test-py`, `pixi run test-gui`, `pixi run -e gazebo test-gz`,
  lint/check-lint, plus assessed Filament geometry/inertia evidence and two
  clean final reviews.
- **Regression evidence:** four new tests fail on the original parser and
  pass with the fix (`build/sdf-pose-red.log`, `build/sdf-pose-green.log`).
  Expanded cases check every soft rest point for box, sphere, ellipsoid and
  cylinder; independent off-diagonal inertia values and unchanged COM;
  empty/whitespace poses with invalid attributes; both loaders and the explicit
  1.4–1.9 version allowlist; both universal-joint axes; unknown joint links;
  modern frame-composition rejection and preserved legacy admission.
- **Completed gates:** `pixi run build`; final `pixi run test-all` after
  formatting and the final regression additions: 195/195 CTest cases and
  1,596 Python tests pass. The 26 Python skips are 24 optional `ssik` cases
  and two Python 3.15 shutdown cases; this environment is Python 3.14.
  `pixi run test-gui` with the pinned Vulkan software driver also passes:
  10/10 CTest cases and 529 Python cases, with 25 skips for the same optional
  `ssik` dependency and Python 3.15 GUI-shutdown requirement.
  `pixi run -e gazebo test-gz` exits zero: 199 non-performance plus four
  performance gz-physics CTests, the DART plugin link check, and one gz-sim
  entity-system CTest containing two GTest cases pass. This is the repository's
  patched forward gate, not the unpatched release compatibility lanes.
  The initial aggregate ended with SIGTERM before downstream tests; its log
  is retained separately and is not counted as passing.
- **Model oracle:** the durable fixture is
  `data/sdf/test/pose_frames.world`; its integration regression checks two
  skeletons, inertia, COM, the 114-point count and rest centroid; a separate
  regression checks every rest point across four soft shapes.
  An isolated C++ executable also reports the before/after values in
  `build/sdf-pose-evidence/{before,after}-oracle.json`. The old moment is
  `diag(0.04, 0.16, 0.18)` and the old rest centroid is approximately zero.
  The fixed moment is `[[0.1, -0.06, 0], [-0.06, 0.1, 0], [0, 0, 0.18]]`
  and the centroid is `(0.3, 0.3, 0.2)`, with maximum errors below
  `1.2e-16`. COM remains `(0.15, 0, 0)` and both loads contain 114 points.
- **Matched visual evidence:**
  `build/sdf-pose-evidence/{before,after}/capture_main.png`, matching capture
  sidecars, pixel-integrity verdicts and `comparison.png`. Both native
  Filament captures use Vulkan llvmpipe, 1,200 by 800 pixels, zero steps,
  body frames, inertia boxes and labels, with identical explicit cameras.
  Parent and independent reviewer opened the images: rigid geometry stays
  fixed while the inertia box rotates; the soft ellipsoid shifts and rotates
  relative to its unchanged body frame. Both integrity verdicts pass.
  These observations agree with the text oracle; they do not prove dynamics,
  numeric precision or general SDFormat frame support.
- **Comparison boundary:** the before library recompiles only the parser
  from main `e378f985f` with the current build's flags and unchanged IO objects,
  then loads it through `LD_PRELOAD`. It is a matched parser substitution,
  not a clean historical engine build. Reproduction commands and source/library
  digests are in `build/sdf-pose-before/{reproduce.py,provenance.json}`;
  loaded-library mappings verify each run. The isolated text oracle can be
  repeated with
  `pixi run python build/sdf-pose-evidence/reproduce_text_oracle.py`.
- **Reviews:** two independent post-fix source reviews are clean. The second
  also assessed both native captures and identified the required GUI gate,
  which is included above.
- **Changelog decision:** a standalone IO bug-fix entry is required and added;
  this is outside the consolidated M1 feature entry.
- **Final checks:** `pixi run lint`, `pixi run check-lint`, the local-path
  scanner and `git diff --check` pass. The complete offline PR body and
  staged evidence attachment are under `build/sdf-pose-evidence/`; screenshots
  and transient baseline/oracle files stay outside Git.
- **Principle audit:** admission and numeric evidence precede claims;
  installed headers, components, layouts and stepping paths are unchanged;
  publication text excludes private paths; runtime and visual limits remain
  explicit; existing open packet branches retain their ownership.
- **Status:** done; merged in [#3782](https://github.com/dartsim/dart/pull/3782)
  as `79a87e6ed` after implementation, verification and two clean independent
  reviews. This follow-up does not establish full SDFormat schema
  support or DART 7 program completion.

### WP-7.0.7 Scope artifacts

- **Pull request:** [#3728](https://github.com/dartsim/dart/pull/3728).
- **Scope:**
  - the dependency/order/priority refinement dated 2026-10-11; historical
    evidence dates stay;
  - API freeze `F`, declared once every 7.0 must-have, breaking and additive,
    has merged and C2 has passed; the breaking must-haves include the breaking
    parts of otherwise additive work, such as `step` rejecting `bool`
    arguments; only additive work that is not a must-have and misses `F`
    moves to 7.1;
  - C1 after the first three M2 kernel PRs and their stop rule, with M3 able to
    land before `F` without delaying any other must-have, and so `F`;
  - C2 before `F`, so release candidates carry the final render API;
  - D13 as amended on 2026-10-12: FCL stays the C++ default in 7.0, while
    dartpy's `dart.World` and the MJCF world loader default to the DART
    detector (M1f), which lands after the capability matrix and M1b and
    before `F`;
  - the maintainer's 2026-10-12 settings for M0's exit, the M1a PR shape, the
    `dart7-base` ref and the step-path Gazebo gates;
  - the M3 thresholds: at least 15% `Ir` gain on `dyn`/`robot`, improved K = 64
    wall time, and the flagged spike passing guards and the shim with `memcpy`
    snapshots at most 0.1× the gather cost; M2 stops below 10% gain from its
    first three kernel PRs;
  - the 7.0 must-have list, which summarizes everything the acceptance
    criteria and the design doc require, plus the work those requirements
    wait for, and explicit deferrals;
  - the target-shape acceptance tests, marked `xfail(strict=True)`; the
    headless-render test needs a graphics device, so `pixi run test-gui` runs
    it instead of `pixi run test-py`.
- **Evidence on 2026-10-12** (this branch, merged with main `722f28ddd`):
  - `pixi run test-py` collects the ten tests in
    `python/tests/acceptance/test_target_shape.py`, all XFAIL with no XPASS
    (688 passed, 2 skipped, 10 XFAIL), and does not collect the render test.
  - `pixi run test-gui` passes: its CTest step passes 13 of 13 tests, and its
    `pytest` step, over `python/tests/unit/gui` and `python/tests/integration`,
    collects `python/tests/unit/gui/test_target_shape_render.py` as XFAIL with
    no XPASS (262 passed, 1 skipped, 1 XFAIL).
  - With xfail markers ignored, each test fails for its intended missing
    feature. The command is
    `pixi run python -I scripts/run_pytest.py --pythonpath build/default/cpp/Release/python/dartpy --runxfail`
    with each test file:

  | Test | Observed failure |
  | --- | --- |
  | World construction keywords | `AttributeError`: no `dart.World` |
  | edit/compile | `AttributeError`: no `World.compile` |
  | loading and `World.add` | `AttributeError`: no `dart.data` |
  | joint PositionServo | `AttributeError`: no `dart.PositionServo` |
  | IMU sensor | `AttributeError`: no `dart.sensors` |
  | `step(n)` and `step_count` | `TypeError`: `step` accepts only a `bool`, not `20` |
  | `Transform` composition | `AttributeError`: no `dart.Transform` |
  | non-root link `world_transform` composed with a `Transform` | `AttributeError`: the legacy `Skeleton` has no `links` |
  | capture/restore state | `AttributeError`: no `World.capture_state` |
  | WorldBatch set/step | `AttributeError`: no `dart.WorldBatch` |
  | Renderer RGB (`test-gui`) | `AttributeError`: no `dart.render` |

  `pyproject.toml` already sets `xfail_strict = true`, and each test also
  passes `strict=True`. The contract lives only in these tests, not in
  `python/examples/`, because that directory's examples run automatically.
- **Review follow-up on 2026-10-12** (main `3c4c68813` merged locally):
  - The four Codex findings on `a2301f5fd` are addressed: M1f's MJCF
    switch belongs only in `dart.io.load_world`; constructor and MJCF loader
    acceptance check `world.collision.detector`; the batch fixture configures
    its own PositionServo with matching nonzero serial and batch targets;
    and `step(True)` must raise `TypeError`.
  - `pixi run lint`, `pixi run check-lint` and `pixi run build-py-dev` pass.
    `pixi run test-py` reports 688 passed, 2 skipped and 11 XFAIL, with no
    XPASS. `pixi run test-gui` passes 13 CTests and reports 262 passed,
    1 skipped and the render acceptance test XFAIL, with no XPASS.
  - The same `--runxfail` command above fails all 11 acceptance tests and
    the render test. The changed tests fail for missing `dart.World`,
    `dart.io`, integer-count stepping and `dart.PositionServo`, respectively;
    the render test still fails for missing `dart.render`.
  - The minimal MJCF fixture loads through the existing direct parser and
    retains FCL. Visual evidence is unnecessary for these documentation and
    acceptance-contract changes; no simulation implementation changes.
  - Changelog decision:
    - Mode: decide
    - Base evidence: `origin/main` at `3c4c68813`
    - Scope evidence: #3728 review follow-up diff
    - Decision: no entry required
    - Target section: N/A
    - Entry text: N/A
    - PR-body note: Documentation and strict-xfail acceptance coverage only;
      released library behavior is unchanged.
    - Follow-up: none
- **Unresolved-thread sweep on 2026-10-12** (main `2189f8b82` merged locally):
  - All 22 unresolved #3728 threads were checked against the merged branch:
    17 were already fixed and five required acceptance-test changes. No
    GitHub comments, thread resolutions or pushes were made.
  - The batch test now starts with zero velocity and zero gravity and requires
    positive target-driven shoulder motion. The existing native fixture stays
    exactly at zero without a target, so passive motion cannot satisfy it.
  - Separate strict-XFAIL cases cover unsupported collision heightmaps at
    compile time, GIL-free stepping, and exact checkpoint restoration and
    per-frame replay with parameter-mismatch rejection. Checkpoints use the
    native DART detector and compare serialized fields and public state, not
    native memory. M1c still owns the full contact and hidden-history oracle.
    The dynamic snapshot test reads the final `world.time` property.
  - The GIL oracle passes three native calls that release the GIL and rejects
    three native calls that hold it; both paths clean up the worker thread.
  - The focused guarded pytest command above, over the acceptance and render
    modules, reports 15 XFAIL with no XPASS. With `--runxfail`, all 15 fail for
    their intended missing APIs: the new cases reach missing `World.compile`,
    integer-count stepping and `World.capture_checkpoint`, respectively.
    These results validate the pending acceptance contracts, not their future
    simulation implementations; no visual evidence applies to this test-only
    change.
  - `pixi run test-gz-compat-tools` passes all 76 tests. No #3729 inventory
    scanner thread or scanner change is part of this #3728 sweep.
  - `pixi run test-py` passes: 1,178 passed, two skipped and 14 XFAIL, with no
    XPASS. `pixi run build`, `pixi run lint` and `pixi run check-lint` also pass.
  - Two role-separated local review passes checked the final acceptance
    assertions against the design and then checked failure paths and cleanup;
    neither found a remaining issue.
  - Changelog decision:
    - Mode: decide
    - Base evidence: `origin/main` at `2189f8b82`
    - Scope evidence: #3728 unresolved-thread sweep diff
    - Decision: no entry required
    - Target section: N/A
    - Entry text: N/A
    - PR-body note: Strict-XFAIL acceptance coverage and verification notes
      only; released behavior is unchanged.
    - Follow-up: none
- **Contact, counted-step and servo review follow-up on 2026-10-13**:
  - Merged current main through `95282f1b9`, preserving the scope/freeze gates
    alongside the landed Python demo retirement and its durable owner links.
  - Counted stepping now compares positions, velocities, time and step count
    against twenty single steps of a clone containing a moving articulated
    body. Counter-only implementations cannot satisfy this oracle.
  - PositionServo covers both effort limits and unsaturated damped and
    undamped motion. The expected impulse follows the design's compliant-row
    equation and the native inverse mass matrix; generalized constraint forces
    report effort separately from ordinary joint forces.
  - Checkpoints capture a contacting free box after two sleep-enabled steps,
    while it is still awake. Replay compares contact geometry and forces,
    generalized impulses, state and the transition to sleep, so restoring must
    preserve the partially accumulated dwell timer. The independent local
    review suggested this partial-timer case and found no other issue.
  - The scene's unmarked setup gate checks nonzero contacts, contact forces
    and generalized constraint forces, then verifies sleep. Its static ground
    is explicitly immobile. Two thousand ordinary settling steps let the
    contact solver correct impact penetration without weakening sleep rules.
  - Focused guarded pytest reports one passed setup gate and seventeen strict
    XFAILs, with no XPASS. With `--runxfail`, those seventeen cases fail for
    their intended missing APIs and the setup gate still passes.
  - `pixi run test-py` passes: 1,597 passed, 26 skipped and 17 XFAILs, with
    no XPASS. `pixi run build`, `pixi run lint`, `pixi run check-lint`,
    `pixi run check-ai-commands` and the upstream hook regression suite
    (281 passed) succeed. The contact fixture
    checks current native setup; the future actuator/checkpoint implementations
    remain unimplemented. Visual evidence is irrelevant to these pending
    acceptance-contract changes; no simulation implementation changes.
  - Changelog decision: no entry required; this follow-up changes tests and
    documentation, with no released behavior change.
- **Acceptance completeness follow-up on 2026-10-13** (`013621e441` review):
  - Batch rows now have 64 distinct shoulder targets, each compared with its
    own serial clone. The source World must remain unchanged.
  - Snapshots restore to a compatible clone after a parameter change. Adding
    a zero-DOF fixed model leaves array sizes unchanged but changes topology;
    restoring must raise and leave positions, velocities, targets and time
    unchanged.
  - Three strict-XFAIL cases require read-only copies of DOF positions,
    velocities and persistent position targets. Element writes must raise,
    and later native changes must not modify a returned copy.
  - A StepHook marker checks `every=3` across two counted calls. A separate
    busy-access marker holds a stepping hook at a worker barrier, so the
    worker's read must raise `WorldBusyError`. The GIL-progress test remains
    independent. Local review caught and removed an idle-read timing race.
  - Rendering checks floating-point metric depth at the box's front face
    (1.7 meters), integer segmentation distinct from the empty scene, and
    lookup of the corresponding scene object, alongside visible RGB output.
    M1g and M6b settle callback and export-selector signatures; the tests
    label those spellings provisional while keeping behavioral requirements.
  - An independent read-only review of all six fixes found no remaining
    issue. These remain pending API contracts, not runtime acceptance claims.
  - Final focused guarded pytest, including the render module, reports one
    passed setup gate and 23 strict XFAILs with no XPASS. `--runxfail` fails
    those 23 cases at their intended missing APIs; the setup gate passes.
    `pixi run test-py` reports 1,597 passed, 26 skipped and 22 strict XFAILs
    with no XPASS. `pixi run lint` and `pixi run check-lint` pass. Rendering
    remains in the device gate; its future API is unimplemented.
- **Named native APIs and latest-main integration on 2026-10-13** (`5930123c4` review):
  - Added bounded VelocityServo effort, changing/native-reference samples
    for the remaining three sensor classes, core Material assignment, final
    viewer entry points, typed primary-solver selection and duplicate-name
    rejection. Segmentation handles compare by node equality rather than
    Python wrapper identity. The hook-driven batch uses the actual source
    World as its serial oracle and checks that cloning never alters it.
  - The complete milestone/acceptance read also found native M5 gaps:
    standalone immutable Rotation, raising/world-only loaders, live builders
    with immutable keyword-only specifications, collection lookup/error
    contracts and native-to-Python logging. Those now have strict markers.
    Snapshot and read-only-copy coverage include persistent velocity targets.
  - The support-wrench setup was independently probed through built dartpy:
    the welded two-kilogram body's native wrench is `[0,0,0,0,0,19.62]`, then
    `[0,0,0,0,0,9.81]` after halving gravity. Existing `dart.common.warn`
    invokes C++ logging directly and currently produces no Python log record;
    the bridge marker therefore fails for the intended missing behavior.
  - Merged main through `79a87e6ed`, including #3778's Muse guard and #3782's
    SDF fix. Planning conflicts retain the approved freeze/default contracts,
    the SDF follow-up evidence, M1a and its completed baseline, and the M1b
    preparation spike. Current resume steps do not repeat completed work.
    `pixi run build` recompiled the SDF parser and passed.
  - Signature spellings not fixed by the design remain explicitly provisional
    for their M1/M5/M6 owners. Removing a marker still requires that feature
    PR's full contract tests; these representative oracles do not replace
    configuration, mutation, clone-scene, performance or wheel matrices.
  - Final merged-main validation: `pixi run test-py` reports 1,597 passed,
    26 skipped and 37 strict XFAILs with no XPASS. Focused acceptance plus
    both device-gated cases reports one passed setup gate and 39 strict
    XFAILs. With `--runxfail`, those 39 fail at their pending APIs or missing
    logging-bridge behavior; the native setup gate passes. `pixi run build`,
    `pixi run lint`, `pixi run check-lint` and the public-path checks pass.
  - Two independent read-only source reviews of the final test delta are
    clean. They checked exception scopes, analytic servo efforts, native
    sensor oracles, snapshot/control restoration, hook remapping, immutable
    values/builders, logging and bounded viewing. The independent docs pass
    also confirms the conflict resolution and the matrix's release limits.
    No simulation implementation changes or visual evidence applies to this
    pending-contract follow-up; the full graphics-device suite was not rerun.

  | Owner | Representative target-shape oracle or separate release gate |
  | --- | --- |
  | M1a / WP-7.0.5 | Completed stage/baseline evidence; core bit-identical and performance gates |
  | M1b | Compiled edit/frozen membership and owned-object mutation; edited-DOF stepping after recompilation; unsupported-shape rejection |
  | M1c | Snapshot channels/time/compatible clone and independently rebuilt World/structural rejection; exact contact/sleep and hook-state checkpoint replay; parameter/topology/solver mismatch rejection |
  | M1d | Counted-vs-serial motion, bool rejection, GIL progress, synchronized busy reads and bounded re-entrant rejection |
  | M1e / M1f | Typed nondefault primary solver; Python World and MJCF DART detector defaults |
  | M1g | Both bounded compliant servos from nonzero state and against a hard limit, persistent channels and global hook decimation |
  | M1h | IMU orientation, joint-state motion, gravity-dependent support wrench and changing contacts |
  | M1i | Distinct batch targets/serial clones, source isolation, a remapped stateless hook, scalar/indices/mask writes, selected-row snapshot broadcasting of all channels/time and per-clone parameters |
  | M2 | Compiled preparation plus allocation, numeric and performance gates in the core feature PRs |
  | M3 / M4 | Optional/gated work; they do not block the required snapshot/checkpoint contracts |
  | M5 native API | Named model/frame collections, readonly copies, Transform validation and Rotation, configurable loaders/builders/ownership, manual constraints, Collider identity/shared-node capabilities, reset lifecycle, named errors and logging |
  | M5 package/tooling | Rename inventory, private-extension/wheel audits, stub-drift/new-Any/pyright checks and codemod round trips in their dedicated PRs |
  | M6b | RGB color precedence, asymmetric camera axes, metric depth, multiple-object node segmentation lookup, Material, passive sync, bounded managed viewing and legacy GUI import removal |
  | M7 / WP-7.0.6 / WP-7.0.8 | Fresh RC-wheel acceptance/slim import, Gazebo compatibility and separately owned maintenance wheels |

  Required qualification beyond these M0 representative smoke tests remains
  with the feature PR that removes each marker: M1c must test checkpoint
  magic/version metadata and persisted earlier-7.y formats; M1h/M1i must test
  nonzero seeded sensor-noise replay; M5 must test defined errors from retained
  capability handles after aspect detachment; M6b must test cameras following
  moving attached frames. Those APIs and serialized formats are settled by
  their owners, rather than invented by this planning scaffold.

- **Loader/default scope and roadmap follow-up on 2026-10-13** (`c04a253b4` review):
  - The dashboard and critical-path summary now start with the remaining M1
    seams instead of repeating the completed baseline pin and M1a.
  - The existing unmarked direct MJCF parse test requires FCL. This protects
    the C++ parser/direct binding while the strict future Python-wrapper
    marker separately requires DART.
  - A new strict loader marker requires a two-model SDF file to raise without
    `model=`. Selecting each name returns the corresponding one-link or
    two-link model and proves detached ownership by adding it to a World.
    The exact fixed-joint fixture was independently loaded through the built
    native SDF parser, confirming both model names and body-name order.
  - Both independent read-only reviews of these three fixes are clean.
    Focused acceptance, GUI markers and the direct MJCF regression report
    two passed native gates and 40 strict XFAILs with no XPASS. With
    `--runxfail`, the 40 intended pending-feature failures remain and both
    native gates pass. The rebuilt native `test_SdfParser` also passes.
    Full `pixi run test-py` reports 1,597 passed, 26 skipped and 38 strict
    XFAILs with no XPASS. Lint, check-lint and public-path checks pass.
- **Snapshot parameter and solver-observer follow-up** (`740426529` review):
  - Restoring dynamic state into the differently configured clone must leave
    its gravity unchanged. This rejects restoration that overwrites parameters.
  - The typed primary-solver oracle uses an explicitly provisional snake_case
    observer on `world.solver`, so it does not require a legacy camelCase API.
  - Focused guarded pytest still reports two native passes and 40 strict
    XFAILs; `--runxfail` reports 40 intended failures and two native passes.
    Lint, check-lint and public-path checks pass. The last full Python run
    remains `740426529` (1,597 passed, 26 skipped, 38 strict XFAILs); these
    assertion-only refinements do not change native code or fixture counts.
- **Representative core lifecycle follow-up** (`2711784c9` review):
  - The frozen-mode gate also tries owned `add_link`, requires rejection
    without changing links/DOFs, and verifies that `edit()` permits that change.
    The callable and specification resolve before the rejection context.
  - A same-thread hook attempts one nested step, bounding recursion in broken
    implementations. Rejection must preserve counters/time/state, and the
    original outer step must complete exactly once.
  - The reset gate steps a moving robot alongside the contact fixture, then
    requires clocks, contact history and constraint impulses to clear while
    positions, velocities and membership are preserved. Motion resumes after
    reset. These are existing native reset semantics, not a new initial-state
    reset policy. The exact fixture/assertions passed a native preflight using
    existing time/frame/membership getters in place of the final properties.
  - Focused guarded pytest reports two native passes and 42 strict XFAILs;
    `--runxfail` reports 42 intended pending-feature failures and two native
    passes. Lint, check-lint and public-path checks pass. The last full Python
    run remains `740426529`; no native implementation changed in this follow-up.
    These additional representative checks still leave full mutation,
    concurrency, clone-scene and release matrices with their feature owners.
- **Topology, batch and native capability follow-up** (`7fec2595a` review):
  - The edited model remains attached through recompilation; stepping with
    only the new DOF moving proves the new topology participates in execution.
    Removal occurs in a separate edit.
  - A device-free gate first resolves the replacement viewer names, then
    requires both the legacy `gui` attribute and `dartpy.gui` import to be
    absent. The missing-module error must name `dartpy.gui` itself.
  - Batch snapshots restore noncontiguous rows of distinct positions and
    velocities while leaving the other rows and source World unchanged.
    Setup assertions reject ignored row assignments. Full control-channel
    and clock broadcasting remains required of the M1i feature PR.
  - A registered one-body ball constraint holds its anchor while an
    unregistered clone moves. Typed Collider/Visual handles share live
    node geometry and pose; Collider identity and hashing agree with contact
    contexts and distinguish two nodes on the same Link.
  - Native fixture probes verify three DOFs with new-joint motion of 0.001,
    four contacts matching their native ShapeFrames, shared visual/collision
    aspects and live shape/pose updates. The independent constraint probe
    holds the anchor within 1.7e-12 while the free clone advances 0.02.
  - Two independent source reviews are clean. Focused guarded pytest reports
    two native passes and 46 strict XFAILs; `--runxfail` reports 46 intended
    pending-feature failures and two native passes. Lint, check-lint and
    public-path checks pass. The last full Python run remains `740426529`;
    these representative tests add no native implementation.
- **Configuration and persistence follow-up** (`e4b7b44e0` review):
  - The design explicitly defines a separate keyword-only `World(config=...)`
    overload, exclusive with individual constructor options. Its typed PGS
    marker also requires the Python DART default and rejects positional or
    mixed configuration. Native allocator fields are outside the Python API.
  - Selected clones receive distinct time steps through `set_parameter` and
    match serial Worlds with those parameters; the template stays unchanged.
    Transform validation rejects non-finite input, invalid matrices,
    reflections, shear and conflicting forms, with an explicit normalization
    projection case. Constructor spellings/tolerances remain with M5.
  - Checkpoints reject same-size structural and distinct primary-solver
    mismatches without partial restoration. A callable with declared Python
    state drives changing controls through exact replay; undeclarable hook
    state must make capture fail. Build/platform/extension compatibility and
    the full configuration matrix remain feature-owner release obligations.
  - Detached SimpleFrames exercise collection lookup, ownership rejection,
    removal and transfer. The loader resolves a package URI to a real mesh
    and distinguishes fixed zero-DOF from floating six-DOF root models.
  - Native probes confirm the package mesh/mobility fixture, frame membership
    setup, distinct solver types, per-row time-step motion and six exact
    replay frames from the stateful control fixture. Current native Worlds
    allow shared frame membership; the final Python ownership guard is pending.
  - Two independent source reviews are clean. Focused guarded pytest reports
    two native passes and 50 strict XFAILs; `--runxfail` reports 50 intended
    pending-feature failures and two native passes. Lint, check-lint and
    public-path checks pass. The last full Python run remains `740426529`;
    these representative checks do not implement the future runtime features.
- **Final-facade and nonzero-state follow-up** (`b0989ae41` review):
  - Future fixtures and all their oracles use the final World/Multibody/Link
    facade, named DOF properties and capability builders. They retain no
    Skeleton, ShapeNode, camelCase methods or forwarding aliases. Provisional
    builder, solver/sleep and computed-array spellings remain explicitly owned
    by M1/M5. StepHook stays in a provisional canonical submodule, outside the
    curated top-level re-export allowlist; both GUI fixtures use final World.
  - The separate unmarked native contact proof still validates the current
    physical scene. It skips explicitly once the legacy binding disappears;
    M5 retires or migrates that baseline proof and the existing direct-parser
    regression through the normal native-unit-test rename sweep.
  - The palm world-pose oracle uses independent NumPy forward kinematics from
    the fixture offsets and x/z joint axes. Its result exactly matches the
    current native transform, avoiding reuse of the property under test.
  - Both servos start with nonzero position and velocity. The position row
    also has a nonzero velocity target; saturated and unsaturated cases verify
    the complete current-state error and damping terms. Two native probes
    confirm constant diagonal inertia and zero Coriolis bias for this fixture.
  - Selected batch snapshots restore positions, velocities, both target
    channels and nonzero time. All source and per-row perturbations are
    observed before restoration; untouched rows and the source stay unchanged.
    The constructor gate now checks an explicit name, FCL detector and thread
    count as well as the Python defaults.
  - Native probes confirm the moving-servo bias, analytic palm transform and
    unchanged four-contact/sleep scene with a default-mobile zero-DOF weld.
    Two independent final source reviews are clean. Focused guarded pytest
    reports two native passes and 50 strict XFAILs, no XPASS. With `--runxfail`,
    the current missing final facade produces 11 intended API/logging failures
    and 39 setup errors; both native gates pass. Lint, check-lint and public-path
    gates pass. The last full Python run remains `740426529`.

- **Representative contract and qualification-scope follow-up on 2026-10-13**
  (`84f97f938` and `6af40c8e6` reviews):
  - Independently reconstructing the same final robot must accept a dynamic
    snapshot, restoring its perturbed state and clock without clone lineage.
    Batch writes now exercise scalar broadcast, noncontiguous indices and a
    boolean mask, observing selected and untouched rows and the source.
  - Both servos face a hard prismatic upper limit, with saturated and
    unsaturated targets. The provisional isolated effort observer must report
    the bounded row effort while the limit holds motion; an unrestricted
    reference moves. Native fixture evidence holds position/velocity within
    `1e-9` under outward effort, with the matching limit reaction. No aggregate
    force field is treated as the isolated motor effort.
  - The MJCF wrapper must preserve nondefault file time step and gravity.
    The current direct native parser still returns its default World settings;
    this is a pending wrapper contract, not a native behavior claim. The
    unmarked direct-parser FCL regression is retained unchanged.
  - Gravity and Material color exports reject writes and preserve previously
    returned arrays after later updates. Both named quaternion orders are
    compared with an asymmetric rotation. A clone created after registration
    must retain the manual constraint's anchor behavior.
  - Three distinguishable rendered objects require separate segmentation
    ownership. Off-center centroids check both camera axes; green explicit
    Material overrides red legacy RGBA, while blue legacy RGBA remains blue.
    Native parsing confirms the three named bodies and optical-basis geometry;
    GPU rendering remains pending and was not executed.
  - Four review suggestions concern required feature qualification beyond
    this M0 scaffold: detached-handle errors (M5), seeded-noise replay
    (M1h/M1i), versioned/older-7.y persistence fixtures (M1c), and moving
    frame-attached cameras (M6b). The matrix and both test modules explicitly
    require their owners to add those gates before removing feature markers.
    No unsettled API or serialized format is invented here.
  - Two independent final source reviews are clean. Focused guarded pytest
    reports two native passes and 56 strict XFAILs, no XPASS. `--runxfail`
    reports 11 intended API/logging failures and 45 setup errors from the
    absent final World API; both native gates pass. Lint, check-lint and
    public-path checks pass. The last full Python run remains `740426529`.

- **Review passes:**
  - The first review found seven issues: the step test read the legacy frame
    counter instead of `step_count`; the render test ran in the device-free
    suite; the loader test skipped `World.add`; step hooks and type stubs were
    listed as optional although the acceptance criteria and the design doc
    require them; the freeze rule would have moved additive must-haves to 7.1;
    the design doc carried release-train state; and the resume handoff was
    stale. The freeze-rule conflict went to the maintainer, who settled it on
    2026-10-12; the others were fixed.
  - The second review found that the first fix made `F` wait for every
    must-have, which changed the recorded rule that `F` follows the last
    breaking change. It also found that the handoff missed PRs merged since,
    that WP-7.0.3b and WP-7.0.5 were marked done without their review record,
    that M2's dense data was called optional although the design doc's mode
    contract requires it, that no test read `Link.world_transform`, that
    porting the interim GUI tests could drop the render test, that the
    published PR description contradicts the branch and wraps its lines, and
    that the first fix commit's message describes the superseded rule. All but
    the last two were fixed: the description's replacement goes up with the
    approved push, and the commit keeps its message because the squash merge
    uses the PR description.
  - The third review found that the second fix's exemption of the additive
    must-haves from the move to 7.1 also changed the recorded rule without
    approval. The recorded rule was restored, and the conflict went to the
    maintainer. It also found that `step` rejecting `bool` was missing from
    the breaking changes, that M1e could not move to 7.1 while the constraint
    module's 7.0 bindings wait for it, that the link test read a root link,
    whose world pose equals its pose in the parent frame and has no
    translation, that the standalone `Transform` test was gone, leaving M5a's
    `Transform` PR no marker to remove, and that this record claimed every
    second-review finding was fixed. These were fixed; the PR description
    still waits for the approved push.
  - On 2026-10-12 the maintainer settled the freeze rule as the project home
    now states it, amended D13, and set M0's exit and the steps after it, the
    M1a PR shape, the `dart7-base` ref and the step-path Gazebo gates. The
    project home records these instead of a pending decision.
  - The fourth review found that the link test compared an exact zero entry
    with no absolute tolerance and kept the palm's parent link at the origin,
    so a binding that dropped the parent's translation would pass; that the
    adapter evidence named no command in this repository; and that WP-7.0.3b
    and WP-7.0.5 lacked merge, CI and review evidence. The fixture now offsets
    both links and the test uses `atol=1e-12`, the adapter claim is removed,
    and both packets record that evidence while staying in progress. The
    review's other findings were already fixed on this branch: no text calls
    #3730 "in review", the GUI test package docstring names the render test,
    the must-have list says `compile()` prepares M2's dense data as the design
    doc requires, and the design doc carries no release-train state. A focused
    Codex review of the fix found one issue, the adapter bullet still present
    before the evidence update; it is removed.
- **Pending maintainer action:** check PyPI's project history for deleted
  7.0.0 files, because PyPI never accepts a reused file name.
- **Status:** in review.

### WP-7.0.8 Newer-Python wheels for 6.20.x

- **Scope:** add Python 3.14 wheels, and later 3.15, as release-branch
  packaging PRs on `release-6.20`. The migration guide states which Python
  versions `dartpy<7` supports.
- **Risk:** that line's bindings use a different binder, so the 3.15 wheels
  need their own review of GIL acquisition during finalization.
- **Status:** in progress.
  - [#3704](https://github.com/dartsim/dart/pull/3704) added 6.20.1 wheels for
    Python 3.12 to 3.14.
  - [#3730](https://github.com/dartsim/dart/pull/3730) added Linux Python 3.15
    wheels and runs a finalization test against every wheel. Its review of
    that line's pybind11 bindings found no crash or hang during finalization
    on Python 3.15, so the bindings are unchanged.
  - macOS and Windows Python 3.15 wheels wait for conda-forge's Python 3.15
    builds of `bullet-cpp`.


## Demo consolidation evidence

The following records moved from the retired demo project home. They describe
the named earlier packets, including the C++ app and transitional tests that
WP-PD.18 removes; commands and source names here are historical evidence.
Current gates live in [catalog verification](../../onboarding/testing.md#demo-catalog-verification).

### 2026-10-12: WP-PD.0, decision and project home

- **Change:**
  - D31b in the DART 7 decisions, plus the program README notes on the
    milestone, release path and interim bindings.
  - A Rendering bullet in `docs/design/dart7_python_api.md`.
  - The PLAN-622 GUI-example policy, in the dashboard and in the
    deformable-body project home.
  - This project home.
- **Design evidence:**
  - read-only exploration (six slices plus a completeness critic);
  - a design workflow with four sections, an adversarial verifier per
    section, and an integration critic;
  - an independent Codex design and a Codex review of the plan.

  Every finding that changed the plan was verified against the source at
  `9e4bb4e6f`.
- **Checks:** `pixi run lint` and `pixi run check-lint` passed. The
  check-lint aggregate includes `check-ai-commands` and the semantic
  `check-ai-infra` checks. The pre-commit AI gate and the local-path gate also
  passed.
- **Reviews:** every pushed head of #3732 first gets two consecutive clean
  local passes (`codex review --base origin/main`, gpt-6.1-sol at ultra
  effort). The GitHub reviews of the first three heads raised findings that
  the following commits fixed: runnable parity and wheel gates, the active
  setup's retention, the architecture table, the scene batches' helper
  owners, the active context's retention, the wheel catalog, the exact
  SIMBICON files, and this record. Later heads added the drawing-callback
  non-goal, the build-tree C++ parity command, the optional inputs a
  full-catalog run needs, tinkertoy's pick-and-drag check, the Pixi
  interpreter for parity runs, a world oracle for the humanoid scenes, and
  separate capture paths for the two apps.

### 2026-10-12: WP-PD.1 to WP-PD.3, the host stack

Published as stack #3748: #3745 (WP-PD.1), #3746 (WP-PD.2) and #3747
(WP-PD.3). Each PR's Testing section holds the commands and results.

- **WP-PD.1:** the default build, `test-gui`, `test-py`, the vanilla ImGui
  lane, the Linux wheel with its headless checks, and the GUI install test
  for bundled and system ImGui all pass. Reviews raised the bundled-header
  precedence for installed consumers and a consumer test that broke system
  ImGui builds; both are fixed and documented.
- **WP-PD.2:** the four model tests, `cycle-demos`, and the GUI-off and IO-off
  lanes pass; the 16 model files move unchanged.
- **WP-PD.3:** the host tests, `cycle-demos`, `test-gui`, the memory
  diagnostics lanes, the vanilla and GUI-off lanes, the install guard and 28
  before/after command-line runs pass. A `sleeping` capture differs from its
  base only in the two FPS readouts.
- **Reviews:** two consecutive clean local passes on each final head. The
  GitHub reviews then found that a copied `examples` directory no longer
  builds `dart-demos`, now documented as source-tree only (#3746), and that
  the checker rejected `test_GuiApp` as inactive in GUI-off builds (#3747).
  Both are fixed.

### 2026-10-12: WP-PD.8 and WP-PD.9, model bindings

- **WP-PD.8 (#3737):** resting state, `HeightmapShapef` and `VoxelGridShape`
  with occupancy updates. A review found uninitialized heightmap bounds in
  C++, fixed separately in #3744. Later reviews hardened the voxel inputs
  against OctoMap's limits: points must be finite and addressable after
  float narrowing, rays must fit OctoMap's key buffer and have a nonzero
  float length, and resolution is capped at 1e6 so ray lengths cannot
  overflow float. They also fixed a C++ query that read the wrong boundary
  cell.
- **WP-PD.9 (#3751):** whole-body IK bindings plus a C++ guard for expired
  hierarchy constraints. `test`, `test-py`, the GC probes and the Gazebo gate
  pass. A review found that GC could clear an IK module that moved to a live
  Skeleton; the traversal now counts only nodes still in the hierarchy's own
  Skeleton, with a regression test.
- **API snapshots:** #3735 makes `api_surface.py --snapshot` read
  non-arithmetic enums, so the binding PRs can diff the API surface.

### 2026-10-12: WP-PD.17, Python demo launch tasks

- **Change:** `demos`, `cycle-demos` and `capture`, including Windows
  variants, build dartpy and launch `python -m examples.demos`. The task
  working directory is `python/`; the command returns to the repository root
  with both dartpy and the source catalog on `PYTHONPATH`, preserving every
  relative CLI output path, including a forwarded `--shot` override.
  `py-demos` is removed. User docs, Python scene links and Korean PO/MO
  catalogs use the Python app. The C++ app, aliases and CTests remain for
  WP-PD.18.
- **Checks passed:**
  - `pixi run config`, then `pixi run build-py-dev` on the fresh build;
  - `pixi run python -m pip install ssik` (8.1.0, environment only);
  - `pixi run lint`, `pixi run sync-ai-commands` and
    `pixi run check-ai-commands`;
  - `pixi run test-ai-infra`: 1,968 passed;
  - `pixi run cycle-demos`: 37 scenes, twice, two frames each, zero skipped;
  - `pixi run demos -- --list-scenes`: 37 unique scene IDs;
  - `pixi run capture sleeping`: repository-root `capture.png`, 640×480,
    simulation time 2.5 seconds;
  - `pixi run capture soft_bodies -- --steps 50`: the same output path,
    frame 50 and simulation time 0.05 seconds;
  - additional captures with absolute and relative paths containing spaces,
    and a forwarded `--shot` overriding the positional output;
  - `pixi run test-py`: 1,610 passed, two Python 3.15-only skips;
  - `pixi run test-gui`: all 14 CTests passed; Python GUI and integration
    tests reported 550 passed and one Python 3.15-only skip;
  - `pixi run docs-build` and `pixi run docs-build-ko`, repeated after the
    review corrections;
  - changed-file local-path scan and `git diff --check`;
  - final `pixi run lint` and `pixi run check-lint`, including the configured
    semantic AI infrastructure check.
- **Visual assessment:** `pixi run image-verdict capture.png` passed for
  Sleeping. Native image inspection showed the Sleeping scene label,
  colored stacks and native workspace. The subsequent Soft Bodies image
  showed translucent embedded shapes and frame 50. These images corroborate
  the launcher and capture output; they do not re-establish scene parity or
  dynamics correctness. The catalog cycle and Python tests provide the text
  evidence. The default 640×480 workspace clips some panel labels, but the
  scene is visible. Images remain untracked.
- **Reviews:** two role-separated read-only reviews of the final task/docs
  diff are clean after correcting the configure-task name, humanoid slider
  instructions and manual dartpy build prerequisites. Windows definitions
  were read twice; they use the multi-config dartpy path and the existing
  `build-py-dev --config` build.
- **Limitations:** Windows runtime cannot be checked on this host. No commit
  or publication is authorized. The C++ app's removal belongs to WP-PD.18.

## 2026-10-12: WP-PD.18, C++ demo retirement

- **Change:** Delete the C++ application and all 53 files under
  `examples/demos/`, its aliases and four CTests, and both neural-net files
  under `data/humanJointLimits/`. Remove the host's raw panel callback,
  frame-list drag setup, and construction-dump option. Retain the private
  host and native model libraries, their tests, and the other C++ examples.
  Direct headless CLI captures now default to `demos.png`; the Pixi capture
  task still defaults to `capture.png`.
- **References:** Update the example CMake list, Filament CI target and CTest
  lists, Linux and Windows GUI tasks, infrastructure checker and fixtures,
  sync capability ledger and fixtures, dartpy's compiled-out diagnostics
  checker, host comments and memory fixtures, and Python catalog tests.
  Top-level/tests CMake and coverage, Doxygen, lint, and format lists have
  no app-specific references left to remove; their retained rules still
  cover the private libraries and other examples.
- **Promotion:** Architecture owns private targets and build gates;
  `examples/README.md` owns scene rules, coordinates and helper placement;
  Python-bindings guidance owns callbacks and declarative panel validation;
  testing owns construction, interaction and capture parity; profiling owns
  memory diagnostics. Update onboarding/build/CI, public visualization and
  example pages, Korean PO/MO, AI verification skill and generated adapter,
  dashboard, D31b/program handoffs and older task entrypoints. Move the
  deferred entries unchanged into [changelog drafts](changelog-drafts.md),
  preserve earlier evidence above as historical, and delete the demo project
  home. `CHANGELOG.md` is unchanged: its entry is deferred to M6b's
  consolidation PR by maintainer instruction.

| Command | Result |
| --- | --- |
| `pixi run config` | Passed first on the fresh worktree and after removal |
| `pixi run python -m pip install ssik` | Installed 8.1.0 in the environment only |
| `pixi run lint` | Passed |
| `pixi run build` | Passed |
| `pixi run test` | 194 CTests passed |
| `pixi run build-py-dev` | Passed, including the default diagnostics marker check |
| `pixi run test-py` | 1,610 passed; two Python 3.15-only skips |
| `pixi run cycle-demos` | 37 scenes x2, two frames each, zero skipped |
| `pixi run test-gui` | 10 CTests and 550 Python tests passed; one Python 3.15-only skip |
| `pixi run test-ai-infra` | 1,960 passed after correcting the removed-CTest guard |
| `pixi run sync-ai-commands` | Passed |
| `pixi run check-ai-commands` | Passed |
| `pixi run docs-build` | Passed |
| `pixi run docs-build-ko` | Passed; regenerated the changed Korean example catalog |
| `pixi run exercise-agent-scenarios` | Passed |
| `pixi run check-lint` | Passed, including the configured semantic infrastructure check |
| Changed-file local-path scan and `git diff --check` | Passed |
| `pixi run capture sleeping build/default/pd18-sleeping.png -- --width 960 --height 600` | Passed; 150 frames, simulation time 2.5 seconds |
| `pixi run image-verdict build/default/pd18-sleeping.png` | Passed |

Additional build lanes use the same Pixi dependencies and Release options:

```bash
pixi run sh -c 'cmake -G Ninja -S . -B build/default/cpp/pd18-vanilla -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" -DDART_BUILD_DARTPY=ON -DDART_BUILD_UTILS_ASSETS=ON -DDART_BUILD_PROFILE=ON -DDART_USE_SYSTEM_GOOGLEBENCHMARK=ON -DDART_USE_SYSTEM_GOOGLETEST=ON -DDART_USE_SYSTEM_IMGUI=ON -DDART_USE_SYSTEM_TRACY=ON'
pixi run cmake --build build/default/cpp/pd18-vanilla --target tests --parallel 24
pixi run sh -c 'cmake -G Ninja -S . -B build/default/cpp/pd18-gui-off -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" -DDART_BUILD_GUI=OFF -DDART_BUILD_DARTPY=OFF -DDART_BUILD_UTILS_ASSETS=ON -DDART_BUILD_PROFILE=ON -DDART_USE_SYSTEM_GOOGLEBENCHMARK=ON -DDART_USE_SYSTEM_GOOGLETEST=ON -DDART_USE_SYSTEM_TRACY=ON'
pixi run cmake --build build/default/cpp/pd18-gui-off --target tests --parallel 24
pixi run ctest --test-dir build/default/cpp/pd18-gui-off -R '^test_(ContactArrowLayout|AdaptiveSoftContactModel|SoftWormModel|SoftFootSimbiconModel)$' --output-on-failure
```

Both configures and `tests` builds passed. Vanilla ImGui reports the host
and viewer binding compiled out. All four GUI-off model CTests passed.
The initial four-worker builds were intentionally interrupted and restarted
with 24 workers; no source fix was needed.

- **Infrastructure correction:** The first full suite exposed a remaining
  reference to the deleted `app_arguments` variable. Remove that guard;
  the original run was stopped after 102 failures and 879 passes. A focused
  semantic run passed 82 cases before an intentional stop to avoid duplicating
  the restarted full suite. The final 1,960-test run is clean. The worker's
  initial focused ledger/label/reference check also passed 10 cases.
- **Visual assessment:** Native image inspection shows the Sleeping scene,
  the 37/37 navigator, colored stacks, controls, toolbar and diagnostics in
  the native workspace. At 960x600, the docked viewport crops part of a stack;
  this does not establish scene or dynamics
  parity anew. The cycle and tests provide the correctness evidence, while
  the capture corroborates the remaining Python app and host. The image
  stays in the ignored build tree.
- **Reviews:** Two independent read-only reviews found no confirmed defects
  in the final implementation, reference cleanup and promotions after the
  checker fix. The final evidence and construction-count wording also receive
  a narrow review.
- **Limitations:** Linux verification only; Windows/macOS runtime, wheels,
  vanilla dartpy runtime and downstream Gazebo suites were not run for this
  private application removal.

## M1a: World step stages

- **Status:** merged in [#3775](https://github.com/dartsim/dart/pull/3775).
- **Objective:** split `World::step` into private inlined stages while keeping
  stepping bit-identical to main.
- **Baseline:** `dart7-base` is pinned to main `c3134f51b86d` as a local tag.
  Its full nightly smoke finished on 2026-10-13 at 05:16:54 UTC on the
  local Threadripper 3970X performance host (GCC 13.3, glibc 2.39,
  Valgrind 3.22): PASS, all 78 rows `ok`, comprising 31 gated and 47
  diagnostic rows. The run used native and harness revision
  `c3134f51b86d0c147d505aebc3c6e1cf9d92e5c3`; all S7 detector/thread rows
  are present. The completed record's SHA-256 is
  `cfbb7bda672844f883febc66789b45da622f00e4785df01141ee5e13c3d693fc`.
  This establishes the pre-refactor baseline, not later milestone performance.
- **Nightly A/B:** at `a751424b3` against main `6cca697a2`, PASS: 31 neutral
  and 41 diagnostic rows; Ir geomean +0.009%, highest `s2r/dart` at +0.062%.
  Guards and allocations are identical.
- **Correction:** the first full A/B failed `s2r/dart` at +2.49% because the
  seven stage calls went through the PLT under `-fPIC`. Inlining the stages
  and holding the solve's group and filter once fixed it.
- **S7 A/B (2026-10-12):** main `c3134f51b86d` against merged head
  `8afe3251455d`, using
  `pixi run python scripts/perf_regression.py local --base origin/main --head HEAD --rows S7-t1,S7-t4 --jobs 8`
  with a separate private staging root: PASS, six diagnostic rows (no Ir gate).
  All guards and resting counts match base and agree across one and four
  threads: DART and ODE rest 3003/3003 bodies; Bullet rests 2724/3003.
  Allocation counts and requested bytes are identical to base in all six rows.
- **Other evidence:** StepAllocation counts are byte-identical to the base
  build; the moved-line audit is clean.
- **Gates (2026-10-12):** on merged head `8afe3251455d` (main
  `c3134f51b86d`), `pixi run lint`, `pixi run check-lint`, `pixi run build` and
  `pixi run test` pass; 185/185 CTests, including StepAllocation.
- **Gazebo:** the Jetty and Harmonic lanes pass against a base variant. Ionic
  had one load-dependent failure, gz-physics `JointSetPositionWithContact`,
  which passed 15/15 through CTest on both base and head. The sleep oracle
  reports the same two pre-existing mismatches on base and head; it needs the
  C-only probe fix from [#3756](https://github.com/dartsim/dart/pull/3756) to
  configure. `test-gz`: 199/199 + 4/4 + 1/1.
- **Reviews:** two adversarial passes and a local Codex review, all clean.
- **Follow-up for M1d:** the load-dependent result suggests nondeterministic
  parallel chunking.
- **Changelog decision:** no entry required; the diff against main preserves
  API and behavior and documents an internal refactor.

## M1b preparation-scope spike

- **Contract:** preparation readiness follows this World's Skeletons, direct
  solver Skeletons (including shapeless ones), and Skeleton ancestors of
  collision-group frames. Preserve option/filter/thread/group checks, lazy C++
  edits and shared membership. The global structural stamp may be a cheap
  revalidation hint; it must not itself invalidate an unrelated World.
  Bookkeeping fits behind the existing private state pointer, preserving
  installed headers and object layout.
- **Reproduction (2026-10-13):** isolated C++ probe against unchanged World
  source, default FCL, adding a child RevoluteJoint. Preparing independent A
  and B yields `[true, true]`; editing B yields `[false, false]`, although A's
  Skeleton version remains unchanged. The required result is `[true, false]`.
  Preparing A afterward yields `[true, false]`.
- **Existing protection:** two Worlds sharing one Skeleton correctly become
  `[false, false]` after its edit; preparing only A leaves B stale. This is a
  passing protection case, not a second confirmed failure.
- **Artifacts:** `build/world-preparation-scope/` contains source, transition
  JSON, assertions-enabled build/run logs and provenance. Repeat with
  `pixi run python build/world-preparation-scope/reproduce.py`.
- **Next decisive cases:** own in-place topology edits and lazy rebuilding;
  shared-Skeleton invalidation isolation; external solver/group/ancestor
  dependencies and their replacement/removal. Keep unchanged-epoch readiness
  allocation-free. Required verification includes exact state/contact traces,
  StepAllocation and IslandDeactivation, core/Python and downstream gates,
  and detector/thread-specific baseline comparisons.
- **Limits and status:** evidence only; no production implementation, steps,
  allocation/performance or other-detector/platform checks. Full M1b's
  `compile()`, `edit()`, strict mutation tiers and unsupported-shape rejection
  remain open. This spike does not waive the recorded M0 exit or introduce
  exclusive ownership restrictions for lazy C++ Worlds.

## M6a closeout

- **Date:** 2026-10-18.
- **Change:** retire `docs/dev_tasks/dart6_filament_renderer/` and record
  M6a's final state in this program home.
- **Wheels:** every `dartpy-wheels` row passed in two Nightly runs on main:
  [39076746642](https://github.com/dartsim/dart/actions/runs/39076746642)
  (dispatched at `7fccf149e0c` on 2026-10-17) and
  [39128135570](https://github.com/dartsim/dart/actions/runs/39128135570)
  (scheduled at `f630c91c80e` on 2026-10-18). The rows cover Linux x86_64 and
  aarch64 for CPython 3.10, 3.11 and the 3.12 stable ABI, plus the Windows and
  macOS stable-ABI wheels. Each row builds, repairs, verifies and tests its
  wheel, and both Linux stable-ABI rows pass the slim-container headless
  import. The rows took 13 to 53 minutes of their 120-minute limit, so the
  separately cached SDK stage is dropped.
- **Promotion:** the C2 gate takes the quiet-host performance contract and
  the final renderer reviews, the M6a interface takes the wheel state and open
  follow-ups, and testing takes the pixel-bound rule. Owner docs already held the build, test-label,
  migration and renderer facts. The historical OSG comparisons and preview
  evidence remain in Git history and #3696.
- **Routing:** the orchestrator wrote this closeout because it depends on
  M6a context; engines other than the writer reviewed it.
- **Limitations:** no new renderer or wheel run; quiet-host renderer
  performance and the final renderer reviews remain open.
