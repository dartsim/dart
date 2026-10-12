# Verification

Evidence per packet. A packet is done only after its gates pass and two clean
review passes are recorded on its final state. Every claim records its
revision, command, date and scope.

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
- **Status:** done; merged in
  [#3719](https://github.com/dartsim/dart/pull/3719). It changes behavior, so
  it targets main only.

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
  - rows measure stepping plus reading transforms and contacts.
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
- **Baseline procedure:** immediately before the first M1 PR, record the last
  main SHA as `dart7-base`, with its host and the per-row records from
  `pixi run python scripts/perf_regression.py local --smoke --nightly --head <sha>`.
  Each milestone exit runs
  `pixi run python scripts/perf_regression.py local --base dart7-base --head HEAD --rows nightly`
  and compares each detector against its own baseline.
- **Status:** done; merged in
  [#3720](https://github.com/dartsim/dart/pull/3720). The `dart7-base` pin
  waits for the start of M1.

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

### WP-7.0.7 Scope artifacts

- **Pull request:** [#3728](https://github.com/dartsim/dart/pull/3728).
- **Scope:**
  - the dependency/order/priority refinement dated 2026-10-11; historical
    evidence dates stay;
  - API freeze `F`, declared after every must-have, additive ones included,
    has merged and C2 has passed; only additive work outside the must-have
    list moves to 7.1 if it misses `F`;
  - C1 after the first three M2 kernel PRs and their stop rule, with M3 able to
    land before `F` without delaying it;
  - C2 before `F`, so release candidates carry the final render API;
  - the default-detector switch only if its prerequisites pass before M1
    starts, scheduled first so later rebaselines start from the final default;
    otherwise a labeled 7.x change;
  - the M3 thresholds: at least 15% `Ir` gain on `dyn`/`robot`, improved K = 64
    wall time, and the flagged spike passing guards and the shim with `memcpy`
    snapshots at most 0.1× the gather cost; M2 stops below 10% gain from its
    first three kernel PRs;
  - the 7.0 must-have list, covering everything the acceptance criteria and
    the design doc require, and explicit deferrals;
  - the target-shape acceptance tests, marked `xfail(strict=True)`; the
    headless-render test needs a graphics device, so `pixi run test-gui` runs
    it instead of `pixi run test-py`.
- **Evidence on 2026-10-11** (main `9e4bb4e6f`):
  - `pixi run test-py` collects the nine tests in
    `python/tests/acceptance/test_target_shape.py`, all XFAIL with no XPASS
    (688 passed, 2 skipped, 9 XFAIL), and no longer collects the render test.
  - The `pytest` step of `pixi run test-gui`, over `python/tests/unit/gui` and
    `python/tests/integration`, collects
    `python/tests/unit/gui/test_target_shape_render.py` as XFAIL (262 passed,
    1 skipped, 1 XFAIL).
  - With `--runxfail`, each test fails for its intended missing feature:

  | Test | Observed failure |
  | --- | --- |
  | World construction keywords | `AttributeError`: no `dart.World` |
  | edit/compile | `AttributeError`: no `World.compile` |
  | loading and `World.add` | `AttributeError`: no `dart.data` |
  | joint PositionServo | `AttributeError`: no `dart.PositionServo` |
  | IMU sensor | `AttributeError`: no `dart.sensors` |
  | `step(n)` and `step_count` | `TypeError`: `step` accepts only a `bool`, not `20` |
  | Transform composition | `AttributeError`: no `dart.Transform` |
  | capture/restore state | `AttributeError`: no `World.capture_state` |
  | WorldBatch set/step | `AttributeError`: no `dart.WorldBatch` |
  | Renderer RGB (`test-gui`) | `AttributeError`: no `dart.render` |

  `pyproject.toml` already sets `xfail_strict = true`, and each test also
  passes `strict=True`. The contract lives only in these tests, not in
  `python/examples/`, because that directory's examples run automatically.
- **Review passes:** the first review found seven issues: the step test read
  the legacy frame counter instead of `step_count`; the render test ran in the
  device-free suite; the loader test skipped `World.add`; step hooks and type
  stubs were listed as optional although the acceptance criteria and the
  design doc require them; the freeze gate ignored additive must-haves; the
  design doc carried release-train state; and the resume handoff was stale.
  All were fixed.
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
  - [#3730](https://github.com/dartsim/dart/pull/3730), in review, adds Linux
    Python 3.15 wheels and runs a finalization test against every wheel. Its
    review of that line's pybind11 bindings found no crash or hang during
    finalization on Python 3.15, so the bindings are unchanged.
  - macOS and Windows Python 3.15 wheels wait for conda-forge's Python 3.15
    builds of `bullet-cpp`.
