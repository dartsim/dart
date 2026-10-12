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
- **Status:** in progress. It changes behavior, so it targets main only.

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

- **Objective:** pin `dart7-base`, the last main revision before M1, and record
  its host and the `perf_regression.py local ... --rows nightly` results here.
- **Scope:**
  - add `--worlds K` to `tools/perf/portable_step_bench.cpp`, using 6.x C++ APIs
    so both arms run it, with detectors pinned;
  - per PR: K = 1 and 64, with the DART and ODE detectors;
  - nightly: K = 1024, plus FCL, Bullet and the closed-loop joint-friction row;
  - a dartpy loop benchmark written in 6.x spelling, which the M5 codemod later
    converts (doubling as a codemod test);
  - rows measure stepping plus reading transforms and contacts.
- **Gate:** every milestone exit runs `local --base dart7-base --rows nightly`.
- **Status:** planned.

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
- **Status:** planned.

### WP-7.0.7 Scope artifacts

- **Scope:**
  - the freeze date `F`, the checkpoints C1 and C2, and the M3 thresholds;
  - the 7.0 must-have list and explicit deferrals;
  - the target-shape acceptance tests, marked `xfail(strict=True)`;
  - a maintainer check of the PyPI project history for deleted 7.0.0 files,
    because PyPI never accepts a reused file name.
- **Status:** planned.

### WP-7.0.8 Newer-Python wheels for 6.20.x

- **Scope:** add Python 3.14 wheels, and later 3.15, on `release-6.20` through
  `dart-backport-pr`. The migration guide states which Python versions
  `dartpy<7` supports.
- **Risk:** that line's bindings use a different binder, so the 3.15 wheels
  need their own review of GIL acquisition during finalization.
- **Status:** planned.
