# Resume: SKEL Format Evolution

## Current Resume Checkpoint (2026-07-04 UTC)

Phase 3 is decided and committed locally on `work/skel-format-yaml-decision`,
stacked on the Phase 2 removal branch. The decision record is
[`03-yaml-decision.md`](03-yaml-decision.md): do not add YAML as a DART 7
`dart::io` model format, do not build a YAML wrapper over URDF/SDF/MJCF, and do
not carry SKEL syntax forward. If YAML returns, it must be a DART-owned
project/scene serialization design with schema/versioning rules plus import,
export, and round-trip tests.

Phase 2 remains implemented on `feature/remove-skel-dart7-phase2` and awaiting
PR creation/review/merge. The stacked Phase 3 branch is documentation-only and
should not be pushed as the Phase 2 removal PR.

Phase 4 coordination is also recorded locally in
[`04-usd-coordination.md`](04-usd-coordination.md): PR #3109 already landed the
OFF-by-default USD `dart::io` scaffold, and remaining USD loader/viewer/dartpy
implementation stays in `docs/dev_tasks/usd_scene_loader/` instead of this SKEL
task.

Phase 5 export-writer planning is recorded locally in
[`05-export-writers-plan.md`](05-export-writers-plan.md). The plan keeps export
as a separate implementation phase: choose SDF writer, URDF writer, or PLAN-101
project save/load first; define deterministic resource handling and comparison
helpers; and complete the phase only with writer APIs plus read/write/read
tests. This planning record does not complete Phase 5.

The first SDF writer implementation slice is now local on
`work/skel-format-yaml-decision`: `dart::utils::SdfParser` exposes
`tryWriteSkeletonToString()` for a conservative `Skeleton` subset, with
`INTEGRATION_io_SdfWriter` proving write/read round-trip for links, root
FreeJoint/WeldJoint placement, revolute/prismatic/weld/screw/universal child
joints, inertial data, primitive or mesh geometry, link gravity mode, passive
joint dynamics metadata (damping, Coulomb friction, spring reference, and spring
stiffness), screw thread pitch, SDF 1.11+ axis/axis2 mimic metadata with motor
enforcement, topology-only ball child joints, local root/joint/shape poses,
explicit visual material colors, and absolute non-file mesh URI preservation
through a custom retriever. It also checks `WriteOptions` visual/collision
filtering, unsupported-shape diagnostics, missing mesh URI diagnostics,
pre-SDF-1.11 mimic diagnostics, unsupported coupler-style mimic diagnostics,
non-finite visual material diagnostics, non-finite screw pitch diagnostics,
unsupported ball-joint metadata, and non-finite joint dynamics diagnostics. This
is real Phase 5 progress, but Phase 5 is still open until broader SDF coverage
plus the remaining accepted writer targets are implemented or durably deferred.

The SDF writer integration test now uses
`tests/helpers/io_round_trip_helpers.hpp` for reusable body, joint, DoF,
inertia, and shape comparisons, and `tests/CMakeLists.txt` includes the helper
in the formatted test-helper list. This closes the Phase 5 comparison-helper
infrastructure item, but it does not broaden the public writer contract by
itself.

The writer API home decision is now recorded in
`docs/onboarding/io-parsing.md` and
[`05-export-writers-plan.md`](05-export-writers-plan.md): keep export APIs
format-owned until more than one accepted writer contract exists. The SDF
writer stays on `dart::utils::SdfParser`, `dart::io` remains read-side, and
project/editor save-load belongs to the scene/project layer.

Validation for this slice:

- `git diff --check`
- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`
- `pixi run run-cpp-target INTEGRATION_io_SdfParser`
- `pixi run run-cpp-target INTEGRATION_io_Read`
- `pixi run run-cpp-target test_io_readNone`
- `pixi run lint`
- `pixi run build`

Additional validation for the comparison-helper refactor:

- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`

Additional validation for the writer-API-home docs:

- `git diff --check`
- `pixi run lint`

Additional validation for absolute non-file mesh URI writer coverage:

- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`

Additional validation for writer options and missing mesh URI diagnostics:

- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`

Additional validation for SDF 1.11 mimic metadata writer coverage:

- `pixi run run-cpp-target INTEGRATION_io_SdfWriter`

Changelog decision:

- Mode: draft
- Base evidence: `origin/main`
- Scope evidence: local diff adds the parser-specific SDF writer API,
  integration round-trip coverage, IO docs, and SKEL-format task handoff.
- Decision: entry required
- Target section: `CHANGELOG.md` -> DART 7 -> IO and Parsing
- Entry text: present as the conservative SDF writer bullet, without a PR link
  until the implementation PR exists.
- PR-body note: N/A
- Follow-up: add the implementation PR link after a PR exists and maintainer /
  user approval allows the PR update or follow-up push. No additional changelog
  entry is needed for the comparison-helper refactor because it only changes
  test infrastructure and task handoff docs. No additional changelog entry is
  needed for the absolute non-file mesh URI coverage because it only strengthens
  test evidence for the existing SDF writer entry. No additional entry is needed
  for writer-option and missing-mesh-URI coverage because it hardens the same
  existing writer surface before the implementation PR exists. No additional
  entry is needed for SDF 1.11 mimic metadata coverage because it broadens the
  same conservative SDF writer capability before the implementation PR exists.

## Previous Resume Checkpoint (2026-07-03)

Phase 2 is implemented on `feature/remove-skel-dart7-phase2` and awaiting PR
creation/review/merge. The branch removes the C++ SKEL parser, `dart::io`
`.skel` and `<skel>` inference/dispatch, `ModelFormat::Skel`,
parser-specific tests, dartpy SKEL bindings/stubs, user-facing SKEL docs, and
all sample `.skel` fixtures. Non-SKEL Kima Collada meshes were relocated from
`data/skel/kima/` to `data/mesh/kima/` and the mesh tests were updated to load
that path.

Phase 2 validation run on the branch:

- `pixi run python -m pytest python/tests/unit/test_check_dartpy_import_layout.py python/tests/unit/utils/test_utils_stub_import.py python/tests/unit/test_check_dart7_world_promotion_blockers.py`
- `pixi run check-ai-commands`
- `pixi run check-dartpy-import-layout`
- `pixi run run-cpp-target INTEGRATION_io_Read`
- `pixi run run-cpp-target UNIT_dynamics_MeshShape`
- `pixi run run-cpp-target test_mesh_loaderNone`
- `pixi run lint`
- `pixi run build`
- `pixi run test-unit`
- `pixi run test-py`
- `pixi run test-all`
- `CUDAARCHS=89 DART_CUDA_ARCHITECTURES=89 pixi run -e cuda test-all`

The first CUDA full-gate attempt hit a stale/default generated CMake CUDA
architecture (`compute_52`) before target-level `DART_CUDA_ARCHITECTURES`
could apply. Clearing `build/cuda/cpp/Release` and setting both
`CUDAARCHS=89` and `DART_CUDA_ARCHITECTURES=89` matched this host's RTX 5000
Ada GPU and the CUDA full gate passed.

The DART 7 changelog has a draft Breaking Changes entry without a PR link; add
the link after the Phase 2 PR exists.

Changelog decision:

- Mode: draft
- Base evidence: `origin/main`
- Scope evidence: `git diff --stat origin/main...HEAD` removes the SKEL parser,
  `.skel` fixtures, SKEL docs, parser tests, and dartpy SKEL bindings/stubs.
- Decision: entry required
- Target section: `CHANGELOG.md` -> DART 7 -> Breaking Changes
- Entry text: present as the legacy SKEL removal bullet, without a PR link
  until the Phase 2 PR exists.
- Follow-up: add the Phase 2 PR link to the same bullet before merge.

Post-commit completion audit cleared stale active-doc references that still
listed SKEL as a DART 7 model-loading/parser target. The remaining tracked
SKEL-format references outside this task folder are the DART 7 removal
changelog entry and historical DART 6 changelog entries.

## Previous Resume Checkpoint (2026-07-03)

Phase 1 landed on `main` via PR
[#3065](https://github.com/dartsim/dart/pull/3065), merge commit
`5dbe995d221` (2026-06-19): all SKEL files that were referenced by examples,
tests, or benchmarks have SDF replacements and the live
test/resource-retriever references have moved off `dart://sample/skel`. The
converted fixtures are:

- `data/skel/test/single_pendulum.skel` -> `data/sdf/test/single_pendulum.sdf`
- `data/skel/cube.skel` -> `data/sdf/test/cube.sdf`
- `data/skel/shapes.skel` -> `data/sdf/test/shapes.sdf`
- `data/skel/test/test_shapes.skel` -> `data/sdf/test/test_shapes.sdf`

The only remaining `.skel` strings under tests were parser-specific temporary
paths in `tests/integration/io/test_skel_parser.cpp`; they were removed with the
parser in Phase 2.

## Last Session Summary

Captured the strategic decision to close out
[issue #496](https://github.com/dartsim/dart/issues/496) by _not_ converting
SKEL to YAML, and to remove SKEL before the DART 7 release. Optional YAML work
should be a new DART 7 format decision or a front-end over existing URDF/SDF
semantics, not SKEL syntax in YAML. USD adoption remains tracked via
[`usd_scene_loader/`](../usd_scene_loader/), and export writers remain part of
the longer-term round-trip story. The retired `feature/skel_yaml` branch carried
a 1,991-line draft plan; that draft is preserved by SHA so useful phase details
can be recovered without adopting the old SKEL YAML direction.

## Current Branch

`work/skel-format-yaml-decision` — Phase 3 YAML decision, Phase 4 USD
coordination, Phase 5 export-writer planning, and the first SDF writer slice are
local on top of the Phase 2 removal commit. The underlying Phase 2 branch
remains `feature/remove-skel-dart7-phase2`.

## Immediate Next Step

Continue with the remaining real task work: land Phase 2 after maintainer
approval, then continue Phase 5 from
[`05-export-writers-plan.md`](05-export-writers-plan.md) by extending SDF writer
coverage beyond material colors/link gravity mode/passive joint dynamics/screw
thread pitch/universal and ball topology/local poses/absolute non-file mesh URI
preservation/SDF 1.11 mimic metadata or choosing the next accepted writer target
(URDF or PLAN-101 project save/load).

## Context That Would Be Lost

- The 2015 issue #496 proposed converting SKEL XML to YAML. That proposal is
  rejected here; the rationale lives in `README.md` under _Key Decisions_.
- YAML is rejected as a DART 7 `dart::io` model format and as a front-end over
  third-party canonical formats. If added later, it must be a DART-owned
  project/scene serialization format, not SKEL syntax in YAML.
- USD work is tracked separately under
  [`usd_scene_loader/`](../usd_scene_loader/) — do not duplicate.
- Phase 4 is a coordination boundary, not a claim that USD implementation is
  complete. The USD task still owns the OpenUSD-enabled loader, viewer, dartpy,
  dependency, and CI work.
- Export is part of the work, not a follow-up: round-trip enables save /
  load in the dartsim editor (PLAN-101). The current Phase 5 file is a plan,
  and the current SDF writer is only the first implementation slice, not an
  implementation-complete gate.
- The old `feature/skel_yaml` prototype SHA (`1dd83e31586`) is no longer
  reachable in this worktree. Do not depend on it for Phase 3 or Phase 5 unless
  another clone still has the object.

## How to Resume

```bash
git checkout work/skel-format-yaml-decision
git status && git log -4 --oneline
pixi run run-cpp-target INTEGRATION_io_SdfWriter

# To return to the Phase 2 PR-ready branch:
# git checkout feature/remove-skel-dart7-phase2

# Optional archaeology only if another clone still has the object:
# git show 1dd83e31586:docs/dev_tasks/skel_format/phase-02-yaml.md
# git show 1dd83e31586:docs/dev_tasks/skel_format/phase-04-export.md
```

Then continue with the remaining task work. For Phase 5, start from this
folder's `README.md`, `03-yaml-decision.md`, `04-usd-coordination.md`,
`05-export-writers-plan.md`, and current DART 7 requirements, not from the old
prototype.
