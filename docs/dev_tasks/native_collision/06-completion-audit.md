# Native Collision Completion Audit

This audit maps the user-facing north-star requirements to concrete artifacts
and evidence. It is the checkpoint to read before deciding whether
`docs/dev_tasks/native_collision/` can be deleted.

## Objective Restatement

The single PR is complete only when DART's normal collision runtime is one
built-in detector stack, FCL/Bullet/ODE are absent from normal runtime builds,
old detector names are compatibility facades over the built-in detector, and
the built-in layer is proven feature-complete, correct, scalable, and
performance-oriented through tests, benchmarks, package checks, downstream
compatibility, and CI artifacts.

## Completion Decision

Status: not complete.

The current branch has strong local evidence, but the north star still has
unverified external and finalization gates:

- Native-only and gz-physics CI evidence is still missing.
- Full dartpy wheel matrix artifact evidence is still missing.
- GitHub artifact evidence for the scheduled/manual collision benchmark guard
  is still missing.
- Downstream migration/deprecation evidence is still missing.
- Final legacy-runtime deletion or hard-deprecation decisions are still
  pending.
- Final `pixi run test-all` evidence after the final PR state is still
  missing.
- The dev-task folder must remain until final PR evidence is transferred to
  the PR description and durable architecture notes are moved to onboarding
  docs.

## Audit Inputs

Current audited state:

- Branch: `feature/new_coll`
- Head inspected at audit start: `c502a0391c1`
  (`Record native collision benchmark guard evidence`)
- Working tree at audit start: clean, ahead of `origin/feature/new_coll`.
- Remote branch state: `origin/feature/new_coll` was still at
  `96436fd2503`; the local branch was 41 commits ahead after the audit
  checkpoint.
- GitHub PR state: read-only GitHub search for `head:feature/new_coll` in
  `dartsim/dart` returned no pull requests, so no CI or PR artifact evidence
  exists yet for the current local north-star branch head.
- Local command run during this audit:

  ```bash
  python scripts/check_collision_runtime_isolation.py
  ```

- Observed output:

  ```text
  Collision runtime isolation check passed.
  ```

Additional inspected artifacts:

- `docs/dev_tasks/native_collision/README.md`
- `docs/dev_tasks/native_collision/01-design.md`
- `docs/dev_tasks/native_collision/03-evidence-gates.md`
- `docs/dev_tasks/native_collision/04-reference-gap-analysis.md`
- `docs/dev_tasks/native_collision/05-downstream-migration.md`
- `pixi.toml`
- `.github/workflows/ci_ubuntu.yml`
- `.github/workflows/publish_dartpy.yml`
- `scripts/check_collision_runtime_isolation.py`
- `scripts/verify_wheel_collision_isolation.py`
- `dart/collision/**`
- `python/dartpy/collision/collision_detector.cpp`
- `tests/unit/collision/**`
- `tests/benchmark/collision/**`
- `docs/dev_tasks/native_collision/smoke/native_compat_package/**`

## Prompt-To-Artifact Checklist

| Requirement                                                                                                                  | Evidence                                                                                                                                                                                                                                                                               | Status  |
| ---------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------- |
| Track progress on the scale of the path toward the north star.                                                               | `README.md` and `03-evidence-gates.md` both define the 0-12 north-star progress scale. Stage 10 has local design evidence, stage 11 has local source/reference split evidence, and stage 12 remains blocked on CI, migration, final deletion, validation, and PR packaging.            | Done    |
| One PR continues until it fully reaches the north star.                                                                      | `README.md` states the checkpoint is not a final PR boundary, and `02-milestones.md` keeps final PR packaging and dev-task deletion as completion gates.                                                                                                                               | Open    |
| `dart/collision/` stops exposing a real multi-backend runtime selection layer.                                               | `dart/collision/dart/dart_collision_detector.hpp` owns the canonical `dart` registrar plus legacy alias registrars. Top-level FCL/Bullet/ODE headers route through compatibility facades, and old implementation sources live under explicit `reference/` paths.                       | Local   |
| Retained FCL/Bullet/ODE/experimental names are wrappers/adapters over the built-in detector.                                 | Compatibility headers under `dart/collision/{fcl,bullet,ode}/compat/`, Python aliases in `python/dartpy/collision/collision_detector.cpp`, and the package smoke under `docs/dev_tasks/native_collision/smoke/native_compat_package/` verify native-backed compatibility names.        | Local   |
| Any selected "backend" through retained public names always uses the built-in detector.                                      | `UNIT_collision_DartCollisionDetector` covers factory aliases and legacy facade behavior. The native compatibility package smoke verifies package components, factory keys, installed headers, and direct legacy `create()` calls.                                                     | Local   |
| FCL/Bullet/ODE remain only as optional reference engines for tests and benchmarks.                                           | `DART_BUILD_COLLISION_REFERENCE_TESTS` and `DART_BUILD_COLLISION_REFERENCE_BENCHMARKS` exist in CMake/Pixi configure paths; `collision-reference` is the explicit opt-in Pixi environment; reference call sites use `createReference()` and `collision-reference-*` names.             | Local   |
| Native-only builds can opt out of FCL/Bullet/ODE.                                                                            | Normal Pixi configure paths default FCL, Bullet, ODE, reference tests, and reference benchmarks to `OFF`; local native-only build/install/wheel evidence is recorded in `03-evidence-gates.md`.                                                                                        | Local   |
| Default packages and wheels do not carry old collision runtime dependencies.                                                 | Package install probes, Pixi dependency metadata checks, and the `verify_wheel_collision_isolation.py` verifier are recorded. `wheel-verify-core` runs the wheel isolation verifier, and `publish_dartpy.yml` invokes `pixi run -e py${{ matrix.python-version }}-wheel wheel-verify`. | Local   |
| Built-in detector maintains correctness tests.                                                                               | Native unit/integration tests, feature parity tests, DART adapter tests, gz-focused regressions, and reference comparison tests are recorded in `03-evidence-gates.md`.                                                                                                                | Local   |
| Built-in detector maintains benchmarks for performance optimization.                                                         | `pixi run -e collision-reference bm-collision-check` runs checked narrowphase, distance, raycast, mixed-primitive, mesh-heavy, raycast-batch, and public adapter benchmark subsets, writing `.benchmark_results/collision_check_*.json`.                                               | Local   |
| Native performance should beat the best legacy engine on required workloads, with feature first and correctness preserved.   | The current local broad benchmark guard passed at `892e50d02e4`; `README.md` records native wins on the measured primitive, narrowphase, supported distance, raycast, batch, mesh-heavy, and mixed workloads.                                                                          | Local   |
| Built-in layer architecture must be API-clean, scalable, and performance-oriented.                                           | `01-design.md` defines the public API boundary, compatibility shell, DART adapter scene, native scene/query core, query lifecycle, scalability design, performance hooks, and reference harness boundary. `README.md` mirrors this in the Architecture Completion Rubric.              | Local   |
| Architecture evidence must cover code, tests, package/source boundaries, and benchmark/profiling hooks, not only prose docs. | Code evidence exists for factory aliases, compatibility facades, persistent `DartCollisionGroup` scene state, cache IDs, source isolation, wheel verifier wiring, package smoke, adapter benchmarks, and native/reference benchmark JSON. CI/artifact evidence remains missing.        | Partial |
| gz-physics compatibility must be preserved while legacy names migrate.                                                       | Fresh local `pixi run -e gazebo test-gz` passed 65/65 and package smoke passed. `05-downstream-migration.md` defines the compatibility contract and removal gates.                                                                                                                     | Local   |
| Downstream migration/deprecation path must be proven before removing retained facades.                                       | `05-downstream-migration.md` defines the migration order and gates. DART-side package smoke is local evidence only; downstream CI/deprecation evidence is still missing.                                                                                                               | Open    |
| Final PR evidence and cleanup must happen in the same PR.                                                                    | `README.md`, `02-milestones.md`, and `docs/dev_tasks/README.md` require transferring durable evidence to the PR description/onboarding docs and deleting `docs/dev_tasks/native_collision/` only at completion.                                                                        | Open    |

Legend:

- Done: documentation/process requirement is satisfied.
- Local: implemented and locally verified, but still needs final CI or PR
  artifact evidence before north-star completion.
- Partial: some real evidence exists, but coverage is not enough to close the
  requirement.
- Open: required evidence or finalization work is missing.

## Missing Evidence And Required Next Actions

1. Push or otherwise publish the branch and open/update a PR so GitHub CI can
   produce authoritative native-only, gz-physics, wheel matrix, and benchmark
   artifact evidence. The current audit found no PR for `head:feature/new_coll`.
2. Collect CI run links and artifact names for:
   - native-only collision/default build jobs,
   - gz-physics compatibility jobs,
   - dartpy wheel jobs that run `wheel-verify`,
   - `Collision Benchmark Guard` JSON artifacts.
3. Record downstream migration/deprecation evidence proving gz-physics and
   downstream users no longer depend on legacy names as runtime backend
   selectors.
4. Decide whether retained FCL/Bullet/ODE compatibility facades are removed or
   hard-deprecated in the final PR state. Preserve only wrappers required for
   source compatibility, and keep all external engines reference-only.
5. Run final validation after the final code state, including at least
   `pixi run lint` and `pixi run test-all`, plus any CI-specific gates whose
   failures are not covered locally.
6. Transfer durable architecture notes to `docs/onboarding/architecture.md`,
   move final command evidence into the PR description, then delete this
   dev-task folder in the same PR.

## Completion Bar

Do not mark this dev task complete until every row in the checklist is either
`Done` with durable docs evidence or backed by CI/PR artifacts that close the
remaining `Local`, `Partial`, and `Open` statuses. Until then,
`docs/dev_tasks/native_collision/` remains active working documentation.
