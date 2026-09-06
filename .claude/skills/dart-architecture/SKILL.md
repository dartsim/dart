---
name: dart-architecture
description: "DART Architecture: the DART 7 multi-physics, multi-solver, multi-backend simulation pipeline, the living architecture map that draws it, and where each abstraction is owned"
---

# DART 7 Architecture

Load this skill when working on the DART 7 simulation `World`, on
solvers/physics domains/compute backends, when a change adds or renames a
`dart/simulation` module, step-stage slot, or solver family, or whenever a
task needs the big-picture map of how DART 7 is generalized for
multi-physics, multi-solver, and multi-backend simulation.

## The design in one sentence

World owns topology, time and composition. A solver may advance one or several
physical domains, using a shared solve or an explicit coupling strategy.
Semantic dependencies, executable plans and runtime adapters separate physics
from scheduling. Users configure DART-owned method/policy values, never solver
registries, component storage or runtime objects.

## Why three axes of choice

- **Research, apples-to-apples.** A new paper's algorithm should be reproducible
  and benchmarkable _inside_ DART against baselines on shared foundations, not in
  a fork. New methods enter through DART-owned solver _families_.
- **End-user choice.** Users pick the solver method and (internally) backend that
  fit their accuracy/speed/platform needs.
- **Auto-configuration.** Defaults are selected from scene content so the easy
  path stays trivial; the backend seam is designed for later platform/scene-scale
  awareness without changing the public API.

## The living architecture map

The published page `docs/readthedocs/architecture.md` embeds four views that
are rendered at docs-build time from typed JSON under
`docs/assets/architecture/`. The JSON is the source of truth; rendered HTML is
never committed.

| View                                     | Owns                                                                                                                                       |
| ---------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------ |
| `simulation-framework.architecture.json` | facade → scene → selection → schedule → solver families → data/collision → compute; every `dart/simulation/<dir>` except `compute/`        |
| `world-step.dataflow.json`               | the built-in `World::step()` stage slots as nodes (ids are `BuiltInWorldStepStageSlot` names in snake case), flows named by exchanged data |
| `compute-graph.architecture.json`        | semantic graph → executable plan → runtime adapters → evidence; `dart/simulation/compute/**`                                               |
| `library-context.architecture.json`      | every `dart/<module>`, `dartpy`, `dartsim`, external dependencies                                                                          |
| `compute-graph.runtime.json`             | fixture recorded by `tests/unit/simulation/compute/test_architecture_probe.cpp` for the advisory runtime drift check                       |

Read the owning view before changing the code it describes: each node cites
up to three `sources` (path, optional line range), carries a status `tag`, and
the step-flow view's guided views (`meta.views`) show which slots run for the
split rigid, fused multibody, and combined IPC schedules.

**Status tags** are the assessment's labels: `Implemented` (source path and a
scoped test exist), `Partial` (useful implementation with uncovered contract
cells), `Planned` (accepted work without qualifying implementation),
`Undecided` (open design decision). Dashed relationships lead to planned
work.

**Type legend** (archify's fixed palette with DART meanings): `external` =
user-facing facade or third-party library, `frontend` = dartpy or dartsim
surface, `backend` = solver, stage, or compute code, `database` = model,
state, storage, checkpoint, or replay data, `messagebus` = coupling, exchange,
or the step schedule, `cloud` = executor or device runtime, `security` = the
public-API boundary that hides internals.

### Update procedure

A change to a `dart/simulation` module, a `BuiltInWorldStepStageSlot`, an
enumerator of any public selector `enum class` in `world_options.hpp` or
`multibody/multibody_options.hpp` (the gate sweeps every enum there), a
`WorldStepStage` subclass, or a `dart/<module>` directory updates the owning
view in the same change:

1. Edit the view JSON: add or retitle the node, cite its `sources`, set the
   `tag` from the assessment, and label new relationships with the exchanged
   data (dataflow flows) or the mechanism (architecture connections).
2. `pixi run check-architecture-map` (blocking, no Node.js): missing paths,
   out-of-range lines, unresolved symbols, uncovered directories or
   enumerators, unmapped stage classes, and page embeds. Stage classes that
   intentionally have no schedule slot go into `STAGE_CLASS_ALLOWLIST` in
   `scripts/check_architecture_map.py` with a reason.
3. `pixi run render-architecture-map`: archify validation with exact geometry
   diagnostics (label collisions, crossings, readability). Apply the `fixes`
   hint of each diagnostic; showcase acceptance means zero diagnostics. The
   pinned checkout is fetched into `.deps/archify` on first use.
4. `pixi run docs-build` when the page prose changed, then review the built
   architecture page with `pixi run docs-serve` (served on port 8000).
5. Record the change under the packet's or PR's architecture impact line.

### Audit procedure

- `pixi run check-architecture-map` for structure, evidence, and coverage.
- `pixi run check-architecture-map-runtime` compares the committed fixture
  with the views and, when `pixi run build` has produced the
  `test_architecture_probe` binary, with a fresh probe dump. It is advisory;
  pass `--strict` to fail on drift and `--regenerate` after an intentional
  schedule change. Only a profiling-enabled build (the `pixi run build`
  default) records an execution trace; a schedule-only dump is reported
  and is refused for regeneration unless `--allow-schedule-only` is passed.
- Compare each node's `tag` with the current findings in
  `docs/design/dart7_architecture_assessment.md`; the assessment wins.
- Archify upgrades change `ARCHIFY_TAG` and `ARCHIFY_COMMIT` together in
  `scripts/render_architecture_map.py`, then re-render every view and re-check
  the readability diagnostics.

### Authoring constraints that archify enforces

- Architecture views: 6 to 12 components; sublabels at most 28 characters
  (the sublabel font is capped at 9px and must project to 6px at a 1440px
  viewport, so a view stays under about 1390px wide); at most three
  `sources` per component; no crossing connections and no label within 4px of
  another route in showcase mode.
- Dataflow views: at most 5 stages and 5 rows on a fixed 215 by 114 grid with
  112px nodes, so corridors are 103px wide; every flow needs a label; use
  `route: "straight"` for same-row flows, explicit `via` for flows spanning
  more than one stage, and `labelAt` in empty cells or beside corridors for
  labels longer than six characters.

## Design vs current state (read both)

`docs/readthedocs/architecture.md` is the single-page map of the design and
the options at each seam, with honest status markers.
`docs/design/dart7_architecture_assessment.md` is the verified record of
where implementation is partial: complete shared-model states, full CUDA World
stepping, portable continuation, and coverage beyond the existing metrics/corpus.
It owns the continuous audit rule and names the map views as affected
artifacts. New families enter through `docs/plans/solver-family-intake.md`;
`docs/plans/040-dart7-release-hardening.md` coordinates milestone readiness,
and active subsystem plans own packets. PLAN-091 is completed background.
Recheck source evidence before copying a pattern or repeating an absence claim
from an older audit.

## Key owner documents

The architecture page's **Source-of-truth map** is the single owner of the full
topic → owner-doc mapping (solver, API, extension, compute, differentiable,
clean-break, north-star). The docs an agent most often needs inline:

| Topic                                                       | Document                                                                                                                                  |
| ----------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------- |
| Architecture map views, renderer, and checks                | `docs/design/architecture_map.md`, `docs/assets/architecture/`, `scripts/render_architecture_map.py`, `scripts/check_architecture_map.py` |
| Solver abstraction, domain assignment, coupling, schedule   | `docs/design/simulation_solver_architecture.md`                                                                                           |
| Verified findings, standing rule, competitor lessons        | `docs/design/dart7_architecture_assessment.md`                                                                                            |
| Public C++ / dartpy API shape and promotion rules           | `docs/design/simulation_cpp_api.md`, `docs/design/simulation_python_api.md`                                                               |
| CPU / SIMD / GPU decision framework                         | `docs/design/scalable_compute_decisions.md`                                                                                               |
| DART 7 vs DART 6 topology · live progress / readiness gates | `docs/design/dart7_clean_break_strategy.md`, `docs/plans/dashboard.md`                                                                    |

## Public-facade rules (do not violate)

- Do not expose `Solver`, `Coupler`, `PhysicsDomain`, ECS storage, component
  types, concrete runtime/backend types, or solver registries as public API.
  Existing DART-owned abstract executor/stage extension interfaces are distinct
  from concrete runtime implementations.
- Select behavior by documented method-family names and policy value objects.
- A small DART-owned CPU/CUDA preference is accepted public design; library,
  pool, stream, kernel and ISA types remain private. WP-040.2 in
  `docs/plans/040-dart7-release-hardening.md` owns checker-transition
  requirements and status.
- Keep the easy path (`World` + `addRigidBody`/`addMultibody` + `step`) free of
  solver vocabulary.
- Fallbacks must never silently substitute algorithms: validate capabilities
  at finalize or record the substitution in diagnostics.

## Verification

Use `docs/ai/verification.md` to select the docs-only or code gate set for the
change. Map edits add `pixi run check-architecture-map` and
`pixi run render-architecture-map`; the runtime probe runs inside
`pixi run test-unit`. Implementation work that realizes parts of this
architecture also follows the gates in
`docs/design/simulation_solver_architecture.md`, the solver-family intake
checklist, and any active owner plan named in `docs/plans/dashboard.md`.
