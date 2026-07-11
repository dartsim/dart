# DART vs MuJoCo Comparison Harness (v1)

This package benchmarks DART against MuJoCo across a fixed scene matrix so
that native-collision-detector performance work (see
`docs/dev_tasks/dart6_dependency_minimization/`) has a concrete, reproducible
"do we actually win?" signal instead of only internal before/after
comparisons.

It is a **v1**: the scene matrix, step counts, and drive-torque magnitudes
below are considered reasonable starting defaults, not values validated
against a real run (this package was authored without running a build or a
benchmark; see "Known gaps and risks").

## Layout

| File | Role |
| --- | --- |
| `common.py` | Scene-spec dataclasses, JSON IO, the `ResultRow` schema, and MJCF `<option>`/`<actuator>` parsing helpers. No `mujoco`/`dartpy` import, so it loads in either engine's Python environment. |
| `gen_scenes.py` | Deterministic, seeded generator for the box-pile scenes (`pile-120`, `pile-900`, `dyn-stir-120`) and the `ffi-overhead` probe scene. Emits either an MJCF XML string or a JSON body list from the same materialized layout. |
| `mujoco_runner.py` | CLI that loads one scene, steps it with `mujoco.mj_step`, and writes one `ResultRow` JSON. Only file that imports `mujoco`. |
| `dart_runner.py` | CLI mirror of `mujoco_runner.py` using `dartpy`. Only file that imports `dartpy`. |
| `run_comparison.py` | Orchestrator: runs both runners as subprocesses across the scene matrix, aggregates medians across reps, and writes `results.json` + `results.md`. A runner failure marks that scene `blocked` and the rest of the matrix still runs; non-finite runs are reported `inconclusive`, never scored. |
| `fetch_models.py` | Downloads current, license-vetted upstream MJCF models from URLs pinned to release tags, verifies SHA-256 digests, and records per-model license provenance next to the cached file (`.model_cache/`, git-ignored). The bundled `data/mjcf/openai` models are historical gym-era files; per maintainer direction, headline rows should prefer current upstream models. Registered so far: `humanoid` (google-deepmind/mujoco @ 3.3.0, Apache-2.0). Mesh-based models (e.g. MuJoCo Menagerie arms) need multi-asset fetching — tracked in the WS-G lane doc. Point either runner at the cached file via `--scene scripts/mujoco_comparison/.model_cache/humanoid.xml`. |

## Fairness contract

Both runners are built to hold the following constant so that a
steps-per-second/RTF gap reflects engine performance, not a methodology
artifact:

- **Timestep and gravity**: same value in both engines per scene. For the
  three fixed MJCF files (`reacher.xml`, `pusher.xml`, `humanoid.xml`) this
  means the file's own `<option timestep="..." gravity="...">`. **This is
  not automatic on the DART side**: `dart::utils::MjcfParser::createWorld()`
  never reads `<option>` into the `World` it builds (verified against
  `dart/utils/mjcf/MjcfParser.cpp`), so `dart_runner.py` parses the file's
  `<option>` itself (`common.parse_mjcf_option`) and calls
  `world.setTimeStep()`/`world.setGravity()` explicitly after
  `MjcfParser.readWorld()`. Skipping this would silently run DART at its own
  default 0.001s step while MuJoCo runs at the file's step (0.01s for
  reacher/pusher, 0.003s for humanoid), invalidating any comparison. For
  generated scenes, timestep is fixed at 0.001s in `gen_scenes.py`.
- **Box size unit convention**: MuJoCo's box `size` attribute is a
  half-extent; DART's `BoxShape` size is a full extent. `box_half_extent`
  (default 0.1) is the single source of truth; `gen_scenes.to_mjcf_xml` uses
  it verbatim, `dart_runner.py` doubles it. Getting this wrong would
  silently simulate differently-sized boxes per engine.
- **Integrator**: MuJoCo is normalized to `Euler` for headline rows
  (`--integrator euler`), since DART only has a semi-implicit-Euler-style
  integrator with no user-selectable alternative. `run_comparison.py` also
  runs one extra MuJoCo-only rep per MJCF scene with the model's own
  default integrator (RK4 for reacher/pusher/humanoid) as a labeled
  "sensitivity" row — informational, not part of the win/loss verdict.
- **Single-threaded**: `dart_runner.py` calls
  `world.setNumSimulationThreads(1)`; MuJoCo is single-threaded by default
  (no island threading enabled).
- **Default solvers**: neither engine's constraint/LCP solver is
  overridden, only the collision detector (`--detector`, DART-only, see
  below).
- **Warmup excluded**: `--warmup` steps run before timing starts; only
  `--steps` measured steps are included in `wall_s`.
- **Median of reps**: `run_comparison.py` runs `--reps` (default 5)
  repetitions per (scene, engine) and reports the median of `wall_s`,
  `ms_per_step`, `steps_per_s`, and `rtf`.
- **Per-run correctness telemetry**: every `ResultRow` records `finite`
  (whether qpos/qvel, or DART positions/velocities, stayed finite for every
  measured step — the run is stopped early and `steps` reflects the actual
  count if not) and `ncon_mean`/`ncon_max` (contact-count telemetry).
- **FFI overhead row**: `ffi-overhead` is a single free body, no gravity, no
  contact, in both engines. It isolates each engine's fixed per-step
  Python/FFI call overhead from contact-solver cost and is reported
  separately from the 7 scored scenes.

## Scene matrix

| Scene id | Category | Description | Drive | Sleep |
| --- | --- | --- | --- | --- |
| `ARM-REACHER` | MJCF file (`data/mjcf/openai/reacher.xml`) | 2-DOF planar arm | torque-driven | on |
| `ARM-PUSHER` | MJCF file (`data/mjcf/openai/pusher.xml`) | 7-DOF arm + free object | torque-driven | on |
| `HUM-FALL` | MJCF file (`data/mjcf/openai/humanoid.xml`) | passive humanoid drop (explicit opt-in; blocked by DART 6.20 stacked-hinge parsing) | none | on |
| `HUM-ACTIVE` | MJCF file (`data/mjcf/openai/humanoid.xml`) | humanoid with sinusoidal joint torques (explicit opt-in; blocked by DART 6.20 stacked-hinge parsing) | torque-driven | on |
| `PILE-120` | Generated | 120-box pile settling in a walled container | none | on |
| `PILE-900` | Generated | 900-box pile settling in a walled container | none | on |
| `DYN-STIR-120` | Generated | 120-box pile swept by a kinematic bar | kinematic stirrer (not torque) | off (both engines) |
| `FFI-OVERHEAD` | Generated | 1 free body, no gravity, no contact | none | off |

For `PILE-120`/`PILE-900`/`DYN-STIR-120`, `run_comparison.py` uses
100 warmup steps + 2000 measured steps, matching the "measured during active
settling phase" requirement — the pile is still actively colliding/settling
during the measured window, not already at rest.

The default matrix excludes `HUM-FALL` and `HUM-ACTIVE` because DART 6.20's
MJCF parser does not support the model's stacked hinge joints. They remain
selectable with an explicit `--scene` for parser-development validation.

### Drive-torque design (MJCF-file scenes)

DART's MJCF parser ignores `<actuator>` entirely, and even if it didn't,
MuJoCo's per-actuator `gear` scaling has no DART-side equivalent. So instead
of trying to replicate MuJoCo's actuator semantics, both runners:

1. Parse `<actuator><motor joint="...">` from the MJCF file
   (`common.parse_mjcf_actuator_joint_names`) to get the set of driven joint
   names — this is the only thing taken from `<actuator>`.
2. Derive a deterministic phase offset per joint name from a seeded RNG
   (`common.derive_joint_phases`), keyed by joint *name* so ordering
   differences between the two engines' internal joint enumeration don't
   matter.
3. Apply `amplitude * sin(2*pi*frequency*t + phase)` directly as a
   generalized force on that joint's degree of freedom every step —
   MuJoCo via `data.qfrc_applied`, DART via
   `DegreeOfFreedom.setForce()`. Both assume the driven joints are
   single-DOF hinge/slide joints (true for reacher/pusher/humanoid's
   actuator targets).

`amplitude`/`frequency`/`seed` are harness parameters
(`--drive-amplitude`/`--drive-frequency`/`--drive-seed`), not values read
from the MJCF file. `run_comparison.py`'s current defaults (1.0 N·m for
reacher, 5.0 N·m for pusher, 40.0 N·m for humanoid, all at 0.5 Hz) are
placeholders sized by rough order-of-magnitude reasoning about each model's
mass scale, not tuned against an actual run.

### Generated pile layout (`gen_scenes.py`)

Boxes are stacked in horizontal "sheets" above a walled container and
dropped under gravity; the footprint is capped at a 10x10 grid, so denser
scenes (`PILE-900` vs `PILE-120`) add more stacked sheets rather than a
wider, mostly non-interacting monolayer — this keeps the pile genuinely
contact-rich rather than a flat single layer. A small seeded per-box
positional jitter avoids a perfectly regular initial lattice. `DYN-STIR-120`
adds a kinematic bar spanning the container diameter, swept about a fixed
vertical pivot at constant angular velocity (`StirrerSpec`); it is
represented as a MuJoCo `mocap` body and a `mobile=false` DART `Skeleton`,
with its pose recomputed from elapsed time every step in both runners — it
never integrates dynamically or responds to contact forces.

### Result row schema

```json
{
  "scene": "PILE-120",
  "engine": "dart",
  "config": "headline",
  "steps": 2000,
  "wall_s": 1.234,
  "ms_per_step": 0.617,
  "steps_per_s": 1620.0,
  "rtf": 1.62,
  "ncon_mean": 214.3,
  "ncon_max": 402,
  "finite": true,
  "sleeping_bodies": 0,
  "metadata": { "...": "engine/scene-specific extras" }
}
```

## Usage

Generate a scene spec directly (mostly for inspection/debugging;
`run_comparison.py` does this internally):

```sh
python scripts/mujoco_comparison/gen_scenes.py --scene pile-120 --seed 0 \
    --out /tmp/pile-120.json
python scripts/mujoco_comparison/gen_scenes.py --scene pile-120 --seed 0 \
    --out /tmp/pile-120.xml
```

Run a single engine on a single scene:

```sh
python scripts/mujoco_comparison/mujoco_runner.py \
    --scene data/mjcf/openai/reacher.xml --steps 2000 --warmup 100 \
    --integrator euler --drive on --drive-amplitude 1.0 \
    --out /tmp/reacher-mujoco.json

python scripts/mujoco_comparison/dart_runner.py \
    --scene data/mjcf/openai/reacher.xml --steps 2000 --warmup 100 \
    --detector default --drive on --drive-amplitude 1.0 \
    --out /tmp/reacher-dart.json
```

Run the full matrix (both engines assumed reachable via `python` on `PATH`;
override with `--dart-python`/`--mujoco-python` if they live in separate
pixi environments):

```sh
python scripts/mujoco_comparison/run_comparison.py \
    --reps 5 --out-dir .mujoco_comparison_results
```

This writes `.mujoco_comparison_results/results.json` (raw + aggregated) and
`.mujoco_comparison_results/results.md` (a per-scene verdict table: "DART
wins" / "DART loses" / "within noise band (1.1x)").

## DART collision detectors (`--detector`)

`dart_runner.py --detector {default,dart,fcl,bullet,ode,native}` selects the
collision backend via
`world.getConstraintSolver().setCollisionDetector(...)`:

- `default` leaves the `World`'s built-in default detector untouched.
- `dart`/`fcl`/`bullet`/`ode` construct
  `dartpy.collision.{DART,FCL,Bullet,Ode}CollisionDetector()`; `bullet`/`ode`
  exit with a clear error if the optional backend wasn't compiled in.
- `native` constructs `dartpy.collision.NativeCollisionDetector()`, which is
  bound by this change. Use `--detector native` to run the comparison with
  DART's opt-in native collision backend.

`run_comparison.py` uses `default` for every scenario in this v1 (no
per-detector sweep yet); pass `--detector <name>` to override every
scenario's detector for a single run, or invoke `dart_runner.py` directly
per-detector for a manual matrix.

## Sleeping bodies (`sleeping_bodies` field)

DART: `--sleep {on,off}` controls
`world.setDeactivationOptions(DeactivationOptions(mEnabled=...))`.
`sleeping_bodies` is populated on a best-effort basis via
`Skeleton.isResting()` — as of this writing, `isResting()` is **not bound in
dartpy**, so `dart_runner.py` returns `None` for this field until a future
dartpy build exposes it (the runner checks via `getattr` and degrades
gracefully rather than failing).

MuJoCo has no sleeping/deactivation concept, so `sleeping_bodies` is always
`None` on `"mujoco"` rows.

## Known gaps and risks (v1)

- **Limited runtime validation.** The one-repetition smoke covers
  `ARM-REACHER`, `PILE-120`, and `DYN-STIR-120`; the larger matrices and
  representative multi-repetition benchmark runs still need validation on a
  quiet host before their results are treated as evidence.
- **`Skeleton.isResting()` isn't bound in dartpy yet** — DART
  `sleeping_bodies` is `None` until that binding lands.
- **MJCF parser fidelity**: DART's MJCF parser may not fully honor
  `contype`/`conaffinity` collision filtering or per-geom friction override
  the way `reacher.xml`/`pusher.xml` use them (several geoms in those files
  set `contype="0"`/`conaffinity="0"` to exclude cosmetic geoms from
  collision). If DART collides geoms MuJoCo excludes (or vice versa), the
  two engines are not simulating an identical scene for `ARM-REACHER`/
  `ARM-PUSHER`, independent of any performance difference. Cross-check
  `ncon_mean`/`ncon_max` between engines on these two scenes before trusting
  their verdict.
- **Step counts, warmup, and drive-torque magnitudes are placeholders.**
  The team task did not pin these; `run_comparison.py`'s `SCENARIOS` table
  uses a uniform 100-warmup/2000-step window and order-of-magnitude drive
  torques (see "Drive-torque design" above). Revisit after the first real
  run.
- **Container/pile sizing heuristic** (`gen_scenes._pile_layout`): the
  10x10-grid-cap-plus-stacked-sheets footprint/wall-height formula is an
  engineering placeholder, not derived from any target packing density or
  validated against real settling behavior.
- **No per-detector sweep yet.** `run_comparison.py` only exercises
  `--detector default` per scenario; comparing `fcl`/`dart`/`bullet`/`ode`/
  `native` against MuJoCo per scene is future work (tracked as a follow-on
  to closing the performance gap, not part of this v1).
- **Restitution and MuJoCo torsional/rolling friction** are left at each
  engine's own defaults (DART: 0 restitution; MuJoCo: default `solref`/
  `solimp` and torsional/rolling friction terms) since neither engine
  supports the same friction-cone model beyond isotropic sliding friction,
  which is the coefficient actually held constant (`friction` field/
  `box.friction`) between engines.
