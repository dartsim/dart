# dart-demos (Python)

A scene-registry runner that hosts DART's Python demo scenes in the shared
Filament viewer. This is the Python-first World demo surface from PLAN-103; C++
`dart-demos` (PLAN-102) is the smaller World-only C++ companion.

## Run

```bash
pixi run py-demos                                # default: open rigid_body
pixi run py-demos -- --scene rigid_solver_compare # compare SI vs IPC visually
pixi run py-demos -- --scene replay_scrubber     # open the replay timeline
pixi run py-demos -- --scene articulated         # select any scene by id
pixi run py-demos -- --scene gui_fidelity_debug_visuals
pixi run py-demos -- --cycle-scenes --frames 4   # cycle through every scene
pixi run py-demos -- --list                      # print the scene catalog
```

Or directly (without pixi), from the repo root:

```bash
PYTHONPATH=build/default/cpp/Release/python:python \
    .pixi/envs/default/bin/python -m examples.demos --scene rigid_body --frames 5
```

## CLI

The runner mirrors C++ `dart-demos` to keep the cross-language UX consistent:

| Flag                | Meaning                                                    |
| ------------------- | ---------------------------------------------------------- |
| `--scene <id>`      | Select the initial scene (default: `rigid_body`)           |
| `--cycle-scenes`    | Advance through every scene for `--frames` frames and exit |
| `--frames N`        | Per-scene step budget (default 60 single, 4 cycle)         |
| `--screenshot PATH` | Write the final rendered frame as a binary PPM image       |
| `--out DIR`         | Write a numbered binary PPM frame sequence                 |
| `--headless`        | Render without opening an interactive window               |
| `--show-ui`         | Include ImGui panels in headless screenshots               |
| `--list`            | Print the catalog and exit                                 |

In cycle mode, `--frames N` is the per-scene budget. The host prints
`Cycling demo scene <i>/<N>: <scene_id>` as it advances, which keeps long
headless or live-window catalog smokes diagnosable.

`--screenshot` and `--out` use the real Filament render path. For visual
debugging, avoid `--backend noop`: it can exercise CPU code but produces blank
pixels and is not evidence for layout, camera, lighting, or material quality.

## Visual debugging

Capture one frame:

```bash
pixi run py-demo-capture -- --scene rigid_body --frames 2 \
    --width 640 --height 360
```

The helper writes a PPM, converts it to PNG, rejects blank captures, and prints
the artifact paths. On Linux it defaults to the same software Mesa settings used
by GUI tests, so captures stay useful on dev hosts where the desktop GL driver
is unavailable. With `--show-ui`, it also rejects screenshots that do not show
the docked workspace and drops early warm-up frames before ImGui is visible from
the converted frame sequence. For scenes in the maintained rigid workflow, the
single-scene `manifest.json` also records `workflow_guidance` with the row
number, role, workflow phase, focus axis, user question, try-first action,
inspect signals, healthy signal, and scope note. Capture the docked ImGui
workspace:

```bash
pixi run py-demo-capture -- --scene rigid_solver_compare --show-ui --frames 2 \
    --width 1280 --height 720
```

Capture motion evidence for a solver comparison:

```bash
pixi run py-demo-capture -- --scene rigid_solver_compare --frames 72 \
    --width 960 --height 540 --show-ui --video --fps 24
```

The docked workspace has a top `Simulation` toolbar, a searchable `Demos`
navigator, scene-specific panels on the right, bottom scene panels when a demo
owns timeline controls, and a docked DART diagnostics panel. Use `Rebuild` to
reload the active scene, `Restart` to reload and run it from the beginning, and
`Layout` to restore the default docks after rearranging panels. When frame
capture is active, the `Simulation` panel also exposes a timeline scrubber over
the recorded PPM sequence with first/previous/play/next/last controls and the
selected frame path.

The first 36 **World Rigid Body** entries in the interactive `Demos` navigator
are prefixed with their workflow position and role, such as
`01/36 Baseline: World Rigid Body` or
`15/36 Solver family: Rigid Solver Compare`. `--list` keeps the stable scene
titles and ids for scripts.

World-backed scenes also get a bottom `Replay` panel. `Save replay` is enabled
by default and records bounded DART 7 World state snapshots while the
scene runs. Use the replay transport or scrubber to pause the live simulation,
restore a saved frame, and play the saved states without stepping physics again;
`Resume live` continues simulation from the selected restored state. Scenes with
live controller state outside the `World` can provide small replay-state
capture/restore callbacks; the shared panel stores those mutable controller
snapshots beside the World frames instead of storing static scene assets.

## Rigid body visual verification workflow

The rigid-body rows are ordered as a short debugging path instead of a flat
feature list. Start with **`rigid_body`** for default World rigid dynamics,
use **`rigid_body_modes`** when dynamic/static/kinematic mode semantics need a
first check, use **`rigid_free_flight`** when contact-free initial velocity,
gravity, spin, inertia, momentum, or energy diagnostics need inspection, use
**`rigid_frame_hierarchy`** when body-fixed sensor/tool frames, local transforms,
world transforms, or relative transforms need inspection, use
**`rigid_external_loads`** when force and torque accumulator response needs
inspection alongside direct rigid-body impulse momentum, use
**`rigid_link_point_loads`** when off-center force application and world/local
frame semantics need inspection, use
**`rigid_timestep_sensitivity`** when `World.time_step` or gravity tuning
changes integration error, contact timing, clearance, or step cost, use
**`rigid_step_diagnostics`** when per-stage step profiling, ECS counters, or
frame-scratch memory need inspection across scene complexity, use
**`rigid_contact_scale_budget`** when contact-count scaling, per-contact cost,
scratch usage, or a user frame budget needs inspection, use
**`rigid_restitution_ladder`** when material bounce response needs inspection,
use **`rigid_material_mixing`** when body/surface material ownership or pair
mixing rules need inspection, then use **`rigid_contact_inspector`** when raw
contact pairs need inspection, use **`rigid_collision_query_options`** when
`World.collide(options)` body-kind filtering or ignored pairs need inspection, use
**`rigid_collision_casts`** when raycast or swept-sphere hit queries need
inspection, including swept-capsule link/tool proxies and capture metrics, use
**`rigid_solver_compare`**, **`rigid_executor_equivalence`**, and
**`rigid_contact_solver_compare`** when the question is solver family, executor
parity, or contact policy, then use **`contact`** when multibody links need to
drop, slide, or push through solver contact. Continue through the focused
threshold, stability, manipulation, kinematic-driver, joint, distance-spring,
dynamics, and loop-closure rows when those behaviors need inspection.
**World Rigid Body**
here means World-facade rigid debugging: solver-family rows compare sequential
impulse and IPC, executor-equivalence rows hold the selected physics solver
constant across executors, and focused threshold or stability rows may pin one
solver or contact policy when that is the user question. The older **Rigid IPC**
category remains a solver-specific capability shelf for one-off IPC contact
cases such as edge-drops, piles, no-tunneling, and capture-first stack packets.

In the interactive viewer, every numbered **World Rigid Body** row also gets a
compact `Rigid Workflow` panel. It mirrors the maintained PLAN-103 question for
the current row as a small checklist: the workflow phase, the row's focus axis,
what to try first, what to look for, and what not to infer from that row. The
workflow phases group rows into foundations; diagnostics; contact, material,
and query basics; the solver decision path; contact behavior cases; rigid
constraints and joint mechanics; and multibody dynamics and kinematics. The
focus axis names the main comparison or debugging dimension, such as solver
family, executor, contact-policy, workload size, passive joint parameters,
backend/profile diagnostics, or loop-closure family, before the user opens a
scene. The panel also has selectable previous/next numbered rows, a restart
command, a direct row selector, a per-row
live `py-demos --scene` open command, a paired `py-demo-capture` command with
frame/resolution/UI settings, and a ranked text filter over row ids, scene ids,
labels, questions, workflow phases, focus axes, signals, and explicit aliases
such as
`RigidBodySolver`, `SI`, `boxed LCP`, `ContactSolverMethod`, `worker count`,
`accelerated backend`, `compute backend`, `CUDA`, `GPU`, `CUDA backend`,
`GPU backend`, `backend comparison`, `Taskflow executor`,
`executor comparison`, `semi-implicit`, `variational solver`,
`joint damping`, `joint friction`, `throughput`, `latency`, `raycast`,
`swept sphere`, `friction coefficient`, `coefficient of restitution`,
`time-of-impact`, `CollisionQueryOptions`, `generalized force`, `coriolis`,
`compute_impulse_response`, `RigidBody.apply_linear_impulse`, `stack jitter`,
`closed chain`, `closed loop`, `direct rigid body impulse`, `sleep wake`,
`island activation`, and `loop closure compliance` that request in-viewer scene
switches.
The filter folds punctuation, underscores, hyphens, dotted API names,
CamelCase, compact API tokens, and simple plurals, so queries such as
`RigidBody.applyLinearImpulse`, `Multibody.computeImpulseResponse`,
`ray-cast`, `shape-cast`, `body kind filters`, or `resting contacts` find the
same rows as their documented snake_case or singular forms.
Search-result tooltips name the match source, such as maintained alias, row
number, user question, workflow phase, focus axis, related evidence, or scope
caveat, so users can see why the navigator chose a row before switching scenes.
Deferred public-API searches such as `sleep wake`, `island activation`, and
`loop closure compliance` also show a deferred API caveat in the tooltip, guide
payload, workflow manifest, and static review card before opening or reviewing
the closest current verifier row. Workflow manifests include
`deferred_api_caveat_summary`, and review indexes include a top-level
`Deferred API Caveats` table and badge for packet triage.
Broad queries keep the list compact by showing the top six ranked rows, but the
panel also states the full match count so users can tell when a term like
`contact` or `solver` has more rows than are visible.
Backend-status terms route to
`rigid_step_diagnostics`, while executor terms route to the same-solver
`rigid_executor_equivalence` row. Scope caveats
remain visible in the row, so deferred public-API searches route to the closest
current row without claiming unsupported activation or compliance behavior.
Direct rigid-body impulse API searches route to the public load/impulse row,
while multibody impulse-response API searches route to the generalized dynamics
row. The filter ranks positive intent matches first so searches such as
`contact`, `solver`, `step profile`, `backend comparison`,
`GPU backend`, `variational solver`, `joint damping`, `raycast`,
`friction coefficient`, `CollisionQueryOptions`, `generalized force`,
`compute_impulse_response`, `closed chain`,
`executor comparison`, and `sequential impulse` do not get dominated by early
rows that only mention what not to infer. Related-evidence searches such as
`rigid_ipc_tunnel`,
`rigid_ipc_edge_drop`, or `avbd prismatic` open the related shelf scene directly
through the maintained route table. Comparison and parameter rows also label the
comparison axis and held-fixed controls in the panel and capture metrics so
solver, executor, contact-policy, workload-size, and parameter-family changes
are not conflated.
The full workflow capture writes a top-level `review_index.html` contact sheet
next to `manifest.json` so all numbered screenshots, per-scene manifests,
frame directories, live open commands, workflow-row rerun commands, direct
capture commands, and metric summaries can be reviewed from one page. Rows with
scene-owned Replay timeline metadata also write a JSON-safe
`scene_metadata.replay_timeline` summary into each per-scene manifest, and the
review card names the Replay track plus whether it has signal and marker
tracks. The top-level manifest and review header record the exact workflow
packet command that produced the artifact, so a reviewer can rerun the same
selected packet. Workflow manifests also record `workflow_phase_summary`, and
the review header shows a `Workflow Phase Map` with selected numbered row
ranges, phase count, per-phase status, focus axes, and scene ids so row-range
packets remain readable at a glance. Successful scene captures also promote a
machine-readable
`resolved_solver_identity` block from the latest scene metrics into the
per-scene manifest; workflow manifests count those identities and list any
captured row missing one, while the review cards show the resolved solver
summary beside the row's workflow phase, focus axis, comparison axis, and
controls.
Workflow packets require that identity to include both a solver-family field
and a context field such as
executor, contact method, same-solver marker, or held-fixed configuration.
Workflow manifests also count rows with latest scene metrics and list any
captured row missing metrics. Non-dry-run workflow packets return failure status
when a captured row misses scene metrics or solver identity evidence, and the
review index highlights those rows before a reviewer treats the packet as
complete. Review cards also promote nested step-diagnostics lane metrics into
a compact backend-diagnostics line, so backend activity, accelerated-stage
counts, worker counts, top stages, and stage timings are visible without
opening the raw manifest JSON. Contact-query and collision-cast review cards
also promote selected contact/depth, query-filter, ignored-pair, ray hit, swept
probe, time-of-impact, and cast-margin values into `latest signals`, so
reviewers can scan the query rows without opening raw manifests. The panel also
includes a current-row motion packet command; workflow
packets pass `--video --fps` through to the selected row captures and the review
index links MP4 artifacts when `ffmpeg` is available.
Extended workflow packets also keep optional related-evidence, direct Rigid IPC
shelf, and capture-first packet rows self-describing in `manifest.json` and
`review_index.html`: each optional row records its role, user question,
try-first action, inspect signals, healthy signal, and scope note. The manifest
also records guidance completeness, and the review index shows a guidance badge
plus a warning block if any selected row is missing those fields.

| Order | Scene id                         | User question                                      | Primary controls                                      | Visual diagnostics                                            |
| ----- | -------------------------------- | -------------------------------------------------- | ----------------------------------------------------- | ------------------------------------------------------------- |
| 01/36 | `rigid_body`                     | What is the baseline World rigid-body path?        | Solver, materials, replay, force drag, reset          | Speed, height, energy, contacts, step timing                  |
| 02/36 | `rigid_body_modes`               | Which body mode should I choose?                   | Solver, executor, gravity, force, kinematic speed     | Dynamic fall, static drift, kinematic path error              |
| 03/36 | `rigid_free_flight`              | Do initial velocity, gravity, and spin evolve?     | Executor, launch speed/angle, gravity, spin/inertia   | Path error, momentum residual, energy drift, spin ratio       |
| 04/36 | `rigid_frame_hierarchy`          | Where is a sensor/tool frame on a moving body?     | Executor, body yaw/path, local offset/yaw             | Parent frame, world pose, relative/world residuals            |
| 05/36 | `rigid_external_loads`           | How do loads and impulses move/spin bodies?        | Force/torque, direct impulse, mass/inertia, executor  | Acceleration, impulse momentum, angular speed, static drift   |
| 06/36 | `rigid_link_point_loads`         | Do point forces create lever-arm torque?           | Force, point offset, body yaw, executor               | Translation, yaw acceleration, pulse clear, frame split       |
| 07/36 | `rigid_timestep_sensitivity`     | How does time step size change a drop?             | Solver, executor, base time step, gravity scale       | Free-fall error, contact timing, clearance, step time         |
| 08/36 | `rigid_step_diagnostics`         | Where does a World step spend time and memory?     | Solver, executor, reset                               | Stage/domain/backend status, ECS, scratch, contacts           |
| 09/36 | `rigid_contact_scale_budget`     | How much contact fits in my frame budget?          | Solver, executor, frame budget, friction              | Contact count, wall ms, per-contact cost, scratch             |
| 10/36 | `rigid_restitution_ladder`       | How does restitution change bounce height?         | Solver, executor, launch height, restitution scale    | Height, vertical speed, contact, energy trend                 |
| 11/36 | `rigid_material_mixing`          | Which material owns bounce or friction response?   | Impact/tangent speed, low/high e and mu, executor     | Effective max restitution, sqrt friction, swap deltas         |
| 12/36 | `rigid_contact_inspector`        | Which contact pairs and manifold fields exist?     | Shape pair, penetration                               | Contact count, point, normal, depth, shape ids, metrics       |
| 13/36 | `rigid_collision_query_options`  | Which body-kind pairs does a query include?        | Query toggles, ignored-pair selector                  | Active/ignored contacts, body kinds/casts, shape ids          |
| 14/36 | `rigid_collision_casts`          | Where do rays and swept probes hit?                | Ray offset, all-hit, sphere/capsule sweep controls    | Ray fractions, TOI, hit point/normal, cast margins, metrics   |
| 15/36 | `rigid_solver_compare`           | What changes between sequential impulse and IPC?   | Executor, launch speed, friction, restitution         | Speed, wall clearance, solver divergence, step time           |
| 16/36 | `rigid_executor_equivalence`     | Does parallel execution preserve the same physics? | Physics solver, launch speed, friction, restitution   | Pose/velocity divergence, contact count, step time, metrics   |
| 17/36 | `rigid_contact_solver_compare`   | What changes when contact solver policy changes?   | Executor, launch speed, friction, restitution, tilt   | Contact count, depth, clearance, speed, divergence            |
| 18/36 | `contact`                        | Do articulated links contact like rigid bodies?    | Executor, friction, restitution, drop/slide/push      | Link contacts, rebound, slide travel, target travel           |
| 19/36 | `rigid_friction_threshold`       | Where is the stick/slip boundary?                  | IPC fixed, executor, ramp angle, controlled friction  | Friction-threshold axis, drift, speed, clearance, metrics     |
| 20/36 | `rigid_spin_roll_coupling`       | Does friction couple sliding and spin?             | SI fixed, executor, friction, speed, backspin         | Spin/roll axis, slip speed, roll ratio, spin change, metrics  |
| 21/36 | `rigid_stack_stability`          | Does a top-heavy stack jitter or collapse?         | SI vs IPC, executor, top mass ratio, friction         | Solver-family axis, top drift, clearance, divergence, metrics |
| 22/36 | `rigid_contact_manipulation`     | Can a pusher move an object through contact?       | SI vs IPC, pusher speed, friction, pusher mass        | Pusher-contact axis, travel divergence, contact/proximity     |
| 23/36 | `rigid_kinematic_driver`         | Does prescribed motion carry objects by contact?   | IPC grip/slip vs SI caveat, speed, friction, executor | Tangential axis, box travel, slip, caveat, contacts           |
| 24/36 | `rigid_kinematic_normal_push`    | Can prescribed normal motion push a target?        | IPC vs SI caveat, push speed, target mass, executor   | Normal-contact axis, travel divergence, depth, contacts       |
| 25/36 | `rigid_fixed_joint`              | Does a fixed joint preserve its captured pose?     | Sequential joints, static base, perturbation, reset   | Fixed-transform axis, pose error, payload speed, Replay marks |
| 26/36 | `rigid_joint_breakage`           | What happens when a fixed joint breaks?            | AVBD joints, weak break force, static base            | Break-force axis, broken state, release distance, reset       |
| 27/36 | `rigid_distance_spring`          | How do distance springs enforce rest length?       | Executor, rest length, initial stretch, gravity       | Distance-spring axis, lane stretch, offset spin               |
| 28/36 | `rigid_limited_joints`           | Do one-DOF joints keep only their free axis?       | Sequential joints, static bases, z axis               | One-DOF axis, locked errors, free-axis motion                 |
| 29/36 | `rigid_joint_motor_limits`       | Do joint motors and limits clamp commands?         | World joints, velocity/position/effort limits         | Actuator axis, motor clamp, force gap                         |
| 30/36 | `rigid_joint_passive_parameters` | Do passive joint parameters shape motion?          | Executor, spring/rest/damping, friction, drive forces | Passive axis, energy decay, slip, armature lag                |
| 31/36 | `rigid_screw_joint_pitch`        | Does screw pitch couple rotation and translation?  | World screw joints, pitch, mass, inertia              | Screw axis, pitch ratios, travel gap, reverse sign            |
| 32/36 | `rigid_multibody_dynamics_terms` | What do generalized dynamics terms mean?           | World dynamics, acceleration, impulse, gravity        | Dynamics axis, coupling, torque gap, response ratio           |
| 33/36 | `rigid_link_center_of_mass`      | How do COM offsets change gravity torque?          | World links, COM offset, mass, gravity                | COM axis, torque sign, mirror sum, inertia lag                |
| 34/36 | `rigid_link_jacobian`            | What does a link Jacobian map?                     | World Jacobian motion, elbow phase, wrench controls   | Jacobian axis, link speed, gap, torque, residual              |
| 35/36 | `rigid_multibody_solver_family`  | Which multibody solver family supports solves?     | Multibody solve policy, gravity, executor             | Solver-family axis, residual ratio, lane errors               |
| 36/36 | `rigid_loop_closure`             | Which loop-closure family should I use?            | Closure family/policy, gravity, executor              | Closure axis, ratios, family-specific residuals               |

For the focused IPC no-tunneling capability view, use
**`rigid_ipc_tunnel`** from the **Rigid IPC** shelf. It is kept outside the
numbered World Rigid Body workflow because it is an IPC-only capability scene,
not a broad side-by-side solver row. In the viewer, the `Rigid Solver Compare`
row exposes it as a `Related shelf` route. Its capture metrics report wall
clearance, through-wall margin, velocity, contact count, step timing, and
whether the IPC barrier held the fast box before it tunneled through the thin
wall.

For the focused IPC degenerate edge-contact capability view, use
**`rigid_ipc_edge_drop`** from the **Rigid IPC** shelf. It is also routed from
`Rigid Solver Compare` as related evidence. Its panel and capture metrics report
barrier gap, tilt, angular speed, contact count, step timing, and edge-barrier
status as a cube tips near a single edge while IPC keeps positive separation.

The rest of the **Rigid IPC** shelf is also capture-metric backed for direct
inspection. `rigid_ipc` records basic box/ground barrier-settle height, speed,
gap, contact, and timing evidence; `rigid_ipc_slide` records friction-braked
tangential travel and barrier gap; `rigid_ipc_incline` records tilted-ramp
down-slope speed, travel, and ramp gap; and `rigid_ipc_pile` records multi-box
pile speed, height, clearance, span, contact count, and timing evidence. These
direct shelf scenes keep shared replay controls enabled while recording those
metrics.

The older **`floating_base`** and **`articulated`** rows stay in the broader
**World Rigid Body** catalog outside the numbered verifier block. In the
viewer, `Rigid Free Flight` routes to `floating_base` for the floating-joint
SE(3) drift/spin example, and `Rigid Multibody Dynamics Terms` routes to
`articulated` for the compact two-link arm example. Their capture metrics keep
the related routes from being screenshot-only evidence by recording
floating-joint speed/position histories and articulated arm speed, height, and
damping diagnostics.

For AVBD-specific rigid constraint variants, use the **AVBD Rigid Constraints
(sx)** shelf. In the viewer, the `Contact`, `Rigid Joint Breakage`, and `Rigid
Joint Motor Limits` rows expose `Related shelf` routes to
**`avbd_rigid_fixed_joint_contact`**, **`avbd_rigid_breakable_joint`**,
**`avbd_rigid_spherical_breakable_joint`**, **`avbd_rigid_revolute_motor`**,
and **`avbd_rigid_prismatic_motor`**. These remain outside the numbered World
Rigid Body workflow because they are AVBD capability rows, not broad World
solver comparisons.

## Rigid body baseline

The **`rigid_body`** scene is the default front door for DART 7 rigid-body
`World` dynamics. It keeps the first run simple: falling spheres and a box,
static ground, live replay, viewport force drag, solver/material controls, an
explicit reset path, and a compact panel for baseline speed, height, energy,
contact, and step-timing state. Use the focused rows below when a material,
contact-query, solver, executor, friction, stacking, manipulation, body-mode,
kinematic, external-load, point-load, time-step, or joint behavior needs deeper
inspection.

The **`rigid_body_modes`** scene compares the three public rigid-body modes in
one contact-free World. The dynamic lane integrates gravity and force, the
static lane keeps its transform fixed while still reporting the applied load,
and the kinematic lane follows a prescribed path while exposing the mode flag
and path error. The panel and capture metrics now label the rigid-body mode
semantics comparison axis and held-fixed solver/executor/gravity/force/body-mass
context before the lane diagnostics. Use `rigid_kinematic_driver` later in the
workflow when the question is contact-driven prescribed motion.

## Rigid free flight

The **`rigid_free_flight`** scene keeps contacts out of the question while
showing initial rigid-body state evolution directly. Separate no-contact lanes
show zero-gravity linear drift, a gravity arc against analytic trajectory and
momentum references, and low/high-inertia spin bars with the same angular
velocity. The panel exposes executor, launch speed, launch angle, gravity
scale, spin speed, inertia ratio, position error, momentum residual, energy
drift, spin momentum/energy ratios, contact count, and step timing.

## Rigid frame hierarchy

The **`rigid_frame_hierarchy`** scene keeps contacts and forces out of the
question while making rigid frame composition visible. A kinematic rigid body
moves on a simple path with a body-fixed sensor/tool frame attached by a local
offset and yaw. The panel exposes executor, body yaw speed, path radius, local
offset, local yaw, parent-frame name, body and sensor world pose, world-transform
residual, relative-transform residual, orientation error, and step timing.

## Rigid external loads

The **`rigid_external_loads`** scene keeps contacts and gravity out of the
question so users can inspect external rigid-body load semantics directly.
Matched lanes show persistent `apply_force()` response on light and heavy
bodies, direct linear impulse response through
`RigidBody.apply_linear_impulse()`, a one-step force pulse that is explicitly
cleared, persistent `apply_torque()` response on low- and high-inertia bodies,
direct angular impulse response through `RigidBody.apply_angular_impulse()`,
and a static body that retains force/torque accumulators while remaining fixed.
The panel exposes force magnitude, torque magnitude, direct linear impulse,
direct angular impulse, mass ratio, inertia ratio, executor choice, speed,
linear impulse momentum, acceleration versus expected acceleration, angular
response, angular impulse momentum, static drift, and step timing.

## Rigid link point loads

The **`rigid_link_point_loads`** scene covers the public one-shot
`Link.apply_force(force, point, force_in_world_frame, point_in_world_frame)`
path for floating rigid links. It complements `rigid_external_loads` by showing
that centered forces translate without spin, world-space off-center point
forces add lever-arm torque, two force applications before one `World.step()`
accumulate, a one-shot point force is consumed by the next step, and yawed
links respond differently to world-frame and local-frame forces. The panel
exposes force magnitude,
point offset, yaw angle, executor choice, translation acceleration versus
expected acceleration, yaw acceleration versus expected yaw acceleration,
displacement, and step timing.

## Rigid time-step sensitivity

The **`rigid_timestep_sensitivity`** scene makes time-step and gravity tuning
visible without changing the displayed simulation-time advance per frame.
Three matched Worlds drop identical spheres with fine, medium, and coarse
integration steps while using substeps so all lanes reach the same displayed
time. The panel exposes solver, executor, base time step, gravity scale,
free-fall error against the analytic no-contact reference, clearance, first
contact time, contact count, coarse/fine error ratio, and step-profile timing.
It is a convergence and parameter-sensitivity diagnostic, not a solver
correctness proof or exact contact-threshold claim.

## Rigid step diagnostics

The **`rigid_step_diagnostics`** scene makes `World.last_step_profile` and
`World.memory_diagnostics` visible in the GUI. Three side-by-side Worlds run a
single free body, an active contact pair, and a small stack under the same
selected solver and executor so users can inspect how scene complexity changes
wall time, stage totals, top stage, worker count, ECS entity/component counts,
contact count, frame-scratch peak usage, and overflow/reset counters. The top
stage also reports its domain, backend-neutral acceleration mask, and whether
any accelerated backend stage was active, so users can separate executor choice
from actual accelerator use. If step profiling is compiled out, the row still
reports memory and contact diagnostics and marks profile timing as unavailable.
`py-demo-capture` records the same profiling, memory, contact, worker, and
backend-status metrics into the manifest sidecar.

## Rigid contact scale budget

The **`rigid_contact_scale_budget`** scene keeps performance inspection on the
same live-GUI surface as the rest of the workflow. Three matched Worlds run one-,
four-, and nine-box contact workloads under one selected solver, executor, frame
budget, and friction value. The panel reports contact-point count, bodies,
contacts per body, wall time, per-contact cost, top profile stage, frame-scratch
peak usage, ECS counters, worker count, dense/single wall-time ratio, and whether
each lane is within the selected budget. It is a bounded frame-budget diagnostic,
not a benchmark suite or heavy IPC stress packet.
`py-demo-capture` records the same contact workload, frame-budget, wall-time,
scratch-memory, ECS, and budget-status metrics into the manifest sidecar.

## Rigid restitution ladder

The **`rigid_restitution_ladder`** scene makes the public restitution material
parameter visible without mixing it into a broader contact scenario. Three
matched lanes drop identical rigid spheres onto separate ground pads with
low, medium, and high restitution. The panel exposes solver, executor, launch
height, and restitution scale, then plots height, vertical velocity, contact
count, rebound height, and mechanical-energy trends as diagnostics rather than
exact conservation claims.

## Rigid material mixing

The **`rigid_material_mixing`** scene shows what happens when the rigid body and
its contact surface disagree on material values. Two swapped bounce lanes show
that sequential-impulse rigid contact uses the larger restitution value, while
two swapped flat sliding lanes show that friction uses the geometric mean of
the body and surface coefficients. The panel computes the expected pair values
from public `RigidBody.restitution` and `RigidBody.friction` properties, plots
rebound and tangential speed loss, and treats the swapped lanes as an ownership
check rather than a duplicate restitution ladder or incline threshold proof.

## Rigid contact inspector

The **`rigid_contact_inspector`** scene is the contact-observability row for
the rigid workflow. It keeps static targets and kinematic probes in stable
overlap so `World.collide()` can be inspected directly across representative
public shape families: sphere/box, box/ground, plane/sphere, capsule/sphere,
cylinder/sphere, mesh/sphere, and compound/sphere. The panel shows selected
pair count, total contact count, representative point, normal, depth, local
points, and shape indices, including the compound-child index path, while a
marker highlights the selected contact point in the scene. Docked captures now
carry the same selected-pair manifold fields through scene-owned capture metrics.

## Rigid collision query options

The **`rigid_collision_query_options`** scene makes the public
`World.collide(options)` body-kind filters and persistent ignored-pair table
visible. Four matched sphere-overlap lanes cover rigid/rigid, rigid/link,
same-multibody link/link, and cross-multibody link/link contacts. The panel
toggles each `CollisionQueryOptions` include flag, selects one ignored pair,
shows baseline, option-filtered, pair-ignored, and active contact counts, marks
filtered lanes explicitly, and keeps shape-index diagnostics available without
turning the row into a solver comparison.
`py-demo-capture` records the same include-toggle, ignored-pair, contact-count,
body-kind, cast, lane-status, and shape-index metrics into the manifest sidecar.

## Rigid collision casts

The **`rigid_collision_casts`** scene turns public collision cast queries into a
visual debugging row before users compare solver behavior. It shows
`CollisionGroup.raycast_result()` with nearest-hit versus all-hit behavior and
`CollisionGroup.sphere_cast_result()` plus
`CollisionGroup.capsule_cast_result()` time-of-impact queries. The panel
exposes ray lateral offset, all-hit mode, sphere radius, capsule radius, capsule
offset, and capsule cylinder height, then reports target names, hit fractions,
first hit point/normal, time of impact, cast margins, and histories without
claiming contact-solver or CCD time-step behavior. The capsule lane is for
elongated swept-volume debugging, such as robot links, tools, and character
controller proxies that a ray or sphere can under-explain.

## Rigid body solver comparison

The **`rigid_solver_compare`** scene is the first rigid-body visual-debug bench
that runs two matched DART 7 Worlds side by side: sequential impulse on the left
and rigid IPC on the right. It keeps the scene intentionally small, a sliding
box approaching a thin wall, so users can inspect solver-family differences,
clearance, speed, position divergence, and per-step profile timing without
waiting on heavier stack scenes. The panel names the comparison axis as the
rigid-body solver family and also exposes the shared executor choice
(`Sequential` or `Parallel (2 workers)`) so execution changes are framed as
performance/equivalence checks rather than as different physics. Its capture
metrics record the same comparison axis, solver pair, case order, executor
selection, controls, per-solver response metrics, and divergence ranges.

## Rigid executor equivalence

The **`rigid_executor_equivalence`** scene answers the executor question
directly: two matched DART 7 Worlds run the same rigid-body tray with one shared
physics solver, while the left world steps with the sequential executor and the
right world steps with a parallel executor. The panel keeps the physics solver,
friction, restitution, and launch speed explicit, then plots pose divergence,
velocity divergence, contact-count delta, and per-executor step time so users
can tell executor performance changes apart from physics changes. The panel
names the comparison axis as executor-only. Its capture metrics record the same
comparison axis, same-solver identity, selected solver index, controls,
per-executor contact/timing metrics, divergence ranges, and fallback executor
label when parallel execution is unavailable. The shared `Replay` panel uses
pose divergence as the timeline signal and marks frames where executor
divergence or contact-count mismatch appears, matching the solver-family and
contact-policy comparison rows.

## Rigid contact solver comparison

The **`rigid_contact_solver_compare`** scene separates the contact-solver policy
axis from the broader rigid-body solver family. Both lanes use the same
sequential-impulse rigid-body pipeline and the same tilted plank falling into
four-corner ground contact; the left lane keeps the default sequential-impulse
contact policy, while the right lane opts into boxed LCP contacts. The panel
names the comparison axis as contact solver method, keeps executor, launch
speed, friction, restitution, and initial tilt explicit, then plots contact
count, penetration depth, analytic corner clearance, speed, kinetic energy,
pose divergence, and per-step timing. Its capture metrics record the same
comparison axis, contact-policy pair, case order, executor selection, controls,
per-policy response metrics, and divergence ranges.

Capture rows 15-17 together when reviewing the core solver decision path:

```bash
pixi run py-demo-capture -- --rigid-workflow --workflow-start-row 15 \
    --workflow-end-row 17 --output-dir /tmp/dart_capture_rigid_solver_contact
```

That packet keeps the solver-family, executor-only, and contact-policy axes
adjacent in `review_index.html`, while each row card preserves its live open
command, workflow rerun command, direct capture command, guidance, Replay track,
scene-owned metric keys, held-fixed/control values, and the latest decision
signals such as solver pair, executor pair, contact-policy pair, divergence,
and step/contact counters.

Capture rows 19-21 together when reviewing the common contact-failure path:

```bash
pixi run py-demo-capture -- --rigid-workflow --workflow-start-row 19 \
    --workflow-end-row 21 --output-dir /tmp/dart_capture_rigid_contact_failures
```

That packet keeps the friction-threshold, spin/roll, and stack-stability rows
adjacent. Their row cards show the comparison axis, held-fixed solver/executor
or solver-family controls, user-editable controls, and latest signals such as
threshold-lane drift, contact slip/spin change, solver pair, top-x divergence,
and solver clearance.

## Rigid link contact

The **`contact`** scene is the articulated-link contact row in the numbered
rigid workflow. It keeps the stable scene id but now shows three multibody-link
lanes: a prismatic link dropping and rebounding on ground, a link sliding until
friction brakes it, and a prismatic pusher link transferring motion to a rigid
target. The panel exposes executor, ground friction, restitution, drop height,
slide speed, and push speed, then plots link contact counts, rebound, slide
travel, target travel, and step timing. It is a public multibody-link contact
verifier, not a contact-impulse, compliance, or solver-internal force row.

## Rigid friction threshold

The **`rigid_friction_threshold`** scene turns the inclined-ramp friction law
into a three-lane visual verifier. The upper lane uses friction below
`tan(angle)` and should slide, the lower lane uses friction above that threshold
and should stick, and the middle lane is user-controlled. The panel keeps the
threshold calculation visible, resets the run when the controlled friction or
ramp angle changes, names friction relative to the static threshold as the
comparison axis, and lists IPC/executor/time step as held fixed. It plots
down-slope drift and speed so users can debug stick/slip behavior without
relying on exact-at-threshold claims. Capture metrics record the threshold,
controlled-friction delta, per-lane friction, drift, speed, clearance, status,
held-fixed values, step timing, and compact history ranges.

## Rigid spin/roll coupling

The **`rigid_spin_roll_coupling`** scene shows the rotational side of contact
friction after the stick/slip threshold row: matched rolling, no-spin sliding,
backspin scrub, and a low-friction slip baseline move on the same surface. The
panel names spin/roll initial condition as the comparison axis, keeps the
sequential-impulse solver, executor, and time step held fixed, then exposes
contact friction, launch speed, and backspin ratio. It plots contact slip, roll
ratio, travel, kinetic energy, contact count, and step timing. It is a
sequential-impulse visual diagnostic for linear/angular coupling, not a
rolling-resistance or torsional-friction parameter surface. Capture metrics
record the solver/executor, controls, held-fixed values, contact count,
per-lane slip, roll ratio, spin delta, travel, energy, friction, status, step
timing, and compact history ranges.

## Rigid stack stability

The **`rigid_stack_stability`** scene is a compact resting-contact stress test:
sequential impulse and rigid IPC solve the same two-block top-heavy stack side
by side. The stack is intentionally small so it remains usable in the live GUI
while still showing the mass-ratio failure mode users care about: residual
jitter, lateral drift, clearance/overlap, and per-step solve cost. The panel
names the rigid-body solver family as the comparison axis, lists executor, top
mass ratio, friction, and time step as held fixed, lets users tune the top mass
ratio and friction, resets the stack, and compares max speed, top-block drift,
minimum analytic clearance, and solver divergence. Capture metrics record the
solver pair, executor, controls, held-fixed values, per-solver speed, drift,
clearance, height error, step timing, top-x divergence, and compact history
ranges. The shared Replay panel uses top-x divergence as its value track and
marks frames where overlap, low clearance, visible top-block drift, or
solver-family divergence make the stack worth scrubbing.

## Rigid contact manipulation

The **`rigid_contact_manipulation`** scene turns the rigid workflow from
isolated micro-cases into a small task-like push. Sequential impulse and rigid
IPC solve matched table scenes side by side: a heavier moving pusher block
drives a lighter target box toward a goal strip. The panel names the rigid
pusher contact response as the comparison axis, keeps the matched table/goal,
target mass, executor, and time step visible as held fixed, exposes pusher
launch speed, table friction, pusher mass, and executor choice, then plots
target travel, pusher-target gap, contact/proximity evidence, target speed,
lateral drift, solver divergence, and per-step timing.
Capture metrics record the comparison axis, solver pair, executor, held-fixed
values, controls, per-solver target travel, pusher gap, contact count, target
speed, lateral drift, goal error, step timing, travel divergence, and compact
history ranges. The workflow review card surfaces travel divergence, both
solver lanes' target travel, contact counts, and solver pair as latest signals.
The shared Replay panel uses travel divergence as its value track and marks
pusher contact, proximity, target-motion, and solver-divergence frames for fast
scrubbing.

## Rigid kinematic driver

The **`rigid_kinematic_driver`** scene verifies the public prescribed-motion
rigid-body path. Three lanes make the expected behavior and caveat visible: IPC
with grip friction carries a box on a moving kinematic support, IPC with zero
friction lets the support slip under the box, and the sequential-impulse lane
shows the current static-like caveat for kinematic bodies. The panel exposes
the prescribed tangential contact-response comparison axis, held-fixed
executor, tangential support, box mass, and time step, then shows driver speed,
grip friction, executor choice, driver travel, box travel, slip, speed ratio,
support gap, and step timing.
Capture metrics record the comparison axis, solver/case pair, held-fixed
values, IPC grip/slip lanes, the sequential-impulse caveat lane, solver enums,
friction modes, controls, per-lane driver travel, box travel, slip, speed
ratio, support gap, contact count, status, step timing, and compact history
ranges. The workflow review card surfaces IPC grip box travel, grip speed
ratio, low-friction slip, sequential-impulse caveat driver travel, IPC grip
proximity gap, and solver pair as latest signals. The shared Replay panel
uses the IPC grip lane's box travel as its value track and marks contact,
carried-progress, slip, and static-like caveat frames.

The **`rigid_kinematic_normal_push`** scene keeps the next prescribed-motion
question separate: what happens when a kinematic paddle moves normally into a
target? It is a caveat verifier. Sequential impulse pushes the target forward,
while IPC normal and heavier-target lanes expose the current penetration
failure mode instead of silently promoting it as a supported manipulation path.
The panel names prescribed normal contact response as the comparison axis,
keeps executor, normal kinematic paddle, zero friction, and time step visible
as held fixed, and then shows push speed, target mass, driver travel, target
travel, analytic gap, penetration depth, contact count, speed ratio, and step
timing. Capture metrics record lane order, solver pair, controls, held-fixed
values, IPC penetration depth, SI push travel, contact counts, target-travel
divergence, and compact history ranges. The shared Replay panel uses
target-travel divergence as its value track and marks contact, IPC penetration,
SI push-progress, and divergence frames.

Capture rows 22-24 together when reviewing the task-like contact branch:

```bash
pixi run py-demo-capture -- --rigid-workflow --workflow-start-row 22 \
    --workflow-end-row 24 --output-dir /tmp/dart_capture_rigid_kinematic_push
```

That packet keeps the rigid pusher, tangential kinematic driver, and normal
kinematic push caveat adjacent. The row-24 card shows the prescribed normal
contact axis, held-fixed kinematic-paddle context, push controls, IPC
penetration depth, SI target travel, and target-travel divergence.

## Rigid joint constraints

The **`rigid_fixed_joint`** scene is a focused fixed-constraint verifier. Its
panel labels the fixed-relative-transform recovery axis and held-fixed
sequential joint context, can deliberately perturb the child body away from the
captured relative transform, then plots relative offset error, orientation
error, and residual payload speed as the constraint projects the payload back
to the fixed pose. Capture metrics expose the same axis, held-fixed context,
top-level translation/orientation error, payload speed, angular speed, and
nested history summaries so the workflow review card can show the final
fixed-joint pose-recovery signals directly. The shared Replay panel uses
fixed-joint offset error as its value track and marks pose-error or
residual-motion frames during recovery.

The **`rigid_joint_breakage`** scene is the next fixed-joint lifecycle row. It
is explicitly AVBD-pinned: a very weak fixed joint starts intact, crosses a
break-force threshold, turns the connector red when `is_broken` becomes true,
and exposes `reset_breakage()` behavior without claiming sequential-impulse or
IPC parity. The panel includes a log-scale break-force threshold control plus
reset paths for the current threshold, locked reset, and weak re-arm. The panel
and capture metrics label the fixed break-force lifecycle comparison axis, the
held-fixed AVBD/static-base context, active threshold diagnostics, and top-level
breakage signals for release distance, broken state, captured-offset error,
payload speed, and status. The shared Replay panel uses payload release
distance as its value track and marks broken or released frames so saved-state
scrubbing jumps to the breakage event.

The **`rigid_distance_spring`** scene shows the public
`World.add_rigid_body_distance_spring()` path on free rigid bodies before users
move to one-DOF joints. It keeps the solver scope explicit: distance springs
are AVBD-backed rows in the sequential rigid-body World path, while IPC and
multibody Worlds reject this API. The panel compares an unsprung baseline,
soft and stiff center springs, and an off-center anchor lane with editable
rest length and per-lane stiffness controls, stretch, payload speed, angular
speed, and step timing. The panel, replay state, and capture metrics label the
distance-spring response-family comparison axis, the held-fixed
executor/rest-length/payload/time-step context, lane order, public spring
parameters, and top-level review signals for free, soft, stiff, and offset
absolute stretch plus off-center angular speed. The shared Replay panel uses
maximum spring stretch as its value track and marks high-stretch or off-center
spin frames.

The **`rigid_limited_joints`** scene verifies the public revolute and prismatic
rigid-body joint rows without making motor or limit claims. The hinge lane
tracks locked anchor radius and z error while its free z-axis spin continues;
the slider lane tracks locked lateral error while its z-axis travel remains
free. Both lanes expose perturb/reset controls so the bounded errors are
visible in the same live GUI workflow as the contact scenes. The panel and
capture metrics label the one-DOF joint constraint-family axis, held-fixed
sequential-joint/static-base/z-axis context, lane names, and top-level review
signals for locked hinge radius/z error, locked slider orthogonal error, hinge
yaw, slider travel, and free-axis speeds. The shared Replay panel uses
locked-axis error as its value track and marks constraint-error or free-axis
motion frames.

The **`rigid_joint_motor_limits`** scene covers the public World multibody joint
actuator path for rigid links. It keeps three compact lanes visible at once: a
prismatic velocity motor whose commanded speed is clamped by a velocity limit,
a gravity-driven revolute link settling at a position stop, and capped versus
reference force sliders showing the acceleration gap created by an effort
limit. The panel exposes the command and limit values directly, then plots
motor speed, position-limit error, and force-response histories.
`py-demo-capture` records the same row identity, actuator scope, control values,
joint names, motor clamp, position-stop, force/effort-cap, acceleration-gap,
step-timing, and compact history metrics into the manifest sidecar. The panel
and capture metrics label the World multibody actuator/limit comparison axis,
held-fixed x-axis prismatic rail and y-axis revolute-stop context, lane names,
and top-level review signals for motor speed, expected speed, speed error,
position-limit angle/error, force travel gap, and force acceleration gap. The
shared Replay panel uses force travel gap as its value track and marks
velocity-clamp, position-stop, and effort-cap frames.

The **`rigid_joint_passive_parameters`** scene covers the public passive
multibody joint parameters for rigid links without contacts or gravity. Matched
prismatic lanes show a spring-only oscillator, the same spring with damping,
Coulomb stiction versus slip under held efforts, and direct versus
armature-loaded drive. The panel exposes spring stiffness, rest position,
damping, Coulomb friction, separate hold/slip/armature drive forces,
armature, acceleration-versus-expected diagnostics, energy histories, and
step timing.
`py-demo-capture` records the same lane order, executor, spring/rest/damping,
Coulomb friction, hold/slip/armature drive forces, armature, acceleration,
energy, step-timing, and compact history metrics into the manifest sidecar.
The panel and capture metrics label the passive joint-parameter comparison
axis, held-fixed World-prismatic/gravity-off/contact-free/mass/time-step
context, lane names, and top-level review signals for spring energy, damped
energy, damped-energy ratio, slip speed, armature position gap, and armature
acceleration gap. The shared Replay panel uses armature position gap as its
value track and marks damping energy separation, Coulomb slip, and
armature-lag frames.

The **`rigid_screw_joint_pitch`** scene shows the public screw-joint pitch
semantics: pitch is axial translation per radian of rotation. Zero, fine,
coarse, and reverse-pitch lanes run under gravity so users can see rotation
sign, axial travel, travel-per-radian ratio, effective mass, and
expected-versus-actual acceleration from mass and axial inertia.
`py-demo-capture` records lane order, executor, pitch/gravity/mass/inertia
controls, per-lane pitch, angle, axial travel, travel-per-radian, acceleration,
effective mass, mass matrix, step-timing, and compact history metrics into the
manifest sidecar. The panel and capture metrics label the screw pitch coupling
comparison axis, held-fixed contact-free World screw-joint/mass/inertia/time
step context, lane names, and top-level review signals for zero-pitch axial
travel, fine/coarse/reverse pitch, coarse/fine travel gap, reverse angle, and
fine-lane acceleration residual. The shared Replay panel uses coarse/fine
travel gap as its value track and marks pitch-spread, zero-pitch contrast, and
reverse-sign frames.

The **`rigid_multibody_dynamics_terms`** scene makes the public generalized
dynamics accessors visible before the solver-family and loop-closure rows. A
single hinge, a coupled two-link arm, and a heavy-distal two-link arm show
`mass_matrix`, `inverse_mass_matrix`, `compute_inverse_dynamics()`, and
`compute_impulse_response()` under one contact-free World. The panel keeps the
diagnostics compact: matrix shape and conditioning, one off-diagonal coupling
term, inverse-dynamics residual, joint-space impulse residual, torque norm, and
response norm.
`py-demo-capture` records lane order, executor, target-acceleration, impulse,
heavy-mass, gravity, per-lane mass/coupling/conditioning/residual/torque/
response fields, heavy-versus-coupled ratios, step timing, and compact history
metrics into the manifest sidecar. The panel and capture metrics label the
joint-space dynamics term comparison axis, held-fixed contact-free World
dynamics/fixed-base/revolute-link/target-acceleration/impulse/gravity/time step
context, lane names, and top-level review signals for the single-hinge mass
diagonal, coupled off-diagonal mass term, heavy-load torque gap,
coupled-versus-heavy response gap, heavy response ratio, and residual checks.
The shared Replay panel uses coupled-versus-heavy response gap as its value
track and marks response-separation, off-diagonal-coupling, and heavy-load
torque frames.

The **`rigid_link_center_of_mass`** scene keeps the link visual geometry fixed
while moving `Link.center_of_mass` in the link frame. Centered, +X, -X, and
high-inertia lanes show when gravity torque is zero, why mirrored COM offsets
accelerate in opposite directions, and how the same offset produces a smaller
acceleration when the link inertia is larger. Yellow markers show the actual
center of mass, and the panel reports gravity torque, mass matrix, hinge
acceleration, expected acceleration, COM position, energy, and step timing.
`py-demo-capture` records lane order, executor, COM/gravity/mass/inertia
controls, per-lane link/joint/local-COM metadata, mirrored torque/angle/
acceleration sums, reflected mass and acceleration ratios, COM marker position,
energy, step timing, and compact histories into the manifest sidecar.
The panel and capture metrics label the link center-of-mass offset comparison
axis, held-fixed contact-free World revolute-link/fixed-visual-geometry/mass/
gravity/time step context, lane names, and top-level review signals for
centered/positive/negative gravity torque, mirrored angle sum, high-inertia
mass and acceleration ratios, and acceleration residual. The shared Replay
panel uses mirrored COM angle spread as its value track and marks
offset-driven angle divergence, centered-lane stillness, and high-inertia lag
frames.

The **`rigid_link_jacobian`** scene makes link-origin Jacobians concrete before
users move from generalized dynamics terms into solver-family routing. A
contact-free two-link arm shows `get_world_jacobian(link) @ qdot` as the
end-link origin twist, compares it against a finite-difference velocity, and
uses `get_world_jacobian(link).T @ wrench` to show joint torque power
consistency. It is intentionally scoped to the link origin: it does not claim
arbitrary point, COM, contact, IK, or operational-space controller behavior.
`py-demo-capture` records controls, joint names, link-origin position, twist,
finite-difference error, wrench force, transpose-mapped torques,
joint-versus-wrench power, world/body Jacobian gap, and compact histories into
the manifest sidecar.
The panel and capture metrics label the link-origin Jacobian mapping
comparison axis, held-fixed contact-free World two-revolute-link/time-step/
finite-difference context, the Jacobian term family, and top-level review
signals for link linear/angular speed, world/body Jacobian gap,
finite-difference error, transpose-mapped torques, and power residual.
The shared Replay panel uses link-origin speed as its value track and marks
high-twist, wrench-load, world/body Jacobian gap, and residual-alert frames.

The **`rigid_multibody_solver_family`** scene makes the World multibody
integration-family choice visible before users enter the loop-closure family
row. It compares semi-implicit residual-only closure diagnostics, variational
residual-only diagnostics, and variational dynamic closure solving on the same
three-link chain. The panel exposes executor and gravity controls, residual
norms, tip error, tip height, joint speed, step timing, and the residual solve
ratio so users can see why solved loop closures are routed to the variational
path.
`py-demo-capture` records executor/gravity controls, case order, integration
family and closure policy labels, residuals, tip errors/heights, joint speeds,
step timing, residual solve ratio, and compact history metrics into the
manifest sidecar.
The panel and capture metrics label the multibody integration solve-policy
comparison axis, held-fixed contact-free World point-closure/three-link-chain/
gravity/time-step context, solver-family lane names, and top-level review
signals for residual-only residual, solved residual, residual solve ratio,
semi-implicit and variational residuals, solved tip error, and maximum step
profile time.
The shared Replay panel uses residual solve ratio as its value track and marks
solve-advantage, residual-only drift, and solved-tight frames while residual
rows remain loose.

The **`rigid_loop_closure`** scene compares public POINT, DISTANCE, and RIGID
loop-closure families under the variational rigid multibody path. Each family
has residual-only and solved lanes: POINT locks the endpoint position,
DISTANCE holds only the tether length to an anchor, and RIGID welds the full
endpoint pose. The panel plots residual ratios and reports tip error,
distance error, orientation residual, tip height, and joint speed so users can
choose the narrowest closure family that matches the model.
`py-demo-capture` records executor/gravity controls, family and policy order,
per-case family/policy labels, residuals, tip/distance/orientation errors,
joint speeds, step timing, solved ratios, and compact history metrics into the
manifest sidecar.
The panel and capture metrics label the loop-closure family/policy comparison
axis, held-fixed contact-free variational rigid-multibody/four-link-chain/
gravity/time-step context, closure family and policy lane names, and top-level
review signals for POINT/DISTANCE/RIGID residual ratios, distance-family
distance/tip error, RIGID orientation error, and maximum step profile time.
The shared Replay panel uses max closure residual ratio as its value track and
marks solve-advantage, residual-versus-solved separation, distance-family tip
drift, and rigid-orientation frames.

Use the workflow mode to plan or run the full numbered rigid capture set. It
writes a top-level manifest that points at every per-scene manifest and a
`review_index.html` contact sheet for scanning the 36 screenshots, row
questions, try-first guidance, scope notes, live open commands, capture
commands, workflow-row rerun commands, and metric summaries from one page: the
in-viewer `Rigid Workflow` panel also shows the full numbered packet,
current-row rerun, and extended related/IPC-shelf/packet commands.
Numbered rows carry their workflow phase and focus axis in the manifest and
review card, while optional related, direct IPC shelf, and capture-first packet
rows carry the same row-guidance fields in the manifest and review index, so
extended packets remain readable without opening the live GUI. The same outputs
also include a guidance-completeness audit and the exact top-level workflow
command for the selected packet.

```bash
pixi run py-demo-capture -- --rigid-workflow --dry-run
pixi run py-demo-capture -- --rigid-workflow \
    --output-dir /tmp/dart_capture_rigid_workflow
```

For long review packets, add `--continue-on-failure` when later rows should
still be captured after one row fails. The manifest records
`continue_on_failure=true` plus a `failed_rows` list, and
`review_index.html` shows a Failed Rows summary with workflow row-range rerun
commands that preserve the packet flags and absolute row numbering. The final
workflow status and process exit code still report failure if any selected row
failed.

```bash
pixi run py-demo-capture -- --rigid-workflow --include-related \
    --include-ipc-shelf --include-packets --continue-on-failure \
    --output-dir /tmp/dart_capture_rigid_workflow_resilient
```

Add `--include-related` when the review packet should also capture the
non-numbered related evidence routes after the 36-row workflow:

```bash
pixi run py-demo-capture -- --rigid-workflow --include-related --dry-run
pixi run py-demo-capture -- --rigid-workflow --include-related \
    --output-dir /tmp/dart_capture_rigid_workflow_with_related
```

Add `--include-ipc-shelf` when the packet should also capture the direct
metric-backed Rigid IPC shelf scenes after the numbered rows and any related
evidence routes:

```bash
pixi run py-demo-capture -- --rigid-workflow --include-ipc-shelf --dry-run
pixi run py-demo-capture -- --rigid-workflow --include-related \
    --include-ipc-shelf --output-dir /tmp/dart_capture_rigid_workflow_with_ipc_shelf
```

Add `--include-packets` when the packet should also include capture-first
rigid IPC evidence that is intentionally outside the live 36-row workflow.
Capture-first packets are appended after related evidence and direct Rigid IPC
shelf rows when those groups are requested:

```bash
pixi run py-demo-capture -- --rigid-workflow --include-packets --dry-run
pixi run py-demo-capture -- --rigid-workflow --include-related \
    --include-packets --output-dir /tmp/dart_capture_rigid_workflow_with_packets
```

Capture the direct Rigid IPC shelf routes with the docked UI visible:

These commands are also included after the numbered rows, and after related
evidence when present, by
`py-demo-capture -- --rigid-workflow --include-ipc-shelf`.

```bash
pixi run py-demo-capture -- --scene rigid_ipc --frames 72 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_ipc_slide --frames 72 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_ipc_incline --frames 72 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_ipc_pile --frames 72 \
    --width 960 --height 540 --show-ui
```

For targeted reruns after a failed or manually inspected row, keep the same
workflow packet but bound the row range. Row numbers stay absolute, so row 37
still writes under `scenes/37_<scene>` when related evidence is included. With
`--include-related --include-packets`, rows 48-49 are the two
capture-first stack packets. If `--include-ipc-shelf` is also requested, those
packet rows become 52-53.

```bash
pixi run py-demo-capture -- --rigid-workflow \
    --workflow-start-row 15 --workflow-end-row 17 --dry-run
pixi run py-demo-capture -- --rigid-workflow --include-related \
    --include-packets --workflow-start-row 48 --workflow-end-row 49 \
    --output-dir /tmp/dart_capture_rigid_workflow_packet_rerun
```

For row-range packets, manifest fields such as `include_related`,
`include_ipc_shelf`, and `include_packets` record the requested packet shape;
`selected_include_related`, `selected_include_ipc_shelf`, and
`selected_include_packets` record which optional groups are present in the
selected row range. The generated `review_index.html` header mirrors this with
row-span, requested-groups, and selected-groups badges.

Capture the focused rigid verifier scenes with the docked UI visible:

```bash
pixi run py-demo-capture -- --scene rigid_body --frames 180 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_body_modes --frames 72 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_free_flight --frames 96 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_frame_hierarchy --frames 72 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_external_loads --frames 72 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_link_point_loads --frames 72 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_timestep_sensitivity --frames 96 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_step_diagnostics --frames 72 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_contact_scale_budget --frames 72 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_restitution_ladder --frames 96 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_material_mixing --frames 72 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_contact_inspector --frames 24 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_collision_query_options --frames 24 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_collision_casts --frames 48 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_solver_compare --frames 24 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_executor_equivalence --frames 24 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_contact_solver_compare --frames 72 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene contact --frames 144 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_friction_threshold --frames 24 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_spin_roll_coupling --frames 96 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_stack_stability --frames 24 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_contact_manipulation --frames 72 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_kinematic_driver --frames 72 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_kinematic_normal_push --frames 72 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_fixed_joint --frames 24 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_joint_breakage --frames 48 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_distance_spring --frames 72 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_limited_joints --frames 24 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_joint_motor_limits --frames 96 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_joint_passive_parameters --frames 120 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_screw_joint_pitch --frames 96 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_multibody_dynamics_terms --frames 96 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_link_center_of_mass --frames 72 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_link_jacobian --frames 96 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_multibody_solver_family --frames 72 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_loop_closure --frames 72 \
    --width 960 --height 540 --show-ui
```

Capture the related Rigid IPC no-tunneling or degenerate edge-contact views
when those focused capabilities are the target:

```bash
pixi run py-demo-capture -- --scene rigid_ipc_tunnel --frames 24 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_ipc_edge_drop --frames 72 \
    --width 960 --height 540 --show-ui
```

Capture the heavier Rigid IPC stack packets when the question is what happens
beyond the live workflow budget. These scenes stay in the **Rigid IPC** shelf,
outside the numbered workflow, and their panels show min clearance, contact
count, top drift, height error, max speed, wall time, top mass, and the
`bm_rigid_ipc_solver` benchmark pointer:

```bash
pixi run py-demo-capture -- --scene rigid_ipc_stack_packet --frames 24 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_ipc_heavy_stack_packet --frames 12 \
    --width 960 --height 540 --show-ui
```

These packets are included after the numbered rows, optional related evidence
routes, and optional direct Rigid IPC shelf routes by
`py-demo-capture -- --rigid-workflow --include-packets`.

For scenes that expose `SceneSetup.info["capture_metrics"]`,
`py-demo-capture` also writes `scene_metrics.jsonl` and summarizes the full
scene-owned physics/runtime stream in `manifest.json`: first and latest events,
per-key presence counts, numeric ranges for top-level metrics, and the
`resolved_solver_identity` derived from the latest solver/contact/executor
metrics. Workflow captures treat those metrics as required evidence for
completed rows. The step diagnostics and contact-scale budget rows use that path
for profiling, memory, contact, and frame-budget evidence; the IPC no-tunneling
route uses it for clearance and through-wall margin evidence; and the stack
packet uses it for clearance, drift, wall time, and benchmark metadata. The
differentiable contact-gradient routes use it for target/rest height, analytic
versus complementarity-aware thrust/final-height/loss values, height and
target-error gaps, pre-contact counts, identical forward-state evidence,
analytic-freefall error, surrogate block magnitude, vertical sensitivity,
fallback status, and compact history summaries. The AVBD related routes use it
for fixed-joint contact offset/clearance/contact counts, spherical breakage
anchor/orientation drift, and free-rigid revolute/prismatic motor tracking. The
early numbered rigid workflow rows use it for body-mode flags, free-flight
momentum/energy residuals, frame-transform residuals,
force/torque accumulator response, point-load lever-arm response,
time-step error ratios, restitution rebound, and pair-material mixing fields,
so the first-run fundamentals now preserve physics diagnostics in capture
artifacts instead of only screenshots.

When forward rigid contact looks correct but a differentiable optimization is
stuck, jump to **`diff_drone_liftoff`** in the **Differentiable** shelf. It uses
the same rigid `World` contact-gradient modes to show `ANALYTIC` stalling at a
clamping contact while `COMPLEMENTARITY_AWARE` escapes, with thrust, loss,
gradient, and height histories. Its capture metrics report the optimized
mode outcome when diff bindings are enabled. When the body is approaching but
not touching, jump to **`diff_pre_contact_surrogate`**. It shows identical
forward motion for `ANALYTIC` and `PRE_CONTACT_SURROGATE` while the surrogate
lane adds a backward-only pre-contact Jacobian block. Both scenes record finite
fallback payloads when the default build has `DART_BUILD_DIFF=OFF`. In the
viewer, the `Rigid Contact Solver Compare` row exposes them as `Related shelf`
routes:

```bash
pixi run py-demo-capture -- --scene diff_drone_liftoff --frames 96 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene diff_pre_contact_surrogate --frames 24 \
    --width 960 --height 540 --show-ui
```

Capture every non-numbered related-evidence route with the docked UI visible:

These commands are also included after the numbered rows by
`py-demo-capture -- --rigid-workflow --include-related`.

```bash
pixi run py-demo-capture -- --scene floating_base --frames 72 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene articulated --frames 72 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_ipc_tunnel --frames 24 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_ipc_edge_drop --frames 72 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene diff_drone_liftoff --frames 96 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene diff_pre_contact_surrogate --frames 24 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene avbd_rigid_fixed_joint_contact --frames 72 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene avbd_rigid_breakable_joint --frames 72 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene avbd_rigid_spherical_breakable_joint \
    --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene avbd_rigid_revolute_motor --frames 72 \
    --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene avbd_rigid_prismatic_motor --frames 72 \
    --width 960 --height 540 --show-ui
```

The lower-level viewer still accepts `--screenshot` and `--out` directly when
you need raw PPM output. Inspect the generated PNG or MP4 before calling a
visual change done.

## LCP Physics

The **`lcp_physics`** scene is the Python demo baseline for DART 7 contact LCP
work. It runs two matched `World` instances side by side: the default
sequential-impulse contact path and the boxed-LCP contact path. Both worlds run
the same representative packets:

| Packet                | Shows                                      | Metric surfaced in the panel               |
| --------------------- | ------------------------------------------ | ------------------------------------------ |
| Sliding friction      | Tangential Coulomb response on a flat slab | Sliding speed and contacts                 |
| Static-friction ramp  | Hold/slip behavior near the friction limit | Ramp-parallel slide                        |
| Billiard collision    | Symmetric contact impulse transfer         | Momentum and kinetic-energy error          |
| High-mass-ratio stack | Contact stability under large mass ratios  | Stack lateral drift and per-step wall time |
| Thin card pile        | Thin high-aspect-ratio stacked contacts    | Card spread and height loss                |

This scene compares the DART 7 public contact methods. The panel also surfaces
the 24-solver standalone LCP manifest, solver support matrix, and benchmark
packet map used for apples-to-apples solver work. It runs the flat `dartpy`
standalone LCP binding on a shared standard smoke problem for every solver, so
the panel can show status, iteration count, and solution error without going
through a contact `World`. The representative problem table lists every solver
on each packet and separates native support from delegated fallback solves, with
the fastest native solve highlighted for quick triage. The detailed table also
surfaces residual and complementarity so accuracy comparisons do not rely only
on solution error. A per-solver profile rolls those rows back up by solver so
native surfaces, delegated coverage, pass counts, worst error, worst residual,
and total demo-measured solve time are visible in one apples-to-apples matrix.
Its standard-LCP packet set includes well-conditioned, ill-conditioned,
near-singular, and moderate scale cases so conditioning and size changes are
visible without changing the solver roster. Its friction-index rows include both
a simple normal-scaled contact and a coupled two-contact active-bound packet.
The benchmark packet map includes per-packet benchmark filters, including the
active friction-index microbenchmark and the native `world_card_pile` packet for
high-aspect-ratio card-pile contact scaling.
Authoritative performance runs remain owned by
`tests/common/lcpsolver`, `tests/unit/math/lcp`, and `tests/benchmark/lcpsolver`.
The panel also points to the checked performance-profile CSVs under
`docs/background/lcp/figures` and summarizes the current leader/laggard
families per Standard, Boxed, and FrictionIndex profile row family. Refresh
those artifacts with:

```bash
pixi run python scripts/lcp_performance_profile.py --run \
    --cache build/lcp_profile_full.json \
    --output docs/background/lcp/figures \
    --benchmark-timeout 900
```

For a quick profile-pipeline smoke that allows partial native solver coverage
without rewriting checked artifacts, write to a scratch output directory:

```bash
pixi run python scripts/lcp_performance_profile.py --run \
    --allow-partial \
    --benchmark-filter BM_LcpCompare/Standard/Dantzig/12 \
    --benchmark-min-time 0.01 \
    --cache build/lcp_profile_smoke.json \
    --output build/lcp_profile_smoke \
    --benchmark-timeout 120
```

Run the benchmark smoke with:

```bash
pixi run bm lcp_compare -- --benchmark_filter=BM_LCP_COMPARE_SMOKE
```

The `lcp_physics` scene metadata exposes the derived
`representative_benchmark_command` for the full representative packet filter;
keep the long generated filter list in the scene metadata instead of copying it
into this README.

## DART 7 Body Deactivation

`deactivation_sleeping` showcases the opt-in automatic body deactivation path:
quiet rigid bodies sleep, an active striker wakes them through contact, and an
untouched reference body remains asleep. The scene exposes `is_sleeping` and
`deactivation_group_index` in a diagnostics table and works with the shared
headless capture path:

```bash
pixi run py-demo-capture -- --scene deactivation_sleeping --show-ui \
    --frames 150 --width 1280 --height 720
```

## AVBD Rigid Constraints (sx)

The dedicated **`AVBD Rigid Constraints (sx)`** category groups the first
user-visible Augmented VBD rigid-constraint scenes from PLAN-104:

The fixed-contact, spherical breakable, revolute-motor, and prismatic-motor
free-rigid rows are also related evidence from the numbered rigid workflow.
They expose scene-owned capture metrics so docked captures preserve the
constraint/contact/motor signal instead of relying on screenshots alone.

| Scene id                                                | Shows                                                                                       | AVBD capability exercised                                                        |
| ------------------------------------------------------- | ------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------- |
| `avbd_empty_baseline`                                   | An empty variational-integrator World stepping with render-only axes                        | Source-demo corpus baseline with 2D/3D empty-row revision/default metadata       |
| `avbd_demo2d_ground`                                    | The `avbd-demo2d` Ground source scene as its static 2D ground slab                          | Source-demo static ground row with matched source geometry and metadata          |
| `avbd_demo2d_motor`                                     | The `avbd-demo2d` Motor source scene as a pinned rigid bar with a velocity motor            | First non-empty source-demo row using public AVBD revolute motor rows            |
| `avbd_demo2d_dynamic_friction`                          | The `avbd-demo2d` Dynamic Friction source scene as sliding boxes over a ground slab         | Source-demo rigid friction row with matched source geometry and metadata         |
| `avbd_demo2d_static_friction`                           | The `avbd-demo2d` Static Friction source scene as rotated 2D boxes on an inclined slab      | Source-demo inclined-slab friction row with matched source geometry/metadata     |
| `avbd_demo2d_pyramid`                                   | The `avbd-demo2d` Pyramid source scene as a triangular pile of 2D rigid boxes               | Source-demo rigid stacking row with matched source geometry and metadata         |
| `avbd_demo2d_cards`                                     | The `avbd-demo2d` Cards source scene as a thin-card rigid contact tower                     | Source-demo card-tower contact row with matched source geometry and metadata     |
| `avbd_demo2d_stack`                                     | The `avbd-demo2d` Stack source scene as falling 2D boxes over a ground slab                 | Source-demo rigid stacking row with matched source geometry and metadata         |
| `avbd_demo2d_stack_ratio`                               | The `avbd-demo2d` Stack Ratio source scene as geometric-size 2D rigid boxes                 | Source-demo size-ratio stacking row with matched source geometry and metadata    |
| `avbd_demo2d_rod`                                       | The `avbd-demo2d` Rod source scene as a horizontal all-axis fixed-joint chain               | Source-demo rod row with matched hard constraints, geometry, and metadata        |
| `avbd_demo2d_soft_body`                                 | The `avbd-demo2d` Soft Body source scene as two finite-stiffness box lattices               | Source-demo soft-body row with finite all-axis joint stiffness and metadata      |
| `avbd_demo2d_joint_grid`                                | The `avbd-demo2d` Joint Grid source scene as a 25x25 fixed-joint box lattice                | Source-demo large joint-grid row with matched hard constraints and metadata      |
| `avbd_demo2d_rope`                                      | The `avbd-demo2d` Rope source scene as a horizontal point-jointed rigid chain               | Source-demo rope row with matched anchors, geometry, and metadata                |
| `avbd_demo2d_heavy_rope`                                | The `avbd-demo2d` Heavy Rope source scene with a large endpoint block                       | Source-demo high-ratio rope row with matched anchors, geometry, and metadata     |
| `avbd_demo2d_hanging_rope`                              | The `avbd-demo2d` Hanging Rope source scene as vertical point-jointed links                 | Source-demo long rope row with matched anchors, geometry, and metadata           |
| `avbd_demo2d_spring`                                    | The `avbd-demo2d` Spring source scene as one radial rigid distance spring                   | Source-demo finite-stiffness spring row with matched anchors and metadata        |
| `avbd_demo2d_spring_ratio`                              | The `avbd-demo2d` Spring Ratio source scene as alternating-stiffness radial springs         | Source-demo stiffness-ratio row with matched anchors, geometry, and metadata     |
| `avbd_demo2d_net`                                       | The `avbd-demo2d` Net source scene as endpoint-pinned links under falling rigid blocks      | Source-demo net row with matched anchors, geometry, and metadata                 |
| `avbd_demo2d_fracture`                                  | The `avbd-demo2d` Fracture source scene as falling 2D boxes over weak fixed joints          | Source-demo breakable fixed-joint row with public AVBD break thresholds          |
| `avbd_demo3d_ground`                                    | The `avbd-demo3d` Ground source scene as a falling rigid box over a static ground box       | Source-demo rigid contact row with matched source geometry and metadata          |
| `avbd_demo3d_dynamic_friction`                          | The `avbd-demo3d` Dynamic Friction source scene as sliding rigid boxes over ground          | Source-demo rigid friction row with matched source geometry and metadata         |
| `avbd_demo3d_static_friction`                           | The `avbd-demo3d` Static Friction source scene as rigid boxes on an inclined ramp           | Source-demo inclined-ramp friction row with matched source geometry/metadata     |
| `avbd_demo3d_pyramid`                                   | The `avbd-demo3d` Pyramid source scene as a triangular pile of rigid boxes                  | Source-demo rigid stacking row with matched source geometry and metadata         |
| `avbd_demo3d_rope`                                      | The `avbd-demo3d` Rope source scene as anchored rigid links with linear-only point joints   | Source-demo rope row with matched anchors, geometry, and metadata                |
| `avbd_demo3d_heavy_rope`                                | The `avbd-demo3d` Heavy Rope source scene with a large endpoint block                       | Source-demo high-ratio rope row with matched anchors, geometry, and metadata     |
| `avbd_demo3d_stack`                                     | The `avbd-demo3d` Stack source scene as ten rigid boxes over a static ground box            | Source-demo vertical stacking row with matched source geometry and metadata      |
| `avbd_demo3d_stack_ratio`                               | The `avbd-demo3d` Stack Ratio source scene as a four-box geometric size tower               | Source-demo size-ratio stacking row with matched source geometry and metadata    |
| `avbd_demo3d_soft_body`                                 | The `avbd-demo3d` Soft Body source scene as three finite-stiffness box lattices             | Source-demo 3D soft-body row with finite all-axis joint stiffness and metadata   |
| `avbd_demo3d_bridge`                                    | The `avbd-demo3d` Bridge source scene with paired point-jointed planks and load boxes       | Source-demo bridge row with matched anchors, geometry, and metadata              |
| `avbd_demo3d_breakable`                                 | The `avbd-demo3d` Breakable source scene as rigid boxes, contact shapes, and weak joints    | Source-demo breakable fixed-joint row with public AVBD break thresholds          |
| `avbd_rigid_fixed_joint_contact`                        | A fixed rigid payload sliding against static contact                                        | Public fixed-joint rows plus ordinary contact                                    |
| `avbd_rigid_revolute_motor`                             | A bounded revolute motor driving a free hinge                                               | Public velocity actuator mapped to AVBD motor rows                               |
| `avbd_rigid_prismatic_motor`                            | A bounded prismatic motor driving a free rigid slider                                       | Public linear velocity actuator mapped to AVBD motor rows                        |
| `avbd_articulated_revolute_motor`                       | A bounded revolute motor reversing an articulated floating link                             | Public articulated velocity motor updates through the bridge                     |
| `avbd_articulated_prismatic_motor`                      | A bounded prismatic motor reversing an articulated floating carriage                        | Public articulated linear motor updates through the bridge                       |
| `avbd_articulated_motor_breakable_joint`                | A weak articulated revolute motor releasing and re-engaging an articulated floating link    | Public articulated motor break/reset lifecycle through the bridge                |
| `avbd_articulated_prismatic_pair_motor_breakable_joint` | A weak same-multibody prismatic motor releasing and re-engaging an articulated carriage     | Public articulated linear motor break/reset lifecycle through the bridge         |
| `avbd_articulated_prismatic_motor_breakable_joint`      | A weak world-anchored prismatic motor releasing and re-engaging an articulated carriage     | Public articulated linear motor break/reset lifecycle through the bridge         |
| `avbd_articulated_world_revolute_motor_breakable_joint` | A weak world-anchored revolute motor releasing and re-engaging an articulated floating link | Public world-anchored articulated motor break/reset lifecycle through the bridge |
| `avbd_articulated_high_ratio_chain`                     | A five-link variational chain swinging with a 200:1 heavy tip                               | Narrow articulated high mass-ratio smoke for the AVBD paper gap                  |
| `avbd_paper_scale_high_ratio_chain`                     | A 50-link variational chain swinging with a 50,000:1 heavy tip                              | Paper-scale articulated high mass-ratio visual smoke for the AVBD paper gap      |
| `avbd_rigid_breakable_joint`                            | A weak fixed joint releasing and re-engaging a rigid payload                                | Public break-force threshold, reset, and broken-state path                       |
| `avbd_rigid_spherical_breakable_joint`                  | A weak spherical point joint releasing and re-engaging only a rigid payload anchor          | Public free-rigid spherical break/reset while orientation stays free             |
| `avbd_articulated_breakable_joint`                      | A world-anchored floating link released and reset through a public articulated fixed joint  | Public articulated world-link break/reset path through the variational bridge    |
| `avbd_articulated_fixed_pair_breakable_joint`           | A same-multibody fixed joint releasing and restoring a captured relative pose               | Public articulated link-link fixed break/reset through the variational bridge    |
| `avbd_articulated_spherical_breakable_joint`            | A world-anchored floating socket that releases and resets only its anchor                   | Public articulated spherical break/reset with orientation left free              |
| `avbd_articulated_spherical_pair_breakable_joint`       | A same-multibody floating socket that releases and resets only its anchor                   | Public articulated link-link spherical break/reset with orientation left free    |

This is an early AVBD rigid-row showcase, not a paper-complete reproduction.
The remaining AVBD corpus still needs the full 2D/3D reference demos, paper
figures, video/headline scenes, CPU/GPU benchmark packets, and performance
comparisons recorded in PLAN-104.

## Planned World Ports

The **`Planned World Ports`** category keeps important DART 6 demo concepts
visible without keeping the removed DART 6 implementations in the catalog. These
entries are lightweight launchable placeholders with status panels. Each panel
now tells users which current py-demo row to try first, which World/API or asset
gap blocks the full port, and what condition will replace the placeholder with a
usable scene. They track World-native follow-ups for inverse kinematics,
SIMBICON walking, operational-space control, robot puppets, and mobile
manipulation. The old collision sandbox placeholder is retired: use
`rigid_contact_inspector`, `rigid_collision_query_options`, and
`rigid_collision_casts` for concrete collision debugging workflows.

## PLAN-083 CPU Corpus

The **`PLAN-083 Mixed Corpus`**, **`PLAN-083 Constraints Corpus`**,
**`PLAN-083 Robot Corpus`**, and **`PLAN-083 ABD Corpus`** categories expose
launchable placeholders for the unified Newton-barrier paper/deck scene rows.
Each placeholder records its manifest row IDs, smoke command, long-horizon
visual capture command, benchmark packet target, and current limitation. They
are not paper-scene reproductions yet; the checked corpus sidecar lives at
`docs/plans/083-unified-newton-barrier-multibody/cpu-scene-corpus.json` and
keeps the py-demo, visual evidence, and CPU benchmark obligations explicit until
runtime mixed-domain stepping and ABD scene support land.

## IPC Deformable scenes

The dedicated **`IPC Deformable`** category groups the deformable-body
scenes driven by the `dartpy` IPC solver
(point-mass/spring with IPC-style clamped-log barriers, sparse projected Newton,
lagged smoothed-Coulomb friction, and conservative CCD). They live in their own
category so the IPC showcases stay together:

| Scene id                            | Shows                                                 | IPC capability exercised                   |
| ----------------------------------- | ----------------------------------------------------- | ------------------------------------------ |
| `ipc_deformable_net`                | A pinned spring net sagging/swaying under gravity     | Projected-Newton elastodynamics            |
| `ipc_deformable_drape`              | A mat draping over a step onto a ground barrier       | Ground + self-contact clamped-log barrier  |
| `ipc_deformable_trampoline`         | A corner-pinned membrane sagging and rebounding       | Taut-membrane projected-Newton dynamics    |
| `ipc_deformable_friction_slide`     | A launched mat skidding to rest on a frictional floor | Lagged smoothed-Coulomb ground friction    |
| `ipc_deformable_fem_bar`            | A tetrahedral cantilever sagging under gravity        | Stable neo-Hookean **FEM** elasticity      |
| `ipc_deformable_fem_twist`          | A tetrahedral bar twisted at both ends, then released | **FEM** volumetric shear (toward Fig 4/14) |
| `ipc_deformable_fem_drop`           | A FEM cube dropped onto a ground barrier, settling    | **FEM** volumetric body + IPC contact      |
| `ipc_deformable_fem_sphere`         | A FEM slab draping over a sphere obstacle             | **FEM** + sphere obstacle barrier (Newton) |
| `ipc_deformable_fem_box`            | A FEM slab draping over a box "table" obstacle        | **FEM** + box obstacle barrier (Newton)    |
| `ipc_deformable_fem_msh`            | A FEM cantilever loaded from a GMSH `.msh` tet mesh   | **FEM** body from an imported tet mesh     |
| `ipc_deformable_scripted_dirichlet` | A banner billowing under a scripted Dirichlet BC      | Scripted Dirichlet boundary conditions     |

These are DART-native showcases, **not** faithful IPC paper-figure
reproductions. The _contact, barrier, friction, and projected-Newton machinery_
is genuine IPC; most scenes use a mass-spring elasticity model, while the
`ipc_deformable_fem_*` scenes (cantilever, twist, ground-barrier drop) exercise
the new opt-in stable neo-Hookean **FEM** volumetric elasticity (PLAN-081 M1,
the keystone toward the paper's volumetric scenes), including a volumetric body
in IPC contact. Codimensional collision and the upstream mesh corpus are still
needed for true figure reproduction. The catalog of upstream paper scenes lives at
`docs/plans/081-deformable-implicit-barrier-solver/ipc_scene_corpus_manifest.json`;
faithful corpus reproduction stays `planned` (it needs vendored assets,
codimensional collision, and broader FEM coverage), so these scenes evoke the
paper's _themes_ (volumetric elasticity, draping, self-contact, friction,
scripted boundaries) rather than reproducing
its figures.

The scenes share `_ipc_deformable_bridge.py`, which builds the grid topology and
mirrors each deformable onto the render world as per-node spheres plus a spring
wireframe (the dynamic surface-mesh render path is not yet exposed through
`dartpy.gui.run_demos`). `IpcDeformableBridge.build_diagnostics_panel(...)`
adds shared solver diagnostics such as node counts, fixed nodes, z-range, and a
minimum-height plot to custom scene panels. Add another deformable scene by
building its `DeformableBodyOptions` with `build_grid_options(...)` and
rendering it through `IpcDeformableBridge`.

## Simulation Replay

`replay_scrubber` demonstrates the DART 7 `World` replay recorder:
the scene records a rigid-body rollout once, restores the first frame, and
exposes a bottom-docked replay timeline with a scrubber, frame marks, a cursor
track, transport controls, loop/rate controls, and cursor details. The scene
uses the reusable `PanelBuilder.timeline(...)` widget from `dartpy.gui` for the
timeline lanes. Moving the scrubber calls `World.restore_replay_frame(...)` at
timestep resolution and does not re-run physics.

The same saved-state replay path is injected by the runner into every
`SceneSetup` that exposes a DART 7 `World` in `info["sx_world"]` or
`info["physics_world"]`. The shared panel stores only bounded mutable World
snapshots plus optional small scene-provided mutable controller snapshots;
static topology, geometry, materials, and scene construction data remain owned
by the scene and render bridge.

The shared demos toolbar uses the same timeline widget for captured-frame
playback after `Capture` records viewer frames. That path stores only the
captured image files and derives bounded marker/cursor tracks for the UI each
frame, so large capture directories do not require an equally large in-memory
timeline cache.

## Add a scene

1. Create `scenes/<name>.py` defining a module-level `SCENE` of type
   `PythonDemoScene` whose `build()` returns a `SceneSetup` (a `world` plus an
   optional custom `pre_step`, `step(n)`, `force_drag`, scene panels, and
   `info` dict). A scene that wants capture manifests to include scene-owned
   physics evidence may put a zero-argument metrics callable in
   `info["capture_metrics"]`.
2. Import it in `registry.py` and append it to the ordered list in
   `make_demo_scenes()`, placed within its category group.

That's the only change needed; no CMake edits, no test edits beyond the cycle
smoke (which iterates the registry).

Scene-specific controls use renderer-neutral `ScenePanel` callbacks:

```python
def build_panel(builder, context):
    builder.text(f"time: {world.time:.3f} s")
    changed, friction = builder.slider("Friction", body.friction, 0.0, 1.0)
    if changed:
        body.friction = friction

return SceneSetup(
    world=bridge.render_world,
    pre_step=bridge.pre_step,
    panels=[ScenePanel("Rigid IPC Slide", build_panel)],
)
```

The callback receives DART's `PanelBuilder` abstraction, not ImGui. Use the
builder for text, buttons, checkboxes, sliders, selects, plots, and tables. Use
`context` for viewer state and lifecycle requests such as pause, single-step,
or switching to another demo scene.
