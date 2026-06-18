# Milestones — py-demos Framework

The driving idea (from the task owner): py-demos is the **visual inspection**
arm of DART 7's human-facing harness (alongside tests for headless correctness,
benchmarks for performance, and the website user-guide for understanding). In an
AI-native project where code changes fast, humans focus on these harnesses. So
py-demos must be **stable, no-crash, scalable, and modern**, and must grow
deliberately: a strong minimal core first, then feature-rich one capability at a
time, each step backed by tests and verification.

---

## M0 — Establish the framework

**Definition of done:** the framework is trustworthy. Running py-demos and
browsing the full catalog never crashes; adding a scene is a clean, tested,
repeatable operation; the core is modern and lean.

### M0 exit criteria

1. **No-crash, full catalog.** Every registered scene builds + steps + renders
   for a few frames without raising, in both CPU and `-e cuda` configurations.
   A full-catalog headless smoke enforces this (today only the first 3 scenes
   are GUI-smoke-tested — see `python/tests/integration/test_demos_cycle.py`).
2. **Loud, honest status.** Scenes that cannot run today are explicitly marked
   (planned / placeholder / skipped-with-reason) rather than crashing or
   silently rendering nothing. The live catalog the user browses is all-green.
3. **Scalable contract.** The scene contract (`SceneSetup`, `ScenePanel`,
   `PythonDemoScene`) is the single, documented way to add a capability; a lint
   or test catches an unregistered/duplicate/ill-formed scene.
4. **Modern + lean core.** The runner stays maintainable; shared machinery
   (panels, replay, debug overlays, capture) is reused, not copy-pasted per
   scene. No dead/placeholder scenes inflating the live catalog.
5. **Verification baked in.** The full-catalog smoke runs in the test suite, so
   the catalog stays green as it grows.

### M0 first actions

- Run the full-catalog no-crash smoke → record the baseline failure set.
- Triage each failure: fix-now / quarantine (mark planned + skip) / defer.
- Promote the smoke into the test suite.

### M0 smoke coverage tiers

- **Tier 1 (now, `scripts/py_demos_smoke.py`):** per-scene build, headless
  step, and render/debug provider calls, subprocess-isolated. This is the
  baseline no-crash signal.
- **Tier 2 (after baseline is green):** also exercise each `ScenePanel.build`
  with a fake builder. Reuse/extract `_FakePanelBuilder` / `_FakePanelContext`
  from `python/tests/unit/test_py_demo_panels.py` (must cover the _full_ builder
  API or it yields false failures). Panels are a real live-viewer crash source.
- **Tier 3 (later):** a full-catalog headless _render_ pass through the real
  viewer (`--cycle-scenes --headless`) for the Filament render path.

### M0 triage policy (CONFIRMED): quarantine to green

Decision by the task owner: the **live catalog the user browses must be 100%
green**. Scenes that cannot run yet (intentional `PLANNED_*` / PLAN-083
placeholders, or anything blocked on a missing engine feature) are **explicitly
marked skipped/planned and excluded from the live no-crash guarantee**, with the
deferred work tracked here — _not_ left to crash and _not_ force-fixed now.
Genuine bugs in scenes that are supposed to run are fix-now. The smoke must
distinguish "intentionally quarantined" from "unexpectedly broken" so a
regression in a runnable scene still fails loudly.

---

## M1 — Best domain + solver, done excellently

**Definition of done:** one flagship domain + solver pair is the reference-grade
visual inspection experience — every relevant feature visible, with clear
panels, debug overlays, replay, and capture, plus a side-by-side trust check
against a known-good baseline.

**Domain:** rigid body (multibody) — DART's core competency and the most widely
used domain in robotics.

### Solver decision (CONFIRMED)

**Direction: AVBD flagship + DART 6 boxed-LCP reference baseline.** Confirmed by
the task owner. IPC remains the deferred accuracy/no-penetration specialist.

| Option                                                           | What it is                                                                                                                         | Role in M1                                                                              |
| ---------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------- |
| **DART 6 boxed-LCP** (Dantzig / PGS, `BoxedLcpConstraintSolver`) | The battle-tested constraint solver robotics users (DART 6, Gazebo / gz-physics) rely on today                                     | **Reference baseline** — the known-good for visual A/B trust checks                     |
| **AVBD** (Augmented Vertex Block Descent, 2024)                  | The modern successor/generalization of XPBD/VBD; robust at high stiffness and high mass ratios; unifies rigid → articulated → soft | **Flagship** — the modern path we showcase and inspect deeply                           |
| **IPC** (Incremental Potential Contact, 2020)                    | Guaranteed intersection-free contact; high fidelity but compute-heavy                                                              | **Specialist** — reserve for the accuracy / no-penetration showcase, not the M1 default |

Rationale:

- **Adoption + trust:** boxed-LCP is what real DART/Gazebo users run; keeping it
  as the visual reference lets users trust the modern path by comparison. The
  harness already has the comparison scaffolding (`rigid_solver_compare`,
  `rigid_contact_solver_compare`, `rigid_multibody_solver_family`).
- **Modern differentiator:** AVBD is the most modern approach DART 7 ships and
  already has deep investment (`docs/dev_tasks/avbd_solver/`, dozens of AVBD
  scenes). It handles stiff/high-mass-ratio cases that trip classic solvers —
  exactly the failure modes worth _visually_ inspecting.
- **Sets up "beyond M1":** AVBD unifies rigid → articulated → soft, so the same
  flagship path expands outward cleanly into later milestones.
- **Why not IPC as default:** its strength (provable non-penetration) comes with
  compute cost and it is less of a default in interactive robotics; it shines as
  a dedicated accuracy showcase, not the everyday rigid workhorse.

> Decision locked: AVBD-flagship + LCP-baseline (IPC deferred to a later
> milestone as the accuracy/no-penetration specialist).

### M1 architecture reality (verified in source)

DART 7's solver surfaces are not one flat list — this shapes how M1 wires the
flagship comparison (see `dart/simulation/world_options.hpp`):

- **`World.RigidBodySolver`** (the free rigid-body solver _family_ in the
  built-in `World::step()` schedule) has exactly two values:
  `SequentialImpulse` (default) and `Ipc`. **AVBD is not here.**
- **`ContactSolverMethod`** is a separate contact-stage knob:
  `SequentialImpulse` (Gauss-Seidel) vs **`BoxedLcp`** — the Dantzig-style
  pivoting boxed-LCP, i.e. the **DART 6-equivalent** contact solve.
- **AVBD is its own solver track** (PLAN-104, the `avbd_*` scenes / `sx` AVBD
  path), not selectable via `World.RigidBodySolver`.

Implication for the locked decision: the "DART 6 LCP baseline" is the
`World` rigid path with `ContactSolverMethod::BoxedLcp` (and/or the default
`SequentialImpulse`), and the "AVBD flagship" is the separate AVBD track shown
alongside it. The flagship visual comparison therefore spans two solver
_tracks_, not two values of one enum.

### M1 engagement mechanism (verified post-M0)

Investigated against the built binding. AVBD and LCP are **complementary, not
head-to-head** on a single scenario:

- **Contact solve** (bodies touching): `world.rigid_body_solver` ∈
  {`SEQUENTIAL_IMPULSE`, `IPC`} and `world.contact_solver_method` ∈
  {`SEQUENTIAL_IMPULSE`, `BOXED_LCP`}. **`BOXED_LCP` is the DART 6 baseline.**
- **Rigid constraints** (joints, motors, breakable joints): **AVBD**, engaged
  implicitly by using the `world.add_rigid_body_*_joint(...)` API — there is no
  AVBD enum. The `avbd_*` scenes are this track.
- Both tracks use the **same `sx.World` + `WorldRenderBridge` + `SceneSetup`**
  pattern (verified: `avbd_rigid_revolute_motor.py` vs `rigid_body.py`), so a
  unified flagship scene is feasible at the construction level.

So the M1 flagship is **two complementary showcases**, both reference-grade:

1. **Contact-solver comparison** — SI / IPC / BoxedLcp on contact-rich rigid
   scenes (BoxedLcp = the DART 6 trust baseline). First increment: the flagship
   `rigid_body` scene exposes `contact_solver_method` while keeping its
   rigid-body solver fixed to the realtime Sequential Impulse path; the
   dedicated `rigid_solver_compare` scene owns SI-vs-IPC visual inspection.
2. **AVBD constraint showcase** — joints / motors / breakable constraints solved
   by AVBD (the modern differentiator), curated from the `avbd_*` scenes.

**Remaining M1 scoping point for the task owner:** whether the completed M1
experience should stay as complementary contact and AVBD scenes, or grow into a
new unified comparison scene.

---

## Beyond M1 — expand outward, repeating

Repeat the M1 pattern one capability at a time: add the next solver and/or
domain (articulated, then soft/deformable: VBD, IPC-deformable; other solvers:
variational integrators, unified Newton-barrier), each promoted into the live
catalog only once it is no-crash and verified. Breadth never outruns stability.
