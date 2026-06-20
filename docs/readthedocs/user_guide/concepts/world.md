# The World

`dart.World` is the heart of DART 7. A single `World` object owns your scene's
topology (bodies, multibodies, and the connections between them), the simulation
clock, gravity, and the step pipeline that advances physics. Almost everything
you do in DART starts from a world.

## Creating a world

You can create a world with defaults and configure it later, or pass options up
front:

```python
import dartpy as dart

# Defaults
world = dart.World()

# Configured at construction
world = dart.World(
    time_step=1.0 / 1000.0,
    rigid_body_solver=dart.RigidBodySolver.SEQUENTIAL_IMPULSE,
    contact_solver_method=dart.ContactSolverMethod.SEQUENTIAL_IMPULSE,
)
```

Solver choices are covered in {doc}`../interaction/solvers`.

## Two modes: design, then simulate

A DART world has two phases, and keeping them separate is what makes stepping
fast and predictable:

- **Design mode** (the default after construction) is where you _build_ the
  scene: add rigid bodies, add multibodies, attach shapes, and set initial
  state.
- **Simulation mode** is where you _run_ it. Calling `enter_simulation_mode()`
  finalizes the topology so DART can allocate runtime state once, up front.

```python
# --- design mode: build the scene ---
ground = world.add_rigid_body("ground", ground_opts)
robot = world.add_multibody("arm")
# ...

# --- switch to simulation mode, then step ---
world.enter_simulation_mode()
world.step()
```

`world.step()` will enter simulation mode for you on the first call, so the
explicit call is mainly there when you want deterministic allocation or to catch
topology errors at a known point. After finalization, restructuring the scene
requires an explicit reset or rebuild rather than ad-hoc edits — this is what
lets the per-step path stay allocation-free.

## What a step does

`world.step()` runs a **content-aware pipeline**: it executes only the stages
your scene needs. A scene of free rigid bodies runs rigid-body integration and
contact resolution; add an articulated robot and the multibody dynamics stage
joins in; the pipeline finishes with a kinematics refresh so every transform you
read afterwards is current. You select _capabilities and policies_ (which solver,
which contact method) rather than wiring stages together by hand.

## Kinematics without dynamics

Sometimes you only need to update where things are — for motion planning,
collision queries, playback, or visualization — without integrating forces. Set
the poses you care about and refresh the frames:

```python
world.enter_simulation_mode()
# ... set joint or body positions ...
world.update_kinematics()     # refresh transforms; no force integration
```

This reuses the _same_ world, bodies, and frames as full physics, so you never
have to maintain a separate "kinematics-only" copy of your scene.

## Inspecting the world

A world exposes simple lookups and counters for the objects it owns, plus the
clock you read in the loop:

```python
world.time            # current simulated time (seconds)
world.time_step       # integration interval
world.collide()       # run collision detection now; returns the contacts
```

## Next

With the world understood, the next two pages cover the things you put inside it:
{doc}`rigid bodies <rigid_bodies>` and {doc}`articulated systems
<articulated_systems>`.
