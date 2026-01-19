# Simulation Experimental Hello World

Demonstrates the ECS-based experimental simulation API.

## What This Example Shows

- Creating a `World` with the experimental API
- Building a `MultiBody` with multiple links and joints
- Setting joint positions via handles
- Creating a `RigidBody` with mass/inertia properties
- Applying forces to rigid bodies

## Build

From this directory:

```bash
mkdir build && cd build
cmake ..
make
```

Or from the DART repo root:

```bash
pixi run build-examples
```

## Run

```bash
./simulation_experimental_hello_world
```

## Expected Output

```
=== Simulation Experimental Hello World ===

Created base link: base

Robot structure:
  Links: 4
  Joints: 3
  DOFs: 3

Joint positions:
  shoulder: 0.5
  elbow: -0.3
  wrist: 0.1

Rigid body:
  Name: falling_box
  Mass: 1 kg
  Position: 0 0 2
  Applied force: 0 0 -9.81 N

World summary:
  MultiBodies: 1
  RigidBodies: 1

=== Done ===
```

## Notes

- Physics simulation (`World::step()`) is not yet implemented (Phase 5)
- No GUI visualization (experimental API doesn't have GUI integration yet)
- Joint positions don't compute link transforms yet (forward kinematics is Phase 5)
