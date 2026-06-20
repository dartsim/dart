# Hello, DART

Let's write the smallest useful DART program: drop a box onto the ground and
watch it fall. Every DART simulation follows the same three beats — **build a
world, lock it, step it** — and this example shows all three.

## The full program

```python
import dartpy as dart
import numpy as np

# 1. Create a world. The time step is the physics integration interval.
world = dart.World(time_step=1.0 / 1000.0)
world.gravity = np.array([0.0, 0.0, -9.81])

# 2. Add a static ground plate. CollisionShape.box takes *half* extents, so this
#    is a 4 m x 4 m x 0.1 m slab; placing its center at z = -0.05 puts the top
#    surface at z = 0.
ground_opts = dart.RigidBodyOptions()
ground_opts.is_static = True
ground_opts.position = np.array([0.0, 0.0, -0.05])
ground = world.add_rigid_body("ground", ground_opts)
ground.set_collision_shape(dart.CollisionShape.box(np.array([2.0, 2.0, 0.05])))

# 3. Add a 1 kg box one metre above the ground.
box_opts = dart.RigidBodyOptions()
box_opts.mass = 1.0
box_opts.position = np.array([0.0, 0.0, 1.0])
box = world.add_rigid_body("box", box_opts)
box.set_collision_shape(dart.CollisionShape.box(np.array([0.1, 0.1, 0.1])))

# 4. Lock the topology, then advance the simulation.
world.enter_simulation_mode()
for step in range(1000):
    world.step()
    if step % 100 == 0:
        height = float(box.translation[2])
        print(f"t = {world.time:5.3f} s   box height = {height:6.4f} m")
```

Running it prints the box falling and settling on the ground:

```text
t = 0.000 s   box height = 1.0000 m
t = 0.100 s   box height = 0.9519 m
t = 0.200 s   box height = 0.8076 m
t = 0.300 s   box height = 0.5670 m
t = 0.400 s   box height = 0.2303 m
t = 0.500 s   box height = 0.1000 m
...
```

(The exact numbers depend on your DART version and solver settings; the shape of
the motion — accelerate, then rest on the ground — is what matters.)

## What each step does

1. **Build a world.** {doc}`dart.World <../concepts/world>` is the single object
   that owns everything: bodies, time, gravity, and the step pipeline. The
   `time_step` is how much simulated time each `step()` advances.

2. **Add bodies.** A `RigidBodyOptions` value object carries the initial state
   (mass, pose, velocity, whether the body is static). `add_rigid_body` returns
   a handle you keep to read or change the body later. A _static_ body never
   moves — perfect for ground and walls. Giving a body a **collision shape**
   lets it take part in contact.

3. **Lock the topology.** `enter_simulation_mode()` finalizes the scene so DART
   can allocate runtime state. After this point you step rather than restructure;
   see {doc}`../concepts/world` for the design-mode / simulation-mode split.

4. **Step.** Each `world.step()` integrates the physics forward by one
   `time_step` and refreshes every body's state. Reading `box.translation`
   afterwards returns the fresh world-frame position.

## Things to try

- Change `box_opts.position` to start the box off-center and watch it topple as
  it lands.
- Give the box an initial spin with `box_opts.angular_velocity =
np.array([0.0, 4.0, 0.0])`.
- Add a second box with a different name and mass, then print both heights.

## Next

You have a running simulation. Next, look more closely at the loop that drives
it in {doc}`simulation_loop`.
