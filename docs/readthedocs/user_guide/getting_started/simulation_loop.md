# The simulation loop

Every DART program is a loop around `world.step()`. This page explains what a
step is, how time and gravity work, and how to read results back out.

## Time steps

A `World` advances in fixed increments called the **time step**. Set it when you
create the world, or change it later through the `time_step` property:

```python
import dartpy as dart

world = dart.World(time_step=1.0 / 1000.0)   # 1 ms per step
print(world.time_step)                       # -> 0.001
```

Each call to `world.step()` advances simulated time by exactly one time step and
updates `world.time`:

```python
world.enter_simulation_mode()
for _ in range(500):
    world.step()
print(f"simulated {world.time:.3f} seconds")   # -> 0.500 seconds
```

Smaller time steps are more accurate and more stable but cost more per second of
simulated time. A common starting point is `1/1000` s for contact-rich scenes
and `1/240`–`1/120` s for lighter ones. To advance a fixed amount of wall-clock
behavior, choose the time step first, then loop the right number of steps.

## Gravity

Gravity is a world-level vector in metres per second squared. DART pulls along
**−Z** by default; set it explicitly when you want a different magnitude or
direction:

```python
import numpy as np

world.gravity = np.array([0.0, 0.0, -9.81])   # Earth, −Z is "down"
world.gravity = np.array([0.0, 0.0, -1.62])   # the Moon
world.gravity = np.array([0.0, 0.0, 0.0])     # free space
```

## Reading state during the loop

After each step, body and joint state is fresh, so you can read it directly.
Handles returned by `add_rigid_body` expose the body's current pose and motion:

```python
world.enter_simulation_mode()
for step in range(1000):
    world.step()
    position = np.asarray(box.translation)        # world-frame xyz
    velocity = np.asarray(box.linear_velocity)    # world-frame linear velocity
    if step % 100 == 0:
        speed = float(np.linalg.norm(velocity))
        print(f"t={world.time:5.3f}  z={position[2]:.3f}  speed={speed:.3f}")
```

This is also where a controller belongs: read state at the top of the loop,
compute a command, apply it, then step. Keeping _read → decide → act → step_ in
that order makes the loop deterministic and easy to test.

## A note on determinism

`world.step()` is **synchronous and deterministic**: when it returns, the step
is complete and every output it produced is fresh. The same world, built the
same way and stepped the same number of times, produces the same result. That
predictability is deliberate — it keeps simulations reproducible for research,
testing, and controller development.

## Next

You now know the loop. The next section looks at the central object it revolves
around: {doc}`the World <../concepts/world>`.
