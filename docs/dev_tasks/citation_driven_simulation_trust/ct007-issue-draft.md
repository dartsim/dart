# Issue draft: default contact solver fails high-mass-ratio stacks

Status: DRAFT, not posted. Posting to GitHub needs maintainer approval.
Working copy for the CT-007 finding; delete with this dev-task folder after
the issue is filed (or a decision is recorded not to file it).

---

Title: `SEQUENTIAL_IMPULSE contact solver lets a heavy box sink completely
through a light one at mass ratios >= 100`

**Environment:**

- DART version: `main` (20501341226; reproduced during PLAN-123 CT-007 work)
- OS: Linux (Ubuntu-based, kernel 7.0.0)
- Installation: source, `pixi run build` (Release)
- Compiler: repository default toolchain

**Expected vs Current Behavior:**

A rigid two-box stack at rest should stay stacked regardless of the mass
ratio between the boxes. With the DART 7 `World` default contact solver
(`ContactSolverMethod.SEQUENTIAL_IMPULSE`), a heavy box resting on a light
one sinks completely through it once the mass ratio reaches ~100:1: over
about one second the heavy box descends exactly one box height, the pair
comes to rest fully interpenetrated (box centers coincident) at near-zero
velocity, and stays that way. The light box below barely moves (microns), so
this is the box-box contact failing, not the ground contact. The state stays
finite and the run is bit-deterministic, which makes the failure silent.

`ContactSolverMethod.BOXED_LCP` holds the identical stack across every tested
ratio (1 to 1000), with steady-state overlap in the 1e-5 m range.

**Steps to Reproduce:**

```python
import dartpy as dart
import numpy as np

RATIO = 100.0  # upper box mass / lower box mass

for method in ("SEQUENTIAL_IMPULSE", "BOXED_LCP"):
    world = dart.simulation.World(
        time_step=0.002,
        contact_solver_method=dart.simulation.ContactSolverMethod[method],
    )
    ground = world.add_rigid_body("ground")
    ground.is_static = True
    ground.set_collision_shape(
        dart.simulation.CollisionShape.box(np.array([1.0, 1.0, 0.05]))
    )
    tf = np.eye(4); tf[2, 3] = -0.05
    ground.transform = tf

    boxes = []
    for i, mass in enumerate((1.0, RATIO)):
        body = world.add_rigid_body(f"box{i}")
        body.mass = mass
        moment = mass * (0.2**2 + 0.2**2) / 12.0
        body.inertia = np.diag([moment] * 3)
        body.set_collision_shape(
            dart.simulation.CollisionShape.box(np.array([0.1, 0.1, 0.1]))
        )
        body.friction = 0.8
        body.restitution = 0.0
        tf = np.eye(4); tf[2, 3] = 0.1 + i * 0.2  # exactly stacked, zero gap
        body.transform = tf
        boxes.append(body)

    world.enter_simulation_mode()
    print(f"--- {method} ---")
    for step in range(1001):
        if step % 100 == 0:
            z0 = float(boxes[0].translation[2])
            z1 = float(boxes[1].translation[2])
            print(f"t={step*0.002:4.1f}s  gap={z1 - z0 - 0.2:+.5f} m")
        world.step()
```

Observed output (gap = center separation minus one box height; 0 means
resting contact, -0.2 means the boxes fully coincide):

```
--- SEQUENTIAL_IMPULSE ---        --- BOXED_LCP ---
t= 0.0s  gap=+0.00000 m           t= 0.0s  gap=-0.00000 m
t= 0.4s  gap=-0.13251 m           t= 0.4s  gap=-0.00001 m
t= 0.8s  gap=-0.19965 m           t= 0.8s  gap=-0.00007 m
t= 1.0s  gap=-0.19999 m           t= 1.0s  gap=-0.00002 m
t= 2.0s  gap=-0.19999 m           t= 2.0s  gap=-0.00002 m
```

Sweep over mass ratio (steady-state relative closure of the loaded contact
at a 2 s horizon; 1.0 = fully interpenetrated):

| solver             | 1:1    | 10:1   | 100:1      | 1000:1     |
| ------------------ | ------ | ------ | ---------- | ---------- |
| SEQUENTIAL_IMPULSE | 1.4e-4 | 1.0e-3 | **1.0000** | **1.0000** |
| BOXED_LCP          | 2.0e-4 | 2.0e-4 | 3.3e-5     | -3.0e-4    |

**Notes:**

- The scene starts with exactly zero overlap, so the closure is produced by
  the solve, not by initial interpenetration recovery.
- Both outcomes are bit-identical across repeated runs.
- The behavior is consistent with iterative Gauss-Seidel contact under a
  fixed iteration budget failing to propagate impulses through a
  high-mass-ratio contact pair, but the mechanism is not confirmed: the
  sequential-impulse path does not expose a per-solve residual
  (`StepMetrics.last_step_residual` is structurally zero on the rigid contact
  path), so there is no algebraic signal to inspect. Exposing a comparable
  residual is tracked as PLAN-123 WS4 work.
- Mass ratios of 100:1 are ordinary in robotics scenes (a 100 kg robot
  standing on a 1 kg object), and the failure is silent: finite state,
  deterministic, no warning.
- Workaround: construct the `World` with
  `contact_solver_method=dart.simulation.ContactSolverMethod.BOXED_LCP`.
- DART 6 (`release-6.20`) is not affected on its default path: the classic
  constraint pipeline defaults to the Dantzig boxed LCP. This is specific to
  the DART 7 `World` sequential-impulse contact stage.
- Full evidence packet (raw rows, per-cell resolved solver identity,
  deterministic repeat hashes): `CT-007-dart7-high-mass-ratio.json` under
  `docs/plans/123-citation-driven-simulation-trust/evidence/` on the
  PLAN-123 branch (pending PR).
