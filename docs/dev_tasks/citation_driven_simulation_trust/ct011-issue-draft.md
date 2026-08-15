# Issue draft: state-vector restore depends on prior contact history

Status: DRAFT, not posted. Posting to GitHub needs maintainer approval.
Working copy for the CT-011 finding; delete with this dev-task folder after
the issue is filed (or a decision is recorded not to file it).

---

Title: `World.state_vector restore is not a function of the restored state
once the world has contact history`

**Environment:**

- DART version: `main` (20501341226; found during PLAN-123 CT-011 work)
- OS: Linux (Ubuntu-based, kernel 7.0.0)
- Installation: source, `pixi run build` (Release)
- Compiler: repository default toolchain

**Expected vs Current Behavior:**

For reset-heavy research workflows (RL episodes, planning rollouts,
trajectory optimization), `world.state_vector = snapshot` should make the
subsequent trajectory a function of the restored state: restoring the same
snapshot should always produce the same continuation. On `main` it does not,
once the world has ever had contact:

- Restoring a snapshot in place and re-stepping diverges from the original
  continuation at the **first** post-restore step (max state-vector delta
  ~5e-3 to 1e-2 within a 0.2 s window in the repro below).
- Restoring the **same snapshot twice** into the same world produces **two
  different continuations** when different amounts of stepping precede the
  two restores.
- A **freshly built** world restoring the same snapshot is bit-exact and
  repeatable; two separately built worlds with identical step histories
  agree bit-exactly; a world whose history ends before its first contact
  behaves like a fresh world; and a ballistic (no-contact) scene restores
  bit-exactly.

Everything is deterministic given full history — this is not nondeterminism.
Some result-affecting contact state survives the `state_vector` write, so
the continuation depends on what the world did before the restore. The
failure is silent: state stays finite and each individual run is
reproducible, which makes it easy to mistake restored rollouts for
equivalent ones. Both `SEQUENTIAL_IMPULSE` and `BOXED_LCP` are affected
identically, which points at shared contact-pipeline state rather than a
solver-specific cache. Deactivation/sleeping is ruled out: no body was
asleep at the snapshot, and disabling deactivation changes nothing.

**Steps to Reproduce:**

```python
import hashlib
import dartpy as dart
import numpy as np

def build(method):
    w = dart.simulation.World(
        time_step=0.002,
        contact_solver_method=dart.simulation.ContactSolverMethod[method])
    g = w.add_rigid_body("g"); g.is_static = True
    g.set_collision_shape(dart.simulation.CollisionShape.box(np.array([1., 1., .05])))
    tf = np.eye(4); tf[2, 3] = -0.05; g.transform = tf
    for i in range(5):
        b = w.add_rigid_body(f"s{i}")
        b.mass = 0.5
        b.inertia = np.diag([0.4 * 0.5 * 0.06**2] * 3)
        b.set_collision_shape(dart.simulation.CollisionShape.sphere(0.06))
        b.friction = 0.6; b.restitution = 0.2
        tf = np.eye(4)
        tf[:3, 3] = (0.03 * (i % 2 * 2 - 1) * (i + 1) / 5,
                     0.025 * ((i // 2) % 2 * 2 - 1) * (i + 1) / 5,
                     0.2 + 0.13 * i)
        b.transform = tf
    w.enter_simulation_mode()
    return w

def run_hash(w, k=100):
    h = hashlib.sha256()
    for _ in range(k):
        w.step()
        h.update(np.ascontiguousarray(w.state_vector).tobytes())
    return h.hexdigest()

for method in ("SEQUENTIAL_IMPULSE", "BOXED_LCP"):
    w = build(method)
    for _ in range(250):  # settle: contact starts around step 85
        w.step()
    snap = np.array(w.state_vector, copy=True); t0 = w.time

    h_continue = run_hash(w)                      # original continuation
    w.state_vector = snap; w.time = t0
    h_restore1 = run_hash(w)                      # in-place restore #1
    w.state_vector = snap; w.time = t0
    h_restore2 = run_hash(w)                      # in-place restore #2

    f1 = build(method); f1.state_vector = snap; f1.time = t0
    f2 = build(method); f2.state_vector = snap; f2.time = t0
    h_fresh1, h_fresh2 = run_hash(f1), run_hash(f2)

    print(f"--- {method} ---")
    print("continue == restore1:", h_continue == h_restore1)   # False
    print("restore1 == restore2:", h_restore1 == h_restore2)   # False
    print("fresh1   == fresh2:  ", h_fresh1 == h_fresh2)       # True
```

Observed on `main` for both solvers:

```
continue == restore1: False
restore1 == restore2: False
fresh1   == fresh2:   True
```

Control results (from the full evidence protocol): identical-history worlds
agree bit-exactly; a pre-contact-history world matches a fresh world;
ballistic scenes restore bit-exactly; `update_kinematics()` after the
restore changes nothing.

**Notes:**

- The DART 7 design docs already state the requirement this violates:
  "Cloning, serialization, and reset preserve or intentionally clear all
  result-affecting configuration and history" and "Reset semantics must
  explicitly choose whether solver/contact history is preserved"
  (`docs/design/contact_trust_and_observability.md`). Currently the choice
  is implicit and history leaks through.
- Either resolution is defensible — clear contact history on state writes,
  or capture it in the state vector — but the semantics should be explicit,
  documented, and tested. The one-model/many-state work (PLAN-030/PLAN-123
  WS7) needs this settled.
- Workaround: restore into a freshly built world; that is bit-exact and
  repeatable.
- Full evidence packet (seven protocol arms, all hashes, deterministic
  repeats): `CT-011-dart7-restore-equivalence.json` under
  `docs/plans/123-citation-driven-simulation-trust/evidence/` on the
  PLAN-123 branch (pending PR).
