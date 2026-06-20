# Choosing solvers

DART solves two related problems each step: how to **integrate** body motion and
how to **resolve contact**. DART 7 lets you choose the method for each so you can
trade accuracy against speed for your scene. You select methods by _capability_,
not by naming an internal backend.

## Setting solver methods

Choose methods when you create the world:

```python
import dartpy as dart

world = dart.World(
    time_step=1.0 / 1000.0,
    rigid_body_solver=dart.RigidBodySolver.SEQUENTIAL_IMPULSE,
    contact_solver_method=dart.ContactSolverMethod.SEQUENTIAL_IMPULSE,
)
```

The same choices are available as world properties, so you can switch between
runs:

```python
world.rigid_body_solver = dart.RigidBodySolver.SEQUENTIAL_IMPULSE
world.contact_solver_method = dart.ContactSolverMethod.BOXED_LCP
```

## Contact solver methods

Two contact methods are useful to know:

- **Sequential impulse** (`ContactSolverMethod.SEQUENTIAL_IMPULSE`) iterates over
  contacts applying impulses. It is fast and robust for real-time, contact-rich
  scenes — a good default.
- **Boxed LCP** (`ContactSolverMethod.BOXED_LCP`) formulates contact as a linear
  complementarity problem. It is the more traditional DART formulation and a
  useful reference when comparing results.

If you are unsure, start with sequential impulse and switch to boxed LCP when you
want to cross-check behavior.

## Thinking in capabilities

DART describes solvers by what they _do_ rather than by an engine name. When you
reach for more advanced behavior, you are really choosing along axes like these:

| Capability         | Example choices                                         |
| ------------------ | ------------------------------------------------------- |
| Integration family | semi-implicit Euler, implicit Euler, variational        |
| Constraint solve   | projected Gauss-Seidel / sequential impulse, direct LCP |
| Coordinates        | generalized (articulated), maximal (free rigid bodies)  |
| Supported features | contacts, joints, friction, closed chains               |

This vocabulary is stable even as implementations improve underneath, which means
the method you request keeps working while DART gets faster.

```{admonition} A developing area
:class: note

The exact set of solver and contact methods is still expanding in DART 7. The
[`rigid_solver_compare` and `rigid_contact_solver_compare` demos](https://github.com/dartsim/dart/tree/main/python/examples/demos)
are the best way to *see* the differences between methods side by side. Treat the
enum values above as the current, stable starting set rather than the final list.
```

## See the differences

Run the comparison demos from a source checkout to watch methods diverge on the
same scene:

```bash
pixi run py-demos -- --scene rigid_solver_compare
pixi run py-demos -- --scene rigid_contact_solver_compare
```

## Next

To watch any of this happen, open the {doc}`viewer <../visualization>`.
