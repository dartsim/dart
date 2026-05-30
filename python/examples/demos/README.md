# dart-demos (Python)

A headless scene-registry runner that hosts DART's Python demo scenes. This is
the Python-first surface from PLAN-103; C++ `dart-demos` (PLAN-102) stays frozen.

## Run

```bash
pixi run py-demos                                # default: run the first scene
pixi run py-demos -- --scene sx_articulated      # select a scene by id
pixi run py-demos -- --cycle-scenes --frames 4   # cycle through every scene
pixi run py-demos -- --list                      # print the scene catalog
```

Or directly (without pixi), from the repo root:

```bash
PYTHONPATH=build/default/cpp/Release/python:python \
    python -m examples.demos --scene hello_world --frames 5
```

## CLI

The runner mirrors C++ `dart-demos` to keep the cross-language UX consistent:

| Flag                | Meaning                                                    |
| ------------------- | ---------------------------------------------------------- |
| `--scene <id>`      | Select the initial scene (default: first registered)       |
| `--cycle-scenes`    | Advance through every scene for `--frames` frames and exit |
| `--frames N`        | Per-scene step budget (default 60 single, 4 cycle)         |
| `--screenshot PATH` | Phase 1: write a JSON state snapshot at PATH               |
| `--headless`        | Accepted no-op (Python demos are always headless)          |
| `--list`            | Print the catalog and exit                                 |

`--screenshot` writes a real non-blank PPM once Phase 2 wires the `dartpy.gui`
headless screenshot path; today it writes a deterministic JSON state snapshot
so the contract works end-to-end.

## IPC Deformable (sx) scenes

The dedicated **`IPC Deformable (sx)`** category groups the deformable-body
scenes driven by the `dartpy.simulation_experimental` IPC solver
(point-mass/spring with IPC-style clamped-log barriers, sparse projected Newton,
lagged smoothed-Coulomb friction, and conservative CCD). They live in their own
category so the IPC showcases stay together instead of mixing into the general
`Experimental` (sx) scenes:

| Scene id                            | Shows                                                 | IPC capability exercised                   |
| ----------------------------------- | ----------------------------------------------------- | ------------------------------------------ |
| `ipc_deformable_net`                | A pinned spring net sagging/swaying under gravity     | Projected-Newton elastodynamics            |
| `ipc_deformable_drape`              | A mat draping over a step onto a ground barrier       | Ground + self-contact clamped-log barrier  |
| `ipc_deformable_trampoline`         | A corner-pinned membrane sagging and rebounding       | Taut-membrane projected-Newton dynamics    |
| `ipc_deformable_friction_slide`     | A launched mat skidding to rest on a frictional floor | Lagged smoothed-Coulomb ground friction    |
| `ipc_deformable_fem_bar`            | A tetrahedral cantilever sagging under gravity        | Stable neo-Hookean **FEM** elasticity      |
| `ipc_deformable_fem_twist`          | A tetrahedral bar twisted at both ends, then released | **FEM** volumetric shear (toward Fig 4/14) |
| `ipc_deformable_scripted_dirichlet` | A banner billowing under a scripted Dirichlet BC      | Scripted Dirichlet boundary conditions     |

These are DART-native showcases, **not** faithful IPC paper-figure
reproductions. The _contact, barrier, friction, and projected-Newton machinery_
is genuine IPC; most scenes use a mass-spring elasticity model, while
`ipc_deformable_fem_bar` and `ipc_deformable_fem_twist` exercise the new opt-in
stable neo-Hookean **FEM** volumetric elasticity (PLAN-081 M1, the keystone
toward the paper's volumetric scenes). Codimensional collision and the upstream
mesh corpus are still needed for true figure reproduction. The catalog of upstream paper scenes lives at
`docs/plans/081-deformable-implicit-barrier-solver/ipc_scene_corpus_manifest.json`;
faithful corpus reproduction stays `planned` (it needs vendored assets,
codimensional collision, and broader FEM coverage), so these scenes evoke the
paper's _themes_ (volumetric elasticity, draping, self-contact, friction,
scripted boundaries) rather than reproducing
its figures.

The scenes share `_ipc_deformable_bridge.py`, which builds the grid topology and
mirrors each deformable onto the render world as per-node spheres plus a spring
wireframe (the dynamic surface-mesh render path is not yet exposed through
`dartpy.gui.run_demos`). Add another deformable scene by building its
`DeformableBodyOptions` with `build_grid_options(...)` and rendering it through
`IpcDeformableBridge`.

## Add a scene

1. Create `scenes/<name>.py` defining a module-level `SCENE` of type
   `PythonDemoScene` whose `build()` returns a `SceneSetup` (a `world` plus an
   optional custom `step(n)` and `info` dict).
2. Import it in `registry.py` and append it to the ordered list in
   `make_demo_scenes()`, placed within its category group.

That's the only change needed; no CMake edits, no test edits beyond the cycle
smoke (which iterates the registry).
