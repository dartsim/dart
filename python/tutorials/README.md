# Python tutorials

These four tutorials teach DART's simulation and control APIs through numbered
Python exercises. Each folder contains `main.py` with the exercises and
`main_finished.py` with their answers. Folder names omit the redundant
`tutorial_` prefix.

| Folder | Topics | Published lessons |
| --- | --- | --- |
| `multi_pendulum` | Shapes, joint/body forces, springs, damping, constraints | [Multi Pendulum](https://dart.readthedocs.io/en/latest/dartpy/user_guide/tutorials/multi-pendulum.html) |
| `biped` | Joint limits, PD/SPD, balance, skeleton editing, wheel actuators, IK | [Biped](https://dart.readthedocs.io/en/latest/dartpy/user_guide/tutorials/biped.html) |
| `collisions` | Rigid/soft/hybrid bodies, frames, spawning, closed rings | [Collisions](https://dart.readthedocs.io/en/latest/dartpy/user_guide/tutorials/collisions.html) |
| `dominoes` | Cloning, URDF loading, robot control, replay, contact forces | [Dominoes](https://dart.readthedocs.io/en/latest/dartpy/user_guide/tutorials/dominoes.html) |

From the repository root, build dartpy and launch a tutorial:

```console
pixi run build-py-dev
pixi run tu-multi-pendulum-fi
```

The tutorial tasks build dartpy before running the script. Remove `-fi` to run
an exercise:

| Exercise | Finished solution |
| --- | --- |
| `pixi run tu-multi-pendulum` | `pixi run tu-multi-pendulum-fi` |
| `pixi run tu-biped` | `pixi run tu-biped-fi` |
| `pixi run tu-collisions` | `pixi run tu-collisions-fi` |
| `pixi run tu-dominoes` | `pixi run tu-dominoes-fi` |

The parameterized task is also available, for example `pixi run py-tu biped`
or `pixi run py-tu biped_finished`. With dartpy installed in your Python
environment, run any script directly:

```console
python python/tutorials/multi_pendulum/main.py
python python/tutorials/multi_pendulum/main_finished.py
python python/tutorials/biped/main.py
python python/tutorials/biped/main_finished.py
python python/tutorials/collisions/main.py
python python/tutorials/collisions/main_finished.py
python python/tutorials/dominoes/main.py
python python/tutorials/dominoes/main_finished.py
```

The OSG viewer requires a desktop session with OpenGL. Scripts print their
controls, and Space starts or pauses physics. The biped model is Y-up; the
other scenes are Z-up. Sample models resolve through `dart://sample/`.
Unfinished structural exercises raise `NotImplementedError` identifying the
lesson that must be implemented before the scene can run.

| Scene | Tutorial controls |
| --- | --- |
| Multi-pendulum | `1`–`9`/`0` select a coordinate/body; `-` reverses force; `f` switches torque/body force; `q`/`a` adjust rest angles; `w`/`s` adjust stiffness; `e`/`d` adjust damping; `r` toggles the tip constraint; `p` replays |
| Biped | `.`/`,` push forward/backward; `a`/`A` increase wheel speed; `s`/`S` decrease wheel speed |
| Collisions | `1`–`5` toss a ball, soft body, hybrid, chain, or ring; `d` deletes the oldest tossed object; `r` toggles randomization |
| Dominoes | Before first start: `q`/`w`/`e` place left/straight/right; `d` deletes the last clone. After starting: `f` applies an external push; `r` uses the robot. `p` replays and `v` toggles contact arrows |

Each script exposes `build_scene()` returning a world node without opening a
window. The node retains its world, controller (when applicable), and handler.
Use its actual `customPreStep()` / `customPostStep()` callbacks when stepping
scenes without a viewer, so controllers and recordings follow the same path
as the interactive application.

The previous `chain/main.py` and `chain/main_finished.py` commands forward to
the multi-pendulum exercise and solution. The C++ tutorial executables and
`tutorials` aggregate CMake target have been retired.
