Section: Examples

  * Replace the C++ `dart-demos` application with the Python demo
    catalog (`python -m examples.demos`). It runs on the same native workspace:
    - navigator and scene panels;
    - inspector, contact and drag tools, and profiler;
    - log console;
    - headless list, cycle and capture.

    `pixi run demos`, `cycle-demos` and `capture` launch it, and `py-demos` is
    removed. The `dart-demos`, `soft_bodies`, `soft_cubes` and
    `soft_open_chain` executables are removed, as is the TinyDNN-only
    `human_joint_limits` scene. Use
    `pixi run demos -- --scene soft_bodies`, with `--collision-detector` and
    `--threads` as before.
    [#3762](https://github.com/dartsim/dart/pull/3762),
    [#3780](https://github.com/dartsim/dart/pull/3780)
