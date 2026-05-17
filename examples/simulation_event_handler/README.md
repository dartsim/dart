# Simulation Event Handler Example

This example demonstrates `dart::gui` keyboard actions, panels, world-owned
sensors, and DART-owned force-arrow visuals for a simple rigid-body scene.

Run it with:

```bash
pixi run ex simulation_event_handler
```

Headless capture:

```bash
pixi run ex simulation_event_handler --headless --frames 2 --screenshot /tmp/simulation_event_handler.ppm
```

Controls:

- Space: play/pause through the shared viewer control
- `s`: step one frame
- `r`: reset the simulation
- Tab / Backspace: select the next or previous rigid body
- Arrow keys and `u`/`d`: apply forces to the selected body
- `q`/`w`/`e` and `a`/`z`/`c`: apply positive and negative torques
- `=` / `-`: adjust force and torque magnitudes
- `.` / `,`: adjust the simulation timestep
- `v`: toggle force-arrow visualization
- `i`: print simulation state
- `h` / `?`: print controls
