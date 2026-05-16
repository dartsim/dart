# LCP Physics Example

## Summary

- Goal: exercise contact-heavy rigid-body scenarios with selectable LCP
  solvers.
- Concepts/APIs: `dart::constraint::ConstraintSolver`, Dantzig and PGS LCP
  solvers, rigid contact dynamics, custom `dart::gui` panels, and
  example-specific command-line flags.
- Expected output: one selected benchmark scene rendered with the promoted DART
  GUI.
- Controls: Space starts or pauses simulation. The panel can pause, step, and
  shows the selected scenario, selected solver, time, and contact count.

## Scenarios

| Scenario         | Description                           |
| ---------------- | ------------------------------------- |
| `mass_ratio`     | Heavy box (1000kg) on light box (1kg) |
| `box_stack`      | Pyramid of stacked boxes              |
| `ball_drop`      | Many spheres dropping and settling    |
| `dominos`        | Domino chain impulse propagation      |
| `inclined_plane` | Block sliding on a ramp               |

## Run

From the source tree:

```bash
pixi run ex lcp_physics
```

List scenarios and solvers:

```bash
pixi run ex lcp_physics --list
```

Select a scenario and solver:

```bash
pixi run ex lcp_physics --scenario box_stack --solver pgs
```

Headless capture is provided by the promoted `dart::gui` runner:

```bash
pixi run ex lcp_physics --scenario dominos --headless --frames 2 --screenshot /tmp/lcp_physics.ppm
```

## References

- Guendelman et al. (2003), "Nonconvex Rigid Bodies with Stacking"
- SimBenchmark, physics-engine benchmark scenes
- Stewart and Trinkle (1996), implicit time-stepping with friction
