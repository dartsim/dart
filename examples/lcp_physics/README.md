# LCP Physics Example

Physics simulations demonstrating challenging LCP solver scenarios from research
literature.

## Scenarios

| Scenario         | Description                           | Challenge                                  |
| ---------------- | ------------------------------------- | ------------------------------------------ |
| `mass_ratio`     | Heavy box (1000kg) on light box (1kg) | Solver stability with 1000:1 mass ratio    |
| `box_stack`      | Pyramid of stacked boxes              | Shock propagation (Guendelman et al. 2003) |
| `ball_drop`      | 75 balls dropping                     | Many simultaneous contacts                 |
| `dominos`        | Domino chain reaction                 | Sequential impulse propagation             |
| `inclined_plane` | Block sliding on ramp                 | Friction model accuracy                    |

## Run

```bash
pixi run config
cmake --build build/default/cpp/Release --target lcp_physics
./build/default/cpp/Release/bin/lcp_physics
```

## Command-Line Options

```
--scenario <name>  Scenario to run (default: mass_ratio)
--headless         Run without display window
--frames <n>       Number of frames (default: 300)
--out <dir>        Output directory for frame capture
--width <n>        Viewport width (default: 1280)
--height <n>       Viewport height (default: 720)
--gui-scale <n>    GUI scale factor (default: 1.0)
--list             List available scenarios
```

## Headless Mode

```bash
./lcp_physics --list

./lcp_physics --headless --scenario box_stack --frames 500 --out ./output

./lcp_physics --headless --scenario mass_ratio --frames 300 --out ./mass_ratio_frames
```

Headless captures render the ImGui control panel; use `--gui-scale` to adjust
the UI size in the saved frames.

## References

- Guendelman et al. (2003) "Nonconvex Rigid Bodies with Stacking" - Stanford
- SimBenchmark (leggedrobotics) - Physics engine benchmark suite
- Stewart & Trinkle (1996) "Implicit time-stepping with friction"
