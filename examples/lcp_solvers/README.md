# LCP Solvers (ImGui)

This example provides an ImGui-based dashboard for exploring DART's LCP solvers.
It presents toy through real-world inspired LCP scenarios, exposes shared solver
options, and reports solver metrics with a contract check based on
`dart::math::LcpValidation`.

## What It Shows

- Scenario selector spanning toy, intermediate, and real-world inspired cases
- Solver selector grouped by pivoting, projection, Newton, and other methods
- Shared `LcpOptions` applied across solvers for fair comparisons
- Contract metrics: residual, complementarity, bound violation, contract_ok
- Per-solver runtime summaries with history plots
- A minimal ground plane and axes for visual reference

## Scenario Catalog

Toy / Didactic:

- 2x2 Standard LCP (Refs: Lemke1965, CottleDantzig1968)
- 3x3 Boxed LCP with active bounds (Refs: Murty1988, CottlePangStone1992)
- Single-contact friction pyramid with findex (Refs: Smith2000, Smith2006, StewartTrinkle1996)

Intermediate / Stress:

- Near-singular standard LCP (Refs: Kojima1991, Wright1997)
- Large mass-ratio block (Refs: Silcowitz2009, Silcowitz2010)

Real-world inspired:

- Rigid-body time-stepping contact (Refs: StewartTrinkle1996, AnitescuPotra1997)
- Granular pile / multi-contact blocks (Refs: Moreau1988, Jean1999, AcaryBrogliato2008)
- Stacking / shock propagation layers (Refs: Guendelman2003)
- Staggered normal/tangent solve (Refs: Kaufman2008, Tournier2015)
- Newton accuracy-focused case (Refs: Fischer1992, QiSun1993)

All problems are generated once per scenario with fixed seeds to keep
comparisons consistent.

## Run

```bash
# From the repo root
pixi run config
cmake --build build/default/cpp/Release --target lcp_solvers
build/default/cpp/Release/bin/lcp_solvers
```

Optional:

```bash
build/default/cpp/Release/bin/lcp_solvers --gui-scale 1.2
```

The window size scales with `--gui-scale` to keep the widgets readable.

## Command-Line Options

```
--gui-scale <n>   Scale factor for ImGui widgets (default: 1.0)
--headless        Run benchmark with headless 3D rendering
--list            List available examples and solvers
--example <name>  Filter examples by name substring
--solver <name>   Filter solvers by name substring
--runs <n>        Runs per solver (default: 100 for long horizon testing)
--frames <n>      Number of frames to render (default: 100)
--out <dir>       Output directory for captured frame images
--width <n>       Viewport width (default: 1280)
--height <n>      Viewport height (default: 720)
```

## Headless Mode

Run solvers against scenarios with headless 3D rendering and frame capture:

```bash
# List available examples and solvers
./lcp_solvers --list

# Run with frame capture for visual verification
./lcp_solvers --headless --frames 100 --out ./output

# Run specific example with specific solver
./lcp_solvers --headless --example "friction" --solver "Dantzig" --out ./output

# Full benchmark: all examples, 100 runs each, 500 frames
./lcp_solvers --headless --runs 100 --frames 500 --out ./output

# Quick smoke test (10 runs, 20 frames)
./lcp_solvers --headless --runs 10 --frames 20
```

### Frame Capture

When `--out <dir>` is specified, PNG frames are saved as `frame_000000.png`,
`frame_000001.png`, etc. The 3D scene shows:

- Ground plane with coordinate axes (RGB = XYZ)
- Static reference scene for visual verification

Use ffmpeg to create a video from captured frames:

```bash
ffmpeg -framerate 30 -i output/frame_%06d.png -c:v libx264 -pix_fmt yuv420p output.mp4
```

### Output Format

Each solver-example combination reports:

- **Pass/Total**: Number of runs that passed contract check
- **Status**: PASS if all runs passed, FAIL otherwise

### Exit Codes

- `0`: All solver-example combinations passed all runs
- `1`: At least one combination had failures (for CI integration)

### Long Horizon Testing

The default 100 runs per solver ensures consistent behavior over time. Some
iterative solvers (Jacobi, Red-Black GS) may fail on ill-conditioned problems
like "Near-singular standard LCP" - this is expected and documented.

## Reference Key

- Lemke1965: Lemke (1965) Bimatrix equilibrium points and mathematical programming
- CottleDantzig1968: Cottle and Dantzig (1968) Complementary pivot theory
- Murty1988: Murty (1988) Linear complementarity, linear and nonlinear programming
- CottlePangStone1992: Cottle, Pang, Stone (1992) The linear complementarity problem
- Smith2000: Smith (2000) Open Dynamics Engine user guide
- Smith2006: Smith (2006) Open Dynamics Engine
- StewartTrinkle1996: Stewart and Trinkle (1996) Implicit time-stepping with friction
- Kojima1991: Kojima et al. (1991) Interior point algorithms for LCP
- Wright1997: Wright (1997) Primal-dual interior-point methods
- Silcowitz2009: Silcowitz et al. (2009) NNCG for interactive contact
- Silcowitz2010: Silcowitz et al. (2010) NNCG for interactive contact (extended)
- AnitescuPotra1997: Anitescu and Potra (1997) Multi-body contact LCP
- Moreau1988: Moreau (1988) Unilateral contact and dry friction
- Jean1999: Jean (1999) Non-smooth contact dynamics
- AcaryBrogliato2008: Acary and Brogliato (2008) Numerical methods for nonsmooth dynamics
- Guendelman2003: Guendelman et al. (2003) Nonconvex rigid bodies with stacking
- Kaufman2008: Kaufman et al. (2008) Staggered projections for frictional contact
- Tournier2015: Tournier et al. (2015) Stable constrained dynamics
- Fischer1992: Fischer (1992) Special Newton-type optimization method
- QiSun1993: Qi and Sun (1993) Nonsmooth Newton method
