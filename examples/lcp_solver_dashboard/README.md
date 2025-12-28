# LCP Solver Dashboard (ImGui)

This example provides an ImGui-based dashboard for exploring DART's LCP solvers.
It will present a set of toy to real-world inspired LCP scenarios, expose common
solver options, and report solver metrics and contract checks.

## Run

```bash
# From the repo root
pixi run config
cmake --build build/default/cpp/Release --target lcp_solver_dashboard
build/default/cpp/Release/bin/lcp_solver_dashboard
```

Optional:

```bash
build/default/cpp/Release/bin/lcp_solver_dashboard --gui-scale 1.2
```
