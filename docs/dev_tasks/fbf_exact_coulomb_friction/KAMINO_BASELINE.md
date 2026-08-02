# Newton/Kamino CUDA comparison lane

This lane runs the public `newton.solvers.SolverKamino` implementation on the
same small, procedural fixtures reconstructed for DART's FBF paper tests. It is
benchmark/example-only and remains isolated in Pixi's optional
`fbf-baselines` environment.

> **Local-bundle status:** the
> `20260711_rtx5000ada_newton1.3.0` bundle is a historical pre-hardening
> snapshot. Its recorded provenance does not bind to the current source runner,
> so it is not current validated evidence. Do not regenerate it in place; a
> new validation run must use a new output directory.

Run the full backspin and incline trajectories with:

```bash
pixi run -e fbf-baselines fbf-kamino-evidence -- \
  --output-dir docs/dev_tasks/fbf_exact_coulomb_friction/assets/kamino_cuda_evidence/<run>
```

For a short integration smoke that does not issue a physical fixture verdict:

```bash
pixi run -e fbf-baselines fbf-kamino-evidence -- \
  --output-dir /tmp/fbf-kamino-smoke --steps 3
```

The runner creates:

- `raw.csv`: synchronized wall time and physical state for every step;
- `metadata.json`: package versions, GPU/driver, precision, solver settings,
  scene parameters, timing boundary, Git revision, and dirty state;
- `summary.csv` and `summary.json`: timing aggregates, real-time fractions,
  contact counts, and physical fixture checks;
- `REPORT.md`: a concise human-readable result.

## Comparison contract

This is not an apples-to-apples reproduction of the paper's Kamino rows. The
pinned public reference now supplies all six current scene/configuration
sources, pins `newton==1.0.0`, exposes the current Kamino PADMM setup, and
records its per-step timer boundary. It does not attest that those current
sources exactly reproduce the historical paper run, and it does not identify
the matched historical hardware or warmup/aggregation protocol. The local
runner predates that source release and instead uses:

- DART's procedural backspin and incline reconstructions from
  `tests/benchmark/integration/fbf_paper_trace.cpp`;
- current public Newton/Warp packages in the optional Pixi environment;
- Newton's internal unified collision detector with an explicit broad phase;
- float32 simulation arrays, `1e-6` PADMM tolerances, a 200-iteration cap,
  container warm starts, and one CUDA world;
- one synchronized `SolverKamino.step` call as the timing boundary.

The timing includes internal collision detection, constrained dynamics, PADMM,
integration, GPU work, and Python launch overhead. Model construction, lazy
kernel warmup, state/contact export, and file I/O are excluded. Full-trajectory
fixture checks reuse the broad physical tolerances in DART's regressions; they
are neither a solver residual certificate nor a paper-parity verdict.

## Historical pre-hardening local run (2026-07-11)

The raw run is preserved under
`assets/kamino_cuda_evidence/20260711_rtx5000ada_newton1.3.0/`.
It used Newton 1.3.0, Warp 1.15.0, CUDA toolkit 12.9, NVIDIA driver
595.71.05, and an RTX 5000 Ada Generation Laptop GPU. All three complete
trajectories passed their DART physical fixture checks:

| Scenario | Steps | Mean / median / p95 | Trajectory RTF | Terminal physical result |
|---|---:|---:|---:|---|
| backspin | 240 | 17.141 / 6.125 / 33.353 ms | 0.972 | `vx=-11.428530 m/s`, `wy=-45.714119 rad/s`, zero slip, one contact |
| incline `mu=0.5` | 120 | 2.517 / 2.496 / 3.148 ms | 6.622 | `0.002494 m` downhill displacement, four contacts |
| incline `mu=0.4` | 120 | 5.026 / 5.001 / 5.338 ms | 3.316 | `1.766686 m` versus `1.754866 m` analytic displacement, four contacts |

In that historical run, both incline trajectories completed every step inside
the `1/60 s` budget. Backspin did not: 56.2% of its steps met the budget and its
full-trajectory RTF was 0.972. These are stale reconstructed-fixture
observations only, not current validated real-time evidence, and they do not
replace or directly validate the paper's reported Kamino factors.
