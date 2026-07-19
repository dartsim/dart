# Newton/Kamino CUDA reconstructed-fixture evidence

This run is **not an apples-to-apples paper reproduction**.
It uses public Newton 1.3.0 / Warp 1.15.0 on `NVIDIA RTX 5000 Ada Generation Laptop GPU` with DART-reconstructed fixtures.
It does not use the authors' scene assets, exact Kamino revision/configuration, or hardware.

Timing encloses one synchronized `SolverKamino.step` call.
It includes internal collision detection, constrained dynamics, PADMM, integration, GPU work, and Python launch overhead.
It excludes construction, kernel compilation/warmup, state export, and CSV I/O.

| Scenario | Mean ms | Median ms | P95 ms | Trajectory RTF | Steps <= dt | Contacts | Fixture check |
|---|---:|---:|---:|---:|---:|---:|---|
| backspin | 17.141 | 6.125 | 33.353 | 0.972 | 56.2% | 1-1 | pass |
| incline_mu_0_5 | 2.517 | 2.496 | 3.148 | 6.622 | 100.0% | 4-4 | pass |
| incline_mu_0_4 | 5.026 | 5.001 | 5.338 | 3.316 | 100.0% | 4-4 | pass |

The fixture check reuses the broad physical tolerances in DART's regression tests;
it is not a solver-residual certificate or a paper-parity verdict.

Files: `raw.csv` contains every measured step; `summary.csv` and `summary.json` contain aggregates.
`metadata.json` records the exact scene, solver, software, device, timing, and comparability contracts.
