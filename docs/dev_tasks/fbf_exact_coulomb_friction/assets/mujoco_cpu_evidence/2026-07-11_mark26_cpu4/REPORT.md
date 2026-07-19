# MuJoCo reconstructed-fixture CPU evidence

**Verdict: this bundle is not paper-parity or apples-to-apples evidence.**
It is a reproducible same-machine run of the local standalone MuJoCo
reconstructions, with the paper's published Newton/elliptic/500-iteration
settings and explicit labels for everything that remains different or unknown.

## Run contract

- MuJoCo: 3.10.0 (float64)
- CPU: 13th Gen Intel(R) Core(TM) i9-13950HX
- Process affinity: [4]
- Repetitions: 10 complete trajectories, sequential, no warmup exclusion
- Timed interval: synchronous `mujoco.mj_step` only; model compilation, state extraction, and control assignment excluded
- Solver: Newton; cone: elliptic; maximum iterations: 500; tolerance: local native default recorded per model

## Aggregates

| Scenario | Mean ms/step | Median | p95 | Contacts/step | Outcome |
|---|---:|---:|---:|---:|---|
| `backspin` | 0.001369 | 0.001302 | 0.001430 | 0.008 | ejected_above_plane |
| `incline_mu_0_5` | 0.005279 | 0.004343 | 0.007559 | 2.933 | slid_with_contact_loss |
| `incline_mu_0_4` | 0.002632 | 0.001446 | 0.005404 | 1.050 | slid_with_contact_loss |
| `turntable_mu_0_2_omega_2` | 0.004413 | 0.001711 | 0.011384 | 1.067 | ejected_or_off_support |
| `turntable_mu_0_2_omega_5` | 0.002759 | 0.001633 | 0.008358 | 0.358 | ejected_or_off_support |
| `turntable_mu_0_5_omega_2` | 0.009512 | 0.010353 | 0.011751 | 4.000 | retained_on_support |
| `turntable_mu_0_5_omega_5` | 0.002958 | 0.001607 | 0.010259 | 0.429 | ejected_or_off_support |

The paper reference columns are retained in `summary.csv` only as transcription
anchors. No local/paper speed ratio is computed because the precision, CPU,
backend, assets, and several scene/contact parameters do not match.

## Non-comparability reasons

- Local official MuJoCo pypi uses float64; the paper reports single-precision CPU runs.
- Local CPU is not the paper's Apple-silicon Mac.
- The author code and scene assets are unavailable, so adapter/backend equivalence cannot be established; this runner uses standalone MuJoCo pypi.
- The paper does not publish the numeric native tolerance, MuJoCo contact solref/solimp, or all integrator settings.
- Initial penetrations are DART reconstruction choices, not published paper parameters.
- Turntable dimensions, mass, radius, exact ramp, drive implementation, and duration are unpublished and reconstructed.
- Only mj_step is timed; the exact timing boundary used by the paper implementation is unavailable.

## Files

- `raw.csv`: every initial state and completed step, including wall time, full tracked-body pose/velocity, contacts, and scenario metrics.
- `summary.csv` / `summary.json`: trajectory timing and physical aggregates.
- `metadata.json`: host, affinity, software, exact model options, scene contracts, validation, provenance, and SHA-256 checksums.
