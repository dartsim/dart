# MuJoCo reconstructed-fixture evidence

This lane provides a reproducible standalone-MuJoCo comparison for the seven
small FBF fixtures whose local reconstruction is sufficiently specified:
backspin, the two published incline cells, and the four published turntable
cells. It is benchmark/evidence code only and is not a DART dependency.

> **Committed-bundle status:** the `2026-07-11_mark26_cpu4` bundle is a
> historical pre-hardening snapshot. Its recorded provenance does not bind to
> the current source runner, so it is not current validated evidence. Do not
> regenerate it in place; a new validation run must use a new output directory.

## Reproduce a fresh bundle

Run the complete matrix sequentially from the optional environment. Pinning to
one otherwise-idle performance core is recommended; choose the CPU for the
host rather than copying this example blindly.

```bash
pixi run -e fbf-baselines fbf-mujoco-evidence -- \
  --output-dir docs/dev_tasks/fbf_exact_coulomb_friction/assets/mujoco_cpu_evidence/<run> \
  --repetitions 10 \
  --cpu-list <cpu>
```

Without `--force`, the runner refuses to overwrite any of its five owned
artifacts (`metadata.json`, `raw.csv`, `summary.csv`, `summary.json`, and
`REPORT.md`). With `--force`, it invalidates that entire owned set before
simulation while preserving unrelated files. It writes the four data/report
siblings first and `metadata.json` last, after recording their hashes, so an
interrupted replacement cannot leave stale metadata authenticating a mixed
bundle.

The runner limits common CPU math thread environments to one thread, applies
the requested process affinity, runs every complete trajectory without a
warmup exclusion, and times each synchronous `mujoco.mj_step` call. Model
compilation, state extraction, and the reconstructed turntable control
assignment are outside the timed interval and are recorded as such. Use
`--force` only when intentionally replacing the runner-owned artifact set.

Each bundle contains:

- `raw.csv`: the initial state and every completed simulation step, including
  wall time, full tracked-body pose and linear/angular velocity, contacts,
  solver iterations, and incline/turntable-specific trajectory metrics;
- `summary.csv` and `summary.json`: per-scenario timing and physical outcome
  aggregates across complete repetitions;
- `metadata.json`: the repository state, host/load/affinity/frequency policy,
  dependency versions, effective thread environment, exact compiled MuJoCo
  options and contact defaults, scene parameters and unknowns, validation
  result, source hashes, and artifact hashes;
- `REPORT.md`: the concise run contract, aggregate table, and explicit
  non-comparability verdict.

## Paper contract and limits

Appendix B publishes that MuJoCo uses its Newton solver, an elliptic cone, and
500 maximum iterations at its native tolerance. The XML fixes the first three
settings. It deliberately does not invent a numeric tolerance: the paper does
not give one, so the active version-specific value is captured from the
compiled model. The pinned current author runner instead sets an explicit
`1e-6` MuJoCo tolerance, which is now an auditable current-source difference
rather than a missing parameter. The paper also reports sequential,
single-precision CPU runs on one Apple-silicon Mac.

The local evidence is **not paper-parity or apples-to-apples evidence**:

- official MuJoCo pypi uses float64 here, on a different CPU;
- the pinned public author reference now exposes the current Newton/Warp
  adapter, six scene/configuration sources, pinned dependency versions, and
  per-step timer code, but this historical local bundle has not been rerun or
  validated against that source;
- the paper and public repository do not attest that the current reference's
  MuJoCo tolerance/contact settings, Apple host, timing boundary, or
  warmup/aggregation behavior exactly reproduce the historical table run;
- the small-scene initial penetrations are local DART reconstruction choices;
- the public turntable geometry, mass, initial radius, smoothstep ramp, and run
  duration are now inspectable, but the local reconstructed scene does not yet
  match them.

The paper's Table 5 numbers remain in the aggregate files only as transcription
anchors. The runner intentionally computes no local-versus-paper speed ratio.
Any local real-time statement also requires both a controlled-affinity record
and a validated physical outcome; raw wall time alone is not sufficient.

## Deliberate omissions

Painleve and card-house MuJoCo scenes are not synthesized by this local runner,
even though the pinned public reference now provides current scene and solver
sources for both. Adding them requires a source-pinned port and matched-run
validation, not new parameter invention. `masonry_arch_101_rigid_ipc` remains
an explicit opt-in mode of `fbf_paper_mujoco_baseline.py`; it uses adapted
Rigid-IPC geometry, not the pinned author scene or historical paper timing
contract, and is excluded from the evidence wrapper's small-scene matrix and
aggregates.
