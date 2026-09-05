# Resume: IPC Deformable Solver

## Current Reality (2026-07-04)

Live status is this folder's `README.md`, the durable PLAN-081 owner docs
(`docs/plans/081-deformable-implicit-barrier-solver/ipc-parity-roadmap.md`,
`ipc-paper-figure-showcase.md`, `ipc-paper-gap-audit.md`,
`ipc_scene_corpus_manifest.json`), `docs/plans/dashboard.md`, and the current
code. Every earlier feature branch, including PR #2821 (matrix-free CG + CG
diagnostics, merged as `74338577982`), has landed; there is no open IPC
feature branch to resume.

Completion-audit result (this session): the dev task is the active working
surface for **PLAN-081 (Status: Active, Horizon: Now)** and is **not
retire-able unilaterally** — the dashboard records that dev-task retirement for
incomplete IPC-family plans needs maintainer direction, and PLAN-081 is far from
parity (figure showcase: 24 planned / 3 in-progress / 6 landed /
3 reference-beaten; M7 scale/perf/GPU, the asset pipeline, and codim obstacles
remain). Continue in bounded, verifiable M7 slices.

Current deformable work keeps entering through the DART-owned deformable solver
families, shared Newton-barrier/VBD components, the built-in World schedule, and
facade-safe `World`/`DeformableBodyOptions`/diagnostics surfaces. Route any new
shared distance/barrier/CCD/Newton primitive through PLAN-083
(`docs/plans/083-unified-newton-barrier-multibody.md`) before duplicating it here.

### Landed Slice: Fig-23 peak-contacts diagnostic — MERGED (PR #3257)

`DeformableSolverDiagnostics.maxActiveContactCount` (dartpy
`max_active_contact_count`) + internal `DeformableSolverStats.maxActiveContactCount`
— the IPC Fig. 23 "max contacts per step" axis — merged to `main` as
`1819b801228`. Captured as the delta of the existing
`selfContactBarrierActiveContacts` counter around the single outer-iteration
objective evaluation that passes it, so the solve is byte-identical (no hot-path
signature change). Surfaced on `BM_DeformableSelfContactBarrierStage` +
`ipc_deformable_cg_contact` demo; C++ peak-invariant + public-propagation
regressions. Changelog: no entry (family-level bullet covers it).

### Landed Slice: Fig-23 statistics packet — MERGED (PR #3264, `dbe6fcccb1c`)

Branch `feature/ipc-deformable-fig23-statistics-packet`, merged as **PR #3264**
against `main` (off `main` after #3257 merged, then merged current `origin/main`
incl. #3250). Adds a machine-checkable **Fig-23-shaped statistics packet** that
distils the `bm_deformable_body` JSON into per-scene Fig-23 axes (per-step
Newton/CG effort, CG residual, assembled sparse-Hessian footprint, per-step wall
time, and the active-contact statistics — consuming the #3257
`max_active_contacts` axis) over DART-runnable scenes. **Pure-additive Python**,
zero C++/behavior change: `scripts/write_plan081_deformable_fig23_packet.py` +
`docs/plans/081-deformable-implicit-barrier-solver/fig23_deformable_statistics_corpus.json`
(manifest, `paper_scale: false`) + `python/tests/unit/test_write_plan081_deformable_fig23_packet.py`
(10 pytest cases; placed under `python/tests/unit/` so the CMake `pytest` target
gates it — root `tests/` is NOT gated by test-all/CI), reusing the
plan091/`benchmark_packet_utils` pattern. Each row's invariant is machine-enforced
via per-row `metric_fields` (finite, non-negative), `positive_fields` (> 0), and
`zero_fields` (== 0): the matrix-free row proves zero sparse-Hessian footprint and
the self-contact row proves a strictly positive min active-contact distance. The
packet output goes to gitignored `.benchmark_results/plan081/` (not committed).
Verified end-to-end against a real `bm_deformable_body` run (7 rows; the
direct-vs-CG crossover and matrix-free zero-footprint are visible); `test-all` 6/6
green. Scoped honestly as shape parity, not paper parity: paper-scale scenes + the
Table-1 CPU reference comparison remain blocked on the M4 asset pipeline.
Changelog: **no entry** (internal evidence tooling, no user-facing API/behavior
change).

## Context That Would Be Lost

- The upstream IPC audit is pinned to commit
  `573d2c7e04104d3f9baf526bdaee7745891a571a`.
- Use `git ls-tree` to enumerate scene paths; `find -type f` misses the 10 SQP
  benchmark symlink aliases.
- The durable row-level source of truth is
  `docs/plans/081-deformable-implicit-barrier-solver/ipc_scene_corpus_manifest.json`.
- The validator must keep zero unclassified family or target-type rows before
  any future IPC parity claim.

## How To Resume

Current checkpoint (2026-07-04) — maintainer-directed next slice:

```bash
git fetch origin main
git checkout main
git merge --ff-only origin/main
git status && git log -3 --oneline
sed -n '270,295p' docs/plans/081-deformable-implicit-barrier-solver/ipc-parity-roadmap.md
```

Do not resume a merged feature branch. Open a new branch only after maintainer
direction chooses one of the currently blocked or policy-gated M7 follow-ups:
matrix-free-CG auto-selection, avg-contacts-per-step fixture design,
process-memory column semantics, M4 asset pipeline work, AMG/multigrid
preconditioning, or on-device GPU assembly + solve.
