# M0 Baseline — Full-Catalog No-Crash Smoke

First systematic full-catalog run of `scripts/py_demos_smoke.py` under
`pixi -e cuda` (CUDA, `gpu=auto`), 3 frames per scene, subprocess-isolated.

## Result: 155 / 155 PASS

- **0** fail, **0** crash (signal), **0** timeout.
- Verified genuine (not a false-green):
  - Timing is realistic: 0.57s floor (subprocess + dartpy import) → 13.9s for
    heavy scenes (`plan083_terrain_vehicle`, IPC stack packets, puppets);
    median 0.74s; **0** scenes finished suspiciously fast.
  - Stability holds deeper: a representative sample (rigid_body,
    rigid_stack_stability, contact, avbd_demo3d_stack, rigid_ipc_pile,
    vbd_cloth, ipc_deformable_drape, diff_throw_to_target) all pass at
    **60 frames**, so it is not a 3-frame artifact.

## Interpretation

The catalog was not broken — it was **unguarded**. Before this, only the first
3 scenes were smoke-tested (`test_demos_cycle.py`). Individual scenes were
solid; what was missing was a systematic full-catalog no-crash guarantee. That
guarantee now exists (tier-1: build + headless step + render/debug providers).

So there are **no Codex fix-tasks** from the baseline. M0 pivots from "fix the
breakage" to "lock the guarantee in and deepen it."

## Remaining M0 work (revised)

1. **Lock the guard into CI** (exit criterion #5): a `pixi` task + a bounded
   pytest so the full catalog stays green as it grows. ← next action.
2. **Tier-2 coverage:** exercise `ScenePanel.build` with a fake builder
   (panels are a live-viewer crash source not covered by tier-1).
3. **Tier-3 coverage (later):** a full-catalog headless *render* pass through
   the real Filament viewer (`--cycle-scenes --headless`).
4. Scalable-contract lint (unregistered/duplicate scene guard) — likely already
   partly covered by `test_registry_has_scenes`; confirm and extend.

## Re-run

```bash
PYTHONPATH=build/cuda/cpp/Release-docking/python:python \
  .pixi/envs/cuda/bin/python scripts/py_demos_smoke.py --json-out /tmp/py_demos_smoke.json
```
