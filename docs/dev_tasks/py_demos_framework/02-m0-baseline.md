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

So there are **no Codex fix-tasks** from the baseline. M0 pivoted from "fix the
breakage" to "lock the guarantee in and deepen it"; that follow-up work is now
complete.

## M0 Follow-Up Status

1. **Locked into CI:** `pixi run -e cuda py-demos-smoke` and the bounded pytest
   guard keep the full catalog green as it grows.
2. **Tier-2 coverage:** `ScenePanel.build` is exercised with a faithful fake
   builder for the full catalog.
3. **Tier-3 coverage:** the real Filament viewer render smoke covers the full
   catalog with non-blank-frame detection.
4. **Scalable-contract lint:** the registry tests catch unregistered,
   duplicate, or ill-formed scene modules.

## Re-run

```bash
PYTHONPATH=build/cuda/cpp/Release-docking/python:python \
  .pixi/envs/cuda/bin/python scripts/py_demos_smoke.py --json-out /tmp/py_demos_smoke.json
```
