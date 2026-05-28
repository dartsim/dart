# Resume: IPC Deformable Solver

## Last Session Summary

The branch established a machine-checkable manifest for the audited upstream
`ipc-sim/IPC` scene corpus and added validation tooling for the 154 tracked
`.txt` scene paths. The manifest is a planning/data grounding artifact only; it
does not implement new solver, renderer, benchmark, or GUI behavior.

## Current Branch

`feature/ipc-paper-corpus-manifest` - manifest, script, and docs changes for
the scene-corpus grounding PR.

## Immediate Next Step

After this manifest PR lands, begin Slice 1 from PLAN-081: mesh/material state,
scene loading, boundary conditions, restart/output diagnostics, and
contact-free mesh stepping with focused tests.

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

```bash
git checkout feature/ipc-paper-corpus-manifest
git status && git log -3 --oneline
pixi run python scripts/check_ipc_scene_manifest.py
```

Then continue with PLAN-081 Slice 1 and update this file when the active
implementation branch changes.
