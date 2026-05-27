# Resume: IPC Deformable Solver

## Last Session Summary

The branch established a machine-checkable manifest for the audited upstream
`ipc-sim/IPC` scene corpus and added validation tooling for the 154 tracked
`.txt` scene paths. The manifest is a planning/data grounding artifact only; it
does not implement new solver, renderer, benchmark, or GUI behavior.

The follow-up mesh/material-state sub-slice adds optional surface and
tetrahedral topology, material validation, density-based lumped mass assembly
for tetrahedral bodies, boundary surface extraction, serialization, benchmark
counters, and GUI rendering from body-owned surface topology. It still steps
through the existing point-mass/spring path and must not be described as FEM,
mesh contact, or full IPC.

The scene/boundary/diagnostics sub-slice adds a contact-free upstream-style
scene text loader, a Gmsh 4.1 tetra-mesh subset importer, generated structural
spring replay edges, scripted DBC/NBC controls, binary restart continuity,
diagnostics JSON, replay/load benchmarks, and `experimental_deformable_gui
--deformable-scene` headless capture. It still ignores/reports contact and
friction directives and must not be described as IPC scene parity.

## Current Branch

`feature/ipc-scene-boundary-diagnostics` - stacked on
`feature/ipc-mesh-material-state`, adding contact-free scene/boundary/replay
scaffolding.

## Immediate Next Step

After this sub-slice lands, continue the rest of PLAN-081 Slice 1: broader
scene-option coverage, BE/NM state, output-file compatibility decisions, and
additional contact-free mesh scene replays with focused tests.

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
git checkout feature/ipc-scene-boundary-diagnostics
git status && git log -3 --oneline
cmake --build build/default/cpp/Release --target test_deformable_scene_io
./build/default/cpp/Release/bin/test_deformable_scene_io
```

Switch to `feature/ipc-paper-corpus-manifest` only when updating the scene
corpus manifest itself, or to `feature/ipc-mesh-material-state` when reviewing
the stacked mesh/material-state base.
