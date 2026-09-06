# Resume: DART 7 M1 Oracle Manifest

## Current Snapshot

Prepared, unclaimed; numerical derivation and acceptance have not started.
Read [README.md](README.md), the linked WP-040.1 packet and
[the dashboard](../../plans/dashboard.md#plan-040-dart-7-readiness-milestones).
The dashboard owns the next action. No numerical tolerance, milestone or
downstream prerequisite was accepted by creating this folder.

## Starting Revision And Ownership

This handoff was created on `docs/retire-task-handoffs-m1` in `task_9`, from
`main` revision `53c9495a95ff80400695775549b719a124c74596`. Those are creation
facts, not commands to resume that branch. Verify the live branch, HEAD, local
changes and landed cleanup before claiming WP-040.1 from current `main`.

No execution owner is assigned. Use Astra Max with one sequential writer and
record effective model/effort and the owner when claimed. No parallel
implementation scope is approved. The maintainer authorized this handoff
preparation, not oracle acceptance or GitHub mutations; carry forward only
explicit action/target/scope authorization.

## Context To Preserve

- Independent RB-01–RB-07 oracles; Float64 first and full CPU/CUDA M1 parity.
- PLAN-040's fixture defaults and error budgets remain proposals until derived
  and independently accepted by WP-040.1.
- Checkpoints require all four CPU/CUDA restart directions, with distinct
  deterministic replay and finite-horizon cross-device tolerance contracts.
- Numerical specification is this packet's work; compute/physics/checkpoint
  implementation and checker migration belong to successor packets.

## Verify Reality

```bash
git status --short --branch
git rev-parse HEAD
git log -3 --oneline
pixi run check-docs-policy
```

Then read the packet's source/test and architecture owners before deriving
oracles. Use a new selected-task branch after the cleanup lands; if working from
an unpublished cleanup checkout, preserve its edits and ownership. Record
actual verification results and limitations instead of inheriting old passes.
