# Deformable-body decisions

- **2026-10-10 release closeout:** Retarget the unmet 6.20 full-parity goal to
  6.21. This changes the release target, not the per-row evidence requirement.
- **2026-07-29 scope:** Keep one DART 6 deformable model, Jain/Liu surface
  flesh on `SoftBodyNode`; volumetric FEM is out of scope. Removed research
  remains in [PR #3404](https://github.com/dartsim/dart/pull/3404).
- **2026-07-23 acceptance:** The earlier Jain/Liu deferrals were retracted.
  Require per-row correctness and CPU-performance evidence on DART's in-tree
  configurations plus normalized paper metrics, with zero rigid-body runtime
  overhead. PLAN-622 owns remaining rows and permitted dispositions.
- **Implementation:** Preserve `SoftBodyNode` as the public API. Activation
  stays opt-in; public matrices exclude retained point-mass acceleration.
  Retained phase mirrors were rejected on measurements; contiguous object
  storage requires an ownership/lifetime redesign.
- **Soft-foot comparison (#3423):** Match rigid-control mass, rest inertia,
  and tessellation; use the approved asset damping and point-mass-aware COM
  sensor. Single-trajectory push thresholds are not ensemble parity evidence.

The [design owner](../../design/dart6_deformable_body.md) holds the full
rationale, activation semantics, per-step seam, and pre-default detector gates.
Revisit these choices only with new evidence and an explicit compatibility
review; release retargeting does not authorize a default or ABI change.
