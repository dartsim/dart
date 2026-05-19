# Compute Resource Access Design

## Boundary

This task builds on the experimental compute graph. It should stay under
`dart::simulation::experimental::compute` and should not change classic
`dart::simulation::World`.

## Proposed Metadata

Use a small access-mode enum and value type:

- `Read`: a node reads a resource without mutating it.
- `Write`: a node overwrites a resource and does not depend on its old value.
- `ReadWrite`: a node reads and mutates a resource in place.
- `Reduce`: multiple nodes may contribute to a deterministic reduction.
- `Scratch`: node-local temporary memory with no inter-node dependency meaning.

The initial resource identifier can be a stable string. Good first examples are
component or stage resource names such as `comps::Transform`,
`comps::Velocity`, `comps::Force`, and `comps::FrameCache`.

## Dependency Model

Keep the first implementation hybrid:

- explicit graph edges remain the correctness source of truth;
- resource access metadata validates and visualizes data hazards;
- dependency inference is deferred until the validation model has tests and real
  graph examples.

This matters because not every dependency is a data hazard. Solver iteration
order, deterministic reductions, frame hierarchy order, and external side
effects can require explicit edges even when the accessed resources look
compatible.

## Validation Rules

The first validator should be conservative:

- read/read sharing is allowed;
- writes to disjoint resources are allowed;
- write/read, read/write, write/write, and read-write combinations on the same
  resource need either an explicit dependency or a declared reduction;
- scratch resources do not create inter-node hazards;
- reduction resources are allowed only when the reduction semantics are
  explicitly declared by the node metadata.

## Visualization And Profiling

DOT output should be able to show resource accesses without adding a GUI
dependency. Keep it readable by making resource output optional through DOT
options if labels become too large.

Profiles should remain timing-oriented, but resource metadata should make it
possible to correlate hot nodes with hot resources or contention points.
