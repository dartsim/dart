# Resume

Read README.md, AGENTS.md, and docs/ai/principles.md. Verify the current branch
and worktree before acting. This project removes native demo implementation
from dartpy while retaining C++ engine, IO, and GUI functionality.

Current branch: `refactor/python-demo-arm-posture`, a local child of
`refactor/python-demo-controllers` at `d13c20545`; that parent starts from main
`4ff47d438`. Fetched main `69526d6ae` adds AI workflow changes only. The separate marker follow-up remains on
`fix/demo-consolidation-followups`; do not fold it into this packet.

The previous packet completed heightmap generation and three controllers in
`python/examples/demos/models/`, removing their native code/bindings/tests and
transferring behavior checks. Numerical and assessed capture evidence, default
and GUI suites, GUI-off/vanilla-ImGui builds, catalog cycle, and two independent
reviews passed. See verification.md for exact counts, scope and limits.

The current packet ports the ssik arm catalog/construction/target helpers and
RelaxedPosture. Native parity, assessed matched captures and two independent
source reviews, final default/GUI/variant gates, cycle and lint pass. The branch
inherits optimizer gradient lifetime fix `0df4896ab` and has separate assessment
traversal fix `6a04cf3ec`. Nine `_demos` binding families remain after these two
ports. Workers are preparing the next boxed-LCP chooser and contact inverse
dynamics ports without editing the repository yet. Root owns shared
registration/docs/binding gaps; disjoint workers own each model and its callers
and tests. Verify each port against the native implementation before rebuilding
away that reference; transfer native tests and remove all callers/registrations.

WAM is the first pending maintainer decision. Do not treat “continue” as
acceptance of changed joint selection or authorization for GitHub mutations.
Continue the independent ports, validate against native behavior, run gates,
record two clean reviews, and update this handoff. No push or PR creation is
authorized. Preserve the task folder until the overall removal is complete.
