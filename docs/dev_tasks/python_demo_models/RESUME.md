# Resume

Read README.md, AGENTS.md, and docs/ai/principles.md. Verify the current branch
and worktree before acting. This project removes native demo implementation
from dartpy while retaining C++ engine, IO, and GUI functionality.

Current branch: `refactor/python-demo-soft-models`, from completed LCP/contact
`3e55e373c`, which inherits arm/posture `e18517de1`. The arm/posture branch is a local child of
`refactor/python-demo-controllers` at `d13c20545`; that parent starts from main
`4ff47d438`. Fetched main `69526d6ae` adds AI workflow changes only. The separate marker follow-up remains on
`fix/demo-consolidation-followups`; do not fold it into this packet.

The previous packet completed heightmap generation and three controllers in
`python/examples/demos/models/`, removing their native code/bindings/tests and
transferring behavior checks. Numerical and assessed capture evidence, default
and GUI suites, GUI-off/vanilla-ImGui builds, catalog cycle, and two independent
reviews passed. See verification.md for exact counts, scope and limits.

The second completed packet ports the ssik arm catalog/construction/target helpers and
RelaxedPosture. Native parity, assessed matched captures and two independent
source reviews, final default/GUI/variant gates, cycle and lint pass. The branch
inherits optimizer gradient lifetime fix `0df4896ab` and has separate assessment
traversal fix `6a04cf3ec`. The next completed packet ports boxed-LCP selection and contact inverse
dynamics replay. Its native parity, focused/default/GUI/variant gates, catalog
cycle and two reviews pass. Soft-worm and adaptive
soft-contact ports and ordinary soft-body bindings are implemented. Their native
helpers/bindings/tests and shared registrations are removed; final validation,
full/GUI/variant gates, cycle, and two final reviews pass. Five native families remain.
Root owns shared registration, docs and both WorldHistory fixture/caller
updates; the three workers own disjoint model and binding files. Use the
immutable mixed-reference extension for direct comparisons; rebuild only after
all source edits/deletions settle. Root owns shared
registration/docs/binding gaps; disjoint workers own each model and its callers
and tests. Verify each port against the native implementation before rebuilding
away that reference; transfer native tests and remove all callers/registrations.

WAM is the first pending maintainer decision. Do not treat “continue” as
acceptance of changed joint selection or authorization for GitHub mutations.
Continue the independent ports, validate against native behavior, run gates,
record two clean reviews, and update this handoff. No push or PR creation is
authorized. Preserve the task folder until the overall removal is complete.
