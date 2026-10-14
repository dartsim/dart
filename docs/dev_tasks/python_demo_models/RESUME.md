# Resume

Read README.md, AGENTS.md, and docs/ai/principles.md. Verify the current branch
and worktree before acting. This project removes native demo implementation
from dartpy while retaining C++ engine, IO, and GUI functionality.

Current branch: `refactor/python-demo-state-mesh`, from completed soft-model
`5a3a36635`, with isolated conversion follow-up `3c9ba6223`. It inherits
LCP/contact `3e55e373c` and arm/posture `e18517de1`.
The arm/posture branch is a local child of
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
full/GUI/variant gates, cycle, and two final reviews pass.
The current packet ports WorldHistory and PointCloudSensor. Both native helpers
are deleted after direct comparisons. Root owns snapshot/point-acceleration bindings,
core snapshot tests, shared registration/docs and build coordination. Disjoint
workers own each model/callers/tests and the mesh/RNG/voxel bindings. Native
baseline probes are recorded for history and seeded sampling; preserve immutable
reference extensions. The owning snapshot and point-cloud library source is
ready and the final post-removal focused suite passes 643 tests. Body and
skeleton restores now reject mismatched soft-point counts before any mutation;
the nested skeleton bypass has failing archived tests and passing rebuilt tests.
Two independent final reviews, full/GUI/variant gates, cycle and lint pass.
Three native families remain. Both next controllers' native baselines are
ready, including the complete nonmonotonic recovery sweep. Commit this packet,
then port SIMBICON and soft-foot SIMBICON together on a local child branch;
preserve existing files and immutable references.

WAM is the first pending maintainer decision. Do not treat “continue” as
acceptance of changed joint selection or authorization for GitHub mutations.
Continue the independent ports, validate against native behavior, run gates,
record two clean reviews, and update this handoff. No push or PR creation is
authorized. Preserve the task folder until the overall removal is complete.
