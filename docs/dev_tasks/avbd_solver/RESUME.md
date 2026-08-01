# Resume: AVBD Solver

Working handoff for PLAN-104's full VBD/AVBD paper-parity implementation. The
durable roadmap and acceptance bar live in
[`../../plans/104-vertex-block-descent-solver.md`](../../plans/104-vertex-block-descent-solver.md);
the fail-closed completion contract is
[`../../plans/104-vertex-block-descent-solver/paper-parity-matrix.md`](../../plans/104-vertex-block-descent-solver/paper-parity-matrix.md).
This file owns only the exact branch/session stop point and takeover sequence.

## Stop Point

At the branch-tip closeout commit containing this handoff, the public
Sequential Impulse (SI) pair-row slice, its warm-start fix, the subsequent
review fixes, and a source-bound six-capture evidence reseal are closed. One
strict quiet-host current-source benchmark is accepted, the AVBD -> VBD -> SI
packets are regenerated from it, all 192 focused tests pass, and the uncached
default and CUDA `test-all` gates pass for the unchanged runtime/capture source
tree. Independent code and architecture/docs/claims reviews approve the final
closeout tree with no actionable finding. If that commit is not yet the local
branch tip, preserve the dirty scope and finish only its final lint and local
commit before continuing.

The 88-row VBD and 88-row AVBD contracts remain fail-closed with no complete
rows. The local three-method Figure 13 packet is useful partial evidence, not
paper parity. Full scope is still all paper/reference algorithms and features,
CPU and GPU implementations, every paper/site/video/source-demo experiment,
and performance that beats the comparable reference and published paper
numbers. Do not reduce this to an MVP, a CPU-only lane, or one wall demo.

## Exact Worktree And Branch

- Worktree: `/home/js/dev/dartsim/dart/task_5`
- Branch: `feature/vbd-avbd-paper-parity-contract`
- Pre-closeout local `HEAD`: `d9af744a9cb0763033d76e01381dbf7c1e6ecaa9`
- Resume tip: the local commit containing this handoff; verify its live SHA
- Last-fetched upstream ref:
  `origin/feature/vbd-avbd-paper-parity-contract` at the same SHA (`+0/-0`)
- Current remote-tracking `origin/main` snapshot:
  `b15e7a652b766897a8a69a121742ef5973c77f3c` (nine commits beyond local
  `main`)

No fetch, push, PR edit, review trigger, or other GitHub mutation was performed
while preparing this handoff. Treat the upstream SHA as a local snapshot, not a
fresh remote assertion. Verify live state before any external action, and do
not push or mutate GitHub without explicit maintainer/user approval.

Immediately before the closeout commit, the working tree intentionally had 57
tracked modifications and six untracked files. The commit containing this
handoff should leave that scope clean. If resuming before it exists, preserve
the dirty tree exactly; do not reset, stash, rebase, switch worktrees, or
discard files. The six pre-closeout untracked files are:

- `dart/simulation/detail/rigid_pair_constraint.hpp`
- `docs/plans/104-vertex-block-descent-solver/avbd-paper-sequential-impulse-comparison-packet.json`
- `python/examples/demos/scenes/sequential_impulse_paper_breakable_wall.py`
- `python/tests/unit/test_write_avbd_paper_sequential_impulse_comparison_packet.py`
- `scripts/capture_source_provenance.py`
- `scripts/write_avbd_paper_sequential_impulse_comparison_packet.py`

The local commit ladder above `main` is:

| Commit        | Local slice                                       |
| ------------- | ------------------------------------------------- |
| `710cbfc1152` | Fail-closed VBD/AVBD paper-parity contracts       |
| `a78f688a178` | AVBD Section 3.5 quasi-Newton geometric stiffness |
| `0b0154573b8` | AVBD Section 4 parallel dual/stiffness update     |
| `761263bbd41` | Articulated finite masked rows                    |
| `9ebd9b895b1` | Articulated finite movable-pair motors            |
| `131981788fa` | Articulated finite load and fracture accounting   |
| `646b447d6cc` | Public AVBD solver and Figure 13 evidence         |
| `d9af744a9cb` | Public fixed-penalty VBD comparison               |
| branch tip    | Public SI pair rows and Figure 13 comparison      |

## Current SI Slice

The dirty slice makes the default public SI family own its hard rigid pair
constraints instead of borrowing the private AVBD projector:

- fixed, spherical, revolute, and prismatic masks interleave with contact rows
  in the configured projected Gauss-Seidel sweeps;
- bounded one-DOF velocity motors use the same SI row ownership;
- break force is derived from accumulated impulse over `dt`, broken rows stay
  excluded, and a non-velocity post-stabilization pass reduces pose drift;
- finite-stiffness public pair rows reject SI and direct callers to VBD/AVBD;
- shared rigid-pair geometry and input extraction have solver-neutral
  ownership, while SI and AVBD retain separate row/continuation state; and
- the matched `sequential_impulse_paper_breakable_wall` demo, writer, packet,
  benchmark row, source-provenance utility, and tests complete the third honest
  Figure 13 method row.

The warm-start ownership rule is deliberately narrower than a full AVBD scratch
clear. Crossing through SI invalidates AVBD contact and hard point-joint/motor
continuation state, but preserves independently active AVBD distance-spring
continuation.

### Post-review correctness fix

The first correctness review found a real P2 defect: when public SI declined a
private `RigidAvbdContactConfig` compatibility envelope, the fallback called
the full AVBD scratch `clear()`. That erased `distanceSpringInventory` and
cold-started unrelated distance springs every step.

The working tree fixes the cause in
`dart/simulation/compute/rigid_body_contact_stage.cpp` by calling
`clearSequentialImpulseOwnedWarmStart()`. That clears contact and hard
point-joint/motor continuation only. The regression
`World.SequentialImpulseFallbackPreservesAvbdDistanceSpringWarmStart` in
`tests/unit/simulation/world/test_world.cpp` compares fallback and control
worlds with a stiffness-ramping distance spring; it failed at the second-step
transform/velocity before the fix and passes after it.

The first architecture/claims review separately rejected the old statement
that every timing CV was below 0.7%. A later source fix invalidated that
otherwise clean benchmark snapshot. Three current-source reruns were rejected
for host contention, and a fourth candidate was rejected because its task
wrapper rebuilt after the quiet gate and its watchdog fired. The accepted run
used the already-built target after a renewed two-minute quiet window and a
clean in-run watchdog. Do not restore an old timing bound or hand-edit
generated packet JSON; all three packets were regenerated from that one JSON
in AVBD -> VBD -> SI order.

## Current Evidence

### Correctness and aggregate gates

- The exact current tree passes an uncached Release build and all 79 enabled
  simulation executables in 906.85 seconds (the two aggregate wrappers remain
  intentionally disabled). The complete exact-current-tree Python suite also
  passes with 1,711 tests passed and 20 expected skips in 238.72 seconds.
- The exact current checker/capture/writer surface passes all 192 focused tests,
  including required top-level paper-capture provenance, exact ordered
  writer-owned source lists, required self-method medians, exact timing-ratio
  keys, future schema rejection, exact legacy filename-to-version pinning,
  transitive packet-link validation, and both live-corpus checks.
- Six regenerated captures and three semantic reviews bind the current source
  digest below. The accepted current-source benchmark and regenerated packet
  chain bind the same digest.
- The complete post-fix, post-reseal tree passes uncached default `test-all`
  with all six phases and uncached CUDA `test-all` with all seven phases. The
  CUDA run passed all 79 enabled simulation executables (the two aggregate
  wrappers remain intentionally disabled) and its CUDA C++ test/benchmark
  smoke path. Rerun affected gates if a review fix changes covered source. A
  green CUDA environment gate still does **not** close the missing VBD/AVBD
  solver GPU-parity rows.

### Source and benchmark provenance

- Current capture-source digest:
  `556f97640ab079316a2e8833307405eb677c836b86fe95d9eaa90eae9508de12`
- Benchmark translation-unit digest:
  `f7a0d76a19f0966f630ccb5223a6b5421c4c4b54dd1b0591ff65510eaa10b370`
- Accepted current-source benchmark:
  `/tmp/plan104-figure13-reseal.FTkRfX/figure13-three-method-accepted.json`
  (SHA-256
  `c2c81c26ca0fa96d1cf11da4c5a166a3a2eb7ead9b80aafac207c20f03cf7c67`).
  Median CPU costs/CVs are 8.612602 ms/2.04% AVBD,
  8.302657 ms/1.94% VBD, and 14.296125 ms/2.12% SI.
- Rejected current-source attempts in
  `/tmp/plan104-figure13-reseal.FTkRfX`:
  - `figure13-three-method.json`: CPU CVs 4.46% AVBD, 7.47% VBD, and 2.51% SI;
  - `figure13-three-method-clean.json`: a fresh external-load spike produced
    43.86% VBD CPU CV;
  - `figure13-three-method-quiet.json`: an unrelated parallel build started
    during the run, producing 25.74% AVBD CPU CV and implausibly inflated VBD
    and SI medians; and
  - `figure13-three-method-rejected-watchdog.json`: the task wrapper rebuilt
    after its quiet gate and the in-run watchdog fired, so the candidate was
    rejected before packet generation.

The regenerated AVBD/VBD/SI packet SHA-256 values are, in dependency order,
`3e15adb9093c10567c0f8c22c5f8e8ad4e5d47a7acd5aca7a107fb97de7fc54c`,
`9f8e6dbc9cb2f37af5d268fa83c13e67f3185cb39e14cebf413a288b6ee298d8`,
and `07151151acec718e2e851e92af2dd127f2833ee65f049445624ccffd29233ce9`.
VBD/AVBD is 0.9640x, SI/AVBD is 1.6599x, and SI/VBD is 1.7219x. Outcomes
intentionally differ, and there is no source achieved-accuracy or
same-hardware denominator for this reconstructed scene; no raw cost ratio is a
speedup, parity, or quality claim.

### Visual evidence

The temporary evidence root is
`/tmp/plan104-figure13-reseal.FTkRfX`.
All six
1280x720 captures passed engine ViewReports, pixel-integrity checks, and
separate original-resolution semantic review. The review records are
`avbd-visual-review.json`, `vbd-visual-review.json`, and
`si-visual-review.json` in that root.

| Method/checkpoint | Image SHA-256                                                      | Assessed observation                                    |
| ----------------- | ------------------------------------------------------------------ | ------------------------------------------------------- |
| AVBD frame 60     | `1d72ad775b690494372585ba9f91eb751aaefb2ba6c94a2466c9009ace94c3d5` | Globally standing wall with visible displacement damage |
| AVBD frame 120    | `199c4f6d676a65c59403da39adce607ace101ac55674f1766c0be6deaf4954d7` | Standing retained structure after fracture              |
| VBD frame 18      | `a94339bf5f07c337133ceb5790b96db33e90471cf24abca7a15f6efccb4d0676` | Bend without fracture                                   |
| VBD frame 120     | `039ee1fe05fc7fc0c350da8818be4acc7d812d7d13fcab481b18562adc84751e` | Recovery with all attachments retained                  |
| SI frame 14       | `24cff2be00f71a3ed24216e577d7a6f3d9519e69d076c724a59f4a1ac108ddee` | Initial fracture while placement remains                |
| SI frame 120      | `e1f9ecfc683fb60974a6ca76905a4f40b6b0a981e53463af230b7241cc40c97f` | Retained-row failure and collapse                       |

The pinned paper inputs used for adjudication were
`/tmp/dart-vbd-avbd-parity.teIxO8/avbd.pdf` (SHA-256
`7957d116b9130cfb0aa5a48ab7cd0d74a64ad79f75c99acd291bdece2be3f2d6`) and
`/tmp/avbd-wall-visuals.SalJ3h/paper-10.png` (SHA-256
`040361603d4e986de4d2da570593b9dc8295f8def2f57fd200de9a3e3836c61b`). All
`/tmp` paths are conveniences, not durable sources. If they have expired,
regenerate evidence with the tracked demos/writers and reacquire the pinned
paper input; never weaken a packet because a temporary artifact disappeared.

### Review state

The first role-separated review pair found the stale CV claim and the
distance-spring warm-start defect described above. A subsequent architecture
review found four more current-tree gaps: the wall claim lacked exact broken
identities and retained-row residuals, packet checking did not fail closed on
source or transitive-link drift, shared SI/AVBD pair geometry still had AVBD
ownership, and one rejection diagnostic was not solver-neutral. The current
tree addresses those findings and reseals all six captures, the accepted
benchmark, and the tracked packet chain to the new source digest. Final
independent code review approved the complete code/evidence tree. Final
architecture/docs/claims review then found two remaining fail-closed gaps: the
canonical Figure 13/video rows omitted the SI packet, and legacy packet names
were not pinned to their exact historical schema versions. The closeout tree
adds the SI evidence links, preserves the honest XPBD/CUDA/performance
blockers, pins each legacy name to version 1 or 3 with downgrade mutation
coverage, corrects active schema-version prose to version 4, and passes both
fresh reviews with no actionable finding.

## Immediate Resume Sequence

1. Stay in the exact worktree and verify the branch, upstream snapshot, status,
   and commit ladder. Expect the SI closeout commit containing this handoff at
   the clean local tip. If it is absent, preserve the recorded dirty scope and
   complete only the already-reviewed closeout procedure below.
2. Recompute the capture-source digest and run the parity, packet, and
   allocation checkers. The expected digest, accepted benchmark hash, and
   packet hashes are the values above. If any differs after a source change,
   regenerate all dependent captures, benchmark, packets, and semantic reviews
   before trusting them. Find stale live claims with:

   ```bash
   rg -n '8\.660|8\.370|13\.905|8\.697|8\.620|13\.945|9\.305|9\.043|15\.308|0\.967|0\.991|0\.972|1\.603|1\.618|1\.645|1\.693|0\.7%|1\.1%|4f8bc725|c46c2de5|bbf00f868375449e|694cde6e' \
     docs/dev_tasks/avbd_solver \
     docs/plans/104-vertex-block-descent-solver.md
   ```

3. If the SI closeout commit is absent, run `pixi run lint` immediately before
   it, re-run packet/provenance checks, inspect `git diff --check` and the exact
   staged scope, and commit locally with a plain descriptive message and no AI
   attribution. The two required independent reviews are already clean; repeat
   them only if the tree changes materially.
4. Merge the latest local `main` into this branch, never rebase it. Reconcile
   conflicts against PLAN-104's solver-family ownership, then rerun every gate
   and regenerate every source-bound artifact invalidated by the merge. Do not
   fetch, push, update a PR, resolve threads, or trigger a review without
   explicit approval.
5. Move directly to the honest public XPBD substep family and its matched
   Figure 13 row. Do not spend the next implementation slice on the two smaller
   maintenance items unless they become blocking.

## Resume Commands

```bash
cd /home/js/dev/dartsim/dart/task_5
git status --short --branch
git branch -vv --no-abbrev
git log --oneline --decorate -12
git diff --stat

pixi run python scripts/capture_source_provenance.py --digest-only
pixi run check-plan104-paper-parity
pixi run check-avbd-packets
pixi run check-plan122-allocation-matrix
```

After any code or evidence change, select proportional focused gates first.
Before SI closeout, the authoritative broad gates are:

```bash
DART_DISABLE_COMPILER_CACHE=ON pixi run test-simulation
DART_DISABLE_COMPILER_CACHE=ON pixi run test-py
DART_DISABLE_COMPILER_CACHE=ON pixi run test-all
DART_DISABLE_COMPILER_CACHE=ON pixi run -e cuda test-all
pixi run lint
git diff --check
```

`pixi run lint` is mandatory immediately before every commit. It may regenerate
or validate source-bound artifacts, so recheck provenance and packets after it.

## Continue The Full Plan After SI Closeout

1. **Honest public XPBD row.** Define a real XPBD substep solver identity and
   semantics; do not relabel SI or fixed-penalty VBD. Wire C++, dartpy,
   serialization/replay, resolved configuration, fail-closed compatibility,
   allocation evidence, focused correctness tests, and the matched Figure 13
   demo/benchmark/packet. Give XPBD an independent quantitative and assessed
   visual oracle on the shared scene fingerprint.
2. **Source-matched four-method Figure 13/video edit.** Compare SI, XPBD, VBD,
   and AVBD with audited method identities and matched inputs. Preserve the
   explicit limitation that exact Figure 13 constants are unpublished. Add
   private CUDA evidence and achieved-accuracy comparison before advancing the
   canonical figure/video rows beyond partial.
3. **Remaining CPU mechanism gaps.** Follow PLAN-104's current-next-gap owner:
   broaden articulated/joint/fracture coverage, rigid contact persistence and
   realistic stacks/piles, then unified rigid/soft rows and coupled scenes.
   Keep allocation, replay, serialization, invalidation, and fail-closed
   behavior first-class for every new world stage or row family.
4. **Complete the paper/source-demo corpus.** Use `avbd-demo-corpus.md` and the
   gap audit for every 2D/3D source scene, paper figure, parameter sweep,
   website/video scene, and comparison. Each row needs solver identity, text
   correctness, benchmark JSON, visual evidence where applicable, and explicit
   CPU/GPU status.
5. **GPU parity.** Route every landed CPU row family, candidate path, color
   update, dual/stiffness pass, and benchmark scene through the private CUDA
   boundary with deterministic warm-state behavior. A green CUDA environment
   gate alone is not solver parity.
6. **Performance leadership.** Optimize CPU and GPU until matched,
   achieved-accuracy packets beat the reference demos and the paper's
   published numbers on comparable hardware. Record hardware, commands,
   source/guard hashes, rejected experiments, and honest negative results.
7. **Completion audit.** Close all 176 canonical predicates, obtain two clean
   current-tree reviews, run default and CUDA full gates, promote durable
   lessons/evidence to their plan/design/onboarding owners, and delete this
   dev-task folder in the completing PR. Do not retire it while any doable or
   unparked parity work remains.

Two valid but lower-priority maintenance items remain: hoist duplicated
`makeCollisionPairKey` logic into one shared `detail` owner, and upgrade the
Spring / Spring Ratio packets from schema version 1 to the current
solver-identity contract.

## Claim Boundaries To Preserve

- Figure 13 does not publish exact wall dimensions, mass/inertia choices, ball
  speed/targets, or break threshold. The DART scene is a
  publication-shaped reconstruction, never an exact source replay.
- The SI frame-14 image cannot identify the five broken constraints, and the
  front-oblique VBD bend is subtle. Counts, residuals, and bend magnitude come
  from the text oracle; do not inflate image-only claims.
- AVBD has 79 of 154 broken identities outside the selected impact regions.
  Claim localized displacement damage and tightly satisfied retained rows,
  not localized broken topology.
- Candidate-only articulated finite-row timings compare work that the exact
  parent skipped. Do not manufacture before/after speedups from them.
- Finite one-DOF motor load is position-level and must be converted by
  `1 / dt^2` and aggregated with its owning compliant rows before comparison to
  public physical break force.
- DART follows paper Section 3.5 for distance-spring geometric stiffness even
  though the pinned 3D Spring source omits that term. Do not claim 3D source
  equivalence without separate adjudication.
- Historical `Complete` labels in the corpus matrix are narrow asset labels.
  The machine-checked 176-row parity contract is the completion authority.
- Previously reported allocator failures now pass in a fresh uncached profile
  build. Do not revive an old blocker without reproducing it on the current
  tree.

## Copy-paste Resume Prompt

```text
Resume PLAN-104 AVBD/VBD paper parity from
/home/js/dev/dartsim/dart/task_5 using
docs/dev_tasks/avbd_solver/RESUME.md as the exact stop-point contract. Stay on
feature/vbd-avbd-paper-parity-contract; verify live state before acting and do
not reset, stash, rebase, switch trees, push, or mutate GitHub without explicit
approval. First read AGENTS.md,
docs/ai/principles.md, docs/ai/verification.md, the PLAN-104 owner, paper gap
audit, demo corpus, parity matrix, and both dev-task files. Verify that the
reviewed SI closeout is the clean branch-tip commit; if it is not, preserve the
recorded dirty scope and finish only its final lint/check/commit sequence.
Merge the latest local main into the branch, refresh invalidated gates and
source-bound evidence, then implement an honest public XPBD substep family and
matched Figure 13 row. Continue with the source-matched four-method/CUDA
evidence and every remaining CPU/GPU, paper/site/video/demo, unified-row, and
performance gap. Keep all claims fail-closed until every one of the 176
canonical predicates passes. Do not stop at an MVP, CPU-only implementation,
or documentation checkpoint.
```
