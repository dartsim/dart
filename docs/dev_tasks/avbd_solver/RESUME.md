# Resume: AVBD Solver

Working handoff for PLAN-104's full VBD/AVBD paper-parity implementation. The
durable roadmap and acceptance bar live in
[`../../plans/104-vertex-block-descent-solver.md`](../../plans/104-vertex-block-descent-solver.md);
the fail-closed completion contract is
[`../../plans/104-vertex-block-descent-solver/paper-parity-matrix.md`](../../plans/104-vertex-block-descent-solver/paper-parity-matrix.md).
This file owns only the exact branch/session stop point and takeover sequence.

## Stop Point

At the branch-tip merge commit containing this handoff, the public Sequential
Impulse (SI) pair-row closeout at `37178193ccf` is integrated with local `main`
snapshot `b15e7a652b7`. The two merge conflicts were reconciled without
changing solver-family ownership: the shared forward declarations follow
`main`, while SI keeps its bounds-safe span indexing and the boxed-LCP scratch
path keeps `main`'s allocator-backed implementation. The merge passes an
uncached Release build and all 81 enabled simulation executables. Six captures,
three semantic reviews, one strict quiet-host benchmark, and the AVBD -> VBD ->
SI packet chain are regenerated against the merged source digest. All 192
focused tests, the 58-packet corpus checker, the 176-row parity checker, and the
18-row allocation matrix pass. The final default/CUDA aggregate gate results
are recorded below.

The 88-row VBD and 88-row AVBD contracts remain fail-closed with no complete
rows. The local three-method Figure 13 packet is useful partial evidence, not
paper parity. Full scope is still all paper/reference algorithms and features,
CPU and GPU implementations, every paper/site/video/source-demo experiment,
and performance that beats the comparable reference and published paper
numbers. Do not reduce this to an MVP, a CPU-only lane, or one wall demo.

## Exact Worktree And Branch

- Worktree: `/home/js/dev/dartsim/dart/task_5`
- Branch: `feature/vbd-avbd-paper-parity-contract`
- Pre-merge local `HEAD`: `37178193ccf090843c5df6afeb1221d090f2b673`
- Resume tip: the branch-tip merge commit containing this handoff; verify its
  live SHA
- Last-fetched feature upstream ref:
  `origin/feature/vbd-avbd-paper-parity-contract` at
  `d9af744a9cb0763033d76e01381dbf7c1e6ecaa9`
- Merged local and remote-tracking `main` snapshot:
  `b15e7a652b766897a8a69a121742ef5973c77f3c`

No fetch, push, PR edit, review trigger, or other GitHub mutation was performed
while closing SI or integrating `main`. Treat the recorded remote-tracking SHAs
as local snapshots, not fresh remote assertions. Verify live state before any
external action, and do not push or mutate GitHub without explicit
maintainer/user approval.

The merge commit containing this handoff should leave the tracked scope clean
and no task-created untracked files. If resuming before that commit exists,
preserve the in-progress merge exactly; do not reset, stash, rebase, switch
worktrees, abort the merge, or discard files.

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
| `37178193ccf` | Public SI pair rows and Figure 13 comparison      |
| branch tip    | Merge local `main` snapshot `b15e7a652b7`         |

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

- The merged tree passes an uncached Release build and all 81 enabled
  simulation executables in 123.64 seconds (the two aggregate wrappers remain
  intentionally disabled). The final aggregate default/CUDA results are
  recorded below.
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
  CUDA run passed 213 core C++ tests, 80 runnable simulation tests with two
  aggregate wrappers intentionally disabled, all eight CUDA device tests, and
  all nine benchmark-smoke commands. Its complete Python and documentation
  phases also passed; no exact pytest count is claimed from the aggregate
  runner's captured output. Rerun affected gates if a review fix changes
  covered source. A green CUDA environment gate still does **not** close the
  missing VBD/AVBD solver GPU-parity rows.

### Source and benchmark provenance

- Current capture-source digest:
  `b3f102b53c820809c0a8b34c0bf3002853b3b1e79ce17f56fb6380d10cccfd51`
- Benchmark translation-unit digest:
  `f7a0d76a19f0966f630ccb5223a6b5421c4c4b54dd1b0591ff65510eaa10b370`
- Accepted current-source benchmark:
  `/tmp/plan104-figure13-fwd-merge.VfZPUI/figure13-three-method.json`
  (SHA-256
  `b377ef3e307a3f5ebbe935ff3c4abcace477cfbb92062296947b4b3537d0427d`).
  Median CPU costs/CVs are 9.053841 ms/0.57% AVBD,
  8.953167 ms/1.79% VBD, and 14.783755 ms/0.48% SI. The initial 1-minute load
  average was 0.76; an armed concurrent watchdog observed no competing build/test
  workload during the 18-second run.
- The earlier merged-source packet root
  `/tmp/plan104-figure13-main-merge.aVseEf` is preserved but rejected because
  the subsequent forward-declaration owner fix changed the capture-source
  digest. Its captures, benchmark, and packets are historical non-evidence.
- Historical rejected pre-merge attempts remain in
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
`494f07ff9b26e61e0e5aa9a8e487d1e94975dca0ea17b0216d1d80f411480391`,
`96652e9c625a4c6307a10ba6a763db26ff03e5f4f2c3749fb4911bb7b40a144b`,
and `6ee2b5242dbdbe39fa2f17b84259bcd54f4a6a492e61a279962d28d96be425ae`.
VBD/AVBD is 0.9889x, SI/AVBD is 1.6329x, and SI/VBD is 1.6512x. Outcomes
intentionally differ, and there is no source achieved-accuracy or
same-hardware denominator for this reconstructed scene; no raw cost ratio is a
speedup, parity, or quality claim.

### Visual evidence

The temporary evidence root is
`/tmp/plan104-figure13-fwd-merge.VfZPUI`.
All six
1280x720 captures passed engine ViewReports, pixel-integrity checks, and
separate original-resolution semantic review. The review records are
`avbd-visual-review.json`, `vbd-visual-review.json`, and
`si-visual-review.json` in that root.

| Method/checkpoint | Image SHA-256                                                      | Assessed observation                                    |
| ----------------- | ------------------------------------------------------------------ | ------------------------------------------------------- |
| AVBD frame 60     | `e0a998b1710f36e1e3b82bfeb3f20c18d99e42ad8f429f42321e364abcdc246e` | Globally standing wall with visible displacement damage |
| AVBD frame 120    | `a75dfc58f790864c9bad224d8c658eead3134a14362cf4bffd0e6a2744e77af9` | Standing retained structure after fracture              |
| VBD frame 18      | `f42b029583607aea696edfa983bde6aaa9815952ff5fae798bfd858831770c92` | Bend without fracture                                   |
| VBD frame 120     | `bd51bb4a2f35168b52fec76aa366567905239b31cc7c47a7c349072f1b11774e` | Recovery with all attachments retained                  |
| SI frame 14       | `41c4e781d0a2d864883bea0af2bf9c46380a3dfbc46c30422293b7ca6fd92019` | Initial fracture while placement remains                |
| SI frame 120      | `665ec2b3b9c3c5a857666ab4304bb59322709cca9ebbaf9e1e4d3c3d26bc6e36` | Retained-row failure and collapse                       |

The pinned paper inputs used for adjudication were
`/tmp/dart-vbd-avbd-parity.teIxO8/avbd.pdf` (SHA-256
`7957d116b9130cfb0aa5a48ab7cd0d74a64ad79f75c99acd291bdece2be3f2d6`) and
`/tmp/avbd-wall-visuals.SalJ3h/paper-10.png` (SHA-256
`040361603d4e986de4d2da570593b9dc8295f8def2f57fd200de9a3e3836c61b`). All
`/tmp` paths are conveniences, not durable sources. If they have expired,
regenerate evidence with the tracked demos/writers and reacquire the pinned
paper input; never weaken a packet because a temporary artifact disappeared.

### Review state

The first pre-merge role-separated review pair found the stale CV claim and the
distance-spring warm-start defect described above. A subsequent architecture
review found four more current-tree gaps: the wall claim lacked exact broken
identities and retained-row residuals, packet checking did not fail closed on
source or transitive-link drift, shared SI/AVBD pair geometry still had AVBD
ownership, and one rejection diagnostic was not solver-neutral. The current
tree addresses those findings and reseals all six captures, the accepted
benchmark, and the tracked packet chain to the new source digest. The final
pre-merge independent code review approved the complete SI code/evidence tree.
The final pre-merge architecture/docs/claims review then found two remaining
fail-closed gaps: the canonical Figure 13/video rows omitted the SI packet, and
legacy packet names were not pinned to their exact historical schema versions.
The SI closeout added the evidence links, preserved the honest
XPBD/CUDA/performance blockers, pinned each legacy name to version 1 or 3 with
downgrade mutation coverage, corrected active schema-version prose to version
4, and passed both fresh pre-merge reviews with no actionable finding. The
subsequent `main` merge, conflict resolutions, provenance reseal, and handoff
corrections received the final local review recorded above.

## Immediate Resume Sequence

1. Stay in the exact worktree and verify the branch, upstream snapshot, status,
   and commit ladder. Expect the SI closeout at `37178193ccf` followed by the
   clean branch-tip merge of local `main` snapshot `b15e7a652b7`. If the merge
   commit is absent, preserve the recorded in-progress merge and complete only
   its final lint/check/commit sequence.
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

3. If the merge commit is absent, run `pixi run lint` immediately before it,
   re-run packet/provenance checks, inspect `git diff --check` and the exact
   staged scope, and commit locally with a plain descriptive message and no AI
   attribution. Repeat review if conflict resolution or a post-merge fix
   changes materially.
4. Do not extend this shared-foundation branch into the remaining parity
   program. After maintainer approval and landing, create one VBD-completion PR
   from updated `main`; only after that lands, create one AVBD-completion PR
   from updated `main`. Never rebase a published PR branch, and do not push or
   mutate GitHub without explicit approval.

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

## Continue The Full Plan After The Shared Foundation

The maintainer directed the remaining work into two ordered completion PRs.
Use multiple reviewable commits and durable work packets inside each PR if
needed, but do not mix the two completion claims.

1. **VBD completion PR first.** Start from updated `main` only after the shared
   SI/VBD/AVBD foundation lands. Close every one of the 88 VBD predicates,
   including the remaining CPU mechanisms, private CUDA parity, complete
   paper/site/video/source-demo corpus, allocation/replay/serialization
   contracts, and achieved-accuracy performance leadership. Implement the
   honest public XPBD substep comparison and source-matched Figure 13/video
   evidence in this PR because they are required comparators for the VBD
   contract; do not relabel SI or fixed-penalty VBD as XPBD. Keep AVBD changes
   limited to unavoidable shared-interface compatibility, with no AVBD
   completion claim.
2. **Land and rebase the plan, not the published branch.** Manage the VBD PR
   through review and CI only with explicit approval. After it lands, refresh
   local `main` and the fail-closed matrices. Never rebase a published PR
   branch.
3. **AVBD completion PR second.** Start from updated `main` containing the VBD
   completion. Close every one of the 88 AVBD predicates: augmented-Lagrangian
   dual/stiffness evolution, quasi-Newton terms, articulated/joint/fracture and
   rigid-contact breadth, unified soft/rigid coupling, all CPU/CUDA paths, the
   remaining corpus, four-method evidence, and matched performance leadership.
   Reuse the landed VBD/XPBD evidence by hash rather than duplicating or
   weakening its acceptance boundary.
4. **Final 176-row audit.** After the AVBD PR is current, obtain clean code and
   architecture/claims reviews, run uncached default and CUDA full gates,
   promote durable artifacts to their owners, and delete this dev-task folder
   in the completing PR. Do not retire it while any doable or unparked parity
   work remains.

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
audit, demo corpus, parity matrix, and both dev-task files. Verify that the SI
closeout at `37178193ccf` is followed by the clean branch-tip merge of local
main snapshot `b15e7a652b7`; if it is not, preserve the in-progress merge and
finish only its final lint/check/commit sequence. Do not extend the shared
foundation branch into the remaining parity program. After maintainer approval
and landing, complete VBD first in one PR from updated main, including honest
XPBD comparison, all CPU/CUDA and corpus rows, and performance leadership.
After that PR lands, complete AVBD in a second PR from updated main, including
all AVBD-specific CPU/CUDA, unified-row, corpus, four-method, and performance
gaps. Keep all claims fail-closed until every one of the 176 canonical
predicates passes. Do not stop at an MVP, CPU-only implementation, or
documentation checkpoint, and do not push or mutate GitHub without explicit
approval.
```
