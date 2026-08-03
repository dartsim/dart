# Resume: AVBD Solver

Working handoff for PLAN-104's full VBD/AVBD paper-parity implementation. The
durable roadmap and acceptance bar live in
[`../../plans/104-vertex-block-descent-solver.md`](../../plans/104-vertex-block-descent-solver.md);
the fail-closed completion contract is
[`../../plans/104-vertex-block-descent-solver/paper-parity-matrix.md`](../../plans/104-vertex-block-descent-solver/paper-parity-matrix.md).
This file owns only the exact branch/session stop point and takeover sequence.

## Stop Point

At the current pre-publication branch state, the public Sequential Impulse (SI)
pair-row closeout at `37178193ccf` is integrated with `origin/main` snapshot
`ef10cb2633b`. Merge conflicts were reconciled without changing solver-family
ownership: the shared forward declarations follow `main`, while SI keeps its
bounds-safe span indexing and the boxed-LCP scratch path keeps `main`'s
allocator-backed implementation. Six captures, three semantic reviews, one
strict quiet-host benchmark, and the AVBD -> VBD -> SI packet chain are
regenerated against the merged source digest. All 192 focused tests, the
58-packet corpus checker, the 176-row parity checker, the 18-row allocation
matrix, and both uncached default and CUDA aggregate gates pass. Exact aggregate
results are recorded below.

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
- Current foundation merge tip before the evidence-refresh commit:
  `52173fba3aa415cdf830858693195a5fbc6397b8`
- Resume tip: the branch-tip evidence-refresh commit after `52173fba3aa`; verify
  its live SHA and subject before acting
- Pre-publication feature upstream snapshot:
  `origin/feature/vbd-avbd-paper-parity-contract` at
  `37a02b4d2c71036bdc8f3c394468c4852ee10efd`
- Merged remote-tracking `main` snapshot:
  `ef10cb2633b32d2fe061a19d1abd5f7d61561fe8`

Treat the recorded remote-tracking SHAs as pre-publication snapshots, not fresh
remote assertions. The foundation may already have been pushed or opened as a
PR after this handoff was committed. Verify live local, remote, PR, CI, and
review state before any external action, and do not infer that a prior session's
mutation approval remains active.

The evidence-refresh commit after `52173fba3aa` should leave the tracked scope
clean and no task-created untracked files. If resuming before that commit
exists, preserve the working tree exactly; do not reset, stash, rebase, switch
worktrees, abort an in-progress merge, or discard files.

The local commit ladder above `main` is:

| Commit        | Local slice                                        |
| ------------- | -------------------------------------------------- |
| `710cbfc1152` | Fail-closed VBD/AVBD paper-parity contracts        |
| `a78f688a178` | AVBD Section 3.5 quasi-Newton geometric stiffness  |
| `0b0154573b8` | AVBD Section 4 parallel dual/stiffness update      |
| `761263bbd41` | Articulated finite masked rows                     |
| `9ebd9b895b1` | Articulated finite movable-pair motors             |
| `131981788fa` | Articulated finite load and fracture accounting    |
| `646b447d6cc` | Public AVBD solver and Figure 13 evidence          |
| `d9af744a9cb` | Public fixed-penalty VBD comparison                |
| `37178193ccf` | Public SI pair rows and Figure 13 comparison       |
| `37a02b4d2c7` | Merge local `main` snapshot `b15e7a652b7`          |
| `52173fba3aa` | Merge fetched `origin/main` snapshot `ef10cb2633b` |
| branch tip    | Refresh Figure 13 evidence after the latest merge  |

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

- The resealed current tree passes the uncached default `test-all` in all six
  phases: 229/229 core C++ tests, all 81 runnable simulation tests plus two
  intentionally disabled aggregate wrappers, and the complete Python and docs
  phases. No exact pytest count is claimed from the aggregate runner's captured
  output.
- The same tree passes the uncached CUDA `test-all` in all seven phases:
  213/213 core C++ tests, all 80 runnable simulation tests plus two disabled
  wrappers, the complete Python and docs phases, all eight CUDA device tests,
  and all nine CUDA benchmark smoke commands. No exact pytest count is claimed
  from the aggregate runner's captured output.
- The exact current checker/capture/writer surface passes all 192 focused tests,
  including required top-level paper-capture provenance, exact ordered
  writer-owned source lists, required self-method medians, exact timing-ratio
  keys, future schema rejection, exact legacy filename-to-version pinning,
  transitive packet-link validation, and both live-corpus checks.
- Six regenerated captures and three semantic reviews bind the current source
  digest below. The accepted current-source benchmark and regenerated packet
  chain bind the same digest.
- Merging `origin/main` at `ef10cb2633b` changed the capture-source digest and
  made the prior aggregate results historical. The resealed current head passes
  the focused 192-test surface, 58-packet corpus checker, 176-row parity
  checker, 18-row allocation matrix, and both aggregate gates above. A green
  CUDA environment gate still does **not** close the missing VBD/AVBD
  solver GPU-parity rows.

### Source and benchmark provenance

- Current capture-source digest:
  `10400178732ca18b4c0717c7b531766e20f31f9c5de01efc35144332f9580b8d`
- Benchmark translation-unit digest:
  `f7a0d76a19f0966f630ccb5223a6b5421c4c4b54dd1b0591ff65510eaa10b370`
- Accepted current-source benchmark:
  `/tmp/plan104-figure13-current-main.C9ZSFU/figure13-three-method.json`
  (SHA-256
  `abab909f3693333c0bb43bdd54b15df9fd6f0c3e9d4b7dd838208fb76d12565a`).
  Median CPU costs/CVs are 9.515544 ms/0.49% AVBD,
  9.292560 ms/0.18% VBD, and 15.651049 ms/0.34% SI. The initial 1-minute load
  average was 0.62; an armed concurrent watchdog observed no competing
  build/test workload during the run.
- The previously accepted forward-merge packet root
  `/tmp/plan104-figure13-fwd-merge.VfZPUI` is preserved but source-stale after
  merging `origin/main` at `ef10cb2633b`; its captures, benchmark, and packets
  are historical non-evidence.
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
`d0098a6fac2f53c9b9f6d1898e27d46cfecb4a208f32f2c08c5703583f3c31bf`,
`5db4c77cad4c35f9a10abc2dc892e8849f6a96e1acbaf8162902ab5091503be7`,
and `60612553068dc5eae674a2fdfac7d5359a7b8a027cf636fabd9ba205842489eb`.
VBD/AVBD is 0.9766x, SI/AVBD is 1.6448x, and SI/VBD is 1.6843x. Outcomes
intentionally differ, and there is no source achieved-accuracy or
same-hardware denominator for this reconstructed scene; no raw cost ratio is a
speedup, parity, or quality claim.

### Visual evidence

The temporary evidence root is
`/tmp/plan104-figure13-current-main.C9ZSFU`.
All six
1280x720 captures passed engine ViewReports, pixel-integrity checks, and
separate original-resolution semantic review. The review records are
`avbd-visual-review.json`, `vbd-visual-review.json`, and
`si-visual-review.json` in that root.

| Method/checkpoint | Image SHA-256                                                      | Assessed observation                                    |
| ----------------- | ------------------------------------------------------------------ | ------------------------------------------------------- |
| AVBD frame 60     | `b44fc5d55df54955f53f67330689b8e9356201c417ce932793366368c15e2a3f` | Globally standing wall with visible displacement damage |
| AVBD frame 120    | `a0777940e033e4684ec7f2f747a1b5b2a2694f849aa88d04a08e82dbb7d0c087` | Standing retained structure after fracture              |
| VBD frame 18      | `dc05890e8f09f42dfc9dd5a1c06662a15bac88bded3197eeda8c333fc40eca06` | Bend without fracture                                   |
| VBD frame 120     | `69a2f2390ff4786f68f60866356291c67968aaf975e00853440c5794fdc6d590` | Recovery with all attachments retained                  |
| SI frame 14       | `678af175bd8cbbea575dd152884996dd36aacec63bf0b66d38a81ebd7d92fc43` | Initial fracture while placement remains                |
| SI frame 120      | `83ca961c52b606a5a90c2232e3f3f4d60d970783cb22c1b0a40dcc97d22fce29` | Retained-row failure and collapse                       |

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
   merges at `37a02b4d2c7` and `52173fba3aa`, then the evidence-refresh commit.
   If the refresh commit is absent, preserve the working tree and complete only
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

3. If the evidence-refresh commit is absent, run `pixi run lint` immediately
   before it, re-run packet/provenance checks, inspect `git diff --check` and
   the exact staged scope, and commit locally with a plain descriptive message
   and no AI attribution. Repeat proportional gates if a post-merge fix changes
   covered source.
4. Inspect live GitHub state. If the foundation PR is open, manage only its
   current head through CI and review under fresh maintainer approval. Never
   rebase a published PR branch; merge the latest base before every follow-up
   push. Do not merge without separate explicit approval.
5. Do not extend this shared-foundation branch into the remaining parity
   program. After maintainer approval and landing, create one VBD-completion PR
   from updated `main`; only after that lands, create one AVBD-completion PR
   from updated `main`.

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
For any material source change, the authoritative broad gates are:

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
needed, but do not mix the two completion claims or create separate media-only
PRs. Each method's completion PR owns its implementation, tests, CPU/GPU
benchmarks, text oracles, and current-head PR-hosted images, GIFs, and videos
for every paper, project-site, official-video, and source-demo row. Keep raw
and generated transient media out of git and expose it through GitHub
attachments, following the same repository-hygiene boundary as PR #3377.

1. **VBD completion PR first.** Start from updated `main` only after the shared
   SI/VBD/AVBD foundation lands. Close every one of the 88 VBD predicates,
   including the remaining CPU mechanisms, private CUDA parity, complete
   paper/site/video/source-demo corpus, allocation/replay/serialization
   contracts, and achieved-accuracy performance leadership. Implement the
   honest public XPBD substep comparison and source-matched Figure 13/video
   evidence in this PR because they are required comparators for the VBD
   contract. Publish the complete VBD replication image/video set in this same
   PR; do not defer it to a presentation follow-up. Do not relabel SI or
   fixed-penalty VBD as XPBD. Keep AVBD changes limited to unavoidable
   shared-interface compatibility, with no AVBD completion claim.
2. **Land and rebase the plan, not the published branch.** Manage the VBD PR
   through review and CI only with explicit approval. After it lands, refresh
   local `main` and the fail-closed matrices. Never rebase a published PR
   branch.
3. **AVBD completion PR second.** Start from updated `main` containing the VBD
   completion. Close every one of the 88 AVBD predicates: augmented-Lagrangian
   dual/stiffness evolution, quasi-Newton terms, articulated/joint/fracture and
   rigid-contact breadth, unified soft/rigid coupling, all CPU/CUDA paths, the
   remaining corpus, four-method evidence, and matched performance leadership.
   Publish the complete AVBD replication image/video set in this same PR.
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
closeout at `37178193ccf` is followed by merges `37a02b4d2c7` and
`52173fba3aa`, then the final evidence-refresh commit. If the foundation PR is
open, manage only its current head through CI and review; do not merge without
separate explicit approval. Do not extend the shared foundation branch into
the remaining parity program. After maintainer approval and landing, complete
VBD first in one PR from updated main, including honest XPBD comparison, all
CPU/CUDA and corpus rows, performance leadership, and all current-head
paper/site/video/source-demo image and video attachments; do not create a
separate media PR.
After that PR lands, complete AVBD in a second PR from updated main, including
all AVBD-specific CPU/CUDA, unified-row, corpus, four-method, performance, and
current-head image/video evidence; do not create a separate media PR. Keep all
claims fail-closed until every one of the 176 canonical predicates passes. Do
not stop at an MVP, CPU-only implementation, or documentation checkpoint, and
do not push or mutate GitHub without explicit approval.
```
