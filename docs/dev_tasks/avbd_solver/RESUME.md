# Resume: AVBD Solver

Working handoff for PLAN-104's full VBD/AVBD paper-parity implementation. The
durable roadmap and acceptance bar live in
[`../../plans/104-vertex-block-descent-solver.md`](../../plans/104-vertex-block-descent-solver.md);
the fail-closed completion contract is
[`../../plans/104-vertex-block-descent-solver/paper-parity-matrix.md`](../../plans/104-vertex-block-descent-solver/paper-parity-matrix.md).
This file owns only the exact branch/session stop point and takeover sequence.

## Stop Point

> **Current Reality (2026-09-04):** the shared VBD/AVBD foundation merged to
> `main` in PR #3432 (`aafc4b66072`, 2026-09-03): the fail-closed 88-row VBD
> and 88-row AVBD parity contracts, AVBD Section 3.5 quasi-Newton geometric
> stiffness and Section 4 parallel dual/stiffness updates, the articulated
> finite-row slices, public Sequential Impulse / VBD / AVBD family selection
> with SI-owned hard pair rows, the named AVBD parameter profiles (D5), the
> `MassScaledReference` public default (D6), and the resealed matched Figure 13
> evidence. No feature branch is open; the next slice starts from updated
> `main`. Open items recorded in the README and the plan: the
> `Paper2025Table2` `beta`/`k_start` units if that profile is ever to run with
> the adaptive initial guess, and sliding friction under the 2D profile
> (bounded, not matched). The session ledger with decisions D1-D6 lives
> outside the tree.

## Base And Branch

- Base: updated `main` containing `aafc4b66072` (PR #3432).
- Branch: none open. Create the VBD-completion branch from `origin/main` when
  that PR starts; create the AVBD-completion branch only after the VBD PR
  lands. Verify live local, remote, PR, CI, and review state before any
  external action; a prior session's mutation approval does not carry over.

## Sealed Evidence

The landed Figure 13 evidence has durable owners; do not copy their digests
here:

- `README.md` § "Current packet chain" names the sealed capture-source digest
  and the schema-6 packets under
  `docs/plans/104-vertex-block-descent-solver/`; the packets record the
  artifact hashes, pinned paper-input hashes, and solver fingerprints and
  validate them transitively. `pixi run check-avbd-packets` and
  `pixi run check-plan104-paper-parity` are the strict claim gates;
  `pixi run lint` runs the same validators with `--stale-source report`, so a
  seal that predates the current source is an advisory there and an error
  here.
- `README.md` § "Current local gates" records the default and CUDA
  `test-all` passes on the sealed bytes. A green CUDA environment gate does
  not close the missing VBD/AVBD solver GPU-parity rows.
- Median CPU cost per step and the cross-method ratios live in the plan's
  progress log; outcomes intentionally differ and no raw cost ratio is a
  speedup, parity, or quality claim.

## Immediate Resume Sequence

1. `git fetch origin main`, confirm `aafc4b66072` is in `origin/main`, and
   branch from it for the next completion PR.
2. Recompute the capture-source digest (expected value: `README.md`
   § "Current packet chain") and run the parity, packet, and allocation
   checkers (commands below). If any digest or hash differs after a
   source change, regenerate all dependent captures, benchmark, packets, and
   semantic reviews before trusting them. Find stale live claims with:

   ```bash
   rg -n '8\.660|8\.370|13\.905|8\.697|8\.620|13\.945|9\.305|9\.043|15\.308|0\.967|0\.991|0\.972|1\.603|1\.618|1\.645|1\.693|0\.7%|1\.1%|4f8bc725|c46c2de5|bbf00f868375449e|694cde6e' \
     docs/dev_tasks/avbd_solver \
     docs/plans/104-vertex-block-descent-solver.md
   ```

3. Run `pixi run lint` immediately before every commit, re-run packet and
   provenance checks after it, inspect `git diff --check` and the exact staged
   scope, and commit with a plain descriptive message and no AI attribution.
4. GitHub mutations (push, PR creation or edits, review re-triggers, merge)
   need explicit approval each time. Never rebase a published PR branch; merge
   the latest base before every follow-up push.
5. Follow "Continue The Full Plan After The Shared Foundation" below: VBD
   completion in one PR from updated `main`, then AVBD completion in a second
   PR.

## Resume Commands

```bash
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
- AVBD has 21 of 36 broken identities outside the selected impact regions and
  no displaced brick. Claim three localized joint-break clusters on a standing
  wall and tightly satisfied retained rows, not the broken-open wall of
  Figure 13(d); the review records that panel's visual agreement as not
  proven.
- Candidate-only articulated finite-row timings compare work that the exact
  parent skipped. Do not manufacture before/after speedups from them.
- Finite one-DOF motor load is position-level and must be converted by
  `1 / dt^2` and aggregated with its owning compliant rows before comparison to
  the public solver-row break metric. Linear-force and angular-torque
  coordinates are not normalized by a characteristic length, so do not call
  that scalar a physical-unit wrench norm.
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
Resume PLAN-104 AVBD/VBD paper parity using
docs/dev_tasks/avbd_solver/RESUME.md as the stop-point contract. The shared
foundation is on main (PR #3432); branch from updated origin/main and verify
live state before acting. Do not reset, stash, rebase, switch trees, push, or
mutate GitHub without explicit approval. First read AGENTS.md,
docs/ai/principles.md, docs/ai/verification.md, the PLAN-104 owner, paper gap
audit, demo corpus, parity matrix, and both dev-task files. Complete VBD first
in one PR, including honest XPBD comparison, all CPU/CUDA and corpus rows,
performance leadership, and all current-head paper/site/video/source-demo
image and video attachments; do not create a separate media PR. After that PR
lands, complete AVBD in a second PR from updated main, including all
AVBD-specific CPU/CUDA, unified-row, corpus, four-method, performance, and
current-head image/video evidence; do not create a separate media PR. Keep all
claims fail-closed until every one of the 176 canonical predicates passes. Do
not stop at an MVP, CPU-only implementation, or documentation checkpoint.
```
