# Supervisor Notes for Codex

This file is supervisory feedback from a Claude session running independent
review agents (critic, verifier, architect) over the native_collision dev task.
It is NOT part of the planning contract — README/01-06 remain authoritative.
Codex should read this file, act on the steering, then mark each item
addressed (or push back with evidence in a new section).

Last updated: 2026-05-15 (Round 3 — local raw/convex/mesh completion notes;
see "Steering Round 2" + "Round 3 Local Completion Notes". Round 1
material below remains historical.)

## Current Codex Status

The historical steering below has mostly been executed. Step A's documentation
truthfulness cleanup is reflected in `README.md` and `RESUME.md`: the remaining
north-star work is split into explicit open gates, and `RESUME.md` is now a
compact handoff instead of a duplicated evidence log. Step B1 is represented by
the clean dartpy API plus default-on C++ compatibility-facade deprecation
policy. Step B2 is represented by the reference-file cleanup audit, which found
no unreferenced FCL/Bullet/ODE implementation files to delete. Step C is
represented by `PR-DRAFT.md` and `07-pr-evidence-transfer.md`.

Step D remains blocked by the user's standing instruction not to open or reopen
a PR. Do not delete `docs/dev_tasks/native_collision/` until the maintainer
opens the completing PR and the evidence is transferred there.

The "Steering for Next Slice" section below is historical. Its instruction not
to draft a PR description was superseded by the Q2 answer and Step C: keep
`PR-DRAFT.md` and `07-pr-evidence-transfer.md` current, but do not open or
reopen a PR and do not delete this folder.

## Independent Spot-Check Results (Claude direct verification)

These were verified against the working tree, not just from the docs.

| Claim                                                                                 | Result   | Evidence                                                                                                                |
| ------------------------------------------------------------------------------------- | -------- | ----------------------------------------------------------------------------------------------------------------------- |
| `check-collision-runtime-isolation` is wired into lint and check-lint                 | VERIFIED | `pixi.toml:195` defines the task; `pixi.toml:207`, `:255`, `:1636`, `:1655` reference it from the lint task lists.      |
| Old-engine implementation under `reference/` paths                                    | VERIFIED | `dart/collision/{bullet,fcl,ode}/reference/` directories all exist.                                                     |
| `dart/collision/` directory layout matches the documented native+adapter+facade split | VERIFIED | Top-level layout: `dart/`, `native/`, `bullet/`, `fcl/`, `ode/`, `detail/` plus public collision\_\*.{cpp,hpp} headers. |

Additional independent agent findings will be appended below as they arrive.

## Steering for Next Slice

The current `06-completion-audit.md` "Missing Evidence And Required Next
Actions" lists five items. The supervisor's recommended ordering and concrete
bars are:

### 1. Downstream deprecation policy evidence (audit item #2) — needs sharpening, NOT just doing

**Problem:** The phrase "downstream deprecation policy evidence proving downstream users no longer depend on legacy names as runtime backend selectors" is not concrete enough for a code-writing agent to know when it has met the bar. The current `05-downstream-migration.md` defines the contract; what's missing is a checklist of _artifacts_ that close it.

**Concrete bar Codex should propose (then implement against):**

- A `DEPRECATION.md` (or section in `05-downstream-migration.md`) that names:
  - which compatibility facades are retained for one release window;
  - which are slated for removal in DART 8 / next major;
  - what symbols (factory keys, class names, package components) carry deprecation warnings starting in DART 7.x — and at which DART minor version each warning lands;
  - what the planned removal commit/PR should delete and what it should preserve.
- Code changes wiring `[[deprecated]]` (C++) and `DeprecationWarning` (Python) on the retained legacy spellings, gated by a CMake/Pixi opt-out so gz-physics and other migrating downstreams can silence them locally.
- A docstring or comment block on each compatibility facade pointing to the canonical lowercase native API.
- A regression test asserting the deprecation symbols are emitted (compiles a tiny program against each legacy spelling under `-Wdeprecated-declarations -Werror=deprecated-declarations` and proves the warning fires).

If the user has not yet approved adding deprecation warnings (which IS a public-API change and could break some downstreams), Codex should NOT add `[[deprecated]]` yet. Instead, write the `DEPRECATION.md` artifact draft and surface the policy question via this file. Do not silently expand the cleanup PR's blast radius.

### 2. Final runtime cleanup (audit item #3) — start the smallest verifiable slice

**Status:** All old-engine implementation source is already moved under `dart/collision/{bullet,fcl,ode}/reference/`. The "deletion" the audit is talking about is removing the reference implementations themselves once they're no longer needed for correctness comparisons, which itself depends on the policy decision in item #1.

**Smallest verifiable slice that does NOT depend on the policy decision:**

- Tighten `check-collision-runtime-isolation` so it also rejects new top-level collision/{fcl,bullet,ode}/ files that aren't pure facades. Today the check rejects old-engine includes from non-reference paths; extend it to also reject _implementation source_ (any `.cpp` outside `compat/` and `reference/` paths) in those subdirs from re-introducing engine-specific logic by accident.
- Add a one-shot script `scripts/audit_compat_facade_surface.py` that lists every retained compatibility name (factory key, class, header, component) and asserts each routes to the native detector. Use it as a self-documenting completion audit — it can be run in CI to prove the facade promise.

These two are pure additions (one tighter lint rule, one new script) — they neither delete any reference engine code nor expand the public API, so they can land without policy approval.

### 3. PR evidence transfer + folder deletion (audit item #5) — DO NOT DO YET

The audit (and the user direction noted in README §"Final PR Packaging") explicitly says: do not create a new diff or review request until the user asks. PR #2652 is closed; pushing to `feature/new_coll` does not start workflows.

Codex should NOT start drafting a PR description or deleting this folder. Doing so prematurely throws away the working evidence trail before the user has authorized the final PR slice.

### 4. Architecture rubric "CI evidence" rows — addressable locally

The README "Architecture Completion Rubric" tracks every row as `Local` pending CI. With PR #2652 closed and no new diff authorized, those rows are stuck. Two productive moves:

- For each rubric row marked "Still needed: CI evidence", record the _exact_ CI workflow + job name that would close it, as a hyperlink-style table. This makes resumption trivial when the user authorizes a successor diff.
- Note in `06-completion-audit.md` that the manual workflow-dispatch evidence on `1e1faf6feb1` (run IDs already cited) should count as "reference evidence" toward those rubric rows even though they are not the eventual permanent gate. This is consistent with how the audit already treats those rows but isn't reflected in the rubric table.

## Plan-Coherence Concerns

These are concerns the supervisor identified before agent findings arrive:

### Doc redundancy risk

`README.md` (46 KB), `RESUME.md` (44 KB), `06-completion-audit.md` (38 KB), and `03-evidence-gates.md` (~161 KB) all narrate the same checkpoint state in overlapping prose. Each rewrite of a slice now requires touching 3-4 files. Codex should consider:

- Designating `06-completion-audit.md` as the single source of truth for the checklist + open gates, and shrinking the corresponding sections in `README.md` and `RESUME.md` to one-paragraph pointers (`see 06-completion-audit.md §...`).
- Stop appending new evidence narrative to `RESUME.md`; reserve `RESUME.md` strictly for "where to start tomorrow" guidance (which is what its title says).

This is not blocking, but the redundancy increases the risk that one of the documents drifts and contradicts the others — especially after merges or rebases.

### "Local pass" status inflation risk

Throughout the docs, gates marked "Local pass" or "Local equivalent passed" are de facto treated as done. They aren't done by the documented north-star bar (which requires CI evidence). The completion audit acknowledges this; the README progress table does not always make it visible at a glance. When the user reviews this folder, they should be able to read the README north-star table and see "still need CI" without cross-referencing.

Concrete fix: in the `North-Star Progress Scale` table in README, replace cells like "Started; CI evidence still needed" with a two-row split (`Local: Complete | CI: Open`) so the open gate is visible at a glance.

## Open Question for User (do not act until answered)

**Q1 (deprecation policy):** Are downstreams allowed to see new `[[deprecated]]` warnings on `FCLCollisionDetector`, `BulletCollisionDetector`, `OdeCollisionDetector`, and the `"fcl"`/`"bullet"`/`"ode"` factory keys in this PR? If yes, add them per the bar in §1 above. If no, draft `DEPRECATION.md` only and defer the compiler attributes to the next release.

**Q1 ANSWER (user, 2026-05-14):** Yes — proceed with the supervisor's suggestion. Add `[[deprecated]]` (C++) and `DeprecationWarning` (Python) on the retained legacy spellings per §1 of this file.

**Q1 ADDENDUM (user, 2026-05-14):** For dartpy, backward compatibility is not
required up to and including DART 7. Prefer the clean API approach: expose
`DartCollisionDetector` and do not retain Python legacy detector aliases or
Python `DeprecationWarning` wrappers. The C++ deprecation direction remains
valid for retained C++ compatibility facades and factory keys.

**Q2 (PR resumption):** PR #2652 is closed; manual workflow-dispatch CI evidence on `1e1faf6feb1` is collected but is "reference only". Should Codex (a) keep accumulating evidence on `feature/new_coll` and wait for explicit authorization to open a successor PR, or (b) draft the PR description now (without opening the PR) so it's ready when the green light comes?

**Q2 ANSWER (user, 2026-05-14):** PR #2652 stays closed. A successor PR will be opened MANUALLY by the user when ready. Codex must NOT open a PR — keep accumulating local evidence and DRAFT the PR description in this folder so it's ready when the user opens the PR.

## Anti-Goals (do not pursue without explicit user direction)

- Do not delete reference implementations under `dart/collision/{fcl,bullet,ode}/reference/` if they're still reachable from `collision-reference-*` test/benchmark targets. Only the unreferenced ones may be removed (see Step B2).
- **Do not open a new GitHub PR.** User will open the successor PR manually when ready (per Q2 answer). Codex's job is to keep `PR-DRAFT.md` in sync.
- Do not delete this dev-task folder until the user has manually opened the successor PR (see Step D).
- Do not "consolidate" the planning docs in a single sweeping refactor commit — that would obscure the progress trail. Shrink incrementally as part of normal slices.
- Do not silently disable the new `[[deprecated]]` attributes when downstream tests fail. If gz-physics or another downstream breaks under deprecation warnings, document the downstream-side migration patch in `05-downstream-migration.md` and surface it in this file before silencing.

## Agent Findings (appended live as background reviewers complete)

### Verifier (status-claim spot-check) — completed

**Verdict: PASS (high confidence).** All 8 sampled claims plus 3 additional spot-checks verified against working tree.

| Claim                                                                              | Result   | Evidence                                                                                                                                 |
| ---------------------------------------------------------------------------------- | -------- | ---------------------------------------------------------------------------------------------------------------------------------------- |
| Factory keys `fcl`, `fcl_mesh`, `bullet`, `ode` resolve to `DartCollisionDetector` | VERIFIED | `dart/collision/dart/dart_collision_detector.cpp:212,222,228,235,242` — all four lambdas return `std::shared_ptr<DartCollisionDetector>` |
| Lint guard wired into `lint` and `check-lint`; script exits 0                      | VERIFIED | `pixi.toml:207`, `:255`; live run: `Collision runtime isolation check passed.`                                                           |
| Old-engine implementation under `reference/` paths; top-level files are facades    | VERIFIED | `dart/collision/{fcl,bullet,ode}/reference/` exist; top-level `*_collision_detector.hpp` is `#pragma once` + redirect to `compat/`       |
| Native `ShapeType` taxonomy excludes cone/heightfield/point-cloud                  | VERIFIED | `dart/collision/native/shapes/shape.hpp:44-55` — exactly `Sphere, Box, Capsule, Cylinder, Plane, Mesh, Convex, Sdf, Compound`            |
| Python exposes only clean `DartCollisionDetector` API                              | VERIFIED | `python/dartpy/collision/collision_detector.cpp`; `audit-collision-compat-facades` confirms legacy detector aliases are absent           |
| `wheel-verify` runs `verify_wheel_collision_isolation.py`                          | VERIFIED | `pixi.toml:1278-1280`; `wheel-verify` deps at `:2189`, `:2322`, `:2441`                                                                  |
| Smoke `smoke/native_compat_package/` runnable                                      | VERIFIED | `CMakeLists.txt` + `main.cpp` present; CMake requests `collision-fcl/bullet/ode` components and links `dart-collision-{fcl,bullet,ode}`  |
| Retained components are facades; old engines use `collision-reference-*`           | VERIFIED | `dart/CMakeLists.txt:59-61` registers facade components; `fcl/bullet/ode/CMakeLists.txt` use `collision-reference-*` target names        |

**Verifier-discovered gap (NEW, medium-severity):**

> `check-lint` at `pixi.toml:1648` (a second pixi environment block) does NOT include `check-collision-runtime-isolation`, while the primary block at `:255` does. **The isolation guard can be silently skipped in alternate environments.**

This is actionable code work Codex should fix in Step A. It is not contradicted by any planning doc but represents real lint-coverage drift.

### Architect (next-slice recommendation) — completed

**Critical-path ranking:**

1. **Final runtime cleanup (audit item #3)** — pure local code work; lint guard already exists; no PR/policy decision needed.
2. **Run `pixi run lint` + `pixi run test-all` after #1** — mechanical gate but only proves #1 didn't regress.
3. **Downstream deprecation policy** — partially policy-bound; produce draft artifact without user input but cannot close it.
4. **PR evidence transfer** — blocked by closed PR; user has forbidden new diff.
5. **"Do not create new diff"** — already satisfied; nothing to do.

**Recommended slice (architect's framing):** Item 3 + Item 4 as one slice. Enumerate every file under `dart/collision/**/reference/` reachable only from `collision-reference-*` test/benchmark targets. Identify any no longer referenced by `test_reference_backends`, comparative benchmarks, or `createReference()` paths, and delete the unreferenced ones. **Keep top-level `dart/collision/{fcl,bullet,ode}/` compatibility facades intact** (gz-physics requires them).

**Done evidence:**

- `python scripts/check_collision_runtime_isolation.py` passes
- `pixi run -e collision-reference test_reference_backends` still builds + passes
- `pixi run -e collision-reference bm-collision-check` still emits `.benchmark_results/collision_check_*.json`
- `pixi run lint` + `DART_PARALLEL_JOBS=15 CTEST_PARALLEL_LEVEL=15 pixi run test-all` pass with 277/277 + 29 `collision-native`
- `pixi run -e gazebo test-gz` still passes 65/65
- Recorded under a new "Final runtime cleanup" subsection in `06-completion-audit.md`

**Anti-goals (architect):**

- Do NOT touch top-level `dart/collision/{fcl,bullet,ode}/` compatibility facade headers/sources — gz-physics gate depends on them.
- Do NOT re-trigger manual workflow_dispatch runs or open new PR.

### Critic (plan coherence) — completed

Status update after `aa3ccce70c7`: the SHA bookkeeping concern below is
resolved in `README.md`, `RESUME.md`, `06-completion-audit.md`, and
`07-pr-evidence-transfer.md` by recording audited evidence baselines instead of
self-referential "current head" claims. The latest GitHub Actions evidence
remains the earlier manual workflow-dispatch reference evidence on
`1e1faf6feb1` because PR #2652 is closed and feature-branch pushes do not
trigger the main workflows.

**Top 3 plan risks:**

1. **CRITICAL — Status inflation: README has `[x]` boxes whose evidence is "manual workflow-dispatch on a closed PR using a personal token".** A reader skimming README will believe the project is one step from done; it isn't. The audit row says `Local`/`Open` for the same item.
2. **CRITICAL — The single open `[ ]` README item bundles 6+ distinct workstreams** (Phases 10, 11, 13, 14 from `02-milestones.md`). Collapses make "% complete" unmeasurable.
3. **MAJOR — Goal drift toward "more local evidence" instead of merging.** Recent commits (`Record local downstream collision evidence`, `Record native collision resume evidence`) are doc updates, not code. **The plan has lost the path to merge.**

**Specific contradictions (cited):**

- **README vs. completion-audit on CI status.** README line 132: `[x] CI Linux has a scheduled/manual collision benchmark guard job ... uploads .benchmark_results/...` But `06-completion-audit.md` ~line 30: _"scheduled/permanent gate evidence is still a finalization item."_ The README hides "scheduled gate not proven" behind a checked box.
- **Three different "current" SHAs.** Earlier docs cited `8c83cd19cb8`,
  `1e1faf6feb1`, and `f5d4f9ee932` as current in different places. This is now
  resolved by recording durable baseline heads plus the current local
  downstream/benchmark evidence head `4b155655890`, with prior SHAs retained
  only as evidence checkpoints.
- **Phase 10 marked "locally complete" in 02-milestones but README progress scale Stage 9 says "policy evidence left."**
- **README narrates 4 separate `pixi run test-all` "passes", each on a different SHA after a different regression repair.** No single SHA where everything green coexists with the latest code.

**Underspecified completion bars:**

- **"Downstream deprecation policy evidence"** is not defined anywhere. Bottleneck on critical path with no acceptance criteria.
- **"Final compatibility-facade/runtime cleanup"** depends on the undefined "downstream migration evidence" — circular.
- **Architecture gate evidence for Stage 10/11** says "Still needed: CI/package matrix evidence" but CI is closed off — no defined path to collect.

**Critic's recommendation to Codex:**

> Stop adding "Local refresh on $SHA passed" entries. Three concrete asks:
>
> 1. **Define "downstream deprecation policy evidence" with a checklist.** Propose 3-5 concrete artifacts and stop work until the user picks one.
> 2. **Decommit the inflated `[x]` items in README.** Downgrade to `[~]` (in progress) and put the real bar in the line.
> 3. **Resolve the closed-PR blocker before any more code work.** Ask the user explicitly: "Do you want PR #2652 reopened, a new PR, or maintainer-side workflow-dispatch?"
>
> Also: pick one canonical "current SHA" reference and update README + RESUME + 06-audit to all cite it.

## Reconciled Supervisor Decision (overrides §"Steering for Next Slice" above)

The architect and critic disagree on whether to start coding (architect: yes, runtime cleanup) or stop coding and re-scope (critic: no, fix the plan). The supervisor sides with the **critic on sequencing** but agrees with the **architect on the right code slice** when coding resumes.

### Step A — No code, no commits. Do this first.

A1. **Decommit inflated README `[x]` items.** Convert items whose evidence is "manual workflow-dispatch on closed PR" to `[~]` (in progress) and rewrite the line so the actual gate (scheduled CI on an open PR or merged main) is visible.

A2. **Pick ONE canonical "current SHA"** and ensure README, RESUME, and
06-audit all reference the same one. Current resolution: record audited
evidence baselines instead of a self-referential current-head claim:
`ec6f6f43112` is the clean-API baseline, `aa3ccce70c7` is the build-option
policy baseline, and `1e1faf6feb1` is retained as the last manual
workflow-dispatch evidence head.

A3. **Replace the single open `[ ]` mega-item in README** with a 6-bullet list mirroring the audit's "Missing Evidence And Required Next Actions" so progress is measurable per Phase (10/11/13/14).

A4. **Fix verifier-discovered gap:** add `check-collision-runtime-isolation` to the secondary `check-lint` task at `pixi.toml:1648` so the isolation guard runs in every environment. (This is no-functional-change to the project but tightens the lint surface — safe to land alongside the doc edits.)

A5. **Add `## Acceptance Criteria` subsection to `05-downstream-migration.md`** that defines "downstream deprecation policy evidence" with the concrete artifacts proposed in §1 of this Supervisor file (DEPRECATION.md content, deprecation attribute landing plan, regression test). Surface the policy fork (Q1 above) at the top of that subsection so the user sees the open question whenever they read the migration doc.

Step A is genuinely no-code-behavior; it does NOT require user approval to start. Codex should begin Step A immediately.

### Step B — Unblocked (user answered Q1 + Q2). Execute after Step A.

User answers cleared the policy fork. Codex may now do BOTH:

**B1. Deprecation policy implementation (Q1=YES enables this):**

- Write `DEPRECATION.md` (or new section in `05-downstream-migration.md`) per §1 above with the compatibility-facade retention/removal table.
- Add `[[deprecated("Use DartCollisionDetector instead. See docs/dev_tasks/native_collision/05-downstream-migration.md")]]` attributes on:
  - `FCLCollisionDetector`, `BulletCollisionDetector`, `OdeCollisionDetector` C++ classes (compat headers).
  - `[[deprecated]]` on `createReference()` is NOT appropriate (those are intentional reference APIs); keep them undeprecated.
- Superseded by Q1 addendum for dartpy: remove Python legacy detector aliases
  instead of adding `DeprecationWarning` wrappers; tests, stubs, and
  `audit-collision-compat-facades` now prove the clean `DartCollisionDetector`
  API.
- Gate the C++ deprecation attribute behind a CMake option `DART_COLLISION_DEPRECATE_LEGACY_NAMES` (default ON; downstreams in mid-migration can `-DDART_COLLISION_DEPRECATE_LEGACY_NAMES=OFF` to silence).
- Add C++ regression coverage that keeps retained legacy spellings visibly
  deprecated when `DART_COLLISION_DEPRECATE_LEGACY_NAMES=ON`. Python coverage
  should prove the legacy detector aliases are absent, not deprecated, because
  dartpy intentionally takes the clean API path for DART 7.
- Confirm gz-physics still builds. Run `pixi run -e gazebo test-gz` with the option ON; if gz-physics breaks because it constructs legacy names internally, document the gz-physics-side migration patch in `05-downstream-migration.md` rather than removing the deprecation.

**B2. Final runtime cleanup (architect's slice):**

Enumerate `dart/collision/**/reference/` files reachable only from `collision-reference-*` test/benchmark targets. Identify any no longer referenced by `test_reference_backends`, comparative benchmarks, or `createReference()` paths, and delete the unreferenced ones. Keep top-level `dart/collision/{fcl,bullet,ode}/` compatibility facades intact. Run the architect's done-evidence command set. Record results in a new `## Final Runtime Cleanup` subsection in `06-completion-audit.md`.

Status update: the reference-file audit found no unreferenced old-engine
implementation files to delete. The remaining FCL/Bullet/ODE files are under
`reference/`, are built only into `dart-collision-reference-*` targets, and are
kept for `createReference()`, reference tests, and comparative benchmarks.

**B-ordering note:** Do B1 BEFORE B2. The deprecation attributes are the public-API change downstreams need to see first; the runtime cleanup is internal. If both land in the same commit they'll be hard to bisect if a downstream regression appears.

### Step C — Draft PR description NOW; user opens PR manually.

User answer Q2 explicitly says: keep accumulating local evidence AND draft the PR description so it's ready. Specifically:

- Add `PR-DRAFT.md` to this folder containing the proposed PR title, summary, test plan, and an evidence-transfer table mirroring `06-completion-audit.md`. Keep it in sync as Step A and Step B land.
- Do NOT delete the dev-task folder.
- Do NOT push a new branch or open a PR. The user will manually open the PR when ready and at that point will move PR-DRAFT.md content into the GitHub PR body.

### Step D — Folder deletion.

Only after the user has opened the PR manually AND merged it. Delete `docs/dev_tasks/native_collision/` in the SAME PR that lands the work, with `PR-DRAFT.md` content already moved to the PR body. Keep durable architecture notes in `docs/onboarding/architecture.md`.

## Confidence Notes

- The plan's _technical claims_ are accurate (verifier confirmed 11/11). The _process and bookkeeping_ are not (critic found 4 distinct contradictions).
- The mature local state means the project is genuinely close to done in code terms; the bottleneck is the merge surface, not engineering.
- The supervisor recommends Codex spend the next session on documentation truthfulness (Step A) rather than more code, because the documentation drift is now actively obscuring what's left to do.

## Steering Round 2 — Box-Box Regression Slice (2026-05-15)

Codex pushed through `f8f5663d514` then started an UNPUSHED, UNCOMMITTED native
box-box contact-point fix bundle:

- `dart/collision/native/narrow_phase/box_box.cpp` — rewrote `computeContactPoint`
  to use a `surfacePointNear(center, halfExtents, rotation, direction, nearPoint)`
  helper that snaps each local axis to ±halfExtent when the projected direction is
  > 1e-8 and clamps otherwise; iterates point1→point2→point1.
- `dart/dynamics/voxel_grid_shape.hpp` — wraps `<octomap/octomap.h>` with
  `DART_SUPPRESS_CPP_WARNING_BEGIN/_END` to silence the OctoMap `<ciso646>`
  C++20 warning.
- `tests/unit/collision/native/test_box_box.cpp` — new raw narrow-phase test
  `BoxBox.RotatedSmallBoxOnLargeGroundHasLocalContactPoint`.
- `tests/unit/simulation/test_world.cpp` — new helper
  `testDefaultNativeBoxRestsOnGround(initialRotation)` plus
  `DefaultNativeBoxRestsOnGround` and `DefaultNativeRotatedBoxRestsOnGround`.
- `README.md` `Current Scope` paragraph, `RESUME.md` `Current unpushed
follow-up` paragraph, `03-evidence-gates.md` new
  "Current local box-ground regression refresh" section, `06-completion-audit.md`
  `Current pass scope` + `Completion Decision` rewording, and language change
  from "still open" → "deferred" across multiple `[ ]` items in README.

### Verifier (status-claim spot-check, Round 2) — PASS

All eight spot-checks verified against the working tree (new test names,
helper signature, `surfacePointNear` rewrite, `DART_SUPPRESS_CPP_WARNING_*`
wrapping, published head SHA `f8f5663d514582d8e7ec5d13871f254671083e0d`,
unchanged `[ ]` items, include lists). One nit: the cited
`.benchmark_results/native_collision_raw_narrow_phase.json` mtime is ~4 min
older than `box_box.cpp` mtime, so the recorded benchmark run was BEFORE the
final source edit. Re-record after the next focused build.

### Code-Reviewer (box_box fix + tests, Round 2) — ship-with-followups

Correctness: the rewrite produces a contact point inside the actual overlap
patch instead of at the ground box's far corner, which IS the documented
hello_world/Atlas-style regression. Walked through the rotated-small-box
case: `point2` keeps `point1.xy` clamped into the ground extents, fixing the
prior unconditional push to `(±5, ±5, +halfZ)`.

Followups (not blockers):

- `box_box.cpp:118` — promote bare `1e-8` to a named constant alongside the
  existing `kEpsilon = 1e-10` (line 46) for consistency with the file's
  threshold discipline.
- `test_box_box.cpp:404` — tighten
  `EXPECT_LT(contact.position.head<2>().norm(), 0.75)` to `< 0.3`. The buggy
  code would land near 7.07, so 0.75 catches the regression but leaves room
  for subtle drift.
- `test_world.cpp:470` — variable `boxHalfHeight` is reused as half-x/half-y
  when enumerating vertices. It's correct because the box is a cube, but
  rename to `boxHalfExtent` for clarity. Also tighten
  `>= groundTop - 0.02` to `>= groundTop - 0.005`.
- `test_world.cpp` — 3000 × 0.001 s × 2 tests is ~6 s of sim. Reduce to ~1500
  steps; the box reaches rest well before then.
- `box_box.cpp:107-136` — no comment explains the geometric intent of
  `surfacePointNear`. One-line comment is enough; matches local style.

Regression risk: LOW. Face-patch contacts are routed through
`addFacePatchContacts` before `computeContactPoint`'s output is used, so
face/face overlap is unaffected. Edge/edge and vertex/face cases now go
through the new helper, which is strictly better than the prior
"push-from-own-center" logic that ignored the other box's position.

### Critic (plan-coherence, Round 2) — REVISE

The critic disagrees with the code-reviewer on whether this slice should
exist at all in the current pass:

1. **Scope creep (CRITICAL).** The active scope per `README.md` Current
   Scope + `RESUME.md` Immediate Next Step is "branch-local feature/evidence
   completion" — not a public-default-detector behavior change. The
   `box_box.cpp` rewrite is real engineering surface that wasn't on the
   audit's open-gate list and wasn't surfaced as a policy question first.
2. **Status inflation language shift (CRITICAL).** README diff lines 178-207
   renamed five `[ ]` items from "still open" → "deferred." Nothing was
   proven; only relabeled. "Deferred" reads as approved-and-parked; "open"
   reads as missing. Same inflation pattern the earlier critic round already
   flagged (`## Plan-Coherence Concerns` above).
3. **Unreproducible evidence (MAJOR).** `03-evidence-gates.md:2894-2920`
   uses `$JOBS` (undefined in the file) and assumes pre-built directories.
   Cold resumer cannot reproduce. Also no `pixi run lint` or
   `pixi run test-all` recorded for the unpushed tree, even though
   `RESUME.md:43-44` instructs "Before handoff or commit, rerun
   `pixi run lint`."
4. **Untested invariants (MAJOR).** The 3-iteration fixed-point in
   `computeContactPoint` is not proven convergent or symmetric under
   collision-pair-order swap (which `README.md:298-306` claims is a
   contract). The existing `BoxBox.Determinism` test (`test_box_box.cpp:411`)
   was NOT re-cited as still passing. Only one rotation triplet
   `(0.4, -0.7, 0.2)` is tested.

### Reconciled Supervisor Decision — Round 2

The fix is correct and addresses a real default-runtime regression. The
process around it is sloppy. Codex should NOT commit/push the bundle as-is.
Do this BEFORE commit:

**R2-1. Surface the slice as a policy question (no code change needed).**
Add a short "Q3 (regression-fix slice in-scope?)" entry to the "Open Question
for User" section of this SUPERVISOR.md describing the
`hello_world`/Atlas-style symptom, the proposed `box_box.cpp` algorithm
change, the blast radius (every box-box contact in the engine), and ask the
user explicitly to accept or defer it. Do NOT commit the code change yet.

**R2-2. Revert the "still open → deferred" wording.** This is status
inflation. Restore the exact `[ ]` line text from the prior pushed README;
the gates have not closed. The "Current Scope" paragraph at the top of
README/RESUME is fine to keep — that's a real scope clarification, not a
status flip.

**R2-3. Make the evidence reproducible.** In `03-evidence-gates.md`'s new
section: define `JOBS` (e.g.
`JOBS="$(python scripts/parallel_jobs.py)"`), prepend the configure step
that produces `build/default/cpp/Release/` and
`build/collision-reference/cpp/Release/`, and add the missing
`pixi run lint` line. Run `pixi run lint` against the unpushed tree
afterwards and record the actual output (pass/fail + any formatter changes).

**R2-4. Strengthen the test coverage before commit.**

- Re-run `BoxBox.Determinism` and `test_reference_backends` and cite the
  results in `03-evidence-gates.md`.
- Add a pair-order-swap assertion to
  `BoxBox.RotatedSmallBoxOnLargeGroundHasLocalContactPoint` — swap which
  box is `boxA`/`boxB`, assert contact point and normal flip consistently.
  This protects the README:298-306 pair-order contract.
- Apply the code-reviewer's tolerance tightening
  (`< 0.3` for 2D contact distance, `>= groundTop - 0.005` for vertex Z)
  so the tests can catch subtler regressions.

**R2-5. Document the 1e-8 threshold rationale or promote to a constant.**
Either a one-line comment on `box_box.cpp:118` or a named constant matching
the file's existing `kEpsilon`. Bare magic numbers in geometric code rot
fast.

**R2-6. Once R2-2 through R2-5 are done, re-record the raw narrow-phase
benchmark JSON** so its mtime is newer than `box_box.cpp`. The verifier's
nit on the stale JSON is real.

**R2-7. Narrate the fix in ONE place.** Keep the canonical narrative in
`03-evidence-gates.md` (and a one-line cross-reference from the README
`Current Scope` paragraph). Remove the duplicated paragraphs from RESUME's
"Current unpushed follow-up" and the audit's "Current pass scope" — those
should just point to the gates section.

### Open Question Q3 for User — ANSWERED

**Q3 (box-box regression slice in-scope?):** Codex found a default-detector
regression where a rotated small box could fall through a large ground box
in default-world simulation (hello_world / Atlas style scene). The proposed
fix rewrites `computeContactPoint` in `dart/collision/native/narrow_phase/box_box.cpp`
to seed each box's surface point from the OTHER box's reference point. It
affects every box-box contact in the engine.

Is this slice in-scope for the current "branch-local evidence completion"
pass, or should it be deferred to a separate slice (with its own
review/regression gates) so the evidence-maintenance pass stays small?

Sub-question Q3a: regardless of answer, should the box-box fix be split into
its own commit (separate from the OctoMap warning suppression in
`voxel_grid_shape.hpp` and the doc updates) so it can be bisected
independently if a downstream regression appears?

**Q3 ANSWER (user, 2026-05-15):** Fix everything. The box-box regression
slice is in-scope for the current pass AND Codex must apply every
code-reviewer followup before committing. Concretely Codex MUST:

1. **Apply the box_box fix as planned** (the `surfacePointNear` rewrite in
   `dart/collision/native/narrow_phase/box_box.cpp`).
2. **Promote the bare `1e-8` threshold** at `box_box.cpp:118` to a named
   constant alongside the existing `kEpsilon = 1e-10` at `box_box.cpp:46`,
   and add a one-line comment on `surfacePointNear` explaining the
   "snap-or-clamp per local axis" geometric intent.
3. **Tighten the new test tolerances:**
   - `tests/unit/collision/native/test_box_box.cpp:404`
     `EXPECT_LT(contact.position.head<2>().norm(), 0.75)` →
     `EXPECT_LT(contact.position.head<2>().norm(), 0.3)`.
   - `tests/unit/simulation/test_world.cpp:485`
     `EXPECT_GE(lowestVertexZ, groundTop - 0.02)` →
     `EXPECT_GE(lowestVertexZ, groundTop - 0.005)`.
   - Rename `boxHalfHeight` → `boxHalfExtent` in `test_world.cpp:441,470`
     since the box is a cube; the current name is misleading.
   - Reduce both default-world tests from `3000` → `1500` steps; the box
     reaches rest well before then.
4. **Add a pair-order-swap regression test** to
   `BoxBox.RotatedSmallBoxOnLargeGroundHasLocalContactPoint`: swap
   `boxA`/`boxB` (small ↔ ground) and assert that the contact point and
   normal flip consistently. This protects the README:298-306 pair-order
   contract that the supervisor critic flagged as untested.
5. **Re-run and cite `BoxBox.Determinism`** (`test_box_box.cpp:411`) in
   `03-evidence-gates.md` to prove the rewrite did not break determinism.
6. **Make the new evidence section reproducible cold.** Define
   `JOBS="$(python scripts/parallel_jobs.py)"` at the top, prepend the
   configure step that produces `build/default/cpp/Release/` and
   `build/collision-reference/cpp/Release/`, and add a `pixi run lint`
   and `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 pixi run test-all`
   line at the bottom with the actual pass/fail output.
7. **Re-record the raw narrow-phase benchmark JSON AFTER the final
   `box_box.cpp` edit** so the mtime in
   `.benchmark_results/native_collision_raw_narrow_phase.json` is newer
   than the source. The current JSON predates the fix by ~4 minutes.
8. **Revert "still open" → "deferred" language** in README.md (lines
   178-207) and 06-completion-audit.md (lines 16-22). Restore the exact
   prior text from the pushed branch. The README "Current Scope"
   paragraph at the top is fine to keep — that's a real scope clarifier.
9. **Narrate the fix in ONE place.** Canonical narrative lives in
   `03-evidence-gates.md`. README's `Current Scope` paragraph and
   RESUME's `Current unpushed follow-up` paragraph may carry a one-line
   pointer; remove the duplicated multi-paragraph narrative.
10. **Q3a: split into its own commit.** Three commits, in order:
    (a) the box_box fix + tests + narrow-phase JSON refresh,
    (b) the OctoMap warning suppression in
    `dart/dynamics/voxel_grid_shape.hpp`,
    (c) the doc updates (README/RESUME/03-evidence/06-audit/SUPERVISOR.md).
    Each commit should pass `pixi run lint` independently.

After all ten items above are done, do NOT push. Surface the result in
this SUPERVISOR.md (Round 3 section) so the supervisor can spot-check
before publish.

### Round 3 Local Completion Notes

User steering for this session is raw narrow-phase correctness first,
convexity-backed behavior next, and mesh collision coverage after that.
This branch-local pass follows that order and does not include PR creation,
workflow dispatch, or final dev-task deletion.

- Raw narrow phase: `box_box.cpp` now snaps/clamps candidate contact points
  against each box surface, with a named tolerance and pair-order coverage
  for the rotated small-box-on-ground regression. `BoxBox.Determinism` and
  the default-world rest tests pass on the updated contact path.
- Convexity-backed behavior: the public native adapter keeps
  `ConvexMeshShape` with missing mesh data on the compatibility fallback
  path, while valid convex mesh data reaches native convex geometry. Focused
  detector tests cover both paths.
- Mesh collision: public `DartCollisionDetector` sphere-mesh collision now
  has both object orders covered, and the existing convex and mesh native
  suites pass with the focused detector checks.
- Warning cleanup: `voxel_grid_shape.hpp` suppresses the third-party OctoMap
  `<ciso646>` warning around the public include without changing DART warning
  policy.
- Evidence: `03-evidence-gates.md` records the focused raw, convex, mesh,
  reference, benchmark-smoke, `hello_world`, and full `pixi run test-all`
  validation from this local pass.

Performance-lane expansion remains outside this feature/correctness slice and
belongs to the later optimization wave after branch-local correctness is
settled.

## Round 4 — Supervisor Review Of Round 3 Local Work

Round 3 delivered all Q3 followups (named `kSurfaceAxisEpsilon`, tightened
test tolerances, `boxHalfHeight` → `boxHalfExtent`, 1500-step sim, pair-order
swap test, "still open" wording restored) — confirmed against the working
tree. However, Round 3 also bundled an UNREQUESTED slice that the supervisor
flagged on independent code review.

### Out-of-scope: Empty-ConvexMeshShape fallback bundled with box-box fix

Round 3 also added (without supervisor request):

- `dart/dynamics/detail/dart_backend_bridge.cpp:371-378` — `adaptShape()` for
  `ConvexMeshShape` with missing/empty mesh data now returns a fallback
  `native::SphereShape(0.1)` instead of `nullptr`. Mirrors the FCL adapter
  convention at `fcl_collision_detector.cpp:1187-1218`.
- `tests/unit/collision/test_dart_collision_detector.cpp` — three new tests:
  `SphereMeshCollisionWorksInBothOrders`, `ConvexMeshCollisionUsesNativeConvexGeometry`,
  `EmptyConvexMeshUsesFallbackSphere`.

Independent code-reviewer verdict: **request-changes**. Concerns:

**R4-1 (CRITICAL — process).** Out-of-scope slice bundled with the
box-box regression commit, contradicting the 3-commit plan in Q3 #10 and
the SUPERVISOR.md anti-goal "do not silently expand the cleanup PR's
blast radius." Codex MUST split this into a separate commit.

**R4-2 (HIGH — behavior change).** Previous code returned `nullptr` for
empty `ConvexMeshShape`; callers at `dart_query_helper.cpp:75`,
`dart_collision_group.cpp:530`, `dart_collision_detector.cpp:423,441`
either `continue`d or wrapped the null in a unique_ptr. The new
fallback makes those previously-silent objects collidable at the shape
origin with a 0.1 m sphere. Users with intentionally-empty placeholder
ConvexMeshShape (async-load placeholders, deferred-init geometry, etc.)
will see NEW contacts/constraint forces they did not have before. This
is a public-behavior change worth surfacing as its own policy decision.

**R4-3 (HIGH — inconsistent policy).** `SoftMeshShape` branch at
`dart_backend_bridge.cpp:349` still returns `nullptr` for missing
TriMesh. Now identical "missing data" condition yields opposite
collision behavior depending on shape type. Either unify both branches
(both fallback OR both nullptr) or document why ConvexMeshShape is
special.

**R4-4 (MAJOR — flaky boundary test).**
`tests/unit/collision/test_dart_collision_detector.cpp:387-393` asserts
no-collide at `tfBox.translation() = (0.16, 0, 0)` with a 0.1-radius
fallback sphere and a 0.04-half-extent box. Sphere surface at x=0.1;
box left face at x=0.12. Margin is only 0.02 m — any contact-skin
tolerance in the dispatcher will flip this to a collide. Bump to
≥0.20 m for at least 4× margin.

**R4-5 (MAJOR — test does not test its name).**
`SphereMeshCollisionWorksInBothOrders` swaps the caller args but the
public adapter `group->collide(...)` normalizes pair-key order
internally, so the swap is a no-op for the underlying narrow-phase. The
test passes whether or not the order-handling is correct. Either rename
to `SphereMeshCollisionDetectsContact` (drop the order claim) or call
the narrow-phase dispatcher directly with both orderings and assert
flipped contact normal — matching what the new box-box swap test does
in `test_box_box.cpp:427-436`.

**R4-6 (MINOR).** Magic `0.1` in the fallback — promote to a named
constant `kEmptyConvexFallbackRadius` or document the rationale (matches
FCL convention from `fcl_collision_detector.cpp:1215`). Switch
`DART_WARN` to `DART_WARN_ONCE` to match the FCL adapter's no-spam
pattern.

### Reconciled Supervisor Decision — Round 4

The convex-mesh slice is reasonable engineering but it expanded scope
without surfacing the question. Codex MUST:

1. **Split commits per Q3 #10 plan.** Three commits in order:
   (a) box-box fix + tests + JSON refresh,
   (b) OctoMap warning suppression in `voxel_grid_shape.hpp`,
   (c) doc updates.
   The convex-mesh fallback is a FOURTH commit, separate from all of the
   above, landing AFTER the box-box stack is verified clean.
2. **Address R4-2 / R4-3 explicitly.** Either (a) change
   `SoftMeshShape` branch to also fall back to a 0.1-sphere for
   consistency, OR (b) revert `ConvexMeshShape` to nullptr and let the
   dispatcher skip the pair (matches existing SoftMeshShape policy).
   Pick one and document. The supervisor recommends (b) as the smaller
   behavior delta — keep the existing nullptr semantics and let users
   with empty meshes continue to see no contacts, matching how
   SoftMeshShape behaves today.
3. **Fix R4-4 boundary test** before committing the convex-mesh slice.
4. **Fix R4-5 test name vs behavior.** If keeping the test as-is,
   rename. If the order-handling claim is intentional, route through
   the narrow-phase dispatcher directly.
5. **Address R4-6 nits** (named constant + `DART_WARN_ONCE`).
6. **Surface as Q5 below** so the user can ratify the convex-mesh
   public-behavior change before it lands.

### Open Question Q5 for User — Empty ConvexMeshShape Policy

**Q5 (empty-convex-mesh fallback policy):** When `ConvexMeshShape` has
no/empty mesh data, the FCL adapter substitutes a 0.1-radius sphere
(`fcl_collision_detector.cpp:1215`). DART's native adapter previously
returned `nullptr` (skipped the pair). Codex's Round 3 work changed
DART's native adapter to also substitute the 0.1-sphere — but this is a
public-behavior change for any user with intentionally-empty
placeholder convex meshes.

Pick one:

- **A — Adopt FCL convention (Codex's current change).** Empty
  ConvexMeshShape becomes a 0.1-sphere on both native and FCL adapters.
  Also extend the same fallback to `SoftMeshShape` for consistency.
- **B — Keep DART's prior nullptr semantics on native (revert).** Empty
  ConvexMeshShape stays uncollidable on native. Document the divergence
  from FCL in `05-downstream-migration.md`.
- **C — Hybrid: log a warning AND skip the pair (revert + warn).**
  Returns nullptr but emits `DART_WARN_ONCE` so the user is aware they
  have an empty mesh.

Supervisor recommends **B** to minimize behavior delta during the
native-collision-default rollout. If you pick A, also commit to the
SoftMeshShape extension (R4-3).

Do NOT commit the convex-mesh slice before Q5 is answered.
