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
- Convexity-backed behavior: valid convex mesh data reaches native convex
  geometry. A later Q5 policy update supersedes Round 3's temporary empty-mesh
  fallback and makes invalid convex/soft mesh data non-collidable with
  `DART_WARN_ONCE`.
- Mesh collision: public `DartCollisionDetector` sphere-mesh collision has
  focused contact coverage, and the existing convex and mesh native suites pass
  with the focused detector checks.
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

### Open Question Q5 for User — ANSWERED

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

**Q5 ANSWER (user, 2026-05-15):** Prefer the cleaner long-term approach
over backward-compatibility-preserving conservative steps. Concretely:
adopt a **modified Option B** — revert ConvexMeshShape to `nullptr` on
native, BUT make that nullptr the consistent policy across all shape
types, surface it to users via `DART_WARN_ONCE`, and treat FCL's silent
sphere-substitution as technical debt slated for removal in DART 8.

Codex MUST do all of the following in the standalone "convex-mesh
slice" commit (NOT bundled with the box-box fix):

1. **Revert Round 3's `adaptShape()` change** in
   `dart/dynamics/detail/dart_backend_bridge.cpp:371-378` — restore the
   `nullptr` return for empty/missing `ConvexMeshShape`, but ADD a
   `DART_WARN_ONCE` so users see the problem.
2. **Unify policy across shape adapters.** The `SoftMeshShape` branch
   at `dart_backend_bridge.cpp:349` already returns `nullptr` silently —
   add the same `DART_WARN_ONCE` to it. Same condition → same behavior
   AND same observable signal. No silent divergence between shape
   types.
3. **Introduce a clean invariant predicate.** Add
   `ConvexMeshShape::hasValidMesh() const` and
   `SoftMeshShape::hasValidMesh() const` returning false when mesh
   pointer is null OR has no vertices/triangles. `adaptShape()` calls
   that predicate so the "what counts as empty" logic is owned by the
   shape class, not the adapter. This is the long-term invariant: the
   shape itself answers "am I collidable."
4. **Document FCL's 0.1-sphere fallback as legacy.** Edit
   `dart/collision/fcl/reference/fcl_collision_detector.cpp:1187-1218`
   with a `// TODO(DART 8): align with native adapter — remove silent
sphere substitution; return nullptr and emit DART_WARN_ONCE.`
   Also note this divergence in `05-downstream-migration.md` so
   downstream users can see the planned removal.
5. **Fix the new tests accordingly.**
   - `EmptyConvexMeshUsesFallbackSphere` — rename to
     `EmptyConvexMeshIsNotCollidable` and assert no contacts in both
     positions (the previous 0.12 case will now be non-colliding too,
     because there is no fallback). Also assert that `DART_WARN_ONCE`
     fires once across the two calls if there's a hook to test that.
   - `SphereMeshCollisionWorksInBothOrders` — either rename to
     `SphereMeshCollisionDetectsContact` (drop the order claim that the
     test doesn't actually exercise) OR rewrite to call the narrow-phase
     dispatcher directly with both orderings and assert flipped contact
     normal (matching the new box-box swap test at
     `test_box_box.cpp:427-436`). Pick rename — it's the smaller delta
     and the order-handling claim is already covered by the box-box
     swap test.
   - `ConvexMeshCollisionUsesNativeConvexGeometry` — keep as-is; this
     one exercises a real path and the math is correct.
6. **Apply R4-6 nits.** Remove the now-unused magic `0.1` from
   `dart_backend_bridge.cpp` entirely (nullptr path needs no constant).
   For the FCL adapter, leave the `0.1` as a clearly-marked legacy
   constant `kLegacyEmptyMeshFallbackRadius` with a comment pointing
   at the DART 8 removal TODO.
7. **Commit ordering remains 4 sequential commits** per Q3 #10 + the
   convex-mesh slice as the fourth:
   (a) box-box fix + tests + narrow-phase JSON refresh,
   (b) OctoMap warning suppression in `voxel_grid_shape.hpp`,
   (c) doc updates (README/RESUME/03-evidence/06-audit/SUPERVISOR.md),
   (d) convex/soft-mesh nullptr-unification + FCL legacy TODO + tests.
   Each commit passes `pixi run lint` independently.

After all seven items above are done, do NOT push. Surface the result
in a Round 5 SUPERVISOR.md section so the supervisor can spot-check
before publish.

### Round 5 Local Completion Notes

Q5 policy is implemented locally:

- `ConvexMeshShape::hasValidMesh()` and `SoftMeshShape::hasValidMesh()` own the
  valid-mesh predicate for vertices plus triangle indices.
- Native `DartCollisionDetector` shape adaptation returns `nullptr` for invalid
  convex/soft mesh data and emits `DART_WARN_ONCE`, rather than creating
  placeholder collision geometry.
- FCL reference `ConvexMeshShape` adaptation keeps the legacy 0.1-radius sphere
  fallback behind `kLegacyEmptyMeshFallbackRadius` and a DART 8 TODO to align
  with native invalid-mesh behavior.
- `EmptyConvexMeshUsesFallbackSphere` is renamed to
  `EmptyConvexMeshIsNotCollidable`; the near and far cases both assert no
  contacts. `SphereMeshCollisionWorksInBothOrders` is renamed to
  `SphereMeshCollisionDetectsContact`.
- `05-downstream-migration.md`, `PR-DRAFT.md`, `07-pr-evidence-transfer.md`,
  `README.md`, `RESUME.md`, `03-evidence-gates.md`, and `CHANGELOG.md` now
  describe invalid convex/soft meshes as non-collidable on the native runtime
  path, with the FCL fallback documented as legacy reference behavior.

## Round 6 — Reference Code Belongs Under tests/ (architectural cleanup)

User direction (2026-05-15): "Reference code (FCL, ODE, Bullet) should be
only in tests/ directory, not in the dart/\*\* folder."

This is an architectural cleanup, not a behavior change. Today the layout
is:

- `dart/collision/{fcl,bullet,ode}/` — top-level public C++ compatibility
  facade headers + small `compat/` shim headers. These route to
  `DartCollisionDetector` and are required for source-compatibility of
  downstream code like gz-physics. **STAY in `dart/`**.
- `dart/collision/{fcl,bullet,ode}/reference/` — full historical
  FCL/Bullet/ODE collision implementation: `*_collision_detector.{hpp,cpp}`,
  `*_collision_group.{hpp,cpp}`, `*_collision_object.{hpp,cpp}`,
  `*_collision_shape.{hpp,cpp}`, `*_types.{hpp,cpp}`, plus engine-specific
  `detail/`. These are consumed ONLY by test/benchmark targets via
  `createReference()` and the `dart-collision-reference-{fcl,bullet,ode}`
  CMake targets. **MOVE under `tests/`** because they have no runtime
  callers inside `dart/`.

Confirmed today (no production caller of `createReference`):

- All `createReference()` call sites live under `tests/`
  (`tests/integration/collision/*`, `tests/unit/collision/*`,
  `tests/benchmark/collision/comparative/*`). Re-run
  `git grep -lE "createReference|reference/(fcl|bullet|ode)_collision" -- dart/ python/`
  before acting to confirm nothing under `dart/` consumes the reference
  trees.

### Target layout

```
tests/reference_collision/
  fcl/
    fcl_collision_detector.{hpp,cpp}
    fcl_collision_group.{hpp,cpp}
    fcl_collision_object.{hpp,cpp}
    collision_shapes.hpp
    backward_compatibility.{hpp,cpp}
    fcl_types.{hpp,cpp}
    tri_tri_intersection_test.hpp
    CMakeLists.txt          # builds test-only `dart-test-reference-fcl`
  bullet/
    bullet_collision_detector.{hpp,cpp}
    bullet_collision_group.{hpp,cpp}
    bullet_collision_object.{hpp,cpp}
    bullet_collision_shape.{hpp,cpp}
    bullet_include.hpp
    bullet_types.{hpp,cpp}
    detail/...
    CMakeLists.txt          # builds test-only `dart-test-reference-bullet`
  ode/
    ode_collision_detector.{hpp,cpp}
    ode_collision_group.{hpp,cpp}
    ode_collision_object.{hpp,cpp}
    ode_types.{hpp,cpp}
    detail/...
    CMakeLists.txt          # builds test-only `dart-test-reference-ode`
  README.md                  # explains why this lives under tests/, what
                             # downstream contract still survives in dart/
                             # via the compat facades, and how to extend
                             # reference engines for new comparison tests
  CMakeLists.txt             # add_subdirectory(fcl|bullet|ode) gated by
                             # DART_BUILD_COLLISION_REFERENCE_TESTS or
                             # DART_BUILD_COLLISION_REFERENCE_BENCHMARKS
```

After the move, `dart/collision/{fcl,bullet,ode}/reference/` directories
are DELETED. The top-level `dart/collision/{fcl,bullet,ode}/*.hpp` facade
headers and `compat/*.hpp` shims remain in place — those are the
downstream source-compatibility surface for gz-physics and others.

### Why this is right (long-term, scalable)

1. **`dart/` is the runtime library; `tests/` is test infrastructure.**
   Reference engines are correctness/perf comparison fixtures. They have
   no business shipping in the production library tree.
2. **Removes the "are these runtime backends?" ambiguity.** Today the
   `dart/collision/fcl/reference/` path could be misread as "an FCL
   backend exists inside dart/". Moving to `tests/reference_collision/`
   makes the test-only intent self-documenting.
3. **Shrinks the public install surface.** Reference headers will no
   longer be installable from `dart/`; only test binaries will see them.
   No downstream needs to include them.
4. **Cleaner DART 8 path.** When the reference comparison harness is
   eventually retired (post-DART 8), deleting `tests/reference_collision/`
   is one `rm -rf` instead of touching the runtime tree.

### Concrete migration plan for Codex

Treat this as a sixth commit landing AFTER the box-box / OctoMap / doc /
convex-mesh stack from Rounds 3–5. Single commit, mechanical move.

1. **Confirm no production caller** with the grep above. If anything pops
   up, fix it FIRST (likely a stale include).
2. **`git mv` the source trees** (preserves blame):
   - `git mv dart/collision/fcl/reference tests/reference_collision/fcl`
   - `git mv dart/collision/bullet/reference tests/reference_collision/bullet`
   - `git mv dart/collision/ode/reference tests/reference_collision/ode`
3. **Move + rewrite the per-engine `CMakeLists.txt`** out of
   `dart/collision/{fcl,bullet,ode}/CMakeLists.txt` and into the
   matching `tests/reference_collision/{fcl,bullet,ode}/CMakeLists.txt`.
   Rename the targets:
   - `dart-collision-reference-fcl` → `dart-test-reference-fcl`
   - `dart-collision-reference-bullet` → `dart-test-reference-bullet`
   - `dart-collision-reference-ode` → `dart-test-reference-ode`
     Targets stay gated by the existing
     `DART_BUILD_COLLISION_REFERENCE_TESTS` /
     `DART_BUILD_COLLISION_REFERENCE_BENCHMARKS` options. Update every
     `target_link_libraries(... dart-collision-reference-*)` line in
     `tests/unit/collision/CMakeLists.txt`,
     `tests/integration/collision/CMakeLists.txt`,
     `tests/benchmark/collision/CMakeLists.txt`, and any other test
     CMakeLists referencing the old name.
4. **Drop the installable `collision-reference-*` package components.**
   Today the per-engine CMakeLists install those components under
   `lib/cmake/...`. After the move these are TEST artifacts and should
   NOT install. Remove the `install(...)` blocks for the reference
   targets entirely so they don't ship from `dart/` or `tests/`.
   Update `05-downstream-migration.md` to record this removal as part
   of the reference-engine cleanup — downstream users were never
   intended to link those components and the existing migration doc
   already labels them reference-only.
5. **Update include paths in all reference-consuming tests.** Every
   include of the form
   `#include <dart/collision/{fcl,bullet,ode}/reference/...>` must
   become an include rooted at the new test-only location. See Q6 below
   for the exact path convention.
6. **Update the `check-collision-runtime-isolation` lint script**
   (`scripts/check_collision_runtime_isolation.py`) so its allowed-path
   list moves from `dart/collision/**/reference/` to
   `tests/reference_collision/`. The runtime guard now becomes
   "nothing under `dart/collision/` (except `compat/` facades) may
   include reference engine headers" — which is even stricter than
   today.
7. **Update the `audit_collision_compat_facades` script** to reflect
   the new test-only location for reference adapters.
8. **Update `README.md`, `06-completion-audit.md`,
   `05-downstream-migration.md`,** and `docs/onboarding/architecture.md`
   to describe the new layout. The "Native Detector + Compatibility
   Facades + Optional Reference Engines (now under tests/)" framing is
   the architecture story for DART 7.
9. **Refresh evidence.** Run `pixi run lint`, `pixi run test-all`,
   `pixi run -e collision-reference test-cpp` (and the
   `bm-collision-check` benchmark guard) on the post-move tree. Record
   in `03-evidence-gates.md` as a new dated entry, and write a Round 7
   spot-check section in this SUPERVISOR.md.

### Anti-goals for this slice

- Do NOT touch the top-level `dart/collision/{fcl,bullet,ode}/` facade
  headers or the `compat/` shims. Those are the downstream
  source-compatibility surface and stay in `dart/`.
- Do NOT change runtime behavior. This is a pure file move + CMake
  rename + include-path update. If any test fails after the move,
  diagnose and fix the build wiring rather than reverting the move.
- Do NOT bundle this with the box-box / OctoMap / doc / convex-mesh
  commits. It is the SIXTH commit in the stack and lands AFTER all five
  prior commits are clean.
- Do NOT push the new layout to `origin/feature/new_coll` without
  explicit user/maintainer approval.

### Open Question Q6 for User — ANSWERED

**Q6 (test-only reference engine include path):** After the move, test
sources need to include reference headers. Two options:

- **A — Relative includes within `tests/`.**
  `#include "../../reference_collision/fcl/fcl_collision_detector.hpp"`.
  Cheapest, no install metadata at all.
- **B — Pretend-installable test include root.**
  `target_include_directories(dart-test-reference-fcl PUBLIC tests/)` so
  callers can `#include <reference_collision/fcl/fcl_collision_detector.hpp>`.
  Cleaner at the call site, slightly more CMake.

Supervisor recommends **B** because it keeps test includes angle-bracket
style consistent with the rest of the codebase and avoids fragile
relative paths.

**Q6 ANSWER (user, 2026-05-15):** Option B — long-term, scalable,
proper approach. Codex MUST:

1. **Set the test include root explicitly per reference target.**
   `target_include_directories(dart-test-reference-fcl PUBLIC
${CMAKE_SOURCE_DIR}/tests)` (and same for bullet/ode). Use
   `$<BUILD_INTERFACE:...>` so the include path is build-only — these
   targets must NEVER export an INSTALL_INTERFACE include path, because
   reference engines do not ship.
2. **Use the `dart/test/reference_collision/...` include namespace, not
   bare `reference_collision/...`.** Rationale: `tests/` is the include
   root, and the natural angle-bracket form rooted at that directory is
   `#include <reference_collision/fcl/fcl_collision_detector.hpp>`.
   That's fine for in-tree builds but reads as a top-level namespace
   collision risk for any downstream that ever points its include path
   at `tests/`. Wrap with one more directory level under
   `tests/dart/test/` so every include is unambiguously DART-namespaced:
   `tests/dart/test/reference_collision/fcl/fcl_collision_detector.hpp`,
   and callers write
   `#include <dart/test/reference_collision/fcl/fcl_collision_detector.hpp>`.
   Set the include root to `${CMAKE_SOURCE_DIR}/tests`. This matches
   the rest of the codebase's `<dart/...>` convention exactly.
3. **Move with the path-corrected layout.** Update the migration plan
   from `tests/reference_collision/{fcl,bullet,ode}/...` to
   `tests/dart/test/reference_collision/{fcl,bullet,ode}/...`. Update
   every step in the Round 6 plan accordingly:
   - `git mv dart/collision/fcl/reference
tests/dart/test/reference_collision/fcl`
   - (same for bullet, ode)
   - Include rewrite:
     `#include <dart/collision/fcl/reference/...>` →
     `#include <dart/test/reference_collision/fcl/...>`.
4. **README under the new directory** lives at
   `tests/dart/test/reference_collision/README.md` and explains:
   reference engines are test infrastructure only; no install metadata;
   no INSTALL_INTERFACE; new comparison engines should be added under
   the same namespace; the `dart-test-reference-*` targets are gated by
   `DART_BUILD_COLLISION_REFERENCE_TESTS` /
   `DART_BUILD_COLLISION_REFERENCE_BENCHMARKS`.
5. **Lint guard update.** The runtime isolation script must now reject
   any include of `<dart/test/reference_collision/...>` from outside
   `tests/`. The existing guard at
   `scripts/check_collision_runtime_isolation.py` already enforces a
   similar rule for `dart/collision/*/reference/...`; extend it to
   cover the new path AND keep the old path forbidden so a stale
   reintroduction is rejected.
6. **Onboarding doc update.**
   `docs/onboarding/architecture.md` needs a one-paragraph note saying
   reference engines now live under `tests/dart/test/reference_collision/`
   and are excluded from the install surface.

Apply Q6 alongside Round 6 as the sixth commit of the stack; do not
push without explicit user/maintainer approval. After the move, write
the Round 7 spot-check section.

## Round 7 — Box-Box Rest Stability Regression (NEW, user-observed)

User direction (2026-05-15, with screenshot from `hello_world`): a box
dropped from the air does NOT settle on a face. Instead it settles
balanced on a single edge, tilted ~30°, with no toppling-to-face
behavior. This is a visible default-detector quality regression, not a
test-pass regression.

### Root cause (Claude direct inspection)

User reports a TWO-stage failure: (1) box settles balanced on an edge,
not a face, (2) after ~15 s the box rotates and tunnels through the
ground entirely. These are caused by THREE compounding bugs in
`dart/collision/native/narrow_phase/box_box.cpp`, not one.

**Root cause 1 — Single-point fallback for rotated face contact**
(`box_box.cpp:168-259`). `addFacePatchContacts` requires
`incidentFaceAlignment >= 1.0 - 1e-6` (line 198). For a rotated box on
a flat ground, the incident face's normal is NOT axis-aligned with the
contact normal — `cos(rotation_angle) < 1.0` — so the check fails and
`collideBoxes` falls back to a single contact point at line 367. A
single point has zero torque arm; the solver cannot resist further
rotation, so the box "balances" on a knife edge. This is Stage 1 of
the user-visible failure.

**Root cause 2 — Unnormalized cross-axis SAT comparison**
(`box_box.cpp:317-332`). The SAT loop tries 15 separating axes: 3 face
normals on box1, 3 on box2, and 9 cross products `axes1[i] x axes2[j]`.
The cross-axis loop has TWO problems:

- The `crossAxis` is NOT pre-normalized OR length-checked. When two
  box axes are nearly parallel (which happens continuously as a
  rotated cube wobbles toward face alignment), `axes1[i].cross(axes2[j])`
  shrinks toward zero. `testAxis` normalizes the axis internally for
  the normal but the projected `overlap` it compares against
  `best.penetration` (line 93) is computed on the un-rescaled axis,
  so a near-degenerate cross-axis can artificially "win" SAT with a
  spurious small overlap.
- A proper SAT implementation skips cross-axes whose length squared is
  below ~1e-6. The current code doesn't, so as the cube rotates toward
  axis-alignment the chosen separating axis flickers between cross-axes
  and face axes. The resulting normal flickers direction frame-to-frame,
  causing manifold cache thrash, warm-start invalidation, and
  inconsistent impulse direction. Over many frames the cube's
  position drifts downward.

**Root cause 3 — SAT picks wrong axis under deep penetration**
(`box_box.cpp:281-332` + `testAxis` at lines 66-100). SAT chooses the
axis of MINIMUM overlap as the separating axis. Once Stage-2 drift has
pushed a box vertex below the ground surface deeper than the box's own
half-height, the minimum-overlap axis flips from "world +Z" (push box
up) to one of the box's OWN face normals (push box sideways or DOWN
into the ground, escaping through the closer face). The reported
penetration then becomes the box's own thickness instead of the ground
penetration, and the contact normal points along that wrong axis. The
solver dutifully resolves "penetration" by pushing the box deeper into
the ground until the pair is no longer detected as colliding. That's
Stage 2 — tunneling through.

The Round-3 `computeContactPoint` rewrite addressed neither RC1, RC2,
nor RC3. The Round-3 unit tests pass because they only check static
"box does not fall through one timestep" (vertex_z >= ground_top -
0.005), not steady-state stability or 15-second behavior.

### Why this matters

This is the user-visible default-detector quality bar for the
native-collision-default rollout. If gz-physics or any user app drops
a box on a floor and it balances on an edge, the perception is "DART
native is unstable" regardless of what the test suite says. Reference
engines (FCL, Bullet, ODE) all produce multi-contact manifolds for
rotated-box-on-flat-ground via Sutherland-Hodgman-style face/face
clipping — DART must match.

### Correct behavior bar

For a small box rotated 30° about an arbitrary axis, dropped on a
large flat ground:

1. The narrow-phase must emit **at least 3 contact points** spanning
   the actual contact polygon (the rotated-box face's intersection
   with the ground face, clipped by both faces' edges).
2. The constraint solver, given those 3+ contacts, must allow the box
   to topple from edge-rest to face-rest over a short simulation
   window (< 1 s of sim time for a 0.3 m cube falling from 1 m).
3. Steady-state should be face-rest with the box's lowest face flush
   on the ground (within 1e-3 m of `groundTop + halfExtent` for axis
   alignment).

### Algorithm path

The correct fix has THREE parts, addressing each root cause:

**Fix 1 — Sutherland-Hodgman face/face polygon clipping** (addresses
RC1). NOT just relaxing the `1.0 - 1e-6` threshold:

1. Identify the reference face (the face perpendicular to the SAT
   best-axis on the reference box). Today this is correctly chosen at
   `box_box.cpp:189-192`.
2. Identify the incident face — for the rotated box, this is the
   face whose outward normal is MOST anti-aligned with the contact
   normal, NOT the face that's perpendicular to one of the incident
   box's local axes. Today the code's "incident face alignment" check
   conflates these.
3. Clip the incident face polygon against the reference face's 4 edge
   half-planes (Sutherland-Hodgman in 2D, projected into the reference
   face's tangent plane). Result: 0-8 clipped polygon vertices.
4. Project each clipped vertex back onto the reference face plane;
   keep only points with penetration > 0 (i.e., on the wrong side of
   the reference face). Each kept point becomes a contact point with
   normal = reference face normal and depth = signed distance.
5. If the clipped polygon has more than `maxNumContacts - contactsBefore`
   points, reduce via a "best 4 contacts" heuristic
   (Bullet's `btPolyhedralContactClipping::reduceContacts`):
   keep the deepest point + the 3 points farthest from it in
   projection.

**Fix 2 — Skip degenerate cross-axes in SAT** (addresses RC2). In
`box_box.cpp:317-332`:

```cpp
for (int i = 0; i < 3; ++i) {
  for (int j = 0; j < 3; ++j) {
    Eigen::Vector3d crossAxis = axes1[i].cross(axes2[j]);
    const double lenSq = crossAxis.squaredNorm();
    if (lenSq < kCrossAxisDegenerateEpsilon) {  // e.g. 1e-12
      continue;  // axes nearly parallel — face-axis SAT already covers
    }
    crossAxis /= std::sqrt(lenSq);  // normalize BEFORE testAxis
    if (!testAxis(crossAxis, ...)) return false;
  }
}
```

This is the classical SAT-for-boxes recipe (Gottschalk's "Collision
Queries Using Oriented Bounding Boxes", §3.2). Without it, projected
overlaps on a near-zero axis are comparable in magnitude to projected
overlaps on a face axis, even though only the face axis is
geometrically meaningful.

**Fix 3 — Bias SAT axis preference toward face axes under near-tie**
(addresses RC3 and tightens RC2). When two SAT axes report
near-equal overlap (within ~1% relative or 1e-4 absolute), prefer
the face axis over the cross-axis. This kills SAT axis flicker as the
box rotates toward face alignment, and prevents the
deep-penetration normal-flip that causes tunneling.

```cpp
// In testAxis, replace `overlap < best.penetration` with:
const bool isFaceAxis = (axisIndex < 6);
const bool isBestFaceAxis = (best.axisIndex >= 0 && best.axisIndex < 6);
const double tieThreshold = 1e-4;
const bool clearlyBetter = (best.penetration - overlap) > tieThreshold;
const bool preferOverTie =
    !isBestFaceAxis && isFaceAxis &&
    std::abs(overlap - best.penetration) <= tieThreshold;
if (clearlyBetter || preferOverTie) {
  best.penetration = overlap;
  best.axis = normalizedAxis;
  best.axisIndex = axisIndex;
}
```

Combined, these three fixes match how Bullet's `btBoxBoxDetector` and
ODE's `dBoxBox` handle the rotated-box-on-ground case. FCL routes
through GJK+EPA which falls back to similar polygon clipping for box
pairs.

### Concrete acceptance criteria for Codex

1. **New regression test** in `tests/unit/collision/native/test_box_box.cpp`:
   `BoxBox.RotatedBoxOnFlatGroundEmitsFacePatch` — rotated small box
   (any non-axis-aligned rotation) resting on a large ground box;
   assert `result.numContacts() >= 3` AND
   the contact points span a polygon area > some threshold
   (e.g., bounding-box area of contact positions >
   `(0.5 * box.halfExtent)^2`).
2. **New default-world test** in
   `tests/unit/simulation/test_world.cpp`:
   `WorldTests.DefaultNativeRotatedBoxSettlesOnFace` — drop a rotated
   0.3 m box from 1 m onto a large ground over 3000 steps. Assert
   that after 2 s of simulation, the box's orientation has at least
   one local axis aligned with world ±Z to within 5° (face-rest
   condition).
3. **NEW long-horizon stability test** in
   `tests/unit/simulation/test_world.cpp`:
   `WorldTests.DefaultNativeRotatedBoxStaysOnGround15s` — same scene
   as #2 but run 15 000 steps (15 s sim). Assert at every 100-step
   sample point that:
   - box vertex_z >= ground_top - 0.005 (no tunneling);
   - box center_z stays within [ground_top + halfExtent - 0.02,
     ground_top + halfExtent * 1.74 + 0.02] (no drift; the upper
     bound covers the edge-balanced transient since edge-rest height
     is `halfExtent * sqrt(3)`);
   - box center_z is monotonically non-increasing within the rest
     phase (after step 2000) — no oscillation upward.
     This is the test that catches RC2/RC3 directly.
4. **Cross-engine parity check** in benchmark/integration: confirm
   that for the same rotated-box-on-ground scenario, native emits a
   similar number of contacts as Bullet/ODE (the reference engines'
   call sites at
   `tests/integration/collision/test_capsule_ground_contact.cpp` or a
   new equivalent test for box-on-ground).
5. **SAT-axis-flicker probe test**: a rotated box held statically just
   above a ground (no gravity); call `collideBoxes` 100 times with
   tiny rotational perturbations (~0.001 rad). Assert that
   `best.axisIndex` returned by SAT does not flip between face-axis
   and cross-axis indices across the sweep. This is the unit test for
   Fix 2 + Fix 3.
6. The existing `BoxBox.Determinism` (`test_box_box.cpp:411`) and the
   Round-3 `RotatedSmallBoxOnLargeGroundHasLocalContactPoint` must
   continue to pass.
7. Native vs reference perf for `BM_NarrowPhase_BoxBox_Native` may
   regress slightly (face clipping is more work than a single contact
   point). Acceptable up to 2-3× the current ~544 ns; surface the
   actual number in the Round-8 spot-check.

### Anti-goals

- Do NOT just relax the `1.0 - 1e-6` threshold to `1.0 - 0.1` or
  similar. That makes the face-patch path emit a 4-corner manifold
  using the WRONG polygon (the incident box's own face corners,
  unclipped against the reference face). The result will be contacts
  outside the actual overlap region — same root cause as the original
  box-box regression the Round-3 fix already addressed.
- Do NOT just bump the contact count by adding edge/vertex midpoints
  to the existing single-contact result. The contacts must lie inside
  the actual overlap polygon, with correct depths per point, or the
  constraint solver will compute wrong torques.
- Do NOT bundle this fix with the Round-6 reference-code move. Round 7
  is a separate algorithm-change slice with its own
  test/perf/regression evidence requirements.

### Open Question Q7 for User — ANSWERED

**Q7 (box-box face/face clipping):** This is a real default-detector
quality bug. Two scope choices:

- **A — Fix now, before the reference-code move (Round 6).** Bumps Q7
  ahead of Q6 in the commit stack. Justification: the visible quality
  issue is the user-facing blocker; reference-code reorg can wait.
- **B — Fix after the reference-code move.** Keeps the stack linear:
  box-box → octomap → docs → convex-mesh → tests/ move → face-clipping.
  Justification: each prior commit is already done locally; Round 7 is
  the heaviest algorithm work and benefits from a clean tree.

Supervisor recommends **A** because the visible quality bar dominates
the architectural cleanup priority. The reference-code move is mostly
mechanical and can land after a real algorithm fix.

Either way, Round 7 is its own commit, lands on the existing branch,
and does not push without user approval. Do NOT start Round 7
implementation until Q7 is answered.

**Q7 ANSWER (user, 2026-05-15):** Option A — fix the box-box quality
bug first, then do the Round-6 tests/ move after. Apply the cleanest,
most scalable implementation, not a minimum-diff conservative patch.
gz-physics compatibility is the one hard constraint (the box-box fix
must not break the gz-physics build / test surface).

Codex MUST do the following for Round 7, in this order:

1. **Refactor `box_box.cpp` toward the clean target shape FIRST**
   before adding face-clipping. The current file mixes SAT,
   contact-point heuristics, and a single-purpose face-patch helper;
   the clean shape is:
   - `box_box/sat.{hpp,cpp}` — SAT axis search (15 axes, with
     degenerate cross-axis skipping per Fix 2 and face-axis tie-bias
     per Fix 3). One pure function returning `SatResult`.
   - `box_box/face_clip.{hpp,cpp}` — Sutherland-Hodgman polygon
     clipping per Fix 1. Pure function: given two box transforms +
     `SatResult`, return up to 8 clipped contact points.
   - `box_box/contact_reduction.{hpp,cpp}` — Bullet-style "best 4
     contacts" reducer. Pure function: given N contact points + a
     `maxNumContacts` budget, return the reduced set.
   - `box_box.cpp` — orchestration only: SAT → clip → reduce → emit.
     This splits a 389-line file into 4 focused files (~100 lines each).
     Cleaner, testable in isolation, scalable for the future
     per-pair specializations the next perf wave will need.
2. **Apply Fix 1 + Fix 2 + Fix 3 as documented in Round 7 above** in
   the new files. The single-point fallback at current
   `box_box.cpp:367` goes away entirely — there is no scenario where
   a box-box overlap should produce only one contact. If the
   reduced-contact set is empty, the pair is not colliding and we
   return `false`; if non-empty, we always emit ≥1 contact (from the
   reducer's output), most of the time ≥3.
3. **Write all 7 acceptance-criteria tests from Round 7 BEFORE the
   implementation lands** (TDD-style). Specifically: criteria #1, #2,
   #3 (15-second long-horizon), #5 (SAT-axis-flicker probe), and
   #6 (Determinism + Round-3 regression carry-over). Each test fails
   first against the current implementation, then passes after the
   new implementation lands. The test names from Round 7 stay as
   specified; do not rename.
4. **Cross-engine parity (#4):** add a new
   `tests/integration/collision/test_box_ground_contact_parity.cpp`
   that runs the same rotated-box-on-ground scene through native +
   FCL + Bullet + ODE reference detectors and asserts the contact
   count is within ±1 of Bullet's count (Bullet is the reference for
   "correct" box-box behavior). This is the gz-physics-style
   parity gate.
5. **Validate the gz-physics hard constraint.** Run
   `pixi run -e gazebo test-gz` after the Round-7 implementation
   lands. If anything fails, do NOT silence it — surface the failure
   in a Round-8 spot-check section with the actual gz-physics test
   name and output. Most likely the gz tests will benefit from
   better box-box behavior; if any depend on the OLD single-point
   contact (unlikely but possible), document the divergence and ask
   the user before changing gz expectations.
6. **Re-record the raw narrow-phase benchmark JSON** with the new
   box-box path. The acceptance bar is ≤ 3× current 544 ns (≤ 1632
   ns). If the actual number is worse, surface in the Round-8
   spot-check and ask before continuing — but do not pre-optimize.
   The clean-architecture-first principle (see user durable
   guidance) means correctness > perf during this slice.
7. **Update `08-pair-coverage.md`** Algorithm Map and Performance
   tables with the new BoxBox numbers and the new
   `box_box/{sat,face_clip,contact_reduction}.cpp` source citations.
8. **Commit ordering** updated:
   (a) box-box fix from Round 3 [DONE],
   (b) OctoMap warning [DONE],
   (c) doc updates [DONE],
   (d) convex-mesh policy [DONE],
   (e) **Round 7: box-box face-clip refactor + SAT fix + tests + JSON**
   [THIS SLICE],
   (f) Round 6: tests/ reference move [AFTER Round 7],
   (g) Round 4: per-pair bench parity (Q4) [AFTER Round 6].

After items 1-7 are done, do NOT push. Write Round 8 spot-check in
this SUPERVISOR.md so the supervisor can verify the gz-physics
hard-constraint result, the contact-count parity vs Bullet, the
perf delta, and that the new `box_box/` subdirectory layout matches
the target shape.

### Anti-goals for Round 7

- Do NOT keep the single-point fallback "as a fast path for
  axis-aligned simple cases". The clean architecture is one path:
  always SAT → clip → reduce → emit. Specializations come in the
  perf wave, gated by benchmark evidence, not preemptively.
- Do NOT preserve the current `addFacePatchContacts` function shape.
  The whole helper is replaced by `face_clip.cpp` + `contact_reduction.cpp`.
- Do NOT pre-optimize the SAT loop with SIMD or batched dot products.
  Clean architecture first; perf wave after.
- Do NOT skip writing the long-horizon (15 s) test on the grounds that
  "it's slow." That test is the load-bearing acceptance criterion for
  this slice — if it slows the suite, gate it behind a CTest label
  (`-L collision-native-stability`) that's part of `test-all` but
  excluded from `test-unit` for fast inner-loop dev iteration.

### Durable user preference (recorded 2026-05-15)

For all future decisions on this branch: prefer clean / scalable /
long-term-correct solutions over minimum-diff conservative patches.
gz-physics compatibility is the one hard backward-compat constraint;
inside that boundary, take the clean path. This applies to Round 7,
Round 6 (when it lands), Q4 implementation, and any future open
question that surfaces during the supervisor session.

## Round 8 — Test & Benchmark Coverage Matrix (TDD)

User direction (2026-05-15, with TDD-mode active): "Review widely and
deeply the tests/benchmarks of the Bullet, ODE, and FCL codebase, list
them all up, and restructure/reorganize so that DART collision (native)
tests/benchmarks have the superset of them ... so that eventually DART
native collision is fully harnessed the superset of tests/benchmarks,
which leading to feature complete and performance beating all other
libraries."

A new tracker `09-test-coverage-matrix.md` enumerates the superset and
DART's current status. Categories were derived from a cross-survey of
upstream Bullet, ODE, and FCL test+benchmark suites but the tracker
does NOT attribute categories to specific upstreams — DART owns the
taxonomy as its own quality bar.

**Current state per the new matrix:**

- 112 DONE / 8 PARTIAL / 74 GAP rows across 194 total.
- DART native is at ~58% of the proposed superset.
- Gap concentration: less-common shape pairs against
  capsule/cylinder/mesh/convex/sdf/compound, algorithm-isolation tests
  for SAT internals, long-horizon stability scenes, per-engine raw
  narrow-phase parity benchmarks.

**Codex MUST integrate this matrix into the TDD workflow.** Round 7
(box-box face clipping) is the first slice that follows the workflow
end-to-end: tests fail first, implementation lands, tests pass, matrix
flips DONE.

### Codex follow-up plan (no user question)

For all future feature/regression slices on this branch, follow this
order:

1. **Add the row to `09-test-coverage-matrix.md` as `GAP`** with a
   proposed codename in the existing kebab-case style. If a similar
   row already exists, use its codename and update the Status column.
2. **Write the failing test under that codename** (use the exact
   codename in the test name, e.g.
   `TEST(BoxBox, boxbox_edge_edge)` or
   `TEST(WorldTests, boxbox_15s_no_tunneling)`).
3. **Run the test** and confirm it fails for the geometrically correct
   reason (not a compile error, not a missing fixture). Cite the
   failure output in the commit message.
4. **Implement the smallest scope that makes the test pass.**
5. **Re-run and confirm green.**
6. **Flip the matrix row to `DONE`**, refresh the summary counters at
   the bottom of `09-test-coverage-matrix.md`, and re-record the
   benchmark JSON if the row is a benchmark.

### Next-priorities ordering (Codex must follow this)

The "Next Priorities" section at the bottom of
`09-test-coverage-matrix.md` is the load-bearing ordering. Codex MUST
work through it top-to-bottom:

1. Round 7 box-box GAPs (already steered separately).
2. Q4 bench parity (already steered separately).
3. Round 6 tests/ reference move (already steered separately).
4. Capsule × {Mesh, Convex, Compound} — 3 GAP rows.
5. Cylinder × {Mesh, Convex, Compound, Sdf} — 4 GAP rows.
6. SAT algorithm-isolation suite.
7. Long-horizon stability scenes (`stacked_boxes_n10/n100`,
   `mixed_primitive_stack` unit version, `ragdoll_capsule_pile`,
   `thin_box_no_tunneling`).
8. Scope-decision items (auto-diff, float/double parity, k-DOP / OBB
   / RSS BVH variants) — surface as future Q before adding rows.

Cross-cutting: when a benchmark row lands, it MUST go through the
apples-to-apples adapter path (see Q4 ANSWER) so the perf comparison
to FCL/Bullet/ODE is fair.

### Open Question Q8 for User — Stretch scope-decision items

**Q8 (stretch scope):** `09-test-coverage-matrix.md` lists three
families that are real upstream collision-test categories but may or
may not belong in DART's superset:

- **Auto-diff narrow-phase** — run pair tests under
  `Eigen::AutoDiffScalar` to verify gradient correctness. Adds CI cost
  but unlocks differentiable physics. Useful if DART positions for
  ML / optimization workloads.
- **Float / double parity** — DART is double-only today. Adding
  `float` instantiation costs ~2× test runtime and ~2× narrow-phase
  template instantiations. Useful for embedded / mobile / GPU paths.
- **k-DOP / OBB / RSS BVH variants** — DART uses AABB BVH today.
  Other libraries support tighter bounding volumes for mesh-heavy
  scenes. ~5-15% perf win on mesh stress, but adds 3-5 new code paths
  to maintain.

Pick one of:

- **A — In-scope, all three.** Maximizes superset coverage; biggest
  maintenance surface.
- **B — In-scope auto-diff + float/double, defer BV variants.** Most
  user-visible value; defers the algorithm-implementation work.
- **C — Defer all three until a real user need surfaces.** Smallest
  scope; keeps the matrix's "scope-decision items" honest.

Supervisor recommends **C** because (1) the existing 74 GAP rows
already cover concrete user-visible quality issues (box-box stability,
long-horizon tests, missing shape pairs) and (2) the cleanest
long-term path is to land those gaps first, then revisit stretch
items with real perf/feature evidence rather than speculative adds.
This is consistent with the durable preference: clean + scalable, but
not pre-built.

Do NOT start any Q8-scoped work until Q8 is answered.

## Round 8 Spot-Check - Round 7 Box-Box Slice (2026-05-15)

Codex completed a focused Round 7 raw box-box pass without pushing or
opening a PR.

Implementation shape:

- `box_box.cpp` is now orchestration only: it builds box data, runs SAT,
  computes contact candidates, reduces the contact patch, and emits the
  manifold.
- The raw implementation is split into
  `box_box/{sat,face_clip,contact_reduction}.{hpp,cpp}` so SAT axis
  selection, face clipping, and patch reduction are independently testable.
- The path preserves the existing `CollisionDetector` surface, factory keys,
  and native-backed compatibility facades.

Fail-first evidence:

- `BoxBox.RotatedBoxOnFlatGroundEmitsFacePatch` failed on the old code with a
  single contact where the expected face patch required at least 3 contacts.
- `WorldTests.DefaultNativeRotatedBoxSettlesOnFace` failed on the old code,
  reproducing the user-visible rotated box/ground instability.

Validation evidence:

- Default focused tests:
  `ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^(test_box_box|UNIT_simulation_World)$'` passed 2/2.
- Reference parity test:
  `pixi run -e collision-reference -- ctest --test-dir
build/collision-reference/cpp/Release --output-on-failure -R
'^INTEGRATION_collision_BoxGroundContactParity$'` passed.
- gz-physics downstream gate:
  `pixi run -e gazebo test-gz` passed 65/65 tests and built the DART plugin.
- Canonical raw narrow-phase benchmark JSON was refreshed at
  `.benchmark_results/native_collision_raw_narrow_phase.json`.
  Box-box mean timings from that run:
  - Native: 428 ns real / 425 ns CPU.
  - FCL reference: 1410 ns real / 1401 ns CPU.
  - Bullet reference: 851 ns real / 845 ns CPU.
  - ODE reference: 1520 ns real / 1510 ns CPU.
- The one-off Round 7 box-box benchmark JSON is retained at
  `.benchmark_results/native_collision_box_box_round7.json`; its native
  timing stayed within the 3x acceptance budget.

Coverage matrix update:

- `09-test-coverage-matrix.md` was updated for the new box-box face patch,
  rotated ground, 15s no-tunneling, SAT stability, and cross-backend contact
  parity coverage.
- Current matrix summary after this slice: 123 DONE, 5 PARTIAL, 66 GAP
  (~63% complete).

## Round 9 — Batch + SIMD as First-Class Collision Dimension (NEW)

User direction (2026-05-15): "Going back to collision detection,
consider (not making this one-way door) optimizing not only for
pair-wise collision detection but also batch collision detection,
fully utilizing SIMD, as well."

This is an **architectural direction**, not a slice. It must be
baked into the Round 7 box-box refactor and every subsequent
narrow-phase rewrite, so the code shape supports batch + SIMD without
a second rewrite. The instruction is explicitly "not making this
one-way door" — meaning current pair-at-a-time call sites must keep
working AND a batch path must exist alongside.

### Why this changes the Round 7 design

Round 7's clean refactor already splits `box_box.cpp` into
`box_box/{sat,face_clip,contact_reduction}.cpp`. Pair-at-a-time only.
If we land Round 7 as-currently-specified, then add batch+SIMD later,
we will rewrite the same three files twice. Instead, Round 7 must
land with a batch-aware function signature from the start, even if
the initial implementation does scalar pair-at-a-time internally.

### Critical context — world experimental ECS already has SoA layout

User direction (2026-05-15): "there is world experimental where the
data structure is memory friendly, and batch processing with the
memory friendly data structure would be perfect match."

`dart/simulation/experimental/` is an EnTT-backed Entity-Component-System
world (see `dart/simulation/experimental/world.{hpp,cpp}`,
`ecs/entity_object*.hpp`, `comps/{dynamics,joint,link,rigid_body,multi_body,frame}.hpp`,
`space/state_space.hpp`). Components live in EnTT's pool storage,
which IS the SoA layout — same component type for many entities sits
in a contiguous array. EnTT views/groups give cache-friendly
iteration over (entity, components...) tuples without scatter.

**Implication: the batch collision API should consume EnTT views/groups
directly, not invent a parallel `std::span<BoxPair>` layout.** The
canonical batch input becomes something like:

```cpp
// world experimental ECS, native collision integration:
void collideBoxesBatchEcs(
    entt::registry& registry,
    entt::view<entt::get_t<comps::RigidBody, comps::BoxShapeTag, comps::Frame>> pairs,
    std::span<CollisionResult> results,
    const CollisionOption& option);
```

The `std::span<BoxPair>` API stays as the lower-level "raw" batch
entry for tests / micro-benchmarks / non-ECS callers (e.g.
gz-physics). The ECS entry is the production path the world
experimental simulation loop calls.

This makes Round 7's box-box refactor a perfect first slice for the
ECS-batch design: scalar fallback today, SoA-friendly today, but
shaped for an EnTT view tomorrow.

### Two-tier API shape (revised)

```cpp
// Tier 1 — single-pair API (unchanged, existing call sites):
bool collideBoxes(const BoxShape& a, const Eigen::Isometry3d& tfA,
                  const BoxShape& b, const Eigen::Isometry3d& tfB,
                  CollisionResult& result,
                  const CollisionOption& option);

// Tier 2 — raw batch API (Round 7 must include):
void collideBoxesBatch(std::span<const BoxPair> pairs,
                       std::span<CollisionResult> results,
                       const CollisionOption& option);

// Tier 3 — ECS batch API (lands when native collision wires into
// world experimental; can be a later slice but the signature is
// committed to NOW so Tier 2 doesn't paint Tier 3 into a corner):
void collideBoxesBatchEcs(
    entt::registry& registry,
    auto view,                       // EnTT view of box-pair entities
    std::span<CollisionResult> results,
    const CollisionOption& option);
```

Tier 1 → calls Tier 2 with a 1-element span.
Tier 2 → SoA-friendly internal loop. SIMD-target.
Tier 3 → adapts EnTT view into the SoA layout Tier 2 wants.

Round 7 lands Tier 1 + Tier 2 (with scalar-loop body). Tier 3 lands
when native collision integrates into `dart/simulation/experimental/`
— likely Round 10 or after, gated on Q9 + a separate ECS-integration
slice.

### Architectural invariants Round 7 onward must hold

1. **Every narrow-phase pair entry point exposes Tier 1 + Tier 2
   APIs.** Tier 1 is a 1-element call into Tier 2.
2. **SAT axes, projection intervals, and face-clipping inner loops
   are written against `std::span<const Pair>` (or a moral
   equivalent).** No naked single-pair-only loops over `axes1[i]`.
3. **Data layout is SoA-ready at the Tier 2 boundary.** Inside the
   scalar implementation it's OK to use the existing AoS
   `Eigen::Isometry3d`, but the Tier 2 entry takes spans of pairs so
   a future perf-wave commit can swap to SoA without changing call
   sites.
4. **The internal SoA buffer shape Tier 2 settles on MUST match what
   an EnTT view yields when iterated.** EnTT view iteration gives
   `(entity, component_ref, component_ref, ...)` — Tier 2 should
   accept either a span of structs or a parallel-arrays layout
   compatible with EnTT view destructuring. Pick the parallel-arrays
   layout because EnTT view's pool storage IS parallel-arrays.
5. **Determinism contract scales to batch AND across tiers.** Tier 1,
   Tier 2, and Tier 3 must produce bit-identical results for the
   same logical pair set, regardless of batch ordering. This makes
   the existing `test_parallel_determinism.cpp` + new
   `*_batch_determinism` tests load-bearing. Add a third
   `*_ecs_batch_determinism_vs_raw_batch` test when Tier 3 lands.
6. **CollisionOption applies per-batch, not per-pair.** Same option
   for every pair in a batch is fine (matches real solver usage).
   If per-pair option becomes needed later, that's an additive change.
7. **Benchmark surface gets matching batch versions.** Every
   `BM_NarrowPhase_*_Native` per-pair bench gets sibling
   `BM_NarrowPhase_*_Native_Batch_N{1,10,100,1000}` AND eventually
   `BM_NarrowPhase_*_Native_EcsBatch_N{1,10,100,1000}` to measure
   the EnTT-adapter overhead vs raw batch.

### SIMD strategy (deferred to perf wave, but planned now)

When the perf wave lands, the Tier 2 batch loop body becomes the
optimization target. Strategy:

- **First win:** SoA layout for the 16 SAT axes per pair (3 face1 +
  3 face2 + 9 cross + 1 unused-but-aligned). Process 4 / 8 pairs in
  parallel with one SAT axis vector each — straightforward AVX2 /
  NEON.
- **Second win:** Polygon clipping is harder to SIMD-ize per-pair
  but trivial across pairs at the per-pair level (process N pairs'
  polygons in parallel; each pair has its own 4-edge clip).
- **Third win:** Contact-reduction "best 4 of N" is per-pair scalar;
  amortize via batch-level prefetching.
- **Fourth win (ECS-specific):** Tier 3 can pre-sort the EnTT view
  by pair-type and contact-cache bucket, so the batch handed to
  Tier 2 has even better locality than what raw call sites can
  provide. This is unique value the ECS-integration slice unlocks.
- **Tooling:** prefer `xsimd` (header-only, BSD, already a common
  DART-ecosystem dep) or `std::simd` (C++26). Reject hand-written
  intrinsics — too platform-fragmented for OSS DART.

### Apply to every future narrow-phase pair, not just box-box

Same architecture applies when Codex eventually writes
`sphere_sphere_batch`, `capsule_capsule_batch`, `convex_convex_batch`,
`mesh_mesh_batch`. Cylinder/Capsule analytic pairs especially benefit
because they're high-frequency in stacked / ragdoll scenes.

### Acceptance criteria

1. **Round 7 commit MUST land with the Tier 2 raw-batch entry function
   present** for box-box, even if the implementation body is a scalar
   loop. No "we'll add batch later" — that's the one-way-door the
   user explicitly rejected.
2. **At least one new test:** `BoxBoxBatch.DeterminismVsSinglePair`
   — assert that `collideBoxesBatch(N=100 random pairs)` produces
   bit-identical results to looping `collideBoxes` over the same 100
   pairs. Codename: `boxbox_batch_determinism_vs_single`.
3. **At least one new benchmark:** `BM_NarrowPhase_BoxBox_Native_Batch_N100`
   in `bm_narrow_phase.cpp`. Initial number will look identical to
   the per-pair number × 100; that's expected. The benchmark exists
   so future perf-wave wins are visible. Codename:
   `bench_narrow_phase_per_pair_batch`.
4. **`09-test-coverage-matrix.md` gets a new "Batch + SIMD" section**
   with codenames for: batch-determinism per pair, batch-throughput
   bench per pair, SIMD-target codepath, ECS-batch entry path
   (currently GAP — perf wave + future ECS integration slice).
5. **No SIMD work in Round 7.** Round 7 is "make the API
   batch-shaped." SIMD is the perf-wave slice. Document the
   intended SIMD strategy in `01-design.md` so future-Codex doesn't
   pick a different one.
6. **No Tier 3 ECS code in Round 7.** Tier 3 lands in a separate
   "native collision in world experimental" slice. But Round 7's
   Tier 2 signature MUST be compatible with EnTT view destructuring
   — i.e. parallel-arrays-friendly, not AoS-only.

### Open Question Q9 for User — Batch API ownership + ECS integration timing

**Q9a (batch API ownership):** The Tier 2 batch API lives somewhere.
Three options:

- **A — Per-pair file owns its batch API.** `box_box/box_box.hpp`
  exposes both `collideBoxes` and `collideBoxesBatch`. Same for
  every other pair file. Pro: locality. Con: each pair file
  re-implements the batch loop; consistency relies on convention.
- **B — Generic `NarrowPhase::collideBatch` dispatcher fans out per
  pair-type.** Public API is `NarrowPhase::collideBatch(span<Pair>)`
  which groups pairs by type and dispatches to pair-specific batch
  functions. Pro: one public entry point; users don't think about
  pair types. Con: dispatch overhead on the batch boundary.
- **C — Both: per-pair batch functions exist AND
  `NarrowPhase::collideBatch` dispatcher exists.** Public users go
  through dispatcher; advanced users / internal callers can call
  pair-specific batch directly. Pro: best of both. Con: two surfaces
  to maintain.

Supervisor recommends **C** — matches the existing
`NarrowPhase::collide(shape1, shape2, ...)` dispatcher pattern at
`narrow_phase.cpp:103-431`, just extends it to batch.

**Q9b (Tier 3 ECS integration timing):** When does native collision
wire into `dart/simulation/experimental/`?

- **A — Right after Round 7.** Tier 3 becomes Round 10; lands
  before the Round 6 tests/ move and before the Q4 bench parity.
  Risk: world experimental ECS is itself still evolving; adding a
  collision adapter on top while both are moving is hard to
  bisect.
- **B — After Round 7 + Round 6 + Q4 are all clean.** Tier 3 lands
  after the box-box quality bug is fixed, after reference code
  moved to tests/, and after benchmark parity is in place. Cleaner
  base; tier 2 batch has had time to bake.
- **C — Defer Tier 3 indefinitely until world experimental
  stabilizes AND a real downstream user asks.** Keep the Tier 2 API
  ECS-compatible (parallel-arrays-shaped) but don't write Tier 3
  yet. Lowest risk, fewest unknowns.

Supervisor recommends **B** — Round 7 stability comes first; ECS
adapter is the natural next architectural slice after that's clean.
Picking B also means the Round 7 design has to commit only to the
Tier 2 shape, not the Tier 3 implementation.

Do NOT start the Round 7 implementation phase until Q9a is answered.
Q9b can be answered later but should be answered before any
ECS-integration work begins. Round 7's test-writing phase (the 7
acceptance tests + the new batch-determinism test) can proceed in
parallel since the tests don't depend on the batch ownership choice
yet — but the implementation phase must wait for Q9a because the
function signatures depend on the chosen ownership.

## Round 10 — DART 7 vs DART 8 Compatibility Horizon (NEW)

User direction (2026-05-15): "The backward compatibility for
gz-physics in terms of dart/collision/ is only for DART 7. In DART 8,
we should aim for much cleaner API, folder structure, without
worrying about backward compatibility; also in that case, we're
willing to help gz-physics to adopt to the new API."

This is a **policy clarification**, not a new code slice. It changes
how Codex frames every future decision on this branch and updates the
durable preference.

### Updated rules

**DART 7 (current release, what `feature/new_coll` is targeting):**

- gz-physics compatibility is the one HARD constraint. Retained C++
  legacy names (`FCLCollisionDetector`, `BulletCollisionDetector`,
  `OdeCollisionDetector`, factory keys `"fcl"` / `"bullet"` /
  `"ode"` / `"experimental"`, package components `collision-fcl` /
  `collision-bullet` / `collision-ode`, installed legacy headers)
  MUST keep working as native-backed facades.
- Dartpy intentionally does NOT keep legacy detector aliases (per
  Q1 addendum). Clean Python API only.
- `[[deprecated]]` warnings on C++ facades are ON by default per
  Q1 ANSWER, gated by `DART_COLLISION_DEPRECATE_LEGACY_NAMES`.
- The reference engines stay reachable via `createReference()` +
  `collision-reference-*` targets (and after Round 6, those move
  under `tests/dart/test/reference_collision/`).

**DART 8 (next major, post-current-branch):**

- **NO backward-compat constraint in `dart/collision/`.** Codex
  should design the cleanest possible API + folder structure as if
  starting fresh.
- Legacy C++ detector names / factory keys / package components
  are REMOVED. `compat/` shims DELETED. Installed legacy headers
  REMOVED.
- User will actively help gz-physics adopt the new API in
  coordinated parallel PRs. Don't preserve gz-physics-specific
  behavior just because gz-physics currently depends on it.
- Reference engine code (whatever survives Round 6 in
  `tests/dart/test/reference_collision/`) is fair game for further
  cleanup if tests / benchmarks no longer need it.

### How this changes ongoing work

**Round 7 (box-box face-clipping):** No change — quality fix is
DART 7 work. gz-physics still expects boxbox to produce contacts.

**Round 6 (tests/ reference move):** No change — moves legacy
reference code OUT of `dart/`. DART 8 may delete it entirely
afterward.

**Round 9 (batch + SIMD + ECS Tier 3):** Significant change. The
Tier 3 ECS API can be designed without gz-physics input — DART 8
gives free hand. Tier 3 spec should reflect the cleanest possible
EnTT integration, not "what gz-physics can also call." gz-physics
will adopt whatever DART 8 ships.

**Q5 (invalid mesh → nullptr):** Continues. DART 8 will likely also
remove the FCL `kLegacyEmptyMeshFallbackRadius` since FCL itself
will be moved out of `dart/collision/` entirely in DART 6 → DART 7
→ DART 8 progression.

**Future slices not yet planned:**

- DART 8 collision API design doc (new file, `08-dart8-api-design.md`
  or rename to `DART8-DESIGN.md` later). Codex should draft this
  AFTER Round 7/6/9 land, capturing the user-visible DART 7 → 8
  migration items as the input.
- gz-physics migration PR sketch (in `05-downstream-migration.md` or
  a new section).

### Authoring guidance for Codex

When writing or modifying any file in `dart/collision/`:

1. **Mark gz-physics-compat code paths with `// TODO(DART 8): remove
— gz-physics will migrate to <clean equivalent>`.** This creates
   the inventory of DART 8 cleanup items automatically.
2. **When the cleanest design choice is blocked only by gz-physics
   compat, document the divergence** in `05-downstream-migration.md`
   AND surface as a Q in this SUPERVISOR.md, BUT make the DART 7
   choice that preserves gz-physics. The supervisor will plan the
   DART 8 cleanup separately.
3. **For new code (Round 7 box-box, Round 9 batch APIs):** design
   for DART 8. No legacy facades, no backward-compat shims.
   gz-physics will adopt.
4. **For changes to existing legacy surface
   (e.g. `dart/collision/{fcl,bullet,ode}/*.hpp` facade headers):**
   minimum-diff for DART 7. List the DART 8 cleanup as a separate
   item in `02-milestones.md`.
5. **For DART 8 design docs:** describe the target API without
   gz-physics compat baked in. The gz-physics migration is a
   COORDINATED change, not a constraint.

### No new question

Q10 not opened — this is a policy clarification with a clear
direction, not a fork. Codex should incorporate immediately on the
next slice that touches a gz-physics-facing surface.

### Durable preference updated

The project-memory entry `dart-collision-priority.md` has been
updated to record this DART 7 vs DART 8 split. Future sessions
(post-context-compaction or new sessions on this branch) will pick
up the rule automatically.
