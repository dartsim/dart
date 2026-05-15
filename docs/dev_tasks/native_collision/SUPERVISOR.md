# Supervisor Notes for Codex

This file is supervisory feedback from a Claude session running independent
review agents (critic, verifier, architect) over the native_collision dev task.
It is NOT part of the planning contract — README/01-06 remain authoritative.
Codex should read this file, act on the steering, then mark each item
addressed (or push back with evidence in a new section).

Last updated: 2026-05-14 (live supervisor session; user answered Q1+Q2 — see §"Open Question for User")

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
| Python compat names construct `DartCollisionDetector`                              | VERIFIED | `python/dartpy/collision/collision_detector.cpp:43-46`                                                                                   |
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

**Top 3 plan risks:**

1. **CRITICAL — Status inflation: README has `[x]` boxes whose evidence is "manual workflow-dispatch on a closed PR using a personal token".** A reader skimming README will believe the project is one step from done; it isn't. The audit row says `Local`/`Open` for the same item.
2. **CRITICAL — The single open `[ ]` README item bundles 6+ distinct workstreams** (Phases 10, 11, 13, 14 from `02-milestones.md`). Collapses make "% complete" unmeasurable.
3. **MAJOR — Goal drift toward "more local evidence" instead of merging.** Recent commits (`Record local downstream collision evidence`, `Record native collision resume evidence`) are doc updates, not code. **The plan has lost the path to merge.**

**Specific contradictions (cited):**

- **README vs. completion-audit on CI status.** README line 132: `[x] CI Linux has a scheduled/manual collision benchmark guard job ... uploads .benchmark_results/...` But `06-completion-audit.md` ~line 30: _"scheduled/permanent gate evidence is still a finalization item."_ The README hides "scheduled gate not proven" behind a checked box.
- **Three different "current" SHAs.** README cites `8c83cd19cb8`; RESUME line 286 says latest pushed code head is `1e1faf6feb1`; `git log` shows current HEAD is `f5d4f9ee932`. A reviewer cannot tell what state is being claimed.
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

A2. **Pick ONE canonical "current SHA"** and ensure README, RESUME, and 06-audit all reference the same one. Suggest: latest pushed head `1e1faf6feb1`, with a footnote that `f5d4f9ee932` is a docs-only update on top of it (which `git log` confirms).

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
- Add Python `DeprecationWarning` on the alias assignments at `python/dartpy/collision/collision_detector.cpp:43-46`. Use `_warn_deprecated_collision_alias` helper if it doesn't already exist.
- Superseded by Q1 addendum for dartpy: remove Python legacy detector aliases
  instead of adding `DeprecationWarning` wrappers; tests and stubs should prove
  the clean `DartCollisionDetector` API.
- Gate the C++ deprecation attribute behind a CMake option `DART_COLLISION_DEPRECATE_LEGACY_NAMES` (default ON; downstreams in mid-migration can `-DDART_COLLISION_DEPRECATE_LEGACY_NAMES=OFF` to silence).
- Add a regression test `UNIT_collision_LegacyDeprecationWarnings` that compiles a tiny program against each legacy spelling under `-Wdeprecated-declarations -Werror=deprecated-declarations` and asserts the warning fires; symmetric Python `pytest` proves `DeprecationWarning` is raised when the alias is constructed.
- Confirm gz-physics still builds. Run `pixi run -e gazebo test-gz` with the option ON; if gz-physics breaks because it constructs legacy names internally, document the gz-physics-side migration patch in `05-downstream-migration.md` rather than removing the deprecation.

**B2. Final runtime cleanup (architect's slice):**

Enumerate `dart/collision/**/reference/` files reachable only from `collision-reference-*` test/benchmark targets. Identify any no longer referenced by `test_reference_backends`, comparative benchmarks, or `createReference()` paths, and delete the unreferenced ones. Keep top-level `dart/collision/{fcl,bullet,ode}/` compatibility facades intact. Run the architect's done-evidence command set. Record results in a new `## Final Runtime Cleanup` subsection in `06-completion-audit.md`.

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
