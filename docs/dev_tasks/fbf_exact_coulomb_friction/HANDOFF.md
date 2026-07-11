# Fresh Session Handoff Prompt

Copy the prompt below into a fresh AI session that starts in this worktree.
It is intentionally explicit because this task has already suffered from
premature completion claims.

```text
You are continuing the DART 6.20 exact-Coulomb FBF friction task in:

  /home/js/dev/dartsim/dart/task_6

Branch:

  research/fbf-friction-release620

Primary task folder:

  docs/dev_tasks/fbf_exact_coulomb_friction/

User's actual objective:

  Fully implement the exact reduced Coulomb friction paper parity work discussed
  in this task, without redefining success around partial tests, reduced
  scaffolds, or GUI smoke checks. The task is not complete until the paper's
  tests, benchmarks, GUI examples, comparison report, citation/credit, and
  verification gates are all covered and proven by current evidence.

North-star goal:

  Deliver a DART 6.20 release-branch implementation and report that can stand
  against the SCA 2026 paper's own evidence: exact reduced Coulomb law in DART,
  preserved default boxed-LCP behavior, all paper scenes represented by tests,
  benchmarks, traces, GUI examples, visual artifacts, residual/sweep figures,
  external comparisons where available, and clear citation/credit. The end
  state is not "the solver compiles" or "a reduced scaffold runs"; it is
  paper-parity evidence plus maintainable DART integration.

Mandatory first answer if asked whether coverage is complete:

  No.

Why: as of 2026-07-09, this branch is still an active partial DART-side
prototype. It has exact-Coulomb math, an opt-in DART 6 solver route, paper
fixture scaffolds, benchmark/trace helpers, and self-contained `dart-demos`
Research scenes, but it does not yet prove full paper parity.

Exit criteria before claiming done:

1. Paper-scene tests:
   - Each paper fixture has a current DART-side test or a documented, cited
     blocker when the exact author scene data is unavailable.
   - Incline, backspin, turntable, Painleve or its documented replacement,
     four-level 26-card house, 25-stone arch projectile/rest, 101-stone arch
     balance, and 10-level card-house dynamics are all covered.
   - Reduced scaffolds remain labeled as reduced and are not counted as full
     paper-scene coverage.
2. Benchmarks and traces:
   - Exact-FBF, default boxed-LCP, and practical external baselines are covered
     where dependencies are available.
   - Kamino, MuJoCo, Rigid-IPC, or paper implementation dependencies may be
     added only as test, example, or benchmark dependencies, never as core
     library dependencies.
   - Hardware, build type, solver settings, step counts, contacts, residuals,
     fallback counts, timeouts, and omitted rows are recorded in `PR_REPORT.md`
     and the specialized task reports.
3. Demos exit criteria:
   - All paper examples appear under `Research` in
     `pixi run demos -- --list-scenes`.
   - Each `fbf_paper_*` scene launches with
     `pixi run demos -- --scene <id>`.
   - Each scene's `Scene` tab is self-contained: overview, what the example
     shows, expected result, coverage limits, diagnostics, available actions,
     and any mismatch from the paper are visible without reading code.
   - `pixi run demos -- --verify-fbf-scene-docs` passes.
   - A factory smoke such as `pixi run demos -- --cycle-scenes --frames 1`, or
     the repository-equivalent scene-cycle command, passes after demo edits.
   - Headless captures exist for paper-matched states, not only reduced
     scaffolds.
   - Relevant `pixi run capture` or `pixi run capture-action` commands pass,
     `pixi run image-verdict <capture>` passes, and the image has been visually
     inspected for nonblank content, correct framing, expected state visibility,
     and no incoherent UI overlap.
   - OSG renderer, camera/capture paths, `dart-demos` host, reusable ImGui
     widgets, overlays, and diagnostics are improved when needed to make these
     examples understandable and verifiable.
4. Reports and docs:
   - `PR_REPORT.md`, `paper-parity-matrix.md`, `gui-capture-report.md`, and
     `residual-history-report.md` describe current results, not aspirational
     targets.
   - Source comments and RTD/user docs clearly cite and credit the SCA 2026
     paper and any reused project assets, including Rigid-IPC geometry if used.
   - The task folder is not retired until durable docs have been promoted to
     their final home and the completion audit says the dev task can be removed.

Before editing anything, read these files in order:

1. AGENTS.md
2. docs/dev_tasks/fbf_exact_coulomb_friction/AGENT_CONTINUATION.md
3. docs/dev_tasks/fbf_exact_coulomb_friction/README.md
4. docs/dev_tasks/fbf_exact_coulomb_friction/RESUME.md
5. docs/dev_tasks/fbf_exact_coulomb_friction/PR_REPORT.md
6. docs/dev_tasks/fbf_exact_coulomb_friction/paper-parity-matrix.md
7. docs/dev_tasks/fbf_exact_coulomb_friction/gui-capture-report.md
8. docs/dev_tasks/fbf_exact_coulomb_friction/residual-history-report.md
9. docs/ai/verification.md
10. docs/onboarding/contributing.md

Current branch/worktree state at handoff:

- Branch is local-only in the latest observed state; no PR was associated with
  `research/fbf-friction-release620`.
- The branch is based on DART 6.20 work and must be refreshed against
  `origin/release-6.20` before publishing.
- The working tree has a broad local feature set, including many untracked
  new files. Do not delete or revert them.
- Do not push, open a PR, mutate GitHub, rerun CI, or delete branches without
  explicit user approval.
- Do not add AI/tool attribution to commits or PRs.

Most recent implemented slice:

- Added a reduced 25-stone masonry-arch projectile scaffold.
- Test:
  `ExactCoulombFbfPaperFixtures.MasonryArch25ProjectileScaffoldRuns`
- Trace scenario:
  `masonry_arch_25_projectile_reduced_contact`
- Benchmark rows:
  `masonry_arch_25_projectile_reduced_contact_boxed_lcp`
  `masonry_arch_25_projectile_reduced_contact_exact_fbf`
- GUI scene:
  `fbf_paper_masonry_arch_25`
- GUI action:
  `p` / `Launch projectile`
- Screenshot:
  `docs/dev_tasks/fbf_exact_coulomb_friction/assets/fbf_paper_masonry_arch_25_projectile.png`
- Widget improvement:
  shared exact-FBF diagnostics show `Exact diagnostics: not run yet` before the
  first exact solve.
- No renderer-level OSG scene-graph change was needed for that reduced
  scaffold, but OSG renderer, camera/capture, host, overlay, and reusable
  ImGui widget improvements are explicitly in scope whenever paper GUI
  examples need them.

Latest verified results to preserve:

- `pixi run cmake --build build/default/cpp/Release --target test_ExactCoulombFbfPaperFixtures fbf_paper_trace BM_INTEGRATION_exact_coulomb_fbf_paper dart-demos --parallel 8`
  passed after formatting.
- `./build/default/cpp/Release/tests/integration/test_ExactCoulombFbfPaperFixtures --gtest_filter=ExactCoulombFbfPaperFixtures.MasonryArch25ProjectileScaffoldRuns`
  passed in `10611 ms`.
- `./build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace masonry_arch_25_projectile_reduced_contact exact_fbf 1 1 nan tracked`
  ended with 48 contacts, one exact solve, zero fallbacks, residual
  `9.9901843912475669e-07`, and `success`.
- The boxed-LCP tracked trace and exact-FBF `dynamic_bodies` trace emitted rows;
  the dynamic-body trace includes 23 mobile arch stones plus the projectile.
- `./build/default/cpp/Release/bin/BM_INTEGRATION_exact_coulomb_fbf_paper --benchmark_filter='BM_PaperFixtureStepTime/masonry_arch_25_projectile_reduced_contact_(boxed_lcp|exact_fbf)$' --benchmark_min_time=0.01s --benchmark_repetitions=1 --benchmark_display_aggregates_only=false`
  reported boxed LCP `0.497 ms` and exact FBF `4921 ms`, 48 contacts, one
  exact solve, zero failures/fallbacks, max residual `999.018n`, and `2205`
  FBF iterations.
- `pixi run demos -- --verify-fbf-scene-docs` and direct
  `./build/default/cpp/Release/bin/dart-demos --verify-fbf-scene-docs`
  checked all nine FBF paper scenes.
- `pixi run demos -- --list-scenes` listed `fbf_paper_masonry_arch_25` under
  `Research`.
- `pixi run capture-action fbf_paper_masonry_arch_25 p docs/dev_tasks/fbf_exact_coulomb_friction/assets/fbf_paper_masonry_arch_25_projectile.png 1280 720 0`
  regenerated the 1280x720 action capture.
- `pixi run image-verdict docs/dev_tasks/fbf_exact_coulomb_friction/assets/fbf_paper_masonry_arch_25_projectile.png`
  passed. Visual inspection showed the projectile, Scene-tab overview,
  expected result, coverage text, `Exact diagnostics: not run yet`, and
  `Projectile: launched`.
- `pixi run docs-build`, `pixi run lint`, `pixi run build`,
  `git diff --check`, and the corrected binary-skipping untracked whitespace
  loop passed after the final report/tracker edits.

Known local command hazards:

- Do not run two `pixi run demos`, `pixi run capture`, or build commands in
  parallel against the same Ninja tree. Earlier simultaneous invocations caused
  `.ninja_deps` recovery warnings and an archive/link race. Rerun serially.
- Ninja may print `ninja: warning: premature end of file; recovering` from the
  earlier race. It has recovered and linked successfully in later serial runs.
- In zsh, do not use `path` as a loop variable. It clobbers the tied `PATH`
  array. Use `f` or `file_path` for whitespace loops.
- Quote benchmark filters containing parentheses or `|`.

Current strongest implemented subset:

- Internal exact-Coulomb cone math, contact problem helpers, FBF solver helpers,
  residual diagnostics, residual-history sampling, and focused unit tests.
- DART 6 exact-contact adapter and opt-in `ExactCoulombFbfConstraintSolver`.
- Partial matrix-free `W*x` route and operator-local seed diagnostics.
- Default boxed-LCP solver behavior remains preserved; exact FBF is opt-in.
- Headless paper-fixture tests and benchmark/trace rows for small fixtures.
- Reduced 26-card one-step scaffold and reduced two-step settle/projectile
  scaffold.
- Reduced 25-stone and 101-stone masonry-arch scaffolds, plus the latest
  reduced 25-stone projectile scaffold.
- Construction-only 10-level card-house scaffold.
- `dart-demos` Research scenes for nine FBF paper examples, each with
  self-contained Scene-tab overview, expected result, and coverage limits.
- Action-aware GUI capture for 26-card projectiles and 25-stone arch
  projectile.
- RTD/source credit for Song, Fan, Ascher, and Pai's SCA 2026 paper, plus
  tracked note for Rigid-IPC masonry-arch geometry credit where used.

Progress snapshot by requirement:

- Solver and math: reference exact-Coulomb cone/problem/FBF helpers are present
  with tests; DART integration is opt-in and partial, not the default solver.
- Preserved behavior: boxed-LCP remains the default route and must keep passing
  existing solver tests whenever exact-FBF code changes.
- Paper-scene tests: small fixture and reduced scaffold coverage exists; full
  paper dynamic scenes remain missing or incomplete.
- Benchmarks and traces: reduced rows exist for boxed LCP and exact FBF; full
  long-run paper rows, omitted-row accounting, and external baselines remain
  incomplete.
- GUI examples: nine `dart-demos` Research scenes exist and their Scene tabs
  are self-contained for the current reduced scope; paper-matched dynamic
  visual outcomes and full-contact captures remain incomplete.
- Reports: `PR_REPORT.md` and the specialized reports exist and should be kept
  as the PR-ready evidence bundle, but they still describe a partial state.
- Citations and credit: current docs include paper credit and a Rigid-IPC
  geometry credit reminder; keep both source comments and RTD pages in sync as
  new assets or baselines are added.

Major missing paper-parity requirements:

- Full-contact bounded exact-FBF dynamics for the 26-card four-level house.
  Current practical default is 56 contacts; 59 is no-fallback but too slow;
  60 remains the one-step fallback boundary; 64 is not clean.
- Paper Fig. 6 full 6.7 s no-creep settle and 10 s projectile impact sequence
  with long-run traces, dynamic snapshots, residual plots, and timing parity.
- Author/Rigid-IPC geometry for the 25-stone and 101-stone arches.
- Paper Fig. 7 25-stone pinned/projectile physical outcome. The current
  projectile case is only a reduced one-step scaffold at 48 contacts.
- Bounded 25-stone 100-contact row. Current 64/4 and 80/4 cap rows solve, but
  80/4 takes about 118 s and 100/4 timed out under 120 s.
- Paper Fig. 8 101-stone long-run balance outcome, snapshots, and timing
  parity. One-step approximate scaffold solves through 100/2 but is not enough.
- Exact-FBF dynamics for the 10-level card-house GPU comparison scene.
- Figure 9 residual histories and Figure 10 gamma sweeps for full-contact,
  long-run paper scenes, not only reduced one-step scaffolds.
- Kamino, MuJoCo, and paper-code comparison harnesses. These may be added only
  as test/example/benchmark dependencies, not core library dependencies.
- Painleve remains a DART-side proxy until author parameters/files or a clear
  source-level unavailability note are available.
- Full paper-matched GUI snapshots/overlays and visual outcome explanations
  for the contact-rich scenes.

Approach lanes:

- Lane A, contact-rich convergence and performance: make the 26-card and arch
  rows progress at full or bounded paper-like contact counts. Focus on matrix-
  free product cost, scratch-backed products, warm starts that do not hide
  failures, and residual/fallback evidence. Do not repeat rejected seed-only,
  Anderson-only, or boxed-seed-only experiments as if they were new proof.
- Lane B, paper assets, geometry, and baselines: obtain or recreate author-
  faithful geometry and fixture parameters, especially Rigid-IPC masonry arch
  data if available. Add external comparison dependencies only in tests,
  examples, or benchmarks.
- Lane C, GUI and presentation parity: make the paper scenes inspectable in
  `dart-demos`, with Scene-tab explanations, action buttons, overlays,
  diagnostics, captures, and OSG/host/widget improvements when the current
  demo stack cannot show the result clearly.
- Lane D, reporting and durable docs: keep `PR_REPORT.md`, the parity matrix,
  residual-history report, GUI report, RTD citation page, and source comments
  synchronized with the actual evidence.

Concrete next steps from current state:

1. Re-read `AGENT_CONTINUATION.md`, `README.md`, `RESUME.md`, and
   `PR_REPORT.md`, then run `git status -sb` to confirm the branch still
   matches this handoff.
2. Pick one lane and one falsifiable gap before editing. The best next choices
   are usually the 26-card 60-contact fallback boundary, the 25-stone arch
   author/Rigid-IPC geometry path, or the matrix-free product cost that makes
   high-contact rows impractical.
3. Build the exact touched targets before direct binary runs. Do not assume
   `pixi run build` created every integration test or benchmark executable.
4. Run a focused probe that can fail meaningfully, record the command and
   result, and update the task tracker after that slice.
5. If the slice affects a GUI scene, update the Scene tab, capture/action
   commands, image verdict, visual inspection note, and any needed OSG,
   camera, host, overlay, or ImGui widget behavior in the same slice.
6. After any implementation slice, update these files in the same turn:
   - `docs/dev_tasks/fbf_exact_coulomb_friction/AGENT_CONTINUATION.md`
   - `docs/dev_tasks/fbf_exact_coulomb_friction/README.md`
   - `docs/dev_tasks/fbf_exact_coulomb_friction/RESUME.md`
   - `docs/dev_tasks/fbf_exact_coulomb_friction/PR_REPORT.md`
   - `docs/dev_tasks/fbf_exact_coulomb_friction/paper-parity-matrix.md`
   - any specialized report touched by the slice, such as
     `gui-capture-report.md` or `residual-history-report.md`
7. Before claiming PR-ready, run the focused tests/benchmarks/traces, GUI
   capture checks when touched, `pixi run lint`, `pixi run build`,
   `git diff --check`, and the binary-skipping untracked whitespace check.
8. Do not mark the task complete while
   `docs/dev_tasks/fbf_exact_coulomb_friction/` still exists. Completion also
   requires durable promotion out of the task folder and folder cleanup.

Required GUI rule:

Every promoted GUI example must be understandable from the `Scene` tab alone.
The tab must include overview, expected result, and coverage/limits. If the
current OSG renderer, camera/snapshot path, `dart-demos` host, or ImGui widgets
cannot make that true, improve those surfaces in this task before claiming GUI
coverage.

Verification expectations:

- Use focused CMake targets before direct CTest/binary runs. `pixi run build`
  does not build every test binary.
- For touched C++ code, run the focused tests/benchmarks/traces that prove the
  changed path, then `pixi run lint` and `pixi run build`.
- For GUI changes, run:
  `pixi run demos -- --verify-fbf-scene-docs`
  and a relevant `pixi run capture` or `pixi run capture-action` plus
  `pixi run image-verdict`. Inspect the image yourself.
- For RTD/user docs changes, run `pixi run docs-build` when practical.
- Before saying PR-ready, run `git diff --check` and a binary-skipping
  untracked whitespace check.

Changelog decision:

- This handoff-only document does not require a changelog entry.
- The broader feature likely needs a changelog decision before PR publication
  because it adds user-visible demos/benchmarks/docs and an opt-in solver path.
  Follow `docs/onboarding/changelog.md`.

Final reminder:

Do not answer that all paper tests, benchmarks, and GUI examples are present
until the completion gate in `AGENT_CONTINUATION.md` is fully satisfied with
current evidence. A passing focused test, benchmark row, or GUI screenshot is
progress evidence only.
```
