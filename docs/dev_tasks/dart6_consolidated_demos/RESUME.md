# Resume Instructions

Current phase: Phase 2 (scene ports). B1 accepted (commit 49d1db3, 12 scenes
live). B2 ("Constraints & soft": dynamic_joint_constraints,
joint_constraints, human_joint_limits, mixed_chain, soft_bodies, tinkertoy,
heightmap) dispatched to a Codex worker 2026-07-04 evening.

## Orchestration playbook (learned, follow it)

1. One implementation worker at a time — batches share
   build/default/cpp/Release; parallel builds there corrupt/contend (a
   load-84 pileup happened). Read-only recon/review agents parallelize
   freely.
2. Codex workers implement well but STALL waiting on build notifications
   (3 incidents). If a worker idles mid-task: check
   `git status --short` + `pgrep -fl ninja` + grep its last transcript text;
   verify claimed edits exist in source (rg for the specific symbols) before
   believing any report. Route FIX ROUNDS to a general-purpose Claude agent
   (fixer-b1 pattern) — they run to completion.
3. Acceptance per batch = all of: (a) 9-way-per-scene review workflow
   (parity vs parity/parity-bN.json + crash-risk); (b) orchestrator reads
   EVERY new scene PNG (code review misses render bugs — simple_frames was
   caught only by screenshot); (c) cycle-scenes x2 green run by
   orchestrator; (d) lint green; (e) worker parity report reconciled.
4. Commits: pre-commit guard runs check-lint-quick + a cmake configure in
   the SHARED build dir — never commit while a worker builds; run
   `pixi run lint` as a separate command BEFORE any command containing
   `git commit` (the guard intercepts the whole command first).
5. Host conventions (enforce in review): AlwaysClamp+isfinite sliders,
   lowercase keys, world-owned visuals (no raw OSG injection), build-time
   visual init, short panel labels + SetNextItemWidth 0.6.

## Next steps in order

1. When codex-b2 reports (or stalls): verify edits in source, run
   acceptance per playbook §3 (launch review workflow like
   review-b1-ports with parity-b2.json), fix round via Claude agent if
   needed, accept + commit.
2. Between B2 and B3 (build dir free): (a) commit any queued docs;
   (b) Phase 3 PR-a — InteractiveFrameDnD destructor fix in
   dart/gui/osg/DragAndDrop.{hpp,cpp} (delete mDnDs; ABI-safe out-of-line
   dtor), build + test + changelog, separate topic branch off
   origin/release-6.20; (c) build dartpy (`pixi run build-py-dev`) to
   unblock Phase 4.
3. B3 (Control & IK), B4 (Humanoids & robots) same pattern; B4 includes
   optional-dep guards (octomap/ikfast/MJCF).
4. Phase 3 suite per BRIEF-phase3.md; Phase 4 py-demos per
   EVIDENCE-dartpy-bindings.md (keyboard navigator, no new bindings);
   Phase 5 cleanup per EVIDENCE-cleanup-refs.md; Phase 6 wrap per PLAN.md.

## Session log

- 2026-07-04: Phases 0-1 done (see PLAN.md). B1 ported by codex-b1
  (stalled 3x; stopped), fix round by fixer-b1 (11/11 items,
  commit 49d1db3), accepted after screenshot + 9-agent review. Key
  library findings recorded in PLAN Phase 3: InteractiveFrameDnD leak;
  bullet backend needs --no-as-needed when linked only for its
  self-registration.
