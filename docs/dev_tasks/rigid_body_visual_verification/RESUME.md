# Resume: Rigid Body Visual Verification

## Current Handoff (2026-06-12)

The latest continuation makes failed rows from resilient workflow packets
directly actionable. `py-demo-capture -- --rigid-workflow
--continue-on-failure` now records `failed_rows` with workflow row-range rerun
commands that preserve optional packet flags and absolute row numbering, while
the final workflow manifest and process exit code still fail when any selected
row failed. The in-viewer `Rigid Workflow` panel exposes the resilient
extended-packet command, and README/PLAN-103 describe the same user path.

Previous checkpoint: the public packet row-range rerun examples were refreshed
after the capture-first heavy stack packet expansion. The README and PLAN-103
sidecar now tell users to rerun rows 47-48 when they request
`--include-related --include-packets`, so both `rigid_ipc_stack_packet` and
`rigid_ipc_heavy_stack_packet` are included. The drift guard scopes the public
rerun example and prevents it from regressing to the old row-47-only packet
command.

Previous checkpoint: the capture-first heavier Rigid IPC stack packet slice
completed. The existing four-box `rigid_ipc_stack_packet` now shares a
spec-driven implementation with a six-box `rigid_ipc_heavy_stack_packet`; both
stay in the non-numbered Rigid IPC shelf and only enter workflow capture
packets through the opt-in `--include-packets` group. The default 36-row World
Rigid Body workflow remains unchanged.

Historical stop-only note: after the heavy packet commit, the user requested
no further implementation and no further verification while hand-off docs were
ensured. That note was superseded by later active-goal continuations.

Observed repository state at this hand-off:

- Branch: `feature/rigid-body-gui-visual-verification`.
- Resume check: verify whether the latest failed-row rerun-command slice is in
  history as `Add rigid workflow failed-row reruns` with `git status -sb` and
  `git log -5 --oneline` before resuming. If it is not committed yet, inspect
  the uncommitted diff for the same slice before continuing. Do not push it
  without explicit approval.
- Latest local slices update `scripts/capture_py_demo.py`,
  `python/examples/demos/runner.py`, `python/tests/unit/test_capture_py_demo.py`,
  `python/tests/unit/test_py_demo_panels.py`,
  `python/tests/integration/test_demos_cycle.py`,
  `python/examples/demos/README.md`, and the PLAN-103 rigid sidecar to support
  and document resilient workflow packets, failed-row summaries, and
  workflow-context rerun commands.
- Previous row-range slice updated `python/examples/demos/README.md`,
  `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`, and
  `python/tests/integration/test_demos_cycle.py` to keep the packet row-range
  example aligned with the two capture-first packet rows.
- Latest local failed-row rerun-command slice is titled
  `Add rigid workflow failed-row reruns`; it builds on
  `93a1ce752e5 Summarize rigid workflow failed rows`,
  `c9f00c351d6 Let rigid workflow packets continue after failures`, and
  `12df4e1f083 Refresh rigid packet row range guidance`.
- Origin tip observed before these local slices was
  `bdf757db2c9 Refresh rigid handoff stop state`.
- There is no PR associated with this branch.
- Do not push without explicit approval; always verify with `git status -sb`
  and `git log -5 --oneline` before resuming.
- The local implementation adds/updates
  `python/examples/demos/scenes/rigid_ipc_stack_packet.py`,
  `python/examples/demos/registry.py`, `scripts/capture_py_demo.py`,
  `python/tests/unit/test_py_demo_panels.py`,
  `python/tests/unit/test_capture_py_demo.py`,
  `python/tests/integration/test_demos_cycle.py`,
  `python/examples/demos/README.md`, and the PLAN-103 rigid sidecar.
- Verification for this slice: syntax checks passed; the focused Python suite
  below reported `11 passed`; the extended
  workflow dry-run planned rows `51-52 / 52` with both packet variants and
  complete guidance; the real row-52 docked workflow capture completed with one
  captured row, no failed rows, a nonblank docked screenshot, 11 PNG frames, 12
  metric events, `box_count=6.0`, `top_mass=4.25`, and
  `status=capture-first`; `pixi run lint` passed; the current continuation
  reran the focused pytest after lint with `11 passed`; and `git diff --check`
  was clean before committing the failed-row rerun-command slice.
- Verification for the latest row-range sync: the focused
  `test_rigid_visual_capture_first_packets_are_documented` pytest passed after
  the new guard was made whitespace/history scoped; the exact updated dry-run
  command planned rows `47-48 / 48` with two capture-first packet rows and
  complete guidance under
  `/tmp/dart_capture_rigid_workflow_packet_rows_47_48_1781287569`;
  `pixi run lint` passed; the post-lint focused pytest passed again; and
  `git diff --check` was clean.
- Verification for the latest failure-resilience slice: the focused pytest
  covering fail-fast behavior, continue mode, workflow-only flag validation,
  panel command rendering, failed-row summaries, workflow row rerun commands,
  and docs guard reported `11 passed`. The public
  dry-run with `--continue-on-failure` over rows 51-52 reported
  `status=planned`, `continue_on_failure=true`, `capture_count=2`,
  `workflow_total_count=52`, `guidance_complete=true`, and both capture-first
  stack packet rows. The current continuation reran the focused pytest after
  adding the review-index failure-mode badge, then extended the manifest and
  review index with failed-row triage summaries plus workflow row-range rerun
  commands for resilient packets, verified the public dry-run reports
  `failed_rows` length `0` and review-index `failure mode=continue` for a
  planned packet, verified planned rows 51 and 52 carry
  `workflow_rerun_command` values preserving the optional packet flags and
  absolute row ids, and then ran the required pre-commit lint gate.

Previous Replay capture-metadata checkpoint context: this checkpoint added
reviewer-facing Replay timeline metadata to the capture packet after the
numbered 36-row Replay timeline pass completed. Single-scene
`py-demo-capture` manifests include a JSON-safe
`scene_metadata.replay_timeline` summary with the Replay panel name,
value-track label, and signal/marker availability; workflow
`review_index.html` cards show that Replay track next to the existing row
guidance and metric summaries.

Previous loop-closure checkpoint context: it completed the
`rigid_loop_closure` Replay timeline slice after
the multibody solver-family checkpoint. The variational rigid multibody
loop-closure row now uses max closure residual ratio as its Replay value track
and marks solve-advantage, residual-versus-solved separation, distance-family
tip drift, and rigid-orientation frames.

Expected repository state for that earlier checkpoint:

- Branch: `feature/rigid-body-gui-visual-verification`.
- Local `HEAD` before the loop-closure implementation work:
  `1add2036097 Add multibody solver family replay timeline`.
- `git status -sb` before this continuation resumed showed the branch ahead of
  `origin/feature/rigid-body-gui-visual-verification` by fourteen commits with
  uncommitted row 36 scene/test edits and stop-only hand-off docs; this
  checkpoint supersedes that hand-off state.
- Latest implementation checkpoints covered by this hand-off:
  `4c9f367bcd0 Preserve requested rigid workflow packet groups`,
  `f48187d6ce2 Summarize rigid workflow packet groups in review index`, and
  `f01f471bae7 Expose rigid workflow packet commands in the panel`, followed
  by `3c5b9e517d3 Enable rigid workflow video packets`,
  `d5c6de2bee1 Describe optional rigid workflow rows`,
  `5a4529f0083 Audit rigid workflow guidance coverage`, and
  `ad013e62069 Refresh rigid guidance audit handoff`, followed by the current
  live open-command, stack Replay timeline, pushed hand-off, contact
  manipulation Replay timeline, kinematic-driver Replay timeline,
  normal-push Replay timeline, fixed-joint Replay timeline, and
  joint-breakage Replay timeline, distance-spring Replay timeline,
  limited-joints Replay timeline, motor-limits Replay timeline,
  passive-parameters Replay timeline, screw-joint pitch Replay timeline,
  multibody dynamics-terms Replay timeline, link center-of-mass Replay
  timeline, link-Jacobian Replay timeline, multibody solver-family Replay
  timeline, and loop-closure Replay timeline checkpoint.
- `d98abdde973 Refresh rigid visual verification handoff` is a docs-only pushed
  checkpoint after the stack Replay timeline slice.
- Local `HEAD` before the link center-of-mass implementation commit was
  `7438e13cca9 Add multibody dynamics replay timeline`; the branch was
  observed clean and ahead of
  `origin/feature/rigid-body-gui-visual-verification` by eleven commits before
  this slice.
- Local `HEAD` before the link-Jacobian implementation work was
  `c7042091c2b Add link center of mass replay timeline`; the branch was ahead
  of `origin/feature/rigid-body-gui-visual-verification` by twelve commits and
  carried docs-only stop-handoff edits that this checkpoint supersedes.
- Local `HEAD` before the multibody dynamics-terms implementation commit was
  `29ab458fc01 Add screw joint replay timeline`; the branch was observed clean
  and ahead of `origin/feature/rigid-body-gui-visual-verification` by ten
  commits before this slice.
- Local `HEAD` before the screw-joint pitch implementation commit was
  `6f307966524 Add passive joint replay timeline`; the branch was ahead of
  `origin/feature/rigid-body-gui-visual-verification` by nine commits before
  that slice.
- The kinematic-driver Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata. The intended value track label is
  `IPC grip box travel`, with markers for IPC grip contact/carry progress,
  slip-lane slip, and static-like caveat frames.
- The normal-push Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata. The intended value track label is
  `Target travel divergence`, with markers for contact, IPC penetration, SI
  push-progress, and divergence frames.
- The current fixed-joint Replay timeline slice adds `replay_timeline_signal(...)`,
  `replay_timeline_marker(...)`, and `info["replay_timeline"]` metadata. The
  intended value track label is `Fixed-joint offset error`, with markers for
  pose-error and residual-motion recovery frames.
- The current joint-breakage Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`,
  `replay_capture_state`, `replay_restore_state`, and
  `info["replay_timeline"]` metadata through the shared AVBD breakable-joint
  builder. The intended value track label is `Payload release distance`, with
  markers for broken or released frames.
- The current distance-spring Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata. The intended value track label is
  `Max spring stretch`, with markers for high-stretch or off-center-spin
  frames.
- The current limited-joints Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata. The intended value track label is
  `Locked-axis error`, with markers for locked-error or free-axis-motion
  frames.
- The current motor-limits Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata. The intended value track label is
  `Force travel gap`, with markers for velocity-clamp, position-stop, or
  effort-cap frames.
- The current passive-parameters Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata. The intended value track label is
  `Armature position gap`, with markers for damping energy separation, Coulomb
  slip, or armature-lag frames.
- The current screw-joint pitch Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata. The intended value track label is
  `Coarse/fine travel gap`, with markers for pitch-spread, zero-pitch
  contrast, or reverse-sign frames.
- The current multibody dynamics-terms Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata. The intended value track label is
  `Response norm gap`, with markers for response separation, off-diagonal
  coupling, or heavy-load torque frames.
- The current link center-of-mass Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata. The intended value track label is
  `Mirrored COM angle spread`, with markers for mirrored-angle,
  centered-still, or high-inertia-lag frames.
- The current link-Jacobian Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata. The intended value track label is
  `Link-origin speed`, with markers for high-twist, wrench-load, world/body
  Jacobian gap, or residual-alert frames.
- The current multibody solver-family Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata. The intended value track label is
  `Residual solve ratio`, with markers for solve-advantage, residual-only
  drift, or solved-tight frames while residual rows remain loose.
- The current loop-closure Replay timeline slice adds
  `_last_float(...)`, `replay_timeline_signal(...)`,
  `replay_timeline_marker(...)`, and `info["replay_timeline"]` metadata to
  `python/examples/demos/scenes/rigid_loop_closure.py`. The intended value
  track label is `Max closure residual ratio`, with markers for large closure
  solve advantage, residual-versus-solved separation, distance-family tip
  drift, and rigid-orientation solve frames.
- There is no PR associated with this branch at checkpoint time.
- The numbered rigid workflow Replay timeline pass is now complete through row 36. A future session should re-evaluate the durable sidecar and dashboard
  before selecting the next bounded rigid visual-verification slice.
- The current continuation resumed implementation from the active persistent
  goal and finished the pending guidance-audit checks after the previous
  hand-off-only stop checkpoint.
- The live open-command continuation finished the previously documented WIP,
  added capture-helper tests and public docs, and reran focused tests, a public
  dry-run artifact inspection, and `pixi run lint`.
- The stack Replay timeline continuation added `replay_timeline` metadata to
  `rigid_stack_stability`, updated tests and docs, and ran focused tests plus a
  real docked capture.
- The contact-manipulation Replay timeline continuation added
  `replay_timeline` metadata to `rigid_contact_manipulation`, updated tests and
  docs, and ran focused tests plus a real docked capture.
- The kinematic-driver Replay timeline continuation added
  `replay_timeline` metadata to `rigid_kinematic_driver`, updated tests and
  docs, and ran focused tests plus a real docked capture.
- The normal-push Replay timeline continuation added `replay_timeline` metadata
  to `rigid_kinematic_normal_push`, updated tests and docs, and ran focused
  tests plus a real docked capture.
- The fixed-joint Replay timeline continuation added `replay_timeline` metadata
  to `rigid_fixed_joint`, updated tests and docs, and ran focused tests plus a
  real docked capture.
- The joint-breakage Replay timeline continuation added `replay_timeline`
  metadata to the shared AVBD breakable-joint builder used by
  `rigid_joint_breakage`, updated tests and docs, and ran focused tests plus a
  real docked capture.
- The distance-spring Replay timeline continuation added `replay_timeline`
  metadata to `rigid_distance_spring`, updated tests and docs, and ran focused
  tests, drift guards, a real docked capture, `pixi run lint`, and
  `git diff --check`.
- The limited-joints Replay timeline continuation added `replay_timeline`
  metadata to `rigid_limited_joints`, updated tests and docs, and ran focused
  tests, drift guards, a real docked capture, `pixi run lint`, and
  `git diff --check`.
- The motor-limits Replay timeline continuation added `replay_timeline`
  metadata to `rigid_joint_motor_limits`, updated tests and docs, raised its
  maintained capture budget to 96 frames so the position stop is visible, and
  ran focused tests, drift guards, a real docked capture, `pixi run lint`, and
  `git diff --check`.
- The passive-parameters Replay timeline continuation added `replay_timeline`
  metadata to `rigid_joint_passive_parameters`, updated tests and docs, and
  ran focused tests, drift guards, a real docked capture, `pixi run lint`, and
  `git diff --check`.
- The screw-joint pitch Replay timeline continuation added `replay_timeline`
  metadata to `rigid_screw_joint_pitch`, updated tests and docs, and ran
  focused tests, drift guards, a real docked capture, `pixi run lint`, and
  `git diff --check`.
- The multibody dynamics-terms Replay timeline continuation added
  `replay_timeline` metadata to `rigid_multibody_dynamics_terms`, updated tests
  and docs, and ran focused tests, drift guards, a real docked capture,
  `pixi run lint`, and `git diff --check`.
- The link center-of-mass Replay timeline continuation added `replay_timeline`
  metadata to `rigid_link_center_of_mass`, updated tests and docs, and ran
  focused tests, drift guards, a real docked capture, `pixi run lint`, and
  `git diff --check`.
- The multibody solver-family Replay timeline continuation added
  `replay_timeline` metadata to `rigid_multibody_solver_family`, updated tests
  and docs, ran focused tests, drift guards, a real docked capture,
  `pixi run lint`, and `git diff --check`, then committed locally as
  `1add2036097 Add multibody solver family replay timeline`.
- The loop-closure Replay timeline continuation added `replay_timeline`
  metadata to `rigid_loop_closure`, updated tests and docs, and ran focused
  tests plus a real docked capture. Drift guards, lint, and diff checks are
  recorded in the validation section below.
- Historical note: these checkpoints were local and unpushed at that moment.
  Current branch reality is recorded in the hand-off and readiness audit above.
- Before any future commit, rerun the repository-mandated `pixi run lint`.

## Last Session Summary

The latest continuation completed the heavy capture-first Rigid IPC stack
packet. `rigid_ipc_heavy_stack_packet` is a six-box, top-heavy variant that
shares the stack-packet implementation, stays outside the numbered 36-row
workflow, and appears only in the optional packet group after
`rigid_ipc_stack_packet`. Syntax checks, focused pytest, a rows 51-52 extended
dry-run, a real row-52 docked workflow capture, lint, and diff checks passed.
The current continuation then refreshed the public row-range rerun examples so
the related-plus-packets command covers rows 47-48 instead of only the original
row 47 packet, and added focused drift coverage for that public command.
The newest continuation adds `--continue-on-failure` to workflow packets,
surfaces a resilient extended-packet command in the live `Rigid Workflow`
panel, documents the command in README/PLAN-103, and adds focused regression
coverage for failure continuation and workflow-only flag validation.

The previous implementation session fixed `py-demos --cycle-scenes` so a
bounded `--frames N` value applies to every scene instead of ending the whole
catalog after the first frame. It updated tests and docs, validated the full
151-scene headless cycle command, and committed the change locally as
`0e38e3e807d Fix py-demos cycle scene frame budget`.

The latest session added uncommitted work for two user-facing rigid workflow
gaps: broader in-viewer search aliases for DART 7 solver/backend/contact terms,
and `py-demo-capture -- --rigid-workflow` for planning or running all numbered
rigid captures into a workflow-level manifest. Focused tests and the public
dry-run command passed, and the full 36-scene workflow capture later completed
successfully under `/tmp/dart_capture_rigid_workflow_full_1781257174`.
After that full capture, the capture helper was adjusted to flush workflow
progress and isolate each per-scene capture in a child process. The user then
requested handoff-only work and no further verification.

The current continuation inspected the uncommitted workflow diff, fixed a
missing-evidence bug where a child process could exit 0 without writing its
per-scene manifest, and added a regression so that case fails the top-level
workflow manifest. Focused tests, public dry-run, lint, and `git diff --check`
passed after the fix.

The latest continuation added two more user-facing GUI improvements: the
`Rigid Workflow` panel now shows each row's exact `py-demo-capture` command,
and adjacent parameter/profiling rows now label their comparison axis plus
held-fixed controls in both panels and capture metrics. Focused pytest,
the public workflow dry-run, lint, and `git diff --check` passed.

The current continuation adds the next workflow review surface:
`py-demo-capture -- --rigid-workflow` now rewrites a top-level
`review_index.html` contact sheet on dry-run, progress updates, failure, and
completion. The contact sheet links every numbered row's manifest, screenshot,
frame directory, command, and comparison/metrics summary so reviewers can scan
all 36 outputs without opening per-scene folders manually. Focused pytest,
public dry-run, lint, and `git diff --check` passed; the dry-run manifest
recorded `artifacts.review_index`, `capture_count=36`, and `status=planned`.
The latest continuation then ran the slow full workflow capture with the review
index enabled. It completed all 36 numbered rows under
`/tmp/dart_capture_rigid_workflow_full_review_index_1781259714`; the top-level
manifest reported `status=complete`, `completed_count=36`, `failed_count=0`,
and `elapsed_s=314.278`, and the generated `review_index.html` contained
36 screenshot thumbnails plus comparison/metric summaries.
The verified rigid workflow search/capture/review-index slice was then
committed and pushed as
`e8278b6fb53 Improve rigid workflow capture evidence`.

The current continuation makes one focused viewer-routing improvement:
related-evidence `Find row` matches such as `avbd prismatic` now open the
matched non-numbered shelf scene directly instead of landing on the numbered
source row first.

The newest continuation adds an opt-in related-evidence capture bundle:
`py-demo-capture -- --rigid-workflow --include-related` appends the ten
non-numbered related shelf routes after the 36 numbered rows and records them
in the same manifest/review index with `workflow_group=related_evidence`.

The latest continuation adds a second opt-in bundle:
`py-demo-capture -- --rigid-workflow --include-packets` appends the
capture-first `rigid_ipc_stack_packet` after the numbered rows and optional
related-evidence routes with `workflow_group=capture_first_packet`.

The current continuation adds workflow row-range selection:
`py-demo-capture -- --rigid-workflow --workflow-start-row N --workflow-end-row M`
captures or dry-runs only the selected absolute workflow rows while keeping
their original row labels and `scenes/NN_<scene>` output directories.

The latest continuation enriches the generated workflow evidence packet:
numbered rigid workflow entries now carry the same row guidance as the
in-viewer `Rigid Workflow` panel, including role label, user question,
try-first action, inspect signals, healthy signal, and scope note in both the
manifest and `review_index.html`.

The current continuation promotes `rigid_ipc_edge_drop` into a self-describing
related-evidence route from `rigid_solver_compare`, with a panel and capture
metrics for degenerate edge-barrier gap, tilt, angular speed, contact count,
step timing, and status. The scene intentionally remains outside the numbered
36-row World Rigid Body workflow because it is an IPC-specific capability view.

The newest continuation adds scene-owned capture metrics to the remaining
direct Rigid IPC shelf scenes: `rigid_ipc`, `rigid_ipc_slide`,
`rigid_ipc_incline`, and `rigid_ipc_pile`. These scenes still stay outside the
numbered 36-row workflow and outside the related-evidence route table, but
their direct docked captures now carry barrier-settle, tangential-slide,
inclined-slide, and pile speed/gap/contact/timing evidence while preserving
shared replay controls.

The current continuation adds `--include-ipc-shelf` to
`py-demo-capture -- --rigid-workflow`. The new opt-in group appends
`rigid_ipc`, `rigid_ipc_slide`, `rigid_ipc_incline`, and `rigid_ipc_pile` to
the workflow manifest and `review_index.html` with
`workflow_group=rigid_ipc_shelf`, without changing the default 36-row workflow
or the existing related/packet-only row counts.

The earlier stop checkpoint was hand-off documentation only. The current
continuation supersedes it by completing and verifying the link-Jacobian Replay
timeline slice.

The current continuation fixes a review-packet metadata edge case:
row-range workflow manifests now keep `include_related`, `include_ipc_shelf`,
and `include_packets` tied to the requested packet shape, while new
`selected_include_related`, `selected_include_ipc_shelf`, and
`selected_include_packets` fields describe the optional groups actually present
in the selected rows. The workflow `review_index.html` header shows the same
row-span, requested-groups, and selected-groups context for reviewers.

The newest continuation closes the loop back into the live GUI: the
`Rigid Workflow` panel now lists full numbered packet, current-row range rerun,
and extended related/IPC-shelf/packet commands under a `Review packet` section.

The current continuation adds workflow-packet motion evidence: `--video --fps`
is now accepted with `--rigid-workflow`, passed through to each selected row
capture, recorded in per-scene manifests, and linked from the workflow
`review_index.html`; the live `Rigid Workflow` panel exposes a current-row
motion packet command.

The final action in this checkpoint is hand-off documentation only. The user
explicitly requested stopping all further implementation and verification, so
no new tests, lint, dry-runs, or diff checks were run for this docs-only
handoff update.

The newest continuation adds optional-row self-description to workflow
evidence packets. Related-evidence, direct Rigid IPC shelf, and capture-first
packet rows now carry role, user question, try-first action, inspect signals,
healthy signal, scope, and related-source metadata into `manifest.json` and
`review_index.html`, rather than relying only on the `workflow_group` label.

The current continuation adds a guidance-coverage audit to the same outputs.
`manifest.json` records `guidance_complete`, `guidance_missing_count`, and
`guidance_missing_rows`, and `review_index.html` shows a guidance badge plus a
warning block when a selected row is missing required self-description fields.
Focused pytest and the public rows 37-51 extended-packet dry-run both passed
after the stop checkpoint was superseded by the active-goal continuation.

The latest continuation finishes the live open-command slice. The
`Rigid Workflow` panel and generated workflow review cards now put a live
`pixi run py-demos -- --scene ...` command next to the existing reproducible
capture command, and the manifest records the same command as
`viewer_command`. Focused pytest, public row-15 dry-run artifact inspection,
and `pixi run lint` passed.

The current continuation adds replay-guided timeline metadata to
`rigid_stack_stability`. The shared Replay panel now uses top-x divergence as
its value track and marks overlap, low-clearance, top-drift, or
solver-divergence frames for targeted visual debugging. Focused pytest and a
real docked capture passed.

That session then stopped at hand-off documentation only. That historical
stop-only handoff was later superseded by the active-goal continuation; the
contact-manipulation, kinematic-driver, normal-push, and fixed-joint timeline
slices below are the current local continuation state.

The current continuation adds replay-guided timeline metadata to
`rigid_contact_manipulation`. The shared Replay panel now uses travel
divergence as its value track and marks pusher contact/proximity,
target-motion, or solver-divergence frames for targeted visual debugging.
Focused pytest, sidecar/README drift guards, and a real docked capture passed.

The latest continuation completes the next Replay timeline slice for
`rigid_kinematic_driver`. The shared Replay panel now uses IPC grip box travel
as its value track and marks contact/carry progress, slip-lane slip, and
static-like caveat frames for targeted visual debugging. Focused pytest,
sidecar/README drift guards, `pixi run lint`, `git diff --check`, and a real
docked capture passed.

The current continuation completes the next Replay timeline slice for
`rigid_kinematic_normal_push`. The shared Replay panel now uses target-travel
divergence as its value track and marks contact, IPC penetration, SI
push-progress, and divergence frames for targeted visual debugging. Focused
pytest, sidecar/README drift guards, `pixi run lint`, `git diff --check`, and a
real docked capture passed before the local commit
`3ea4d1fcd4e Add normal push replay timeline`.

The latest continuation completes the next Replay timeline slice for
`rigid_fixed_joint`. The shared Replay panel now uses fixed-joint offset error
as its value track and marks pose-error or residual-motion frames during
recovery. Focused pytest, sidecar/README drift guards, `pixi run lint`,
`git diff --check`, and a real docked capture passed before the local fixed-joint
commit.

The current continuation completes the next Replay timeline slice for
`rigid_joint_breakage`. The shared Replay panel now uses payload release
distance as its value track and marks broken or released frames during the AVBD
break-force lifecycle. Focused pytest, sidecar/README drift guards,
`pixi run lint`, `git diff --check`, and a real docked capture passed before
the local joint-breakage commit.

The latest continuation completes the next Replay timeline slice for
`rigid_distance_spring`. The shared Replay panel now uses maximum spring
stretch as its value track and marks high-stretch or off-center-spin frames.
Focused pytest, sidecar/README drift guards, `pixi run lint`,
`git diff --check`, and a real docked capture passed before the local
distance-spring commit.

The current continuation completes the next Replay timeline slice for
`rigid_limited_joints`. The shared Replay panel now uses locked-axis error as
its value track and marks locked-error or free-axis-motion frames. Focused
pytest, sidecar/README drift guards, `pixi run lint`, `git diff --check`, and a
real docked capture passed before the local limited-joints commit.

The latest continuation completes the next Replay timeline slice for
`rigid_joint_motor_limits`. The shared Replay panel now uses force travel gap as
its value track and marks velocity-clamp, position-stop, or effort-cap frames.
The maintained workflow capture budget is now 96 frames, so the generated
visual packet reaches the position stop as well as the velocity clamp and
effort-cap lanes. Focused pytest, sidecar/README drift guards, `pixi run lint`,
`git diff --check`, and a real docked capture passed before the local
motor-limits commit.

The current continuation completes the next Replay timeline slice for
`rigid_joint_passive_parameters`. The shared Replay panel now uses armature
position gap as its value track and marks damping energy separation, Coulomb
slip, or armature-lag frames. Focused pytest, sidecar/README drift guards,
`pixi run lint`, `git diff --check`, and a real docked capture passed before
the local passive-parameters commit.

The latest continuation completes the next Replay timeline slice for
`rigid_screw_joint_pitch`. The shared Replay panel now uses coarse/fine travel
gap as its value track and marks pitch-spread, zero-pitch contrast, or
reverse-sign frames. Focused pytest, sidecar/README drift guards,
`pixi run lint`, `git diff --check`, and a real docked capture passed before
the local screw-joint pitch commit.

The current continuation completes the next Replay timeline slice for
`rigid_link_jacobian` after the multibody dynamics-terms and link
center-of-mass slices. The shared Replay panel now uses link-origin speed as
its value track and marks high-twist, wrench-load, world/body Jacobian-gap, or
residual-alert frames. Focused pytest, sidecar/README drift guards, and a real
docked capture passed. `pixi run lint` passed, the focused pytest plus
sidecar/README drift guards passed again after lint, and `git diff --check` was
clean before committing this checkpoint.

The current continuation completes the next Replay timeline slice for
`rigid_multibody_solver_family`. The shared Replay panel now uses residual
solve ratio as its value track and marks solve-advantage, residual-only drift,
or solved-tight frames while residual rows remain loose. Focused pytest and a
real docked capture passed before this hand-off update.

The latest continuation completes the next Replay timeline slice for
`rigid_loop_closure`. The shared Replay panel now uses max closure residual
ratio as its value track and marks solve-advantage,
residual-versus-solved separation, distance-family tip drift, and
rigid-orientation frames. Focused pytest and a real docked capture passed
before the continuation resumed; drift guards, lint, and diff checks are
recorded below.

## Current Branch

`feature/rigid-body-gui-visual-verification`

Current snapshot:

- Latest local commit being handed off is titled
  `Add rigid workflow failed-row reruns`; it builds on the
  `Summarize rigid workflow failed rows` triage slice, the
  `Let rigid workflow packets continue after failures` failure-resilience slice,
  the row-range guidance commit, and the earlier heavy-packet implementation
  commit.
- The previous local slice refreshes the README/PLAN-103 row-range packet
  examples and guard for rows 47-48.
- The newest local slice adds `--continue-on-failure` for workflow packets.
- The current follow-up adds `failed_rows` to workflow manifests and a Failed
  Rows summary with workflow row-range rerun commands to `review_index.html`.
- The origin tip before these local slices was
  `bdf757db2c9 Refresh rigid handoff stop state`.
- The previous heavy-packet slice adds the verified
  `rigid_ipc_heavy_stack_packet` capture-first packet and was verified with
  focused tests, a dry-run, a real row capture, lint, and diff checks before
  its local commit.
- There is no PR associated with this branch at checkpoint time.
- No push was performed for this slice. Do not push without explicit approval
  in the session that performs the push.

## Immediate Next Step

A future session should inspect `git status -sb` and `git log -5 --oneline`
first. If the row-range guidance, failure-resilience, failed-row triage, and
failed-row rerun-command commits are present and the tree is clean, the next
decision returns to the completion/retirement readiness audit: either get
maintainer acceptance and prepare the completion cleanup PR, or choose the next
bounded slice from durable PLAN-103 follow-ups. If the failed-row rerun-command
follow-up is not in history, review the uncommitted diff and either commit or
intentionally discard it according to the user's latest instruction. Do not
push without explicit approval in the session that performs the push.

Replay capture-metadata checks for this slice:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_capture_metadata_projects_replay_timeline_for_manifests python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/unit/test_capture_py_demo.py::test_visual_capture_manifest_records_image_evidence python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_links_scene_videos python/tests/integration/test_demos_cycle.py::test_rigid_visual_replay_timeline_rows_publish_scene_metadata python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run py-demo-capture -- --scene rigid_loop_closure --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_loop_closure_replay_metadata_1781289000
pixi run py-demo-capture -- --rigid-workflow --workflow-start-row 36 --workflow-end-row 36 --output-dir /tmp/dart_capture_rigid_workflow_replay_metadata_row36_1781289001
pixi run lint
git diff --check
```

The post-lint focused suite reported `10 passed`. The real single-scene
capture recorded `scene_metadata.replay_timeline.signal_label` as
`Max closure residual ratio` with both signal and markers enabled; the one-row
workflow capture completed row 36 and the generated `review_index.html` showed
`Max closure residual ratio (signal, markers)`.

Full replay-metadata workflow refresh:

```bash
pixi run py-demo-capture -- --rigid-workflow --output-dir /tmp/dart_capture_rigid_workflow_replay_metadata_full_1781284053
jq -r '.status, .capture_count, .completed_count, .failed_count, .guidance_complete, .guidance_missing_count, .elapsed_s, .artifacts.review_index' /tmp/dart_capture_rigid_workflow_replay_metadata_full_1781284053/manifest.json
rg -c 'Replay timeline coverage' docs/plans/103-examples-strategy/rigid-body-visual-verification.md
find /tmp/dart_capture_rigid_workflow_replay_metadata_full_1781284053/scenes -name manifest.json -print0 | xargs -0 jq -r 'select(.scene_metadata.replay_timeline != null) | .scene' | wc -l
rg -o '<dt>replay</dt>' /tmp/dart_capture_rigid_workflow_replay_metadata_full_1781284053/review_index.html | wc -l
find /tmp/dart_capture_rigid_workflow_replay_metadata_full_1781284053/scenes -name manifest.json -print0 | xargs -0 jq -r 'select((.capture.converted_frames // 0) <= 0 or .visual_evidence.screenshot.docked_workspace != true or (.visual_evidence.screenshot.unique_rgb_count // 0) <= 1) | [.scene, (.capture.converted_frames|tostring), (.visual_evidence.screenshot.docked_workspace|tostring), (.visual_evidence.screenshot.unique_rgb_count|tostring)] | @tsv'
```

The full refresh reported `status=complete`, `capture_count=36`,
`completed_count=36`, `failed_count=0`, `guidance_complete=true`,
`guidance_missing_count=0`, and elapsed `310.791`. The sidecar,
per-scene manifests, and review index each reported nineteen Replay rows; the
per-scene manifest anomaly query printed no rows.

Optional extended-packet capture refresh:

```bash
DART_PARALLEL_JOBS=4 CTEST_PARALLEL_LEVEL=4 CMAKE_BUILD_PARALLEL_LEVEL=4 pixi run py-demo-capture -- --rigid-workflow --include-related --include-ipc-shelf --include-packets --workflow-start-row 37 --workflow-end-row 51 --output-dir /tmp/dart_capture_rigid_workflow_optional_rows_37_51_1781285053
jq -r '.status, .capture_count, .completed_count, .failed_count, .workflow_total_count, .workflow_row_start, .workflow_row_end, .include_related, .include_ipc_shelf, .include_packets, .selected_include_related, .selected_include_ipc_shelf, .selected_include_packets, .guidance_complete, .guidance_missing_count, .elapsed_s, .artifacts.review_index' /tmp/dart_capture_rigid_workflow_optional_rows_37_51_1781285053/manifest.json
find /tmp/dart_capture_rigid_workflow_optional_rows_37_51_1781285053/scenes -name manifest.json -print0 | xargs -0 jq -r 'select((.capture.converted_frames // 0) <= 0 or .visual_evidence.screenshot.docked_workspace != true or (.visual_evidence.screenshot.unique_rgb_count // 0) <= 1) | [.scene, (.capture.converted_frames|tostring), (.visual_evidence.screenshot.docked_workspace|tostring), (.visual_evidence.screenshot.unique_rgb_count|tostring)] | @tsv'
jq -r '.captures | group_by(.workflow_group)[] | [.[0].workflow_group, length] | @tsv' /tmp/dart_capture_rigid_workflow_optional_rows_37_51_1781285053/manifest.json
```

The optional packet reported `status=complete`, `capture_count=15`,
`completed_count=15`, `failed_count=0`, `workflow_total_count=51`, selected
rows `37-51`, all requested/selected related, IPC-shelf, and packet groups as
`true`, `guidance_complete=true`, `guidance_missing_count=0`, and elapsed
`149.609`. The per-scene manifest anomaly query printed no rows. The group
counts were ten `related_evidence`, four `rigid_ipc_shelf`, and one
`capture_first_packet`.

Passive-parameters checks for this slice:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_joint_passive_parameters_order_passive_response -q
pixi run py-demo-capture -- --scene rigid_joint_passive_parameters --frames 120 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_joint_passive_parameters_timeline_1781277900
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_joint_passive_parameters_order_passive_response python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
```

The focused passive-parameters test reported `1 passed`; the focused
Replay/passive-parameters pytest plus drift guards reported `6 passed`; and
the real docked capture wrote a nonblank 960x540 screenshot with docked UI,
119 PNG frames, and 120 scene-metric events. The manifest recorded row
`rigid_joint_passive_parameters`, spring energy about `2.1583`, damped energy
about `1.3403`, damped-energy ratio about `0.6210`, stiction position `0.0`,
slip position about `0.1742` m, slip speed about `0.7200` m/s, armature
acceleration gap about `2.25` m/s^2, and armature position gap about `0.2614`
m. `pixi run lint` passed and `git diff --check` was clean.

Screw-joint pitch checks for this slice:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_screw_joint_pitch_couples_rotation_and_translation python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run py-demo-capture -- --scene rigid_screw_joint_pitch --frames 96 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_screw_joint_pitch_timeline_1781278553
pixi run lint
git diff --check
```

The focused Replay/screw-joint pytest plus drift guards reported `6 passed`;
the real docked capture wrote a nonblank 960x540 screenshot with docked UI,
95 PNG frames, and 96 scene-metric events. The manifest recorded row
`rigid_screw_joint_pitch`, fine angle about `-0.8317` rad, coarse angle about
`-0.6162` rad, reverse angle about `0.8317` rad, fine axial travel about
`-0.2329` m, coarse axial travel about `-0.3451` m, reverse axial travel about
`-0.2329` m, coarse/fine travel gap about `0.1122` m, and travel-per-radian
values `0.28`, `0.56`, and `-0.28`. `pixi run lint` passed and
`git diff --check` was clean.

Multibody dynamics-terms checks for this slice:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_multibody_dynamics_terms_expose_generalized_terms python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run py-demo-capture -- --scene rigid_multibody_dynamics_terms --frames 96 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_multibody_dynamics_terms_timeline_1781279136
```

The focused Replay/multibody-dynamics pytest plus drift guards reported
`6 passed`; the real docked capture wrote a nonblank 960x540 screenshot with
docked UI, 95 PNG frames, and 96 scene-metric events. The manifest recorded row
`rigid_multibody_dynamics_terms`, coupled response norm about `15.46`, heavy
response norm about `8.63`, heavy/coupled response ratio about `0.558`,
heavy-minus-coupled torque norm about `18.14`, coupled/heavy off-diagonal
coupling about `0.357`/`1.427`, max impulse residual about `2.7e-14`, and
historical max coupling about `0.373`.

Link center-of-mass checks for this slice:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_link_center_of_mass_offsets_gravity_torque python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run py-demo-capture -- --scene rigid_link_center_of_mass --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_link_center_of_mass_timeline_1781279455
```

The focused Replay/link-center-of-mass pytest plus drift guards reported
`6 passed`; the real docked capture wrote a nonblank 960x540 screenshot with
docked UI, 71 PNG frames, and 72 scene-metric events. The manifest recorded row
`rigid_link_center_of_mass`, mirrored positive/negative angles about
`+/-0.449`, high-inertia angle about `0.153`, centered angle `0.0`, mirrored
torques about `+/-3.182`, high/positive acceleration ratio about `0.370`,
high/positive mass-matrix ratio about `2.948`, and historical max
positive/high-inertia angles about `0.449`/`0.153`.

Link-Jacobian checks for this slice:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_link_jacobian_maps_link_origin_twist_and_wrench python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run py-demo-capture -- --scene rigid_link_jacobian --frames 96 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_link_jacobian_timeline_1781280127
```

The focused Replay/link-Jacobian pytest plus drift guards reported `6 passed`;
the real docked capture wrote a nonblank 960x540 screenshot with docked UI,
95 PNG frames, and 96 scene-metric events. The manifest recorded row
`rigid_link_jacobian`, latest linear/angular speed about `0.5652`/`0.6190`,
finite-difference error about `1.52e-7`, power error `0`, world/body Jacobian
gap about `0.1272`, torques about `-0.8650`/`-0.2949`, matching joint/wrench
power about `-0.4212`, historical max link speed about `0.6677`, historical
max world/body gap about `0.2055`, and historical max absolute torques about
`0.8650`/`0.2949`.

## Context That Would Be Lost

- The Replay capture-metadata slice deliberately serializes only a JSON-safe
  projection of `info["replay_timeline"]`: `signal_label`, `has_signal`,
  `has_markers`, and panel name. Do not try to serialize the `signal` or
  `markers` callables in manifests.
- The capture helper cannot inspect `SceneSetup.info` from the parent workflow
  process because each row capture runs as a child process. The runner emits
  `scene_capture_metadata` through the existing `scene_metrics.jsonl` path, and
  `scripts/capture_py_demo.py` stores the latest metadata event in
  `manifest.json` as `scene_metadata`.
- The completed multibody solver-family Replay timeline slice adds
  `_last_float(...)`,
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata to
  `python/examples/demos/scenes/rigid_multibody_solver_family.py`. The value
  track is `Residual solve ratio`.
- The row 35 test edit extends
  `test_rigid_multibody_solver_family_routes_solved_closures` with timeline
  label, signal fallback, solve-ratio marker, residual-drift marker,
  tip-error marker, solved-tight marker, and quiet-frame assertions.
- A read-only row 35 explorer recommended computing the signal from latest
  `solve_ratio_history`, falling back to top-level `residual_solve_ratio`, and
  then to the residual-only over solved residual ratio from `last_metrics`. It
  recommended
  markers for ratio at least `1e8`, residual-only drift at least `0.5`, and
  solved-tight frames where solved residual/tip error is at most `1e-8` while
  residual-only rows remain at least `0.25`.
- Sampling during row 35 exploration showed the residual solve ratio climbing
  above `1e12` while solved residuals dropped near machine precision and
  residual-only lanes stayed visibly loose. That sampling was exploratory
  context; final evidence for this slice is recorded in the current validation
  notes.
- The task goal is not just to add examples; it is to make rigid-body GUI demos
  useful as visual debugging and verification surfaces for solvers, backends,
  parameters, contact behavior, constraints, and corner cases.
- Durable planning context lives in
  `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`.
- That sidecar already documents the 36 rigid workflow rows and current API
  limitations.
- Public `dartpy` is currently believed to lack direct rigid-body impulse,
  sleep/wake/island activation, and loop-compliance APIs. Do not build rows
  around those features without first confirming the API surface changed.
- A read-only explorer subagent was spawned before the stop instruction, but
  the stop instruction superseded waiting for or consuming its result. It later
  returned and identified the workflow-level capture command as the highest
  value bounded follow-up.
- A second read-only explorer identified missing search terms such as
  `RigidBodySolver`, `ContactSolverMethod`, `backend/executor`, and
  `contact model`; those terms are now covered by the workflow search aliases.
- The previous cycle fix changed C++ GUI runner behavior, Python integration
  tests, and docs. It was validated before the stop request, not after.
- The full workflow capture command completed all 36 numbered rigid captures.
  Its top-level manifest reported `status=complete`, `capture_count=36`,
  `completed_count=36`, `failed_count=0`, and all 36 per-scene manifests
  existed.
- The full run exposed parent progress buffering; `scripts/capture_py_demo.py`
  now flushes workflow progress and manifest lines.
- The workflow runner now treats a missing per-scene manifest as
  `failure_reason: missing_manifest` and exits nonzero, even if the child
  process returned 0.
- The in-viewer workflow panel now closes the loop from interactive debugging
  to reproducible visual evidence by printing the row-specific capture command.
- The comparison-contract labels intentionally separate solver family,
  executor-only, contact policy, time-step multiplier, workload shape/contact
  workload, and passive joint parameter family axes so users do not conflate
  backend, solver, policy, and parameter changes.
- The workflow review index is intentionally generated beside the manifest,
  not checked in. `manifest.json` records it under `artifacts.review_index`.
- The related-evidence bundle is intentionally opt-in so default
  `--rigid-workflow` still captures exactly the 36 numbered rows. The
  `--include-related` variant appends the related routes as rows 37-46 in the
  generated review packet.
- The capture-first packet bundle is also opt-in. `--include-packets` appends
  `rigid_ipc_stack_packet` and `rigid_ipc_heavy_stack_packet` after the
  numbered rows and, when present, after related-evidence and direct Rigid IPC
  shelf rows.
- Row-range selection is intended for targeted reruns after a long workflow
  packet fails or needs manual inspection. It keeps absolute row labels and
  output directories, so selected packet row 47 appears as `47/47` and writes to
  `scenes/47_rigid_ipc_stack_packet`.
- The review index now mirrors the numbered rows' in-viewer guidance so the
  capture packet answers "what am I looking at?" without opening the live GUI
  or sidecar table.
- `rigid_ipc_edge_drop` stays outside the numbered World Rigid Body workflow;
  it is an IPC-specific related capability scene, not a new row in the 36-row
  workflow.
- `d5c6de2bee1 Describe optional rigid workflow rows` makes optional
  related-evidence, direct Rigid IPC shelf, and capture-first packet rows
  self-describing in generated manifests and review indexes. It is local and
  unpushed at this checkpoint.
- The current guidance-audit worktree adds `guidance_complete`,
  `guidance_missing_count`, and `guidance_missing_rows` to workflow manifests,
  plus a `guidance` badge and missing-guidance warning in the review index.
- A later read-only explorer recommended this guidance-audit shape:
  non-failing completeness reporting over selected captures, required fields
  for role label, user question, try-first action, inspect signals, healthy
  signal, and scope, plus manifest fields and review-index warning surfacing
  missing guidance.
- The completed live open-command slice adds
  `_rigid_workflow_viewer_command(scene_id, width, height)` in
  `python/examples/demos/runner.py`, writes `viewer_command` entries from
  `scripts/capture_py_demo.py`, renders `open live` and `capture evidence`
  command blocks in generated review cards, and updates
  `python/tests/unit/test_py_demo_panels.py` and
  `python/tests/unit/test_capture_py_demo.py` for panel, manifest,
  review-index, and backend-propagation coverage.
- The latest read-only explorer returned after the stop instruction and was
  later acted on. It recommended a bounded slice for
  `rigid_stack_stability`: add replay-guided timeline metadata for a
  `Top x divergence` value track and diagnostic markers for overlap/collapse,
  low clearance, or visible top-block drift.
- The completed stack Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata to
  `python/examples/demos/scenes/rigid_stack_stability.py`, with tests covering
  the `Top x divergence` label, signal, and marker thresholds.
- The completed contact-manipulation Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata to
  `python/examples/demos/scenes/rigid_contact_manipulation.py`, with tests and
  capture evidence for the `Travel divergence` value track and
  contact/proximity/progress markers.
- The completed kinematic-driver Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata to
  `python/examples/demos/scenes/rigid_kinematic_driver.py`, with tests and
  capture evidence for the `IPC grip box travel` value track and contact,
  carry, slip, and caveat markers.
- The completed normal-push Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata to
  `python/examples/demos/scenes/rigid_kinematic_normal_push.py`, with tests and
  capture evidence for the `Target travel divergence` value track and contact,
  penetration, SI push, and divergence markers.
- The completed fixed-joint Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata to
  `python/examples/demos/scenes/rigid_fixed_joint.py`, with tests and capture
  evidence for the `Fixed-joint offset error` value track and pose-error or
  residual-motion recovery markers.
- The completed joint-breakage Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`,
  `replay_capture_state`, `replay_restore_state`, and
  `info["replay_timeline"]` metadata to the shared AVBD breakable fixed-joint
  builder used by `rigid_joint_breakage`, with tests and capture evidence for
  the `Payload release distance` value track and broken/released markers.
- The completed distance-spring Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata to
  `python/examples/demos/scenes/rigid_distance_spring.py`, with tests for the
  `Max spring stretch` value track and high-stretch or off-center-spin markers.
  It has focused pytest, drift-guard, docked-capture, lint, and diff-check
  evidence.
- The completed limited-joints Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata to
  `python/examples/demos/scenes/rigid_limited_joints.py`, with tests for the
  `Locked-axis error` value track and locked-error/free-axis-motion markers. It
  has focused pytest, drift-guard, docked-capture, lint, and diff-check
  evidence.
- The completed motor-limits Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata to
  `python/examples/demos/scenes/rigid_joint_motor_limits.py`, with tests for the
  `Force travel gap` value track and velocity-clamp, position-stop, and
  effort-cap markers. It has focused pytest, drift-guard, docked-capture, lint,
  and diff-check evidence.
- The completed passive-parameters Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata to
  `python/examples/demos/scenes/rigid_joint_passive_parameters.py`, with tests
  for the `Armature position gap` value track and damping, Coulomb slip, and
  armature-lag markers. It has focused pytest, drift-guard, docked-capture,
  lint, and diff-check evidence.
- The completed screw-joint pitch Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata to
  `python/examples/demos/scenes/rigid_screw_joint_pitch.py`, with tests for the
  `Coarse/fine travel gap` value track and pitch-spread, zero-pitch contrast,
  and reverse-sign markers. It has focused pytest, drift-guard,
  docked-capture, lint, and diff-check evidence.
- The completed link-Jacobian Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata to
  `python/examples/demos/scenes/rigid_link_jacobian.py`, with tests for the
  `Link-origin speed` value track and high-twist, wrench-load, world/body-gap,
  and residual-alert markers.
- The completed multibody solver-family Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata to
  `python/examples/demos/scenes/rigid_multibody_solver_family.py`, with tests
  for the `Residual solve ratio` value track and solve-advantage,
  residual-only-drift, tip-error, solved-tight, fallback, and quiet-frame
  behavior. Its real docked capture wrote 71 PNG frames and 72 scene-metric
  events under
  `/tmp/dart_capture_multibody_solver_family_timeline_1781281303`.
- The completed row 36 slice in
  `python/examples/demos/scenes/rigid_loop_closure.py` adds `_last_float(...)`,
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata. The intended value track is
  `Max closure residual ratio`.
- The row 36 test edit in
  `python/tests/integration/test_demos_cycle.py` extends
  `test_rigid_loop_closure_compares_closure_families` with label, current
  snapshot signal, top-level and `last_metrics` signal fallback, solve-ratio
  marker, residual marker, rigid-orientation marker, distance-tip marker, and
  quiet-frame assertions.
- Row 36 marker intent: flag large closure solve advantage, residual-versus-
  solved closure separation, distance-family tip drift with near-zero distance
  error, and rigid-orientation solve frames.

## How To Resume

```bash
git checkout feature/rigid-body-gui-visual-verification
git status -sb
git log -3 --oneline
```

Then read:

```bash
docs/ai/principles.md
docs/dev_tasks/README.md
docs/dev_tasks/rigid_body_visual_verification/README.md
docs/plans/103-examples-strategy/rigid-body-visual-verification.md
```

If committing in the next session, first run the repository-mandated lint:

```bash
pixi run lint
```

Current multibody solver-family validation already run:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_multibody_solver_family_routes_solved_closures -q
pixi run py-demo-capture -- --scene rigid_multibody_solver_family --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_multibody_solver_family_timeline_1781281303
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_multibody_solver_family_routes_solved_closures python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run lint
```

The row 35 pytest reported `1 passed`; the focused Replay/row 35 pytest plus
sidecar/README/capture-command drift guards reported `6 passed`; and the
capture wrote a nonblank 960x540 docked screenshot, 71 PNG frames, and
72 scene-metric events under
`/tmp/dart_capture_multibody_solver_family_timeline_1781281303`. `pixi run
lint` passed.

Current loop-closure validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_loop_closure_compares_closure_families -q
pixi run py-demo-capture -- --scene rigid_loop_closure --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_loop_closure_timeline_1781285000
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_loop_closure_compares_closure_families python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run lint
git diff --check
```

The row 36 focused pytest reported `1 passed`; the focused Replay/row 36
pytest plus sidecar/README/capture-command drift guards reported `6 passed`;
the real docked capture wrote a nonblank 960x540 docked screenshot, 71 PNG
frames, and 72 scene-metric events under
`/tmp/dart_capture_loop_closure_timeline_1781285000`. The manifest recorded row
`rigid_loop_closure`, solver
`variational_rigid_multibody_loop_closure`, scope
`point_distance_rigid_closure_family_selection`, executor `Sequential`, gravity
scale `1.0`, six cases, point/distance/rigid residual ratios about
`7.595e11`, `7.458e11`, and `7.740e11`, rigid residual orientation error about
`0.149`, solved orientation error near `2.8e-17`, distance solved distance
error near `9.4e-15`, and distance solved tip error about `0.439`.
`pixi run lint` passed and `git diff --check` was clean.

Focused validation already run:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_scene_capture_runs_in_child_process python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run py-demo-capture -- --rigid-workflow --dry-run --output-dir /tmp/dart_capture_rigid_workflow_dry_run
```

The focused pytest reported `6 passed`. The dry-run manifest reported workflow
`rigid_visual_verification`, status `planned`, and 36 planned captures.

Current-continuation validation already run:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_fails_when_scene_manifest_is_missing python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_scene_capture_runs_in_child_process python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run py-demo-capture -- --rigid-workflow --dry-run --output-dir /tmp/dart_capture_rigid_workflow_dry_run
pixi run lint
git diff --check
```

The focused pytest reported `7 passed`. The dry-run command printed all
36 planned capture commands. `pixi run lint` passed and `git diff --check` was
clean.

Latest focused validation already run:

```bash
pixi run lint
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_comparison_panels_label_the_compared_axis python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_guides_expose_capture_commands python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_fails_when_scene_manifest_is_missing python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_scene_capture_runs_in_child_process python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_timestep_sensitivity_orders_freefall_error_by_step_size python/tests/integration/test_demos_cycle.py::test_rigid_step_diagnostics_reports_profile_and_memory_counters python/tests/integration/test_demos_cycle.py::test_rigid_contact_scale_budget_orders_contact_loads python/tests/integration/test_demos_cycle.py::test_rigid_joint_passive_parameters_order_passive_response -q
pixi run py-demo-capture -- --rigid-workflow --dry-run --output-dir /tmp/dart_capture_rigid_workflow_dry_run
test -f /tmp/dart_capture_rigid_workflow_dry_run/review_index.html && jq -r '.artifacts.review_index, .capture_count, .status' /tmp/dart_capture_rigid_workflow_dry_run/manifest.json && rg -n "DART rigid workflow review index|rigid_loop_closure" /tmp/dart_capture_rigid_workflow_dry_run/review_index.html
git diff --check
```

The focused pytest reported `14 passed`. The dry-run printed all 36 planned
capture commands. The dry-run manifest reported
`/tmp/dart_capture_rigid_workflow_dry_run/review_index.html`, `36`, and
`planned`, and the generated index contained the final
`36/36 rigid_loop_closure` row. `pixi run lint` passed and `git diff --check`
was clean.

Full workflow capture with the review index already run:

```bash
pixi run py-demo-capture -- --rigid-workflow --output-dir /tmp/dart_capture_rigid_workflow_full_review_index_1781259714
```

The command exited 0. The manifest at
`/tmp/dart_capture_rigid_workflow_full_review_index_1781259714/manifest.json`
reported `status=complete`, `capture_count=36`, `completed_count=36`,
`failed_count=0`, and `elapsed_s=314.278`. Every capture entry had
`return_code=0`, `status=captured`, and `manifest_exists=true`. The generated
review index at
`/tmp/dart_capture_rigid_workflow_full_review_index_1781259714/review_index.html`
contained 36 screenshot thumbnails and linked the first/last screenshots plus
comparison-axis and held-fixed metric summaries for representative rows.

Current related-search routing validation already run:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_opens_related_evidence_search_matches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_related_evidence_targets python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_related_evidence_routes_to_other_shelves -q
pixi run lint
git diff --check
```

The focused pytest reported `3 passed`. `pixi run lint` passed and
`git diff --check` was clean.

Current related-evidence capture-bundle validation already run:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_related_evidence python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_include_related_requires_workflow python/tests/integration/test_demos_cycle.py::test_rigid_visual_related_evidence_capture_commands_are_documented -q
pixi run py-demo-capture -- --rigid-workflow --include-related --dry-run --output-dir /tmp/dart_capture_rigid_workflow_related_dry_run_current
jq -r '.include_related, .capture_count, .captures[36].workflow_group, .captures[36].scene, .captures[45].scene, .artifacts.review_index' /tmp/dart_capture_rigid_workflow_related_dry_run_current/manifest.json
rg -n "related_evidence|avbd_rigid_prismatic_motor|46/46" /tmp/dart_capture_rigid_workflow_related_dry_run_current/review_index.html
```

The focused pytest reported `5 passed`. The public dry-run completed with
46 planned capture commands, the manifest reported `include_related=true` and
`capture_count=46`, and the generated review index contained the final
`46/46 avbd_rigid_prismatic_motor` related-evidence row.

Current capture-first packet bundle validation already run:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_related_evidence python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_capture_first_packets python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_extra_groups_require_workflow python/tests/integration/test_demos_cycle.py::test_rigid_visual_related_evidence_capture_commands_are_documented python/tests/integration/test_demos_cycle.py::test_rigid_visual_capture_first_packets_are_documented -q
pixi run py-demo-capture -- --rigid-workflow --include-related --include-packets --dry-run --output-dir /tmp/dart_capture_rigid_workflow_packets_edge_drop_dry_run
jq -r '.include_related, .include_packets, .capture_count, .captures[46].workflow_group, .captures[46].scene, .artifacts.review_index' /tmp/dart_capture_rigid_workflow_packets_edge_drop_dry_run/manifest.json
rg -n "capture_first_packet|rigid_ipc_stack_packet|47/47" /tmp/dart_capture_rigid_workflow_packets_edge_drop_dry_run/review_index.html
```

The focused pytest reported `8 passed`. The public dry-run completed with
47 planned capture commands, the manifest reported `include_related=true`,
`include_packets=true`, `capture_count=47`, and the generated review index
contained the final `47/47 rigid_ipc_stack_packet` capture-first packet row.

Current row-range workflow rerun validation already run:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_select_row_range python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_can_resume_from_selected_row python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_extra_groups_require_workflow python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_row_selection_validates_bounds -q
pixi run py-demo-capture -- --rigid-workflow --include-related --include-packets --workflow-start-row 47 --workflow-end-row 47 --dry-run --output-dir /tmp/dart_capture_rigid_workflow_row_rerun_edge_drop_dry_run
jq -r '.capture_count, .workflow_total_count, .workflow_row_start, .workflow_row_end, .captures[0].order, .captures[0].count, .captures[0].scene, .captures[0].workflow_group' /tmp/dart_capture_rigid_workflow_row_rerun_edge_drop_dry_run/manifest.json
rg -n "47/47|rigid_ipc_stack_packet|capture_first_packet" /tmp/dart_capture_rigid_workflow_row_rerun_edge_drop_dry_run/review_index.html
```

The focused pytest reported `11 passed`. The public dry-run completed with
one selected row-47 command, the manifest reported `capture_count=1`,
`workflow_total_count=47`, selected row order/count `47/47`, and the generated
review index contained the absolute `47/47 rigid_ipc_stack_packet` row.

Current review-index guidance validation already run:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_select_row_range python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_can_resume_from_selected_row python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_extra_groups_require_workflow python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_row_selection_validates_bounds -q
pixi run py-demo-capture -- --rigid-workflow --workflow-start-row 15 --workflow-end-row 15 --dry-run --output-dir /tmp/dart_capture_rigid_workflow_guidance_dry_run
jq -r '.captures[0].workflow_label, .captures[0].user_question, .captures[0].try_first, .captures[0].healthy_signal, .captures[0].scope' /tmp/dart_capture_rigid_workflow_guidance_dry_run/manifest.json
rg -n "How do the rigid method families differ visually|Healthy: wall clearance|Generic thin-wall comparison" /tmp/dart_capture_rigid_workflow_guidance_dry_run/review_index.html
```

The focused pytest reported `11 passed`. The public row-15 dry-run completed,
the manifest reported the `Solver family` role plus the maintained user
question, try-first guidance, healthy signal, and scope note, and the generated
review index contained the same solver-family guidance beside the row card.

Current rigid IPC edge-drop validation already run:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_ipc_edge_drop_reports_degenerate_contact_metrics python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_related_evidence_routes_are_valid python/tests/integration/test_demos_cycle.py::test_rigid_visual_routes_publish_self_describing_capture_metrics python/tests/integration/test_demos_cycle.py::test_rigid_visual_related_evidence_capture_commands_are_documented python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_related_evidence_routes_to_other_shelves python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_related_evidence_targets python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_opens_related_evidence_search_matches -q
```

The focused pytest reported `9 passed`. The new edge-drop route reports
`row=rigid_ipc_edge_drop`, `related_source_row=rigid_solver_compare`,
`solver=rigid_ipc`, `scope=degenerate_edge_contact_capability`, near-barrier
clearance, and nonzero angular/tilt motion. The related route table, capture
command list, capture-helper specs, panel coverage, search routing, direct
related route, and unnumbered-route guard all include `rigid_ipc_edge_drop`.
The public `--include-related` dry-run reported `capture_count=46`,
`40/46 rigid_ipc_edge_drop`, and final `46/46 avbd_rigid_prismatic_motor`. The
public row-range packet dry-run reported `workflow_total_count=47` and
`47/47 rigid_ipc_stack_packet`. The real docked capture wrote a nonblank
960x540 screenshot with docked UI, 71 converted PNG frames, latest status
`edge-barrier`, minimum barrier gap about `0.000384` m, maximum tilt about
`55.33` degrees, and maximum angular speed about `0.555` rad/s.

Current Rigid IPC shelf metrics validation already run:

```bash
pixi run lint
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_ipc_shelf_scenes_report_capture_metrics python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q
pixi run py-demo-capture -- --scene rigid_ipc_pile --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_rigid_ipc_pile_metrics_current
git diff --check
```

`pixi run lint` passed. The focused pytest reported `2 passed`. The direct
Rigid IPC shelf scenes
`rigid_ipc`, `rigid_ipc_slide`, `rigid_ipc_incline`, and `rigid_ipc_pile` now
report scene-owned capture metrics for row identity, IPC solver label, scope,
time-step/world-time, friction, status, contact counts, step timing, and
scene-specific gap, speed, travel, height, span, or pile summaries while
preserving shared replay controls.
A real docked
`pixi run py-demo-capture -- --scene rigid_ipc_pile --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_rigid_ipc_pile_metrics_current`
capture wrote a 960x540 screenshot with docked UI detected, 23 converted PNG
frames, 24 scene-metric events, latest row `rigid_ipc_pile`, scope
`multi_box_barrier_pile`, 25 history samples, `box_count=3`, maximum history
speed about `1.177` m/s, and minimum history clearance about `0.276` m.
`git diff --check` was clean.

Current direct Rigid IPC shelf capture-bundle validation already run:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_related_evidence python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_direct_ipc_shelf python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_capture_first_packets python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_extra_groups_require_workflow python/tests/integration/test_demos_cycle.py::test_rigid_visual_related_evidence_capture_commands_are_documented python/tests/integration/test_demos_cycle.py::test_rigid_visual_capture_first_packets_are_documented python/tests/integration/test_demos_cycle.py::test_rigid_visual_direct_ipc_shelf_captures_are_documented -q
pixi run py-demo-capture -- --rigid-workflow --include-ipc-shelf --dry-run --output-dir /tmp/dart_capture_rigid_workflow_ipc_shelf_only_dry_run_current
jq -r '.include_related, .include_ipc_shelf, .include_packets, .capture_count, .workflow_total_count, .captures[36].workflow_group, .captures[36].scene, .captures[39].scene' /tmp/dart_capture_rigid_workflow_ipc_shelf_only_dry_run_current/manifest.json
rg -n "37/40 rigid_ipc|40/40 rigid_ipc_pile|rigid_ipc_shelf" /tmp/dart_capture_rigid_workflow_ipc_shelf_only_dry_run_current/review_index.html
pixi run py-demo-capture -- --rigid-workflow --include-related --include-ipc-shelf --include-packets --workflow-start-row 47 --workflow-end-row 51 --dry-run --output-dir /tmp/dart_capture_rigid_workflow_ipc_shelf_dry_run_current
jq -r '.include_related, .include_ipc_shelf, .include_packets, .capture_count, .workflow_total_count, .captures[0].order, .captures[0].workflow_group, .captures[0].scene, .captures[3].scene, .captures[4].workflow_group, .captures[4].scene' /tmp/dart_capture_rigid_workflow_ipc_shelf_dry_run_current/manifest.json
rg -n "47/51 rigid_ipc|50/51 rigid_ipc_pile|51/51 rigid_ipc_stack_packet|rigid_ipc_shelf" /tmp/dart_capture_rigid_workflow_ipc_shelf_dry_run_current/review_index.html
```

The focused pytest reported `11 passed`. The public `--include-ipc-shelf`
dry-run completed with 40 planned captures, the manifest reported
`include_ipc_shelf=true`, `capture_count=40`, `workflow_total_count=40`, first
IPC shelf scene `rigid_ipc`, final IPC shelf scene `rigid_ipc_pile`, and
`workflow_group=rigid_ipc_shelf`. The generated review index contained
`37/40 rigid_ipc`, `40/40 rigid_ipc_pile`, and the `rigid_ipc_shelf` group. The
combined row-range dry-run with related evidence, IPC shelf rows, and packets
selected rows 47-51; it reported the direct IPC shelf rows as absolute 47-50
and `rigid_ipc_stack_packet` as the explicit combined-mode row 51.

Current workflow manifest and review index metadata validation already run:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_related_evidence python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_direct_ipc_shelf python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_capture_first_packets python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_select_row_range python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_row_range_preserves_requested_extra_groups python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_can_resume_from_selected_row python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_extra_groups_require_workflow python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_row_selection_validates_bounds -q
pixi run py-demo-capture -- --rigid-workflow --include-related --include-ipc-shelf --include-packets --workflow-start-row 47 --workflow-end-row 51 --dry-run --output-dir /tmp/dart_capture_rigid_workflow_requested_groups_dry_run_current
jq -r '.include_related, .include_ipc_shelf, .include_packets, .selected_include_related, .selected_include_ipc_shelf, .selected_include_packets, .capture_count, .workflow_total_count, .workflow_row_start, .workflow_row_end, .captures[0].workflow_group, .captures[0].scene, .captures[4].workflow_group, .captures[4].scene' /tmp/dart_capture_rigid_workflow_requested_groups_dry_run_current/manifest.json
rg -n "47/51 rigid_ipc|50/51 rigid_ipc_pile|51/51 rigid_ipc_stack_packet|rigid_ipc_shelf|capture_first_packet" /tmp/dart_capture_rigid_workflow_requested_groups_dry_run_current/review_index.html
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_row_range_preserves_requested_extra_groups python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_capture_first_packets python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests -q
pixi run py-demo-capture -- --rigid-workflow --include-related --include-ipc-shelf --include-packets --workflow-start-row 47 --workflow-end-row 51 --dry-run --output-dir /tmp/dart_capture_rigid_workflow_review_index_groups_dry_run_current
jq -r '.include_related, .include_ipc_shelf, .include_packets, .selected_include_related, .selected_include_ipc_shelf, .selected_include_packets, .capture_count, .workflow_total_count, .workflow_row_start, .workflow_row_end' /tmp/dart_capture_rigid_workflow_review_index_groups_dry_run_current/manifest.json
rg -n "<strong>rows</strong> 47-51 / 51|<strong>requested groups</strong> numbered, related, ipc shelf, packets|<strong>selected groups</strong> ipc shelf, packets|47/51 rigid_ipc|51/51 rigid_ipc_stack_packet" /tmp/dart_capture_rigid_workflow_review_index_groups_dry_run_current/review_index.html
pixi run lint
git diff --check
```

The focused pytest reported `16 passed` for manifest metadata and `4 passed`
for the review-index metadata follow-up. The public combined row-range dry-run
selected rows 47-51 and wrote requested flags `include_related=true`,
`include_ipc_shelf=true`, and `include_packets=true` while separately
reporting `selected_include_related=false`,
`selected_include_ipc_shelf=true`, and `selected_include_packets=true`.
The manifest also reported `capture_count=5`, `workflow_total_count=51`,
`workflow_row_start=47`, `workflow_row_end=51`, first selected scene
`rigid_ipc`, and final selected scene `rigid_ipc_stack_packet`. The generated
review index header contained row-span `47-51 / 51`, requested-groups
`numbered, related, ipc shelf, packets`, and selected-groups
`ipc shelf, packets`. `pixi run lint` passed and `git diff --check` was clean.

Current workflow panel packet-command validation already run:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_guides_expose_capture_commands python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_route_rows_request_scene_switches -q
pixi run lint
```

The focused panel pytest reported `3 passed`. The tested `Rigid Workflow`
panel now emits the per-row capture command plus a `Review packet` section
containing the full numbered workflow command, the current-row row-range rerun
command, and the extended related/IPC-shelf/packet command. `pixi run lint`
passed. `git diff --check` was not rerun after the final handoff edits because
the user explicitly requested no further verification.

Current workflow video packet validation already run:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_visual_capture_manifest_records_video_artifact python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_request_video_commands python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_links_scene_videos python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/integration/test_demos_cycle.py::test_rigid_visual_motion_capture_video_flags_are_documented -q
DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run py-demo-capture -- --rigid-workflow --workflow-start-row 15 --workflow-end-row 15 --video --fps 24 --dry-run --output-dir /tmp/dart_capture_rigid_workflow_video_dry_run_current
jq -r '.capture_count, .workflow_total_count, .workflow_row_start, .workflow_row_end, .captures[0].command' /tmp/dart_capture_rigid_workflow_video_dry_run_current/manifest.json
rg -n "15/36 rigid_solver_compare|--video --fps 24|selected groups|requested groups" /tmp/dart_capture_rigid_workflow_video_dry_run_current/review_index.html
pixi run lint
git diff --check
```

The focused pytest reported `5 passed`. The public row-15 workflow dry-run
completed with exit code 0, generated a selected-row command ending in
`--show-ui --video --fps 24`, and wrote a manifest with `capture_count=1`,
`workflow_total_count=36`, `workflow_row_start=15`, and `workflow_row_end=15`.
The generated review index contained row `15/36 rigid_solver_compare`,
requested/selected group badges, and the `--video --fps 24` command.
`pixi run lint` passed and `git diff --check` was clean.

Current guidance-audit validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_related_evidence python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_direct_ipc_shelf python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_capture_first_packets python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_row_range_preserves_requested_extra_groups python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_full_extended_plan_has_complete_guidance python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_manifest_reports_missing_guidance -q
pixi run py-demo-capture -- --rigid-workflow --include-related --include-ipc-shelf --include-packets --workflow-start-row 37 --workflow-end-row 51 --dry-run --output-dir /tmp/dart_capture_rigid_workflow_guidance_audit_dry_run_current
jq -r '.guidance_complete, .guidance_missing_count, (.guidance_missing_rows | length), .capture_count, .workflow_total_count, .workflow_row_start, .workflow_row_end, .captures[0].workflow_label, .captures[10].workflow_label, .captures[14].workflow_label' /tmp/dart_capture_rigid_workflow_guidance_audit_dry_run_current/manifest.json
rg -n "<strong>guidance</strong> complete|Rows Missing Guidance|Related evidence|Rigid IPC shelf|Capture-first packet|37/51 floating_base|51/51 rigid_ipc_stack_packet" /tmp/dart_capture_rigid_workflow_guidance_audit_dry_run_current/review_index.html
```

The focused pytest reported `7 passed`. The public dry-run selected rows 37-51
from the fully extended packet and wrote `capture_count=15`,
`workflow_total_count=51`, `workflow_row_start=37`, and `workflow_row_end=51`.
The generated manifest reported `guidance_complete=true`,
`guidance_missing_count=0`, and an empty `guidance_missing_rows` list, with
row labels for `Related evidence`, `Rigid IPC shelf`, and
`Capture-first packet`. The generated review index contained
`<strong>guidance</strong> complete`, the absolute `37/51 floating_base` and
`51/51 rigid_ipc_stack_packet` rows, and no `Rows Missing Guidance` warning.

Current live open-command validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_request_video_commands python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_full_extended_plan_has_complete_guidance python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_manifest_reports_missing_guidance -q
pixi run py-demo-capture -- --rigid-workflow --workflow-start-row 15 --workflow-end-row 15 --dry-run --output-dir /tmp/dart_capture_rigid_workflow_live_command_dry_run_current_1781271290
jq -r '.capture_count, .workflow_total_count, .workflow_row_start, .workflow_row_end, .captures[0].scene, .captures[0].viewer_command, .captures[0].command' /tmp/dart_capture_rigid_workflow_live_command_dry_run_current_1781271290/manifest.json
rg -n "open live|capture evidence|pixi run py-demos -- --scene rigid_solver_compare --width 960 --height 540|pixi run py-demo-capture -- --scene rigid_solver_compare" /tmp/dart_capture_rigid_workflow_live_command_dry_run_current_1781271290/review_index.html
pixi run lint
```

The focused pytest reported `5 passed` before and after `pixi run lint`
reformatted `scripts/capture_py_demo.py`. The public row-15 dry-run wrote
`capture_count=1`, `workflow_total_count=36`, `workflow_row_start=15`,
`workflow_row_end=15`, scene `rigid_solver_compare`, viewer command
`pixi run py-demos -- --scene rigid_solver_compare --width 960 --height 540`,
and the paired `pixi run py-demo-capture -- --scene rigid_solver_compare`
capture command. The generated `review_index.html` contained `open live`,
`capture evidence`, the live viewer command, and the capture command.
`pixi run lint` passed.

Current stack Replay timeline validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_stack_stability_keeps_ipc_stack_ordered python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata -q
pixi run py-demo-capture -- --scene rigid_stack_stability --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_stack_stability_timeline_1781271696
jq -r '.scene, .capture.converted_frames, .visual_evidence.screenshot.docked_workspace, .visual_evidence.screenshot.unique_rgb_count, .scene_metrics.event_count, .scene_metrics.latest.metrics.row, .scene_metrics.latest.metrics.top_x_divergence, .scene_metrics.latest.metrics.history.max_top_x_divergence, .scene_metrics.latest.metrics.history.ipc_min_clearance' /tmp/dart_capture_stack_stability_timeline_1781271696/manifest.json
```

The focused pytest reported `2 passed`. The real docked capture completed with
exit code 0 and wrote a nonblank 960x540 screenshot with docked UI detected,
23 PNG frames, and 24 scene-metric events. The capture manifest reported row
`rigid_stack_stability`, current and historical maximum top-x divergence about
`0.00405`, and IPC minimum clearance `0.0`.

Current contact-manipulation Replay timeline validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_contact_manipulation_pushes_target_toward_goal python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata -q
pixi run py-demo-capture -- --scene rigid_contact_manipulation --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_contact_manipulation_timeline_1781272246
jq -r '.scene, .capture.converted_frames, .visual_evidence.screenshot.docked_workspace, .visual_evidence.screenshot.unique_rgb_count, .scene_metrics.event_count, .scene_metrics.latest.metrics.row, .scene_metrics.latest.metrics.travel_divergence, .scene_metrics.latest.metrics.history.max_travel_divergence, .scene_metrics.latest.metrics.history.ipc_max_target_travel, .scene_metrics.latest.metrics.history.sequential_impulse_min_gap' /tmp/dart_capture_contact_manipulation_timeline_1781272246/manifest.json
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
```

The focused Replay/contact pytest reported `2 passed`. The real docked capture
completed with exit code 0 and wrote a nonblank 960x540 screenshot with docked
UI detected, 71 PNG frames, and 72 scene-metric events. The manifest reported
row `rigid_contact_manipulation`, current and historical maximum travel
divergence about `0.0217`, IPC target travel about `0.2243` m, and
sequential-impulse minimum gap about `-0.0017` m. The
sidecar/README/capture-command drift guard reported `4 passed`.

Current kinematic-driver Replay timeline validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_kinematic_driver_carries_box_with_ipc python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata -q
pixi run py-demo-capture -- --scene rigid_kinematic_driver --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_kinematic_driver_timeline_1781273002
jq -r '.scene, .capture.converted_frames, .visual_evidence.screenshot.docked_workspace, .visual_evidence.screenshot.unique_rgb_count, .scene_metrics.event_count, .scene_metrics.latest.metrics.row, .scene_metrics.latest.metrics.ipc_grip_box_travel, .scene_metrics.latest.metrics.history.ipc_grip_max_box_travel, .scene_metrics.latest.metrics.history.ipc_slip_max_slip, .scene_metrics.latest.metrics.history.si_caveat_max_abs_driver_travel' /tmp/dart_capture_kinematic_driver_timeline_1781273002/manifest.json
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run lint
git diff --check
```

The focused Replay/kinematic pytest reported `2 passed`. The real docked
capture completed with exit code 0 and wrote a nonblank 960x540 screenshot with
docked UI detected, 71 PNG frames, and 72 scene-metric events. The manifest
reported row `rigid_kinematic_driver`, current and historical IPC grip box
travel about `0.0896` m, IPC slip maximum about `0.1008` m, and
sequential-impulse caveat driver travel `0.0`. The sidecar/README/capture
command drift guard reported `4 passed`. `pixi run lint` passed and
`git diff --check` was clean.

Current normal-push Replay timeline validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_kinematic_normal_push_exposes_normal_pusher_caveat python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata -q
pixi run py-demo-capture -- --scene rigid_kinematic_normal_push --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_kinematic_normal_push_timeline_1781273413
jq -r '.scene, .capture.converted_frames, .visual_evidence.screenshot.docked_workspace, .visual_evidence.screenshot.unique_rgb_count, .scene_metrics.event_count, .scene_metrics.latest.metrics.row, .scene_metrics.latest.metrics.lanes.ipc_normal.status, .scene_metrics.latest.metrics.lanes.ipc_normal.max_depth, .scene_metrics.latest.metrics.lanes.ipc_normal.target_travel, .scene_metrics.latest.metrics.lanes.si_caveat.status, .scene_metrics.latest.metrics.lanes.si_caveat.target_travel, .scene_metrics.latest.metrics.lanes.si_caveat.analytic_gap' /tmp/dart_capture_kinematic_normal_push_timeline_1781273413/manifest.json
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
```

The focused Replay/normal-push pytest reported `2 passed`. The real docked
capture completed with exit code 0 and wrote a nonblank 960x540 screenshot with
docked UI detected, 71 PNG frames, and 72 scene-metric events. The manifest
reported row `rigid_kinematic_normal_push`, IPC normal status
`ipc penetration caveat`, IPC normal max depth about `0.125` m, near-zero IPC
target travel, SI status `pushed`, SI target travel about `0.123` m, and SI
analytic gap about `-0.00055` m. The sidecar/README/capture-command drift
guard reported `4 passed`.

Current fixed-joint Replay timeline validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_fixed_joint_verifier_restores_captured_transform python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata -q
pixi run py-demo-capture -- --scene rigid_fixed_joint --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_fixed_joint_timeline_1781274052
jq -r '.scene, .capture.converted_frames, .visual_evidence.screenshot.docked_workspace, .visual_evidence.screenshot.unique_rgb_count, .scene_metrics.event_count, .scene_metrics.latest.metrics.row, .scene_metrics.latest.metrics.metrics.translation_error, .scene_metrics.latest.metrics.metrics.orientation_error, .scene_metrics.latest.metrics.metrics.payload_speed, .scene_metrics.latest.metrics.history.max_translation_error, .scene_metrics.latest.metrics.history.max_orientation_error' /tmp/dart_capture_fixed_joint_timeline_1781274052/manifest.json
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
```

The focused Replay/fixed-joint pytest reported `2 passed`. The real docked
capture completed with exit code 0 and wrote a nonblank 960x540 screenshot with
docked UI detected, 23 PNG frames, and 24 scene-metric events. The manifest
reported row `rigid_fixed_joint`, final translation error about `2.17e-10` m,
final orientation error `0.0` rad, final payload speed `0.0` m/s, historical
maximum translation error about `1.09e-8` m, and historical maximum orientation
error about `2.67e-7` rad. The sidecar/README/capture-command drift guard
reported `4 passed`. `pixi run lint` passed and `git diff --check` was clean.

Current joint-breakage Replay timeline validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_joint_breakage_marks_and_resets_breakage python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata -q
pixi run py-demo-capture -- --scene rigid_joint_breakage --frames 48 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_joint_breakage_timeline_1781274802
jq -r '.scene, .capture.converted_frames, .visual_evidence.screenshot.docked_workspace, .visual_evidence.screenshot.unique_rgb_count, .scene_metrics.event_count, .scene_metrics.latest.metrics.row, .scene_metrics.latest.metrics.metrics.status, .scene_metrics.latest.metrics.metrics.broken, .scene_metrics.latest.metrics.metrics.payload_release_distance, .scene_metrics.latest.metrics.metrics.payload_speed, .scene_metrics.latest.metrics.history.saw_broken, .scene_metrics.latest.metrics.history.max_payload_release_distance' /tmp/dart_capture_joint_breakage_timeline_1781274802/manifest.json
```

The focused Replay/joint-breakage pytest reported `2 passed`. The real docked
capture completed with exit code 0 and wrote a nonblank 960x540 screenshot with
docked UI detected, 47 PNG frames, and 48 scene-metric events. The manifest
reported row `rigid_joint_breakage`, status `broken`, broken flag `1.0`, final
payload release distance about `0.413` m, final payload speed about `1.332`
m/s, `saw_broken` `1.0`, and historical maximum payload release distance about
`0.590` m. The sidecar/README/capture-command drift guard reported `4 passed`.
`pixi run lint` passed and `git diff --check` was clean.

Current distance-spring Replay timeline validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_distance_spring_reduces_stretch_and_spins_offset_anchor python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata -q
pixi run py-demo-capture -- --scene rigid_distance_spring --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_distance_spring_timeline_1781275675
jq -r '.scene, .capture.converted_frames, .visual_evidence.screenshot.docked_workspace, .visual_evidence.screenshot.unique_rgb_count, .scene_metrics.event_count, .scene_metrics.latest.metrics.row, .scene_metrics.latest.metrics.lanes.free.stretch, .scene_metrics.latest.metrics.lanes.soft.stretch, .scene_metrics.latest.metrics.lanes.stiff.stretch, .scene_metrics.latest.metrics.lanes.offset.stretch, .scene_metrics.latest.metrics.lanes.offset.angular_speed, .scene_metrics.latest.metrics.history.max_sprung_abs_stretch, .scene_metrics.latest.metrics.history.max_offset_angular_speed' /tmp/dart_capture_distance_spring_timeline_1781275675/manifest.json
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run lint
git diff --check
```

The focused Replay/distance-spring pytest reported `2 passed`. The real docked
capture completed with exit code 0 and wrote a nonblank 960x540 screenshot with
docked UI detected, 71 PNG frames, and 72 scene-metric events. The manifest
reported row `rigid_distance_spring`, final unsprung stretch `0.330` m,
soft/stiff stretch about `-0.113` m and `-0.125` m, offset stretch about
`-0.001` m, offset angular speed about `1.956` rad/s, historical maximum spring
stretch about `0.346` m, and historical maximum offset angular speed about
`21.04` rad/s. The sidecar/README/capture-command drift guard reported
`4 passed`. `pixi run lint` passed and `git diff --check` was clean.

Current limited-joints Replay timeline validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_one_dof_joint_verifier_preserves_locked_directions python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata -q
pixi run py-demo-capture -- --scene rigid_limited_joints --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_limited_joints_timeline_1781276042
jq -r '.scene, .capture.converted_frames, .visual_evidence.screenshot.docked_workspace, .visual_evidence.screenshot.unique_rgb_count, .scene_metrics.event_count, .scene_metrics.latest.metrics.row, .scene_metrics.latest.metrics.metrics.hinge_radius_error, .scene_metrics.latest.metrics.metrics.hinge_z_error, .scene_metrics.latest.metrics.metrics.hinge_yaw, .scene_metrics.latest.metrics.metrics.slider_orthogonal_error, .scene_metrics.latest.metrics.metrics.slider_axis_travel, .scene_metrics.latest.metrics.history.max_hinge_radius_error, .scene_metrics.latest.metrics.history.max_hinge_z_error, .scene_metrics.latest.metrics.history.max_slider_orthogonal_error, .scene_metrics.latest.metrics.history.max_abs_hinge_yaw, .scene_metrics.latest.metrics.history.max_slider_axis_travel' /tmp/dart_capture_limited_joints_timeline_1781276042/manifest.json
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run lint
git diff --check
```

The focused Replay/limited-joints pytest reported `2 passed`. The real docked
capture completed with exit code 0 and wrote a nonblank 960x540 screenshot with
docked UI detected, 23 PNG frames, and 24 scene-metric events. The manifest
reported row `rigid_limited_joints`, hinge radius error `0.0`, hinge z error
`0.0`, hinge yaw about `0.168` rad, slider orthogonal error `0.0`, slider axis
travel about `0.604` m, zero historical locked-axis error, historical maximum
hinge yaw about `0.168` rad, and historical maximum slider axis travel about
`0.604` m. The sidecar/README/capture-command drift guard reported `4 passed`.
`pixi run lint` passed and `git diff --check` was clean.

Current motor-limits Replay timeline validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_joint_motor_limits_clamp_commands_and_effort python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run py-demo-capture -- --scene rigid_joint_motor_limits --frames 96 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_joint_motor_limits_timeline_1781276821
jq -r '.scene, .capture.requested_frames, .capture.converted_frames, .show_ui, .visual_evidence.screenshot.docked_workspace, .visual_evidence.screenshot.unique_rgb_count, .scene_metrics.event_count, .scene_metrics.latest.metrics.row, .scene_metrics.latest.metrics.motor_speed, .scene_metrics.latest.metrics.motor_expected_speed, .scene_metrics.latest.metrics.motor_speed_error, .scene_metrics.latest.metrics.position_limit_angle, .scene_metrics.latest.metrics.position_limit_upper, .scene_metrics.latest.metrics.position_limit_error, .scene_metrics.latest.metrics.force_position_gap, .scene_metrics.latest.metrics.force_acceleration_gap, .scene_metrics.latest.metrics.history.max_force_position_gap, .scene_metrics.latest.metrics.history.max_force_acceleration_gap' /tmp/dart_capture_joint_motor_limits_timeline_1781276821/manifest.json
pixi run lint
git diff --check
```

The focused Replay/motor-limits pytest and drift guards reported `6 passed`.
The real docked capture completed with exit code 0 and wrote a nonblank
960x540 screenshot with docked UI detected, 95 PNG frames, and
96 scene-metric events. The maintained workflow capture budget was raised from
72 to 96 frames because the 72-frame packet showed velocity clamp and effort
gap, but did not reach the position stop. The 96-frame packet reached the stop
with `position_limit_angle` equal to `position_limit_upper` (`0.35` rad), motor
speed and expected speed `0.3` m/s, motor speed error `0.0`, position-limit
error `0.0`, force travel gap about `0.6984` m, and force acceleration gap
about `6.0` m/s^2. `pixi run lint` passed and `git diff --check` was clean.
