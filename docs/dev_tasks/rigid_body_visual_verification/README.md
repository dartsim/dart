# Rigid Body Visual Verification - Dev Task

## Current Handoff (2026-06-12)

Stop/push handoff: the current session was redirected to stop code changes and
stop further verification, update only the dev-task handoff docs, merge latest
`origin/main`, and push to `origin`. The running broad validation command
`timeout 7200s pixi run test-all --skip-lint --skip-build` was terminated on
request while inside the simulation-labeled ctest stage. Its visible progress
had already passed lint, Release configure/build, the direct re-run of
`pixi run build-tests ON Release`, and the 219/219 C++ unit tests; visible
simulation progress was 52/65 when output last streamed. Treat this as a
stopped verification attempt, not a completed `test-all` pass. CUDA validation
was not started. The docs-only handoff refresh intentionally does not run
additional verification because the user explicitly requested no further
verification in this session.

Latest merge state: `git fetch origin main && git merge --no-edit origin/main`
reported `Already up to date`, so the branch remains aligned with the latest
`origin/main` and the PR #2986 DART 7 architecture/work-packet harness before
the handoff docs push. The user explicitly requested `push to origin` for this
handoff; no PR creation, milestone mutation, CI rerun, or review-thread action
was requested or approved by that push instruction.

Latest local follow-up: fetched `origin/main` and merged it again; Git reported
`Already up to date`, so this branch remains aligned with the PR #2986 DART 7
architecture/work-packet harness. The native Filament `SIGBUS` blocker was
traced to Linux headless engine creation using Filament's `/tmp`-backed
ashmem-style command-buffer mapping under tmpfs user-quota pressure. During
the failing repros, `quota -s` reported `tmpfs 25573M/25573M`; the current
occupancy can drift, so recheck it before drawing host-level conclusions.
`dart::gui` now forces Filament's anonymous "soft" CircularBuffer fallback
only during Linux headless `Engine::Builder().build()`, then restores the
original file-size limit before screenshots and frames are written. The failed
`build/captures/rigid_workflow_rows_01_36_1781311276` packet and earlier
`/tmp` reproducers remain failure evidence only.

Latest local follow-up: after the headless engine-creation fix, the DART 7
harness identity-complete packets now regenerate successfully. The full
numbered packet
`build/captures/rigid_workflow_rows_01_36_1781312968` mirrors the original
`/dev/shm/dart_rigid_workflow_rows_01_36_1781312968` run and completed with
`status=complete`, `capture_count=36`, `completed_count=36`,
`failed_count=0`, `workflow_total_count=36`, `workflow_row_start=1`,
`workflow_row_end=36`, `guidance_complete=true`,
`guidance_missing_count=0`, `resolved_solver_identity_complete=true`,
`resolved_solver_identity_count=36`, `resolved_solver_identity_missing_count=0`,
`failed_rows=[]`, and 2388 frame PNGs. The optional rows 37-52 packet
`build/captures/rigid_workflow_optional_rows_37_52_1781313357` mirrors the
original `/dev/shm/dart_rigid_workflow_optional_rows_37_52_1781313357` run and
completed with `status=complete`, `capture_count=16`, `completed_count=16`,
`failed_count=0`, `workflow_total_count=52`, `workflow_row_start=37`,
`workflow_row_end=52`, `include_related=true`, `include_ipc_shelf=true`,
`include_packets=true`, `selected_include_related=true`,
`selected_include_ipc_shelf=true`, `selected_include_packets=true`,
`guidance_complete=true`, `guidance_missing_count=0`,
`resolved_solver_identity_complete=true`, `resolved_solver_identity_count=16`,
`resolved_solver_identity_missing_count=0`, `failed_rows=[]`, and 1004 frame
PNGs. A read-only HTML asset audit found 0 missing local assets in both copied
review indexes: 181/181 links for rows 01-36 and 81/81 links for rows 37-52.
The copied `review_index.html` files use relative asset links; the JSON
manifests preserve the original `/dev/shm` command/output paths as capture
provenance.

Latest local follow-up: regenerated the current-HEAD review packets after the
workflow-command provenance and review-index link-normalization fix. The full
numbered packet
`build/captures/rigid_workflow_rows_01_36_1781309127` completed with
`status=complete`, `capture_count=36`, `completed_count=36`,
`failed_count=0`, `workflow_total_count=36`, `guidance_complete=true`,
`guidance_missing_count=0`, and `failed_rows=[]`. It wrote
`manifest.json`, `review_index.html`, 36 docked row screenshots, and 2388
frame PNGs; the first row was `rigid_body` and the last was
`rigid_loop_closure`. The optional extended packet
`build/captures/rigid_workflow_optional_rows_37_52_1781309448` completed with
`status=complete`, `capture_count=16`, `completed_count=16`,
`failed_count=0`, `workflow_total_count=52`, `workflow_row_start=37`,
`workflow_row_end=52`, `include_related=true`, `include_ipc_shelf=true`,
`include_packets=true`, `continue_on_failure=true`,
`guidance_complete=true`, `guidance_missing_count=0`, and `failed_rows=[]`.
It wrote `manifest.json`, `review_index.html`, 16 docked row screenshots, and
1004 frame PNGs; the first selected scene was `floating_base` and the last was
`rigid_ipc_heavy_stack_packet`. A read-only HTML asset audit found 0 missing
local assets in both review indexes: 181/181 links for rows 01-36 and 81/81
links for rows 37-52. These packets supersede the older
`1781305407`/`1781305860` packet paths as review-index scan artifacts; keep the
older packet paths only as historical completion evidence.

Latest local follow-up: the workflow packet manifest and review index now
record the exact top-level `pixi run py-demo-capture -- --rigid-workflow ...`
command that produced the packet, and review-index screenshot/frame/video links
now normalize workspace-relative artifact paths from per-scene manifests so
they resolve from `review_index.html`. Focused guard:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_request_video_commands python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_review_links_resolve_workspace_relative_artifacts python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_links_scene_videos -q`
reported `4 passed`. The regenerated packets above exercise this fix from the
current HEAD.

Latest local follow-up: fetched `origin/main` and merged it into
`feature/rigid-body-gui-visual-verification`; Git reported the branch was
already up to date with the PR #2986 DART 7 architecture/work-packet harness.
The workflow review-index path now has a focused unit guard that each captured
row card renders exactly one screenshot thumbnail instead of duplicating the
same image in the static contact sheet. Focused guard:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests -q`
reported `1 passed`.

Latest local follow-up: the API-deferred gap audit still finds no public
direct `RigidBody` impulse surface, no public sleep/wake or island activation
surface, and no public loop-closure compliance/stiffness/damping surface. The
in-viewer `Rigid Workflow` search now routes deferred-user terms such as
`direct rigid body impulse`, `sleep wake`, `island activation`, and
`loop closure compliance` to the closest current rows while preserving explicit
non-claim caveats instead of adding speculative numbered rows. Focused guard:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_routes_deferred_api_terms python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_deferred_api_gaps_are_documented python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
reported `6 passed`.

Latest local state: after the row-36 loop-closure implementation commit, a
fresh full numbered workflow packet completed all 36 World Rigid Body rows in
one `py-demo-capture -- --rigid-workflow` run. That packet exercises the same
docked Filament/ImGui capture path, per-row manifests, screenshots, frame
directories, workflow guidance, and `review_index.html` contact sheet that a
maintainer would use to review the row-15-through-row-36 DART 7 harness pass.
The current optional extended packet has also been rechecked for rows 37-52,
covering related evidence, direct Rigid IPC shelf routes, and capture-first IPC
stress packets.

Latest implementation slice: row 36 gives the variational rigid multibody
loop-closure family row the same reviewable comparison shape as the surrounding
rigid workflow rows while continuing to follow the DART 7
architecture/work-packet harness from PR #2986. `rigid_loop_closure` names
`loop_closure_family_policy_selection` as its comparison axis in the panel and
capture metrics, records held-fixed contact-free variational rigid
multibody/four-link-chain/gravity/time-step context, exports top-level
POINT/DISTANCE/RIGID residual ratios, distance-family distance/tip error,
RIGID orientation error, and maximum step-time metrics, and feeds decisive
latest signals into the workflow review index.

Evidence for this slice:

- Fresh full numbered workflow packet:
  `build/captures/rigid_workflow_rows_01_36_1781309127` completed with
  `status=complete`, `capture_count=36`, `completed_count=36`,
  `failed_count=0`, `workflow_total_count=36`, `guidance_complete=true`,
  `guidance_missing_count=0`, and `failed_rows=[]`. It wrote
  `manifest.json`, `review_index.html`, 36 docked row screenshots, and 2388
  frame PNGs across rows 01-36; the first row was `rigid_body` and the last row
  was `rigid_loop_closure`.
- Fresh optional extended workflow packet:
  `build/captures/rigid_workflow_optional_rows_37_52_1781309448` completed with
  `status=complete`, `capture_count=16`, `completed_count=16`,
  `failed_count=0`, `workflow_total_count=52`, `workflow_row_start=37`,
  `workflow_row_end=52`, `include_related=true`, `include_ipc_shelf=true`,
  `include_packets=true`, `continue_on_failure=true`,
  `guidance_complete=true`, `guidance_missing_count=0`, and
  `failed_rows=[]`. It wrote `manifest.json`, `review_index.html`, 16 docked
  row screenshots, and 1004 frame PNGs; the first selected scene was
  `floating_base` and the last was `rigid_ipc_heavy_stack_packet`.
- Focused row/panel/docs-order/review-index pytest subset reported `6 passed`.
  It included
  `python/tests/integration/test_demos_cycle.py::test_rigid_loop_closure_compares_closure_families`,
  the comparison-axis panel coverage, docs/sidecar order checks, capture-command
  sync check, and the unit guard that row-36 latest signals prioritize closure
  family ratios, distance-family distance/tip error, RIGID orientation error,
  and solver.
- Real row-36 workflow capture completed under
  `build/captures/rigid_loop_closure_row_36_1781304923` with
  `status=complete`, `capture_count=1`, `completed_count=1`,
  `failed_count=0`, `workflow_total_count=36`, and
  `guidance_complete=true`.
- Row 36 reported `comparison_axis=loop_closure_family_policy_selection`,
  held-fixed `contacts=off`, `integration_family=variational integrator`,
  `joint_family=four_revolute_links`, `chain_links=4`, `link_length=0.56`,
  `link_mass=0.55`, `initial_bend=0.18`, `gravity_scale=1.0`,
  `solver=variational_rigid_multibody_loop_closure`, `time_step_ms=5.0`,
  controls `executor_index=0.0`, `gravity_scale=1.0`,
  `closure_family_lanes=[POINT, DISTANCE, RIGID]`,
  `closure_policy_lanes=[residual, solved]`,
  `loop_closure_point_residual_ratio=759510268605.6781`,
  `loop_closure_distance_residual_ratio=745798996483.9451`,
  `loop_closure_rigid_residual_ratio=773988932063.1805`,
  `loop_closure_distance_solved_distance_error=9.43689570931383e-15`,
  `loop_closure_distance_solved_tip_error=0.43866082264064005`,
  `loop_closure_rigid_residual_orientation_error=0.14900677447295246`,
  `loop_closure_rigid_solved_orientation_error=2.7755575615628914e-17`, and
  `loop_closure_max_step_ms=0.483382`.
- `review_index.html` showed the row-36 card with `axis`, `held fixed`,
  `controls`, Replay signal/markers, metric-key summary, and latest-signal
  ordering for POINT/DISTANCE/RIGID residual ratios, distance-family
  distance/tip error, RIGID orientation error, and solver.
- The per-scene capture wrote a nonblank docked screenshot with 2446 unique
  colors and 71 PNG frames for `rigid_loop_closure` from the 72-frame workflow
  row capture under `build/captures/`.

Previous completed and verified slice: row 35 gives the World multibody
solver-family routing row the same reviewable comparison shape as the
surrounding rigid workflow rows. `rigid_multibody_solver_family` names
`multibody_integration_solve_policy_family` as its comparison axis in the panel
and capture metrics, records held-fixed contact-free World
point-closure/three-link-chain/gravity/time-step context, exports top-level
residual-only residual, solved residual, residual solve ratio, lane residuals,
solved tip error, and maximum step-time metrics, and feeds decisive latest
signals into the workflow review index. Evidence: focused
row/panel/docs-order/review-index pytest subset reported `6 passed`; the real
row-35 workflow capture completed under
`build/captures/rigid_multibody_solver_family_row_35_1781304535` with
`status=complete`, `capture_count=1`, `completed_count=1`, `failed_count=0`,
`workflow_total_count=36`, `guidance_complete=true`, a nonblank docked
screenshot, and 71 PNG frames.

Previous completed and verified slice: row 34 gives the World multibody link
Jacobian row the same reviewable comparison shape as the surrounding rigid
workflow rows. `rigid_link_jacobian` names
`link_origin_jacobian_mapping_family` as its comparison axis in the panel and
capture metrics, records held-fixed contact-free World
two-revolute-link/time-step/finite-difference context, exports top-level link
linear/angular speed, world/body Jacobian gap, finite-difference residual,
transpose-mapped torque, and power-residual metrics, and feeds decisive latest
signals into the workflow review index. Evidence: focused
row/panel/docs-order/review-index pytest subset reported `6 passed`; the real
row-34 workflow capture completed under
`build/captures/rigid_link_jacobian_row_34_1781304169` with
`status=complete`, `capture_count=1`, `completed_count=1`, `failed_count=0`,
`workflow_total_count=36`, `guidance_complete=true`, a nonblank docked
screenshot, and 95 PNG frames.

Previous completed and verified slice: row 33 gives the World multibody link
center-of-mass offset row the same reviewable comparison shape as the
surrounding rigid workflow rows. `rigid_link_center_of_mass` names
`link_center_of_mass_offset_family` as its comparison axis in the panel and
capture metrics, records held-fixed contact-free World
revolute-link/fixed-visual-geometry/mass/gravity/time-step context, exports
top-level centered/positive/negative gravity torque, mirrored-angle,
high-inertia ratio, and acceleration-residual metrics, and feeds decisive
latest signals into the workflow review index. Evidence: focused
row/panel/docs-order/review-index pytest subset reported `6 passed`; the real
row-33 workflow capture completed under
`build/captures/rigid_link_center_of_mass_row_33_1781303757` with
`status=complete`, `capture_count=1`, `completed_count=1`, `failed_count=0`,
`workflow_total_count=36`, `guidance_complete=true`, a nonblank docked
screenshot, and 71 PNG frames.

Previous completed and verified slice: row 32 gives the World multibody
generalized dynamics terms row the same reviewable comparison shape as the
surrounding rigid workflow rows. `rigid_multibody_dynamics_terms` names
`joint_space_dynamics_term_family` as its comparison axis in the panel and
capture metrics, records held-fixed contact-free World dynamics/fixed-base/
revolute-link/target-acceleration/impulse/gravity/time-step context, exports
top-level scalar mass, coupling, torque-gap, response-gap, response-ratio, and
residual metrics, and feeds decisive latest signals into the workflow review
index. Evidence: focused row/panel/docs-order/review-index pytest subset
reported `6 passed`; the real row-32 workflow capture completed under
`build/captures/rigid_multibody_dynamics_terms_row_32_1781303198` with
`status=complete`, `capture_count=1`, `completed_count=1`, `failed_count=0`,
`workflow_total_count=36`, `guidance_complete=true`, a nonblank docked
screenshot, and 95 PNG frames.

Previous completed and verified slice: row 31 gives the World multibody
screw-joint pitch row the same reviewable comparison shape as the surrounding
rigid workflow rows. `rigid_screw_joint_pitch` names
`screw_pitch_coupling_family` as its comparison axis in the panel and capture
metrics, records held-fixed contact-free World screw-joint/z-axis/mass/inertia/
time-step context, exports top-level pitch, travel-gap, reverse-sign, and
acceleration-residual metrics, and feeds decisive latest signals into the
workflow review index. Evidence: focused row/panel/docs-order/review-index
pytest subset reported `6 passed`; the real row-31 workflow capture completed
under `build/captures/rigid_screw_joint_pitch_row_31_1781302747` with
`status=complete`, `capture_count=1`, `completed_count=1`, `failed_count=0`,
`workflow_total_count=36`, `guidance_complete=true`, a nonblank docked
screenshot, and 95 PNG frames.

Previous completed and verified slice: row 30 gives the World multibody
passive joint parameter row the same reviewable comparison shape as the
surrounding rigid workflow rows. `rigid_joint_passive_parameters` names
`passive_joint_parameter_family` as its comparison axis in the panel and
capture metrics, records held-fixed contact-free World-prismatic/mass/time-step
context, exports top-level energy, slip, and armature-gap metrics, and feeds
decisive latest signals into the workflow review index. Evidence: focused
row/panel/docs-order/review-index pytest subset reported `6 passed`; the real
row-30 workflow capture completed under
`build/captures/rigid_joint_passive_parameters_row_30_1781302360` with
`status=complete`, `capture_count=1`, `completed_count=1`, `failed_count=0`,
`workflow_total_count=36`, `guidance_complete=true`, a nonblank docked
screenshot, and 119 PNG frames.

Previous completed and verified slice: row 29 gives the World multibody joint
motor/limit row the same reviewable comparison shape as the surrounding rigid
workflow rows. `rigid_joint_motor_limits` names
`world_multibody_actuator_limit_family` as its comparison axis in the panel and
capture metrics, records held-fixed World-joint/axis/mass/time-step context,
exports top-level motor clamp, position-limit, and force-gap metrics, and feeds
decisive latest signals into the workflow review index. Evidence: focused
row/panel/docs-order/review-index pytest subset reported `6 passed`; the real
row-29 workflow capture completed under
`build/captures/rigid_joint_motor_limits_row_29_1781301981` with
`status=complete`, `capture_count=1`, `completed_count=1`, `failed_count=0`,
`workflow_total_count=36`, `guidance_complete=true`, a nonblank docked
screenshot, and 95 PNG frames.

Previous completed and verified slice: row 28 gives the rigid-body one-DOF
joint row the same reviewable comparison shape as the surrounding rigid
workflow rows. `rigid_limited_joints` names
`one_dof_joint_constraint_family` as its comparison axis in the panel and
capture metrics, records held-fixed sequential-joint/static-base/z-axis
context, exports top-level locked-axis and free-axis motion metrics, and feeds
decisive latest signals into the workflow review index. Evidence: focused
row/panel/docs-order/review-index pytest subset reported `6 passed`; the real
row-28 workflow capture completed under
`/tmp/dart_capture_rigid_limited_joints_row_28_1781301510` with
`status=complete`, `capture_count=1`, `completed_count=1`, `failed_count=0`,
`workflow_total_count=36`, `guidance_complete=true`, a nonblank docked
screenshot, and 23 PNG frames.

Previous completed and verified slice: row 27 gives the rigid-body
distance-spring row the same reviewable comparison shape as the surrounding
rigid workflow rows. `rigid_distance_spring` names
`distance_spring_response_family` as its comparison axis in the panel and
capture metrics, records held-fixed executor/rest-length/payload/time-step
context, exports top-level distance-spring stretch and offset-spin metrics, and
feeds decisive latest signals into the workflow review index. Evidence:
focused row/panel/docs-order/review-index pytest subset reported `6 passed`;
the real row-27 workflow capture completed under
`/tmp/dart_capture_rigid_distance_spring_row_27_1781300614` with
`status=complete`, `capture_count=1`, `completed_count=1`, `failed_count=0`,
`workflow_total_count=36`, `guidance_complete=true`, a nonblank docked
screenshot, and 71 PNG frames.

Previous completed and verified slice: row 26 gives the AVBD-pinned fixed-joint
breakage row the same reviewable comparison shape as the surrounding rigid
workflow rows.
`rigid_joint_breakage` names `fixed_break_force_lifecycle` as its comparison
axis in the panel and capture metrics, records the held-fixed AVBD/static-base
context, exports top-level breakage metrics, and feeds decisive latest signals
into the workflow review index.

Evidence for this slice:

- Focused row/panel/docs-order/review-index pytest subset reported `7 passed`.
  It included
  `python/tests/integration/test_demos_cycle.py::test_rigid_joint_breakage_marks_and_resets_breakage`
  and the unit guard that row-26 latest signals prioritize payload release
  distance, broken state, captured-offset error, payload speed, status, and
  solver. The existing reset-lifecycle panel test was also included in the
  focused run.
- Real row-26 workflow capture completed under
  `/tmp/dart_capture_rigid_joint_breakage_row_26_1781299982` with
  `status=complete`, `capture_count=1`, `completed_count=1`,
  `failed_count=0`, `workflow_total_count=36`, and
  `guidance_complete=true`.
- Row 26 reported `comparison_axis=fixed_break_force_lifecycle`, held-fixed
  `base=static`, `captured_offset_m=0.62`, `ground_friction=0.6`,
  `payload_mass=1.0`, `solver=AVBD rigid joints`, `time_step_ms=4.0`,
  controls `break_force=1e-12`, `reset_break_force=1e12`,
  `breakage_payload_release_distance=0.41319290960568955`,
  `breakage_broken=1.0`,
  `breakage_captured_offset_error=0.27164249515625993`,
  `breakage_payload_speed=1.331957542419467`,
  `breakage_status=broken`, and history maxima
  `max_payload_release_distance=0.59045308585389`,
  `max_payload_speed=49.220347596711996`.
- `review_index.html` showed the row-26 card with `axis`, `held fixed`,
  `controls`, Replay signal/markers, metric-key summary, and visible latest
  signals for payload release distance, broken state, captured-offset error,
  payload speed, status, and solver.
- The per-scene capture wrote a nonblank docked screenshot and 47 PNG frames
  for `rigid_joint_breakage` from the 48-frame workflow row capture.

Repository state notes:

- Branch: `feature/rigid-body-gui-visual-verification`; no PR is associated
  with this branch.
- Latest completed implementation commit for the row-15-through-row-36 pass:
  `608a8792afa Surface loop closure workflow signals`.
- Do not push without explicit approval in the session that performs the push.
- Resume check: inspect `git status -sb` and `git log -8 --oneline`. Expect the
  latest implementation commit to describe the loop-closure workflow signals;
  if a docs-only evidence commit exists after it, inspect that diff first.

Recommended next action: use the full row-01-through-row-36 packet plus the
optional rows 37-52 packet as maintainer-review evidence for the current rigid
workflow. If more local progress is requested before pushing/review, audit the
API-deferred gaps rather than adding speculative numbered rows.

Previous checkpoint: row 25 gives the fixed-joint verifier the same reviewable
comparison shape as the surrounding rigid workflow rows. `rigid_fixed_joint`
names `fixed_relative_transform_recovery` as its comparison axis in the panel
and capture metrics, records the held-fixed sequential-joint context, exports
top-level pose-recovery metrics, and feeds fixed-joint translation error,
orientation error, payload speed, angular speed, and solver into the workflow
review index.

Previous checkpoint: row 23 gives the tangential kinematic-driver row the same
reviewable comparison shape as the surrounding rigid contact rows.
`rigid_kinematic_driver` names `prescribed_tangential_contact_response` as its
comparison axis in the panel and capture metrics, records the held-fixed
tangential-support context, exports solver/case/lane metadata, and feeds IPC
grip travel, grip speed ratio, low-friction slip, sequential-impulse caveat
driver travel, IPC proximity gap, and solver pair into the workflow review
index.

Previous checkpoint: row 22 gives the task-like rigid pusher row the same
reviewable comparison shape as the surrounding rigid contact rows.
`rigid_contact_manipulation` names `rigid_pusher_contact_response` as its
comparison axis in the panel and capture metrics, records the held-fixed
matched table/goal context, exports solver/case/lane metadata, and feeds travel
divergence, both solver lanes' target travel, sequential-impulse contact
evidence, IPC proximity gap, and solver pair into the workflow review index.

Previous checkpoint: row 24 gives the normal kinematic-push caveat the same
reviewable shape as the surrounding rigid contact rows.
`rigid_kinematic_normal_push` names `prescribed_normal_contact_response` as its
comparison axis in the panel and capture metrics, records the held-fixed
normal-paddle context, exports solver/case/lane metadata, and feeds
target-travel divergence, SI target travel, IPC penetration depth, solver pair,
case pair, and solver label into the workflow review index.

Previous checkpoint: rows 19-21 form a contact-failure comparison mini-packet
with explicit user-facing comparison axes and held-fixed controls.
`rigid_friction_threshold`, `rigid_spin_roll_coupling`, and
`rigid_stack_stability` label their comparison axis in the panel and
scene-owned capture metrics, export held-fixed values, and feed the workflow
review index latest-signal summaries for threshold drift, contact slip/spin
change, solver pair, top-x divergence, and clearance.

Evidence for the rows 19-21 slice:

- Focused row/panel/review-index pytest subset reported `5 passed`.
- Real workflow capture completed under
  `/tmp/dart_capture_rigid_contact_failure_rows_19_21_1781296166` with
  `status=complete`, `capture_count=3`, `completed_count=3`, `failed_count=0`,
  and `guidance_complete=true`.
- `review_index.html` showed `axis`, `held fixed`, `controls`, and
  `latest signals` for rows 19-21. The latest row cards reported
  `friction_threshold_lane`, `spin_roll_initial_condition`, and
  `rigid_body_solver_family` as the three comparison axes.

Previous checkpoint: workflow `review_index.html` row cards now surface actual
decision values from scene-owned metrics, not only metric keys. Captured rows
show held-fixed values when present, control settings, and compact latest
signals such as solver pair, executor pair, contact-policy pair, divergence,
and step/contact counters.

Previous checkpoint: refreshed the core solver/contact-policy visual evidence
packet for rows 15-17. The real workflow capture completed
`rigid_solver_compare`, `rigid_executor_equivalence`, and
`rigid_contact_solver_compare` together, keeping solver-family,
executor-only, and contact-policy decisions adjacent in `review_index.html`.

Previous checkpoint: direct single-scene `py-demo-capture -- --scene ...`
manifests for maintained rigid workflow rows now include a `workflow_guidance`
block with row number, role, user question, try-first action, inspect signals,
healthy signal, and scope note. This keeps a one-row capture self-describing
when it is handed to a reviewer outside the full workflow packet.

Previous checkpoint: the rigid workflow `review_index.html` now shows each
selected row's packet-preserving `workflow_rerun_command` directly on the row
card, not only inside the Failed Rows summary. This lets reviewers rerun any
planned, captured, or failed row with the same workflow packet shape, absolute
row number, and row-specific rerun output directory.

Previous checkpoint: the C++ `dart-demos --list` path is now a non-graphics
catalog command. `runDemos()` prints the registered C++ demo catalog before GUI
backend initialization, `examples/demos/CMakeLists.txt` registers
`EXAMPLE_dart_demos_list` as a normal non-graphics example/catalog test, and
`examples/README.md` documents `pixi run demos -- --list` for listing scenes
without opening the GUI.

Previous checkpoint: commit
`41186522838 Retire C++ collision sandbox placeholder` brings the C++
companion back into sync with the completed py-demos collision-sandbox
retirement. The stale
`planned_collision_sandbox` row is removed from `examples/demos/registry.cpp`,
and legacy C++ runner requests for `collision_sandbox`, `point_cloud`, and
`polyhedron_visual` now route users to the concrete Python collision-debugging
GUI rows: `rigid_contact_inspector`, `rigid_collision_query_options`, and
`rigid_collision_casts`.

Previous checkpoint: commit `18c3750a7af Retire planned collision sandbox row`
retires the `planned_collision_sandbox` placeholder from the py-demos catalog
because the concrete collision-debugging GUI rows already cover that route.
PLAN-103 and the Python demo README name the replacement path.

Previous checkpoint: PLAN-103's remaining planned World-port GUI
rows after the rigid verifier reached maintainer-acceptance readiness. The
Planned World Ports placeholders now tell users which current py-demo route to
try first, which World/API or asset gap blocks the full port, and what
replacement condition will retire the placeholder; focused panel tests guard the
metadata and GUI text.

Previous checkpoint: failed rows from resilient workflow packets became
directly actionable. `py-demo-capture -- --rigid-workflow
--continue-on-failure` records `failed_rows` with workflow row-range rerun
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
- Prior stop-state instruction: the user explicitly requested no further
  implementation and no further verification; only hand-off docs were ensured
  in that turn. This was later superseded by a continuation request for the
  active goal.
- Current continuation started with the branch clean at
  `867c5f5d8a5 Add rigid workflow guidance to capture manifests`, eleven
  commits ahead of `origin/feature/rigid-body-gui-visual-verification`,
  committed `bedc418df0e Record solver contact workflow packet evidence`, then
  improved workflow review-index row cards to expose actual latest metric
  values for solver/contact decision review.
- At the stop-state hand-off, those docs updates were uncommitted local
  modifications and no push was performed. The resumed continuation verified
  this evidence-docs slice locally before choosing the next bounded
  GUI-verification improvement; push still requires explicit approval.
- Resume check: inspect `git status -sb` and `git log -8 --oneline` before any
  resumed work. Expect the latest completed implementation commit to be
  `Surface workflow review latest signals` if this slice has been committed;
  otherwise inspect the uncommitted diff for the latest-signals review-index
  follow-up.
- Do not push without explicit approval in the session that performs the push.
- Latest local slice updates `examples/demos/registry.cpp`,
  `scripts/run_cpp_example.py`, `python/tests/unit/test_run_cpp_example.py`,
  `examples/README.md`, PLAN-102 dashboard text, and this handoff so the C++
  companion no longer advertises the retired collision-sandbox placeholder.
- Verification for the latest C++ companion cleanup: focused
  `python/tests/unit/test_run_cpp_example.py` reported `66 passed` before and
  after `pixi run lint`; `pixi run ex collision_sandbox` exits nonzero with
  the expected `pixi run py-demos -- --scene rigid_contact_inspector`,
  `rigid_collision_query_options`, and `rigid_collision_casts` guidance and no
  `planned_collision_sandbox` text.
- Latest local follow-up makes `dart-demos --list` a non-graphics catalog path:
  `runDemos()` now prints the registered C++ demo catalog before GUI backend
  initialization, and CMake registers `EXAMPLE_dart_demos_list` as a normal
  non-graphics example/catalog test. This removes the previous Mesa/GLX host
  caveat for the list command while leaving launch/cycle behavior unchanged.
- Verification for the latest `dart-demos --list` follow-up: the
  `pixi run demos -- --list` command returned the catalog with `rigid_body` and
  `planned_mobile_manipulation` present and `planned_collision_sandbox` absent.
  The focused `EXAMPLE_dart_demos_list` CTest passed, the focused
  `python/tests/unit/test_run_cpp_example.py` suite reported `66 passed`, and
  `pixi run lint`, `pixi run check-lint-md`, `pixi run check-docs-policy`,
  `pixi run check-lint-spell`, and `git diff --check` passed.
- Latest local review-index follow-up updates `scripts/capture_py_demo.py`,
  `python/tests/unit/test_capture_py_demo.py`, `python/examples/demos/README.md`,
  PLAN-103's rigid sidecar, and this handoff so every selected workflow row card
  shows its packet-preserving rerun command.
- Verification for the latest review-index follow-up: the focused
  `python/tests/unit/test_capture_py_demo.py -k rigid_workflow` suite reported
  `25 passed, 14 deselected` with the built `dartpy` PYTHONPATH. The final
  lint, Markdown-format, docs-policy, spelling, and `git diff --check` gates
  passed.
- Latest local direct-manifest follow-up updates `scripts/capture_py_demo.py`,
  `python/tests/unit/test_capture_py_demo.py`, `python/examples/demos/README.md`,
  PLAN-103's rigid sidecar, and this handoff so direct captures carry rigid
  workflow guidance without requiring the full packet context.
- Verification for the latest direct-manifest follow-up: the focused
  `python/tests/unit/test_capture_py_demo.py -k 'visual_capture_manifest_records_image_evidence or rigid_workflow'`
  suite reported `26 passed, 13 deselected` with the built `dartpy` PYTHONPATH.
  The final lint, Markdown-format, docs-policy, spelling, and `git diff --check`
  gates passed.
- Latest local solver/contact-policy mini-packet evidence updates
  `python/examples/demos/README.md`, PLAN-103's rigid sidecar, and this handoff
  with the current row 15-17 capture command and artifact summary.
- Verification for the latest solver/contact-policy mini-packet evidence: the
  row-range workflow capture completed rows 15-17 under
  `/tmp/dart_capture_rigid_workflow_solver_contact_rows_15_17_1781294600` with
  `capture_count=3`, `completed_count=3`, `failed_count=0`,
  `guidance_complete=true`, docked nonblank screenshots, PNG frame sequences,
  review-index row cards, and scene metrics for solver-family, executor, and
  contact-policy comparison axes. Post-resume `pixi run lint`,
  `pixi run check-lint-md`, `pixi run check-docs-policy`,
  `pixi run check-lint-spell`, and `git diff --check` passed for this
  evidence-docs slice.
- Latest local review-index latest-signals follow-up updates
  `scripts/capture_py_demo.py`, `python/tests/unit/test_capture_py_demo.py`,
  `python/examples/demos/README.md`, PLAN-103's rigid sidecar, and this handoff
  so row cards render held-fixed/control values and latest comparison signals
  directly from `scene_metrics.latest.metrics`.
- Verification for the latest review-index latest-signals follow-up: the
  focused rigid-workflow aggregate/dry-run/video unit subset in
  `python/tests/unit/test_capture_py_demo.py` reported `3 passed, 36
deselected`. A real rows 15-17 workflow capture completed under
  `/tmp/dart_capture_rigid_workflow_latest_signals_rows_15_17_1781295362` with
  `capture_count=3`, `completed_count=3`, and `failed_count=0`; the review
  index showed control values plus latest signals for the solver pair, executor
  pair, contact-policy pair, and divergence summaries.
- Latest local slices update `python/examples/demos/registry.py`,
  `python/examples/demos/scenes/planned.py`,
  `python/tests/unit/test_py_demo_panels.py`,
  `python/tests/integration/test_demos_cycle.py`,
  `python/examples/demos/README.md`, PLAN-103, and this handoff to remove the
  obsolete collision sandbox placeholder and route users to the real collision
  rows.
- Previous local slices update `python/examples/demos/scenes/planned.py`,
  `python/examples/demos/scenes/robot_puppets.py`,
  `python/tests/unit/test_py_demo_panels.py`,
  `python/examples/demos/README.md`, PLAN-103, and this handoff to make
  planned World-port rows actionable instead of inert placeholders.
- Earlier local slices update `scripts/capture_py_demo.py`,
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

## Current Status

- [x] Local branch contains the first rigid-body GUI reliability fix:
      `0e38e3e807d Fix py-demos cycle scene frame budget`.
- [x] Full `py-demos --cycle-scenes --headless --frames 1 --hide-ui`
      completed locally across 151 scenes before this handoff.
- [x] PLAN-103 command guidance was updated to use the bounded one-frame
      cycle gate.
- [x] Handoff docs were restored because the task is still active and spans
      sessions.
- [x] The `Rigid Workflow` panel search now recognizes more DART 7 solver,
      contact-policy, backend, executor, and worker-count terminology.
- [x] `py-demo-capture -- --rigid-workflow` now plans or runs the full numbered
      rigid capture workflow and writes a workflow-level manifest.
- [x] The full `py-demo-capture -- --rigid-workflow` path completed all
      36 numbered rigid captures locally.
- [x] The workflow runner now treats a missing per-scene manifest as a failed
      capture even when the child process exits 0.
- [x] The in-viewer `Rigid Workflow` panel now shows the exact per-row capture
      command, frame count, resolution, and docked-UI mode.
- [x] Adjacent comparison/parameter rows now label the comparison axis and
      held-fixed controls in both the panel and capture metrics.
- [x] Workflow captures now write a top-level `review_index.html` contact
      sheet so all 36 row screenshots, manifests, commands, and metric
      summaries can be reviewed from one page.
- [x] Related-evidence `Find row` matches now open the matched non-numbered
      shelf scene directly instead of requiring an intermediate stop on the
      numbered source row.
- [x] `py-demo-capture -- --rigid-workflow --include-related` now appends the
      non-numbered related-evidence shelf routes to the same workflow manifest
      and review index.
- [x] `py-demo-capture -- --rigid-workflow --include-packets` now appends the
      capture-first rigid IPC stack packets to the same workflow manifest and
      review index without changing the default 36-row workflow.
- [x] `py-demo-capture -- --rigid-workflow --workflow-start-row N --workflow-end-row M`
      now supports targeted workflow reruns while
      preserving absolute row numbers and output directories.
- [x] Numbered rigid workflow captures now carry the in-viewer row guidance
      into `manifest.json` and `review_index.html`: role label, user question,
      try-first action, inspect signals, healthy signal, and scope note.
- [x] `rigid_ipc_edge_drop` is now a self-describing related-evidence route
      from `rigid_solver_compare`, with a panel and capture metrics for
      degenerate edge-barrier gap, tilt, angular speed, contact count, step
      timing, and status.
- [x] The remaining Rigid IPC shelf scenes (`rigid_ipc`, `rigid_ipc_slide`,
      `rigid_ipc_incline`, and `rigid_ipc_pile`) now publish scene-owned
      capture metrics for direct docked captures while preserving shared replay
      controls.
- [x] `py-demo-capture -- --rigid-workflow --include-ipc-shelf` now appends the
      four direct metric-backed Rigid IPC shelf scenes to the workflow manifest
      and `review_index.html` without changing the default 36-row workflow.
- [x] Row-range workflow manifests now preserve the requested optional packet
      groups in `include_related`, `include_ipc_shelf`, and `include_packets`
      while exposing `selected_include_*` fields for the selected slice.
- [x] The workflow `review_index.html` header now shows row-span,
      requested-groups, and selected-groups badges for row-range packets.
- [x] The in-viewer `Rigid Workflow` panel now shows workflow-level review
      packet commands in addition to the per-row direct capture command.
- [x] Workflow packets now accept `--video --fps` and link per-row MP4 motion
      artifacts from `review_index.html`; the in-viewer panel shows a
      current-row motion packet command.
- [x] Optional related-evidence, direct Rigid IPC shelf, and capture-first
      packet rows now carry self-describing row guidance into generated
      workflow manifests and review indexes.
- [x] Workflow manifests and review indexes now self-audit guidance coverage
      for the selected packet, and focused tests guard the full extended packet
      against unlabeled rows.
- [x] Workflow panel commands, workflow manifests, and generated review cards
      now expose live `pixi run py-demos -- --scene ...` open commands beside
      reproducible `py-demo-capture` evidence commands.
- [x] `rigid_stack_stability` now exposes a Replay timeline value track for
      top-x divergence and markers for overlap, low-clearance, top-drift, or
      solver-divergence frames.
- [x] `rigid_contact_manipulation` now exposes a Replay timeline value track
      for travel divergence and markers for contact/proximity/progress frames.
- [x] `rigid_kinematic_driver` now exposes a Replay timeline value track for
      IPC grip box travel and markers for contact/carry/slip/caveat frames.
- [x] `rigid_kinematic_normal_push` now exposes a Replay timeline value track
      for target-travel divergence and markers for
      contact/penetration/push/divergence frames.
- [x] `rigid_fixed_joint` now exposes a Replay timeline value track for
      fixed-joint offset error and markers for pose-error or residual-motion
      recovery frames.
- [x] `rigid_joint_breakage` now exposes a Replay timeline value track for
      payload release distance and markers for broken or released frames.
- [x] `rigid_distance_spring` now exposes a Replay timeline value track for
      maximum spring stretch and markers for high-stretch or off-center-spin
      frames.
- [x] `rigid_limited_joints` now exposes a Replay timeline value track for
      locked-axis error and markers for locked-error or free-axis-motion
      frames.
- [x] `rigid_joint_motor_limits` now exposes a Replay timeline value track for
      force travel gap and markers for velocity-clamp, position-stop, or
      effort-cap frames.
- [x] `rigid_joint_passive_parameters` now exposes a Replay timeline value
      track for armature position gap and markers for damping energy
      separation, Coulomb slip, or armature-lag frames.
- [x] `rigid_screw_joint_pitch` now exposes a Replay timeline value track for
      coarse/fine travel gap and markers for pitch-spread, zero-pitch contrast,
      or reverse-sign frames.
- [x] `rigid_multibody_dynamics_terms` now exposes a Replay timeline value
      track for coupled-versus-heavy response gap and markers for response
      separation, off-diagonal coupling, or heavy-load torque frames.
- [x] `rigid_link_center_of_mass` now exposes a Replay timeline value track for
      mirrored COM angle spread and markers for mirrored-angle, centered-still,
      or high-inertia-lag frames.
- [x] `rigid_link_jacobian` now exposes a Replay timeline value track for
      link-origin speed and markers for high-twist, wrench-load,
      world/body-gap, or residual-alert frames.
- [x] `rigid_multibody_solver_family` now exposes a Replay timeline value
      track for residual solve ratio and markers for solve-advantage,
      residual-only drift, or solved-tight frames while residual rows remain
      loose.
- [x] `rigid_loop_closure` now exposes a Replay timeline value track for max
      closure residual ratio and markers for solve-advantage, family-drift,
      distance-tip, or rigid-orientation frames.
- [x] `py-demo-capture` now records JSON-safe scene-owned Replay timeline
      metadata in per-scene manifests and shows Replay track labels in workflow
      `review_index.html` cards.
- [x] A fresh full 36-row rigid workflow capture after the Replay metadata
      change completed successfully and showed nineteen Replay-labeled review
      cards, matching the sidecar-documented Replay rows.
- [x] A real extended optional-row capture for rows 37-52 completed the
      related-evidence, direct Rigid IPC shelf, and both capture-first packet
      groups with docked screenshots, frame sequences, metric events, complete
      guidance, and no failed rows.
- [x] The optional capture-first packet group now includes the six-box
      `rigid_ipc_heavy_stack_packet` row after the existing four-box packet;
      an extended dry-run planned rows 51-52 / 52 with complete guidance, and a
      real row-52 docked workflow capture completed with finite heavy-stack
      metrics.
- [x] Public row-range packet rerun examples now target rows 47-48 for the
      `--include-related --include-packets` packet shape, and a focused guard
      keeps README/PLAN-103 from drifting back to the old row-47-only command.
- [x] Workflow packets now support `--continue-on-failure`, preserving
      later-row evidence after a failed row while still failing the final
      workflow manifest/exit status; failed rows are summarized in
      `manifest.json` and `review_index.html` with workflow row-range rerun
      commands.

## Goal

Make DART's Python GUI demos a practical visual-debugging surface for rigid
body simulation. The rigid category should expose the important solver,
backend, parameter, contact, constraint, and corner-case behavior in ways that
are easy to inspect, cycle, capture, and regression-test.

## Non-Goals For The Current Slice

- Do not start a broad public API expansion unless the missing API is first
  confirmed and scoped.
- Do not implement direct rigid-body impulse, sleep/wake, island activation, or
  loop-compliance rows until public `dartpy` APIs exist for them.
- Do not add new C++ demos for this task unless a future plan explicitly calls
  for it.

## Completion/Retirement Readiness Audit (2026-06-12)

Result: the current scoped rigid visual-verification slice is close to
retirement, but the dev-task folder should not be deleted yet. The missing
piece is not another scene; it is maintainer acceptance of the current scope and
the normal completion-PR cleanup that promotes the final durable note and
removes this working folder in the same change.

Evidence supporting readiness:

- The latest local checkpoint is
  `08f710b793b Record regenerated rigid workflow evidence`, which records the
  current-HEAD packet refresh and review-index link audit. At the latest recon,
  the branch was clean, had no associated pull request, and was 38 commits
  ahead of
  `origin/feature/rigid-body-gui-visual-verification`; a `gh pr list` query for
  the current branch reported no pull request.
- The durable sidecar
  `docs/plans/103-examples-strategy/rigid-body-visual-verification.md` owns the
  curated 36-row workflow, optional related/IPC-shelf/packet rows, public
  API-gap audit, capture commands, validation snapshot, and follow-up list.
- `docs/dev_tasks/rigid_body_visual_verification/PR_DRAFT.md` is the
  maintainer-facing PR body seed for the current branch. It points back to the
  durable sidecar, py-demos README, generated manifests, current review-index
  artifacts, and required acceptance checkbox.
- `docs/dev_tasks/rigid_body_visual_verification/COMPLETION_AUDIT.md` maps the
  original objective to current evidence and records the remaining external
  gates: push/PR approval, DART 7.0 milestone, maintainer acceptance, and
  same-PR dev-task cleanup. It also records the AI principle audit for this
  substantial AI-assisted workflow change.
- The user-facing `python/examples/demos/README.md` describes the same ordered
  rigid workflow, in-viewer `Rigid Workflow` panel, replay metadata, workflow
  capture packet, related-evidence routes, and optional packet guidance.
- `CHANGELOG.md` already records the DART 7 rigid workflow command, review
  packet, related-evidence groups, replay metadata, and rigid scene additions,
  so the PR draft checklist marks the changelog item complete.
- The fresh full workflow capture
  `build/captures/rigid_workflow_rows_01_36_1781309127` completed all 36
  numbered rows with `failed_count=0`, guidance complete, docked screenshots,
  frame sequences, and a generated `review_index.html`.
- The fresh optional extended packet
  `build/captures/rigid_workflow_optional_rows_37_52_1781309448` completed rows
  37-52 with `failed_count=0`, guidance complete, and all selected related,
  direct IPC shelf, and capture-first packet groups present, including the
  heavy stack packet at row 52.
- The generated review indexes above were produced after the current
  review-index link-normalization and top-level command-provenance fix. The
  link audit found 0 missing local assets in both review indexes.
- Long optional packets now have an explicit failure-recovery path:
  `--continue-on-failure` records `failed_rows`, review-index Failed Rows
  summaries, and workflow row-range rerun commands that preserve
  `--include-related`, `--include-ipc-shelf`, `--include-packets`, and absolute
  row numbering.
- Drift guards in `python/tests/integration/test_demos_cycle.py`,
  `python/tests/unit/test_capture_py_demo.py`, and
  `python/tests/unit/test_py_demo_panels.py` cover sidecar/README ordering,
  capture-command sync, deferred public API gaps, workflow guidance, optional
  packet guidance completeness, Replay timeline metadata, and review-index
  manifest fields.
- The current API-gap guard
  `test_rigid_visual_verification_deferred_api_gaps_are_documented` passed, and
  live `dartpy` introspection still finds direct rigid-body impulse,
  sleep/wake, island activation, and loop-closure compliance rows blocked by
  missing public surfaces. Point-force, joint-space impulse response,
  energy/momentum, breakage, and spring/compliance-like rows are already covered
  or intentionally scoped by existing rows.

Remaining before retirement:

- Get explicit maintainer acceptance that the maintained 36-row workflow plus
  optional packet groups, now totaling 52 rows when all optional groups are
  selected, are the completed scope for this dev task.
- Use `docs/dev_tasks/rigid_body_visual_verification/PR_DRAFT.md` as the PR
  body seed after push/PR creation is explicitly approved.
- Use `docs/dev_tasks/rigid_body_visual_verification/COMPLETION_AUDIT.md` to
  avoid conflating local review readiness with completed maintainer acceptance.
- If accepted, do the cleanup in the completing PR: add only a short durable
  close-out note if the sidecar or Python README still lacks it, then
  `git rm -r docs/dev_tasks/rigid_body_visual_verification`.
- Keep direct rigid-body impulse, sleep/wake, island activation, and
  loop-closure compliance rows as durable PLAN-103 follow-ups until public
  `dartpy` APIs exist; those deferrals are not blockers to retiring the current
  scoped GUI workflow.
- Run the repository-mandated `pixi run lint` before the cleanup commit, then
  run the strongest relevant docs/Python gates for the final PR.

Acceptance decision packet:

- Accept current scope if the maintainer agrees that the durable sidecar,
  Python README, workflow panel, review index, capture helper, and drift guards
  are the completed rigid-body visual verification surface for this cycle.
- Do not accept by default if a maintainer wants a new public API first; record
  that as a new durable PLAN-103 or API-promotion follow-up rather than keeping
  this working folder open indefinitely.
- Once accepted, the completing change should be small: update the durable
  owner docs only if the final close-out note is missing, delete this dev-task
  folder, run the docs/Python gates, and request approval before pushing.

## Branch Snapshot

- Branch: `feature/rigid-body-gui-visual-verification`
- Latest committed local checkpoint is
  `08f710b793b Record regenerated rigid workflow evidence`; the branch had no
  associated pull request and was 38 commits ahead of
  `origin/feature/rigid-body-gui-visual-verification`.
- Earlier branch snapshots below are historical archaeology from the
  multi-session implementation and should not override the current handoff,
  readiness audit, or `git status -sb`.
- Historical implementation checkpoints covered by this hand-off included the
  failed-row rerun, failed-row summary, continue-on-failure, row-range
  guidance, and heavy-packet slices.
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
- `d98abdde973 Refresh rigid visual verification handoff` is a pushed
  docs-only checkpoint.
- Local `HEAD` before the link center-of-mass implementation commit was
  `7438e13cca9 Add multibody dynamics replay timeline`; the branch was
  observed clean and eleven commits ahead of
  `origin/feature/rigid-body-gui-visual-verification` before this slice.
- Local `HEAD` before the link-Jacobian implementation work was
  `c7042091c2b Add link center of mass replay timeline`; the branch was ahead
  of `origin/feature/rigid-body-gui-visual-verification` by twelve commits and
  carried docs-only stop-handoff edits that this checkpoint supersedes.
- Local `HEAD` before the multibody dynamics-terms implementation commit was
  `29ab458fc01 Add screw joint replay timeline`; the branch was observed clean
  and ten commits ahead of
  `origin/feature/rigid-body-gui-visual-verification` before this slice.
- Local `HEAD` before the screw-joint pitch implementation commit was
  `6f307966524 Add passive joint replay timeline`; it was nine commits ahead
  of `origin/feature/rigid-body-gui-visual-verification` before that slice.
- The contact-manipulation, kinematic-driver, normal-push, fixed-joint,
  joint-breakage, distance-spring, limited-joints, motor-limits,
  passive-parameters, screw-joint pitch, multibody dynamics-terms, link
  center-of-mass, link-Jacobian, multibody solver-family, loop-closure, Replay
  capture-metadata, full workflow refresh, and optional-packet hand-off
  checkpoints are now expected in branch history through
  `91f53e5ae8e Record rigid workflow handoff evidence`; verify with
  `git status -sb` before acting.
- There is no PR associated with this branch at checkpoint time.
- Do not push again without explicit approval in that session.

## What The Local Commit Changed

- `dart/gui/detail/application.cpp`
  - Treats `--frames N` as the per-scene budget when `--cycle-scenes` is used.
  - Stops forwarding that value as the global `RunOptions.maxFrames` in cycle
    mode.
  - Sets `appOptions.run.maxFrames = -1` for cycle mode so headless defaults do
    not end the whole catalog after one global frame.
  - Prints and flushes progress lines such as
    `Cycling demo scene 1/151: rigid_body`.
- `python/tests/integration/test_demos_cycle.py`
  - Adds coverage that the cycle frame budget applies per scene.
  - Keeps the basic cycle success test bounded with `--frames 1`.
- `CHANGELOG.md`, `docs/plans/dashboard.md`,
  `docs/plans/103-examples-strategy.md`, and
  `python/examples/demos/README.md`
  - Document the bounded cycle command, the 151-scene catalog state, and the
    per-scene meaning of `--frames`.

## What The Latest Pushed Commit Changed

- `python/examples/demos/runner.py`
  - Adds aliases so users can find the intended rigid workflow rows using
    terms such as `RigidBodySolver`, `ContactSolverMethod`, `Taskflow
executor`, `backend/executor`, `worker count`, `contact model`, and
    `solver policy`.
  - Adds maintained capture specs to each `RigidWorkflowGuide`, so the
    in-viewer workflow panel shows the exact command for regenerating that
    row's visual evidence.
- `python/tests/unit/test_py_demo_panels.py`
  - Locks the new search routes to `rigid_solver_compare`,
    `rigid_executor_equivalence`, `rigid_step_diagnostics`, and
    `rigid_contact_solver_compare`.
  - Locks the per-row workflow capture command panel text and comparison-axis
    labels.
- `python/examples/demos/scenes/rigid_timestep_sensitivity.py`,
  `python/examples/demos/scenes/rigid_step_diagnostics.py`,
  `python/examples/demos/scenes/rigid_contact_scale_budget.py`, and
  `python/examples/demos/scenes/rigid_joint_passive_parameters.py`
  - Add explicit `comparison_axis` and `held_fixed` panel/capture-metrics
    labels so users can tell solver, executor, workload, and parameter-family
    changes apart.
- `scripts/capture_py_demo.py`
  - Adds maintained `rigid_workflow_capture_specs()` and
    `--rigid-workflow`.
  - `--rigid-workflow --dry-run` writes a top-level plan manifest without
    rendering.
  - `--rigid-workflow` runs each numbered rigid capture into
    `scenes/NN_<scene_id>/` and updates a workflow-level manifest with each
    per-scene manifest path and status.
  - Missing per-scene manifests are hard failures with
    `failure_reason: missing_manifest`, so the top-level workflow cannot report
    complete without evidence for every row.
  - Every dry-run, progress update, failure, and successful run also rewrites
    `review_index.html`, a static contact sheet that links each row's
    manifest, screenshot, frames directory, command, and metrics summary.
- `python/tests/unit/test_capture_py_demo.py`
  - Covers dry-run plan output, fake scene-manifest aggregation, and child
    process isolation for per-scene workflow captures.
  - Adds a regression that a child process returning 0 without a scene manifest
    fails the workflow and stops at the missing-evidence row.
  - Checks that the workflow review index exists and lists planned, captured,
    metric-bearing, and failed rows.
- `python/tests/integration/test_demos_cycle.py`
  - Keeps the helper's maintained capture spec synchronized with the sidecar
    and README command lists.
  - Checks the workflow-panel capture commands and comparison-axis metadata
    remain synchronized with the maintained row specs.
- `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`,
  `python/examples/demos/README.md`, and `CHANGELOG.md`
  - Document the workflow capture command, review index contact sheet, and new
    search terminology.

## Verified Before The Stop Request

These commands were run before the user requested no further verification:

```bash
pixi run build
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_runner_cycle_frames_budget_applies_per_scene python/tests/integration/test_demos_cycle.py::test_runner_cycle_returns_zero -q
PYTHONPATH=build/default/cpp/Release-docking/python:build/default/cpp/Release-docking/python/dartpy:python DART_PARALLEL_JOBS=3 CTEST_PARALLEL_LEVEL=3 CMAKE_BUILD_PARALLEL_LEVEL=3 timeout 180s pixi run py-demos -- --cycle-scenes --headless --frames 1 --hide-ui
pixi run lint
git diff --check
```

Observed results before the stop request:

- Build passed.
- Focused cycle tests passed: `2 passed`.
- The public cycle command completed all 151 demo scenes with exit code 0.
- Lint passed.
- `git diff --check` was clean.

## Verified In The Latest Session

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run py-demo-capture -- --rigid-workflow --dry-run --output-dir /tmp/dart_capture_rigid_workflow_dry_run
```

Observed results:

- Focused pytest initially reported `5 passed`.
- The public dry-run command wrote
  `/tmp/dart_capture_rigid_workflow_dry_run/manifest.json`.
- The dry-run manifest reports workflow `rigid_visual_verification`, status
  `planned`, `capture_count` 36, first scene `rigid_body`, and last scene
  `rigid_loop_closure`.
- The dry-run command triggered Pixi's `py-demo-capture` build path and rebuilt
  the GUI Python target because the branch includes the earlier C++ GUI change.

After the full workflow capture exposed buffered parent progress, the progress
flush and child-process capture updates were added. These focused checks were
then rerun before the final stop request:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_scene_capture_runs_in_child_process python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run py-demo-capture -- --rigid-workflow --dry-run --output-dir /tmp/dart_capture_rigid_workflow_dry_run
```

Observed results:

- Focused pytest reported `6 passed`.
- The public dry-run command completed with exit code 0 and printed the full
  36-command plan.
- `pixi run lint` had been started after those updates, but the user then
  requested no further verification. Do not claim a latest lint or
  `git diff --check` result until rerun in a future session.

## Verified In The Current Continuation

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_fails_when_scene_manifest_is_missing python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_scene_capture_runs_in_child_process python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run py-demo-capture -- --rigid-workflow --dry-run --output-dir /tmp/dart_capture_rigid_workflow_dry_run
pixi run lint
git diff --check
```

Observed results:

- Focused pytest reported `7 passed`.
- The public dry-run command completed with exit code 0 and printed the full
  36-command plan.
- `pixi run lint` passed; Black reformatted `scripts/capture_py_demo.py`.
- `git diff --check` was clean.

After two read-only explorer passes, the current continuation also added the
per-row capture command to the `Rigid Workflow` panel and the comparison
contract labels for adjacent parameter/profiling rows. This focused check
covered the workflow panel, capture helper failure paths, capture-spec sync,
and comparison metrics:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_comparison_panels_label_the_compared_axis python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_guides_expose_capture_commands python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_fails_when_scene_manifest_is_missing python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_scene_capture_runs_in_child_process python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_timestep_sensitivity_orders_freefall_error_by_step_size python/tests/integration/test_demos_cycle.py::test_rigid_step_diagnostics_reports_profile_and_memory_counters python/tests/integration/test_demos_cycle.py::test_rigid_contact_scale_budget_orders_contact_loads python/tests/integration/test_demos_cycle.py::test_rigid_joint_passive_parameters_order_passive_response -q
pixi run py-demo-capture -- --rigid-workflow --dry-run --output-dir /tmp/dart_capture_rigid_workflow_dry_run
pixi run lint
git diff --check
```

Observed result:

- Focused pytest reported `14 passed`.
- The public dry-run command completed with exit code 0 and printed the full
  36-command plan.
- `pixi run lint` passed.
- `git diff --check` was clean.

The current continuation then added the workflow review index/contact sheet.
After `pixi run lint` reformatted `scripts/capture_py_demo.py`, this focused
check was rerun:

```bash
pixi run lint
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_comparison_panels_label_the_compared_axis python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_guides_expose_capture_commands python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_fails_when_scene_manifest_is_missing python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_scene_capture_runs_in_child_process python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_timestep_sensitivity_orders_freefall_error_by_step_size python/tests/integration/test_demos_cycle.py::test_rigid_step_diagnostics_reports_profile_and_memory_counters python/tests/integration/test_demos_cycle.py::test_rigid_contact_scale_budget_orders_contact_loads python/tests/integration/test_demos_cycle.py::test_rigid_joint_passive_parameters_order_passive_response -q
pixi run py-demo-capture -- --rigid-workflow --dry-run --output-dir /tmp/dart_capture_rigid_workflow_dry_run
test -f /tmp/dart_capture_rigid_workflow_dry_run/review_index.html && jq -r '.artifacts.review_index, .capture_count, .status' /tmp/dart_capture_rigid_workflow_dry_run/manifest.json && rg -n "DART rigid workflow review index|rigid_loop_closure" /tmp/dart_capture_rigid_workflow_dry_run/review_index.html
git diff --check
```

Observed result:

- Focused pytest reported `14 passed`.
- The public dry-run completed with exit code 0 and printed all 36 planned
  capture commands.
- The dry-run manifest reported
  `/tmp/dart_capture_rigid_workflow_dry_run/review_index.html`, `36`, and
  `planned`; the generated review index contained the
  `DART rigid workflow review index` heading and the final
  `rigid_loop_closure` row.
- `pixi run lint` passed.
- `git diff --check` was clean.

## Verified In The Full Workflow Capture Session

```bash
pixi run py-demo-capture -- --rigid-workflow --output-dir /tmp/dart_capture_rigid_workflow_full_review_index_1781259714
```

Observed results:

- The command completed with exit code 0.
- The top-level manifest
  `/tmp/dart_capture_rigid_workflow_full_review_index_1781259714/manifest.json`
  reported workflow `rigid_visual_verification`, status `complete`,
  `capture_count` 36, `completed_count` 36, `failed_count` 0, and elapsed time
  `314.278` seconds.
- Every capture entry had `return_code=0`, `status=captured`, and
  `manifest_exists=true`.
- The first per-scene manifest was
  `/tmp/dart_capture_rigid_workflow_full_review_index_1781259714/scenes/01_rigid_body/manifest.json`.
- The last per-scene manifest was
  `/tmp/dart_capture_rigid_workflow_full_review_index_1781259714/scenes/36_rigid_loop_closure/manifest.json`.
- The top-level manifest recorded
  `/tmp/dart_capture_rigid_workflow_full_review_index_1781259714/review_index.html`
  under `artifacts.review_index`.
- The generated review index existed, contained 36 screenshot thumbnails, and
  linked the first and last screenshots plus comparison-axis/held-fixed metric
  summaries for representative workflow rows.

## Verified In The Related-Search Routing Continuation

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_opens_related_evidence_search_matches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_related_evidence_targets python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_related_evidence_routes_to_other_shelves -q
pixi run lint
git diff --check
```

Observed results:

- Focused pytest reported `3 passed`.
- `pixi run lint` passed.
- `git diff --check` was clean.

## Verified In The Related-Evidence Capture Bundle Continuation

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_related_evidence python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_include_related_requires_workflow python/tests/integration/test_demos_cycle.py::test_rigid_visual_related_evidence_capture_commands_are_documented -q
pixi run py-demo-capture -- --rigid-workflow --include-related --dry-run --output-dir /tmp/dart_capture_rigid_workflow_related_dry_run_current
jq -r '.include_related, .capture_count, .captures[36].workflow_group, .captures[36].scene, .captures[45].scene, .artifacts.review_index' /tmp/dart_capture_rigid_workflow_related_dry_run_current/manifest.json
rg -n "related_evidence|avbd_rigid_prismatic_motor|46/46" /tmp/dart_capture_rigid_workflow_related_dry_run_current/review_index.html
```

Observed results:

- Focused pytest reported `5 passed`.
- The public dry-run completed with exit code 0 and printed all 46 planned
  capture commands: 36 numbered rows plus 10 related-evidence routes.
- The dry-run manifest reported `include_related=true`, `capture_count=46`,
  `captures[36].workflow_group=related_evidence`, first related scene
  `floating_base`, final scene `avbd_rigid_prismatic_motor`, and a
  `review_index.html` path.
- The generated review index contained related-evidence cards and the final
  `46/46 avbd_rigid_prismatic_motor` row.

## Verified In The Capture-First Packet Bundle Continuation

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_related_evidence python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_capture_first_packets python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_extra_groups_require_workflow python/tests/integration/test_demos_cycle.py::test_rigid_visual_related_evidence_capture_commands_are_documented python/tests/integration/test_demos_cycle.py::test_rigid_visual_capture_first_packets_are_documented -q
pixi run py-demo-capture -- --rigid-workflow --include-related --include-packets --dry-run --output-dir /tmp/dart_capture_rigid_workflow_packets_edge_drop_dry_run
jq -r '.include_related, .include_packets, .capture_count, .captures[46].workflow_group, .captures[46].scene, .artifacts.review_index' /tmp/dart_capture_rigid_workflow_packets_edge_drop_dry_run/manifest.json
rg -n "capture_first_packet|rigid_ipc_stack_packet|47/47" /tmp/dart_capture_rigid_workflow_packets_edge_drop_dry_run/review_index.html
```

Observed results:

- Focused pytest reported `8 passed`.
- The public dry-run completed with exit code 0 and printed all 47 planned
  capture commands: 36 numbered rows, 10 related-evidence routes, and the
  capture-first rigid IPC stack packet.
- The dry-run manifest reported `include_related=true`,
  `include_packets=true`, `capture_count=47`,
  `captures[46].workflow_group=capture_first_packet`, final scene
  `rigid_ipc_stack_packet`, and a `review_index.html` path.
- The generated review index contained the final
  `47/47 rigid_ipc_stack_packet` packet row.

## Verified In The Row-Range Rerun Continuation

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_select_row_range python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_can_resume_from_selected_row python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_extra_groups_require_workflow python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_row_selection_validates_bounds -q
pixi run py-demo-capture -- --rigid-workflow --include-related --include-packets --workflow-start-row 47 --workflow-end-row 47 --dry-run --output-dir /tmp/dart_capture_rigid_workflow_row_rerun_edge_drop_dry_run
jq -r '.capture_count, .workflow_total_count, .workflow_row_start, .workflow_row_end, .captures[0].order, .captures[0].count, .captures[0].scene, .captures[0].workflow_group' /tmp/dart_capture_rigid_workflow_row_rerun_edge_drop_dry_run/manifest.json
rg -n "47/47|rigid_ipc_stack_packet|capture_first_packet" /tmp/dart_capture_rigid_workflow_row_rerun_edge_drop_dry_run/review_index.html
```

Observed results:

- Focused pytest reported `11 passed`.
- The public dry-run completed with exit code 0 and printed only the selected
  row-47 packet capture command.
- The dry-run manifest reported `capture_count=1`,
  `workflow_total_count=47`, `workflow_row_start=47`, `workflow_row_end=47`,
  selected row order/count `47/47`, scene `rigid_ipc_stack_packet`, and
  `workflow_group=capture_first_packet`.
- The generated review index contained the absolute
  `47/47 rigid_ipc_stack_packet` row.

## Verified In The Review-Index Guidance Continuation

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_select_row_range python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_can_resume_from_selected_row python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_extra_groups_require_workflow python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_row_selection_validates_bounds -q
pixi run py-demo-capture -- --rigid-workflow --workflow-start-row 15 --workflow-end-row 15 --dry-run --output-dir /tmp/dart_capture_rigid_workflow_guidance_dry_run
jq -r '.captures[0].workflow_label, .captures[0].user_question, .captures[0].try_first, .captures[0].healthy_signal, .captures[0].scope' /tmp/dart_capture_rigid_workflow_guidance_dry_run/manifest.json
rg -n "How do the rigid method families differ visually|Healthy: wall clearance|Generic thin-wall comparison" /tmp/dart_capture_rigid_workflow_guidance_dry_run/review_index.html
```

Observed results:

- Focused pytest reported `11 passed`.
- The public row-15 dry-run completed with exit code 0.
- The dry-run manifest reported the `Solver family` role, the maintained
  solver-family user question, try-first guidance, healthy signal, and scope
  note.
- The generated review index contained the same solver-family question,
  healthy signal, and scope note beside the row card.

## Verified In The Rigid IPC Edge-Drop Continuation

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_ipc_edge_drop_reports_degenerate_contact_metrics python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_related_evidence_routes_are_valid python/tests/integration/test_demos_cycle.py::test_rigid_visual_routes_publish_self_describing_capture_metrics python/tests/integration/test_demos_cycle.py::test_rigid_visual_related_evidence_capture_commands_are_documented python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_related_evidence_routes_to_other_shelves python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_related_evidence_targets python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_opens_related_evidence_search_matches -q
```

Observed results:

- Focused pytest reported `9 passed`.
- The new edge-drop route reports `row=rigid_ipc_edge_drop`,
  `related_source_row=rigid_solver_compare`, `solver=rigid_ipc`,
  `scope=degenerate_edge_contact_capability`, near-barrier clearance, and
  nonzero angular/tilt motion.
- The related-evidence route table, capture command list, capture-helper specs,
  panel coverage, search routing, direct related route, and unnumbered-route
  guard all include `rigid_ipc_edge_drop`.
- The public `--include-related` dry-run reported `capture_count=46`,
  `40/46 rigid_ipc_edge_drop`, and final
  `46/46 avbd_rigid_prismatic_motor`.
- The public row-range packet dry-run reported `workflow_total_count=47` and
  `47/47 rigid_ipc_stack_packet`.
- The real docked capture wrote a nonblank 960x540 screenshot with docked UI,
  71 converted PNG frames, latest status `edge-barrier`, minimum barrier gap
  about `0.000384` m, maximum tilt about `55.33` degrees, and maximum angular
  speed about `0.555` rad/s.

## Verified In The Rigid IPC Shelf Metrics Continuation

```bash
pixi run lint
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_ipc_shelf_scenes_report_capture_metrics python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q
pixi run py-demo-capture -- --scene rigid_ipc_pile --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_rigid_ipc_pile_metrics_current
git diff --check
```

Observed results:

- `pixi run lint` passed.
- Focused pytest reported `2 passed`.
- The direct Rigid IPC shelf scenes `rigid_ipc`, `rigid_ipc_slide`,
  `rigid_ipc_incline`, and `rigid_ipc_pile` now report scene-owned capture
  metrics for row identity, IPC solver label, scope, time-step/world-time,
  friction, status, contact counts, step timing, and scene-specific gap, speed,
  travel, height, span, or pile summaries. The focused test also checks that
  each scene preserves `replay_sync` and declares
  `replay_live_step_is_stateless`.
- A real docked
  `pixi run py-demo-capture -- --scene rigid_ipc_pile --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_rigid_ipc_pile_metrics_current`
  capture wrote a 960x540 screenshot with docked UI detected, 23 converted PNG
  frames, 24 scene-metric events, latest row `rigid_ipc_pile`, scope
  `multi_box_barrier_pile`, 25 history samples, `box_count=3`, maximum history
  speed about `1.177` m/s, and minimum history clearance about `0.276` m.
- `git diff --check` was clean.

## Verified In The Direct Rigid IPC Shelf Capture Bundle Continuation

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_related_evidence python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_direct_ipc_shelf python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_capture_first_packets python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_extra_groups_require_workflow python/tests/integration/test_demos_cycle.py::test_rigid_visual_related_evidence_capture_commands_are_documented python/tests/integration/test_demos_cycle.py::test_rigid_visual_capture_first_packets_are_documented python/tests/integration/test_demos_cycle.py::test_rigid_visual_direct_ipc_shelf_captures_are_documented -q
pixi run py-demo-capture -- --rigid-workflow --include-ipc-shelf --dry-run --output-dir /tmp/dart_capture_rigid_workflow_ipc_shelf_only_dry_run_current
jq -r '.include_related, .include_ipc_shelf, .include_packets, .capture_count, .workflow_total_count, .captures[36].workflow_group, .captures[36].scene, .captures[39].scene' /tmp/dart_capture_rigid_workflow_ipc_shelf_only_dry_run_current/manifest.json
rg -n "37/40 rigid_ipc|40/40 rigid_ipc_pile|rigid_ipc_shelf" /tmp/dart_capture_rigid_workflow_ipc_shelf_only_dry_run_current/review_index.html
pixi run py-demo-capture -- --rigid-workflow --include-related --include-ipc-shelf --include-packets --workflow-start-row 47 --workflow-end-row 51 --dry-run --output-dir /tmp/dart_capture_rigid_workflow_ipc_shelf_dry_run_current
jq -r '.include_related, .include_ipc_shelf, .include_packets, .capture_count, .workflow_total_count, .captures[0].order, .captures[0].workflow_group, .captures[0].scene, .captures[3].scene, .captures[4].workflow_group, .captures[4].scene' /tmp/dart_capture_rigid_workflow_ipc_shelf_dry_run_current/manifest.json
rg -n "47/51 rigid_ipc|50/51 rigid_ipc_pile|51/51 rigid_ipc_stack_packet|rigid_ipc_shelf" /tmp/dart_capture_rigid_workflow_ipc_shelf_dry_run_current/review_index.html
```

Observed results:

- Focused pytest reported `11 passed`.
- The public `--include-ipc-shelf` dry-run completed with 40 planned captures:
  36 numbered rows plus direct Rigid IPC shelf rows 37-40.
- The dry-run manifest reported `include_ipc_shelf=true`,
  `capture_count=40`, `workflow_total_count=40`, first IPC shelf scene
  `rigid_ipc`, final IPC shelf scene `rigid_ipc_pile`, and
  `workflow_group=rigid_ipc_shelf`.
- The generated review index contained `37/40 rigid_ipc`,
  `40/40 rigid_ipc_pile`, and the `rigid_ipc_shelf` group.
- The combined row-range dry-run with related evidence, IPC shelf rows, and
  packets selected rows 47-51. It reported the direct IPC shelf rows as
  absolute 47-50 and `rigid_ipc_stack_packet` as the explicit combined-mode
  row 51.

## Verified In The Workflow Manifest And Review Index Metadata Continuations

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

Observed results:

- Focused pytest reported `16 passed` for manifest metadata and `4 passed` for
  the review-index metadata follow-up.
- The public combined row-range dry-run selected rows 47-51 and wrote a
  workflow manifest where requested flags were all true:
  `include_related=true`, `include_ipc_shelf=true`, and
  `include_packets=true`.
- The same manifest reported the selected slice separately:
  `selected_include_related=false`, `selected_include_ipc_shelf=true`, and
  `selected_include_packets=true`, with `capture_count=5`,
  `workflow_total_count=51`, `workflow_row_start=47`, and
  `workflow_row_end=51`.
- The generated review index contained the absolute `47/51 rigid_ipc`,
  `50/51 rigid_ipc_pile`, and `51/51 rigid_ipc_stack_packet` rows.
- The updated review index header also contained row-span
  `47-51 / 51`, requested-groups
  `numbered, related, ipc shelf, packets`, and selected-groups
  `ipc shelf, packets`.
- `pixi run lint` passed.
- `git diff --check` was clean.

## Verified In The Workflow Panel Packet-Command Continuation

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_guides_expose_capture_commands python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_route_rows_request_scene_switches -q
pixi run lint
```

Observed results:

- Focused panel pytest reported `3 passed`.
- The tested `Rigid Workflow` panel now emits the per-row capture command plus
  a `Review packet` section containing the full numbered workflow command, the
  current-row row-range rerun command, and the extended
  related/IPC-shelf/packet command.
- `pixi run lint` passed.
- `git diff --check` was not rerun after the final handoff edits because the
  user explicitly requested no further verification.

## Verified In The Workflow Video Packet Continuation

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_visual_capture_manifest_records_video_artifact python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_request_video_commands python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_links_scene_videos python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/integration/test_demos_cycle.py::test_rigid_visual_motion_capture_video_flags_are_documented -q
DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run py-demo-capture -- --rigid-workflow --workflow-start-row 15 --workflow-end-row 15 --video --fps 24 --dry-run --output-dir /tmp/dart_capture_rigid_workflow_video_dry_run_current
jq -r '.capture_count, .workflow_total_count, .workflow_row_start, .workflow_row_end, .captures[0].command' /tmp/dart_capture_rigid_workflow_video_dry_run_current/manifest.json
rg -n "15/36 rigid_solver_compare|--video --fps 24|selected groups|requested groups" /tmp/dart_capture_rigid_workflow_video_dry_run_current/review_index.html
pixi run lint
git diff --check
```

Observed results:

- Focused pytest reported `5 passed`.
- The public row-15 workflow dry-run completed with exit code 0 and generated a
  selected-row command ending in `--show-ui --video --fps 24`.
- The dry-run manifest reported `capture_count=1`,
  `workflow_total_count=36`, `workflow_row_start=15`, and
  `workflow_row_end=15`.
- The generated review index contained row `15/36 rigid_solver_compare`,
  requested/selected group badges, and the `--video --fps 24` command.
- `pixi run lint` passed.
- `git diff --check` was clean.

## Verified In The Guidance-Audit Continuation

The current guidance-audit continuation adds a self-audit for rigid workflow
row guidance coverage:

- `scripts/capture_py_demo.py` adds required workflow guidance fields, reports
  missing self-description rows in `manifest.json`, and shows a guidance badge
  plus warning block in `review_index.html`.
- `python/tests/unit/test_capture_py_demo.py` adds coverage for complete
  guidance in the full extended packet and for manifest/review-index reporting
  when a row is missing guidance.
- `CHANGELOG.md`,
  `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`, and
  `python/examples/demos/README.md` describe the generated guidance audit.

Focused validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_related_evidence python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_direct_ipc_shelf python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_capture_first_packets python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_row_range_preserves_requested_extra_groups python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_full_extended_plan_has_complete_guidance python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_manifest_reports_missing_guidance -q
```

Public dry-run and artifact inspection:

```bash
pixi run py-demo-capture -- --rigid-workflow --include-related --include-ipc-shelf --include-packets --workflow-start-row 37 --workflow-end-row 51 --dry-run --output-dir /tmp/dart_capture_rigid_workflow_guidance_audit_dry_run_current
jq -r '.guidance_complete, .guidance_missing_count, (.guidance_missing_rows | length), .capture_count, .workflow_total_count, .workflow_row_start, .workflow_row_end, .captures[0].workflow_label, .captures[10].workflow_label, .captures[14].workflow_label' /tmp/dart_capture_rigid_workflow_guidance_audit_dry_run_current/manifest.json
rg -n "<strong>guidance</strong> complete|Rows Missing Guidance|Related evidence|Rigid IPC shelf|Capture-first packet|37/51 floating_base|51/51 rigid_ipc_stack_packet" /tmp/dart_capture_rigid_workflow_guidance_audit_dry_run_current/review_index.html
```

Observed results:

- Focused pytest reported `7 passed`.
- The public dry-run selected rows 37-51 from the fully extended packet and
  wrote `capture_count=15`, `workflow_total_count=51`,
  `workflow_row_start=37`, and `workflow_row_end=51`.
- The generated manifest reported `guidance_complete=true`,
  `guidance_missing_count=0`, and an empty `guidance_missing_rows` list.
- The selected packet carried row labels for `Related evidence`,
  `Rigid IPC shelf`, and `Capture-first packet`.
- The generated `review_index.html` contained
  `<strong>guidance</strong> complete`, the absolute
  `37/51 floating_base` and `51/51 rigid_ipc_stack_packet` rows, and no
  `Rows Missing Guidance` warning.

## Verified In The Live Open-Command Continuation

The current continuation closes the loop from generated workflow evidence back
into the live GUI:

- `python/examples/demos/runner.py` adds
  `_rigid_workflow_viewer_command(scene_id, width, height)` and prints a live
  `pixi run py-demos -- --scene ... --width ... --height ...` command in the
  `Rigid Workflow` panel before the existing row capture command.
- `scripts/capture_py_demo.py` adds `_viewer_command(...)`, writes
  `viewer_command` into each workflow capture entry, and renders review cards
  with separate `open live` and `capture evidence` command blocks.
- `python/tests/unit/test_capture_py_demo.py` and
  `python/tests/unit/test_py_demo_panels.py` lock the manifest field,
  review-index labels, backend propagation, and panel tooltip.
- `CHANGELOG.md`, `python/examples/demos/README.md`, and
  `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`
  describe the live-open versus capture-evidence command split.

Focused validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_request_video_commands python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_full_extended_plan_has_complete_guidance python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_manifest_reports_missing_guidance -q
pixi run py-demo-capture -- --rigid-workflow --workflow-start-row 15 --workflow-end-row 15 --dry-run --output-dir /tmp/dart_capture_rigid_workflow_live_command_dry_run_current_1781271290
jq -r '.capture_count, .workflow_total_count, .workflow_row_start, .workflow_row_end, .captures[0].scene, .captures[0].viewer_command, .captures[0].command' /tmp/dart_capture_rigid_workflow_live_command_dry_run_current_1781271290/manifest.json
rg -n "open live|capture evidence|pixi run py-demos -- --scene rigid_solver_compare --width 960 --height 540|pixi run py-demo-capture -- --scene rigid_solver_compare" /tmp/dart_capture_rigid_workflow_live_command_dry_run_current_1781271290/review_index.html
pixi run lint
```

Observed results:

- Focused pytest reported `5 passed` before and after `pixi run lint`
  reformatted `scripts/capture_py_demo.py`.
- The public row-15 dry-run completed with exit code 0 and wrote a manifest
  with `capture_count=1`, `workflow_total_count=36`,
  `workflow_row_start=15`, `workflow_row_end=15`, scene
  `rigid_solver_compare`, viewer command
  `pixi run py-demos -- --scene rigid_solver_compare --width 960 --height 540`,
  and the paired `pixi run py-demo-capture -- --scene rigid_solver_compare`
  capture command.
- The generated `review_index.html` contained `open live`,
  `capture evidence`, the live viewer command, and the capture command.
- `pixi run lint` passed.

## Key Context

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
  context; final evidence for this slice is recorded in the row 35 validation
  section below.
- The completed row 36 slice in `rigid_loop_closure.py` computes a Replay
  value from latest per-family `residual_ratio_history`, then top-level
  `*_residual_ratio` fields, then residual-over-solved ratios from
  `last_metrics`. Its marker logic currently flags large solve-ratio frames,
  residual-versus-solved closure separation, distance-family tip drift with
  near-zero distance error, and rigid-orientation solve frames.
- The row 36 test edit in
  `test_rigid_loop_closure_compares_closure_families` covers the label, current
  snapshot signal, top-level and `last_metrics` fallbacks, solve-ratio marker,
  residual marker, rigid-orientation marker, distance-tip marker, and
  quiet-frame behavior.
- The durable rigid workflow sidecar is
  `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`.
- The sidecar already records the rigid visualization scope, API-gap audit, and
  36 workflow rows.
- The latest API-gap audit in that sidecar says public `dartpy` currently lacks
  direct rigid-body impulse, sleep/wake/island activation, and loop-compliance
  APIs, so those rows should remain deferred unless that changes.
- Two read-only explorer subagents informed the latest work:
  one recommended a single rigid workflow capture mode, and one recommended
  additional solver/backend/contact-policy search aliases.
- A later read-only explorer recommended this guidance-audit shape:
  non-failing completeness reporting over selected captures, required fields
  for role label, user question, try-first action, inspect signals, healthy
  signal, and scope, plus manifest fields and review-index warning surfacing
  missing guidance.
- The latest read-only explorer returned after the stop instruction and was
  later acted on. It recommended a bounded slice for
  `rigid_stack_stability`: add replay-guided timeline metadata for a
  `Top x divergence` value track and diagnostic markers for overlap/collapse,
  low clearance, or visible top-block drift.

## Verified In The Stack Replay Timeline Continuation

The current continuation makes the `rigid_stack_stability` row easier to debug
from the shared Replay panel:

- `python/examples/demos/scenes/rigid_stack_stability.py` adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata.
- `python/tests/integration/test_demos_cycle.py` covers the `Top x divergence`
  timeline label, signal, and marker thresholds.
- `CHANGELOG.md`, `python/examples/demos/README.md`, and
  `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`
  describe the stack timeline value track and markers.

Focused validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_stack_stability_keeps_ipc_stack_ordered python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata -q
pixi run py-demo-capture -- --scene rigid_stack_stability --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_stack_stability_timeline_1781271696
jq -r '.scene, .capture.converted_frames, .visual_evidence.screenshot.docked_workspace, .visual_evidence.screenshot.unique_rgb_count, .scene_metrics.event_count, .scene_metrics.latest.metrics.row, .scene_metrics.latest.metrics.top_x_divergence, .scene_metrics.latest.metrics.history.max_top_x_divergence, .scene_metrics.latest.metrics.history.ipc_min_clearance' /tmp/dart_capture_stack_stability_timeline_1781271696/manifest.json
```

Observed results:

- Focused pytest reported `2 passed`.
- The real docked capture completed with exit code 0 and wrote a nonblank
  960x540 screenshot with docked UI detected, 23 PNG frames, and
  24 scene-metric events.
- The capture manifest reported row `rigid_stack_stability`, current and
  historical maximum top-x divergence about `0.00405`, and IPC minimum
  clearance `0.0`.

## Verified In The Contact Manipulation Replay Timeline Continuation

The current continuation makes the `rigid_contact_manipulation` row easier to
debug from the shared Replay panel:

- `python/examples/demos/scenes/rigid_contact_manipulation.py` adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata.
- `python/tests/integration/test_demos_cycle.py` covers the
  `Travel divergence` timeline label, signal, contact/proximity/progress
  markers, and quiet-frame behavior.
- `CHANGELOG.md`, `python/examples/demos/README.md`, and
  `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`
  describe the contact-manipulation timeline value track and markers.

Focused validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_contact_manipulation_pushes_target_toward_goal python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata -q
pixi run py-demo-capture -- --scene rigid_contact_manipulation --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_contact_manipulation_timeline_1781272246
jq -r '.scene, .capture.converted_frames, .visual_evidence.screenshot.docked_workspace, .visual_evidence.screenshot.unique_rgb_count, .scene_metrics.event_count, .scene_metrics.latest.metrics.row, .scene_metrics.latest.metrics.travel_divergence, .scene_metrics.latest.metrics.history.max_travel_divergence, .scene_metrics.latest.metrics.history.ipc_max_target_travel, .scene_metrics.latest.metrics.history.sequential_impulse_min_gap' /tmp/dart_capture_contact_manipulation_timeline_1781272246/manifest.json
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
```

Observed results:

- Focused Replay/contact pytest reported `2 passed`.
- The real docked capture completed with exit code 0 and wrote a nonblank
  960x540 screenshot with docked UI detected, 71 PNG frames, and
  72 scene-metric events.
- The capture manifest reported row `rigid_contact_manipulation`, current and
  historical maximum travel divergence about `0.0217`, IPC target travel about
  `0.2243` m, and sequential-impulse minimum gap about `-0.0017` m.
- The sidecar/README/capture-command drift guard reported `4 passed`.

## Verified In The Kinematic Driver Replay Timeline Continuation

The current continuation makes the `rigid_kinematic_driver` row easier to debug
from the shared Replay panel:

- `python/examples/demos/scenes/rigid_kinematic_driver.py` adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata.
- `python/tests/integration/test_demos_cycle.py` covers the
  `IPC grip box travel` timeline label, signal, contact/carry/slip/caveat
  markers, the driver-history fallback for caveat frames, and quiet-frame
  behavior.
- `CHANGELOG.md`, `python/examples/demos/README.md`, and
  `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`
  describe the kinematic-driver timeline value track and markers.

Focused validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_kinematic_driver_carries_box_with_ipc python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata -q
pixi run py-demo-capture -- --scene rigid_kinematic_driver --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_kinematic_driver_timeline_1781273002
jq -r '.scene, .capture.converted_frames, .visual_evidence.screenshot.docked_workspace, .visual_evidence.screenshot.unique_rgb_count, .scene_metrics.event_count, .scene_metrics.latest.metrics.row, .scene_metrics.latest.metrics.ipc_grip_box_travel, .scene_metrics.latest.metrics.history.ipc_grip_max_box_travel, .scene_metrics.latest.metrics.history.ipc_slip_max_slip, .scene_metrics.latest.metrics.history.si_caveat_max_abs_driver_travel' /tmp/dart_capture_kinematic_driver_timeline_1781273002/manifest.json
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run lint
git diff --check
```

Observed results:

- Focused Replay/kinematic pytest reported `2 passed`.
- The real docked capture completed with exit code 0 and wrote a nonblank
  960x540 screenshot with docked UI detected, 71 PNG frames, and
  72 scene-metric events.
- The capture manifest reported row `rigid_kinematic_driver`, current and
  historical IPC grip box travel about `0.0896` m, IPC slip maximum about
  `0.1008` m, and sequential-impulse caveat driver travel `0.0`.
- The sidecar/README/capture-command drift guard reported `4 passed`.
- `pixi run lint` passed.
- `git diff --check` was clean.

## Verified In The Kinematic Normal Push Replay Timeline Continuation

The current continuation makes the `rigid_kinematic_normal_push` row easier to
debug from the shared Replay panel:

- `python/examples/demos/scenes/rigid_kinematic_normal_push.py` adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata.
- `python/tests/integration/test_demos_cycle.py` covers the
  `Target travel divergence` timeline label, signal,
  contact/penetration/push/divergence markers, and quiet-frame behavior.
- `CHANGELOG.md`, `python/examples/demos/README.md`, and
  `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`
  describe the normal-push timeline value track and markers.

Focused validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_kinematic_normal_push_exposes_normal_pusher_caveat python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata -q
pixi run py-demo-capture -- --scene rigid_kinematic_normal_push --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_kinematic_normal_push_timeline_1781273413
jq -r '.scene, .capture.converted_frames, .visual_evidence.screenshot.docked_workspace, .visual_evidence.screenshot.unique_rgb_count, .scene_metrics.event_count, .scene_metrics.latest.metrics.row, .scene_metrics.latest.metrics.lanes.ipc_normal.status, .scene_metrics.latest.metrics.lanes.ipc_normal.max_depth, .scene_metrics.latest.metrics.lanes.ipc_normal.target_travel, .scene_metrics.latest.metrics.lanes.si_caveat.status, .scene_metrics.latest.metrics.lanes.si_caveat.target_travel, .scene_metrics.latest.metrics.lanes.si_caveat.analytic_gap' /tmp/dart_capture_kinematic_normal_push_timeline_1781273413/manifest.json
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
```

Observed results:

- Focused Replay/normal-push pytest reported `2 passed`.
- The real docked capture completed with exit code 0 and wrote a nonblank
  960x540 screenshot with docked UI detected, 71 PNG frames, and
  72 scene-metric events.
- The capture manifest reported row `rigid_kinematic_normal_push`, IPC normal
  status `ipc penetration caveat`, IPC normal max depth about `0.125` m,
  near-zero IPC target travel, SI status `pushed`, SI target travel about
  `0.123` m, and SI analytic gap about `-0.00055` m.
- The sidecar/README/capture-command drift guard reported `4 passed`.

## Verified In The Fixed-Joint Replay Timeline Continuation

The current continuation makes the `rigid_fixed_joint` row easier to debug from
the shared Replay panel:

- `python/examples/demos/scenes/rigid_fixed_joint.py` adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata.
- `python/tests/integration/test_demos_cycle.py` covers the
  `Fixed-joint offset error` timeline label, signal, pose-error markers,
  residual-motion markers, and quiet-frame behavior.
- `CHANGELOG.md`, `python/examples/demos/README.md`, and
  `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`
  describe the fixed-joint timeline value track and markers.

Focused validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_fixed_joint_verifier_restores_captured_transform python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata -q
pixi run py-demo-capture -- --scene rigid_fixed_joint --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_fixed_joint_timeline_1781274052
jq -r '.scene, .capture.converted_frames, .visual_evidence.screenshot.docked_workspace, .visual_evidence.screenshot.unique_rgb_count, .scene_metrics.event_count, .scene_metrics.latest.metrics.row, .scene_metrics.latest.metrics.metrics.translation_error, .scene_metrics.latest.metrics.metrics.orientation_error, .scene_metrics.latest.metrics.metrics.payload_speed, .scene_metrics.latest.metrics.history.max_translation_error, .scene_metrics.latest.metrics.history.max_orientation_error' /tmp/dart_capture_fixed_joint_timeline_1781274052/manifest.json
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
```

Observed results:

- Focused Replay/fixed-joint pytest reported `2 passed`.
- The real docked capture completed with exit code 0 and wrote a nonblank
  960x540 screenshot with docked UI detected, 23 PNG frames, and
  24 scene-metric events.
- The capture manifest reported row `rigid_fixed_joint`, final translation
  error about `2.17e-10` m, final orientation error `0.0` rad, final payload
  speed `0.0` m/s, historical maximum translation error about `1.09e-8` m, and
  historical maximum orientation error about `2.67e-7` rad.
- The sidecar/README/capture-command drift guard reported `4 passed`.
- `pixi run lint` passed.
- `git diff --check` was clean.

## Verified In The Joint-Breakage Replay Timeline Continuation

The current continuation makes the `rigid_joint_breakage` row easier to debug
from the shared Replay panel:

- `python/examples/demos/scenes/avbd_rigid_breakable_joint.py` adds
  `release_history`, `replay_timeline_signal(...)`,
  `replay_timeline_marker(...)`, `replay_capture_state`,
  `replay_restore_state`, and `info["replay_timeline"]` metadata for the
  shared AVBD breakable fixed-joint builder.
- `python/tests/integration/test_demos_cycle.py` covers the
  `Payload release distance` timeline label, signal, broken/released markers,
  quiet-frame behavior, and reset behavior.
- `CHANGELOG.md`, `python/examples/demos/README.md`, and
  `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`
  describe the breakage timeline value track and markers.

Focused validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_joint_breakage_marks_and_resets_breakage python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata -q
pixi run py-demo-capture -- --scene rigid_joint_breakage --frames 48 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_joint_breakage_timeline_1781274802
jq -r '.scene, .capture.converted_frames, .visual_evidence.screenshot.docked_workspace, .visual_evidence.screenshot.unique_rgb_count, .scene_metrics.event_count, .scene_metrics.latest.metrics.row, .scene_metrics.latest.metrics.metrics.status, .scene_metrics.latest.metrics.metrics.broken, .scene_metrics.latest.metrics.metrics.payload_release_distance, .scene_metrics.latest.metrics.metrics.payload_speed, .scene_metrics.latest.metrics.history.saw_broken, .scene_metrics.latest.metrics.history.max_payload_release_distance' /tmp/dart_capture_joint_breakage_timeline_1781274802/manifest.json
```

Observed results so far:

- Focused Replay/joint-breakage pytest reported `2 passed`.
- The real docked capture completed with exit code 0 and wrote a nonblank
  960x540 screenshot with docked UI detected, 47 PNG frames, and
  48 scene-metric events.
- The capture manifest reported row `rigid_joint_breakage`, status `broken`,
  broken flag `1.0`, final payload release distance about `0.413` m, final
  payload speed about `1.332` m/s, `saw_broken` `1.0`, and historical maximum
  payload release distance about `0.590` m.
- The sidecar/README/capture-command drift guard reported `4 passed`.
- `pixi run lint` passed.
- `git diff --check` was clean.

## Verified In The Distance-Spring Replay Timeline Continuation

The current continuation makes the `rigid_distance_spring` row easier to debug
from the shared Replay panel:

- `python/examples/demos/scenes/rigid_distance_spring.py` adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata.
- `python/tests/integration/test_demos_cycle.py` covers the
  `Max spring stretch` timeline label, signal, high-stretch markers,
  off-center-spin markers, and quiet-frame behavior.
- `CHANGELOG.md`, `python/examples/demos/README.md`, and
  `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`
  describe the distance-spring timeline value track and markers.

Focused validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_distance_spring_reduces_stretch_and_spins_offset_anchor python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata -q
pixi run py-demo-capture -- --scene rigid_distance_spring --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_distance_spring_timeline_1781275675
jq -r '.scene, .capture.converted_frames, .visual_evidence.screenshot.docked_workspace, .visual_evidence.screenshot.unique_rgb_count, .scene_metrics.event_count, .scene_metrics.latest.metrics.row, .scene_metrics.latest.metrics.lanes.free.stretch, .scene_metrics.latest.metrics.lanes.soft.stretch, .scene_metrics.latest.metrics.lanes.stiff.stretch, .scene_metrics.latest.metrics.lanes.offset.stretch, .scene_metrics.latest.metrics.lanes.offset.angular_speed, .scene_metrics.latest.metrics.history.max_sprung_abs_stretch, .scene_metrics.latest.metrics.history.max_offset_angular_speed' /tmp/dart_capture_distance_spring_timeline_1781275675/manifest.json
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run lint
git diff --check
```

Observed results so far:

- Focused Replay/distance-spring pytest reported `2 passed`.
- The real docked capture completed with exit code 0 and wrote a nonblank
  960x540 screenshot with docked UI detected, 71 PNG frames, and
  72 scene-metric events.
- The capture manifest reported row `rigid_distance_spring`, final unsprung
  stretch `0.330` m, soft/stiff stretch about `-0.113` m and `-0.125` m,
  offset stretch about `-0.001` m, offset angular speed about `1.956` rad/s,
  historical maximum spring stretch about `0.346` m, and historical maximum
  offset angular speed about `21.04` rad/s.
- The sidecar/README/capture-command drift guard reported `4 passed`.
- `pixi run lint` passed.
- `git diff --check` was clean.

## Verified In The Limited-Joints Replay Timeline Continuation

The current continuation makes the `rigid_limited_joints` row easier to debug
from the shared Replay panel:

- `python/examples/demos/scenes/rigid_limited_joints.py` adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata.
- `python/tests/integration/test_demos_cycle.py` covers the
  `Locked-axis error` timeline label, signal, locked-error markers,
  free-axis-motion markers, and quiet-frame behavior.
- `CHANGELOG.md`, `python/examples/demos/README.md`, and
  `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`
  describe the limited-joints timeline value track and markers.

Focused validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_one_dof_joint_verifier_preserves_locked_directions python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata -q
pixi run py-demo-capture -- --scene rigid_limited_joints --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_limited_joints_timeline_1781276042
jq -r '.scene, .capture.converted_frames, .visual_evidence.screenshot.docked_workspace, .visual_evidence.screenshot.unique_rgb_count, .scene_metrics.event_count, .scene_metrics.latest.metrics.row, .scene_metrics.latest.metrics.metrics.hinge_radius_error, .scene_metrics.latest.metrics.metrics.hinge_z_error, .scene_metrics.latest.metrics.metrics.hinge_yaw, .scene_metrics.latest.metrics.metrics.slider_orthogonal_error, .scene_metrics.latest.metrics.metrics.slider_axis_travel, .scene_metrics.latest.metrics.history.max_hinge_radius_error, .scene_metrics.latest.metrics.history.max_hinge_z_error, .scene_metrics.latest.metrics.history.max_slider_orthogonal_error, .scene_metrics.latest.metrics.history.max_abs_hinge_yaw, .scene_metrics.latest.metrics.history.max_slider_axis_travel' /tmp/dart_capture_limited_joints_timeline_1781276042/manifest.json
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run lint
git diff --check
```

Observed results so far:

- Focused Replay/limited-joints pytest reported `2 passed`.
- The real docked capture completed with exit code 0 and wrote a nonblank
  960x540 screenshot with docked UI detected, 23 PNG frames, and
  24 scene-metric events.
- The capture manifest reported row `rigid_limited_joints`, hinge radius error
  `0.0`, hinge z error `0.0`, hinge yaw about `0.168` rad, slider orthogonal
  error `0.0`, slider axis travel about `0.604` m, zero historical locked-axis
  error, historical maximum hinge yaw about `0.168` rad, and historical maximum
  slider axis travel about `0.604` m.
- The sidecar/README/capture-command drift guard reported `4 passed`.
- `pixi run lint` passed.
- `git diff --check` was clean.

## Verified In The Joint Motor Limits Replay Timeline Continuation

Current motor-limits Replay timeline validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_joint_motor_limits_clamp_commands_and_effort python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run py-demo-capture -- --scene rigid_joint_motor_limits --frames 96 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_joint_motor_limits_timeline_1781276821
jq -r '.scene, .capture.requested_frames, .capture.converted_frames, .show_ui, .visual_evidence.screenshot.docked_workspace, .visual_evidence.screenshot.unique_rgb_count, .scene_metrics.event_count, .scene_metrics.latest.metrics.row, .scene_metrics.latest.metrics.motor_speed, .scene_metrics.latest.metrics.motor_expected_speed, .scene_metrics.latest.metrics.motor_speed_error, .scene_metrics.latest.metrics.position_limit_angle, .scene_metrics.latest.metrics.position_limit_upper, .scene_metrics.latest.metrics.position_limit_error, .scene_metrics.latest.metrics.force_position_gap, .scene_metrics.latest.metrics.force_acceleration_gap, .scene_metrics.latest.metrics.history.max_force_position_gap, .scene_metrics.latest.metrics.history.max_force_acceleration_gap' /tmp/dart_capture_joint_motor_limits_timeline_1781276821/manifest.json
pixi run lint
git diff --check
```

- Focused Replay/motor-limits pytest and drift guards reported `6 passed`.
- The real docked capture completed with exit code 0 and wrote a nonblank
  960x540 screenshot with docked UI detected, 95 PNG frames, and
  96 scene-metric events.
- The maintained workflow capture budget was raised from 72 to 96 frames
  because the 72-frame packet showed velocity clamp and effort gap, but did not
  reach the position stop. The 96-frame packet reached the stop with
  `position_limit_angle` equal to `position_limit_upper` (`0.35` rad).
- The capture manifest reported row `rigid_joint_motor_limits`, motor speed
  and expected speed `0.3` m/s, motor speed error `0.0`, position-limit error
  `0.0`, force travel gap about `0.6984` m, and force acceleration gap about
  `6.0` m/s^2.
- `pixi run lint` passed.
- `git diff --check` was clean.

## Verified In The Passive Parameters Replay Timeline Continuation

The current continuation makes the `rigid_joint_passive_parameters` row easier
to debug from the shared Replay panel:

- `python/examples/demos/scenes/rigid_joint_passive_parameters.py` adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata.
- The shared Replay panel value track label is `Armature position gap`.
- Timeline markers cover damping energy separation, Coulomb slip, and
  armature-lag frames.
- `python/tests/integration/test_demos_cycle.py` covers the label, signal,
  energy-separation marker, stiction/slip marker, armature-lag marker, and
  quiet-frame behavior.
- `CHANGELOG.md`, `python/examples/demos/README.md`, and
  `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`
  document the new Replay timeline value track and markers.

Focused validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_joint_passive_parameters_order_passive_response -q
pixi run py-demo-capture -- --scene rigid_joint_passive_parameters --frames 120 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_joint_passive_parameters_timeline_1781277900
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_joint_passive_parameters_order_passive_response python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run lint
git diff --check
```

Observed results:

- The focused passive-parameters test reported `1 passed`.
- The focused Replay/passive-parameters pytest plus drift guards reported
  `6 passed`.
- The real docked capture completed with exit code 0 and wrote a nonblank
  960x540 screenshot with docked UI detected, 119 PNG frames, and
  120 scene-metric events.
- The capture manifest reported row `rigid_joint_passive_parameters`, spring
  energy about `2.1583`, damped energy about `1.3403`, damped-energy ratio
  about `0.6210`, stiction position `0.0`, slip position about `0.1742` m,
  slip speed about `0.7200` m/s, armature acceleration gap about `2.25` m/s^2,
  and armature position gap about `0.2614` m.
- `pixi run lint` passed.
- `git diff --check` was clean.

## Verified In The Screw-Joint Pitch Replay Timeline Continuation

The current continuation makes the `rigid_screw_joint_pitch` row easier to
debug from the shared Replay panel:

- `python/examples/demos/scenes/rigid_screw_joint_pitch.py` adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata.
- The shared Replay panel value track label is `Coarse/fine travel gap`.
- Timeline markers cover pitch-spread, zero-pitch contrast, and reverse-sign
  frames.
- `python/tests/integration/test_demos_cycle.py` covers the label, signal,
  coarse/fine travel-gap marker, zero/fine travel-contrast marker, reverse-sign
  marker, and quiet-frame behavior.
- `CHANGELOG.md`, `python/examples/demos/README.md`, and
  `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`
  document the new Replay timeline value track and markers.

Focused validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_screw_joint_pitch_couples_rotation_and_translation python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run py-demo-capture -- --scene rigid_screw_joint_pitch --frames 96 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_screw_joint_pitch_timeline_1781278553
pixi run lint
git diff --check
```

Observed results:

- The focused Replay/screw-joint pytest plus drift guards reported `6 passed`.
- The real docked capture completed with exit code 0 and wrote a nonblank
  960x540 screenshot with docked UI detected, 95 PNG frames, and
  96 scene-metric events.
- The capture manifest reported row `rigid_screw_joint_pitch`, fine angle
  about `-0.8317` rad, coarse angle about `-0.6162` rad, reverse angle about
  `0.8317` rad, fine axial travel about `-0.2329` m, coarse axial travel about
  `-0.3451` m, reverse axial travel about `-0.2329` m, coarse/fine travel gap
  about `0.1122` m, and travel-per-radian values `0.28`, `0.56`, and `-0.28`.
- `pixi run lint` passed.
- `git diff --check` was clean.

## Verified In The Multibody Dynamics Terms Replay Timeline Continuation

The current continuation makes the `rigid_multibody_dynamics_terms` row easier
to debug from the shared Replay panel:

- `python/examples/demos/scenes/rigid_multibody_dynamics_terms.py` adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata.
- The shared Replay panel value track label is `Response norm gap`.
- Timeline markers cover response separation, off-diagonal coupling, and
  heavy-load torque frames.
- `python/tests/integration/test_demos_cycle.py` covers the label, signal,
  response-gap marker, coupling marker, heavy-load torque marker, and
  quiet-frame behavior.
- `CHANGELOG.md`, `python/examples/demos/README.md`, and
  `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`
  document the new Replay timeline value track and markers.

Focused validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_multibody_dynamics_terms_expose_generalized_terms python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run py-demo-capture -- --scene rigid_multibody_dynamics_terms --frames 96 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_multibody_dynamics_terms_timeline_1781279136
```

Observed results:

- The focused Replay/multibody-dynamics pytest plus drift guards reported
  `6 passed`.
- The real docked capture completed with exit code 0 and wrote a nonblank
  960x540 screenshot with docked UI detected, 95 PNG frames, and
  96 scene-metric events.
- The capture manifest reported row `rigid_multibody_dynamics_terms`, coupled
  response norm about `15.46`, heavy response norm about `8.63`,
  heavy/coupled response ratio about `0.558`, heavy-minus-coupled torque norm
  about `18.14`, coupled/heavy off-diagonal coupling about `0.357`/`1.427`,
  max impulse residual about `2.7e-14`, and historical max coupling about
  `0.373`.

## Verified In The Link Center Of Mass Replay Timeline Continuation

The current continuation makes the `rigid_link_center_of_mass` row easier to
debug from the shared Replay panel:

- `python/examples/demos/scenes/rigid_link_center_of_mass.py` adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata.
- The shared Replay panel value track label is `Mirrored COM angle spread`.
- Timeline markers cover mirrored-angle divergence, centered-lane stillness,
  and high-inertia lag frames.
- `python/tests/integration/test_demos_cycle.py` covers the label, signal,
  mirrored-angle marker, centered-still marker, high-inertia-lag marker, and
  quiet-frame behavior.
- `CHANGELOG.md`, `python/examples/demos/README.md`, and
  `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`
  document the new Replay timeline value track and markers.

Focused validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_link_center_of_mass_offsets_gravity_torque python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run py-demo-capture -- --scene rigid_link_center_of_mass --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_link_center_of_mass_timeline_1781279455
```

Observed results:

- The focused Replay/link-center-of-mass pytest plus drift guards reported
  `6 passed`.
- The real docked capture completed with exit code 0 and wrote a nonblank
  960x540 screenshot with docked UI detected, 71 PNG frames, and
  72 scene-metric events.
- The capture manifest reported row `rigid_link_center_of_mass`, mirrored
  positive/negative angles about `+/-0.449`, high-inertia angle about `0.153`,
  centered angle `0.0`, mirrored torques about `+/-3.182`, high/positive
  acceleration ratio about `0.370`, high/positive mass-matrix ratio about
  `2.948`, and historical max positive/high-inertia angles about
  `0.449`/`0.153`.
- The link-Jacobian Replay timeline slice then passed the focused
  Replay/link-Jacobian pytest plus sidecar/README/capture-command drift guards:
  `6 passed`.
- Its real docked capture wrote a nonblank 960x540 screenshot with docked UI,
  95 PNG frames, and 96 scene-metric events under
  `/tmp/dart_capture_link_jacobian_timeline_1781280127`.
- The capture manifest reported row `rigid_link_jacobian`, latest
  linear/angular speed about `0.5652`/`0.6190`, finite-difference error about
  `1.52e-7`, power error `0`, world/body Jacobian gap about `0.1272`, torques
  about `-0.8650`/`-0.2949`, matching joint/wrench power about `-0.4212`,
  historical max link speed about `0.6677`, historical max world/body gap about
  `0.2055`, and historical max absolute torques about `0.8650`/`0.2949`.

## Verified In The Multibody Solver Family Replay Timeline Continuation

The current continuation makes the `rigid_multibody_solver_family` row easier
to debug from the shared Replay panel:

- `python/examples/demos/scenes/rigid_multibody_solver_family.py` adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata.
- The shared Replay panel value track label is `Residual solve ratio`.
- Timeline markers cover large solve-ratio separation, residual-only drift,
  and solved-tight frames while residual-only rows remain loose.
- `python/tests/integration/test_demos_cycle.py` covers the label, signal,
  top-level fallback, `last_metrics` fallback, solve-ratio marker,
  residual-drift marker, tip-error marker, solved-tight marker, and quiet-frame
  behavior.
- `CHANGELOG.md`, `python/examples/demos/README.md`, and
  `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`
  document the new Replay timeline value track and markers.

Focused validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_multibody_solver_family_routes_solved_closures -q
pixi run py-demo-capture -- --scene rigid_multibody_solver_family --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_multibody_solver_family_timeline_1781281303
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_multibody_solver_family_routes_solved_closures python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
```

Observed results:

- The focused row 35 pytest reported `1 passed`.
- The focused Replay/row 35 pytest plus sidecar/README/capture-command drift
  guards reported `6 passed`.
- The real docked capture completed with exit code 0 and wrote a nonblank
  960x540 screenshot with docked UI detected, 71 PNG frames, and
  72 scene-metric events.
- The capture manifest reported row `rigid_multibody_solver_family`, solver
  `world_multibody_integration_family`, scope
  `multibody_closure_solve_routing`, executor `Sequential`, gravity scale
  `1.0`, three cases, residual-only residual about `0.7642`, solved residual
  clamped at `1e-12`, residual solve ratio about `7.642e11`, variational solved
  residual/tip error about `6.66e-16`, and historical max residual solve ratio
  about `7.642e11`.

## Verified In The Loop Closure Replay Timeline Continuation

The current continuation makes the `rigid_loop_closure` row easier to debug
from the shared Replay panel:

- `python/examples/demos/scenes/rigid_loop_closure.py` has
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata for the `Max closure residual ratio`
  value track.
- `python/tests/integration/test_demos_cycle.py` has assertions in
  `test_rigid_loop_closure_compares_closure_families` for the row 36 timeline
  label, signal fallbacks, marker cases, and quiet-frame behavior.
- `CHANGELOG.md`, `python/examples/demos/README.md`, and
  `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`
  document the new Replay timeline value track and markers.

Focused validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_loop_closure_compares_closure_families -q
pixi run py-demo-capture -- --scene rigid_loop_closure --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_loop_closure_timeline_1781285000
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_loop_closure_compares_closure_families python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run lint
git diff --check
```

Observed results: the focused row 36 pytest reported `1 passed`; the focused
Replay/row 36 pytest plus sidecar/README/capture-command drift guards reported
`6 passed`; the real docked capture wrote a nonblank 960x540 docked screenshot,
71 PNG frames, and 72 scene-metric events. The manifest recorded row
`rigid_loop_closure`, solver `variational_rigid_multibody_loop_closure`, scope
`point_distance_rigid_closure_family_selection`, executor `Sequential`, gravity
scale `1.0`, six cases, point/distance/rigid residual ratios about
`7.595e11`, `7.458e11`, and `7.740e11`, rigid residual orientation error about
`0.149`, solved orientation error near `2.8e-17`, distance solved distance
error near `9.4e-15`, and distance solved tip error about `0.439`.
`pixi run lint` passed and `git diff --check` was clean.

## Verified In The Replay Metadata Capture Continuation

This continuation makes the completed Replay timeline pass visible in
reviewer-facing capture packets:

- `python/examples/demos/runner.py` emits a `scene_capture_metadata` JSONL event
  through the existing capture metrics log after shared Replay controls attach.
- `scripts/capture_py_demo.py` stores that event as
  `scene_metadata.replay_timeline` in each per-scene manifest and displays the
  Replay track label plus signal/marker availability in workflow review cards.
- Tests now guard the JSON-safe projection, per-scene manifest field,
  workflow review-card rendering, and every sidecar row that claims
  `Replay timeline coverage`.

Focused validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_capture_metadata_projects_replay_timeline_for_manifests python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/unit/test_capture_py_demo.py::test_visual_capture_manifest_records_image_evidence python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_links_scene_videos python/tests/integration/test_demos_cycle.py::test_rigid_visual_replay_timeline_rows_publish_scene_metadata python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run py-demo-capture -- --scene rigid_loop_closure --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_loop_closure_replay_metadata_1781289000
jq -r '.scene, .scene_metadata.replay_timeline.signal_label, .scene_metadata.replay_timeline.has_signal, .scene_metadata.replay_timeline.has_markers, .scene_metadata.replay_timeline.panel, .scene_metrics.event_count, .visual_evidence.screenshot.docked_workspace' /tmp/dart_capture_loop_closure_replay_metadata_1781289000/manifest.json
pixi run py-demo-capture -- --rigid-workflow --workflow-start-row 36 --workflow-end-row 36 --output-dir /tmp/dart_capture_rigid_workflow_replay_metadata_row36_1781289001
jq -r '.status, .capture_count, .completed_count, .failed_count, .captures[0].scene, .captures[0].manifest' /tmp/dart_capture_rigid_workflow_replay_metadata_row36_1781289001/manifest.json
jq -r '.scene_metadata.replay_timeline.signal_label, .scene_metadata.replay_timeline.has_signal, .scene_metadata.replay_timeline.has_markers' /tmp/dart_capture_rigid_workflow_replay_metadata_row36_1781289001/scenes/36_rigid_loop_closure/manifest.json
rg -n "replay</dt>|Max closure residual ratio|signal, markers|36/36 rigid_loop_closure" /tmp/dart_capture_rigid_workflow_replay_metadata_row36_1781289001/review_index.html
pixi run lint
git diff --check
```

Observed results: the first focused pytest reported `4 passed`; after lint,
the focused metadata/replay/drift suite reported `10 passed`. The real
single-scene capture reported `rigid_loop_closure`,
`Max closure residual ratio`, `true`, `true`, `Replay`, `72`, and docked UI
`true`. The one-row workflow capture reported `status=complete`,
`capture_count=1`, `completed_count=1`, `failed_count=0`, scene
`rigid_loop_closure`, and the row-36 per-scene manifest path. That per-scene
manifest reported `Max closure residual ratio`, `true`, and `true`; the
generated `review_index.html` contained `36/36 rigid_loop_closure` and
`<dt>replay</dt><dd>Max closure residual ratio (signal, markers)</dd>`.
`pixi run lint` passed and `git diff --check` was clean.

## Verified In The Full Replay Metadata Workflow Capture Refresh

This continuation refreshed the complete numbered workflow packet after
`scene_metadata.replay_timeline` landed, and tightened the sidecar drift guard
so every numbered row that publishes Replay metadata must document
`Replay timeline coverage`.

Commands:

```bash
pixi run py-demo-capture -- --rigid-workflow --output-dir /tmp/dart_capture_rigid_workflow_replay_metadata_full_1781284053
jq -r '.status, .capture_count, .completed_count, .failed_count, .guidance_complete, .guidance_missing_count, .elapsed_s, .artifacts.review_index' /tmp/dart_capture_rigid_workflow_replay_metadata_full_1781284053/manifest.json
rg -c 'Replay timeline coverage' docs/plans/103-examples-strategy/rigid-body-visual-verification.md
find /tmp/dart_capture_rigid_workflow_replay_metadata_full_1781284053/scenes -name manifest.json -print0 | xargs -0 jq -r 'select(.scene_metadata.replay_timeline != null) | .scene' | wc -l
rg -o '<dt>replay</dt>' /tmp/dart_capture_rigid_workflow_replay_metadata_full_1781284053/review_index.html | wc -l
find /tmp/dart_capture_rigid_workflow_replay_metadata_full_1781284053/scenes -name manifest.json -print0 | xargs -0 jq -r 'select((.capture.converted_frames // 0) <= 0 or .visual_evidence.screenshot.docked_workspace != true or (.visual_evidence.screenshot.unique_rgb_count // 0) <= 1) | [.scene, (.capture.converted_frames|tostring), (.visual_evidence.screenshot.docked_workspace|tostring), (.visual_evidence.screenshot.unique_rgb_count|tostring)] | @tsv'
```

Observed results: the workflow manifest reported `complete`, `36`, `36`, `0`,
`true`, `0`, elapsed `310.791`, and review index
`/tmp/dart_capture_rigid_workflow_replay_metadata_full_1781284053/review_index.html`.
The sidecar, per-scene manifests, and review index each reported nineteen
Replay rows. The per-scene manifest anomaly query printed no rows. The review
index included representative Replay cards such as row 21
`Top x divergence (signal, markers)` and row 36
`Max closure residual ratio (signal, markers)`.

## Verified In The Optional Extended Packet Capture Refresh

This continuation captured the optional rows that sit outside the default
36-row workflow but are exposed by the generated review packet: related
evidence, direct Rigid IPC shelf rows, and the capture-first IPC stack packet.

Commands:

```bash
DART_PARALLEL_JOBS=4 CTEST_PARALLEL_LEVEL=4 CMAKE_BUILD_PARALLEL_LEVEL=4 pixi run py-demo-capture -- --rigid-workflow --include-related --include-ipc-shelf --include-packets --workflow-start-row 37 --workflow-end-row 51 --output-dir /tmp/dart_capture_rigid_workflow_optional_rows_37_51_1781285053
jq -r '.status, .capture_count, .completed_count, .failed_count, .workflow_total_count, .workflow_row_start, .workflow_row_end, .include_related, .include_ipc_shelf, .include_packets, .selected_include_related, .selected_include_ipc_shelf, .selected_include_packets, .guidance_complete, .guidance_missing_count, .elapsed_s, .artifacts.review_index' /tmp/dart_capture_rigid_workflow_optional_rows_37_51_1781285053/manifest.json
find /tmp/dart_capture_rigid_workflow_optional_rows_37_51_1781285053/scenes -name manifest.json -print0 | xargs -0 jq -r 'select((.capture.converted_frames // 0) <= 0 or .visual_evidence.screenshot.docked_workspace != true or (.visual_evidence.screenshot.unique_rgb_count // 0) <= 1) | [.scene, (.capture.converted_frames|tostring), (.visual_evidence.screenshot.docked_workspace|tostring), (.visual_evidence.screenshot.unique_rgb_count|tostring)] | @tsv'
jq -r '.captures | group_by(.workflow_group)[] | [.[0].workflow_group, length] | @tsv' /tmp/dart_capture_rigid_workflow_optional_rows_37_51_1781285053/manifest.json
rg -n '<strong>rows</strong> 37-51 / 51|<strong>requested groups</strong>|<strong>selected groups</strong>|37/51 floating_base|46/51 avbd_rigid_prismatic_motor|47/51 rigid_ipc|51/51 rigid_ipc_stack_packet|Related evidence|Rigid IPC shelf|Capture-first packet' /tmp/dart_capture_rigid_workflow_optional_rows_37_51_1781285053/review_index.html
```

Observed results: the workflow manifest reported `complete`, `15`, `15`, `0`,
`workflow_total_count=51`, selected rows `37-51`, all requested/selected
related, IPC-shelf, and packet groups as `true`, `guidance_complete=true`,
`guidance_missing_count=0`, elapsed `149.609`, and review index
`/tmp/dart_capture_rigid_workflow_optional_rows_37_51_1781285053/review_index.html`.
The per-scene manifest anomaly query printed no rows. The group-count query
reported ten `related_evidence` rows, four `rigid_ipc_shelf` rows, and one
`capture_first_packet` row. The review index contained the row-span and group
badges plus representative cards for `37/51 floating_base`,
`46/51 avbd_rigid_prismatic_motor`, `47/51 rigid_ipc`, and
`51/51 rigid_ipc_stack_packet`.

## Verified In The Heavy Rigid IPC Stack Packet Slice

This continuation finished the second capture-first Rigid IPC stack packet.
The new `rigid_ipc_heavy_stack_packet` row is a six-box, top-heavy IPC barrier
stress packet; it stays outside the numbered workflow and appears only in the
optional packet group after the existing four-box stack packet.

Commands:

```bash
python -m py_compile python/examples/demos/scenes/rigid_ipc_stack_packet.py scripts/capture_py_demo.py python/tests/integration/test_demos_cycle.py python/tests/unit/test_py_demo_panels.py python/tests/unit/test_capture_py_demo.py
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_registry_has_scenes python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_capture_first_ipc_packets_are_documented python/tests/integration/test_demos_cycle.py::test_rigid_visual_capture_first_packets_are_documented python/tests/integration/test_demos_cycle.py::test_rigid_ipc_stack_packet_reports_capture_first_metrics python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_ipc_stack_packet_panel_exposes_capture_first_signals python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_capture_first_packets python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_full_extended_plan_has_complete_guidance -q
pixi run py-demo-capture -- --rigid-workflow --include-related --include-ipc-shelf --include-packets --workflow-start-row 51 --workflow-end-row 52 --dry-run --output-dir /tmp/dart_capture_rigid_workflow_heavy_stack_dry_run_1781286611
DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run py-demo-capture -- --rigid-workflow --include-related --include-ipc-shelf --include-packets --workflow-start-row 52 --workflow-end-row 52 --output-dir /tmp/dart_capture_rigid_workflow_heavy_stack_row52_1781286631
```

Observed results: syntax checks passed; focused pytest reported `11 passed`
before and after `pixi run lint`. The dry-run reported `status=planned`,
`capture_count=2`,
`workflow_total_count=52`, selected rows `51-52`,
`guidance_complete=true`, and `guidance_missing_count=0`; its review index
showed `51/52 rigid_ipc_stack_packet` and
`52/52 rigid_ipc_heavy_stack_packet` as capture-first packet rows. The real
row-52 workflow capture reported `status=complete`, `capture_count=1`,
`completed_count=1`, `failed_count=0`, `workflow_total_count=52`, selected row
`52`, `guidance_complete=true`, and `guidance_missing_count=0`. The per-scene
manifest recorded 11 PNG frames, a docked nonblank screenshot with 3539 unique
RGB colors, 12 scene-metric events, row `rigid_ipc_heavy_stack_packet`,
`box_count=6.0`, `top_mass=4.25`, and `status=capture-first`. `pixi run lint`
passed and `git diff --check` was clean.

## Immediate Next Steps

1. Resume from `git status -sb` and `git log -5 --oneline`.
2. Expect the current local checkpoint to be
   `08f710b793b Record regenerated rigid workflow evidence`, with no PR for
   `feature/rigid-body-gui-visual-verification` unless one was created after
   this handoff.
3. Use
   `build/captures/rigid_workflow_rows_01_36_1781309127/review_index.html` and
   `build/captures/rigid_workflow_optional_rows_37_52_1781309448/review_index.html`
   as the current static review artifacts; both were regenerated from current
   HEAD and passed local asset-link audits.
4. Use `docs/dev_tasks/rigid_body_visual_verification/PR_DRAFT.md` as the PR
   body seed once push/PR creation is explicitly approved.
5. Check `docs/dev_tasks/rigid_body_visual_verification/COMPLETION_AUDIT.md`
   before claiming completion; local evidence is review-ready but external
   acceptance is still missing.
6. Do not add new numbered rows unless a public API gap has been resolved and
   the durable PLAN-103 scope is updated first.
7. Retire this dev-task folder only if the maintainer explicitly accepts the
   current scope as complete.
8. Do not push again unless the user explicitly approves pushing in that
   session; before pushing a PR branch, merge latest base rather than rebasing.

## Commit And Push Notes

- Do not commit without following the repository pre-commit rule:
  `pixi run lint` is mandatory before every commit.
- Do not push unless the user explicitly approves pushing in that session.
- Before pushing an existing PR branch, merge the latest base branch into the
  branch first; do not rebase a published PR branch.
