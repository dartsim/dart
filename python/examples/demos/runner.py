"""Scene-registry runner for the DART Python demos.

`pixi run py-demos` (and `python -m examples.demos`) launches the C++
Filament viewer through `dartpy.gui.run_demos` with the Python scene
catalog. The CLI mirrors `dart-demos`:

- `--scene <id>`: select a starting scene
- `--cycle-scenes`: advance through every scene then exit
- `--frames N`: per-scene frame budget (for headless cycles)
- `--screenshot <path>`: write a PPM at <path> (real Filament render)
- `--headless`: render without opening a window
- `--width N`, `--height N`, `--backend ...`: forward to the viewer
- `--list`: print the Python scene catalog and exit (no viewer)
- `--gpu` / `--no-gpu`: force GPU (CUDA) deformable solve on/off; the default
  ("auto") enables it when a CUDA device is available (e.g. under
  `pixi run -e cuda py-demos`). `DART_PY_DEMOS_GPU=on|off|auto` overrides the
  default. The GPU offload covers the deformable projected-Newton PSD
  projection and is a process-wide toggle (also exposed in an in-viewer panel).

Scenes with a Python-side controller (`SceneSetup.pre_step`) have that
callable forwarded to the viewer's per-step hook so the controller still
runs inside the interactive loop. `SceneSetup.step` remains a headless
whole-step escape hatch for legacy parity tests; interactive demos should use
`pre_step` so the viewer owns `world.step()`.
"""

from __future__ import annotations

import argparse
import contextlib
import copy
import json
import math
import os
import re
import signal
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Iterable, Mapping

# Frames-per-scene defaults match the C++ host so cycle behavior is identical.
CYCLE_FRAMES_PER_SCENE = 4
SINGLE_SCENE_DEFAULT_FRAMES = 60
DEFAULT_INITIAL_SCENE_ID = "rigid_body"
SCENE_BUILD_TIMEOUT_ENV = "DART_PY_DEMO_SCENE_BUILD_TIMEOUT_MS"
DEMO_SCENE_STARTUP_TIMEOUT_ENV = "DART_DEMO_SCENE_STARTUP_TIMEOUT_MS"
DEFAULT_SCENE_BUILD_TIMEOUT_MS = 5000.0
DEFAULT_REPLAY_MAX_FRAMES = 900
REPLAY_TIMELINE_INFO_KEY = "replay_timeline"
REPLAY_WORLD_INFO_KEYS = ("sx_world", "physics_world")
CAPTURE_METRICS_INFO_KEY = "capture_metrics"
CAPTURE_METADATA_EVENT_NAME = "scene_capture_metadata"
CAPTURE_METRICS_EVENT_NAME = "scene_capture_metrics"
_REPLAY_RATE_LABELS = (
    "1 frame/tick",
    "2 frames/tick",
    "4 frames/tick",
    "8 frames/tick",
)
_REPLAY_RATE_STEPS = (1, 2, 4, 8)
RIGID_VISUAL_WORKFLOW_CATEGORY = "World Rigid Body"
RIGID_VISUAL_WORKFLOW_LABELS = (
    ("rigid_body", "Baseline"),
    ("rigid_body_modes", "Body modes"),
    ("rigid_free_flight", "Free flight"),
    ("rigid_frame_hierarchy", "Frame hierarchy"),
    ("rigid_external_loads", "Loads"),
    ("rigid_link_point_loads", "Point loads"),
    ("rigid_timestep_sensitivity", "Time step"),
    ("rigid_step_diagnostics", "Step diagnostics"),
    ("rigid_contact_scale_budget", "Contact budget"),
    ("rigid_restitution_ladder", "Restitution"),
    ("rigid_material_mixing", "Material mixing"),
    ("rigid_contact_inspector", "Contact inspector"),
    ("rigid_collision_query_options", "Query filters"),
    ("rigid_collision_casts", "Collision casts"),
    ("rigid_solver_compare", "Solver family"),
    ("rigid_executor_equivalence", "Executor equivalence"),
    ("rigid_contact_solver_compare", "Contact policy"),
    ("contact", "Link contact"),
    ("rigid_friction_threshold", "Friction threshold"),
    ("rigid_spin_roll_coupling", "Spin/roll coupling"),
    ("rigid_stack_stability", "Stack stability"),
    ("rigid_contact_manipulation", "Manipulation"),
    ("rigid_kinematic_driver", "Kinematic driver"),
    ("rigid_kinematic_normal_push", "Normal push"),
    ("rigid_fixed_joint", "Fixed joint"),
    ("rigid_joint_breakage", "Breakage lifecycle"),
    ("rigid_distance_spring", "Distance spring"),
    ("rigid_limited_joints", "One-DOF joints"),
    ("rigid_joint_motor_limits", "Motor limits"),
    ("rigid_joint_passive_parameters", "Passive joints"),
    ("rigid_screw_joint_pitch", "Screw pitch"),
    ("rigid_multibody_dynamics_terms", "Dynamics terms"),
    ("rigid_link_center_of_mass", "COM offset"),
    ("rigid_link_jacobian", "Link Jacobian"),
    ("rigid_multibody_solver_family", "Multibody solver"),
    ("rigid_loop_closure", "Loop closure"),
)

RIGID_VISUAL_WORKFLOW_CAPTURE_SPECS: tuple[
    tuple[str, int, int, int, bool], ...
] = (
    ("rigid_body", 180, 960, 540, True),
    ("rigid_body_modes", 72, 960, 540, True),
    ("rigid_free_flight", 96, 960, 540, True),
    ("rigid_frame_hierarchy", 72, 960, 540, True),
    ("rigid_external_loads", 72, 960, 540, True),
    ("rigid_link_point_loads", 72, 960, 540, True),
    ("rigid_timestep_sensitivity", 96, 960, 540, True),
    ("rigid_step_diagnostics", 72, 960, 540, True),
    ("rigid_contact_scale_budget", 72, 960, 540, True),
    ("rigid_restitution_ladder", 96, 960, 540, True),
    ("rigid_material_mixing", 72, 960, 540, True),
    ("rigid_contact_inspector", 24, 960, 540, True),
    ("rigid_collision_query_options", 24, 960, 540, True),
    ("rigid_collision_casts", 48, 960, 540, True),
    ("rigid_solver_compare", 24, 960, 540, True),
    ("rigid_executor_equivalence", 24, 960, 540, True),
    ("rigid_contact_solver_compare", 72, 960, 540, True),
    ("contact", 144, 960, 540, True),
    ("rigid_friction_threshold", 24, 960, 540, True),
    ("rigid_spin_roll_coupling", 96, 960, 540, True),
    ("rigid_stack_stability", 24, 960, 540, True),
    ("rigid_contact_manipulation", 72, 960, 540, True),
    ("rigid_kinematic_driver", 72, 960, 540, True),
    ("rigid_kinematic_normal_push", 72, 960, 540, True),
    ("rigid_fixed_joint", 24, 960, 540, True),
    ("rigid_joint_breakage", 48, 960, 540, True),
    ("rigid_distance_spring", 72, 960, 540, True),
    ("rigid_limited_joints", 24, 960, 540, True),
    ("rigid_joint_motor_limits", 96, 960, 540, True),
    ("rigid_joint_passive_parameters", 120, 960, 540, True),
    ("rigid_screw_joint_pitch", 96, 960, 540, True),
    ("rigid_multibody_dynamics_terms", 96, 960, 540, True),
    ("rigid_link_center_of_mass", 72, 960, 540, True),
    ("rigid_link_jacobian", 96, 960, 540, True),
    ("rigid_multibody_solver_family", 72, 960, 540, True),
    ("rigid_loop_closure", 72, 960, 540, True),
)

_RIGID_VISUAL_WORKFLOW_CAPTURE_BY_SCENE = {
    scene_id: (frames, width, height, show_ui)
    for scene_id, frames, width, height, show_ui in RIGID_VISUAL_WORKFLOW_CAPTURE_SPECS
}


@dataclass(frozen=True)
class RigidWorkflowGuide:
    scene_id: str
    index: int
    count: int
    label: str
    question: str
    try_first: str
    inspect: tuple[str, ...]
    healthy_signal: str
    scope: str
    previous_scene_id: str | None
    next_scene_id: str | None
    capture_frames: int
    capture_width: int
    capture_height: int
    capture_show_ui: bool
    capture_command: str


@dataclass(frozen=True)
class RigidWorkflowRelatedEvidence:
    label: str
    scene_id: str
    shelf: str
    reason: str


_RIGID_VISUAL_WORKFLOW_GUIDE_TEXT: Mapping[
    str, tuple[str, tuple[str, ...], str]
] = {
    "rigid_body": (
        "What is the baseline DART 7 World rigid-body path?",
        ("Solver/material controls", "Contacts, energy, step timing"),
        "Baseline front door; focused edge cases stay in the specialized verifier rows.",
    ),
    "rigid_body_modes": (
        "Which body mode should I choose?",
        ("Dynamic vs static vs kinematic lanes", "Static drift and path error"),
        "Contact-free mode semantics row; no sleep/wake or island activation API claim.",
    ),
    "rigid_free_flight": (
        "Do initial velocity, gravity, and spin evolve?",
        ("Path error and momentum residual", "Energy drift and spin ratios"),
        "No-contact initial-state row; not a load, restitution, contact, or solver row.",
    ),
    "rigid_frame_hierarchy": (
        "Where is a sensor/tool frame on a moving body?",
        ("Local-to-world transform residual", "Relative transform and parent frame"),
        "Kinematics/frame row only; not a force, contact, sensor model, or solver-family claim.",
    ),
    "rigid_external_loads": (
        "How do external loads and direct impulses move and spin bodies?",
        (
            "Mass-scaled acceleration",
            "Direct impulse momentum and static drift",
        ),
        "Contact-free zero-gravity load and direct rigid-body impulse row; no contact or solver-family claim.",
    ),
    "rigid_link_point_loads": (
        "Do point forces create lever-arm torque?",
        ("Centered vs off-center force lanes", "World/local frame semantics"),
        "Contact-free one-shot Link.apply_force row; not persistent rigid-body accumulator behavior.",
    ),
    "rigid_timestep_sensitivity": (
        "How does time-step size change free fall/contact?",
        ("Fine/medium/coarse error trend", "Contact timing and clearance"),
        "Parameter-sensitivity row; not a solver correctness proof or exact contact threshold.",
    ),
    "rigid_step_diagnostics": (
        "Where does a World step spend time and memory?",
        ("Top profile stage and backend status", "Scratch memory and ECS counters"),
        "Profiling may be compiled out; memory/contact diagnostics remain visible.",
    ),
    "rigid_contact_scale_budget": (
        "How much contact fits in my frame budget?",
        ("One/four/nine-box contact scaling", "Per-contact cost and budget status"),
        "Bounded live-GUI budget row; not a benchmark suite or heavy IPC stress packet.",
    ),
    "rigid_restitution_ladder": (
        "How does restitution change bounce height?",
        ("Low/medium/high rebound trend", "Velocity, contact, and energy trend"),
        "Relative rebound diagnostic only; energy plots are not exact conservation claims.",
    ),
    "rigid_material_mixing": (
        "Which material owns bounce or friction response?",
        ("Swapped body/surface ownership", "Max restitution and sqrt friction"),
        "Pair-rule ownership row only; not an IPC restitution claim or incline stick/slip proof.",
    ),
    "rigid_contact_inspector": (
        "Which contact pairs and manifold fields exist?",
        ("Selected pair, point, normal, depth", "Local points and shape indices"),
        "Query-focused row; not a solver-family benchmark or all-pairs native sandbox.",
    ),
    "rigid_collision_query_options": (
        "Which body-kind pairs does a query include?",
        ("Query toggles and ignored-pair selector", "Baseline, active, filtered, and ignored contacts"),
        "Query-filter and persistent ignored-pair row only; not a shape-family manifold inspector.",
    ),
    "rigid_collision_casts": (
        "Where do rays and swept probes hit?",
        (
            "Nearest/all ray hit fractions",
            "Sphere/capsule time of impact",
        ),
        "Public collision-cast query row; not a contact-solver, no-tunneling, or CCD time-step guarantee.",
    ),
    "rigid_solver_compare": (
        "How do the rigid method families differ visually?",
        ("Sequential impulse vs IPC lanes", "Wall clearance and divergence"),
        "Generic thin-wall comparison; not the sole no-tunneling proof.",
    ),
    "rigid_executor_equivalence": (
        "Does a parallel executor preserve the same physics?",
        ("Pose and velocity divergence", "Contact-count delta and timing"),
        "Same-solver executor-equivalence row; not a solver-family comparison.",
    ),
    "rigid_contact_solver_compare": (
        "What changes when contact solver policy changes?",
        ("Sequential impulse vs boxed-LCP policy", "Depth, clearance, divergence"),
        "Contact-policy row only; it does not compare IPC against sequential impulse.",
    ),
    "contact": (
        "Do articulated links contact like rigid bodies?",
        ("Drop, slide, and pusher link lanes", "Link/rigid contact counts and travel"),
        "Multibody-link contact row only; not a contact-impulse or compliance inspector.",
    ),
    "rigid_friction_threshold": (
        "Where is the inclined-ramp stick/slip boundary?",
        ("Below/controlled/above threshold lanes", "Down-slope drift and speed"),
        "Near-threshold behavior is tunable visual evidence, not an exact proof point.",
    ),
    "rigid_spin_roll_coupling": (
        "How does contact friction couple sliding and spin?",
        ("Slip speed and roll ratio", "Spin change and energy"),
        "Spin/rolling visual diagnostic only; no public rolling-resistance or torsional-friction parameter claim.",
    ),
    "rigid_stack_stability": (
        "Does a top-heavy mass-ratio stack stay ordered?",
        ("Top-block drift and clearance", "Sequential impulse vs IPC divergence"),
        "Compact two-box stack only; taller IPC stacks remain benchmark/capture-first.",
    ),
    "rigid_contact_manipulation": (
        "Can a rigid pusher move an object through contact?",
        ("Target travel and pusher gap", "Contact evidence and goal error"),
        "Task-like pusher row only; not a full arm/gripper manipulation controller.",
    ),
    "rigid_kinematic_driver": (
        "Does prescribed motion carry objects by contact?",
        ("IPC grip vs zero-friction slip", "Driver/box travel and speed ratio"),
        "Tangential kinematic-driver row only; normal pushing is routed to the next row.",
    ),
    "rigid_kinematic_normal_push": (
        "Can prescribed normal motion push a target?",
        ("IPC penetration caveat vs SI push", "Gap, depth, contact count, and target travel"),
        "Normal kinematic-pusher caveat row only; IPC exposes penetration rather than a robust manipulation path.",
    ),
    "rigid_fixed_joint": (
        "Does a fixed joint preserve its captured pose?",
        ("Offset and orientation error", "Payload residual speed"),
        "Fixed-pose row only; no motor, limit, or break-force claims.",
    ),
    "rigid_joint_breakage": (
        "What happens when a fixed joint breaks?",
        ("Broken/intact state and connector color", "Payload release and reset"),
        "AVBD-pinned editable-threshold breakage row; no sequential-impulse or IPC break-force parity claim.",
    ),
    "rigid_distance_spring": (
        "How do rigid-body distance springs enforce rest length?",
        ("Soft/stiff center springs", "Off-center anchor torque"),
        "World rigid-body distance-spring row only; IPC and multibody worlds reject this public API.",
    ),
    "rigid_limited_joints": (
        "Do one-DOF joints keep only their free axis?",
        ("Locked-direction errors", "Hinge yaw and slider travel"),
        "Revolute/prismatic constraint row only; public motor/limit behavior is out of scope.",
    ),
    "rigid_joint_motor_limits": (
        "Do joint motors and limits clamp commands?",
        ("Velocity and position limit behavior", "Effort cap acceleration gap"),
        "Multibody joint-actuator row; rigid-body joint motor behavior is not claimed.",
    ),
    "rigid_joint_passive_parameters": (
        "Do passive joint parameters shape motion?",
        ("Spring/damping/stiction lanes", "Energy and expected acceleration"),
        "Contact-free passive-parameter row; no motor, limit, or contact-load claims.",
    ),
    "rigid_screw_joint_pitch": (
        "Does screw pitch couple rotation and translation?",
        ("Travel-per-radian ratio", "Effective mass and acceleration"),
        "Contact-free screw-pitch row; no contact, motor, limit, or loop-closure claims.",
    ),
    "rigid_multibody_dynamics_terms": (
        "What do generalized dynamics terms mean?",
        ("Mass and inverse-mass terms", "Inverse dynamics and impulse residuals"),
        "Contact-free joint-space dynamics row; not a Cartesian point-force or COM-Jacobian claim.",
    ),
    "rigid_link_center_of_mass": (
        "How does a link center-of-mass offset change inertia and gravity torque?",
        ("Centered vs mirrored COM offsets", "Gravity torque and mass matrix"),
        "Link inertial-offset row only; not arbitrary-point Jacobians, contact, IK, or operational-space control.",
    ),
    "rigid_link_jacobian": (
        "What does a link Jacobian map?",
        ("J qdot vs finite difference", "J.T wrench power consistency"),
        "Link-origin kinematic/wrench map only; not arbitrary point, COM, contact, IK, or OSC.",
    ),
    "rigid_multibody_solver_family": (
        "Which multibody integration family supports solves?",
        ("Residual-only vs solved lanes", "Residual ratio and tip error"),
        "Solver-family routing row; closure family selection remains in the next row.",
    ),
    "rigid_loop_closure": (
        "Which loop-closure family should I use?",
        ("POINT, DISTANCE, and RIGID families", "Residual-only vs solved behavior"),
        "Public-family comparison row; not a compliance, breakage, or distance-family solver sweep.",
    ),
}

_RIGID_VISUAL_WORKFLOW_CHECKLIST_TEXT: Mapping[str, tuple[str, str]] = {
    "rigid_body": (
        "Try solver/material controls, then reset the baseline.",
        "Healthy: contacts settle while energy, speed, and step timing remain bounded.",
    ),
    "rigid_body_modes": (
        "Change gravity, force, and kinematic speed with the same solver.",
        "Healthy: dynamic moves, static stays put, kinematic follows its path.",
    ),
    "rigid_free_flight": (
        "Adjust launch, gravity, spin speed, and inertia ratio.",
        "Healthy: drift/arc references stay close and contact count remains zero.",
    ),
    "rigid_frame_hierarchy": (
        "Move the body yaw/path and local sensor offset.",
        "Healthy: world and relative transform residuals stay near zero.",
    ),
    "rigid_external_loads": (
        "Change force, torque, impulse magnitudes, mass ratio, and inertia ratio.",
        "Healthy: acceleration scales with mass and impulse momentum matches the direct kick.",
    ),
    "rigid_link_point_loads": (
        "Move the point offset and compare centered/off-center lanes.",
        "Healthy: centered force translates; off-center force also yaws.",
    ),
    "rigid_timestep_sensitivity": (
        "Change base time step and gravity scale across matched lanes.",
        "Healthy: coarse error is larger while lane times remain comparable.",
    ),
    "rigid_step_diagnostics": (
        "Switch solver/executor and reset while watching stage summaries.",
        "Healthy: profile, ECS, contact, and scratch counters stay finite.",
    ),
    "rigid_contact_scale_budget": (
        "Set the frame budget and compare one, four, and nine-box lanes.",
        "Healthy: denser lanes cost more and the budget status changes predictably.",
    ),
    "rigid_restitution_ladder": (
        "Move launch height and restitution scale before changing solver.",
        "Healthy: higher restitution rebounds higher than lower restitution.",
    ),
    "rigid_material_mixing": (
        "Change low/high restitution and friction for swapped ownership lanes.",
        "Healthy: swapped lanes share max restitution and sqrt friction effects.",
    ),
    "rigid_contact_inspector": (
        "Pick a shape pair and penetration depth.",
        "Healthy: selected contacts expose finite point, normal, depth, and indices.",
    ),
    "rigid_collision_query_options": (
        "Toggle one include flag or select one ignored pair at a time.",
        "Healthy: option-filtered and pair-ignored lanes separate cleanly.",
    ),
    "rigid_collision_casts": (
        "Sweep ray offset, all-hit, sphere radius, and capsule controls.",
        "Healthy: hit fractions, time of impact, and cast margins update consistently.",
    ),
    "rigid_solver_compare": (
        "Change launch speed/friction, then compare SI and IPC lanes.",
        "Healthy: wall clearance and divergence make solver-family differences visible.",
    ),
    "rigid_executor_equivalence": (
        "Hold the physics solver fixed and compare executor timing.",
        "Healthy: pose, velocity, and contact-count deltas stay small.",
    ),
    "rigid_contact_solver_compare": (
        "Change launch speed, friction, restitution, and initial tilt.",
        "Healthy: contact-policy divergence appears without changing solver family.",
    ),
    "contact": (
        "Tune friction/restitution, then compare drop, slide, and pusher lanes.",
        "Healthy: link contacts rebound, brake sliding, and move the target.",
    ),
    "rigid_friction_threshold": (
        "Move ramp angle and controlled friction around the threshold.",
        "Healthy: below/above lanes separate into slip and stick trends.",
    ),
    "rigid_spin_roll_coupling": (
        "Change contact friction, launch speed, and backspin ratio.",
        "Healthy: slip-to-roll and backspin lanes change spin and travel distinctly.",
    ),
    "rigid_stack_stability": (
        "Increase top mass ratio and friction, then compare SI and IPC.",
        "Healthy: top drift, clearance, and divergence expose stack stability.",
    ),
    "rigid_contact_manipulation": (
        "Adjust pusher speed, friction, and pusher mass.",
        "Healthy: target travel grows while gap/contact evidence stays coherent.",
    ),
    "rigid_kinematic_driver": (
        "Change driver speed and grip friction.",
        "Healthy: IPC grip carries the box; zero-friction and SI caveat lanes separate.",
    ),
    "rigid_kinematic_normal_push": (
        "Change push speed and target mass.",
        "Healthy: SI pushes the target while IPC depth exposes the current caveat.",
    ),
    "rigid_fixed_joint": (
        "Use perturb, then reset the captured fixed-pose verifier.",
        "Healthy: offset/orientation errors return toward the captured pose.",
    ),
    "rigid_joint_breakage": (
        "Let the AVBD break-force lifecycle run, then reset breakage.",
        "Healthy: broken state, connector color, and payload release agree.",
    ),
    "rigid_distance_spring": (
        "Reset stretched soft, stiff, and off-center spring lanes.",
        "Healthy: springs reduce stretch while off-center anchors spin the payload.",
    ),
    "rigid_limited_joints": (
        "Use perturb, then compare hinge yaw and slider travel.",
        "Healthy: free axes move while locked-direction errors stay bounded.",
    ),
    "rigid_joint_motor_limits": (
        "Change speed command, limits, requested force, and effort cap.",
        "Healthy: speed, stops, and acceleration clamp at the displayed limits.",
    ),
    "rigid_joint_passive_parameters": (
        "Change spring/rest, damping, Coulomb friction, and armature.",
        "Healthy: energy, stiction/slip, and acceleration follow the parameter change.",
    ),
    "rigid_screw_joint_pitch": (
        "Change pitch scale, gravity scale, mass, and axial inertia.",
        "Healthy: axial travel tracks pitch sign and acceleration scales with mass.",
    ),
    "rigid_multibody_dynamics_terms": (
        "Change target acceleration, impulse, distal mass, and gravity.",
        "Healthy: mass, inverse dynamics, and impulse residuals stay consistent.",
    ),
    "rigid_link_center_of_mass": (
        "Move the COM offset, mass, gravity, and high-inertia multiplier.",
        "Healthy: centered COM stays still; mirrored COM offsets accelerate in opposite directions.",
    ),
    "rigid_link_jacobian": (
        "Change motion speed, elbow phase, and wrench controls.",
        "Healthy: J qdot matches finite difference and J.T wrench matches power.",
    ),
    "rigid_multibody_solver_family": (
        "Change gravity scale while comparing residual-only and solved lanes.",
        "Healthy: variational solved lanes reduce closure residuals.",
    ),
    "rigid_loop_closure": (
        "Change gravity scale and compare POINT, DISTANCE, and RIGID families.",
        "Healthy: each closure family reduces its matching residual in solved lanes.",
    ),
}

_RIGID_VISUAL_WORKFLOW_SEARCH_ALIASES: Mapping[str, tuple[str, ...]] = {
    "rigid_body_modes": (
        "activation state",
        "body activation",
        "body deactivation",
        "deactivation",
        "island activation",
        "sleep",
        "sleep state",
        "sleep wake",
        "sleeping body",
        "wake",
    ),
    "rigid_external_loads": (
        "apply angular impulse",
        "apply force",
        "apply linear impulse",
        "apply torque",
        "apply_angular_impulse",
        "apply_force",
        "apply_linear_impulse",
        "apply_torque",
        "angular impulse",
        "angular momentum",
        "direct impulse",
        "direct rigid body impulse",
        "force accumulator",
        "impulse kick",
        "instant impulse",
        "linear impulse",
        "linear momentum",
        "rigid body apply angular impulse",
        "rigid body apply force",
        "rigid body apply linear impulse",
        "rigid body apply torque",
        "rigidbody.apply_angular_impulse",
        "rigidbody.apply_force",
        "rigidbody.apply_linear_impulse",
        "rigidbody.apply_torque",
        "rigid body impulse",
        "torque accumulator",
        "velocity impulse",
    ),
    "rigid_timestep_sensitivity": (
        "dt",
        "gravity tuning",
        "substep",
        "substeps",
        "time step size",
        "time-step size",
    ),
    "rigid_step_diagnostics": (
        "step profile",
        "step profiling",
        "step time",
        "step timing",
        "stage timing",
        "accelerated backend",
        "backend active",
        "backend comparison",
        "backend equivalence",
        "backend/executor",
        "compute backend",
        "cuda",
        "cuda acceleration",
        "cuda backend",
        "gpu acceleration",
        "gpu",
        "gpu backend",
        "execution backend",
        "backend status",
        "latency",
        "memory diagnostics",
        "parallel backend",
        "step latency",
        "step wall time",
        "timing breakdown",
        "thread count",
        "wall time",
        "worker count",
        "worker threads",
    ),
    "rigid_contact_scale_budget": (
        "contact budget",
        "contact count",
        "contact throughput",
        "fps",
        "frame time",
        "frame budget",
        "perf",
        "performance",
        "throughput",
    ),
    "rigid_restitution_ladder": (
        "bounce coefficient",
        "coefficient of restitution",
        "elasticity",
        "restitution coefficient",
    ),
    "rigid_material_mixing": (
        "combine friction",
        "contact material",
        "coefficient of friction",
        "friction coefficient",
        "friction mixing",
        "material combine",
        "material coefficient",
        "material friction",
        "mu",
    ),
    "rigid_collision_casts": (
        "capsule cast",
        "collisiongroup",
        "convex cast",
        "ray query",
        "raycast_result",
        "ray cast",
        "ray casting",
        "raycast",
        "shape cast",
        "sphere cast",
        "sweep collision",
        "sweep query",
        "sweep test",
        "swept volume",
        "swept capsule",
        "swept shape",
        "swept sphere",
        "time of impact",
        "time-of-impact",
        "toi",
    ),
    "rigid_solver_compare": (
        "method family",
        "rigid body solver",
        "rigid body solver family",
        "rigid solver method",
        "rigid-body solver",
        "rigid-body solver family",
        "rigidbodysolver",
        "si",
        "si vs ipc",
        "sequential impulse",
        "sequential impulse vs ipc",
        "solver method",
        "ipc",
    ),
    "rigid_executor_equivalence": (
        "compute executor",
        "executor comparison",
        "executor only",
        "multithreaded executor",
        "parallel executor",
        "parallel rollout",
        "parallelism",
        "same solver parallel",
        "same physics solver",
        "taskflow",
        "taskflow executor",
        "threaded executor",
    ),
    "rigid_contact_solver_compare": (
        "boxed lcp",
        "boxed-lcp",
        "complementarity-aware",
        "contact complementarity",
        "contact formulation",
        "contact method",
        "contact model",
        "contact policy",
        "contact solvermethod",
        "contact solver method",
        "contact solver policy",
        "contact-policy pair",
        "contact-solver method",
        "contactsolvermethod",
        "lcp",
        "sequential impulse contacts",
        "sequential impulse vs boxed lcp",
        "solver policy",
    ),
    "rigid_collision_query_options": (
        "body kind filter",
        "body-kind filter",
        "body type filter",
        "collision filter",
        "collisionqueryoptions",
        "ignored collision pair",
    ),
    "rigid_friction_threshold": (
        "ipc friction",
        "static friction",
        "static friction coefficient",
        "stick slip",
    ),
    "rigid_spin_roll_coupling": (
        "dynamic friction",
        "kinetic friction",
        "rolling friction",
        "sliding friction",
        "spin friction",
    ),
    "rigid_stack_stability": (
        "box pile",
        "collapse",
        "ipc stack",
        "jitter",
        "pile",
        "resting contact",
        "sequential impulse vs ipc stack",
        "settling",
        "stable stack",
        "stacking",
        "stack jitter",
        "top heavy",
        "top-heavy stack",
    ),
    "rigid_kinematic_driver": ("ipc kinematic",),
    "rigid_kinematic_normal_push": ("ipc normal", "sequential impulse normal"),
    "rigid_joint_passive_parameters": (
        "joint armature",
        "joint damping",
        "joint friction",
        "joint stiffness",
        "passive joint",
        "passive joint parameters",
        "spring damping",
        "spring stiffness",
    ),
    "rigid_multibody_dynamics_terms": (
        "bias forces",
        "compute_impulse_response",
        "compute_inverse_dynamics",
        "coriolis",
        "coriolis_forces",
        "generalized force",
        "generalized forces",
        "generalized impulse",
        "gravity compensation",
        "impulse response",
        "inverse_mass_matrix",
        "inverse dynamics",
        "joint space dynamics",
        "joint-space dynamics",
        "mass matrix",
        "mass_matrix",
        "multibody.compute_impulse_response",
    ),
    "rigid_multibody_solver_family": (
        "multibody solver family",
        "residual only",
        "semi implicit",
        "semi-implicit",
        "semiimplicit",
        "solved closure",
        "variational integrator",
        "variational solver",
    ),
    "rigid_loop_closure": (
        "closed chain",
        "closed chain robot",
        "closed loop",
        "closed-chain",
        "closed-loop",
        "closure compliance",
        "closure constraint",
        "closure damping",
        "closure stiffness",
        "compliance",
        "compliant loop closure",
        "compliant constraints",
        "constraint damping",
        "constraint stiffness",
        "distance constraint",
        "four bar",
        "four-bar linkage",
        "loop closure compliance",
        "loop closure damping",
        "loop closure stiffness",
        "loop constraint",
        "loop compliance",
        "point constraint",
    ),
}


def _rigid_workflow_capture_command(
    scene_id: str,
    frames: int,
    width: int,
    height: int,
    show_ui: bool,
) -> str:
    command = (
        "pixi run py-demo-capture -- "
        f"--scene {scene_id} --frames {frames} --width {width} --height {height}"
    )
    if show_ui:
        command = f"{command} --show-ui"
    return command


def _rigid_workflow_viewer_command(scene_id: str, width: int, height: int) -> str:
    return f"pixi run py-demos -- --scene {scene_id} --width {width} --height {height}"


def _rigid_workflow_packet_command(
    *,
    include_related: bool = False,
    include_ipc_shelf: bool = False,
    include_packets: bool = False,
    start_row: int | None = None,
    end_row: int | None = None,
    video: bool = False,
    fps: int = 24,
    continue_on_failure: bool = False,
    output_dir: str | None = None,
) -> str:
    command = "pixi run py-demo-capture -- --rigid-workflow"
    if include_related:
        command = f"{command} --include-related"
    if include_ipc_shelf:
        command = f"{command} --include-ipc-shelf"
    if include_packets:
        command = f"{command} --include-packets"
    if start_row is not None:
        command = f"{command} --workflow-start-row {start_row}"
    if end_row is not None:
        command = f"{command} --workflow-end-row {end_row}"
    if video:
        command = f"{command} --video --fps {fps}"
    if continue_on_failure:
        command = f"{command} --continue-on-failure"
    if output_dir:
        command = f"{command} --output-dir {output_dir}"
    return command


def _rigid_workflow_row_packet_command(guide: RigidWorkflowGuide) -> str:
    return _rigid_workflow_packet_command(
        start_row=guide.index,
        end_row=guide.index,
        output_dir=(
            "/tmp/"
            f"dart_capture_rigid_workflow_row_{guide.index:02d}_{guide.scene_id}"
        ),
    )


def _rigid_workflow_row_video_packet_command(guide: RigidWorkflowGuide) -> str:
    return _rigid_workflow_packet_command(
        start_row=guide.index,
        end_row=guide.index,
        video=True,
        fps=24,
        output_dir=(
            "/tmp/"
            f"dart_capture_rigid_workflow_row_{guide.index:02d}_{guide.scene_id}_motion"
        ),
    )


def _make_rigid_workflow_guides() -> dict[str, RigidWorkflowGuide]:
    scene_ids = [scene_id for scene_id, _label in RIGID_VISUAL_WORKFLOW_LABELS]
    count = len(scene_ids)
    guides: dict[str, RigidWorkflowGuide] = {}
    for index, (scene_id, label) in enumerate(
        RIGID_VISUAL_WORKFLOW_LABELS, start=1
    ):
        question, inspect, scope = _RIGID_VISUAL_WORKFLOW_GUIDE_TEXT[scene_id]
        try_first, healthy_signal = _RIGID_VISUAL_WORKFLOW_CHECKLIST_TEXT[scene_id]
        frames, width, height, show_ui = _RIGID_VISUAL_WORKFLOW_CAPTURE_BY_SCENE[
            scene_id
        ]
        guides[scene_id] = RigidWorkflowGuide(
            scene_id=scene_id,
            index=index,
            count=count,
            label=label,
            question=question,
            try_first=try_first,
            inspect=inspect,
            healthy_signal=healthy_signal,
            scope=scope,
            previous_scene_id=scene_ids[index - 2] if index > 1 else None,
            next_scene_id=scene_ids[index] if index < count else None,
            capture_frames=frames,
            capture_width=width,
            capture_height=height,
            capture_show_ui=show_ui,
            capture_command=_rigid_workflow_capture_command(
                scene_id, frames, width, height, show_ui
            ),
        )
    return guides


RIGID_VISUAL_WORKFLOW_GUIDES = _make_rigid_workflow_guides()
_RIGID_VISUAL_WORKFLOW_BADGES = {
    scene_id: (index, label)
    for index, (scene_id, label) in enumerate(
        RIGID_VISUAL_WORKFLOW_LABELS, start=1
    )
}

_RIGID_WORKFLOW_RELATED_EVIDENCE: Mapping[
    str, tuple[RigidWorkflowRelatedEvidence, ...]
] = {
    "rigid_free_flight": (
        RigidWorkflowRelatedEvidence(
            label="World Rigid Body / floating_base - broader floating-joint row",
            scene_id="floating_base",
            shelf="World Rigid Body",
            reason=(
                "Broader floating-joint SE(3) drift/spin example; use the "
                "numbered row for baseline rigid-body initial-state diagnostics."
            ),
        ),
    ),
    "rigid_multibody_dynamics_terms": (
        RigidWorkflowRelatedEvidence(
            label="World Rigid Body / articulated - broader two-link arm row",
            scene_id="articulated",
            shelf="World Rigid Body",
            reason=(
                "Broader two-link arm example; use the numbered row for "
                "mass, inverse-dynamics, and impulse-response diagnostics."
            ),
        ),
    ),
    "rigid_solver_compare": (
        RigidWorkflowRelatedEvidence(
            label="Rigid IPC / rigid_ipc_tunnel - focused no-tunneling view",
            scene_id="rigid_ipc_tunnel",
            shelf="Rigid IPC",
            reason=(
                "Focused IPC capability scene; not a broad solver comparison "
                "or general proof."
            ),
        ),
        RigidWorkflowRelatedEvidence(
            label=(
                "Rigid IPC / rigid_ipc_edge_drop - degenerate edge-contact view"
            ),
            scene_id="rigid_ipc_edge_drop",
            shelf="Rigid IPC",
            reason=(
                "Focused IPC degenerate edge-contact capability scene; not a "
                "broad solver comparison or contact-manifold inspector."
            ),
        ),
    ),
    "rigid_contact_solver_compare": (
        RigidWorkflowRelatedEvidence(
            label="Differentiable / diff_drone_liftoff - contact-gradient route",
            scene_id="diff_drone_liftoff",
            shelf="Differentiable",
            reason=(
                "Analytic vs complementarity-aware clamping-contact "
                "optimization; not a solver row."
            ),
        ),
        RigidWorkflowRelatedEvidence(
            label=(
                "Differentiable / diff_pre_contact_surrogate - "
                "pre-contact gradient route"
            ),
            scene_id="diff_pre_contact_surrogate",
            shelf="Differentiable",
            reason=(
                "Analytic vs pre-contact surrogate backward-only gradient "
                "for an approaching but not touching body; not a solver row."
            ),
        ),
    ),
    "contact": (
        RigidWorkflowRelatedEvidence(
            label=(
                "AVBD Rigid Constraints (sx) / avbd_rigid_fixed_joint_contact "
                "- fixed-joint contact route"
            ),
            scene_id="avbd_rigid_fixed_joint_contact",
            shelf="AVBD Rigid Constraints (sx)",
            reason=(
                "Variational fixed-joint/contact capability scene; not a "
                "World contact-policy comparison."
            ),
        ),
    ),
    "rigid_joint_breakage": (
        RigidWorkflowRelatedEvidence(
            label=(
                "AVBD Rigid Constraints (sx) / avbd_rigid_breakable_joint "
                "- free-rigid fixed break/reset"
            ),
            scene_id="avbd_rigid_breakable_joint",
            shelf="AVBD Rigid Constraints (sx)",
            reason=(
                "Dedicated free-rigid fixed break/reset row; not sequential-"
                "impulse or IPC parity evidence."
            ),
        ),
        RigidWorkflowRelatedEvidence(
            label=(
                "AVBD Rigid Constraints (sx) / "
                "avbd_rigid_spherical_breakable_joint "
                "- spherical anchor break/reset"
            ),
            scene_id="avbd_rigid_spherical_breakable_joint",
            shelf="AVBD Rigid Constraints (sx)",
            reason=(
                "Dedicated free-rigid spherical anchor break/reset row; "
                "orientation remains intentionally free."
            ),
        ),
    ),
    "rigid_joint_motor_limits": (
        RigidWorkflowRelatedEvidence(
            label=(
                "AVBD Rigid Constraints (sx) / avbd_rigid_revolute_motor "
                "- free-rigid hinge motor"
            ),
            scene_id="avbd_rigid_revolute_motor",
            shelf="AVBD Rigid Constraints (sx)",
            reason=(
                "AVBD free-rigid revolute velocity motor; not a World "
                "multibody motor/limit comparison."
            ),
        ),
        RigidWorkflowRelatedEvidence(
            label=(
                "AVBD Rigid Constraints (sx) / avbd_rigid_prismatic_motor "
                "- free-rigid slider motor"
            ),
            scene_id="avbd_rigid_prismatic_motor",
            shelf="AVBD Rigid Constraints (sx)",
            reason=(
                "AVBD free-rigid prismatic velocity motor; not a World "
                "multibody motor/limit comparison."
            ),
        ),
    ),
}


def rigid_workflow_related_evidence_by_scene() -> Mapping[
    str, tuple[RigidWorkflowRelatedEvidence, ...]
]:
    return _RIGID_WORKFLOW_RELATED_EVIDENCE


@dataclass
class ScenePanel:
    """A renderer-neutral custom panel owned by a Python demo scene.

    ``build(builder, context)`` is called once per UI frame. ``builder`` is the
    DART panel abstraction (text, buttons, sliders, plots, tables, etc.);
    ``context`` is a read-only snapshot of viewer state. Scene panels should
    capture scene-owned state explicitly instead of mutating ``context``.
    """

    title: str
    build: Callable[[Any, Any], None]
    dock_side: str = "right"
    initial_size: tuple[float, float] | None = (320.0, 440.0)
    initial_position: tuple[float, float] | None = None
    auto_resize: bool = False
    background_alpha: float | None = None
    horizontal_scrollbar: bool = False
    menu_bar: bool = False


@dataclass
class SceneSetup:
    """The per-scene state returned by a scene's ``build()``.

    ``world`` is the scene's headless stepping object. It may be ``None`` for
    descriptor-only scenes.

    ``pre_step`` is an optional callable invoked before each viewer step
    (controllers, sensor updates). It receives no arguments and returns
    nothing.

    ``step`` is the whole-loop variant: ``step(frames)`` advances the world by
    ``frames`` steps in headless runner paths. The interactive viewer does not
    use it; interactive controllers should use ``pre_step``.

    ``force_drag`` is an optional callable invoked by the viewer's mouse
    "force-drag" while the user left-drags one of this scene's renderables that
    is not backed by a legacy BodyNode (e.g. the sx SimpleFrame mirrors). It
    receives a single event mapping with keys ``renderable_id`` (int),
    ``renderable_name`` (str), ``application_point`` (np.ndarray, world frame),
    ``force`` (np.ndarray, world frame), and ``active`` (bool), and returns
    nothing. The handler must (re)apply the one-shot force on every step while
    ``active`` is true and stop once an event with ``active`` false arrives.

    ``panels`` carries optional custom UI panels for scene-specific controls
    and diagnostics. Panels are rendered through DART's renderer-neutral
    ``PanelBuilder`` abstraction and dock on the right by default.

    ``renderable_provider`` returns the current GUI descriptors for interactive
    rendering. When omitted, the runner also checks whether ``world`` provides a
    ``renderable_provider`` method.

    ``info`` carries scene-specific metadata for tests and panels. Scenes may
    expose ``info["capture_metrics"]`` as a zero-argument callable returning a
    JSON-like mapping for ``py-demo-capture`` manifests, or
    ``info["replay_timeline"]`` metadata for the shared Replay timeline.
    """

    world: Any | None = None
    pre_step: Callable[[], None] | None = None
    step: Callable[[int], None] | None = None
    force_drag: Callable[[dict[str, Any]], None] | None = None
    panels: list[ScenePanel] = field(default_factory=list)
    renderable_provider: Callable[[], list[Any]] | None = None
    info: dict[str, Any] = field(default_factory=dict)


@dataclass
class PythonDemoScene:
    """One entry in the demos catalog."""

    id: str
    title: str
    category: str
    summary: str
    build: Callable[[], SceneSetup]


def _has_world_replay_api(world: Any) -> bool:
    return all(
        hasattr(world, name)
        for name in (
            "replay_recording_enabled",
            "replay_frame_count",
            "replay_cursor",
            "restore_replay_frame",
            "clear_replay_recording",
            "get_replay_frame_time",
            "get_replay_simulation_frame",
        )
    )


def _replay_world_from_setup(setup: SceneSetup) -> Any | None:
    for key in REPLAY_WORLD_INFO_KEYS:
        world = setup.info.get(key)
        if world is not None and _has_world_replay_api(world):
            return world
    if _has_world_replay_api(setup.world):
        return setup.world
    return None


def _sync_callback_from_pre_step(
    pre_step: Callable[[], None] | None, replay_world: Any
) -> Callable[[], None] | None:
    """Find a WorldRenderBridge.sync callback captured by a scene pre-step."""

    def sync_from_object(value: Any) -> Callable[[], None] | None:
        if (
            getattr(value, "_physics_world", None) is replay_world
            and hasattr(value, "sync")
        ):
            return getattr(value, "sync")
        return None

    if pre_step is None:
        return None

    bound_sync = sync_from_object(getattr(pre_step, "__self__", None))
    if bound_sync is not None:
        return bound_sync

    for cell in getattr(pre_step, "__closure__", ()) or ():
        try:
            value = cell.cell_contents
        except ValueError:
            continue
        cell_sync = sync_from_object(value)
        if cell_sync is not None:
            return cell_sync
    return None


def _is_bridge_pre_step(pre_step: Callable[[], None] | None, replay_world: Any) -> bool:
    if pre_step is None:
        return False
    owner = getattr(pre_step, "__self__", None)
    return (
        getattr(owner, "_physics_world", None) is replay_world
        and getattr(pre_step, "__name__", "") == "pre_step"
    )


def _replay_state_callbacks(
    setup: SceneSetup,
) -> tuple[Callable[[], Any] | None, Callable[[Any], None] | None]:
    capture = setup.info.get("replay_capture_state")
    restore = setup.info.get("replay_restore_state")
    if capture is None and restore is None:
        return None, None
    if callable(capture) and callable(restore):
        return capture, restore
    setup.info["shared_replay_skipped_reason"] = (
        "replay_capture_state and replay_restore_state must both be callable"
    )
    return None, None


def _timeline_sample_count(frame_count: int) -> int:
    return max(0, min(int(frame_count), 1024))


def _frame_for_timeline_sample(
    sample_index: int, sample_count: int, frame_count: int
) -> int:
    if sample_count <= 1 or frame_count <= 1:
        return 0
    normalized = float(sample_index) / float(sample_count - 1)
    return int(round(normalized * float(frame_count - 1)))


def _frame_mark_series(frame_count: int) -> list[float]:
    sample_count = _timeline_sample_count(frame_count)
    if sample_count <= 0:
        return []
    marker_interval = max(1, int(frame_count) // 12)
    values = [0.0] * sample_count
    for sample in range(sample_count):
        frame = _frame_for_timeline_sample(sample, sample_count, frame_count)
        if frame == 0 or frame + 1 == frame_count or frame % marker_interval == 0:
            values[sample] = 1.0
    return values


def _cursor_series(frame_count: int, frame: int) -> list[float]:
    sample_count = _timeline_sample_count(frame_count)
    if sample_count <= 0:
        return []
    clamped = max(0, min(int(frame), max(0, int(frame_count) - 1)))
    if frame_count <= 1:
        cursor_sample = 0
    else:
        cursor_sample = int(
            round(float(clamped) * float(sample_count - 1) / float(frame_count - 1))
        )
    values = [0.0] * sample_count
    values[max(0, min(cursor_sample, sample_count - 1))] = 1.0
    return values


def _finite_float(value: Any) -> float | None:
    try:
        converted = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(converted):
        return None
    return converted


def _timeline_source_values(
    source: Any,
    *,
    frame_count: int,
    scene_states: list[Any],
    marker: bool,
) -> list[float]:
    sample_count = _timeline_sample_count(frame_count)
    if sample_count <= 0 or source is None:
        return []

    raw_values: list[Any]
    if callable(source):
        raw_values = []
        for sample in range(sample_count):
            frame = _frame_for_timeline_sample(sample, sample_count, frame_count)
            snapshot = scene_states[frame] if frame < len(scene_states) else None
            try:
                raw_values.append(source(snapshot))
            except Exception:  # noqa: BLE001
                raw_values.append(None)
    else:
        if isinstance(source, str | bytes):
            return []
        try:
            source_values = list(source)
        except TypeError:
            return []
        if not source_values:
            return []
        raw_values = []
        source_count = len(source_values)
        for sample in range(sample_count):
            if source_count == sample_count:
                source_index = sample
            elif source_count >= frame_count:
                source_index = _frame_for_timeline_sample(
                    sample, sample_count, frame_count
                )
            else:
                source_index = _frame_for_timeline_sample(
                    sample, sample_count, source_count
                )
            raw_values.append(source_values[max(0, min(source_index, source_count - 1))])

    values: list[float] = []
    valid_count = 0
    for raw in raw_values:
        value = _finite_float(raw)
        if value is not None:
            valid_count += 1
        if marker:
            values.append(1.0 if value is not None and abs(value) > 0.0 else 0.0)
        else:
            values.append(value if value is not None else 0.0)
    if valid_count == 0:
        return []
    return values


def _timeline_label(metadata: Mapping[str, Any] | None) -> str:
    if metadata is None:
        return "Saved states"
    label = metadata.get("signal_label", metadata.get("label", "Saved states"))
    label_text = str(label).strip()
    return label_text or "Saved states"


class _ReplayController:
    """Shared replay recorder/scrubber for py-demos DART 7 worlds."""

    def __init__(
        self,
        world: Any,
        sync: Callable[[], None] | None,
        capture_state: Callable[[], Any] | None = None,
        restore_state: Callable[[Any], None] | None = None,
        timeline_metadata: Mapping[str, Any] | None = None,
        *,
        max_frames: int = DEFAULT_REPLAY_MAX_FRAMES,
    ) -> None:
        self._world = world
        self._sync = sync or (lambda: None)
        self._capture_state = capture_state
        self._restore_state = restore_state
        self._timeline_metadata = (
            dict(timeline_metadata)
            if isinstance(timeline_metadata, Mapping)
            else None
        )
        self._scene_states: list[Any] = []
        self._max_frames = max(1, int(max_frames))
        self.save_replay = True
        self.replay_active = False
        self.playing = False
        self.loop = False
        self.rate_index = 0
        self.selected_frame = 0
        self.status = "recording"
        self._set_recording(True)
        self.selected_frame = self._current_frame()
        self._capture_scene_state()
        self._sync()

    @property
    def panel(self) -> ScenePanel:
        return ScenePanel(
            "Replay",
            self.build_panel,
            dock_side="bottom",
            initial_size=(960.0, 260.0),
            horizontal_scrollbar=True,
        )

    @property
    def frame_count(self) -> int:
        try:
            return int(self._world.replay_frame_count)
        except Exception:  # noqa: BLE001
            return 0

    def _set_recording(self, enabled: bool) -> None:
        try:
            self._world.replay_recording_enabled = bool(enabled)
        except Exception:  # noqa: BLE001
            self.save_replay = False

    def _recording_enabled(self) -> bool:
        try:
            return bool(self._world.replay_recording_enabled)
        except Exception:  # noqa: BLE001
            return False

    def _current_frame(self) -> int:
        try:
            cursor = self._world.replay_cursor
        except Exception:  # noqa: BLE001
            cursor = None
        if cursor is None:
            return max(0, min(self.selected_frame, max(0, self.frame_count - 1)))
        return max(0, min(int(cursor), max(0, self.frame_count - 1)))

    def _frame_time(self, frame: int) -> float:
        if self.frame_count <= 0:
            return 0.0
        try:
            return float(
                self._world.get_replay_frame_time(
                    max(0, min(int(frame), self.frame_count - 1))
                )
            )
        except Exception:  # noqa: BLE001
            return float(getattr(self._world, "time", 0.0))

    def _simulation_frame(self, frame: int) -> int:
        if self.frame_count <= 0:
            return 0
        try:
            return int(
                self._world.get_replay_simulation_frame(
                    max(0, min(int(frame), self.frame_count - 1))
                )
            )
        except Exception:  # noqa: BLE001
            return int(getattr(self._world, "frame", 0))

    def _pause_viewer(self, context: Any) -> None:
        if hasattr(context, "set_paused"):
            context.set_paused(True)

    def _resume_viewer(self, context: Any) -> None:
        if hasattr(context, "set_paused"):
            context.set_paused(False)

    def _copy_state(self, state: Any) -> Any:
        try:
            return copy.deepcopy(state)
        except Exception:  # noqa: BLE001
            return state

    def _capture_scene_state(self) -> None:
        if self._capture_state is None:
            return
        count = self.frame_count
        if count <= 0:
            self._scene_states.clear()
            return

        current = self._current_frame()
        if len(self._scene_states) > count:
            del self._scene_states[count:]
        while len(self._scene_states) < count:
            self._scene_states.append(None)
        try:
            self._scene_states[current] = self._copy_state(self._capture_state())
        except Exception:  # noqa: BLE001
            self.save_replay = False
            self.playing = False
            self._set_recording(False)
            self.status = "replay state capture failed"

    def _restore_scene_state(self, frame: int) -> None:
        if self._restore_state is None:
            return
        if frame < 0 or frame >= len(self._scene_states):
            return
        snapshot = self._scene_states[frame]
        if snapshot is None:
            return
        try:
            self._restore_state(self._copy_state(snapshot))
        except Exception:  # noqa: BLE001
            self.playing = False
            self.replay_active = False
            self.status = "replay state restore failed"

    def restore_frame(self, index: int) -> None:
        count = self.frame_count
        if count <= 0:
            self.replay_active = False
            self.playing = False
            self.selected_frame = 0
            return
        clamped = max(0, min(int(index), count - 1))
        self._world.restore_replay_frame(clamped)
        self._restore_scene_state(clamped)
        self.selected_frame = clamped
        self.replay_active = True
        self.status = "replay"
        self._sync()

    def on_live_step_complete(self) -> None:
        if not self.save_replay:
            self.status = "live"
            return
        self.selected_frame = self._current_frame()
        self._capture_scene_state()
        if self.frame_count >= self._max_frames:
            self._set_recording(False)
            self.save_replay = False
            self.status = f"saved {self.frame_count} frames"
        else:
            self.status = "recording"

    def pre_step(self, live_step: Callable[[], None] | None) -> None:
        if self.replay_active:
            self._advance_replay()
            return
        if live_step is not None:
            live_step()
        self.on_live_step_complete()

    def _advance_replay(self) -> None:
        count = self.frame_count
        if count <= 0:
            self.replay_active = False
            self.playing = False
            return
        if not self.playing:
            self.restore_frame(self.selected_frame)
            return
        rate = _REPLAY_RATE_STEPS[
            max(0, min(int(self.rate_index), len(_REPLAY_RATE_STEPS) - 1))
        ]
        next_frame = self.selected_frame + rate
        if next_frame >= count:
            if self.loop:
                next_frame = next_frame % count
            else:
                next_frame = count - 1
                self.playing = False
        self.restore_frame(next_frame)

    def _set_save_replay(self, enabled: bool) -> None:
        self.save_replay = bool(enabled)
        if self.save_replay:
            self.replay_active = False
            self.playing = False
            self._set_recording(True)
            self.selected_frame = self._current_frame()
            self._capture_scene_state()
            self.status = "recording"
        else:
            self._set_recording(False)
            self.status = "live"

    def _clear(self) -> None:
        self.playing = False
        self.replay_active = False
        self._world.clear_replay_recording()
        self.selected_frame = self._current_frame()
        self._capture_scene_state()
        self._sync()

    def _resume_live(self, context: Any) -> None:
        self.playing = False
        self.replay_active = False
        if self.save_replay and not self._recording_enabled():
            self._set_recording(True)
        self.status = "recording" if self.save_replay else "live"
        self._resume_viewer(context)

    def _timeline_tracks(self, count: int) -> tuple[list[float], list[float], str]:
        metadata = self._timeline_metadata
        if metadata is None:
            return [], _frame_mark_series(count), "Saved states"

        signal = metadata.get("signal", metadata.get("value"))
        markers = metadata.get("markers", metadata.get("marker"))
        value_track = _timeline_source_values(
            signal,
            frame_count=count,
            scene_states=self._scene_states,
            marker=False,
        )
        marker_track = _timeline_source_values(
            markers,
            frame_count=count,
            scene_states=self._scene_states,
            marker=True,
        )
        if not value_track and not marker_track:
            return [], _frame_mark_series(count), "Saved states"
        if value_track and not marker_track:
            marker_track = _frame_mark_series(count)
        return value_track, marker_track, _timeline_label(metadata)

    def build_panel(self, builder: Any, context: Any) -> None:
        count = self.frame_count
        current = max(0, min(self.selected_frame, max(0, count - 1)))
        if not self.replay_active:
            current = self._current_frame()
            self.selected_frame = current
        status = "replaying" if self.replay_active else self.status
        if self.save_replay and self._recording_enabled() and not self.replay_active:
            status = "recording"
        builder.text(
            f"{status} | frames {count}/{self._max_frames} | "
            f"time {self._frame_time(current):.3f} s | "
            f"sim frame {self._simulation_frame(current)}"
        )

        changed, save = builder.checkbox("Save replay", bool(self.save_replay))
        builder.item_tooltip(
            "Record DART 7 World states while the demo runs; disabling it "
            "keeps existing replay frames but stops storing new states."
        )
        if changed:
            self._set_save_replay(bool(save))
            current = self.selected_frame
            count = self.frame_count

        builder.same_line()
        if builder.button("Clear replay"):
            self._clear()
            count = self.frame_count
            current = self.selected_frame
        builder.item_tooltip("Discard saved frames and restart from the current state.")
        builder.same_line()
        if builder.button("Resume live"):
            self._resume_live(context)
            current = self.selected_frame
        builder.item_tooltip("Leave replay mode and continue simulating from this state.")

        if count <= 0:
            builder.separator()
            builder.text("No replay frames saved yet.")
            return

        builder.separator()
        if builder.button("|<##replay_first"):
            self.playing = False
            self.restore_frame(0)
            self._pause_viewer(context)
        builder.item_tooltip("First replay frame.")
        builder.same_line()
        if builder.button("<##replay_previous"):
            self.playing = False
            self.restore_frame(current - 1)
            self._pause_viewer(context)
        builder.item_tooltip("Previous replay frame.")
        builder.same_line()
        if builder.button("Pause##replay_pause" if self.playing else "Play##replay_play"):
            self.replay_active = True
            self.playing = not self.playing
            if self.playing:
                self._resume_viewer(context)
            else:
                self._pause_viewer(context)
        builder.item_tooltip("Play or pause saved-state replay.")
        builder.same_line()
        if builder.button(">##replay_next"):
            self.playing = False
            self.restore_frame(current + 1)
            self._pause_viewer(context)
        builder.item_tooltip("Next replay frame.")
        builder.same_line()
        if builder.button(">|##replay_last"):
            self.playing = False
            self.restore_frame(count - 1)
            self._pause_viewer(context)
        builder.item_tooltip("Last replay frame.")
        builder.same_line()
        changed, loop = builder.checkbox("Loop replay", bool(self.loop))
        if changed:
            self.loop = bool(loop)
        builder.same_line()
        changed, rate_index = builder.select(
            "Rate", int(self.rate_index), list(_REPLAY_RATE_LABELS)
        )
        if changed:
            self.rate_index = max(
                0, min(int(rate_index), len(_REPLAY_RATE_STEPS) - 1)
            )

        current = max(0, min(self.selected_frame, count - 1))
        value_track, marker_track, value_track_label = self._timeline_tracks(count)
        selected = float(current)
        changed, selected = builder.timeline(
            "Saved states##py_demo_replay_timeline",
            selected,
            0.0,
            float(max(0, count - 1)),
            value_track=value_track,
            marker_track=marker_track,
            cursor_track=_cursor_series(count, current),
            value_track_label=value_track_label,
        )
        if changed:
            self.playing = False
            self.restore_frame(int(round(selected)))
            self._pause_viewer(context)

        builder.text(f"selected {self.selected_frame + 1}/{count}")


def _attach_replay_controls(scene: PythonDemoScene, setup: SceneSetup) -> SceneSetup:
    if setup.info.get("disable_shared_replay"):
        return setup

    replay_world = _replay_world_from_setup(setup)
    if replay_world is None:
        return setup

    capture_state, restore_state = _replay_state_callbacks(setup)
    if (
        setup.pre_step is not None
        and not _is_bridge_pre_step(setup.pre_step, replay_world)
        and capture_state is None
        and not setup.info.get("replay_live_step_is_stateless")
    ):
        setup.info["shared_replay_skipped_reason"] = (
            "custom pre_step needs replay_capture_state/replay_restore_state "
            "or replay_live_step_is_stateless"
        )
        return setup

    sync = setup.info.get("replay_sync")
    if sync is None:
        sync = _sync_callback_from_pre_step(setup.pre_step, replay_world)
    controller = _ReplayController(
        replay_world,
        sync,
        capture_state=capture_state,
        restore_state=restore_state,
        timeline_metadata=setup.info.get(REPLAY_TIMELINE_INFO_KEY),
    )
    live_pre_step = setup.pre_step

    def replay_pre_step() -> None:
        controller.pre_step(live_pre_step)

    setup.pre_step = replay_pre_step
    setup.panels = [*setup.panels, controller.panel]
    setup.info["replay_world"] = replay_world
    setup.info["replay_controller"] = controller
    setup.info["replay_panel_title"] = controller.panel.title
    setup.info["replay_scene_id"] = scene.id
    return setup


def _capture_metrics_json_value(value: Any) -> Any:
    if value is None or isinstance(value, bool | int | str):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if hasattr(value, "item"):
        try:
            return _capture_metrics_json_value(value.item())
        except Exception:  # noqa: BLE001
            pass
    if hasattr(value, "tolist"):
        try:
            return _capture_metrics_json_value(value.tolist())
        except Exception:  # noqa: BLE001
            pass
    if isinstance(value, Mapping):
        return {
            str(key): _capture_metrics_json_value(item)
            for key, item in value.items()
        }
    if isinstance(value, list | tuple):
        return [_capture_metrics_json_value(item) for item in value]
    return str(value)


def _capture_metrics_mapping(value: Any) -> dict[str, Any]:
    json_value = _capture_metrics_json_value(value)
    if isinstance(json_value, dict):
        return json_value
    return {"value": json_value}


def _append_capture_metrics_event(
    event_log_path: str,
    scene_id: str,
    frame: int,
    metrics: Mapping[str, Any],
) -> None:
    parent = os.path.dirname(os.path.abspath(event_log_path))
    if parent:
        os.makedirs(parent, exist_ok=True)
    payload = {
        "event": CAPTURE_METRICS_EVENT_NAME,
        "frame": int(frame),
        "metrics": dict(metrics),
        "scene": scene_id,
        "source": "py-demo-scene",
    }
    with open(event_log_path, "a", encoding="utf-8") as stream:
        stream.write(json.dumps(payload, sort_keys=True) + "\n")


def _replay_timeline_capture_metadata(
    setup: SceneSetup,
) -> dict[str, Any] | None:
    metadata = setup.info.get(REPLAY_TIMELINE_INFO_KEY)
    if not isinstance(metadata, Mapping):
        return None
    replay_panel_title = setup.info.get("replay_panel_title")
    if not isinstance(replay_panel_title, str) or not replay_panel_title:
        return None
    signal = metadata.get("signal", metadata.get("value"))
    markers = metadata.get("markers", metadata.get("marker"))
    return {
        "has_markers": markers is not None,
        "has_signal": signal is not None,
        "panel": replay_panel_title,
        "signal_label": _timeline_label(metadata),
    }


def _capture_metadata_mapping(setup: SceneSetup) -> dict[str, Any]:
    metadata: dict[str, Any] = {}
    replay_timeline = _replay_timeline_capture_metadata(setup)
    if replay_timeline is not None:
        metadata[REPLAY_TIMELINE_INFO_KEY] = replay_timeline
    skipped_reason = setup.info.get("shared_replay_skipped_reason")
    if isinstance(skipped_reason, str) and skipped_reason:
        metadata["shared_replay_skipped_reason"] = skipped_reason
    return metadata


def _append_capture_metadata_event(
    event_log_path: str,
    scene_id: str,
    setup: SceneSetup,
) -> None:
    if not event_log_path:
        return
    metadata = _capture_metadata_mapping(setup)
    if not metadata:
        return
    parent = os.path.dirname(os.path.abspath(event_log_path))
    if parent:
        os.makedirs(parent, exist_ok=True)
    payload = {
        "event": CAPTURE_METADATA_EVENT_NAME,
        "metadata": metadata,
        "scene": scene_id,
        "source": "py-demo-scene",
    }
    with open(event_log_path, "a", encoding="utf-8") as stream:
        stream.write(json.dumps(payload, sort_keys=True) + "\n")


def _attach_capture_metrics_recording(
    scene: PythonDemoScene,
    setup: SceneSetup,
    event_log_path: str,
) -> SceneSetup:
    if not event_log_path or setup.pre_step is None:
        return setup
    capture_metrics = setup.info.get(CAPTURE_METRICS_INFO_KEY)
    if not callable(capture_metrics):
        return setup

    original_pre_step = setup.pre_step
    frame = 0

    def metrics_pre_step() -> None:
        nonlocal frame
        original_pre_step()
        frame += 1
        try:
            metrics = _capture_metrics_mapping(capture_metrics())
        except Exception as exc:  # noqa: BLE001
            metrics = {
                "error": str(exc),
                "status": "capture-metrics-failed",
            }
        _append_capture_metrics_event(event_log_path, scene.id, frame, metrics)

    setup.pre_step = metrics_pre_step
    return setup


def _step(setup: SceneSetup, frames: int) -> None:
    """Advance the scene by ``frames`` steps headlessly.

    Honors (in order of precedence):
      1. SceneSetup.step (whole-loop callable, owns world.step() itself).
      2. SceneSetup.pre_step (run the controller body, then world.step()
         here). Same pattern the interactive viewer uses, so the same
         scene runs in both surfaces without duplicate logic.
      3. Plain world.step() per frame.
    """

    if setup.step is not None:
        setup.step(frames)
        return
    pre = setup.pre_step
    for _ in range(max(0, frames)):
        if pre is not None:
            pre()
        if setup.world is not None and hasattr(setup.world, "step"):
            setup.world.step()


def _print_catalog(scenes: list[PythonDemoScene]) -> None:
    last_category: str | None = None
    for entry in scenes:
        if entry.category != last_category:
            print(f"\n[{entry.category}]")
            last_category = entry.category
        print(f"  {entry.id:<28s} {entry.title} — {entry.summary}")


def _viewer_catalog_title(scene: PythonDemoScene) -> str:
    """Return the display title used by the interactive Demos navigator."""

    if scene.category != RIGID_VISUAL_WORKFLOW_CATEGORY:
        return scene.title
    badge = _RIGID_VISUAL_WORKFLOW_BADGES.get(scene.id)
    if badge is None:
        return scene.title
    index, label = badge
    count = len(RIGID_VISUAL_WORKFLOW_LABELS)
    return f"{index:02d}/{count:02d} {label}: {scene.title}"


def _workflow_route_text(prefix: str, scene_id: str | None) -> str:
    if scene_id is None:
        endpoint = "start" if prefix == "Previous" else "done"
        return f"{prefix}: {endpoint}"
    guide = RIGID_VISUAL_WORKFLOW_GUIDES[scene_id]
    return f"{prefix}: {guide.index:02d}/{guide.count:02d} {guide.label}"


def _request_workflow_scene(context: Any, scene_id: str) -> None:
    request_scene_switch = getattr(context, "request_scene_switch", None)
    if callable(request_scene_switch):
        request_scene_switch(scene_id)


def _request_workflow_replay(context: Any, scene_id: str) -> None:
    request_scene_replay = getattr(context, "request_scene_replay", None)
    if callable(request_scene_replay):
        request_scene_replay(scene_id)
    else:
        _request_workflow_scene(context, scene_id)


def _workflow_route_row(
    builder: Any,
    context: Any,
    prefix: str,
    scene_id: str | None,
) -> None:
    label = _workflow_route_text(prefix, scene_id)
    if scene_id is None:
        builder.text(label)
        return

    if builder.selectable(f"{label}##rigid_workflow_{prefix.lower()}", False):
        _request_workflow_scene(context, scene_id)
    builder.item_tooltip(f"Open {label.lower()}.")


def _workflow_replay_row(
    builder: Any,
    context: Any,
    guide: RigidWorkflowGuide,
) -> None:
    label = f"Restart row: {guide.index:02d}/{guide.count:02d} {guide.label}"
    if builder.selectable(f"{label}##rigid_workflow_restart", False):
        _request_workflow_replay(context, guide.scene_id)
    builder.item_tooltip("Reload this workflow row and clear replay playback.")


def _workflow_related_evidence_rows(
    builder: Any,
    context: Any,
    guide: RigidWorkflowGuide,
) -> None:
    entries = _RIGID_WORKFLOW_RELATED_EVIDENCE.get(guide.scene_id, ())
    if not entries:
        return

    builder.text("Related evidence")
    for entry in entries:
        label = f"Related shelf: {entry.label}"
        if builder.selectable(
            f"{label}##rigid_workflow_related_{entry.scene_id}",
            False,
        ):
            _request_workflow_scene(context, entry.scene_id)
        builder.item_tooltip(
            f"Open {entry.scene_id} from the {entry.shelf} shelf. {entry.reason}"
        )


def _workflow_jump_choices() -> tuple[str, ...]:
    count = len(RIGID_VISUAL_WORKFLOW_LABELS)
    return tuple(
        f"{index:02d}/{count:02d} {label}"
        for index, (_scene_id, label) in enumerate(
            RIGID_VISUAL_WORKFLOW_LABELS, start=1
        )
    )


def _workflow_jump_row(builder: Any, context: Any, guide: RigidWorkflowGuide) -> None:
    changed, next_index = builder.select(
        "Jump to row",
        guide.index - 1,
        _workflow_jump_choices(),
    )
    if changed and 0 <= int(next_index) < len(RIGID_VISUAL_WORKFLOW_LABELS):
        next_scene_id = RIGID_VISUAL_WORKFLOW_LABELS[int(next_index)][0]
        if next_scene_id != guide.scene_id:
            _request_workflow_scene(context, next_scene_id)


def _workflow_search_row_id_text(guide: RigidWorkflowGuide) -> str:
    row_id = f"{guide.index:02d}/{guide.count:02d}"
    unpadded_row_id = f"{guide.index}/{guide.count}"
    return f"row {guide.index} row {guide.index:02d} {row_id} {unpadded_row_id}"


def _workflow_search_words(text: str) -> tuple[str, ...]:
    split_camel = re.sub(r"(?<=[A-Z])(?=[A-Z][a-z])", " ", text)
    split_camel = re.sub(r"(?<=[a-z0-9])(?=[A-Z])", " ", split_camel)
    return tuple(re.findall(r"[a-z0-9]+", split_camel.lower()))


def _workflow_search_tokens(query: str) -> tuple[str, ...]:
    return _workflow_search_words(query)


def _workflow_search_phrase(text: str) -> str:
    return " ".join(_workflow_search_words(text))


def _workflow_search_word_variants(words: Iterable[str]) -> set[str]:
    variants = set(words)
    for word in words:
        if len(word) <= 3:
            continue
        if word.endswith("ies"):
            variants.add(f"{word[:-3]}y")
        elif word.endswith("s"):
            if not word.endswith(("is", "ss", "us")):
                variants.add(word[:-1])
        else:
            variants.add(f"{word}s")
    return variants


def _workflow_text_matches(text: str, tokens: tuple[str, ...]) -> bool:
    if not tokens:
        return False
    words = _workflow_search_words(text)
    word_variants = _workflow_search_word_variants(words)
    phrase = " ".join(words)
    compact = "".join(words)
    for token in tokens:
        token_variants = _workflow_search_word_variants((token,))
        if len(token) <= 2:
            if word_variants.isdisjoint(token_variants):
                return False
        elif not any(
            variant in word_variants or variant in phrase or variant in compact
            for variant in token_variants
        ):
            return False
    return True


def _workflow_core_search_score(
    guide: RigidWorkflowGuide, tokens: tuple[str, ...]
) -> int:
    query_text = " ".join(tokens)
    aliases = _RIGID_VISUAL_WORKFLOW_SEARCH_ALIASES.get(guide.scene_id, ())
    positive_fields = (
        (1000, _workflow_search_row_id_text(guide)),
        (900, guide.scene_id),
        (800, guide.label),
        (600, guide.question),
        (350, guide.try_first),
        (300, " ".join(guide.inspect)),
        (250, guide.healthy_signal),
    )

    positive_score = 0
    if query_text == _workflow_search_phrase(guide.scene_id):
        positive_score += 1800
    if query_text == _workflow_search_phrase(guide.label):
        positive_score += 1600
    if query_text in {_workflow_search_phrase(alias) for alias in aliases}:
        positive_score += 4000
    elif len(tokens) > 1 and any(
        _workflow_text_matches(alias, tokens) for alias in aliases
    ):
        positive_score += 700
    for weight, text in positive_fields:
        if _workflow_text_matches(text, tokens):
            positive_score += weight

    return positive_score


def _workflow_related_evidence_matches(
    guide: RigidWorkflowGuide, tokens: tuple[str, ...]
) -> tuple[RigidWorkflowRelatedEvidence, ...]:
    if not tokens:
        return ()

    matches: list[RigidWorkflowRelatedEvidence] = []
    for entry in _RIGID_WORKFLOW_RELATED_EVIDENCE.get(guide.scene_id, ()):
        text = " ".join((entry.scene_id, entry.shelf, entry.label))
        if _workflow_text_matches(text, tokens):
            matches.append(entry)
    return tuple(matches)


def _workflow_search_score(guide: RigidWorkflowGuide, tokens: tuple[str, ...]) -> int:
    positive_score = _workflow_core_search_score(guide, tokens)
    if positive_score > 0:
        return positive_score

    if _workflow_related_evidence_matches(guide, tokens):
        return 200

    scope = guide.scope.lower()
    if _workflow_text_matches(scope, tokens):
        return 25
    return 0


def _workflow_matching_guides(query: str) -> tuple[RigidWorkflowGuide, ...]:
    tokens = _workflow_search_tokens(query)
    if not tokens:
        return ()

    matches: list[tuple[int, RigidWorkflowGuide]] = []
    for scene_id, _label in RIGID_VISUAL_WORKFLOW_LABELS:
        candidate = RIGID_VISUAL_WORKFLOW_GUIDES[scene_id]
        score = _workflow_search_score(candidate, tokens)
        if score > 0:
            matches.append((score, candidate))

    matches.sort(key=lambda item: (-item[0], item[1].index))
    return tuple(candidate for _score, candidate in matches[:6])


def _workflow_search_rows(
    builder: Any,
    context: Any,
    guide: RigidWorkflowGuide,
    query: str,
) -> str:
    changed, next_query = builder.text_input("Find row", query)
    if changed:
        query = str(next_query)

    tokens = _workflow_search_tokens(query)
    matches = _workflow_matching_guides(query)
    for match in matches:
        selected = match.scene_id == guide.scene_id
        related_matches = ()
        if _workflow_core_search_score(match, tokens) <= 0:
            related_matches = _workflow_related_evidence_matches(match, tokens)
        label = (
            f"{match.index:02d}/{match.count:02d} {match.label} - "
            f"{match.scene_id}"
        )
        if related_matches:
            related_scene_ids = ", ".join(
                entry.scene_id for entry in related_matches[:2]
            )
            label = f"{label} (related: {related_scene_ids})"
        if builder.selectable(
            f"{label}##rigid_workflow_find_{match.scene_id}", selected
        ):
            if related_matches:
                _request_workflow_scene(context, related_matches[0].scene_id)
            elif selected:
                _request_workflow_replay(context, match.scene_id)
            else:
                _request_workflow_scene(context, match.scene_id)
        tooltip = match.question
        if related_matches:
            related_notes = "; ".join(
                f"{entry.shelf} / {entry.scene_id}: {entry.reason}"
                for entry in related_matches[:2]
            )
            tooltip = (
                f"{tooltip} Related evidence match: {related_notes}. "
                f"Click opens {related_matches[0].scene_id}."
            )
        builder.item_tooltip(tooltip)

    if query.strip() and not matches:
        builder.text("No matching workflow rows")

    return query


def _make_rigid_workflow_panel(scene: PythonDemoScene) -> ScenePanel | None:
    if scene.category != RIGID_VISUAL_WORKFLOW_CATEGORY:
        return None
    guide = RIGID_VISUAL_WORKFLOW_GUIDES.get(scene.id)
    if guide is None:
        return None

    search_query = ""

    def build(builder: Any, _context: Any) -> None:
        nonlocal search_query

        builder.text(f"{guide.index:02d}/{guide.count:02d} {guide.label}")
        builder.text(scene.title)
        builder.separator()
        builder.text("Question")
        builder.text(guide.question)
        builder.separator()
        builder.text("Try first")
        builder.text(guide.try_first)
        builder.separator()
        builder.text("Look for")
        for signal in guide.inspect:
            builder.text(signal)
        builder.text(guide.healthy_signal)
        builder.separator()
        builder.text("Do not infer")
        builder.text(guide.scope)
        builder.separator()
        builder.text("Capture evidence")
        ui_mode = "docked UI" if guide.capture_show_ui else "headless"
        builder.text(
            f"{guide.capture_frames} frames | "
            f"{guide.capture_width}x{guide.capture_height} | {ui_mode}"
        )
        builder.text(
            _rigid_workflow_viewer_command(
                guide.scene_id, guide.capture_width, guide.capture_height
            )
        )
        builder.item_tooltip("Open this row live in py-demos for interactive debugging.")
        builder.text(guide.capture_command)
        builder.item_tooltip("Run from the repository root to regenerate this row.")
        builder.separator()
        builder.text("Review packet")
        builder.text(
            _rigid_workflow_packet_command(
                output_dir="/tmp/dart_capture_rigid_workflow"
            )
        )
        builder.item_tooltip("Capture all numbered rows and write review_index.html.")
        builder.text(_rigid_workflow_row_packet_command(guide))
        builder.item_tooltip("Capture only this workflow row in a review packet.")
        builder.text(_rigid_workflow_row_video_packet_command(guide))
        builder.item_tooltip(
            "Capture this row as a review packet with PNG frames and MP4 motion."
        )
        builder.text(
            _rigid_workflow_packet_command(
                include_related=True,
                include_ipc_shelf=True,
                include_packets=True,
                output_dir="/tmp/dart_capture_rigid_workflow_extended",
            )
        )
        builder.item_tooltip(
            "Capture numbered rows plus related, Rigid IPC shelf, and packet rows."
        )
        builder.text(
            _rigid_workflow_packet_command(
                include_related=True,
                include_ipc_shelf=True,
                include_packets=True,
                continue_on_failure=True,
                output_dir="/tmp/dart_capture_rigid_workflow_resilient",
            )
        )
        builder.item_tooltip(
            "Capture the extended packet while preserving later-row evidence "
            "after a row fails."
        )
        builder.separator()
        builder.text("Route")
        _workflow_replay_row(builder, _context, guide)
        _workflow_route_row(builder, _context, "Previous", guide.previous_scene_id)
        _workflow_route_row(builder, _context, "Next", guide.next_scene_id)
        _workflow_related_evidence_rows(builder, _context, guide)
        _workflow_jump_row(builder, _context, guide)
        search_query = _workflow_search_rows(
            builder, _context, guide, search_query
        )

    return ScenePanel(
        "Rigid Workflow",
        build,
        dock_side="right",
        initial_size=(340.0, 360.0),
    )


def _canonical_scene_id(scene_id: str) -> str:
    return scene_id.replace("-", "_")


def _validate_scene(scene_id: str | None, scenes: list[PythonDemoScene]) -> None:
    if scene_id is None:
        return
    canonical = _canonical_scene_id(scene_id)
    if not any(entry.id == canonical for entry in scenes):
        available = ", ".join(entry.id for entry in scenes)
        raise SystemExit(
            f"unknown --scene '{scene_id}'. Known scenes: {available}"
        )


def _scene_build_timeout_ms() -> float | None:
    value = os.environ.get(SCENE_BUILD_TIMEOUT_ENV)
    python_specific_override = value is not None and value != ""
    if value is None or value == "":
        value = os.environ.get(DEMO_SCENE_STARTUP_TIMEOUT_ENV)
    if value is None or value == "":
        return DEFAULT_SCENE_BUILD_TIMEOUT_MS

    try:
        timeout_ms = float(value)
    except ValueError:
        return DEFAULT_SCENE_BUILD_TIMEOUT_MS
    if timeout_ms <= 0.0:
        if python_specific_override:
            return None
        return DEFAULT_SCENE_BUILD_TIMEOUT_MS
    return timeout_ms


@contextlib.contextmanager
def _bounded_scene_callback(scene_id: str, callback_name: str):
    timeout_ms = _scene_build_timeout_ms()
    if (
        timeout_ms is None
        or threading.current_thread() is not threading.main_thread()
        or not hasattr(signal, "SIGALRM")
        or not hasattr(signal, "ITIMER_REAL")
        or not hasattr(signal, "getitimer")
        or not hasattr(signal, "setitimer")
    ):
        yield
        return

    previous_handler = signal.getsignal(signal.SIGALRM)
    previous_timer = signal.getitimer(signal.ITIMER_REAL)
    start = time.monotonic()

    def _handle_timeout(_signum: int, _frame: object) -> None:
        raise TimeoutError(
            f"Python demo scene '{scene_id}' {callback_name} exceeded "
            f"{timeout_ms:g} ms"
        )

    signal.signal(signal.SIGALRM, _handle_timeout)
    signal.setitimer(signal.ITIMER_REAL, timeout_ms / 1000.0)
    try:
        yield
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0.0)
        signal.signal(signal.SIGALRM, previous_handler)
        previous_delay, previous_interval = previous_timer
        if previous_delay > 0.0:
            elapsed = time.monotonic() - start
            signal.setitimer(
                signal.ITIMER_REAL,
                max(1.0e-6, previous_delay - elapsed),
                previous_interval,
            )


@contextlib.contextmanager
def _bounded_scene_build(scene_id: str):
    with _bounded_scene_callback(scene_id, "build"):
        yield


def _make_world_factory(
    scene: PythonDemoScene,
    gpu_panel: ScenePanel | None = None,
    capture_metrics_event_log: str = "",
) -> Callable[[], Any]:
    """Wrap scene.build() so dart.gui.run_demos can call it as a factory.

    Returns a tuple consumed by the viewer binding:
      ``(pre_step, force_drag, panels, renderable_provider)``.
    """

    def factory() -> Any:
        with _bounded_scene_build(scene.id):
            setup = scene.build()
        setup = _attach_replay_controls(scene, setup)
        _append_capture_metadata_event(capture_metrics_event_log, scene.id, setup)
        setup = _attach_capture_metrics_recording(
            scene, setup, capture_metrics_event_log
        )
        pre_step = setup.pre_step
        if pre_step is not None:
            original_pre_step = pre_step

            def bounded_pre_step() -> None:
                with _bounded_scene_callback(scene.id, "pre_step"):
                    original_pre_step()

            pre_step = bounded_pre_step
        panels = list(setup.panels)
        if gpu_panel is not None:
            # A runner-injected, scene-independent GPU compute toggle (only
            # present when CUDA is available); see _make_gpu_panel.
            panels.append(gpu_panel)
        renderable_provider = setup.renderable_provider
        if renderable_provider is None and setup.world is not None:
            renderable_provider = getattr(setup.world, "renderable_provider", None)
        workflow_panel = _make_rigid_workflow_panel(scene)
        if workflow_panel is not None:
            panels.insert(0, workflow_panel)
        return (
            pre_step,
            setup.force_drag,
            panels if panels else None,
            renderable_provider,
        )

    return factory


def _gpu_preference(cli_pref: bool | None) -> str:
    """Resolve the GPU-compute preference: ``on`` / ``off`` / ``auto``.

    Precedence: an explicit ``--gpu`` / ``--no-gpu`` flag, then the
    ``DART_PY_DEMOS_GPU`` environment variable, else ``auto`` (enable when a
    CUDA device is available).
    """

    if cli_pref is not None:
        return "on" if cli_pref else "off"
    env = os.environ.get("DART_PY_DEMOS_GPU", "auto").strip().lower()
    if env in {"1", "on", "true", "yes", "gpu", "cuda"}:
        return "on"
    if env in {"0", "off", "false", "no", "cpu"}:
        return "off"
    return "auto"


def _strip_gpu_flags(argv: list[str]) -> list[str]:
    """Drop ``--gpu`` / ``--no-gpu`` before forwarding argv to the C++ viewer.

    The viewer's argument parser does not know these runner-local flags, so they
    must not reach it.
    """

    return [arg for arg in argv if arg not in {"--gpu", "--no-gpu"}]


def _strip_runner_local_flags(argv: list[str]) -> list[str]:
    """Drop runner-local flags before forwarding argv to the C++ viewer."""

    stripped: list[str] = []
    skip_next = False
    for arg in argv:
        if skip_next:
            skip_next = False
            continue
        if arg in {"--gpu", "--no-gpu"}:
            continue
        if arg == "--capture-metrics-event-log":
            skip_next = True
            continue
        if arg.startswith("--capture-metrics-event-log="):
            continue
        stripped.append(arg)
    return stripped


def _default_initial_scene_args(
    argv: list[str], scene_arg: str | None, environ: Mapping[str, str]
) -> list[str]:
    """Forward a showcase default without changing the catalog order."""

    if scene_arg is not None:
        return []
    if environ.get("DART_DEMOS_SCENE"):
        return []
    if "--cycle-scenes" in argv:
        return []
    return ["--scene", DEFAULT_INITIAL_SCENE_ID]


def _make_gpu_panel(sx: Any) -> ScenePanel:
    """A small in-viewer panel that toggles GPU (CUDA) deformable solve.

    The PSD-projection backend is process-wide, so this single toggle affects
    every deformable scene. Only injected when CUDA is available.
    """

    def build(builder: Any, _context: Any) -> None:
        builder.text("GPU compute (CUDA)")
        builder.separator()
        enabled = bool(sx.is_accelerated_deformable_solve_enabled())
        changed, new_enabled = builder.checkbox("GPU deformable solve", enabled)
        if changed:
            sx.set_accelerated_deformable_solve(bool(new_enabled))
        builder.text("Offloads the deformable projected-Newton")
        builder.text("PSD projection to the GPU (CUDA).")
        builder.text("Process-wide: affects every deformable scene.")

    return ScenePanel("GPU", build, dock_side="right", initial_size=(300.0, 150.0))


def _configure_gpu_compute(dart: Any, cli_pref: bool | None) -> ScenePanel | None:
    """Apply the resolved GPU preference and return an in-app toggle panel.

    Returns a ``ScenePanel`` to inject into every scene when CUDA is available,
    otherwise ``None`` (the default/CPU build is left untouched). Prints a
    one-line status so headless and interactive runs both report the mode.
    """

    # In DART 7 the ECS simulation API is flat on the dartpy module, so the GPU
    # deformable-solve controls live directly on it when the World stack is built.
    sx = dart
    if not hasattr(sx, "set_accelerated_deformable_solve"):
        return None

    available = bool(sx.is_accelerated_deformable_solve_available())
    preference = _gpu_preference(cli_pref)
    if preference == "off":
        enabled = bool(sx.set_accelerated_deformable_solve(False))
    elif preference == "on":
        enabled = bool(sx.set_accelerated_deformable_solve(True))
        if not enabled:
            print(
                "py-demos: GPU requested but CUDA is unavailable; using CPU.",
                file=sys.stderr,
            )
    else:  # auto
        enabled = bool(sx.set_accelerated_deformable_solve(available))

    if available:
        print(
            f"py-demos: GPU deformable solve (CUDA) "
            f"{'ON' if enabled else 'off'} [{preference}]; "
            f"toggle in the GPU panel or with --gpu/--no-gpu."
        )
        return _make_gpu_panel(sx)

    print(
        "py-demos: CUDA not available in this build; " "deformable solve runs on CPU."
    )
    return None


def run(argv: Iterable[str], scenes: list[PythonDemoScene]) -> int:
    """Entry point. ``argv`` is the program argv excluding argv[0]."""

    if not scenes:
        print("runner: no scenes registered", file=sys.stderr)
        return 1

    argv = list(argv)

    # Intercept --list locally so callers don't need the viewer to enumerate.
    parser = argparse.ArgumentParser(
        prog="dart-demos (python)",
        description="Run DART Python demo scenes through the dartpy.gui viewer.",
        add_help=False,
    )
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--scene", default=None)
    parser.add_argument("--help", "-h", action="store_true")
    # GPU (CUDA) deformable-solve toggle. Default (unset) is "auto": enable when
    # a CUDA device is available (e.g. under `pixi run -e cuda py-demos`),
    # otherwise run on CPU. DART_PY_DEMOS_GPU overrides the default.
    parser.add_argument("--gpu", dest="gpu", action="store_true", default=None)
    parser.add_argument("--no-gpu", dest="gpu", action="store_false")
    parser.add_argument("--capture-metrics-event-log", default="")
    known, _passthrough = parser.parse_known_args(argv)

    if known.list:
        _print_catalog(scenes)
        return 0

    _validate_scene(known.scene, scenes)
    default_initial_scene_args = _default_initial_scene_args(
        argv, known.scene, os.environ
    )
    if default_initial_scene_args:
        _validate_scene(DEFAULT_INITIAL_SCENE_ID, scenes)

    # Import dartpy.gui lazily so `--list` works even when the GUI
    # backend isn't built (e.g. on CI variants without filament).
    import dartpy as dart

    if not hasattr(dart, "gui") or not hasattr(dart.gui, "run_demos"):
        # Defensive fallback: the GUI binding isn't present. Stop with a
        # helpful error — the catalog is still printable via --list.
        print(
            "dartpy.gui.run_demos not available (build dartpy with GUI support).",
            file=sys.stderr,
        )
        return 2

    # Resolve the GPU-compute preference before scenes build/step. Returns an
    # in-viewer toggle panel when CUDA is available, else None (CPU build).
    gpu_panel = _configure_gpu_compute(dart, known.gpu)

    # Build the catalog for the viewer.
    catalog = [
        (
            scene.id,
            _viewer_catalog_title(scene),
            scene.category,
            scene.summary,
            _make_world_factory(
                scene,
                gpu_panel,
                capture_metrics_event_log=known.capture_metrics_event_log,
            ),
        )
        for scene in scenes
    ]

    # The viewer's argument parser does not know the runner-local flags.
    full_argv = [
        "py-demos",
        *default_initial_scene_args,
        *_strip_runner_local_flags(argv),
    ]
    return int(dart.gui.run_demos(catalog, full_argv))
