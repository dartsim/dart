#!/usr/bin/env python3
"""Write the CT-011 restore-equivalence evidence packet (PLAN-123 WS2/WS7).

Bounded claim (corpus row CT-011, from RobotDART): research workflows need
fast reset, concurrency, low overhead, and deterministic synchronous stepping
around DART. This packet measures the restore-equivalence half of that need
on DART 7 `main`: after `world.state_vector = snapshot`, is the continuation
a function of the restored state alone?

The fixture is a five-sphere pile settling on a ground box, run per contact
solver, with a ballistic (no-ground) control scene. Arms, each hashed over
the FULL rigid-body state (pose including orientation, linear and angular
velocity, per body) for every post-restore step -- hashing the translational
`state_vector` itself would blind the comparison to exactly the rotational
divergence this packet documents:

- `continuation`: keep stepping from the snapshot point (baseline);
- `inplace_restore` x2: restore the snapshot into the same world twice, with
  different amounts of history before each restore;
- `fresh_restore` x2: restore the snapshot into two freshly built worlds;
- `same_history_restore`: two separately built worlds with identical step
  histories, both restored and continued;
- `precontact_history_restore`: a world whose history ends before the first
  contact, restored and continued;
- `ballistic_control`: the same protocol with no ground, where restore must
  be trivially exact if the state vector is complete for free motion.

Reset cost, allocation, and concurrency are the other halves of the corpus
row and are explicitly not measured here (typed unsupported).

Usage (after `pixi run build`):

    PYTHONPATH=build/default/cpp/Release/python pixi run python \
        scripts/write_citation_ct011_restore_equivalence_packet.py
"""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import subprocess
import sys
from pathlib import Path
from typing import Any

import dartpy as sx
import numpy as np
from citation_packet_utils import (
    UNSUPPORTED_SOLVER_RESIDUAL,
    preserve_review,
    target_fetch_hint,
    world_resolved_configuration,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = (
    REPO_ROOT
    / "docs"
    / "plans"
    / "123-citation-driven-simulation-trust"
    / "evidence"
    / "CT-011-dart7-restore-equivalence.json"
)

SCENE_PARAMETERS: dict[str, Any] = {
    "scene_id": "ct011_restore_equivalence_pile",
    "description": (
        "Five 0.5 kg spheres (radius 0.06 m) dropped from staggered offsets "
        "into a loose pile on a static ground box; snapshot taken after a "
        "0.5 s warm-up (well after first contact), then 0.2 s continuations "
        "compared bit-exactly across restore protocols. A ballistic variant "
        "with no ground is the control."
    ),
    "gravity_mps2": [0.0, 0.0, -9.81],
    "sphere_count": 5,
    "sphere_radius_m": 0.06,
    "sphere_mass_kg": 0.5,
    "friction": 0.6,
    "restitution": 0.2,
    "ground_half_extents_m": [1.0, 1.0, 0.05],
    "timestep_s": 0.002,
    "warmup_steps": 250,
    "continuation_steps": 100,
    "precontact_history_steps": 50,
    "extra_history_steps": 100,
    "contact_solver_methods": ["SEQUENTIAL_IMPULSE", "BOXED_LCP"],
    "deterministic_repeats": 2,
}


def scene_digest(parameters: dict[str, Any]) -> str:
    canonical = json.dumps(parameters, sort_keys=True, separators=(",", ":"))
    return "sha256:" + hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def build_world(
    method_name: str, parameters: dict[str, Any], *, with_ground: bool
) -> Any:
    world = sx.World(
        time_step=float(parameters["timestep_s"]),
        gravity=np.asarray(parameters["gravity_mps2"], dtype=float),
        contact_solver_method=sx.ContactSolverMethod[method_name],
    )
    if with_ground:
        ground_half = np.asarray(parameters["ground_half_extents_m"], dtype=float)
        ground = world.add_rigid_body("ct011_ground")
        ground.is_static = True
        ground.set_collision_shape(sx.CollisionShape.box(ground_half))
        transform = np.eye(4)
        transform[2, 3] = -ground_half[2]
        ground.transform = transform
        ground.friction = float(parameters["friction"])
        ground.restitution = float(parameters["restitution"])

    radius = float(parameters["sphere_radius_m"])
    mass = float(parameters["sphere_mass_kg"])
    moment = 0.4 * mass * radius * radius
    for index in range(int(parameters["sphere_count"])):
        body = world.add_rigid_body(f"ct011_sphere{index}")
        body.mass = mass
        body.inertia = np.diag([moment, moment, moment])
        body.set_collision_shape(sx.CollisionShape.sphere(radius))
        body.friction = float(parameters["friction"])
        body.restitution = float(parameters["restitution"])
        transform = np.eye(4)
        transform[:3, 3] = (
            0.03 * (index % 2 * 2 - 1) * (index + 1) / 5.0,
            0.025 * ((index // 2) % 2 * 2 - 1) * (index + 1) / 5.0,
            0.2 + 0.13 * index,
        )
        body.transform = transform
    world.enter_simulation_mode()
    return world


def full_state_array(world: Any) -> np.ndarray:
    """Full rigid-body state: pose (4x4) + linear + angular velocity per
    dynamic body, bodies in name order.

    `World.state_vector` is translational by design (the very CT-011
    finding), so hashing it would blind these comparisons to rotational
    divergence -- especially in the ballistic control, where rotation never
    feeds translation. This observable closes that hole.
    """
    parts: list[np.ndarray] = []
    for name in sorted(world.get_rigid_body_names()):
        body = world.get_rigid_body(name)
        if body.is_static:
            continue
        parts.append(np.asarray(body.transform, dtype=float).reshape(-1))
        parts.append(np.asarray(body.linear_velocity, dtype=float))
        parts.append(np.asarray(body.angular_velocity, dtype=float))
    return np.concatenate(parts)


def hash_continuation(world: Any, steps: int) -> str:
    digest = hashlib.sha256()
    for _ in range(steps):
        world.step()
        digest.update(np.ascontiguousarray(full_state_array(world)).tobytes())
    return digest.hexdigest()


def divergence_profile(
    reference: list[np.ndarray], world: Any, steps: int
) -> dict[str, Any]:
    first = None
    max_delta = 0.0
    for index in range(steps):
        world.step()
        delta = float(np.max(np.abs(full_state_array(world) - reference[index])))
        if delta > 0.0 and first is None:
            first = index
        max_delta = max(max_delta, delta)
    return {
        "first_divergent_step": (
            first
            if first is not None
            else {
                "status": "unsupported",
                "reason": (
                    "The two trajectories are bit-identical over the whole "
                    "window, so there is no divergent step; the value is "
                    "absent, not zero."
                ),
            }
        ),
        "max_abs_state_delta": max_delta,
    }


def run_protocol(method_name: str, parameters: dict[str, Any]) -> dict[str, Any]:
    warmup = int(parameters["warmup_steps"])
    steps = int(parameters["continuation_steps"])
    precontact = int(parameters["precontact_history_steps"])
    extra = int(parameters["extra_history_steps"])

    # Baseline world: warm up, snapshot, continue, and keep the reference
    # trajectory for divergence measurement.
    world = build_world(method_name, parameters, with_ground=True)
    resolved = {
        "world_resolution": world_resolved_configuration(world),
        "contact_solver_method": world.contact_solver_method.name,
        "time_step_s": float(world.time_step),
    }
    for _ in range(warmup):
        world.step()
    snapshot = np.array(world.state_vector, copy=True)
    snapshot_time = float(world.time)

    reference_states: list[np.ndarray] = []
    reference_digest = hashlib.sha256()
    for _ in range(steps):
        world.step()
        state = full_state_array(world)
        reference_states.append(state)
        reference_digest.update(np.ascontiguousarray(state).tobytes())
    continuation_hash = reference_digest.hexdigest()

    # In-place restore, twice, with different history before each restore.
    world.state_vector = snapshot
    world.time = snapshot_time
    inplace_first = hash_continuation(world, steps)
    world.state_vector = snapshot
    world.time = snapshot_time
    inplace_second = hash_continuation(world, steps)

    inplace_probe = build_world(method_name, parameters, with_ground=True)
    for _ in range(warmup + extra):
        inplace_probe.step()
    inplace_probe.state_vector = snapshot
    inplace_probe.time = snapshot_time
    inplace_divergence = divergence_profile(reference_states, inplace_probe, steps)

    # Fresh worlds restoring the same snapshot.
    fresh_hashes = []
    for _ in range(2):
        fresh = build_world(method_name, parameters, with_ground=True)
        fresh.state_vector = snapshot
        fresh.time = snapshot_time
        fresh_hashes.append(hash_continuation(fresh, steps))

    # Two separately built worlds with identical histories.
    same_history_hashes = []
    for _ in range(2):
        twin = build_world(method_name, parameters, with_ground=True)
        for _ in range(warmup):
            twin.step()
        twin.state_vector = snapshot
        twin.time = snapshot_time
        same_history_hashes.append(hash_continuation(twin, steps))

    # History that ends before the first contact.
    precontact_world = build_world(method_name, parameters, with_ground=True)
    for _ in range(precontact):
        precontact_world.step()
    precontact_world.state_vector = snapshot
    precontact_world.time = snapshot_time
    precontact_hash = hash_continuation(precontact_world, steps)

    # Ballistic control: no ground, restore must be exact.
    ballistic = build_world(method_name, parameters, with_ground=False)
    for _ in range(precontact):
        ballistic.step()
    ballistic_snapshot = np.array(ballistic.state_vector, copy=True)
    ballistic_time = float(ballistic.time)
    ballistic_continuation = hash_continuation(ballistic, steps)
    ballistic.state_vector = ballistic_snapshot
    ballistic.time = ballistic_time
    ballistic_restored = hash_continuation(ballistic, steps)

    return {
        "contact_solver_method": method_name,
        "resolved": resolved,
        "continuation_sha256": continuation_hash,
        "inplace_restore_sha256": [inplace_first, inplace_second],
        "fresh_restore_sha256": fresh_hashes,
        "same_history_restore_sha256": same_history_hashes,
        "precontact_history_restore_sha256": precontact_hash,
        "ballistic_control_sha256": [
            ballistic_continuation,
            ballistic_restored,
        ],
        "findings": {
            "inplace_restore_matches_continuation": (
                inplace_first == continuation_hash
            ),
            "inplace_restores_match_each_other": (inplace_first == inplace_second),
            "fresh_restores_match_each_other": (fresh_hashes[0] == fresh_hashes[1]),
            "fresh_restore_matches_continuation": (
                fresh_hashes[0] == continuation_hash
            ),
            "same_history_restores_match": (
                same_history_hashes[0] == same_history_hashes[1]
            ),
            "precontact_history_matches_fresh": (precontact_hash == fresh_hashes[0]),
            "ballistic_restore_exact": (ballistic_continuation == ballistic_restored),
        },
        "inplace_divergence_vs_continuation": inplace_divergence,
    }


def git_head() -> str:
    """HEAD commit, refusing to attribute a dirty tree's results to it.

    A packet's target.commit claims the recorded results come from that
    commit's code; uncommitted modifications to tracked files would make
    that attribution false, so generation aborts instead. (The packet
    output file itself is checked before being overwritten, so an
    unmodified existing packet does not block regeneration.)
    """
    dirty = subprocess.run(
        [
            "git",
            "status",
            "--porcelain",
            "--untracked-files=no",
            "--",
            ".",
            ":(exclude)docs/plans/123-citation-driven-simulation-trust/evidence",
            ":(exclude)docs/design/dart6_citation_driven_contact_trust/evidence",
        ],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    if dirty:
        raise SystemExit(
            "refusing to generate evidence from a dirty tree; commit or "
            "stash these tracked modifications first:\n" + dirty
        )
    return subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def build_packet(output_path: Path | None = None) -> dict[str, Any]:
    parameters = SCENE_PARAMETERS
    rows: list[dict[str, Any]] = []
    determinism_failures: list[str] = []
    resolved_by_cell: dict[str, dict[str, Any]] = {}

    for method in parameters["contact_solver_methods"]:
        repeats = [
            run_protocol(method, parameters)
            for _ in range(int(parameters["deterministic_repeats"]))
        ]
        # The whole protocol must reproduce bit-exactly across repeats.
        keys = (
            "continuation_sha256",
            "inplace_restore_sha256",
            "fresh_restore_sha256",
            "same_history_restore_sha256",
            "precontact_history_restore_sha256",
            "ballistic_control_sha256",
        )
        for key in keys:
            values = {json.dumps(repeat[key]) for repeat in repeats}
            if len(values) != 1:
                determinism_failures.append(
                    f"{method}: {key} differs across protocol repeats"
                )
        row = repeats[0]
        for repeat in repeats:
            if repeat["resolved"] != row["resolved"]:
                raise SystemExit(
                    f"{method}: resolved configuration differs between repeats"
                )
            if repeat["resolved"]["contact_solver_method"] != method:
                raise SystemExit(
                    f"requested contact solver {method} but World readback "
                    f"reports {repeat['resolved']['contact_solver_method']}"
                )
        resolved_by_cell[method] = row["resolved"]
        rows.append(row)

    if determinism_failures:
        raise SystemExit(
            "protocol repeats failed:\n  " + "\n  ".join(determinism_failures)
        )

    finding_summary = {row["contact_solver_method"]: row["findings"] for row in rows}
    # "Function of the restored state alone" must hold across EVERY protocol
    # arm: two worlds given the same restored vector must continue
    # identically regardless of what preceded the restore. Considering only
    # the in-place arms would let a fresh-world or cross-history divergence
    # (a direct counterexample) flip this aggregate to true.
    state_function_arm_keys = (
        "inplace_restore_matches_continuation",
        "inplace_restores_match_each_other",
        "fresh_restores_match_each_other",
        "fresh_restore_matches_continuation",
        "same_history_restores_match",
        "precontact_history_matches_fresh",
        "ballistic_restore_exact",
    )
    restore_is_state_function = all(
        all(row["findings"][key] for key in state_function_arm_keys) for row in rows
    )
    # This writer's claim boundary and limitations assert one SPECIFIC
    # measured pattern: the two in-place arms diverge while the fresh-world,
    # same-history, pre-contact, and ballistic arms match (the omitted
    # rotational state explains exactly that split). ANY deviation -- not
    # just all-arms-exact -- would make the regenerated narrative false, so
    # the writer aborts on the first arm whose outcome changes and forces a
    # conscious rewrite.
    expected_arm_pattern = {
        "inplace_restore_matches_continuation": False,
        "inplace_restores_match_each_other": False,
        "fresh_restores_match_each_other": True,
        "fresh_restore_matches_continuation": False,
        "same_history_restores_match": True,
        "precontact_history_matches_fresh": True,
        "ballistic_restore_exact": True,
    }
    for row in rows:
        deviations = {
            key: row["findings"][key]
            for key, expected in expected_arm_pattern.items()
            if row["findings"][key] != expected
        }
        if deviations:
            raise SystemExit(
                f"CT-011 {row['contact_solver_method']}: measured arm "
                f"outcomes {deviations} deviate from the divergence pattern "
                "this packet's claim boundary asserts (in-place restores "
                "diverge; fresh-world, same-history, pre-contact, and "
                "ballistic arms match). Restore behavior has changed; "
                "rewrite the claim boundary, limitations, and disposition "
                "from the new findings instead of regenerating the old "
                "conclusion."
            )

    # CT-011 is a requirements claim about research workflows; running a
    # fixture cannot reproduce a need, so the row is not promoted. What the
    # packet establishes is whether main currently meets the
    # restore-determinism half of that need.
    disposition = "unresolved"

    command = (
        "PYTHONPATH=build/default/cpp/Release/python pixi run python "
        "scripts/write_citation_ct011_restore_equivalence_packet.py"
    )
    packet: dict[str, Any] = {
        "schema": "dart.citation_claim_evidence/v1",
        "claim_id": "CT-011",
        "title": (
            "State-vector restore equivalence under contact (DART 7 first " "packet)"
        ),
        "source": {
            "url": "https://doi.org/10.21105/joss.06771",
            "claim": (
                "Research workflows need fast reset, concurrency, low "
                "overhead, and deterministic synchronous stepping around "
                "DART."
            ),
        },
        "target": {
            "branch": "main",
            "commit": (head_commit := git_head()),
            "fetch_hint": target_fetch_hint(head_commit),
            "commit_role": (
                "Source state measured: the library and fixture ran at this "
                "commit, which is HEAD at capture time. The packet and its "
                "writer land in a later commit."
            ),
        },
        "scene": {
            "id": parameters["scene_id"],
            "digest": scene_digest(parameters),
            "description": parameters["description"],
            "parameters": parameters,
        },
        "configuration": {
            "requested": {
                "contact_solver_method_sweep": parameters["contact_solver_methods"],
                "restore_mechanism": (
                    "World.state_vector write plus World.time write; "
                    "update_kinematics() made no difference in probing and "
                    "is not part of the protocol"
                ),
                "precision": "float64",
                "backend": "cpu",
                "threads": "World default sequential step",
            },
            "resolved": {"by_cell": resolved_by_cell},
            "resolved_provenance": (
                "The resolved identity is the World's own bake-time "
                "resolution (World.resolved_configuration), recorded per "
                "domain with requested, resolved, reason, and a substitution "
                "flag. The requested contact solver is additionally asserted "
                "against readback per repeat."
            ),
            "detector": (
                "DART 7 native World collision pipeline (the World step API "
                "exposes no detector selection on main)"
            ),
            "timestep": parameters["timestep_s"],
            "substeps": 1,
            "iterations": "World defaults",
            "fallback_policy": (
                "World defaults; no per-island fallback reporting is exposed " "on main"
            ),
        },
        "ensemble": {
            "kind": "protocol-arms-with-deterministic-repeats",
            "sweep": [
                {"contact_solver_method": method}
                for method in parameters["contact_solver_methods"]
            ],
            "deterministic_repeats": int(parameters["deterministic_repeats"]),
            "deterministic_repeats_identical": not determinism_failures,
            "measurement_window": {
                "warmup_steps": parameters["warmup_steps"],
                "continuation_steps": parameters["continuation_steps"],
            },
        },
        "metrics": {
            "physical": {
                "method": (
                    "Bit-exact comparison of full state-vector trajectories "
                    "(SHA-256 over every post-restore step) across restore "
                    "protocols, plus first divergent step and max state "
                    "delta for the in-place arm against the continuation"
                ),
                "per_solver_findings": finding_summary,
                "restore_is_a_function_of_restored_state": (restore_is_state_function),
                "inplace_divergence": {
                    row["contact_solver_method"]: row[
                        "inplace_divergence_vs_continuation"
                    ]
                    for row in rows
                },
                # A first divergent step of exactly zero is the measured
                # finding -- the very first post-restore step already
                # differs -- not a missing value.
                "measured_zero_fields": [
                    f"inplace_divergence.{row['contact_solver_method']}"
                    ".first_divergent_step"
                    for row in rows
                    if row["inplace_divergence_vs_continuation"]["first_divergent_step"]
                    == 0
                ],
            },
            "numerical": {
                "method": (
                    "Hash bookkeeping only; every hash the findings compare "
                    "is published in raw_rows so the booleans can be "
                    "recomputed"
                ),
                "protocol_arms": 7,
                "solver_residual": dict(UNSUPPORTED_SOLVER_RESIDUAL),
            },
            "performance": {
                "status": "unsupported",
                "reason": (
                    "The corpus row also asks for reset cost and overhead; "
                    "no timing methodology was applied, so cost is not "
                    "measured"
                ),
            },
            "allocation": {
                "status": "unsupported",
                "reason": (
                    "Reset allocation behavior is PLAN-122 territory and is "
                    "not measured by this packet"
                ),
            },
        },
        "evidence": {
            "commands": [command],
            "raw_rows": rows,
            "visual": {
                "status": "not-applicable",
                "reason": (
                    "The oracle is bit-exact hash equality between "
                    "trajectories; there is nothing visual to assess."
                ),
            },
        },
        "result": {
            "disposition": disposition,
            "claim_boundary": (
                "DART 7 main, this commit, a five-sphere pile with a 0.5 s "
                "contact-rich warm-up, restored via World.state_vector, "
                "both contact solvers, all arms compared over the FULL "
                "rigid-body state (pose including orientation, linear and "
                "angular velocity) so rotational divergence is visible even "
                "where it does not feed translation. Established: restore is NOT a "
                "function of the restored state once the world has contact "
                "history -- the in-place continuation differs from the "
                "original at the first step, and two in-place restores of "
                "the same snapshot differ from each other when different "
                "history precedes them -- while a freshly built world "
                "restoring the same vector is bit-exact and repeatable, "
                "identical histories give identical continuations, "
                "pre-contact history is harmless, and the ballistic control "
                "is exact. Everything is deterministic given full history; "
                "nothing here is nondeterminism. Root cause: the state "
                "vector is translational by design and silently omits "
                "orientation and angular velocity, so the restore is "
                "partial. The corpus row's reset "
                "cost, overhead, and concurrency halves are not measured. "
                "Says nothing about Skeleton-based state APIs, clone(), "
                "replay restore, historical DART versions, or DART 6."
            ),
            "limitations": [
                "The mechanism is identified: World.state_vector is a "
                "translational state by design (per rigid body it carries "
                "position and linear velocity only -- 30 entries for the "
                "five-sphere scene -- as documented for the differentiable "
                "rigid-body path), so orientation and angular velocity are "
                "never captured or restored. A restored world keeps whatever "
                "rotational state it already had, which is exactly the "
                "history dependence measured here: after the warm-up the "
                "spheres carry finite spin (about 7 rad/s on s0) and rolled "
                "orientations, a fresh world has identity orientations and "
                "zero spin, and friction couples that rotational state into "
                "the continuation. Contact history matters only because "
                "contact is what makes spheres spin.",
                "Full-state alternatives on this branch are "
                "World.save_binary/load_binary (full entity serialization "
                "through a file) and restore_replay_frame; neither is a "
                "lightweight in-memory reset, and no state-vector-like API "
                "carries orientation and angular velocity today.",
                "One scene, one snapshot point, 0.2 s windows; the "
                "magnitude of divergence (up to ~1e-2 in state units within "
                "the window) will vary with scene and horizon.",
                "Reset cost, allocation, and thread/lane isolation are "
                "typed unsupported; they are the other halves of the corpus "
                "row.",
                "The replay-frame restore mechanism "
                "(World.restore_replay_frame) is not tested here and may "
                "have different semantics.",
            ],
        },
        "review": {"passes": []},
        "host": {
            "platform": platform.platform(),
            "python": sys.version.split()[0],
            "machine": platform.machine(),
            "performance_valid": False,
            "note": (
                "Host recorded for provenance only; no timing methodology "
                "was applied"
            ),
        },
    }
    if output_path is not None:
        # Rebind after assembly: only passes whose content_digest matches the
        # regenerated packet survive (see preserve_review).
        packet["review"] = preserve_review(output_path, packet)
    return packet


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help=f"Packet output path (default: {DEFAULT_OUTPUT})",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    packet = build_packet(args.output)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(packet, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    physical = packet["metrics"]["physical"]
    print(f"wrote {args.output}")
    for method, findings in physical["per_solver_findings"].items():
        print(f"  {method}:")
        for key, value in findings.items():
            print(f"    {key}: {value}")
    print(
        "  restore is a function of restored state: "
        f"{physical['restore_is_a_function_of_restored_state']}"
    )
    print(f"  disposition: {packet['result']['disposition']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
