#!/usr/bin/env python3
"""Orchestrate the DART-vs-MuJoCo comparison harness across the fixed scene
matrix, aggregate medians across reps, and emit results.json/results.md.

Runs ``dart_runner.py`` and ``mujoco_runner.py`` as separate subprocesses
(``--dart-python``/``--mujoco-python``, both default ``python``) so each can
live in its own pixi environment. See README.md for the full fairness
contract and scene matrix this implements.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import common
import gen_scenes

THIS_DIR = Path(__file__).resolve().parent
REPO_ROOT = THIS_DIR.parents[1]
DART_RUNNER = THIS_DIR / "dart_runner.py"
MUJOCO_RUNNER = THIS_DIR / "mujoco_runner.py"

# A row is a "win"/"loss" only if it clears this ratio; inside the band is
# reported as noise, per the fairness contract in README.md.
WIN_RATIO = 1.1


@dataclass(frozen=True)
class Scenario:
    scene_id: str
    category: str  # "mjcf" | "generated" | "overhead"
    scene_file: Optional[str]  # repo-root-relative MJCF path, for "mjcf"
    generator_scene_id: Optional[str]  # gen_scenes.py --scene id
    steps: int
    warmup: int
    drive: bool
    drive_amplitude: float
    drive_frequency: float
    detector: str
    sleep: str
    seed: int = 0


SCENARIOS: list[Scenario] = [
    Scenario(
        scene_id="ARM-REACHER",
        category="mjcf",
        scene_file="data/mjcf/openai/reacher.xml",
        generator_scene_id=None,
        steps=2000,
        warmup=100,
        drive=True,
        drive_amplitude=1.0,
        drive_frequency=0.5,
        detector="default",
        sleep="on",
    ),
    Scenario(
        scene_id="ARM-PUSHER",
        category="mjcf",
        scene_file="data/mjcf/openai/pusher.xml",
        generator_scene_id=None,
        steps=2000,
        warmup=100,
        drive=True,
        drive_amplitude=5.0,
        drive_frequency=0.5,
        detector="default",
        sleep="on",
    ),
    Scenario(
        scene_id="HUM-FALL",
        category="mjcf",
        scene_file="data/mjcf/openai/humanoid.xml",
        generator_scene_id=None,
        steps=2000,
        warmup=100,
        drive=False,
        drive_amplitude=0.0,
        drive_frequency=0.0,
        detector="default",
        sleep="on",
    ),
    Scenario(
        scene_id="HUM-ACTIVE",
        category="mjcf",
        scene_file="data/mjcf/openai/humanoid.xml",
        generator_scene_id=None,
        steps=2000,
        warmup=100,
        drive=True,
        drive_amplitude=40.0,
        drive_frequency=0.5,
        detector="default",
        sleep="on",
    ),
    Scenario(
        scene_id="PILE-120",
        category="generated",
        scene_file=None,
        generator_scene_id="pile-120",
        steps=2000,
        warmup=100,
        drive=False,
        drive_amplitude=0.0,
        drive_frequency=0.0,
        detector="default",
        sleep="on",
    ),
    Scenario(
        scene_id="PILE-900",
        category="generated",
        scene_file=None,
        generator_scene_id="pile-900",
        steps=2000,
        warmup=100,
        drive=False,
        drive_amplitude=0.0,
        drive_frequency=0.0,
        detector="default",
        sleep="on",
    ),
    Scenario(
        scene_id="DYN-STIR-120",
        category="generated",
        scene_file=None,
        generator_scene_id="dyn-stir-120",
        steps=2000,
        warmup=100,
        drive=False,
        drive_amplitude=0.0,
        drive_frequency=0.0,
        detector="default",
        sleep="off",
    ),
]

# DART 6.20 cannot yet load the stacked hinge joints in humanoid.xml. Keep the
# scenarios available for explicit parser-development runs, but do not let an
# ordinary comparison spend time on rows that are known to be blocked.
DEFAULT_SCENE_IDS = {
    "ARM-REACHER",
    "ARM-PUSHER",
    "PILE-120",
    "PILE-900",
    "DYN-STIR-120",
}

OVERHEAD_SCENARIO = Scenario(
    scene_id="FFI-OVERHEAD",
    category="overhead",
    scene_file=None,
    generator_scene_id="ffi-overhead",
    steps=2000,
    warmup=100,
    drive=False,
    drive_amplitude=0.0,
    drive_frequency=0.0,
    detector="default",
    sleep="off",
)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scene",
        action="append",
        default=[],
        help="Restrict the run to these scene ids (repeatable); defaults to "
        "the 5 DART-supported scenario ids plus FFI-OVERHEAD.",
    )
    parser.add_argument("--reps", type=int, default=5, help="Reps per (scene, engine).")
    parser.add_argument(
        "--dart-python",
        default="python",
        help="Python executable used to invoke dart_runner.py.",
    )
    parser.add_argument(
        "--mujoco-python",
        default="python",
        help="Python executable used to invoke mujoco_runner.py.",
    )
    parser.add_argument(
        "--detector",
        default=None,
        help="Override every scenario's --detector for dart_runner.py.",
    )
    parser.add_argument(
        "--seed", type=int, default=0, help="Seed for all generated scenes."
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path(".mujoco_comparison_results"),
        help="Directory for per-rep JSON, results.json, and results.md.",
    )
    parser.add_argument(
        "--skip-sensitivity",
        action="store_true",
        help="Skip the MuJoCo model-default-integrator sensitivity row.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the commands that would run without executing them.",
    )
    return parser.parse_args(argv)


def _selected_scenarios(ids: list[str]) -> list[Scenario]:
    all_scenarios = [*SCENARIOS, OVERHEAD_SCENARIO]
    if not ids:
        return [
            scenario
            for scenario in all_scenarios
            if scenario.scene_id in DEFAULT_SCENE_IDS or scenario is OVERHEAD_SCENARIO
        ]
    wanted = {i.upper() for i in ids}
    selected = [s for s in all_scenarios if s.scene_id.upper() in wanted]
    missing = wanted - {s.scene_id.upper() for s in selected}
    if missing:
        raise SystemExit(f"Unknown --scene id(s): {sorted(missing)}")
    return selected


def _dart_command(
    python_exe: str,
    scene_path: Path,
    scenario: Scenario,
    out_path: Path,
    config: str,
    detector_override: Optional[str],
) -> list[str]:
    detector = detector_override or scenario.detector
    cmd = [
        python_exe,
        str(DART_RUNNER),
        "--scene",
        str(scene_path),
        "--steps",
        str(scenario.steps),
        "--warmup",
        str(scenario.warmup),
        "--out",
        str(out_path),
        "--detector",
        detector,
        "--sleep",
        scenario.sleep,
        "--scene-id",
        scenario.scene_id,
        "--config",
        config,
        "--drive",
        "on" if scenario.drive else "off",
    ]
    if scenario.drive:
        cmd += [
            "--drive-amplitude",
            str(scenario.drive_amplitude),
            "--drive-frequency",
            str(scenario.drive_frequency),
            "--drive-seed",
            str(scenario.seed),
        ]
    return cmd


def _mujoco_command(
    python_exe: str,
    scene_path: Path,
    scenario: Scenario,
    out_path: Path,
    config: str,
    integrator: Optional[str],
) -> list[str]:
    cmd = [
        python_exe,
        str(MUJOCO_RUNNER),
        "--scene",
        str(scene_path),
        "--steps",
        str(scenario.steps),
        "--warmup",
        str(scenario.warmup),
        "--out",
        str(out_path),
        "--scene-id",
        scenario.scene_id,
        "--config",
        config,
        "--drive",
        "on" if scenario.drive else "off",
    ]
    if integrator is not None:
        cmd += ["--integrator", integrator]
    if scenario.drive:
        cmd += [
            "--drive-amplitude",
            str(scenario.drive_amplitude),
            "--drive-frequency",
            str(scenario.drive_frequency),
            "--drive-seed",
            str(scenario.seed),
        ]
    return cmd


def _run(cmd: list[str], dry_run: bool) -> bool:
    print(" ".join(cmd))
    if dry_run:
        return True
    # A failing runner (e.g. a scene the engine cannot load yet, such as
    # humanoid MJCF on a DART build without stacked-joint parser support)
    # must not abort the whole matrix; the scenario is reported as blocked
    # and the remaining scenes still run.
    result = subprocess.run(cmd, cwd=REPO_ROOT)
    if result.returncode != 0:
        print(
            f"WARNING: runner exited with {result.returncode}; "
            "marking this scenario blocked and continuing.",
            file=sys.stderr,
        )
        return False
    return True


def _generate_scene_files(
    scenario: Scenario, seed: int, work_dir: Path
) -> tuple[Path, Path]:
    """Return (dart_scene_path, mujoco_scene_path) for a scenario.

    For "mjcf" scenarios both engines read the same file directly. For
    "generated"/"overhead" scenarios a single seeded layout is emitted as a
    JSON body list (dart) and an equivalent MJCF XML string (mujoco), so
    both engines simulate the identical physical scene.
    """
    if scenario.category == "mjcf":
        path = REPO_ROOT / scenario.scene_file
        return path, path

    layout = gen_scenes.build_layout(scenario.generator_scene_id, seed)
    json_path = work_dir / f"{scenario.scene_id}.json"
    xml_path = work_dir / f"{scenario.scene_id}.xml"
    common.save_generated_layout(layout, json_path)
    # The XML is emitted for inspection/debugging only. Both runners consume
    # the spec JSON: mujoco_runner builds its model from the same layout AND
    # reconstructs the StirrerSpec, whereas an .xml input would leave the
    # stirrer as a static mocap body and silently unbalance DYN-* scenes.
    xml_path.write_text(gen_scenes.to_mjcf_xml(layout), encoding="utf-8")
    return json_path, json_path


def _median_row(rows: list[common.ResultRow]) -> dict:
    wall_s = statistics.median(r.wall_s for r in rows)
    ms_per_step = statistics.median(r.ms_per_step for r in rows)
    steps_per_s = statistics.median(r.steps_per_s for r in rows)
    rtf = statistics.median(r.rtf for r in rows)
    ncon_mean = statistics.mean(r.ncon_mean for r in rows)
    ncon_max = max(r.ncon_max for r in rows)
    finite = all(r.finite for r in rows)
    sleeping_values = [r.sleeping_bodies for r in rows]
    sleeping_bodies = (
        statistics.median(sleeping_values)
        if all(v is not None for v in sleeping_values)
        else None
    )
    return {
        "wall_s": wall_s,
        "ms_per_step": ms_per_step,
        "steps_per_s": steps_per_s,
        "rtf": rtf,
        "ncon_mean": ncon_mean,
        "ncon_max": ncon_max,
        "finite": finite,
        "sleeping_bodies": sleeping_bodies,
        "sample_count": len(rows),
    }


def _verdict(dart_steps_per_s: float, mujoco_steps_per_s: float) -> str:
    if mujoco_steps_per_s <= 0.0 or math.isnan(dart_steps_per_s):
        return "inconclusive"
    ratio = dart_steps_per_s / mujoco_steps_per_s
    if ratio > WIN_RATIO:
        return "DART wins"
    if ratio < 1.0 / WIN_RATIO:
        return "DART loses"
    return f"within noise band ({WIN_RATIO}x)"


def _write_markdown(out_dir: Path, summary: dict) -> None:
    lines = ["# DART vs MuJoCo Comparison", ""]
    lines += [
        "| Scene | DART steps/s | MuJoCo steps/s | Ratio (DART/MuJoCo) | Verdict | Finite (D/M) |",
        "| --- | ---: | ---: | ---: | --- | --- |",
    ]
    for scene_id, entry in summary["scenes"].items():
        dart_row = entry.get("dart")
        mujoco_row = entry.get("mujoco")
        if dart_row is None or mujoco_row is None:
            reason = entry.get("verdict", "missing engine row")
            lines.append(f"| `{scene_id}` | n/a | n/a | n/a | {reason} | n/a |")
            continue
        ratio = (
            dart_row["steps_per_s"] / mujoco_row["steps_per_s"]
            if mujoco_row["steps_per_s"]
            else float("nan")
        )
        lines.append(
            "| `{scene}` | {dart_sps:.1f} | {mujoco_sps:.1f} | {ratio:.2f}x | "
            "{verdict} | {dart_finite}/{mujoco_finite} |".format(
                scene=scene_id,
                dart_sps=dart_row["steps_per_s"],
                mujoco_sps=mujoco_row["steps_per_s"],
                ratio=ratio,
                verdict=entry["verdict"],
                dart_finite=dart_row["finite"],
                mujoco_finite=mujoco_row["finite"],
            )
        )

    if summary.get("sensitivity"):
        lines += ["", "## MuJoCo Model-Default-Integrator Sensitivity", ""]
        lines += [
            "| Scene | Euler steps/s | Model-default steps/s |",
            "| --- | ---: | ---: |",
        ]
        for scene_id, entry in summary["sensitivity"].items():
            lines.append(
                f"| `{scene_id}` | {entry['euler_steps_per_s']:.1f} | "
                f"{entry['model_default_steps_per_s']:.1f} |"
            )

    lines += ["", "## Notes", ""]
    lines.append(
        f"- Win/loss band: ratios within [1/{WIN_RATIO}, {WIN_RATIO}] are "
        "reported as noise, not a win or loss."
    )
    lines.append(
        "- `FFI-OVERHEAD` isolates each engine's fixed per-step call "
        "overhead (single body, no contact, no gravity) and is not part of "
        "the win/loss scoring above."
    )
    lines.append(
        "- See README.md for the full fairness contract (timestep/gravity "
        "parity, integrator normalization, single-threaded, warmup "
        "exclusion, median-of-reps)."
    )

    (out_dir / "results.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    scenarios = _selected_scenarios(args.scene)
    # Resolve to an absolute path before use: dart_runner.py/mujoco_runner.py
    # subprocesses run with cwd=REPO_ROOT (see _run()), so a relative --out
    # path from the *caller's* cwd would otherwise get written under
    # REPO_ROOT by the child while this process kept looking for it relative
    # to its own (possibly different) cwd.
    args.out_dir = args.out_dir.resolve()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    raw_rows: dict[str, dict[str, list[common.ResultRow]]] = {}
    sensitivity: dict[str, dict[str, float]] = {}
    blocked: dict[str, str] = {}

    with tempfile.TemporaryDirectory(prefix="mujoco_comparison_") as tmp:
        work_dir = Path(tmp)
        for scenario in scenarios:
            dart_scene, mujoco_scene = _generate_scene_files(
                scenario, args.seed, work_dir
            )
            raw_rows[scenario.scene_id] = {"dart": [], "mujoco": []}

            scenario_blocked = False
            for rep in range(args.reps):
                dart_out = (
                    args.out_dir / "raw" / f"{scenario.scene_id}-dart-rep{rep}.json"
                )
                mujoco_out = (
                    args.out_dir / "raw" / f"{scenario.scene_id}-mujoco-rep{rep}.json"
                )

                dart_cmd = _dart_command(
                    args.dart_python,
                    dart_scene,
                    scenario,
                    dart_out,
                    config="headline",
                    detector_override=args.detector,
                )
                if not _run(dart_cmd, args.dry_run):
                    blocked[scenario.scene_id] = "dart runner failed"
                    scenario_blocked = True
                    break

                mujoco_cmd = _mujoco_command(
                    args.mujoco_python,
                    mujoco_scene,
                    scenario,
                    mujoco_out,
                    config="headline",
                    integrator="euler",
                )
                if not _run(mujoco_cmd, args.dry_run):
                    blocked[scenario.scene_id] = "mujoco runner failed"
                    scenario_blocked = True
                    break

                if not args.dry_run:
                    raw_rows[scenario.scene_id]["dart"].append(
                        common.read_result_row(dart_out)
                    )
                    raw_rows[scenario.scene_id]["mujoco"].append(
                        common.read_result_row(mujoco_out)
                    )
            if scenario_blocked:
                continue

            if (
                scenario.category == "mjcf"
                and not args.skip_sensitivity
                and not args.dry_run
            ):
                sens_out = (
                    args.out_dir
                    / "raw"
                    / f"{scenario.scene_id}-mujoco-sensitivity.json"
                )
                sens_cmd = _mujoco_command(
                    args.mujoco_python,
                    mujoco_scene,
                    scenario,
                    sens_out,
                    config="model-default-integrator",
                    integrator=None,
                )
                if not _run(sens_cmd, args.dry_run):
                    continue
                headline_median = _median_row(raw_rows[scenario.scene_id]["mujoco"])
                sens_row = common.read_result_row(sens_out)
                sensitivity[scenario.scene_id] = {
                    "euler_steps_per_s": headline_median["steps_per_s"],
                    "model_default_steps_per_s": sens_row.steps_per_s,
                }

    if args.dry_run:
        print("Dry run: no results written.")
        return 0

    scenes_summary: dict[str, dict] = {}
    for scenario in scenarios:
        dart_rows = raw_rows[scenario.scene_id]["dart"]
        mujoco_rows = raw_rows[scenario.scene_id]["mujoco"]
        dart_median = _median_row(dart_rows) if dart_rows else None
        mujoco_median = _median_row(mujoco_rows) if mujoco_rows else None
        if scenario.scene_id in blocked:
            verdict = f"blocked: {blocked[scenario.scene_id]}"
        elif dart_median is None or mujoco_median is None:
            verdict = "missing engine row"
        elif scenario.category == "overhead":
            # FFI-OVERHEAD is a reference measurement (per-step call
            # overhead), not one of the scored scenes; see README.md.
            verdict = "not scored (overhead reference)"
        elif not dart_median["finite"] or not mujoco_median["finite"]:
            # A physically invalid run must never receive a performance
            # verdict; the fairness contract scores plausible runs only.
            verdict = "inconclusive (non-finite run)"
        else:
            verdict = _verdict(dart_median["steps_per_s"], mujoco_median["steps_per_s"])
        scenes_summary[scenario.scene_id] = {
            "category": scenario.category,
            "dart": dart_median,
            "mujoco": mujoco_median,
            "verdict": verdict,
        }

    summary = {
        "reps": args.reps,
        "seed": args.seed,
        "detector_override": args.detector,
        "scenes": scenes_summary,
        "sensitivity": sensitivity,
        "raw_rows": {
            scene_id: {
                engine: [row.to_dict() for row in rows]
                for engine, rows in engines.items()
            }
            for scene_id, engines in raw_rows.items()
        },
    }
    (args.out_dir / "results.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    _write_markdown(args.out_dir, summary)

    print(args.out_dir / "results.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
