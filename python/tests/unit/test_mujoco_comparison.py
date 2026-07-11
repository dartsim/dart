import importlib.util
import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
RUNNER = ROOT / "scripts" / "mujoco_comparison" / "run_comparison.py"


def _load_runner_module():
    script_dir = str(RUNNER.parent)
    if script_dir not in sys.path:
        sys.path.insert(0, script_dir)
    spec = importlib.util.spec_from_file_location("mujoco_run_comparison", RUNNER)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_dart_sleep_override_disables_deactivation_for_selected_rows(tmp_path):
    result = subprocess.run(
        [
            sys.executable,
            str(RUNNER),
            "--scene",
            "ARM-REACHER",
            "--scene",
            "HUM-ACTIVE",
            "--reps",
            "1",
            "--dart-sleep",
            "off",
            "--out-dir",
            str(tmp_path),
            "--dry-run",
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )

    dart_commands = [
        line for line in result.stdout.splitlines() if "dart_runner.py" in line
    ]
    assert len(dart_commands) == 2
    assert all("--sleep off" in command for command in dart_commands)
    assert all("--sleep on" not in command for command in dart_commands)


def test_sleep_override_is_serialized_and_rendered_for_blocked_rows(tmp_path):
    runner = _load_runner_module()
    summary = {
        "detector_override": "native",
        "dart_sleep_override": "off",
        "scenes": {
            "HUM-ACTIVE": {
                "category": "mjcf",
                "dart": None,
                "mujoco": None,
                "verdict": "blocked: dart runner failed",
            }
        },
        "sensitivity": {},
    }

    serialized = json.loads(json.dumps(summary))
    assert serialized["dart_sleep_override"] == "off"

    runner._write_markdown(tmp_path, summary)
    markdown = (tmp_path / "results.md").read_text(encoding="utf-8")
    assert "DART detector override: `native`" in markdown
    assert "DART sleep override: `off` for every selected scenario" in markdown
    assert "blocked: dart runner failed" in markdown
