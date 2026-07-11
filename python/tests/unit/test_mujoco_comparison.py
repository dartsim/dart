import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
RUNNER = ROOT / "scripts" / "mujoco_comparison" / "run_comparison.py"


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
