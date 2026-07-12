#!/usr/bin/env python3
"""Forward native-Windows Codex hook input to DART's shared Git Bash guard."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Optional


def repository_root(requested: Optional[Path] = None) -> Path:
    if requested is not None:
        return requested.resolve()
    result = subprocess.run(
        ["git", "rev-parse", "--show-toplevel"],
        check=True,
        capture_output=True,
        text=True,
    )
    return Path(result.stdout.strip()).resolve()


def validate_payload(payload: bytes) -> None:
    try:
        data = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError("invalid hook JSON: {}".format(exc)) from exc
    tool_input = data.get("tool_input") if isinstance(data, dict) else None
    command = tool_input.get("command") if isinstance(tool_input, dict) else None
    if not isinstance(command, str):
        raise ValueError("hook JSON must contain string tool_input.command")


def find_git_bash() -> Optional[Path]:
    candidates = []
    git = shutil.which("git")
    if git:
        git_dir = Path(git).resolve().parent
        git_root = git_dir.parent
        candidates.extend(
            (
                git_dir / "bash.exe",
                git_root / "bin" / "bash.exe",
                git_root / "usr" / "bin" / "bash.exe",
            )
        )
    for variable in ("ProgramFiles", "ProgramFiles(x86)", "LOCALAPPDATA"):
        base = os.environ.get(variable)
        if base:
            candidates.extend(
                (
                    Path(base) / "Git" / "bin" / "bash.exe",
                    Path(base) / "Programs" / "Git" / "bin" / "bash.exe",
                )
            )
    bash = shutil.which("bash")
    if bash:
        if os.name == "nt":
            candidates.append(Path(bash))
        else:
            candidates.insert(0, Path(bash))
    return next((path for path in candidates if path.is_file()), None)


def run_guard(root: Path, payload: bytes, bash: Path) -> int:
    guard = root / ".claude" / "hooks" / "pre-commit-guard.sh"
    if not guard.is_file():
        print("DART pre-tool hook: missing {}".format(guard), file=sys.stderr)
        return 2
    env = dict(os.environ)
    env["CLAUDE_PROJECT_DIR"] = str(root)
    env["CODEX_PROJECT_DIR"] = str(root)
    env["DART_HOOK_PYTHON"] = str(Path(sys.executable).resolve()).replace("\\", "/")
    result = subprocess.run(
        [
            str(bash),
            "--noprofile",
            "--norc",
            str(guard).replace("\\", "/"),
        ],
        cwd=root,
        env=env,
        input=payload,
        capture_output=True,
    )
    if result.stdout:
        sys.stderr.write(result.stdout.decode(errors="replace"))
    if result.stderr:
        sys.stderr.write(result.stderr.decode(errors="replace"))
    return 0 if result.returncode == 0 else 2


def forward(root: Path, payload: bytes) -> int:
    try:
        validate_payload(payload)
    except ValueError as exc:
        print("DART pre-tool hook: {}".format(exc), file=sys.stderr)
        return 2
    bash = find_git_bash()
    if bash is None:
        print(
            "DART pre-tool hook: Git Bash is required for the shared commit guard",
            file=sys.stderr,
        )
        return 2
    return run_guard(root, payload, bash)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    try:
        root = repository_root(args.root)
        payload = sys.stdin.buffer.read()
    except (OSError, subprocess.SubprocessError, ValueError) as exc:
        print("DART pre-tool hook: {}".format(exc), file=sys.stderr)
        return 2
    return forward(root, payload)


if __name__ == "__main__":
    raise SystemExit(main())
