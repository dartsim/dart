#!/usr/bin/env python3
from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path


def _resolve_example(args: list[str]) -> tuple[str, list[str]]:
    """Return (example, remaining_args) picking the first non-flag arg or EXAMPLE env."""
    remaining = list(args)
    if remaining and not remaining[0].startswith("-"):
        example = remaining.pop(0)
    else:
        example = os.environ.get("EXAMPLE", "hello_world")
    while remaining and remaining[0] == "--":
        remaining.pop(0)
    return example, remaining


def main(argv: list[str]) -> int:
    repo_root = Path(__file__).resolve().parent.parent
    example, remaining = _resolve_example(argv)

    script = repo_root / "python" / "examples" / example / "main.py"
    if not script.exists():
        sys.stderr.write(f"Python example not found: {script}\n")
        return 1

    build_type = os.environ.get("BUILD_TYPE", "Release")
    env_name = os.environ.get("PIXI_ENVIRONMENT_NAME", "default")
    py_path = repo_root / "build" / env_name / "cpp" / build_type / "python"

    env = os.environ.copy()
    existing = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = f"{py_path}:{existing}" if existing else str(py_path)
    # Ensure conda libs (glfw, imgui, OpenGL) are ahead of system paths.
    conda_prefix = env.get("CONDA_PREFIX")
    if conda_prefix:
        ld_paths = env.get("LD_LIBRARY_PATH", "")
        env["LD_LIBRARY_PATH"] = (
            f"{conda_prefix}/lib:{ld_paths}" if ld_paths else f"{conda_prefix}/lib"
        )
        preload = env.get("LD_PRELOAD", "")
        glfw_lib = f"{conda_prefix}/lib/libglfw.so"
        env["LD_PRELOAD"] = f"{glfw_lib}:{preload}" if preload else glfw_lib

    cmd = [sys.executable, str(script), *remaining]
    return subprocess.call(cmd, env=env, cwd=repo_root)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
