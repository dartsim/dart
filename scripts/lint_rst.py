"""Run sphinx-lint on repository RST files while skipping build outputs."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def collect_rst_files(root: Path) -> list[str]:
    paths: list[str] = []
    for path in root.rglob("*.rst"):
        if any(part == "_build" for part in path.parts):
            continue
        paths.append(str(path))
    return sorted(paths)


def main() -> int:
    repo_root = Path(__file__).resolve().parent.parent
    docs_root = repo_root / "docs"
    rst_files = collect_rst_files(docs_root)
    if not rst_files:
        return 0
    result = subprocess.run(["sphinx-lint", *rst_files], check=False)
    return result.returncode


if __name__ == "__main__":
    sys.exit(main())
