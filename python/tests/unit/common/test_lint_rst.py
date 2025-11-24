from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "scripts"))
import lint_rst  # type: ignore  # noqa: E402


def test_collect_rst_files_skips_build_dirs(tmp_path: Path) -> None:
    docs = tmp_path / "docs"
    include = docs / "keep.rst"
    nested = docs / "sub" / "nested.rst"
    skip = docs / "_build" / "skip.rst"

    include.parent.mkdir(parents=True, exist_ok=True)
    nested.parent.mkdir(parents=True, exist_ok=True)
    skip.parent.mkdir(parents=True, exist_ok=True)

    include.write_text("ok\n")
    nested.write_text("nested\n")
    skip.write_text("skip\n")

    results = [Path(p).resolve() for p in lint_rst.collect_rst_files(docs)]

    assert include.resolve() in results
    assert nested.resolve() in results
    assert all("_build" not in path.parts for path in results)
