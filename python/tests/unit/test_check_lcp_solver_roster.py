from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "check_lcp_solver_roster.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("check_lcp_solver_roster", SCRIPT)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_lcp_solver_roster_surfaces_match() -> None:
    module = _load_module()

    module.check_roster()
