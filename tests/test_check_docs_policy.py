"""Tests for the plan-ID uniqueness check in scripts/check_docs_policy.py.

PLAN-091 WP-091.5.
"""

import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "check_docs_policy.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("check_docs_policy", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _entry(plan_id):
    return {
        "id": plan_id,
        "status": "",
        "horizon": "",
        "dimension": "",
        "next_step": "",
        "gate": "",
        "owner": "",
    }


def test_unique_ids_pass():
    module = _load_module()
    entries = [_entry("PLAN-080"), _entry("PLAN-082"), _entry("PLAN-090")]
    assert module.check_plan_id_uniqueness(entries) == []


def test_duplicate_id_is_rejected():
    module = _load_module()
    entries = [_entry("PLAN-080"), _entry("PLAN-082"), _entry("PLAN-080")]
    failures = module.check_plan_id_uniqueness(entries)
    assert any("PLAN-080 identifies 2 dashboard blocks" in f for f in failures)


def test_two_distinct_collisions_are_both_reported():
    module = _load_module()
    entries = [
        _entry("PLAN-080"),
        _entry("PLAN-082"),
        _entry("PLAN-080"),
        _entry("PLAN-082"),
    ]
    failures = module.check_plan_id_uniqueness(entries)
    assert any("PLAN-080 identifies 2" in f for f in failures)
    assert any("PLAN-082 identifies 2" in f for f in failures)


def test_uniqueness_parses_real_dashboard_block_format():
    module = _load_module()
    text = (
        "### PLAN-080: First\n\n- Status: Active\n\n"
        "### PLAN-080: Second\n\n- Status: Active\n"
    )
    entries = module._dashboard_entries(text)
    ids = [e["id"] for e in entries]
    assert ids == ["PLAN-080", "PLAN-080"]
    assert module.check_plan_id_uniqueness(entries)
