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


def test_markdown_link_resolver_handles_repo_root_relative_and_anchor(tmp_path):
    module = _load_module()
    repo = tmp_path
    docs = repo / "docs"
    docs.mkdir()
    current = docs / "README.md"
    current.write_text("# Docs\n", encoding="utf-8")
    target = docs / "ai" / "README.md"
    target.parent.mkdir()
    target.write_text("# AI\n", encoding="utf-8")

    assert (
        module._resolve_markdown_link(
            "docs/ai/README.md", docs, repo_root=repo, current_file=current
        )
        == target.resolve()
    )
    assert (
        module._resolve_markdown_link(
            "#local-heading", docs, repo_root=repo, current_file=current
        )
        == current.resolve()
    )
    assert module._resolve_markdown_link("https://example.com", docs, repo) is None


def test_report_only_link_check_detects_broken_internal_link(tmp_path):
    module = _load_module()
    docs = tmp_path / "docs"
    docs.mkdir()
    (docs / "README.md").write_text("[missing](missing.md)\n", encoding="utf-8")

    warnings = module.check_markdown_internal_links(tmp_path)

    assert any("broken internal link `missing.md`" in warning for warning in warnings)


def test_report_only_link_check_includes_top_level_docs_indexes(tmp_path, monkeypatch):
    module = _load_module()
    docs = tmp_path / "docs"
    docs.mkdir()
    readme = docs / "README.md"
    readme.write_text("[missing](missing.md)\n", encoding="utf-8")
    requested_patterns = []

    def fake_iter_tracked_files(repo_root, patterns):
        requested_patterns.extend(patterns)
        return [readme] if ":(glob)docs/*.md" in patterns else []

    monkeypatch.setattr(module, "iter_tracked_files", fake_iter_tracked_files)

    warnings = module.check_markdown_internal_links(tmp_path)

    assert ":(glob)docs/*.md" in requested_patterns
    assert any("broken internal link `missing.md`" in warning for warning in warnings)


def test_docs_ai_frontmatter_pilot_requires_type_and_owner(tmp_path):
    module = _load_module()
    ai_dir = tmp_path / "docs" / "ai"
    ai_dir.mkdir(parents=True)
    for filename in module.DOCS_AI_FRONTMATTER_FILES:
        (ai_dir / filename).write_text(
            "---\ntype: ai-policy\nowner: self\n---\n# Title\n",
            encoding="utf-8",
        )

    assert module.check_ai_doc_frontmatter(tmp_path) == []

    (ai_dir / "principles.md").write_text(
        "---\ntype: ai-policy\n---\n# Principles\n",
        encoding="utf-8",
    )
    failures = module.check_ai_doc_frontmatter(tmp_path)
    assert any(
        "principles.md: missing frontmatter field `owner`" in f for f in failures
    )


def test_docs_ai_frontmatter_rejects_extra_mutable_fields(tmp_path):
    module = _load_module()
    ai_dir = tmp_path / "docs" / "ai"
    ai_dir.mkdir(parents=True)
    for filename in module.DOCS_AI_FRONTMATTER_FILES:
        (ai_dir / filename).write_text(
            "---\ntype: ai-policy\nowner: self\n---\n# Title\n",
            encoding="utf-8",
        )
    (ai_dir / "north-star.md").write_text(
        "---\ntype: ai-policy\nowner: self\nstatus: active\n---\n# North Star\n",
        encoding="utf-8",
    )

    failures = module.check_ai_doc_frontmatter(tmp_path)

    assert any(
        "unsupported docs/ai frontmatter field(s): `status`" in f for f in failures
    )


def test_papers_catalog_validator_checks_summary_detail_parity_and_values(tmp_path):
    module = _load_module()
    papers = tmp_path / "docs" / "readthedocs" / "papers.md"
    papers.parent.mkdir(parents=True)
    papers.write_text(
        """# Papers

| ID | Reference | Topic | Status | Priority | Verdict |
| --- | --- | --- | --- | --- | --- |
| `paper-1` | Example | contact | implemented | high | adopt |
| `paper-2` | Missing detail | contact | unknown | high | adopt |

### `paper-1`

- **Type:** paper · **Topic:** contact · **Status:** implemented · **Priority:** high · **Verdict:** adopt
- **Where used:** [missing](missing.md)
""",
        encoding="utf-8",
    )

    failures = module.check_papers_catalog(tmp_path)

    assert any("`paper-2` has invalid Status `unknown`" in f for f in failures)
    assert any("summary entry `paper-2` has no detail block" in f for f in failures)
    assert any("`paper-1` has broken local link `missing.md`" in f for f in failures)


def test_papers_catalog_validator_checks_summary_detail_field_parity(tmp_path):
    module = _load_module()
    papers = tmp_path / "docs" / "readthedocs" / "papers.md"
    papers.parent.mkdir(parents=True)
    papers.write_text(
        """# Papers

| ID | Reference | Topic | Status | Priority | Verdict |
| --- | --- | --- | --- | --- | --- |
| `paper-1` | Example | contact | planned | high | adopt |

### `paper-1`

- **Type:** paper · **Topic:** contact · **Status:** implemented · **Priority:** high · **Verdict:** reference
""",
        encoding="utf-8",
    )

    failures = module.check_papers_catalog(tmp_path)

    assert any(
        "`paper-1` has mismatched Status: summary `planned`, detail `implemented`"
        in failure
        for failure in failures
    )
    assert any(
        "`paper-1` has mismatched Verdict: summary `adopt`, detail `reference`"
        in failure
        for failure in failures
    )


def test_north_star_freshness_warns_when_marker_missing(tmp_path):
    module = _load_module()
    north_star = tmp_path / "docs" / "ai" / "north-star.md"
    north_star.parent.mkdir(parents=True)
    north_star.write_text("# North Star\n", encoding="utf-8")

    warnings = module.check_north_star_evidence_freshness(tmp_path)

    assert any(
        "missing advisory evidence freshness marker" in warning for warning in warnings
    )
