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


def _papers_legend():
    return """### Property legend

| Property | Values | Meaning |
| --- | --- | --- |
| **Type** | `textbook`, `paper`, `standard`, `engine` | Kind. |
| **Status** | `referenced`, `planned`, `in-progress`, `implemented`, `deferred`, `rejected` | Relationship. |
| **Priority** | `high`, `medium`, `low`, `—` | Importance. |
| **Verdict** | `adopt`, `baseline`, `reference`, `evaluate`, `reject` | Decision. |
"""


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


def test_complete_plan_pointing_to_numbered_file_is_rejected(tmp_path):
    module = _load_module()
    plans = tmp_path / "docs" / "plans"
    plans.mkdir(parents=True)
    (plans / "001-finished.md").write_text("# Finished\n", encoding="utf-8")
    (plans / "dashboard.md").write_text(
        """### PLAN-001: Finished

- Owner doc: [`001-finished.md`](001-finished.md)
- Status: Complete
- Horizon: Now
- Dimension: AI-native execution
- Next step: None.
- Gate: `pixi run check-docs-policy`
""",
        encoding="utf-8",
    )

    failures = module.check_plan_lifecycle(tmp_path)

    assert any(
        "completed PLAN-001 still points to numbered plan file" in failure
        for failure in failures
    )


def test_unreferenced_plan_file_is_reported(tmp_path):
    module = _load_module()
    plans = tmp_path / "docs" / "plans"
    ai = tmp_path / "docs" / "ai"
    plans.mkdir(parents=True)
    ai.mkdir(parents=True)
    (ai / "principles.md").write_text("# Principles\n", encoding="utf-8")
    (plans / "002-orphan.md").write_text("# Orphan\n", encoding="utf-8")
    (plans / "dashboard.md").write_text(
        """### PLAN-002: Active

- Owner doc: [`../ai/principles.md`](../ai/principles.md)
- Status: Active
- Horizon: Now
- Dimension: AI-native execution
- Next step: Keep policy current.
- Gate: `pixi run check-docs-policy`
""",
        encoding="utf-8",
    )

    failures = module.check_plan_lifecycle(tmp_path)

    assert any(
        "docs/plans/002-orphan.md: numbered plan file is not referenced" in failure
        for failure in failures
    )


def test_plan_file_repeating_dashboard_field_is_rejected(tmp_path):
    module = _load_module()
    plans = tmp_path / "docs" / "plans"
    plans.mkdir(parents=True)
    (plans / "003-active.md").write_text(
        "# Active\n\n- Status: Active\n", encoding="utf-8"
    )
    (plans / "dashboard.md").write_text(
        """### PLAN-003: Active

- Owner doc: [`003-active.md`](003-active.md)
- Status: Active
- Horizon: Now
- Dimension: AI-native execution
- Next step: Keep policy current.
- Gate: `pixi run check-docs-policy`
""",
        encoding="utf-8",
    )

    failures = module.check_plan_lifecycle(tmp_path)

    assert any(
        "docs/plans/003-active.md:3: plan file repeats dashboard field `Status`"
        in failure
        for failure in failures
    )


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


def test_report_only_link_check_ignores_valid_internal_link(tmp_path):
    module = _load_module()
    docs = tmp_path / "docs"
    docs.mkdir()
    (docs / "target.md").write_text("# Target\n", encoding="utf-8")
    (docs / "README.md").write_text("[target](target.md)\n", encoding="utf-8")

    warnings = module.check_markdown_internal_links(tmp_path)

    assert warnings == []


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


def test_docs_ai_frontmatter_normalizes_quoted_owner_self(tmp_path):
    module = _load_module()
    ai_dir = tmp_path / "docs" / "ai"
    ai_dir.mkdir(parents=True)
    for filename in module.DOCS_AI_FRONTMATTER_FILES:
        owner = '"self"' if filename == "principles.md" else "self"
        (ai_dir / filename).write_text(
            f"---\ntype: ai-policy\nowner: {owner}\n---\n# Title\n",
            encoding="utf-8",
        )

    assert module.check_ai_doc_frontmatter(tmp_path) == []


def test_docs_ai_frontmatter_resolves_owner_links(tmp_path):
    module = _load_module()
    ai_dir = tmp_path / "docs" / "ai"
    ai_dir.mkdir(parents=True)
    (ai_dir / "README.md").write_text("# AI\n", encoding="utf-8")
    for filename in module.DOCS_AI_FRONTMATTER_FILES:
        owner = "README.md" if filename == "principles.md" else "self"
        (ai_dir / filename).write_text(
            f"---\ntype: ai-policy\nowner: {owner}\n---\n# Title\n",
            encoding="utf-8",
        )

    assert module.check_ai_doc_frontmatter(tmp_path) == []


def test_docs_ai_frontmatter_roster_rejects_unlisted_markdown_doc(tmp_path):
    module = _load_module()
    ai_dir = tmp_path / "docs" / "ai"
    ai_dir.mkdir(parents=True)
    for filename in module.DOCS_AI_FRONTMATTER_FILES:
        (ai_dir / filename).write_text(
            "---\ntype: ai-policy\nowner: self\n---\n# Title\n",
            encoding="utf-8",
        )
    (ai_dir / "extra.md").write_text(
        "---\ntype: ai-policy\nowner: self\n---\n# Extra\n",
        encoding="utf-8",
    )

    failures = module.check_ai_doc_frontmatter(tmp_path)

    assert any(
        "docs/ai/extra.md: docs/ai frontmatter pilot roster is missing" in failure
        for failure in failures
    )


def test_docs_discoverability_reports_unlinked_ai_doc(tmp_path):
    module = _load_module()
    ai_dir = tmp_path / "docs" / "ai"
    ai_dir.mkdir(parents=True)
    (ai_dir / "README.md").write_text("# AI\n", encoding="utf-8")
    (ai_dir / "principles.md").write_text("# Principles\n", encoding="utf-8")

    warnings = module.check_docs_discoverability(tmp_path)

    assert any(
        "docs/ai/principles.md: not linked from `docs/ai/README.md`" in w
        for w in warnings
    )


def test_papers_catalog_validator_checks_summary_detail_parity_and_values(tmp_path):
    module = _load_module()
    papers = tmp_path / "docs" / "readthedocs" / "papers.md"
    papers.parent.mkdir(parents=True)
    papers.write_text(
        f"""# Papers

{_papers_legend()}

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
        f"""# Papers

{_papers_legend()}

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


def test_papers_catalog_validator_uses_property_legend_values(tmp_path):
    module = _load_module()
    papers = tmp_path / "docs" / "readthedocs" / "papers.md"
    papers.parent.mkdir(parents=True)
    papers.write_text(
        """# Papers

### Property legend

| Property | Values | Meaning |
| --- | --- | --- |
| **Type** | `paper` | Kind. |
| **Status** | `planned` | Relationship. |
| **Priority** | `high` | Importance. |
| **Verdict** | `adopt` | Decision. |

| ID | Reference | Topic | Status | Priority | Verdict |
| --- | --- | --- | --- | --- | --- |
| `paper-1` | Example | contact | implemented | high | adopt |

### `paper-1`

- **Type:** paper · **Topic:** contact · **Status:** implemented · **Priority:** high · **Verdict:** adopt
""",
        encoding="utf-8",
    )

    failures = module.check_papers_catalog(tmp_path)

    assert any("`paper-1` has invalid Status `implemented`" in f for f in failures)


def test_papers_catalog_validator_checks_duplicate_detail_and_property_line(tmp_path):
    module = _load_module()
    papers = tmp_path / "docs" / "readthedocs" / "papers.md"
    papers.parent.mkdir(parents=True)
    papers.write_text(
        f"""# Papers

{_papers_legend()}

| ID | Reference | Topic | Status | Priority | Verdict |
| --- | --- | --- | --- | --- | --- |
| `paper-1` | Example | contact | implemented | high | adopt |

### `paper-1`

- Missing property line.

### `paper-1`

- **Type:** paper · **Topic:** contact · **Status:** implemented · **Priority:** high · **Verdict:** adopt
""",
        encoding="utf-8",
    )

    failures = module.check_papers_catalog(tmp_path)

    assert any("duplicate detail entry `paper-1`" in f for f in failures)
    assert any(
        "detail entry `paper-1` is missing the Type/Topic/Status/Priority/Verdict" in f
        for f in failures
    )


def test_papers_catalog_detail_values_allow_code_ticks(tmp_path):
    module = _load_module()
    papers = tmp_path / "docs" / "readthedocs" / "papers.md"
    papers.parent.mkdir(parents=True)
    papers.write_text(
        f"""# Papers

{_papers_legend()}

| ID | Reference | Topic | Status | Priority | Verdict |
| --- | --- | --- | --- | --- | --- |
| `paper-1` | Example | contact | implemented | high | adopt |

### `paper-1`

- **Type:** `paper` · **Topic:** contact · **Status:** `implemented` · **Priority:** `high` · **Verdict:** `adopt`
""",
        encoding="utf-8",
    )

    assert module.check_papers_catalog(tmp_path) == []


def test_north_star_freshness_warns_when_marker_missing(tmp_path):
    module = _load_module()
    north_star = tmp_path / "docs" / "ai" / "north-star.md"
    north_star.parent.mkdir(parents=True)
    north_star.write_text("# North Star\n", encoding="utf-8")

    warnings = module.check_north_star_evidence_freshness(tmp_path)

    assert any(
        "missing advisory evidence freshness marker" in warning for warning in warnings
    )


def test_north_star_freshness_warns_for_newer_committed_evidence(tmp_path, monkeypatch):
    module = _load_module()
    north_star = tmp_path / "docs" / "ai" / "north-star.md"
    evidence = tmp_path / "docs" / "ai" / "principles.md"
    north_star.parent.mkdir(parents=True)
    evidence.write_text("# Principles\n", encoding="utf-8")
    north_star.write_text(
        """<!-- docs-policy: evidence-last-verified=2026-01-01 -->

# North Star

## Current State

| Evidence |
| --- |
| `docs/ai/principles.md` |

## What Is Missing
""",
        encoding="utf-8",
    )
    monkeypatch.setattr(
        module, "_git_last_commit_date", lambda repo_root, path: "2026-01-02"
    )

    warnings = module.check_north_star_evidence_freshness(tmp_path)

    assert any(
        "`docs/ai/principles.md` changed on 2026-01-02" in warning
        for warning in warnings
    )
