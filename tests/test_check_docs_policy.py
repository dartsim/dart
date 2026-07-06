"""Tests for the plan-ID uniqueness check in scripts/check_docs_policy.py.

PLAN-091 WP-091.5.
"""

import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "check_docs_policy.py"


def test_docs_policy_uses_portable_multi_exception_syntax():
    source = SCRIPT.read_text(encoding="utf-8")

    assert "except OSError, subprocess.CalledProcessError" not in source
    assert "GIT_QUERY_ERRORS = (OSError, subprocess.CalledProcessError)" in source
    assert source.count("except GIT_QUERY_ERRORS:") == 2


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
    assert any("PLAN-080 identifies 2 plan blocks" in f for f in failures)


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


def _dashboard_entry(status="Active", next_step="Do the next thing.", extra_gate=0):
    gate_lines = "".join(f"\n  gate detail line {i}" for i in range(extra_gate))
    return (
        "### PLAN-001: Example\n\n"
        "- Owner doc: [`001-example.md`](001-example.md)\n"
        f"- Status: {status}\n"
        "- Horizon: Now\n"
        "- Dimension: AI-native execution\n"
        f"- Next step: {next_step}\n"
        f"- Gate: `pixi run check-docs-policy`{gate_lines}\n"
    )


def test_bounded_dashboard_entry_passes(tmp_path):
    module = _load_module()
    plans = tmp_path / "docs" / "plans"
    plans.mkdir(parents=True)
    (plans / "dashboard.md").write_text(_dashboard_entry(), encoding="utf-8")

    assert module.check_dashboard_structure(tmp_path) == []


def test_oversized_dashboard_entry_is_rejected(tmp_path):
    module = _load_module()
    plans = tmp_path / "docs" / "plans"
    plans.mkdir(parents=True)
    (plans / "dashboard.md").write_text(
        _dashboard_entry(extra_gate=40), encoding="utf-8"
    )

    failures = module.check_dashboard_structure(tmp_path)

    assert any("PLAN-001 entry is" in f and "at most 40 lines" in f for f in failures)


def test_oversized_next_step_is_rejected(tmp_path):
    module = _load_module()
    plans = tmp_path / "docs" / "plans"
    plans.mkdir(parents=True)
    long_next_step = "First line." + "".join(
        f"\n  continuation line {i}" for i in range(20)
    )
    (plans / "dashboard.md").write_text(
        _dashboard_entry(next_step=long_next_step), encoding="utf-8"
    )

    failures = module.check_dashboard_structure(tmp_path)

    assert any(
        "PLAN-001 `Next step` field is" in f and "at most 15 lines" in f
        for f in failures
    )


def test_dashboard_complete_entry_is_rejected(tmp_path):
    module = _load_module()
    plans = tmp_path / "docs" / "plans"
    plans.mkdir(parents=True)
    (plans / "dashboard.md").write_text(
        _dashboard_entry(status="Complete"), encoding="utf-8"
    )

    failures = module.check_dashboard_structure(tmp_path)

    assert any(
        "PLAN-001 has `Status: Complete`" in f and "archive.md" in f for f in failures
    )


def test_archive_entry_requires_final_status_complete(tmp_path):
    module = _load_module()
    plans = tmp_path / "docs" / "plans"
    plans.mkdir(parents=True)
    (plans / "archive.md").write_text(
        "### PLAN-001: Example\n\n"
        "**Owner doc:** [`001-example.md`](001-example.md)\n\n"
        "**Outcome:** Shipped.\n",
        encoding="utf-8",
    )

    failures = module.check_plan_archive_shape(tmp_path)

    assert any(
        "PLAN-001 is missing the `**Final status:** Complete` marker" in f
        for f in failures
    )


def test_archive_entry_with_final_status_passes(tmp_path):
    module = _load_module()
    plans = tmp_path / "docs" / "plans"
    plans.mkdir(parents=True)
    (plans / "archive.md").write_text(
        "### PLAN-001: Example\n\n"
        "**Final status:** Complete (archived 2026-07-03).\n\n"
        "**Owner doc:** [`001-example.md`](001-example.md)\n\n"
        "**Outcome:** Shipped.\n",
        encoding="utf-8",
    )

    assert module.check_plan_archive_shape(tmp_path) == []


def test_progress_log_prose_does_not_trip_repeats_field(tmp_path):
    module = _load_module()
    plans = tmp_path / "docs" / "plans"
    plans.mkdir(parents=True)
    (plans / "003-active.md").write_text(
        "# Active\n\n## Progress log\n\n"
        "Relocated from the dashboard on 2026-07-03; newest first.\n\n"
        "The next step is to keep the gate green; Status and Horizon stay in the "
        "dashboard, not here.\n",
        encoding="utf-8",
    )
    (plans / "dashboard.md").write_text(
        """### PLAN-003: Active

- Owner doc: [`003-active.md`](003-active.md)
- Status: Active
- Horizon: Now
- Dimension: AI-native execution
- Next step: Keep policy current. History: see [`003-active.md`](003-active.md).
- Gate: `pixi run check-docs-policy`
""",
        encoding="utf-8",
    )

    failures = module.check_plan_lifecycle(tmp_path)

    assert not any("repeats dashboard field" in f for f in failures)


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


def test_docs_information_architecture_owner_is_required(tmp_path):
    module = _load_module()
    docs = tmp_path / "docs"
    docs.mkdir()
    (tmp_path / "AGENTS.md").write_text("# Agents\n", encoding="utf-8")
    (docs / "README.md").write_text("# Docs\n", encoding="utf-8")
    (docs / "AGENTS.md").write_text("# docs/\n", encoding="utf-8")

    failures = module.check_docs_information_architecture(tmp_path)

    assert any("missing docs placement owner" in failure for failure in failures)


def test_docs_information_architecture_must_be_linked(tmp_path):
    module = _load_module()
    docs = tmp_path / "docs"
    docs.mkdir()
    (docs / "information-architecture.md").write_text("# IA\n", encoding="utf-8")
    (tmp_path / "AGENTS.md").write_text(
        "See docs/information-architecture.md\n", encoding="utf-8"
    )
    (docs / "README.md").write_text("# Docs\n", encoding="utf-8")
    (docs / "AGENTS.md").write_text(
        "See docs/information-architecture.md\n", encoding="utf-8"
    )

    failures = module.check_docs_information_architecture(tmp_path)

    assert any("docs/README.md: missing" in failure for failure in failures)
    assert not any("docs/AGENTS.md: missing" in failure for failure in failures)


def test_docs_information_architecture_linked_owner_passes(tmp_path):
    module = _load_module()
    docs = tmp_path / "docs"
    docs.mkdir()
    link = "docs/information-architecture.md"
    (docs / "information-architecture.md").write_text("# IA\n", encoding="utf-8")
    (tmp_path / "AGENTS.md").write_text(f"See {link}\n", encoding="utf-8")
    (docs / "README.md").write_text(f"See {link}\n", encoding="utf-8")
    (docs / "AGENTS.md").write_text(f"See {link}\n", encoding="utf-8")

    assert module.check_docs_information_architecture(tmp_path) == []


def test_docs_information_architecture_root_pointer_is_required(tmp_path):
    module = _load_module()
    docs = tmp_path / "docs"
    docs.mkdir()
    link = "docs/information-architecture.md"
    (docs / "information-architecture.md").write_text("# IA\n", encoding="utf-8")
    (tmp_path / "AGENTS.md").write_text("# Agents\n", encoding="utf-8")
    (docs / "README.md").write_text(f"See {link}\n", encoding="utf-8")
    (docs / "AGENTS.md").write_text(f"See {link}\n", encoding="utf-8")

    failures = module.check_docs_information_architecture(tmp_path)

    assert any("AGENTS.md: missing" in failure for failure in failures)


def test_docs_information_architecture_root_pointer_must_be_repo_relative(tmp_path):
    module = _load_module()
    docs = tmp_path / "docs"
    docs.mkdir()
    link = "docs/information-architecture.md"
    (docs / "information-architecture.md").write_text("# IA\n", encoding="utf-8")
    (tmp_path / "AGENTS.md").write_text(
        "See information-architecture.md\n", encoding="utf-8"
    )
    (docs / "README.md").write_text(f"See {link}\n", encoding="utf-8")
    (docs / "AGENTS.md").write_text(f"See {link}\n", encoding="utf-8")

    failures = module.check_docs_information_architecture(tmp_path)

    assert any("AGENTS.md: missing" in failure for failure in failures)


def test_docs_ai_frontmatter_pilot_requires_type_and_owner(tmp_path):
    module = _load_module()
    ai_dir = tmp_path / "docs" / "ai"
    ai_dir.mkdir(parents=True)
    for filename in module.DOCS_AI_FRONTMATTER_FILES:
        (ai_dir / filename).write_text(
            "---\ntype: ai-principles\nowner: self\n---\n# Title\n",
            encoding="utf-8",
        )

    assert module.check_ai_doc_frontmatter(tmp_path) == []

    (ai_dir / "principles.md").write_text(
        "---\ntype: ai-principles\n---\n# Principles\n",
        encoding="utf-8",
    )
    failures = module.check_ai_doc_frontmatter(tmp_path)
    assert any(
        "principles.md: missing frontmatter field `owner`" in f for f in failures
    )


def test_docs_ai_frontmatter_rejects_out_of_legend_type(tmp_path):
    module = _load_module()
    ai_dir = tmp_path / "docs" / "ai"
    ai_dir.mkdir(parents=True)
    for filename in module.DOCS_AI_FRONTMATTER_FILES:
        (ai_dir / filename).write_text(
            "---\ntype: ai-principles\nowner: self\n---\n# Title\n",
            encoding="utf-8",
        )

    assert module.check_ai_doc_frontmatter(tmp_path) == []

    (ai_dir / "principles.md").write_text(
        "---\ntype: ai-bogus\nowner: self\n---\n# Principles\n",
        encoding="utf-8",
    )
    failures = module.check_ai_doc_frontmatter(tmp_path)
    assert any(
        "principles.md: frontmatter `type` not in legend: `ai-bogus`" in f
        for f in failures
    )


def test_docs_ai_frontmatter_rejects_extra_mutable_fields(tmp_path):
    module = _load_module()
    ai_dir = tmp_path / "docs" / "ai"
    ai_dir.mkdir(parents=True)
    for filename in module.DOCS_AI_FRONTMATTER_FILES:
        (ai_dir / filename).write_text(
            "---\ntype: ai-principles\nowner: self\n---\n# Title\n",
            encoding="utf-8",
        )
    (ai_dir / "north-star.md").write_text(
        "---\ntype: ai-principles\nowner: self\nstatus: active\n---\n# North Star\n",
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
            f"---\ntype: ai-principles\nowner: {owner}\n---\n# Title\n",
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
            f"---\ntype: ai-principles\nowner: {owner}\n---\n# Title\n",
            encoding="utf-8",
        )

    assert module.check_ai_doc_frontmatter(tmp_path) == []


def test_docs_ai_frontmatter_roster_rejects_unlisted_markdown_doc(tmp_path):
    module = _load_module()
    ai_dir = tmp_path / "docs" / "ai"
    ai_dir.mkdir(parents=True)
    for filename in module.DOCS_AI_FRONTMATTER_FILES:
        (ai_dir / filename).write_text(
            "---\ntype: ai-principles\nowner: self\n---\n# Title\n",
            encoding="utf-8",
        )
    (ai_dir / "extra.md").write_text(
        "---\ntype: ai-principles\nowner: self\n---\n# Extra\n",
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


def test_dashboard_entry_exactly_at_budget_passes(tmp_path):
    module = _load_module()
    plans = tmp_path / "docs" / "plans"
    plans.mkdir(parents=True)
    # Base fixture is 8 lines; extra_gate=32 lands exactly on the 40-line budget.
    (plans / "dashboard.md").write_text(
        _dashboard_entry(extra_gate=32), encoding="utf-8"
    )

    assert module.check_dashboard_structure(tmp_path) == []


def test_dashboard_entry_one_over_budget_is_rejected(tmp_path):
    module = _load_module()
    plans = tmp_path / "docs" / "plans"
    plans.mkdir(parents=True)
    (plans / "dashboard.md").write_text(
        _dashboard_entry(extra_gate=33), encoding="utf-8"
    )

    failures = module.check_dashboard_structure(tmp_path)

    assert any("PLAN-001 entry is 41 lines" in f for f in failures)


def test_next_step_exactly_at_budget_passes(tmp_path):
    module = _load_module()
    plans = tmp_path / "docs" / "plans"
    plans.mkdir(parents=True)
    # Bullet line plus 14 continuations lands exactly on the 15-line budget.
    next_step = "First line." + "".join(f"\n  continuation line {i}" for i in range(14))
    (plans / "dashboard.md").write_text(
        _dashboard_entry(next_step=next_step), encoding="utf-8"
    )

    assert module.check_dashboard_structure(tmp_path) == []


def test_next_step_one_over_budget_is_rejected(tmp_path):
    module = _load_module()
    plans = tmp_path / "docs" / "plans"
    plans.mkdir(parents=True)
    next_step = "First line." + "".join(f"\n  continuation line {i}" for i in range(15))
    (plans / "dashboard.md").write_text(
        _dashboard_entry(next_step=next_step), encoding="utf-8"
    )

    failures = module.check_dashboard_structure(tmp_path)

    assert any("PLAN-001 `Next step` field is 16 lines" in f for f in failures)


def test_duplicate_plan_id_across_dashboard_and_archive_is_rejected(tmp_path):
    module = _load_module()
    plans = tmp_path / "docs" / "plans"
    plans.mkdir(parents=True)
    (plans / "001-example.md").write_text("# Example\n", encoding="utf-8")
    (plans / "dashboard.md").write_text(_dashboard_entry(), encoding="utf-8")
    (plans / "archive.md").write_text(
        "### PLAN-001: Example\n\n"
        "**Final status:** Complete (archived 2026-07-03).\n\n"
        "**Outcome:** Shipped.\n",
        encoding="utf-8",
    )

    failures = module.check_plan_lifecycle(tmp_path)

    assert any(
        "PLAN-001 identifies 2 plan blocks" in f
        and "docs/plans/archive.md" in f
        and "docs/plans/dashboard.md" in f
        for f in failures
    )


def test_duplicate_plan_id_within_archive_is_rejected(tmp_path):
    module = _load_module()
    plans = tmp_path / "docs" / "plans"
    plans.mkdir(parents=True)
    (plans / "dashboard.md").write_text(_dashboard_entry(), encoding="utf-8")
    (plans / "001-example.md").write_text("# Example\n", encoding="utf-8")
    archive_block = (
        "### PLAN-050: Done\n\n"
        "**Final status:** Complete (archived 2026-07-03).\n\n"
        "**Outcome:** Shipped.\n\n"
    )
    (plans / "archive.md").write_text(archive_block * 2, encoding="utf-8")

    failures = module.check_plan_lifecycle(tmp_path)

    assert any("PLAN-050 identifies 2 plan blocks" in f for f in failures)


def test_malformed_archive_heading_is_rejected(tmp_path):
    module = _load_module()
    plans = tmp_path / "docs" / "plans"
    plans.mkdir(parents=True)
    (plans / "archive.md").write_text(
        "### PLAN-050 - Done\n\n**Outcome:** Shipped without a final status.\n",
        encoding="utf-8",
    )

    failures = module.check_plan_archive_shape(tmp_path)

    assert any("malformed plan heading" in f for f in failures)


def test_malformed_dashboard_heading_is_rejected(tmp_path):
    module = _load_module()
    plans = tmp_path / "docs" / "plans"
    plans.mkdir(parents=True)
    (plans / "dashboard.md").write_text(
        _dashboard_entry() + "\n#### PLAN-002: Hidden entry\n",
        encoding="utf-8",
    )

    failures = module.check_dashboard_structure(tmp_path)

    assert any("malformed plan heading" in f for f in failures)
