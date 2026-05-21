import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "check_performance_dashboard_pr_body.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("pr_body", SCRIPT)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _template(tmp_path: Path) -> Path:
    path = tmp_path / "PULL_REQUEST_TEMPLATE.md"
    path.write_text(
        """
## Summary

## Motivation / Problem

## Changes / Key Changes

## Testing

## Breaking Changes

## Related Issues / PRs (backports)

#### Checklist
""",
        encoding="utf-8",
    )
    return path


def _body(tmp_path: Path, *, extra: str = "") -> Path:
    module = _load_module()
    path = tmp_path / "PR_BODY.md"
    path.write_text(
        "\n".join(
            [
                "## Summary",
                "## Motivation / Problem",
                "## Changes / Key Changes",
                "## Testing",
                "## Breaking Changes",
                "## Related Issues / PRs (backports)",
                "#### Checklist",
                *module.REQUIRED_SNIPPETS,
                extra,
            ]
        ),
        encoding="utf-8",
    )
    return path


def test_pr_body_check_accepts_launch_ready_body(tmp_path):
    module = _load_module()

    module.verify_pr_body(_body(tmp_path), _template(tmp_path))


def test_pr_body_check_rejects_missing_template_header(tmp_path):
    module = _load_module()
    body = _body(tmp_path)
    body.write_text(body.read_text(encoding="utf-8").replace("## Testing", ""))

    with pytest.raises(RuntimeError, match="missing template header"):
        module.verify_pr_body(body, _template(tmp_path))


def test_pr_body_check_rejects_missing_required_launch_detail(tmp_path):
    module = _load_module()
    body = _body(tmp_path)
    body.write_text(
        body.read_text(encoding="utf-8").replace("trend_summary", ""),
        encoding="utf-8",
    )

    with pytest.raises(RuntimeError, match="missing launch-ready dashboard details"):
        module.verify_pr_body(body, _template(tmp_path))


def test_pr_body_check_rejects_missing_remote_approval_boundary(tmp_path):
    module = _load_module()
    body = _body(tmp_path)
    body.write_text(
        body.read_text(encoding="utf-8").replace("Remote approval boundary", ""),
        encoding="utf-8",
    )

    with pytest.raises(RuntimeError, match="missing launch-ready dashboard details"):
        module.verify_pr_body(body, _template(tmp_path))


def test_pr_body_check_rejects_stale_scratch_path(tmp_path):
    module = _load_module()

    with pytest.raises(RuntimeError, match="stale dashboard PR body details"):
        module.verify_pr_body(
            _body(tmp_path, extra="build/performance-dashboard-pr-body.md"),
            _template(tmp_path),
        )
