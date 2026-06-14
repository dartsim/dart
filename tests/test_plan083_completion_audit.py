import importlib.util
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "check_plan083_completion_audit.py"
AUDIT = (
    ROOT
    / "docs"
    / "plans"
    / "083-unified-newton-barrier-multibody"
    / "completion-audit.md"
)
MANIFEST = (
    ROOT
    / "docs"
    / "plans"
    / "083-unified-newton-barrier-multibody"
    / "paper-deck-manifest.md"
)


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "check_plan083_completion_audit",
        SCRIPT,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _replace_table_status(audit_text: str, requirement: str, status: str) -> str:
    lines: list[str] = []
    for line in audit_text.splitlines():
        if not line.startswith("|"):
            lines.append(line)
            continue
        cells = [cell.strip() for cell in line.strip().strip("|").split("|")]
        if len(cells) >= 2 and cells[0] == requirement:
            cells[1] = status
            line = "| " + " | ".join(cells) + " |"
        lines.append(line)
    return "\n".join(lines) + "\n"


def test_plan083_completion_audit_matches_current_sidecars() -> None:
    module = _load_module()
    audit_text = AUDIT.read_text(encoding="utf-8")
    manifest_counts = module._manifest_status_counts(MANIFEST)
    cpu_row_count, cpu_counts, _ = module._summary_status_counts(
        module.DEFAULT_CPU_CORPUS,
        "scenes",
    )
    gpu_row_count, gpu_counts, _ = module._summary_status_counts(
        module.DEFAULT_GPU_PACKET,
        "rows",
    )

    assert (
        module.validate_audit(
            audit_text,
            manifest_counts,
            cpu_counts,
            gpu_counts,
            cpu_row_count,
            gpu_row_count,
        )
        == []
    )


def test_plan083_completion_audit_requires_not_complete_when_rows_are_planned() -> None:
    module = _load_module()
    audit_text = AUDIT.read_text(encoding="utf-8").replace(
        "Verdict: NOT COMPLETE.",
        "Verdict: COMPLETE.",
    )

    errors = module.validate_audit(
        audit_text,
        Counter({"planned": 1}),
        Counter(),
        Counter(),
        0,
        0,
    )

    assert "audit must stay NOT COMPLETE while manifest planned rows remain" in errors


def test_plan083_completion_audit_requires_blocked_cpu_and_gpu_rows() -> None:
    module = _load_module()
    manifest_counts = module._manifest_status_counts(MANIFEST)
    gpu_row_count, gpu_counts, _ = module._summary_status_counts(
        module.DEFAULT_GPU_PACKET,
        "rows",
    )
    audit_text = _replace_table_status(
        AUDIT.read_text(encoding="utf-8"),
        "CPU packets exist for performance rows",
        "Pass",
    )
    audit_text = _replace_table_status(
        audit_text,
        "GPU packets exist for GPU claims",
        "Pass",
    )

    errors = module.validate_audit(
        audit_text,
        manifest_counts,
        Counter({"planned": 1}),
        gpu_counts,
        1,
        gpu_row_count,
    )

    assert (
        "audit must block CPU packet completion while CPU rows remain planned" in errors
    )
    assert (
        "audit must block GPU packet completion while GPU rows remain incomplete"
        in errors
    )
