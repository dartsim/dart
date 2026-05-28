from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
WORKFLOW = ROOT / ".github" / "workflows" / "performance_dashboard.yml"


def test_performance_dashboard_workflow_runs_compute_gate():
    text = WORKFLOW.read_text(encoding="utf-8")

    assert "scripts/check_compute_graph_benchmarks.py" in text
    assert "Check compute benchmark gate" in text
    assert "python scripts/check_compute_graph_benchmarks.py" in text
    assert "--skip-contact-island-speedup-check" in text
    assert "--output build/compute_graph_check.json" in text


def test_performance_dashboard_workflow_keeps_checker_output_unpublished():
    text = WORKFLOW.read_text(encoding="utf-8")

    assert "--output build/compute_graph_check.json" in text
    assert "--output .benchmark_results/compute_graph_check.json" not in text
