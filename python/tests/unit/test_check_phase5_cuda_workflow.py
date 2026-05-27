import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "check_phase5_cuda_workflow.py"
WORKFLOW = ROOT / ".github" / "workflows" / "ci_cuda.yml"


def _load_module():
    spec = importlib.util.spec_from_file_location("check_phase5_cuda_workflow", SCRIPT)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _write_workflow(tmp_path, text):
    path = tmp_path / "ci_cuda.yml"
    path.write_text(text, encoding="utf-8")
    return path


def _remove_line_containing(text, needle):
    return "\n".join(line for line in text.splitlines() if needle not in line) + "\n"


def test_current_repo_phase5_cuda_workflow_passes():
    module = _load_module()

    assert module.find_violations(WORKFLOW) == []


def test_phase5_cuda_workflow_rejects_missing_manual_trigger(tmp_path):
    module = _load_module()
    path = _write_workflow(
        tmp_path,
        WORKFLOW.read_text(encoding="utf-8").replace("workflow_dispatch:", "push:"),
    )

    messages = [violation.message for violation in module.find_violations(path)]

    assert "CUDA workflow must be manually dispatched" in messages


def test_phase5_cuda_workflow_rejects_unconditional_runtime_job(tmp_path):
    module = _load_module()
    path = _write_workflow(
        tmp_path,
        WORKFLOW.read_text(encoding="utf-8").replace(
            "    if: github.event_name == 'workflow_dispatch'\n",
            "",
        ),
    )

    messages = [violation.message for violation in module.find_violations(path)]

    assert "CUDA runtime job must only run on workflow_dispatch" in messages


def test_phase5_cuda_workflow_rejects_missing_full_benchmark_task(tmp_path):
    module = _load_module()
    path = _write_workflow(
        tmp_path,
        WORKFLOW.read_text(encoding="utf-8").replace(
            "pixi run --locked -e cuda bm-phase5-cuda-full",
            "pixi run --locked -e cuda test-cuda",
        ),
    )

    messages = [violation.message for violation in module.find_violations(path)]

    assert "CUDA workflow must run bm-phase5-cuda-full for the full row" in messages


def test_phase5_cuda_workflow_rejects_missing_packet_flag(tmp_path):
    module = _load_module()
    path = _write_workflow(
        tmp_path,
        _remove_line_containing(
            WORKFLOW.read_text(encoding="utf-8"),
            "--gpu-build-import-gate-passed",
        ),
    )

    messages = [violation.message for violation in module.find_violations(path)]

    assert (
        "CUDA workflow packet step is missing: --gpu-build-import-gate-passed"
        in messages
    )


def test_phase5_cuda_workflow_rejects_missing_artifact_path(tmp_path):
    module = _load_module()
    path = _write_workflow(
        tmp_path,
        WORKFLOW.read_text(encoding="utf-8").replace(
            "            .benchmark_results/phase5_cuda_packet.json\n",
            "",
        ),
    )

    messages = [violation.message for violation in module.find_violations(path)]

    assert (
        "CUDA workflow artifact upload is missing: "
        ".benchmark_results/phase5_cuda_packet.json"
    ) in messages


def test_phase5_cuda_workflow_rejects_non_always_artifact_upload(tmp_path):
    module = _load_module()
    path = _write_workflow(
        tmp_path,
        WORKFLOW.read_text(encoding="utf-8").replace(
            "        if: always()\n",
            "",
        ),
    )

    messages = [violation.message for violation in module.find_violations(path)]

    assert "CUDA workflow packet upload must run always" in messages
