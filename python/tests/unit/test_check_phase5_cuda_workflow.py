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


def test_current_repo_phase5_cuda_workflow_passes():
    module = _load_module()

    assert module.find_violations(WORKFLOW) == []


def test_phase5_cuda_workflow_rejects_self_hosted_runner(tmp_path):
    module = _load_module()
    path = _write_workflow(
        tmp_path,
        WORKFLOW.read_text(encoding="utf-8").replace(
            "    runs-on: ubuntu-latest",
            "    runs-on: [self-hosted, dartsim, cuda]",
        ),
    )

    messages = [violation.message for violation in module.find_violations(path)]

    assert any("must not use a self-hosted runner" in m for m in messages)


def test_phase5_cuda_workflow_rejects_missing_build_job(tmp_path):
    module = _load_module()
    path = _write_workflow(
        tmp_path,
        WORKFLOW.read_text(encoding="utf-8").replace(
            "  cuda-build:",
            "  cuda-other:",
        ),
    )

    messages = [violation.message for violation in module.find_violations(path)]

    assert "CUDA workflow must define a cuda-build job" in messages


def test_phase5_cuda_workflow_rejects_non_ubuntu_build_runner(tmp_path):
    module = _load_module()
    path = _write_workflow(
        tmp_path,
        WORKFLOW.read_text(encoding="utf-8").replace(
            "    runs-on: ubuntu-latest",
            "    runs-on: macos-13",
        ),
    )

    messages = [violation.message for violation in module.find_violations(path)]

    assert "cuda-build must run on a GitHub-hosted ubuntu runner" in messages


def test_phase5_cuda_workflow_rejects_missing_build_import_gate(tmp_path):
    module = _load_module()
    path = _write_workflow(
        tmp_path,
        WORKFLOW.read_text(encoding="utf-8").replace(
            "pixi run --locked -e cuda build-cuda",
            "pixi run --locked -e cuda nvcc --version",
        ),
    )

    messages = [violation.message for violation in module.find_violations(path)]

    assert any("must run the build/import gate" in m for m in messages)


def test_phase5_cuda_workflow_rejects_missing_cuda_env_setup(tmp_path):
    module = _load_module()
    path = _write_workflow(
        tmp_path,
        WORKFLOW.read_text(encoding="utf-8").replace(
            "          environments: cuda\n",
            "",
        ),
    )

    messages = [violation.message for violation in module.find_violations(path)]

    assert "cuda-build must set up the cuda pixi environment" in messages
