import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "check_phase5_cuda_workflow.py"
WORKFLOW = ROOT / ".github" / "workflows" / "ci_cuda.yml"

# The CUDA build job is expected to run on the self-hosted GPU runner, guarded
# so untrusted fork-PR code can never execute on it.
RUNS_ON = "    runs-on: ubuntu-latest-gpu"
FORK_GUARD = "github.event.pull_request.head.repo.full_name == github.repository"


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


def test_phase5_cuda_workflow_requires_self_hosted_gpu_runner(tmp_path):
    # Policy reversed from the old "no self-hosted runner": cuda-build must run
    # on the self-hosted GPU runner so CUDA executes against a real GPU. Any
    # other runner (e.g. a GitHub-hosted one) must be rejected.
    module = _load_module()
    path = _write_workflow(
        tmp_path,
        WORKFLOW.read_text(encoding="utf-8").replace(
            RUNS_ON,
            "    runs-on: ubuntu-latest",
        ),
    )

    messages = [violation.message for violation in module.find_violations(path)]

    assert any("must run on the self-hosted GPU runner" in m for m in messages)


def test_phase5_cuda_workflow_requires_fork_pr_guard(tmp_path):
    # Security: removing the same-repo guard must be rejected, so untrusted
    # fork-PR code can never run on the long-lived self-hosted GPU runner.
    module = _load_module()
    text = WORKFLOW.read_text(encoding="utf-8")
    assert FORK_GUARD in text, "expected the fork-PR guard in the real workflow"
    path = _write_workflow(tmp_path, text.replace(FORK_GUARD, "true"))

    messages = [violation.message for violation in module.find_violations(path)]

    assert any("guard the self-hosted GPU runner against fork" in m for m in messages)


def test_phase5_cuda_workflow_rejects_missing_build_gate(tmp_path):
    module = _load_module()
    path = _write_workflow(
        tmp_path,
        WORKFLOW.read_text(encoding="utf-8").replace(
            "pixi run --locked -e cuda build-cuda",
            "pixi run --locked -e cuda nvcc --version",
        ),
    )

    messages = [violation.message for violation in module.find_violations(path)]

    assert any("must run the CUDA build gate" in m for m in messages)


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
