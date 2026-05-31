import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "check_phase5_cuda_workflow.py"
WORKFLOW = ROOT / ".github" / "workflows" / "ci_cuda.yml"

# The CUDA build job is expected to run on the self-hosted GPU runner for
# trusted events and on a GitHub-hosted fallback for fork PRs.
SELF_HOSTED_RUNNER = "ubuntu-latest-gpu"
HOSTED_FORK_RUNNER = "'ubuntu-latest'"
FORK_FALLBACK = "github.event.pull_request.head.repo.full_name != github.repository"
NON_PR_EVENT_GUARD = "github.event_name != 'pull_request'"
SAME_REPO_PR_GUARD = "github.event.pull_request.head.repo.full_name == github.repository"


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
            SELF_HOSTED_RUNNER,
            "ubuntu-latest-cpu",
        ),
    )

    messages = [violation.message for violation in module.find_violations(path)]

    assert any(
        "must route trusted events to the self-hosted GPU runner" in m
        for m in messages
    )


def test_phase5_cuda_workflow_rejects_job_level_fork_skip(tmp_path):
    # Fork PRs must not skip the required CUDA Build check entirely; they need a
    # hosted compile fallback.
    module = _load_module()
    text = WORKFLOW.read_text(encoding="utf-8")
    path = _write_workflow(
        tmp_path,
        text.replace(
            "    runs-on: >-\n",
            "    if: ${{ github.event.pull_request.head.repo.full_name == github.repository }}\n"
            "    runs-on: >-\n",
        ),
    )

    messages = [violation.message for violation in module.find_violations(path)]

    assert any("must not use a job-level `if:`" in m for m in messages)


def test_phase5_cuda_workflow_requires_hosted_fork_fallback(tmp_path):
    module = _load_module()
    text = WORKFLOW.read_text(encoding="utf-8")
    assert FORK_FALLBACK in text, "expected the fork fallback condition in workflow"
    path = _write_workflow(tmp_path, text.replace(FORK_FALLBACK, "false"))

    messages = [violation.message for violation in module.find_violations(path)]

    assert any("must route fork PRs to the GitHub-hosted" in m for m in messages)


def test_phase5_cuda_workflow_requires_distinct_hosted_fork_runner(tmp_path):
    module = _load_module()
    text = WORKFLOW.read_text(encoding="utf-8")
    assert HOSTED_FORK_RUNNER in text, "expected hosted fork runner in workflow"
    path = _write_workflow(
        tmp_path,
        text.replace(HOSTED_FORK_RUNNER, "'ubuntu-latest-gpu'"),
    )

    messages = [violation.message for violation in module.find_violations(path)]

    assert any("must route fork PRs to the GitHub-hosted" in m for m in messages)


def test_phase5_cuda_workflow_rejects_reversed_runner_expression(tmp_path):
    module = _load_module()
    text = WORKFLOW.read_text(encoding="utf-8")
    path = _write_workflow(
        tmp_path,
        text.replace(
            "        'ubuntu-latest' ||\n"
            "        'ubuntu-latest-gpu'\n",
            "        'ubuntu-latest-gpu' ||\n"
            "        'ubuntu-latest'\n",
        ),
    )

    messages = [violation.message for violation in module.find_violations(path)]

    assert any("runs-on must route fork PRs" in m for m in messages)


def test_phase5_cuda_workflow_requires_build_gate_for_fork_fallback(tmp_path):
    module = _load_module()
    path = _write_workflow(
        tmp_path,
        WORKFLOW.read_text(encoding="utf-8").replace(
            "      - name: Build CUDA targets\n",
            "      - name: Build CUDA targets\n"
            "        if: ${{ github.event.pull_request.head.repo.full_name == github.repository }}\n",
        ),
    )

    messages = [violation.message for violation in module.find_violations(path)]

    assert any("Build CUDA targets must run for both" in m for m in messages)


def test_phase5_cuda_workflow_requires_gpu_test_guard(tmp_path):
    module = _load_module()
    text = WORKFLOW.read_text(encoding="utf-8")
    assert SAME_REPO_PR_GUARD in text, "expected same-repo GPU guard in workflow"
    path = _write_workflow(
        tmp_path,
        text.replace(
            "      - name: Run CUDA tests on GPU\n"
            "        if: >-\n"
            "          ${{\n"
            "            github.event_name != 'pull_request' ||\n"
            "            github.event.pull_request.head.repo.full_name == github.repository\n"
            "          }}\n",
            "      - name: Run CUDA tests on GPU\n",
        ),
    )

    messages = [violation.message for violation in module.find_violations(path)]

    assert any("Run CUDA tests on GPU must be guarded" in m for m in messages)


def test_phase5_cuda_workflow_requires_non_pr_gpu_guard_branch(tmp_path):
    module = _load_module()
    text = WORKFLOW.read_text(encoding="utf-8")
    assert NON_PR_EVENT_GUARD in text, "expected non-PR GPU guard branch in workflow"
    path = _write_workflow(
        tmp_path,
        text.replace(
            "            github.event_name != 'pull_request' ||\n",
            "",
        ),
    )

    messages = [violation.message for violation in module.find_violations(path)]

    assert any("Run CUDA tests on GPU must be guarded" in m for m in messages)
    assert any("Show GPU (nvidia-smi) must be guarded" in m for m in messages)


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
