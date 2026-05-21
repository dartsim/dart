import json
import importlib.util
import subprocess
import sys
from types import SimpleNamespace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "check_performance_dashboard_workflow_registration.py"

SPEC = importlib.util.spec_from_file_location("dashboard_workflow_registration", SCRIPT)
assert SPEC and SPEC.loader
REGISTRATION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(REGISTRATION)


def _workflow_file(tmp_path: Path, body: str | None = None) -> Path:
    path = tmp_path / "performance_dashboard.yml"
    path.write_text(
        body
        or """
name: Performance Dashboard

on:
  push:
    branches:
      - main
    paths:
      - ".github/actions/**"
      - ".github/workflows/performance_dashboard.yml"
      - "CMakeLists.txt"
      - "cmake/**"
      - "dart/**"
      - "data/**"
      - "pixi.lock"
      - "pixi.toml"
      - "scripts/*performance_dashboard*.py"
      - "scripts/check_collision_benchmarks.py"
      - "scripts/report_performance_to_bencher.py"
      - "scripts/run_cpp_benchmark.py"
      - "scripts/summarize_performance_dashboard.py"
      - "scripts/verify_performance_dashboard.py"
      - "tests/benchmark/**"
  schedule:
    - cron: "30 3 * * 0,3"
  workflow_dispatch:

permissions:
  actions: read
  contents: write
  pages: write

jobs:
  publish-performance-dashboard:
    runs-on: ubuntu-latest
    steps:
      - name: Generate performance dashboard
        run: |
          python scripts/generate_performance_dashboard.py \
            --input .benchmark_results \
            --output-dir gh-pages/performance \
            --history gh-pages/performance/data.json \
            --clean-output \
            --allow-empty
      - name: Publish dashboard to gh-pages
        id: publish_dashboard
        if: github.ref == 'refs/heads/main'
        run: echo publish
      - name: Request GitHub Pages build
        if: github.ref == 'refs/heads/main'
        run: |
          gh api "/repos/${GITHUB_REPOSITORY}/pages/builds"
      - name: Verify published dashboard endpoint
        if: github.ref == 'refs/heads/main'
        run: |
          python scripts/check_performance_dashboard_endpoint.py \
            https://dartsim.github.io/dart/performance/status.json \
            --expect-status-file gh-pages/performance/status.json \
            --expect-data-file gh-pages/performance/data.json \
            --expect-dashboard-file gh-pages/performance/index.html \
            --expect-summary-file gh-pages/performance/summary.md \
            --require-dashboard-page \
            --require-summary \
            --require-fresh \
            --attempts 60 \
            --interval 15
      - name: Setup Bencher CLI
        if: github.ref == 'refs/heads/main' && vars.BENCHER_PROJECT != ''
        continue-on-error: true
        uses: bencherdev/bencher@v0.6.6
      - name: Report benchmark JSON to Bencher
        if: github.ref == 'refs/heads/main' && vars.BENCHER_PROJECT != ''
        continue-on-error: true
        env:
          BENCHER_API_KEY: ${{ secrets.BENCHER_API_KEY }}
          BENCHER_HOST: ${{ vars.BENCHER_HOST }}
          BENCHER_PROJECT: ${{ vars.BENCHER_PROJECT }}
        run: |
          python scripts/report_performance_to_bencher.py \
            --skip-if-unconfigured \
            --skip-if-no-input
""",
        encoding="utf-8",
    )
    return path


def _workflows_json(tmp_path: Path, workflows) -> Path:
    path = tmp_path / "workflows.json"
    path.write_text(json.dumps({"workflows": workflows}), encoding="utf-8")
    return path


def _command(local_workflow: Path, workflows_json: Path, *extra: str) -> list[str]:
    return [
        sys.executable,
        str(SCRIPT),
        "--local-workflow",
        str(local_workflow),
        "--workflows-json",
        str(workflows_json),
        "--skip-action-ref-existence",
        *extra,
    ]


def test_workflow_registration_accepts_active_registered_workflow(tmp_path):
    local_workflow = _workflow_file(tmp_path)
    workflows_json = _workflows_json(
        tmp_path,
        [
            {
                "id": 123,
                "name": "Performance Dashboard",
                "path": ".github/workflows/performance_dashboard.yml",
                "state": "active",
            }
        ],
    )

    result = subprocess.run(
        _command(local_workflow, workflows_json),
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )

    assert "performance dashboard workflow registered" in result.stdout
    assert "123" in result.stdout


def test_workflow_registration_allows_missing_when_requested(tmp_path):
    local_workflow = _workflow_file(tmp_path)
    workflows_json = _workflows_json(tmp_path, [])

    result = subprocess.run(
        _command(local_workflow, workflows_json, "--allow-missing"),
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )

    assert "not registered" in result.stdout


def test_workflow_registration_rejects_missing_workflow(tmp_path):
    local_workflow = _workflow_file(tmp_path)
    workflows_json = _workflows_json(tmp_path, [])

    result = subprocess.run(
        _command(local_workflow, workflows_json),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "not registered" in result.stderr


def test_workflow_registration_rejects_inactive_workflow(tmp_path):
    local_workflow = _workflow_file(tmp_path)
    workflows_json = _workflows_json(
        tmp_path,
        [
            {
                "id": 123,
                "name": "Performance Dashboard",
                "path": ".github/workflows/performance_dashboard.yml",
                "state": "disabled_manually",
            }
        ],
    )

    result = subprocess.run(
        _command(local_workflow, workflows_json),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "state 'active'" in result.stderr


def test_workflow_registration_rejects_missing_dispatch_trigger(tmp_path):
    local_workflow = _workflow_file(
        tmp_path,
        """
name: Performance Dashboard

on:
  schedule:
    - cron: "30 3 * * 0,3"

permissions:
  actions: read
  contents: write
  pages: write

jobs:
  publish-performance-dashboard:
    runs-on: ubuntu-latest
    steps:
      - run: echo ok
""",
    )
    workflows_json = _workflows_json(tmp_path, [])

    result = subprocess.run(
        _command(local_workflow, workflows_json, "--allow-missing"),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "workflow_dispatch" in result.stderr


def test_workflow_registration_rejects_missing_push_trigger(tmp_path):
    local_workflow = _workflow_file(
        tmp_path,
        """
name: Performance Dashboard

on:
  schedule:
    - cron: "30 3 * * 0,3"
  workflow_dispatch:

permissions:
  actions: read
  contents: write
  pages: write

jobs:
  publish-performance-dashboard:
    runs-on: ubuntu-latest
    steps:
      - run: echo ok
""",
    )
    workflows_json = _workflows_json(tmp_path, [])

    result = subprocess.run(
        _command(local_workflow, workflows_json, "--allow-missing"),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "push" in result.stderr


def test_workflow_registration_rejects_non_main_push_trigger(tmp_path):
    local_workflow = _workflow_file(
        tmp_path,
        """
name: Performance Dashboard

on:
  push:
    branches:
      - release-6.16
  schedule:
    - cron: "30 3 * * 0,3"
  workflow_dispatch:

permissions:
  actions: read
  contents: write
  pages: write

jobs:
  publish-performance-dashboard:
    runs-on: ubuntu-latest
    steps:
      - run: echo ok
""",
    )
    workflows_json = _workflows_json(tmp_path, [])

    result = subprocess.run(
        _command(local_workflow, workflows_json, "--allow-missing"),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "main push trigger" in result.stderr


def test_workflow_registration_rejects_wrong_dashboard_schedule(tmp_path):
    local_workflow = _workflow_file(tmp_path)
    local_workflow.write_text(
        local_workflow.read_text(encoding="utf-8").replace(
            'cron: "30 3 * * 0,3"', 'cron: "0 0 * * 0"'
        ),
        encoding="utf-8",
    )
    workflows_json = _workflows_json(tmp_path, [])

    result = subprocess.run(
        _command(local_workflow, workflows_json, "--allow-missing"),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "cron schedule" in result.stderr
    assert "30 3 * * 0,3" in result.stderr


def test_workflow_registration_rejects_missing_dashboard_push_path(tmp_path):
    local_workflow = _workflow_file(
        tmp_path,
        """
name: Performance Dashboard

on:
  push:
    branches:
      - main
    paths:
      - ".github/workflows/performance_dashboard.yml"
  schedule:
    - cron: "30 3 * * 0,3"
  workflow_dispatch:

permissions:
  actions: read
  contents: write
  pages: write

jobs:
  publish-performance-dashboard:
    runs-on: ubuntu-latest
    steps:
      - run: echo ok
""",
    )
    workflows_json = _workflows_json(tmp_path, [])

    result = subprocess.run(
        _command(local_workflow, workflows_json, "--allow-missing"),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "main push path filters" in result.stderr
    assert "scripts/*performance_dashboard*.py" in result.stderr


def test_workflow_registration_rejects_missing_publish_permissions(tmp_path):
    local_workflow = _workflow_file(
        tmp_path,
        """
name: Performance Dashboard

on:
  push:
    branches:
      - main
    paths:
      - ".github/actions/**"
      - ".github/workflows/performance_dashboard.yml"
      - "CMakeLists.txt"
      - "cmake/**"
      - "dart/**"
      - "data/**"
      - "pixi.lock"
      - "pixi.toml"
      - "scripts/*performance_dashboard*.py"
      - "scripts/check_collision_benchmarks.py"
      - "scripts/report_performance_to_bencher.py"
      - "scripts/run_cpp_benchmark.py"
      - "scripts/summarize_performance_dashboard.py"
      - "scripts/verify_performance_dashboard.py"
      - "tests/benchmark/**"
  schedule:
    - cron: "30 3 * * 0,3"
  workflow_dispatch:

permissions:
  contents: read

jobs:
  publish-performance-dashboard:
    runs-on: ubuntu-latest
    steps:
      - run: echo ok
""",
    )
    workflows_json = _workflows_json(tmp_path, [])

    result = subprocess.run(
        _command(local_workflow, workflows_json, "--allow-missing"),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "publish permissions" in result.stderr
    assert "actions:read" in result.stderr
    assert "contents:write" in result.stderr
    assert "pages:write" in result.stderr


def test_workflow_registration_rejects_floating_bencher_action_ref(tmp_path):
    local_workflow = _workflow_file(
        tmp_path,
        """
name: Performance Dashboard

on:
  push:
    branches:
      - main
    paths:
      - ".github/actions/**"
      - ".github/workflows/performance_dashboard.yml"
      - "CMakeLists.txt"
      - "cmake/**"
      - "dart/**"
      - "data/**"
      - "pixi.lock"
      - "pixi.toml"
      - "scripts/*performance_dashboard*.py"
      - "scripts/check_collision_benchmarks.py"
      - "scripts/report_performance_to_bencher.py"
      - "scripts/run_cpp_benchmark.py"
      - "scripts/summarize_performance_dashboard.py"
      - "scripts/verify_performance_dashboard.py"
      - "tests/benchmark/**"
  schedule:
    - cron: "30 3 * * 0,3"
  workflow_dispatch:

permissions:
  actions: read
  contents: write
  pages: write

jobs:
  publish-performance-dashboard:
    runs-on: ubuntu-latest
    steps:
      - name: Publish dashboard to gh-pages
        id: publish_dashboard
        if: github.ref == 'refs/heads/main'
        run: echo publish
      - name: Request GitHub Pages build
        if: github.ref == 'refs/heads/main'
        run: |
          gh api "/repos/${GITHUB_REPOSITORY}/pages/builds"
      - name: Verify published dashboard endpoint
        if: github.ref == 'refs/heads/main'
        run: |
          python scripts/check_performance_dashboard_endpoint.py \
            https://dartsim.github.io/dart/performance/status.json \
            --expect-status-file gh-pages/performance/status.json \
            --expect-data-file gh-pages/performance/data.json \
            --expect-dashboard-file gh-pages/performance/index.html \
            --expect-summary-file gh-pages/performance/summary.md \
            --require-dashboard-page \
            --require-summary \
            --require-fresh \
            --attempts 60 \
            --interval 15
      - uses: bencherdev/bencher@main
""",
    )
    workflows_json = _workflows_json(tmp_path, [])

    result = subprocess.run(
        _command(local_workflow, workflows_json, "--allow-missing"),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "floating Bencher action refs" in result.stderr
    assert "bencherdev/bencher@main" in result.stderr


def test_workflow_registration_rejects_floating_external_action_ref(tmp_path):
    local_workflow = _workflow_file(
        tmp_path,
        """
name: Performance Dashboard

on:
  push:
    branches:
      - main
    paths:
      - ".github/actions/**"
      - ".github/workflows/performance_dashboard.yml"
      - "CMakeLists.txt"
      - "cmake/**"
      - "dart/**"
      - "data/**"
      - "pixi.lock"
      - "pixi.toml"
      - "scripts/*performance_dashboard*.py"
      - "scripts/check_collision_benchmarks.py"
      - "scripts/report_performance_to_bencher.py"
      - "scripts/run_cpp_benchmark.py"
      - "scripts/summarize_performance_dashboard.py"
      - "scripts/verify_performance_dashboard.py"
      - "tests/benchmark/**"
  schedule:
    - cron: "30 3 * * 0,3"
  workflow_dispatch:

permissions:
  actions: read
  contents: write
  pages: write

jobs:
  publish-performance-dashboard:
    runs-on: ubuntu-latest
    steps:
      - name: Publish dashboard to gh-pages
        id: publish_dashboard
        if: github.ref == 'refs/heads/main'
        run: echo publish
      - name: Request GitHub Pages build
        if: github.ref == 'refs/heads/main'
        run: |
          gh api "/repos/${GITHUB_REPOSITORY}/pages/builds"
      - name: Verify published dashboard endpoint
        if: github.ref == 'refs/heads/main'
        run: |
          python scripts/check_performance_dashboard_endpoint.py \
            https://dartsim.github.io/dart/performance/status.json \
            --expect-status-file gh-pages/performance/status.json \
            --expect-data-file gh-pages/performance/data.json \
            --expect-dashboard-file gh-pages/performance/index.html \
            --expect-summary-file gh-pages/performance/summary.md \
            --require-dashboard-page \
            --require-summary \
            --require-fresh \
            --attempts 60 \
            --interval 15
      - uses: actions/checkout@main
""",
    )
    workflows_json = _workflows_json(tmp_path, [])

    result = subprocess.run(
        _command(local_workflow, workflows_json, "--allow-missing"),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "floating GitHub Action refs" in result.stderr
    assert "actions/checkout@main" in result.stderr


def test_workflow_registration_rejects_missing_local_action_ref(tmp_path):
    local_workflow = _workflow_file(
        tmp_path,
        """
name: Performance Dashboard

on:
  push:
    branches:
      - main
    paths:
      - ".github/actions/**"
      - ".github/workflows/performance_dashboard.yml"
      - "CMakeLists.txt"
      - "cmake/**"
      - "dart/**"
      - "data/**"
      - "pixi.lock"
      - "pixi.toml"
      - "scripts/*performance_dashboard*.py"
      - "scripts/check_collision_benchmarks.py"
      - "scripts/report_performance_to_bencher.py"
      - "scripts/run_cpp_benchmark.py"
      - "scripts/summarize_performance_dashboard.py"
      - "scripts/verify_performance_dashboard.py"
      - "tests/benchmark/**"
  schedule:
    - cron: "30 3 * * 0,3"
  workflow_dispatch:

permissions:
  actions: read
  contents: write
  pages: write

jobs:
  publish-performance-dashboard:
    runs-on: ubuntu-latest
    steps:
      - name: Publish dashboard to gh-pages
        id: publish_dashboard
        if: github.ref == 'refs/heads/main'
        run: echo publish
      - name: Request GitHub Pages build
        if: github.ref == 'refs/heads/main'
        run: |
          gh api "/repos/${GITHUB_REPOSITORY}/pages/builds"
      - name: Verify published dashboard endpoint
        if: github.ref == 'refs/heads/main'
        run: |
          python scripts/check_performance_dashboard_endpoint.py \
            https://dartsim.github.io/dart/performance/status.json \
            --expect-status-file gh-pages/performance/status.json \
            --expect-data-file gh-pages/performance/data.json \
            --expect-dashboard-file gh-pages/performance/index.html \
            --expect-summary-file gh-pages/performance/summary.md \
            --require-dashboard-page \
            --require-summary \
            --require-fresh \
            --attempts 60 \
            --interval 15
      - uses: ./.github/actions/missing-local-action
""",
    )
    workflows_json = _workflows_json(tmp_path, [])

    result = subprocess.run(
        _command(local_workflow, workflows_json, "--allow-missing"),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "missing local action" in result.stderr
    assert "./.github/actions/missing-local-action" in result.stderr


def test_workflow_registration_rejects_pages_build_gated_on_changed_push(tmp_path):
    local_workflow = _workflow_file(tmp_path)
    workflow = local_workflow.read_text(encoding="utf-8")
    local_workflow.write_text(
        workflow.replace(
            "      - name: Request GitHub Pages build\n"
            "        if: github.ref == 'refs/heads/main'\n",
            "      - name: Request GitHub Pages build\n"
            "        if: github.ref == 'refs/heads/main' && "
            "steps.publish_dashboard.outputs.published == 'true'\n",
        ),
        encoding="utf-8",
    )
    workflows_json = _workflows_json(tmp_path, [])

    result = subprocess.run(
        _command(local_workflow, workflows_json, "--allow-missing"),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "every main publisher run" in result.stderr


def test_workflow_registration_rejects_generator_without_clean_output(tmp_path):
    local_workflow = _workflow_file(tmp_path)
    workflow = local_workflow.read_text(encoding="utf-8")
    local_workflow.write_text(
        workflow.replace("            --clean-output", ""),
        encoding="utf-8",
    )
    workflows_json = _workflows_json(tmp_path, [])

    result = subprocess.run(
        _command(local_workflow, workflows_json, "--allow-missing"),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "--clean-output" in result.stderr


def test_workflow_registration_rejects_missing_live_endpoint_checks(tmp_path):
    local_workflow = _workflow_file(tmp_path)
    workflow = local_workflow.read_text(encoding="utf-8")
    local_workflow.write_text(
        workflow.replace(" --require-fresh", ""),
        encoding="utf-8",
    )
    workflows_json = _workflows_json(tmp_path, [])

    result = subprocess.run(
        _command(local_workflow, workflows_json, "--allow-missing"),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "--require-fresh" in result.stderr


def test_workflow_registration_rejects_missing_exact_surface_match(tmp_path):
    local_workflow = _workflow_file(tmp_path)
    workflow = local_workflow.read_text(encoding="utf-8")
    local_workflow.write_text(
        workflow.replace(
            "--expect-dashboard-file gh-pages/performance/index.html",
            "",
        ),
        encoding="utf-8",
    )
    workflows_json = _workflows_json(tmp_path, [])

    result = subprocess.run(
        _command(local_workflow, workflows_json, "--allow-missing"),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "--expect-dashboard-file" in result.stderr


def test_workflow_registration_rejects_unfiltered_seed_artifacts(tmp_path):
    local_workflow = tmp_path / "performance_dashboard.yml"
    workflow = (ROOT / ".github" / "workflows" / "performance_dashboard.yml").read_text(
        encoding="utf-8"
    )
    local_workflow.write_text(
        workflow.replace('            --branch "${GITHUB_REF_NAME}" \\\n', ""),
        encoding="utf-8",
    )
    workflows_json = _workflows_json(tmp_path, [])

    result = subprocess.run(
        _command(local_workflow, workflows_json, "--allow-missing"),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "GITHUB_REF_NAME" in result.stderr


def test_workflow_registration_rejects_blocking_bencher_reporting(tmp_path):
    local_workflow = _workflow_file(tmp_path)
    workflow = local_workflow.read_text(encoding="utf-8")
    local_workflow.write_text(
        workflow.replace(
            "      - name: Report benchmark JSON to Bencher\n"
            "        if: github.ref == 'refs/heads/main' && "
            "vars.BENCHER_PROJECT != ''\n"
            "        continue-on-error: true\n",
            "      - name: Report benchmark JSON to Bencher\n"
            "        if: github.ref == 'refs/heads/main' && "
            "vars.BENCHER_PROJECT != ''\n",
        ),
        encoding="utf-8",
    )
    workflows_json = _workflows_json(tmp_path, [])

    result = subprocess.run(
        _command(local_workflow, workflows_json, "--allow-missing"),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "Bencher reporting step must be continue-on-error" in result.stderr


def test_workflow_registration_rejects_bencher_without_skip_flags(tmp_path):
    local_workflow = _workflow_file(tmp_path)
    workflow = local_workflow.read_text(encoding="utf-8")
    local_workflow.write_text(
        workflow.replace(" --skip-if-no-input", ""),
        encoding="utf-8",
    )
    workflows_json = _workflows_json(tmp_path, [])

    result = subprocess.run(
        _command(local_workflow, workflows_json, "--allow-missing"),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "--skip-if-no-input" in result.stderr


def test_action_tag_check_falls_back_to_git_when_github_api_is_limited(monkeypatch):
    calls = []

    def fake_read_url_status(url, token, timeout):
        calls.append(("api", url, token, timeout))
        return 403

    def fake_run(command, **kwargs):
        calls.append(("git", command, kwargs))
        return SimpleNamespace(returncode=0, stderr="")

    monkeypatch.setattr(REGISTRATION, "_read_url_status", fake_read_url_status)
    monkeypatch.setattr(REGISTRATION.subprocess, "run", fake_run)

    args = SimpleNamespace(api_url="https://api.github.com", token=None, timeout=15.0)

    assert REGISTRATION._action_tag_exists("actions", "checkout", "v6", args)
    assert calls[0][0] == "api"
    assert calls[1][0] == "git"
    assert "https://github.com/actions/checkout.git" in calls[1][1]
