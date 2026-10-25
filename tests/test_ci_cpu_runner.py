"""Pin the CPU-runner routing policy and run the real cache-init shell."""

import os
import subprocess
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
WORKFLOW = yaml.load(
    (ROOT / ".github/workflows/ci_ubuntu.yml").read_text(encoding="utf-8"),
    Loader=yaml.BaseLoader,
)
ACTION = yaml.load(
    (ROOT / ".github/actions/sccache/action.yml").read_text(encoding="utf-8"),
    Loader=yaml.BaseLoader,
)
RELEASE = WORKFLOW["jobs"]["build"]
ROUTING = (
    "${{ github.repository == 'dartsim/dart' && github.ref == 'refs/heads/main'"
    ' && contains(fromJSON(\'["push","schedule","workflow_dispatch"]\'),'
    " github.event_name) && 'dart-cpu' || 'ubuntu-latest' }}"
)


def step(name_prefix):
    return next(s for s in ACTION["runs"]["steps"] if s["name"].startswith(name_prefix))


def test_release_routing_contract():
    # Static policy check; GitHub evaluates the expression (actionlint checks syntax).
    assert RELEASE["name"] == "Release"
    assert RELEASE["runs-on"] == ROUTING
    others = [j for k, j in WORKFLOW["jobs"].items() if k != "build"]
    assert all("dart-cpu" not in str(j.get("runs-on")) for j in others)


def test_release_apt_and_checkout_ownership():
    steps = RELEASE["steps"]
    checkout = next(s for s in steps if s["uses"].startswith("actions/checkout"))
    assert checkout["with"] == {"persist-credentials": "false"}
    apt = next(s for s in steps if s.get("uses", "").startswith("awalsh128/cache-apt"))
    assert apt["if"] == "runner.environment == 'github-hosted'"


def test_main_restore_is_hosted_only():
    assert step("Restore sccache (main)")["if"] == (
        "inputs.phase == 'restore' && runner.environment == 'github-hosted'"
    )
    assert "github-hosted" not in step("Restore sccache (this PR)")["if"]


def run_init(tmp_path, environment, temp_name, home):
    """Run the real init script with a stub pixi that answers the version probe."""
    bindir = tmp_path / "bin"
    bindir.mkdir(exist_ok=True)
    temp = tmp_path / temp_name
    temp.mkdir()
    out, genv = tmp_path / f"out-{temp_name}", tmp_path / f"env-{temp_name}"
    out.touch()
    genv.touch()
    # The stub is invoked as: pixi run -e ENV cmake -G Ninja -S SRC -B BUILD
    script = step("Initialize sccache")["run"]
    stub = bindir / "pixi"
    stub.write_text(
        '#!/bin/sh\nmkdir -p "${10}"\nprintf 13.2.0 > "${10}/version.txt"\n',
        encoding="utf-8",
    )
    stub.chmod(0o755)
    subprocess.run(
        ["bash", "-ec", script],
        check=True,
        env={
            "PATH": f"{bindir}:{os.environ['PATH']}",
            "HOME": str(home),
            "RUNNER_TEMP": str(temp),
            "RUNNER_ENVIRONMENT": environment,
            "GITHUB_OUTPUT": str(out),
            "GITHUB_ENV": str(genv),
            "CACHE_OS": "linux",
            "CACHE_PART": "Release",
            "PIXI_ENV": "default",
            "SNAPSHOT": "main",
        },
        timeout=30,
    )
    return (
        dict(
            line.split("=", 1) for line in genv.read_text().splitlines() if "=" in line
        ),
        temp,
    )


def test_hosted_cache_uses_runner_temp(tmp_path):
    env, temp = run_init(tmp_path, "github-hosted", "t1", tmp_path / "home")
    assert env["SCCACHE_DIR"] == f"{temp}/sccache"
    assert env["SCCACHE_PREFIX"] == "sccache-v2-linux-Release-13.2.0"
    assert env["SCCACHE_PR_DIR"] == f"{temp}/sccache-pr"


def test_self_hosted_cache_persists_across_runner_temp(tmp_path):
    home = tmp_path / "home"
    first, _ = run_init(tmp_path, "self-hosted", "t1", home)
    cache = Path(first["SCCACHE_DIR"])
    cache.mkdir(parents=True)
    (cache / "object").write_text("warm object")
    second, _ = run_init(tmp_path, "self-hosted", "t2", home)
    expected = f"{home}/.cache/dart-sccache/sccache-v2-linux-Release-13.2.0"
    assert first["SCCACHE_DIR"] == second["SCCACHE_DIR"] == expected
    assert (cache / "object").read_text() == "warm object"
    assert second["SCCACHE_PR_DIR"].endswith("/t2/sccache-pr")


def test_self_hosted_snapshot_uses_hosted_archive_path(tmp_path):
    cache = tmp_path / "persistent"
    cache.mkdir()
    (cache / "object").write_text("compiled object")
    temp = tmp_path / "runner-temp"
    temp.mkdir()
    staging = step("Stage self-hosted main snapshot")
    assert staging["if"].startswith("always() && inputs.phase == 'save'")
    assert "runner.environment == 'self-hosted'" in staging["if"]
    assert "env.SCCACHE_SNAPSHOT == 'main'" in staging["if"]
    subprocess.run(
        ["bash", "-euc", staging["run"]],
        env={**os.environ, "SCCACHE_DIR": str(cache), "RUNNER_TEMP": str(temp)},
        check=True,
        timeout=30,
    )
    assert (temp / "sccache/object").read_text() == "compiled object"
    assert not (temp / "sccache").is_symlink()
    assert (cache / "object").read_text() == "compiled object"
    # A build that never starts the compiler must not publish a stale snapshot.
    subprocess.run(
        ["bash", "-euc", staging["run"]],
        env={
            **os.environ,
            "SCCACHE_DIR": str(cache / "missing"),
            "RUNNER_TEMP": str(temp),
        },
        check=True,
        timeout=30,
    )
    assert not (temp / "sccache").exists()
    assert step("Save sccache")["with"]["path"] == (
        "${{ env.SCCACHE_SNAPSHOT == 'pr' && env.SCCACHE_PR_DIR"
        " || format('{0}/sccache', runner.temp) }}"
    )
