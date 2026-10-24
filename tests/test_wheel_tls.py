"""The container TLS check must restore system trust even after a failure."""

import importlib.util
import re
import subprocess
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.skipif(sys.platform == "win32", reason="POSIX symlinks and process death")
@pytest.mark.parametrize("fail", [False, True])
def test_fallback_ca_restores_original_files_and_symlinks(tmp_path, monkeypatch, fail):
    spec = importlib.util.spec_from_file_location(
        "wheel_tls", ROOT / "scripts/test_wheel_tls.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module.sys, "platform", "linux")
    monkeypatch.setattr(module.os, "geteuid", lambda: 0, raising=False)
    default = tmp_path / "default.pem"
    fallback = tmp_path / "fallback.pem"
    original = tmp_path / "original.pem"
    original.write_text("original system CA")
    default.symlink_to(original)
    fallback.write_text("original fallback CA")
    certificate = tmp_path / "test.pem"
    certificate.write_text("test CA")
    marker = tmp_path / "container"
    marker.touch()
    monkeypatch.setattr(module, "CONTAINER_MARKERS", (marker,))
    monkeypatch.setattr(module, "SYSTEM_CA_BUNDLES", (default, fallback))
    monkeypatch.setattr(module, "DEFAULT_CA_BUNDLE", default)
    monkeypatch.setattr(module, "FALLBACK_CA_BUNDLE", fallback)
    try:
        with module.fallback_ca(certificate):
            assert not default.exists()
            assert fallback.read_text() == "test CA"
            if fail:
                raise RuntimeError("download failed")
    except RuntimeError as error:
        assert fail and str(error) == "download failed"
    assert default.is_symlink()
    assert default.read_text() == original.read_text() == "original system CA"
    assert fallback.read_text() == "original fallback CA"


def test_cross_distribution_tls_uses_both_real_distribution_images(
    monkeypatch, tmp_path
):
    spec = importlib.util.spec_from_file_location(
        "wheel_tls_distros", ROOT / "scripts/test_wheel_tls_distros.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    wheel = tmp_path / "dartpy.whl"
    wheel.touch()
    monkeypatch.setattr("sys.argv", ["test_wheel_tls_distros.py", str(wheel)])
    calls = []
    monkeypatch.setattr(
        module.subprocess, "run", lambda cmd, **kwargs: calls.append((cmd, kwargs))
    )
    module.main()
    assert len(calls) == 4
    for (build, options), (smoke, smoke_options), distro in zip(
        calls[::2], calls[1::2], ("ubuntu:24.04", "debian:12")
    ):
        assert f"BASE_IMAGE={distro}" in build
        assert "--rm" in smoke
        assert "--test-ca-fallback" in smoke[-3]
        assert smoke[-1] == "/w/dartpy.whl"
        assert options == smoke_options == {"check": True}


@pytest.mark.parametrize(
    "platform,uid,container",
    [("darwin", 0, True), ("linux", 1000, True), ("linux", 0, False)],
)
def test_fallback_refuses_non_disposable_hosts(
    tmp_path, monkeypatch, platform, uid, container
):
    spec = importlib.util.spec_from_file_location(
        "wheel_tls", ROOT / "scripts/test_wheel_tls.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    marker = tmp_path / "container"
    if container:
        marker.touch()
    monkeypatch.setattr(module.sys, "platform", platform)
    monkeypatch.setattr(module.os, "geteuid", lambda: uid, raising=False)
    monkeypatch.setattr(module, "CONTAINER_MARKERS", (marker,))
    with pytest.raises(RuntimeError, match="disposable root Linux container"):
        with module.fallback_ca(tmp_path / "test.pem"):
            pytest.fail("Guard allowed host trust modification")


@pytest.mark.skipif(sys.platform == "win32", reason="POSIX symlinks and process death")
def test_fallback_guard_restores_after_smoke_timeout(tmp_path):
    default, fallback, certificate, marker, ready = [
        tmp_path / name
        for name in ("default", "fallback", "certificate", "container", "ready")
    ]
    default.write_text("original default")
    fallback.write_text("original fallback")
    certificate.write_text("test CA")
    marker.touch()
    smoke = (
        "import runpy, time\nfrom pathlib import Path\n"
        f"module = runpy.run_path({str(ROOT / 'scripts/test_wheel_tls.py')!r})\n"
        "scope = module['fallback_ca'].__wrapped__.__globals__\n"
        "for name in ('sys', 'os', 'CONTAINER_MARKERS', 'SYSTEM_CA_BUNDLES', 'FALLBACK_CA_BUNDLE'):\n"
        "    assert name in scope, name\n"
        "scope['sys'].platform = 'linux'\n"
        "scope['os'].geteuid = lambda: 0\n"
        f"scope['CONTAINER_MARKERS'] = (Path({str(marker)!r}),)\n"
        f"scope['SYSTEM_CA_BUNDLES'] = (Path({str(default)!r}), Path({str(fallback)!r}))\n"
        f"scope['FALLBACK_CA_BUNDLE'] = Path({str(fallback)!r})\n"
        f"with module['fallback_ca'](Path({str(certificate)!r})):\n"
        f"    Path({str(ready)!r}).touch()\n"
        "    time.sleep(60)\n"
    )
    with pytest.raises(subprocess.TimeoutExpired):
        subprocess.run([sys.executable, "-c", smoke], timeout=2, check=True)
    assert ready.exists(), "Smoke never reached the displaced-bundle phase"
    deadline = time.monotonic() + 5
    while (
        not default.exists()
        or not fallback.exists()
        or fallback.read_text() != "original fallback"
    ):
        assert time.monotonic() < deadline, "Guard did not restore after SIGKILL"
        time.sleep(0.01)
    assert default.read_text() == "original default"


def test_smoke_hides_every_shipped_ca_candidate_in_documented_order():
    spec = importlib.util.spec_from_file_location(
        "wheel_tls", ROOT / "scripts/test_wheel_tls.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    header = (ROOT / "dart/utils/assets/detail/CaBundle.hpp").read_text()
    candidates = re.findall(r'"(/etc/[^\"]+)"', header)
    assert candidates == [path.as_posix() for path in module.SYSTEM_CA_BUNDLES]
    documentation = (ROOT / "docs/onboarding/io-parsing.md").read_text()
    for number, path in enumerate(candidates, 1):
        assert f"{number}. `{path}`" in documentation
