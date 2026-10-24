#!/usr/bin/env python3
"""Check the installed wheel's TLS verification and optional Linux CA fallback.

--test-ca-fallback temporarily changes system trust files: disposable containers only.
"""

from __future__ import annotations

import argparse
import contextlib
import hashlib
import http.server
import os
import shutil
import ssl
import subprocess
import sys
import tempfile
import threading
from pathlib import Path

DOWNLOAD = r"""
import ssl
import sys
from pathlib import Path
from dartpy.utils import ModelResourceRetriever

retriever = ModelResourceRetriever(sys.argv[2], False)
assert retriever.addManifest(Path(sys.argv[1]).as_uri())
downloaded = retriever.getFilePath("model://wheel/v1/robot.urdf")
if sys.argv[3] == "reject":
    assert not downloaded, "An invalid certificate must fail"
else:
    assert downloaded, "The trusted certificate must verify"
    assert Path(downloaded).read_bytes() == bytes.fromhex(sys.argv[3])
"""

DEFAULT_CA_BUNDLE = Path("/etc/pki/tls/certs/ca-bundle.crt")
FALLBACK_CA_BUNDLE = Path("/etc/ssl/certs/ca-certificates.crt")
SYSTEM_CA_BUNDLES = (
    FALLBACK_CA_BUNDLE,
    DEFAULT_CA_BUNDLE,
    Path("/etc/ssl/ca-bundle.pem"),
    Path("/etc/ssl/cert.pem"),
)
CONTAINER_MARKERS = (Path("/.dockerenv"), Path("/run/.containerenv"))


@contextlib.contextmanager
def fallback_ca(certificate: Path):
    """Hide curl's pinned default and supply only a fallback bundle."""
    if (
        sys.platform != "linux"
        or os.geteuid() != 0
        or not any(path.is_file() for path in CONTAINER_MARKERS)
    ):
        raise RuntimeError(
            "CA fallback check requires a disposable root Linux container"
        )
    # The guard restores on pipe EOF, including when our caller kills this process.
    guard = subprocess.Popen(
        [
            sys.executable,
            "-I",
            "-u",
            str(Path(__file__).resolve()),
            "--guard-ca",
            str(certificate),
            str(FALLBACK_CA_BUNDLE),
            *(str(path) for path in SYSTEM_CA_BUNDLES),
        ],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        text=True,
    )
    try:
        if guard.stdout.readline() != "CA guard ready\n":
            raise RuntimeError("CA guard failed to prepare system bundles")
        yield
    finally:
        guard.stdin.close()
        guard.wait(timeout=10)
        guard.stdout.close()
        if guard.returncode:
            raise RuntimeError("CA guard failed to restore system bundles")


def guard_ca(certificate: Path, fallback: Path, bundles: list[Path]) -> None:
    """Own displaced bundles until the smoke process closes its pipe or dies."""
    with tempfile.TemporaryDirectory() as directory:
        saved = []
        installed = False
        try:
            for path in bundles:
                if path.exists() or path.is_symlink():
                    backup = Path(directory) / str(len(saved))
                    shutil.move(path, backup)
                    saved.append((path, backup))
            fallback.parent.mkdir(parents=True, exist_ok=True)
            installed = True
            shutil.copyfile(certificate, fallback)
            print("CA guard ready", flush=True)
            sys.stdin.read()
        finally:
            if installed:
                fallback.unlink(missing_ok=True)
            for path, backup in reversed(saved):
                shutil.move(backup, path)


def check_tls(*, test_ca_fallback: bool = False) -> None:
    print("Native smoke: importing ModelResourceRetriever", flush=True)
    from dartpy.utils import ModelResourceRetriever

    payload = b'<robot name="wheel_https"><link name="base"/></robot>'
    requests = []

    class Handler(http.server.BaseHTTPRequestHandler):
        def do_GET(self):
            assert self.request_version == "HTTP/1.1"
            assert "Authorization" not in self.headers
            requests.append(self.path)
            self.send_response(200)
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def log_message(self, *_args):
            pass

    certificate = (
        Path(__file__).resolve().parents[1]
        / "python/tests/unit/utils/fixtures/localhost-test.crt"
    )
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(certificate, certificate.with_suffix(".key"))
    env = os.environ.copy()
    for name in ("CURL_CA_BUNDLE", "SSL_CERT_FILE"):
        env.pop(name, None)
    env["NO_PROXY"] = env["no_proxy"] = "localhost,127.0.0.1"
    with http.server.ThreadingHTTPServer(("127.0.0.1", 0), Handler) as server:
        server.socket = context.wrap_socket(server.socket, server_side=True)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            with tempfile.TemporaryDirectory() as directory:
                root = Path(directory)

                def download(label, host="localhost", *, rejection=()):
                    manifest = root / "model.xml"
                    manifest.write_text(
                        '<model schemaVersion="1" id="wheel" revision="v1" entrypoint="robot.urdf">'
                        '<source url="https://example.invalid/pinned"/><license name="MIT"/>'
                        f'<file path="robot.urdf" url="https://{host}:{server.server_port}/robot.urdf" '
                        f'sha256="{hashlib.sha256(payload).hexdigest()}" size="{len(payload)}"/>'
                        "</model>"
                    )
                    result = subprocess.run(
                        [
                            sys.executable,
                            "-I",
                            "-u",
                            "-c",
                            DOWNLOAD,
                            str(manifest),
                            str(root / label),
                            "reject" if rejection else payload.hex(),
                        ],
                        env=env,
                        capture_output=True,
                        text=True,
                        timeout=30,
                    )
                    print(result.stdout, end="", flush=True)
                    print(result.stderr, end="", file=sys.stderr, flush=True)
                    assert result.returncode == 0, label
                    if rejection:
                        diagnostic = result.stderr.lower()
                        assert "failed https download" in diagnostic, diagnostic
                        assert any(
                            error in diagnostic for error in rejection
                        ), diagnostic
                        assert not requests, "TLS must fail before the HTTP request"
                    else:
                        assert requests == ["/robot.urdf"], requests
                        requests.clear()
                    print(f"TLS PASS: {label}", flush=True)

                untrusted_errors = (
                    "ssl certificate problem",
                    "certificate verify failed",
                    "ssl certificate openssl verify result: self-signed certificate",
                    "sec_e_untrusted_root",
                    "cert_trust_is_untrusted_root",
                    "cert_e_untrustedroot",
                    "based on an untrusted root",
                    "ssl peer certificate or ssh remote key was not ok",
                )
                download("untrusted certificate rejected", rejection=untrusted_errors)
                env["CURL_CA_BUNDLE"] = str(certificate)
                download(
                    "hostname mismatch rejected",
                    "127.0.0.1",
                    rejection=(
                        "no alternative certificate subject name matches",
                        "does not match target host",
                        "hostname mismatch",
                        "sec_e_wrong_principal",
                        "cert_e_cn_no_match",
                        # Schannel's IP mismatch may have only curl's peer-verification text.
                        "ssl peer certificate or ssh remote key was not ok",
                        "schannel: certgetnamestring() failed to match connection hostname",
                    ),
                )
                download("explicit CA verified")
                if test_ca_fallback:
                    env.pop("CURL_CA_BUNDLE")
                    system_roots = next(
                        path.read_bytes()
                        for path in SYSTEM_CA_BUNDLES
                        if path.is_file()
                    )
                    combined = root / "system-and-test-ca.pem"
                    combined.write_bytes(
                        system_roots + b"\n" + certificate.read_bytes()
                    )
                    with fallback_ca(combined):
                        assert (
                            "CURL_CA_BUNDLE" not in env and "SSL_CERT_FILE" not in env
                        )
                        download("system CA fallback verified without overrides")
                        FALLBACK_CA_BUNDLE.write_bytes(system_roots)
                        download(
                            "untrusted certificate rejected with system CA fallback",
                            rejection=untrusted_errors,
                        )
                        FALLBACK_CA_BUNDLE.unlink()
                        download(
                            "missing system CA fails closed without overrides",
                            rejection=(
                                "error setting certificate file",
                                "error setting certificate verify locations",
                                "error adding trust anchors",
                            ),
                        )
        finally:
            server.shutdown()
            thread.join(timeout=5)


def main() -> None:
    if sys.argv[1:2] == ["--guard-ca"]:
        certificate, fallback, *bundles = map(Path, sys.argv[2:])
        guard_ca(certificate, fallback, bundles)
        return
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--test-ca-fallback", action="store_true")
    args = parser.parse_args()
    check_tls(test_ca_fallback=args.test_ca_fallback)


if __name__ == "__main__":
    main()
