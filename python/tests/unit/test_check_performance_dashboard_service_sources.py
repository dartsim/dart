import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "check_performance_dashboard_service_sources.py"


class _Response:
    def __init__(
        self,
        status=200,
        url="https://example.invalid/source",
        body="evidence body",
    ):
        self.status = status
        self._url = url
        self._body = body

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback):
        return False

    def geturl(self):
        return self._url

    def read(self):
        return self._body.encode()


def _load_module():
    spec = importlib.util.spec_from_file_location("service_sources", SCRIPT)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_service_source_check_accepts_https_urls(monkeypatch):
    module = _load_module()
    url = "https://example.invalid/source"
    monkeypatch.setitem(module.EXPECTED_URL_SNIPPETS, url, ["evidence body"])
    seen_urls = []

    def fake_urlopen(request, timeout):
        seen_urls.append(request.full_url)
        return _Response(url=request.full_url)

    monkeypatch.setattr(module, "urlopen", fake_urlopen)

    module.verify_service_source_urls(
        [
            {
                "role": "primary-dashboard",
                "evidence_urls": [url],
            }
        ],
        timeout=1.0,
    )

    assert seen_urls == [url]


def test_service_source_check_retries_transient_http_error(monkeypatch):
    module = _load_module()
    url = "https://example.invalid/source"
    monkeypatch.setitem(module.EXPECTED_URL_SNIPPETS, url, ["evidence body"])
    seen_urls = []

    def fake_urlopen(request, timeout):
        seen_urls.append(request.full_url)
        if len(seen_urls) == 1:
            raise module.HTTPError(request.full_url, 520, "Origin Error", None, None)
        return _Response(url=request.full_url)

    monkeypatch.setattr(module, "urlopen", fake_urlopen)

    module.verify_service_source_urls(
        [
            {
                "role": "external-history-and-thresholds",
                "evidence_urls": [url],
            }
        ],
        timeout=1.0,
        attempts=2,
        interval=0.0,
    )

    assert seen_urls == [
        url,
        url,
    ]


def test_service_source_check_reports_timeout_without_traceback(monkeypatch):
    module = _load_module()

    def fake_urlopen(request, timeout):
        raise TimeoutError("timed out")

    monkeypatch.setattr(module, "urlopen", fake_urlopen)

    with pytest.raises(RuntimeError, match="failed after 2 attempts: timed out"):
        module.verify_service_source_urls(
            [
                {
                    "role": "self-hosted-performance-tracker",
                    "evidence_urls": ["https://example.invalid/source"],
                }
            ],
            timeout=1.0,
            attempts=2,
            interval=0.0,
        )


def test_service_source_check_rejects_redirect(monkeypatch):
    module = _load_module()

    def fake_urlopen(request, timeout):
        return _Response(url="https://example.invalid/new-source")

    monkeypatch.setattr(module, "urlopen", fake_urlopen)

    with pytest.raises(RuntimeError, match="redirected"):
        module.verify_service_source_urls(
            [
                {
                    "role": "microbenchmark-pr-pilot",
                    "evidence_urls": ["https://example.invalid/old-source"],
                }
            ],
            timeout=1.0,
        )


def test_service_source_check_rejects_missing_evidence_urls():
    module = _load_module()

    with pytest.raises(RuntimeError, match="no evidence URLs"):
        module.verify_service_source_urls(
            [{"role": "primary-dashboard"}],
            timeout=1.0,
        )


def test_service_source_check_rejects_missing_expected_snippet(monkeypatch):
    module = _load_module()
    url = "https://example.invalid/source"
    monkeypatch.setitem(module.EXPECTED_URL_SNIPPETS, url, ["required text"])

    def fake_urlopen(request, timeout):
        return _Response(url=request.full_url, body="different text")

    monkeypatch.setattr(module, "urlopen", fake_urlopen)

    with pytest.raises(RuntimeError, match="missing expected evidence text"):
        module.verify_service_source_urls(
            [
                {
                    "role": "primary-dashboard",
                    "evidence_urls": [url],
                }
            ],
            timeout=1.0,
        )


def test_service_source_check_rejects_missing_expected_snippet_config(monkeypatch):
    module = _load_module()
    url = "https://example.invalid/unconfigured-source"

    def fake_urlopen(request, timeout):
        return _Response(url=request.full_url)

    monkeypatch.setattr(module, "urlopen", fake_urlopen)

    with pytest.raises(RuntimeError, match="no expected evidence text configured"):
        module.verify_service_source_urls(
            [
                {
                    "role": "primary-dashboard",
                    "evidence_urls": [url],
                }
            ],
            timeout=1.0,
        )
