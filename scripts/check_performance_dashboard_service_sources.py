#!/usr/bin/env python3
"""Verify performance dashboard service-decision evidence links."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from generate_performance_dashboard import service_decision

EXPECTED_URL_SNIPPETS = {
    "https://docs.github.com/en/pages/getting-started-with-github-pages": [
        "GitHub Pages is available in public repositories with GitHub Free",
    ],
    "https://docs.github.com/en/pages/getting-started-with-github-pages/configuring-a-publishing-source-for-your-github-pages-site": [
        "You can publish your site when changes are pushed to a specific branch",
    ],
    "https://github.com/bencherdev/bencher/blob/main/services/console/src/pages/pricing.astro": [
        "Free plan isn't billed for Metrics",
        "Bencher Self-Hosted",
    ],
    "https://github.com/bencherdev/bencher/blob/main/services/console/src/content/docs-how-to/en/github-actions.mdx": [
        "How to use Bencher in GitHub Actions",
        "Continuous Benchmarking in GitHub Actions with Bencher",
    ],
    "https://github.com/bencherdev/bencher/blob/main/services/console/src/chunks/docs-how-to/github-actions/en/base-branch.mdx": [
        "bencher run",
        "BENCHER_API_KEY",
        "--project",
    ],
    "https://github.com/bencherdev/bencher/blob/main/services/console/src/chunks/docs-explanation/adapters/en/cpp-google.mdx": [
        "The C++ Google Adapter",
        "cpp_google",
    ],
    "https://codspeed.io/docs/features/seats-and-billing": [
        "CodSpeed is free and unlimited on public repositories",
    ],
    "https://codspeed.io/docs/integrations/ci/github-actions": [
        "Learn how to setup CodSpeed and run benchmarks within your GitHub Actions CI workflow",
        "tokenless uploads for public repositories",
    ],
    "https://codspeed.io/docs/guides/how-to-benchmark-cpp-with-google-benchmark": [
        "CODSPEED_MODE",
        "google_benchmark",
    ],
    "https://github.com/benchmark-action/github-action-benchmark": [
        "Google Benchmark Framework",
        "GitHub pages",
    ],
    "https://github.com/marketplace/actions/continuous-benchmark": [
        "Continuous Benchmark",
        "benchmark-action/github-action-benchmark",
    ],
    "https://github.com/airspeed-velocity/asv": [
        "Airspeed Velocity",
    ],
    "https://asv.readthedocs.io/en/stable/using.html": [
        "asv run",
    ],
    "https://asv.readthedocs.io/en/stable/commands.html": [
        "asv publish",
    ],
    "https://llvm.org/docs/lnt/": [
        "LLVM's performance tracking software",
    ],
    "https://llvm.org/docs/lnt/concepts.html": [
        "Orders, Machines and Tests",
    ],
    "https://llvm.org/docs/lnt/importing_data.html": [
        "LNT Report File Format",
    ],
    "https://github.com/llvm/llvm-lnt": [
        "LNT is an infrastructure for performance testing",
    ],
    "https://conbench.github.io/conbench/": [
        "benchmarks in any language",
        "persist them for comparison",
    ],
    "https://github.com/conbench/conbench": [
        "Continuous Benchmarking",
    ],
    "https://openbenchmarking.org/": [
        "Storage of Phoronix Test Suite benchmark result data",
    ],
    "https://openbenchmarking.org/features": [
        "Free use of OpenBenchmarking.org is encouraged",
    ],
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--timeout",
        type=float,
        default=20.0,
        help="Per-request timeout in seconds.",
    )
    parser.add_argument(
        "--allow-redirects",
        action="store_true",
        help="Allow evidence URLs that resolve through redirects.",
    )
    parser.add_argument(
        "--attempts",
        type=int,
        default=3,
        help="Fetch attempts for transient network failures and HTTP 5xx responses.",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="Seconds to wait between retry attempts.",
    )
    return parser.parse_args()


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def _read_url(url: str, *, timeout: float) -> tuple[int, str, str]:
    request = Request(
        url,
        headers={"User-Agent": "DART performance dashboard preflight"},
    )
    with urlopen(request, timeout=timeout) as response:
        body = response.read().decode("utf-8", errors="replace")
        return response.status, response.geturl(), body


def _maybe_sleep(attempt: int, *, attempts: int, interval: float) -> None:
    if attempt < attempts and interval > 0:
        time.sleep(interval)


def _transient_http_status(status: int) -> bool:
    return 500 <= status < 600


def _network_error_reason(exc: OSError) -> str:
    return str(getattr(exc, "reason", exc))


def _verify_expected_snippets(url: str, body: str) -> None:
    _require(
        url in EXPECTED_URL_SNIPPETS,
        f"{url} has no expected evidence text configured",
    )
    missing = [snippet for snippet in EXPECTED_URL_SNIPPETS[url] if snippet not in body]
    _require(
        not missing,
        f"{url} is missing expected evidence text: {', '.join(missing)}",
    )


def _verify_url(
    url: str, *, timeout: float, allow_redirects: bool, attempts: int, interval: float
) -> None:
    for attempt in range(1, attempts + 1):
        try:
            status, final_url, body = _read_url(url, timeout=timeout)
        except HTTPError as exc:
            message = f"{url} returned HTTP {exc.code}"
            if _transient_http_status(exc.code) and attempt < attempts:
                _maybe_sleep(attempt, attempts=attempts, interval=interval)
                continue
            if attempt > 1:
                message = f"{message} after {attempt} attempts"
            raise RuntimeError(message) from exc
        except (TimeoutError, URLError, OSError) as exc:
            message = f"{url} failed: {_network_error_reason(exc)}"
            if attempt < attempts:
                _maybe_sleep(attempt, attempts=attempts, interval=interval)
                continue
            if attempt > 1:
                message = f"{url} failed after {attempt} attempts: {_network_error_reason(exc)}"
            raise RuntimeError(message) from exc
        if _transient_http_status(status) and attempt < attempts:
            _maybe_sleep(attempt, attempts=attempts, interval=interval)
            continue
        if not 200 <= status < 300:
            message = f"{url} returned HTTP {status}"
            if attempt > 1:
                message = f"{message} after {attempt} attempts"
            raise RuntimeError(message)
        _require(
            allow_redirects or final_url == url,
            f"{url} redirected to {final_url}; update the evidence URL",
        )
        _verify_expected_snippets(url, body)
        return


def verify_service_source_urls(
    rows: list[dict[str, Any]],
    *,
    timeout: float,
    allow_redirects: bool = False,
    attempts: int = 1,
    interval: float = 0.0,
) -> None:
    _require(rows, "service decision is empty")
    _require(attempts >= 1, "attempts must be at least 1")
    _require(interval >= 0, "interval must be non-negative")
    for row in rows:
        role = row.get("role", "<missing role>")
        urls = row.get("evidence_urls")
        _require(
            isinstance(urls, list) and urls,
            f"service decision row {role!r} has no evidence URLs",
        )
        for url in urls:
            _require(
                isinstance(url, str) and url.startswith("https://"),
                f"service decision row {role!r} has invalid URL {url!r}",
            )
            _verify_url(
                url,
                timeout=timeout,
                allow_redirects=allow_redirects,
                attempts=attempts,
                interval=interval,
            )


def main() -> int:
    args = parse_args()
    try:
        verify_service_source_urls(
            service_decision(),
            timeout=args.timeout,
            allow_redirects=args.allow_redirects,
            attempts=args.attempts,
            interval=args.interval,
        )
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from exc
    print("performance dashboard service source URLs verified")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
