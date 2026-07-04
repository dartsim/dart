#!/usr/bin/env python3
"""Generate the public DART community-signals dashboard."""

from __future__ import annotations

import argparse
import datetime as dt
import html
import json
import os
import re
import sys
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Any

UTC = dt.timezone.utc
OWNER = "dartsim"
REPO = "dart"
GITHUB_API = "https://api.github.com"


def _token() -> str | None:
    for name in ("DART_STATS_GITHUB_TOKEN", "GITHUB_TOKEN", "GH_TOKEN"):
        value = os.environ.get(name)
        if value:
            return value
    return None


def _request(
    url: str,
    *,
    token: str | None = None,
    accept: str = "application/json",
    timeout: int = 30,
) -> tuple[bytes, dict[str, str]]:
    headers = {
        "Accept": accept,
        "User-Agent": "dartsim-community-signals",
    }
    if token:
        headers["Authorization"] = f"Bearer {token}"
        headers["X-GitHub-Api-Version"] = "2022-11-28"
    request = urllib.request.Request(url, headers=headers)
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return response.read(), dict(response.headers.items())


def _json(
    url: str,
    *,
    token: str | None = None,
    timeout: int = 30,
) -> Any:
    body, _ = _request(url, token=token, timeout=timeout)
    return json.loads(body.decode("utf-8"))


def _text(
    url: str,
    *,
    token: str | None = None,
    timeout: int = 30,
) -> str:
    body, _ = _request(url, token=token, accept="*/*", timeout=timeout)
    return body.decode("utf-8", errors="replace")


def _github(path: str, *, token: str | None) -> Any:
    return _json(f"{GITHUB_API}/{path.lstrip('/')}", token=token)


def _next_link(link_header: str | None) -> str | None:
    if not link_header:
        return None
    for part in link_header.split(","):
        match = re.search(r'<([^>]+)>;\s*rel="next"', part.strip())
        if match:
            return match.group(1)
    return None


def _github_pages(path: str, *, token: str | None) -> list[Any]:
    url = f"{GITHUB_API}/{path.lstrip('/')}"
    rows: list[Any] = []
    while url:
        body, headers = _request(url, token=token)
        data = json.loads(body.decode("utf-8"))
        if not isinstance(data, list):
            raise TypeError(f"expected paginated list from {url}")
        rows.extend(data)
        url = _next_link(headers.get("Link") or headers.get("link"))
    return rows


def _safe(label: str, func: Any) -> tuple[Any | None, str | None]:
    try:
        return func(), None
    except urllib.error.HTTPError as error:
        return None, f"{label}: HTTP {error.code}"
    except Exception as error:  # noqa: BLE001 - dashboard should degrade.
        return None, f"{label}: {error}"


def _issue_count(query: str, *, token: str | None) -> int | None:
    encoded = urllib.parse.urlencode({"q": query})
    data = _github(f"search/issues?{encoded}", token=token)
    return int(data["total_count"])


def _sum_conda_downloads(package: str) -> tuple[int | None, str | None]:
    data, error = _safe(
        f"conda-forge/{package}",
        lambda: _json(f"https://api.anaconda.org/package/conda-forge/{package}"),
    )
    if error or not isinstance(data, dict):
        return None, error
    files = data.get("files") or []
    return sum(int(file.get("ndownloads") or 0) for file in files), None


def _nested(data: dict[str, Any] | None, path: tuple[str, ...]) -> Any | None:
    current: Any = data
    for key in path:
        if not isinstance(current, dict):
            return None
        current = current.get(key)
    return current


def _parse_iso(value: str | None) -> dt.datetime | None:
    if not value:
        return None
    try:
        return dt.datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        return None


def _date(value: str | None) -> str:
    parsed = _parse_iso(value)
    if parsed is None:
        return "unavailable"
    return parsed.date().isoformat()


def _month_day_year(value: str | None) -> str:
    parsed = _parse_iso(value)
    if parsed is None:
        return "unavailable"
    return parsed.strftime("%b %-d, %Y")


def _number(value: Any | None) -> str:
    if value is None:
        return "unavailable"
    try:
        return f"{int(value):,}"
    except TypeError:
        return str(value)
    except ValueError:
        return str(value)


def _compact(value: Any | None) -> str:
    if value is None:
        return "unavailable"
    try:
        number = int(value)
    except TypeError:
        return str(value)
    except ValueError:
        return str(value)
    if number >= 1_000_000:
        return f"{number / 1_000_000:.1f}M"
    if number >= 1_000:
        return f"{number / 1_000:.0f}K"
    return str(number)


def _plain_text(value: str | None) -> str:
    text = re.sub(r"<[^>]+>", " ", value or "")
    return re.sub(r"\s+", " ", text)


def _traffic_window(payload: dict[str, Any] | None, key: str) -> str:
    rows = (payload or {}).get(key) or []
    if not rows:
        return "latest 14-day GitHub window"
    first = _date(rows[0].get("timestamp"))
    last = _date(rows[-1].get("timestamp"))
    return f"{first} to {last}"


def _freshness(pushed_at: str | None, now: dt.datetime) -> str:
    parsed = _parse_iso(pushed_at)
    if parsed is None:
        return "unknown"
    age_days = (now - parsed).days
    if age_days < 365:
        return "current"
    if age_days < 365 * 3:
        return "older"
    if age_days < 365 * 5:
        return "3y+ historical"
    return "5y+ historical"


def _years_ago(pushed_at: str | None, now: dt.datetime) -> str:
    parsed = _parse_iso(pushed_at)
    if parsed is None:
        return "unknown"
    years = (now - parsed).days / 365.25
    if years < 1:
        return "within 1 year"
    return f"{years:.1f} years ago"


def collect() -> dict[str, Any]:
    token = _token()
    now = dt.datetime.now(UTC)
    since = (now - dt.timedelta(days=365)).date().isoformat()
    errors: list[str] = []

    def capture(label: str, func: Any) -> Any | None:
        data, error = _safe(label, func)
        if error:
            errors.append(error)
        return data

    repo = capture("github repo", lambda: _github(f"repos/{OWNER}/{REPO}", token=token))
    latest_release = capture(
        "latest release",
        lambda: _github(f"repos/{OWNER}/{REPO}/releases/latest", token=token),
    )
    releases = capture(
        "releases",
        lambda: _github_pages(
            f"repos/{OWNER}/{REPO}/releases?per_page=100", token=token
        ),
    )
    contributors = capture(
        "contributors",
        lambda: _github_pages(
            f"repos/{OWNER}/{REPO}/contributors?per_page=100", token=token
        ),
    )
    contributors_anon = capture(
        "contributors anon",
        lambda: _github_pages(
            f"repos/{OWNER}/{REPO}/contributors?anon=true&per_page=100",
            token=token,
        ),
    )
    traffic_clones = capture(
        "traffic clones",
        lambda: _github(f"repos/{OWNER}/{REPO}/traffic/clones", token=token),
    )
    traffic_views = capture(
        "traffic views",
        lambda: _github(f"repos/{OWNER}/{REPO}/traffic/views", token=token),
    )
    open_issues = capture(
        "open issues",
        lambda: _issue_count(f"repo:{OWNER}/{REPO} is:issue state:open", token=token),
    )
    open_prs = capture(
        "open PRs",
        lambda: _issue_count(f"repo:{OWNER}/{REPO} is:pr state:open", token=token),
    )
    created_prs = capture(
        "created PRs",
        lambda: _issue_count(
            f"repo:{OWNER}/{REPO} is:pr created:>={since}", token=token
        ),
    )
    merged_prs = capture(
        "merged PRs",
        lambda: _issue_count(
            f"repo:{OWNER}/{REPO} is:pr merged:>={since}", token=token
        ),
    )
    created_issues = capture(
        "created issues",
        lambda: _issue_count(
            f"repo:{OWNER}/{REPO} is:issue created:>={since}", token=token
        ),
    )

    conda_downloads: dict[str, int | None] = {}
    for package in ("dartsim", "dartpy", "dartsim-cpp"):
        count, error = _sum_conda_downloads(package)
        conda_downloads[package] = count
        if error:
            errors.append(error)

    homebrew = capture(
        "homebrew", lambda: _json("https://formulae.brew.sh/api/formula/dartsim.json")
    )
    pepy_html = capture(
        "pepy", lambda: _text("https://pepy.tech/project/dartpy", timeout=45)
    )
    pepy_count = None
    if isinstance(pepy_html, str):
        match = re.search(r'"userInteractionCount"\s*:\s*(\d+)', pepy_html)
        if match:
            pepy_count = int(match.group(1))

    crossref = capture(
        "crossref", lambda: _json("https://api.crossref.org/works/10.21105/joss.00500")
    )
    openalex = capture(
        "openalex",
        lambda: _json(
            "https://api.openalex.org/works/https://doi.org/10.21105/joss.00500"
        ),
    )
    semanticscholar = capture(
        "semantic scholar",
        lambda: _json(
            "https://api.semanticscholar.org/graph/v1/paper/"
            "DOI:10.21105/joss.00500?fields=title,year,venue,"
            "citationCount,influentialCitationCount"
        ),
    )

    gazebo_docs = capture(
        "gazebo physics docs",
        lambda: _text("https://gazebosim.org/api/sim/9/physics.html"),
    )
    gz_package = capture(
        "gz-physics package.xml",
        lambda: _text(
            "https://raw.githubusercontent.com/gazebosim/gz-physics/main/package.xml"
        ),
    )
    gz_cmake = capture(
        "gz-physics CMakeLists",
        lambda: _text(
            "https://raw.githubusercontent.com/gazebosim/gz-physics/main/CMakeLists.txt"
        ),
    )
    gz_tree = capture(
        "gz-physics tree",
        lambda: _github(
            "repos/gazebosim/gz-physics/git/trees/main?recursive=1",
            token=token,
        ),
    )
    gz_dartsim_files = None
    if isinstance(gz_tree, dict):
        gz_dartsim_files = sum(
            1
            for item in gz_tree.get("tree", [])
            if item.get("type") == "blob"
            and str(item.get("path", "")).startswith("dartsim/src/")
        )

    downstream_specs = [
        ("Gazebo gz-sim", "gazebosim/gz-sim"),
        ("Gazebo gz-physics", "gazebosim/gz-physics"),
        ("RobotDART", "NOSALRO/robot_dart"),
        ("gym-dart", "dartsim/gym-dart"),
        ("Aikido", "personalrobotics/aikido"),
        ("Nimble Physics", "keenon/nimblephysics"),
        ("MASS", "lsw9021/MASS"),
        ("SimBenchmark", "leggedrobotics/SimBenchmark"),
        ("pydart2", "sehoonha/pydart2"),
    ]
    downstream = []
    for label, full_name in downstream_specs:
        data = capture(
            f"downstream {full_name}",
            lambda name=full_name: _github(f"repos/{name}", token=token),
        )
        if not isinstance(data, dict):
            downstream.append(
                {
                    "label": label,
                    "repo": full_name,
                    "stars": None,
                    "pushed_at": None,
                    "freshness": "unknown",
                    "age": "unknown",
                }
            )
            continue
        downstream.append(
            {
                "label": label,
                "repo": full_name,
                "stars": data.get("stargazers_count"),
                "pushed_at": data.get("pushed_at"),
                "freshness": _freshness(data.get("pushed_at"), now),
                "age": _years_ago(data.get("pushed_at"), now),
            }
        )

    release_count_12m = 0
    if isinstance(releases, list):
        since_dt = dt.datetime.combine(
            dt.date.fromisoformat(since), dt.time(), tzinfo=UTC
        )
        release_count_12m = sum(
            1
            for release in releases
            if (
                _parse_iso(release.get("published_at"))
                or dt.datetime.min.replace(tzinfo=UTC)
            )
            >= since_dt
        )

    conda_combined = sum(value or 0 for value in conda_downloads.values())
    crossref_count = _nested(crossref, ("message", "is-referenced-by-count"))
    openalex_count = _nested(openalex, ("cited_by_count",))
    semantic_scholar_count = _nested(semanticscholar, ("citationCount",))
    indexed_citations = [crossref_count, openalex_count, semantic_scholar_count]
    citation_values = [int(value) for value in indexed_citations if value is not None]

    return {
        "generated_at": now.isoformat().replace("+00:00", "Z"),
        "since": since,
        "sources": {
            "github_repo": f"{GITHUB_API}/repos/{OWNER}/{REPO}",
            "github_latest_release": f"{GITHUB_API}/repos/{OWNER}/{REPO}/releases/latest",
            "github_issues": f"{GITHUB_API}/search/issues?q=repo%3A{OWNER}%2F{REPO}+is%3Aissue+state%3Aopen",
            "github_prs": f"{GITHUB_API}/search/issues?q=repo%3A{OWNER}%2F{REPO}+is%3Apr+state%3Aopen",
            "github_traffic_docs": "https://docs.github.com/en/rest/metrics/traffic",
            "anaconda_dartsim": "https://api.anaconda.org/package/conda-forge/dartsim",
            "anaconda_dartpy": "https://api.anaconda.org/package/conda-forge/dartpy",
            "anaconda_dartsim_cpp": "https://api.anaconda.org/package/conda-forge/dartsim-cpp",
            "pepy_dartpy": "https://pepy.tech/project/dartpy",
            "homebrew_dartsim": "https://formulae.brew.sh/api/formula/dartsim.json",
            "joss": "https://joss.theoj.org/papers/10.21105/joss.00500",
            "crossref": "https://api.crossref.org/works/10.21105/joss.00500",
            "openalex": "https://api.openalex.org/works/https://doi.org/10.21105/joss.00500",
            "semantic_scholar": "https://api.semanticscholar.org/graph/v1/paper/DOI:10.21105/joss.00500?fields=title,year,venue,citationCount,influentialCitationCount",
            "gazebo_physics": "https://gazebosim.org/api/sim/9/physics.html",
            "gz_physics_package": "https://raw.githubusercontent.com/gazebosim/gz-physics/main/package.xml",
            "gz_physics_cmake": "https://raw.githubusercontent.com/gazebosim/gz-physics/main/CMakeLists.txt",
            "gz_physics_tree": f"{GITHUB_API}/repos/gazebosim/gz-physics/git/trees/main?recursive=1",
        },
        "summary": {
            "latest_release": (
                (latest_release or {}).get("tag_name")
                if isinstance(latest_release, dict)
                else None
            ),
            "latest_release_date": _date(
                (latest_release or {}).get("published_at")
                if isinstance(latest_release, dict)
                else None
            ),
            "latest_push": _date(
                (repo or {}).get("pushed_at") if isinstance(repo, dict) else None
            ),
            "stars": (
                (repo or {}).get("stargazers_count") if isinstance(repo, dict) else None
            ),
            "forks": (
                (repo or {}).get("forks_count") if isinstance(repo, dict) else None
            ),
            "watchers": (
                (repo or {}).get("subscribers_count")
                if isinstance(repo, dict)
                else None
            ),
            "open_issues": open_issues,
            "open_prs": open_prs,
            "created_issues_12m": created_issues,
            "created_prs_12m": created_prs,
            "merged_prs_12m": merged_prs,
            "releases_12m": release_count_12m,
            "contributors_login": (
                len(contributors) if isinstance(contributors, list) else None
            ),
            "contributors_with_anon": (
                len(contributors_anon) if isinstance(contributors_anon, list) else None
            ),
        },
        "traffic": {
            "available": isinstance(traffic_clones, dict)
            and isinstance(traffic_views, dict),
            "clones": (
                (traffic_clones or {}).get("count")
                if isinstance(traffic_clones, dict)
                else None
            ),
            "unique_cloners": (
                (traffic_clones or {}).get("uniques")
                if isinstance(traffic_clones, dict)
                else None
            ),
            "views": (
                (traffic_views or {}).get("count")
                if isinstance(traffic_views, dict)
                else None
            ),
            "unique_visitors": (
                (traffic_views or {}).get("uniques")
                if isinstance(traffic_views, dict)
                else None
            ),
            "clone_window": (
                _traffic_window(traffic_clones, "clones")
                if isinstance(traffic_clones, dict)
                else None
            ),
            "view_window": (
                _traffic_window(traffic_views, "views")
                if isinstance(traffic_views, dict)
                else None
            ),
        },
        "packages": {
            "conda": {
                **conda_downloads,
                "combined_gross": conda_combined,
            },
            "pepy_dartpy_all_time": pepy_count,
            "homebrew_installs_365d": _nested(
                homebrew, ("analytics", "install", "365d", "dartsim")
            ),
            "homebrew_on_request_365d": _nested(
                homebrew, ("analytics", "install_on_request", "365d", "dartsim")
            ),
        },
        "citations": {
            "crossref": crossref_count,
            "openalex": openalex_count,
            "semantic_scholar": semantic_scholar_count,
            "semantic_scholar_influential": _nested(
                semanticscholar, ("influentialCitationCount",)
            ),
            "indexed_min": min(citation_values) if citation_values else None,
            "indexed_max": max(citation_values) if citation_values else None,
        },
        "gazebo": {
            "docs_default_dart": bool(
                re.search(
                    r"DART\s+physics\s+engine.*default",
                    _plain_text(gazebo_docs),
                    re.IGNORECASE,
                )
            ),
            "package_depends_dart": "<depend>DART</depend>" in (gz_package or ""),
            "requires_dart_610": "VERSION 6.10" in (gz_cmake or ""),
            "dartsim_plugin_files": gz_dartsim_files,
        },
        "downstream": downstream,
        "errors": errors,
    }


def _link(url: str, text: str) -> str:
    return f'<a href="{html.escape(url)}">{html.escape(text)}</a>'


SourceLinks = str | list[tuple[str, str]]


def _source_links(sources: SourceLinks | None) -> str:
    if sources is None:
        return ""
    if isinstance(sources, str):
        return f'<a href="{html.escape(sources)}">source</a>'
    return " · ".join(_link(url, label) for url, label in sources)


def _card(
    label: str,
    value: str,
    note: str,
    sources: SourceLinks | None = None,
) -> str:
    return f"""
      <article class="metric-card">
        <div class="metric-label">{html.escape(label)}</div>
        <div class="metric-value">{html.escape(value)}</div>
        <div class="metric-note">{html.escape(note)}</div>
        <div class="metric-source">{_source_links(sources)}</div>
      </article>
    """


def _downstream_rows(data: dict[str, Any]) -> str:
    rows = []
    for item in data["downstream"]:
        repo = str(item["repo"])
        rows.append(
            "<tr>"
            f"<td>{_link(f'https://github.com/{repo}', str(item['label']))}</td>"
            f"<td>{_number(item.get('stars'))}</td>"
            f"<td>{html.escape(_month_day_year(item.get('pushed_at')))}</td>"
            f"<td><span class='tag'>{html.escape(str(item.get('freshness')))}</span></td>"
            "</tr>"
        )
    return "\n".join(rows)


def render_html(data: dict[str, Any]) -> str:
    summary = data["summary"]
    traffic = data["traffic"]
    packages = data["packages"]
    citations = data["citations"]
    gazebo = data["gazebo"]
    sources = data["sources"]
    generated = _parse_iso(data["generated_at"])
    generated_text = (
        generated.strftime("%b %-d, %Y %H:%M UTC")
        if generated
        else data["generated_at"]
    )

    traffic_value = (
        f"{_compact(traffic['unique_cloners'])} unique cloners"
        if traffic["available"]
        else "token required"
    )
    traffic_note = (
        f"{_number(traffic['clones'])} clone events, {traffic['clone_window']}"
        if traffic["available"]
        else "Repository traffic requires authenticated collection."
    )
    citation_range = (
        f"{_number(citations['indexed_min'])}-{_number(citations['indexed_max'])}"
        if citations["indexed_min"] is not None
        else "unavailable"
    )
    gazebo_note_parts = []
    if gazebo["docs_default_dart"]:
        gazebo_note_parts.append("default physics-engine docs")
    if gazebo["package_depends_dart"]:
        gazebo_note_parts.append("declares DART dependency")
    if gazebo["requires_dart_610"]:
        gazebo_note_parts.append("requires DART >= 6.10")
    if gazebo["dartsim_plugin_files"] is not None:
        gazebo_note_parts.append(
            f"{gazebo['dartsim_plugin_files']} plugin source files"
        )
    gazebo_note = "; ".join(gazebo_note_parts) or "source checks unavailable"

    cards = "\n".join(
        [
            _card(
                "Latest release",
                str(summary["latest_release"] or "unavailable"),
                f"published {summary['latest_release_date']}; latest push {summary['latest_push']}",
                [
                    ("https://github.com/dartsim/dart/releases/latest", "release"),
                    (sources["github_repo"], "repo"),
                ],
            ),
            _card(
                "GitHub interest",
                f"{_compact(summary['stars'])} stars",
                f"{_number(summary['forks'])} forks; {_number(summary['watchers'])} watchers",
                sources["github_repo"],
            ),
            _card(
                "Recent cloning",
                traffic_value,
                traffic_note,
                sources["github_traffic_docs"],
            ),
            _card(
                "Open backlog",
                f"{_number(summary['open_issues'])} issues / {_number(summary['open_prs'])} PRs",
                f"{_number(summary['created_prs_12m'])} PRs created since {data['since']}",
                [
                    (sources["github_issues"], "issues"),
                    (sources["github_prs"], "PRs"),
                ],
            ),
            _card(
                "Package reach",
                f"{_compact(packages['conda']['dartsim'])} conda downloads",
                f"{_compact(packages['pepy_dartpy_all_time'])} PePy dartpy; "
                f"{_compact(packages['homebrew_installs_365d'])} Homebrew installs/year",
                [
                    (sources["anaconda_dartsim"], "conda"),
                    (sources["pepy_dartpy"], "PePy"),
                    (sources["homebrew_dartsim"], "Homebrew"),
                ],
            ),
            _card(
                "Research citations",
                citation_range,
                "indexed citations across Crossref, OpenAlex, and Semantic Scholar",
                [
                    (sources["crossref"], "Crossref"),
                    (sources["openalex"], "OpenAlex"),
                    (sources["semantic_scholar"], "Semantic Scholar"),
                ],
            ),
            _card(
                "Gazebo signal",
                "public dependency path",
                gazebo_note,
                [
                    (sources["gazebo_physics"], "docs"),
                    (sources["gz_physics_package"], "package"),
                    (sources["gz_physics_cmake"], "CMake"),
                    (sources["gz_physics_tree"], "tree"),
                ],
            ),
            _card(
                "Contributors",
                f"{_number(summary['contributors_login'])} GitHub / "
                f"{_number(summary['contributors_with_anon'])} total",
                "GitHub attribution including anonymous/email-only entries",
                "https://github.com/dartsim/dart/graphs/contributors",
            ),
        ]
    )

    error_summary = ""
    if data["errors"]:
        escaped_errors = "".join(
            f"<li>{html.escape(error)}</li>" for error in data["errors"][:8]
        )
        more = ""
        if len(data["errors"]) > 8:
            more = f"<li>{len(data['errors']) - 8} additional collection warnings omitted.</li>"
        error_summary = f"""
        <details class="warnings">
          <summary>Collection warnings</summary>
          <ul>{escaped_errors}{more}</ul>
        </details>
        """

    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>DART Community Signals</title>
  <style>
    :root {{
      color-scheme: light;
      --bg: #f6f7f9;
      --panel: #ffffff;
      --text: #17202a;
      --muted: #5e6b78;
      --line: #d9dee5;
      --accent: #167f7a;
      --accent-2: #8a5a00;
      --good: #1d7f43;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      background: var(--bg);
      color: var(--text);
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      line-height: 1.5;
    }}
    main {{
      width: min(1120px, calc(100% - 32px));
      margin: 0 auto;
      padding: 28px 0 40px;
    }}
    header {{
      display: grid;
      gap: 10px;
      margin-bottom: 22px;
    }}
    h1 {{
      margin: 0;
      font-size: clamp(1.7rem, 3vw, 2.4rem);
      letter-spacing: 0;
    }}
    h2 {{
      margin: 28px 0 10px;
      font-size: 1.15rem;
      letter-spacing: 0;
    }}
    p {{ margin: 0; }}
    a {{ color: #0c5fb3; }}
    .subhead {{
      max-width: 780px;
      color: var(--muted);
      font-size: 1rem;
    }}
    .updated {{
      color: var(--muted);
      font-size: 0.92rem;
    }}
    .metric-grid {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
      gap: 12px;
    }}
    .metric-card {{
      min-height: 150px;
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 16px;
      display: flex;
      flex-direction: column;
      gap: 8px;
    }}
    .metric-label {{
      color: var(--muted);
      font-size: 0.82rem;
      font-weight: 700;
      text-transform: uppercase;
    }}
    .metric-value {{
      font-size: 1.55rem;
      font-weight: 750;
      line-height: 1.15;
    }}
    .metric-note {{
      color: var(--muted);
      font-size: 0.95rem;
    }}
    .metric-source {{
      margin-top: auto;
      font-size: 0.86rem;
    }}
    .summary-band {{
      margin-top: 16px;
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(260px, 1fr));
      gap: 12px;
    }}
    .callout {{
      background: #eef7f6;
      border: 1px solid #bddbd7;
      border-left: 4px solid var(--accent);
      border-radius: 8px;
      padding: 14px 16px;
    }}
    .callout strong {{ color: #0f5956; }}
    .boundary {{
      background: #fff8e8;
      border-color: #ecd28f;
      border-left-color: var(--accent-2);
    }}
    table {{
      width: 100%;
      border-collapse: collapse;
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      overflow: hidden;
    }}
    th, td {{
      padding: 10px 12px;
      border-bottom: 1px solid var(--line);
      text-align: left;
      vertical-align: top;
    }}
    th {{
      color: var(--muted);
      font-size: 0.82rem;
      text-transform: uppercase;
    }}
    tr:last-child td {{ border-bottom: 0; }}
    .tag {{
      display: inline-block;
      border: 1px solid #ccd6e0;
      border-radius: 999px;
      padding: 2px 8px;
      font-size: 0.82rem;
      color: #24313d;
      background: #f8fafc;
      white-space: nowrap;
    }}
    .warnings {{
      margin-top: 20px;
      color: var(--muted);
      font-size: 0.9rem;
    }}
    footer {{
      margin-top: 26px;
      color: var(--muted);
      font-size: 0.9rem;
    }}
  </style>
</head>
<body>
  <main>
    <header>
      <h1>DART Community Signals</h1>
      <p class="subhead">
        A compact view of public project activity across GitHub, packages,
        citations, and downstream robotics/research projects.
      </p>
      <p class="updated">Last refreshed {html.escape(generated_text)}.</p>
    </header>

    <section class="metric-grid" aria-label="Community signal metrics">
      {cards}
    </section>

    <section class="summary-band" aria-label="Interpretation">
      <p class="callout">
        <strong>Readable headline:</strong> DART has active maintenance and
        thousands of recent activity touchpoints across repository traffic,
        package ecosystems, citation indexes, and downstream projects.
      </p>
      <p class="callout boundary">
        <strong>Boundary:</strong> these are usage and activity signals, not a
        de-duplicated count of human users.
      </p>
    </section>

    <h2>Downstream freshness</h2>
    <table>
      <thead>
        <tr>
          <th>Project</th>
          <th>Stars</th>
          <th>Last push</th>
          <th>Status</th>
        </tr>
      </thead>
      <tbody>
        {_downstream_rows(data)}
      </tbody>
    </table>

    <h2>Source boundaries</h2>
    <p class="subhead">
      GitHub traffic is collected only when the scheduled workflow has a token
      that can read repository traffic. Package and citation counts come from
      public APIs and can drift between refreshes. Downstream rows are public
      evidence examples, not a complete user registry.
    </p>

    {error_summary}

    <footer>
      Data snapshot:
      {_link("data.json", "data.json")} ·
      DART repository:
      {_link("https://github.com/dartsim/dart", "github.com/dartsim/dart")}
    </footer>
  </main>
</body>
</html>
"""


def write_dashboard(data: dict[str, Any], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "data.json").write_text(
        json.dumps(data, indent=2, sort_keys=True) + "\n"
    )
    (output_dir / "index.html").write_text(render_html(data))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("build/community-signals"),
        help="Directory that receives index.html and data.json.",
    )
    args = parser.parse_args()
    data = collect()
    write_dashboard(data, args.output)
    print(f"Wrote {args.output / 'index.html'}")
    print(f"Wrote {args.output / 'data.json'}")
    if data["errors"]:
        print("Collection warnings:", file=sys.stderr)
        for error in data["errors"]:
            print(f"  - {error}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
