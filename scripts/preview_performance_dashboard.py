#!/usr/bin/env python3
"""Render a local preview of the DART 6 performance dashboard."""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from benchmark_display_names import family_of, humanize_name  # noqa: E402

REPO_URL = "https://github.com/dartsim/dart"


def _git(*args: str) -> str:
    try:
        return subprocess.run(
            ["git", *args],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return ""


def _load_results(inputs: list[Path]) -> dict:
    files: list[Path] = []
    for entry in inputs:
        if entry.is_dir():
            files.extend(
                sorted(p for p in entry.glob("*.json") if p.name != "combined.json")
            )
        else:
            files.append(entry)

    benchmarks: list[dict] = []
    for path in files:
        benchmarks.extend(
            json.loads(path.read_text(encoding="utf-8")).get("benchmarks", [])
        )
    return {"benchmarks": benchmarks}


def _to_benches(results: dict) -> list[dict]:
    benches: list[dict] = []
    for row in results.get("benchmarks", []):
        name = row.get("name")
        value = row.get("real_time")
        if name is None or value is None:
            continue
        unit = row.get("time_unit", "ns")
        benches.append(
            {
                "name": humanize_name(name),
                "family": family_of(name),
                "value": value,
                "unit": unit,
                "range": "",
                "extra": (
                    f"cpu_time: {row.get('cpu_time')} {unit}; "
                    f"iterations: {row.get('iterations')}"
                ),
            }
        )
    return benches


def _read_existing(data_js: Path) -> dict:
    if not data_js.is_file():
        return {"lastUpdate": 0, "repoUrl": REPO_URL, "entries": {}}
    text = data_js.read_text(encoding="utf-8")
    match = re.search(r"window\.BENCHMARK_DATA\s*=\s*(\{.*\})\s*;?\s*$", text, re.S)
    if not match:
        return {"lastUpdate": 0, "repoUrl": REPO_URL, "entries": {}}
    return json.loads(match.group(1))


def build_run(results: dict, args: argparse.Namespace) -> dict:
    now_ms = int(time.time() * 1000)
    commit_id = args.commit_id or _git("rev-parse", "HEAD") or "local"
    message = args.commit_message or _git("log", "-1", "--pretty=%s") or "local preview"
    return {
        "commit": {
            "id": commit_id,
            "message": message,
            "timestamp": args.date or _git("log", "-1", "--pretty=%cI"),
            "url": f"{args.repo_url}/commit/{commit_id}",
            "committer": {"username": _git("log", "-1", "--pretty=%an") or "local"},
        },
        "date": now_ms,
        "tool": "googlecpp",
        "benches": _to_benches(results),
    }


INDEX_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>DART 6 Performance Dashboard</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4"></script>
<style>
  :root { color-scheme: light dark; }
  body { font-family: system-ui, sans-serif; margin: 0; background: #0d1117; color: #e6edf3; }
  header { padding: 24px 32px; border-bottom: 1px solid #30363d; }
  header h1 { margin: 0 0 4px; font-size: 22px; }
  header p { margin: 0; color: #8b949e; font-size: 14px; }
  main { padding: 20px 32px; }
  section > h2 { font-size: 16px; margin: 22px 0 4px; }
  .family > h3 { font-size: 13px; font-weight: 600; color: #c9d1d9; margin: 18px 0 10px;
    border-bottom: 1px solid #21262d; padding-bottom: 5px; }
  .grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(320px, 1fr)); gap: 14px; }
  .card { background: #161b22; border: 1px solid #30363d; border-radius: 8px; padding: 12px; }
  .card h4 { margin: 0 0 2px; font-size: 12px; font-weight: 600; }
  .card .val { color: #8b949e; font-size: 12px; margin-bottom: 6px; }
  canvas { max-height: 200px; }
  footer { padding: 16px 32px; color: #8b949e; font-size: 13px; border-top: 1px solid #30363d; }
  a { color: #58a6ff; }
</style>
</head>
<body>
<header>
  <h1>DART 6 Performance Dashboard</h1>
  <p id="meta">Benchmark history for DART 6 LTS, tracked over commits. Lower is better.</p>
</header>
<main id="root"></main>
<footer>
  Local preview of <code>benchmark-action/github-action-benchmark</code>.
  <a id="download" href="#">Download data.js</a>
</footer>
<script src="data.js"></script>
<script>
const COLORS = ["#58a6ff","#3fb950","#f778ba","#d29922","#a371f7","#ff7b72","#39c5cf","#db61a2"];
const data = window.BENCHMARK_DATA || { entries: {}, lastUpdate: 0, repoUrl: "#" };
const root = document.getElementById("root");
const meta = document.getElementById("meta");
if (data.lastUpdate) {
  meta.textContent = "Last updated " + new Date(data.lastUpdate).toUTCString()
    + " - " + (data.repoUrl || "") + " - lower is better";
}
const fmtNum = (v) => (v == null ? "-"
  : Number(v).toLocaleString(undefined, { maximumFractionDigits: 2 }));
for (const [suite, runs] of Object.entries(data.entries)) {
  const section = document.createElement("section");
  const title = document.createElement("h2");
  const allNames = new Set(runs.flatMap(r => r.benches.map(b => b.name)));
  title.textContent = suite + " - " + runs.length + " run(s), " + allNames.size + " benchmarks";
  section.appendChild(title);
  root.appendChild(section);

  const points = runs.map(r => ({
    label: (r.commit && r.commit.id || "").slice(0, 7)
      || (r.date ? new Date(r.date).toLocaleDateString() : "local"),
    id: (r.commit && r.commit.id) || "",
    msg: (r.commit && r.commit.message) || "",
    when: (r.commit && r.commit.timestamp)
      || (r.date ? new Date(r.date).toISOString() : ""),
  }));
  const labels = points.map(p => p.label);

  const families = [];
  for (const r of runs) for (const b of r.benches) {
    const fam = b.family || "Other benchmarks";
    if (!families.includes(fam)) families.push(fam);
  }

  let colorIndex = 0;
  for (const family of families) {
    const names = [...new Set(runs.flatMap(r =>
      r.benches.filter(b => (b.family || "Other benchmarks") === family).map(b => b.name)))];
    if (!names.length) continue;

    const famWrap = document.createElement("div");
    famWrap.className = "family";
    const h3 = document.createElement("h3");
    h3.textContent = family;
    famWrap.appendChild(h3);
    const grid = document.createElement("div");
    grid.className = "grid";
    famWrap.appendChild(grid);
    section.appendChild(famWrap);

    names.forEach((name) => {
      const series = runs.map(r => {
        const b = r.benches.find(x => x.name === name);
        return b ? b.value : null;
      });
      const unit = (runs.map(r => r.benches.find(x => x.name === name)).find(Boolean) || {}).unit || "";
      const latest = [...series].reverse().find(v => v != null);
      const color = COLORS[colorIndex++ % COLORS.length];
      const card = document.createElement("div");
      card.className = "card";
      const h4 = document.createElement("h4");
      h4.textContent = name;
      const val = document.createElement("div");
      val.className = "val";
      val.textContent = "latest: " + fmtNum(latest) + " " + unit;
      const canvas = document.createElement("canvas");
      card.append(h4, val, canvas);
      grid.appendChild(card);
      new Chart(canvas, {
        type: "line",
        data: { labels, datasets: [{
          label: name,
          data: series,
          borderColor: color,
          backgroundColor: color,
          tension: 0.2,
          spanGaps: true,
          pointRadius: 3,
        }] },
        options: {
          responsive: true,
          interaction: { mode: "index", intersect: false },
          plugins: {
            legend: { display: false },
            tooltip: {
              callbacks: {
                title: (items) => {
                  const p = points[items[0].dataIndex];
                  return p.id ? (p.id.slice(0, 7) + (p.msg ? " - " + p.msg : "")) : p.label;
                },
                label: (item) => "time: " + fmtNum(item.parsed.y) + " " + unit,
                afterBody: (items) => {
                  const p = points[items[0].dataIndex];
                  return p.when ? "committed: " + p.when : "";
                },
              },
            },
          },
          scales: {
            x: {
              title: { display: true, text: "Commit (oldest to newest)" },
              ticks: { maxRotation: 0, autoSkip: true },
            },
            y: {
              title: { display: true, text: (unit ? unit + " per op" : "value") + " - lower is better" },
              ticks: { callback: (v) => Number(v).toLocaleString() },
            },
          },
        },
      });
    });
  }
}
const dl = document.getElementById("download");
dl.href = URL.createObjectURL(new Blob(
  ["window.BENCHMARK_DATA = " + JSON.stringify(data, null, 2)],
  { type: "text/javascript" },
));
dl.download = "data.js";
</script>
</body>
</html>
"""


def write_dashboard(
    inputs: list[Path],
    output_dir: Path,
    args: argparse.Namespace,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    data_js = output_dir / "data.js"
    data = (
        _read_existing(data_js)
        if args.append
        else {
            "lastUpdate": 0,
            "repoUrl": args.repo_url,
            "entries": {},
        }
    )
    data["repoUrl"] = args.repo_url
    run = build_run(_load_results(inputs), args)
    data["lastUpdate"] = run["date"]
    data["entries"].setdefault(args.suite_name, []).append(run)

    data_js.write_text(
        "window.BENCHMARK_DATA = " + json.dumps(data, indent=2) + ";\n",
        encoding="utf-8",
    )
    (output_dir / "index.html").write_text(INDEX_HTML, encoding="utf-8")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "inputs",
        nargs="+",
        type=Path,
        help="Google Benchmark JSON files or directories to preview.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("build/performance-dashboard"),
        help="Directory to write index.html and data.js.",
    )
    parser.add_argument(
        "--append",
        action="store_true",
        help="Append to an existing data.js history instead of replacing it.",
    )
    parser.add_argument(
        "--suite-name",
        default="DART 6 Performance",
        help="Dashboard suite name.",
    )
    parser.add_argument("--repo-url", default=REPO_URL)
    parser.add_argument("--commit-id", default="")
    parser.add_argument("--commit-message", default="")
    parser.add_argument("--date", default="")
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    write_dashboard(args.inputs, args.output_dir, args)
    print(f"Wrote dashboard preview to {args.output_dir / 'index.html'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
