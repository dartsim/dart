#!/usr/bin/env python3
"""Record independent review evidence and check outgoing Git branch updates.

Uses Python 3.11+ and the standard library only. The installer copies this
file beside pre-push so historical worktrees cannot silently disable the gate.
Records are local attestations, not authentication or proof of correctness.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
from contextlib import contextmanager
from pathlib import Path

SCHEMA_VERSION = 1
MAX_BYTES = 2 * 1024 * 1024
ID_RE = re.compile(r"[0-9a-f]{64}\Z")
OID_RE = re.compile(r"(?:[0-9a-f]{40}|[0-9a-f]{64})\Z")

PRE_PUSH_TEMPLATE = """\
#!/bin/sh
# DART-MANAGED-PRE-PUSH v1
# Installed with its standalone checker by pixi run install-hooks.
repo_root=$(git rev-parse --show-toplevel) || exit 1
hooks_dir=$(git rev-parse --git-path hooks) || exit 1
case "$hooks_dir" in
    /*|[A-Za-z]:*) ;;
    *) hooks_dir="$repo_root/$hooks_dir" ;;
esac
checker="$hooks_dir/dart-review-gate.py"
if [ ! -f "$checker" ]; then
    echo "DART review gate BLOCKED: installed checker missing; run pixi run install-hooks" >&2
    exit 1
fi
run_checker() {
    runtime="$1"
    shift
    exec "$runtime" -I -c '
import hashlib, pathlib, sys
path, expected = sys.argv[1:3]
try:
    source = pathlib.Path(path).read_bytes()
except OSError as error:
    sys.exit("DART review gate BLOCKED: " + str(error))
if hashlib.sha256(source).hexdigest() != expected:
    sys.exit("DART review gate BLOCKED: installed checker changed; run pixi run install-hooks")
sys.argv = [path, *sys.argv[3:]]
exec(compile(source, path, "exec"), {"__name__": "__main__", "__file__": path})
' "$checker" "@CHECKER_SHA256@" pre-push "$@"
}
if [ -n "${DART_HOOK_PYTHON:-}" ]; then
    if ! "$DART_HOOK_PYTHON" -I -c 'import sys; sys.exit(sys.version_info < (3, 11))' </dev/null >/dev/null 2>&1; then
        echo "DART review gate BLOCKED: DART_HOOK_PYTHON needs Python 3.11+" >&2
        exit 1
    fi
    run_checker "$DART_HOOK_PYTHON" "$@"
fi
for candidate in "$repo_root/.pixi/envs/default/bin/python" "$repo_root/.pixi/envs/default/python.exe" python3 python
do
    if "$candidate" -I -c 'import sys; sys.exit(sys.version_info < (3, 11))' </dev/null >/dev/null 2>&1; then
        run_checker "$candidate" "$@"
    fi
done
echo "DART review gate BLOCKED: Python 3.11+ unavailable; install the Pixi environment" >&2
exit 1
"""


def pre_push_hook(checker: bytes) -> str:
    return PRE_PUSH_TEMPLATE.replace(
        "@CHECKER_SHA256@", hashlib.sha256(checker).hexdigest()
    )


class GateError(Exception):
    """Missing, stale, or inconsistent review evidence."""


# Keep the installed checker parseable on the documented Python 3.11 minimum.
HOOK_INVENTORY_ERRORS = (GateError, OSError, subprocess.SubprocessError)


def require(condition: object, message: str) -> None:
    if not condition:
        raise GateError(message)


def git(root: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(root), *args],
        capture_output=True,
        encoding="utf-8",
        env={**os.environ, "GIT_OPTIONAL_LOCKS": "0"},
        timeout=10,
    )
    if result.returncode:
        raise GateError(f"git {args[0]} failed: {result.stderr.strip()}")
    return result.stdout.strip()


def digest(value: object) -> str:
    encoded = json.dumps(value, sort_keys=True, ensure_ascii=True).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def read_json(path: Path) -> dict:
    require(path.is_file(), f"missing evidence: {path}")
    require(path.stat().st_size <= MAX_BYTES, f"oversized evidence: {path}")
    value = json.loads(path.read_text(encoding="utf-8"))
    require(isinstance(value, dict), f"expected JSON object: {path}")
    return value


def write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    # Commands hold the store lock. Readers cannot observe partial writes.
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def text_field(value: object, label: str) -> str:
    require(isinstance(value, str) and value.strip(), f"missing {label}")
    return value


class Store:
    def __init__(self, root: Path):
        self.root = Path(git(root, "rev-parse", "--show-toplevel"))
        common = Path(git(self.root, "rev-parse", "--git-common-dir"))
        self.path = (self.root / common).resolve() / "dart-review"
        self._candidates: dict[str, dict] = {}

    @contextmanager
    def lock(self):
        self.path.mkdir(parents=True, exist_ok=True)
        lock = self.path / "lock"
        try:
            lock.mkdir()
        except FileExistsError as exc:
            raise GateError(
                f"another review-gate command holds {lock}; after a crash, "
                "verify no command is running before removing that empty directory"
            ) from exc
        try:
            yield
        finally:
            lock.rmdir()

    def candidate_dir(self, candidate_id: str) -> Path:
        require(bool(ID_RE.fullmatch(candidate_id)), "invalid candidate ID")
        return self.path / "candidates" / candidate_id

    def target_path(self, location: str, target: str) -> Path:
        return self.path / "targets" / (digest([location, target]) + ".json")

    def candidate(self, candidate_id: str) -> dict:
        if candidate_id in self._candidates:
            return self._candidates[candidate_id]
        data = read_json(self.candidate_dir(candidate_id) / "candidate.json")
        require(
            data.get("schema_version") == SCHEMA_VERSION, "unsupported candidate schema"
        )
        require(digest(data) == candidate_id, "candidate identity or content changed")
        for name in ("head", "tree", "base", "merge_base"):
            require(
                bool(OID_RE.fullmatch(data.get(name, ""))), f"invalid candidate {name}"
            )
        require(
            git(self.root, "rev-parse", "--verify", data["head"] + "^{tree}")
            == data["tree"],
            "candidate tree does not match its commit",
        )
        require(
            git(self.root, "merge-base", data["base"], data["head"])
            == data["merge_base"],
            "candidate merge base changed",
        )
        require(
            data["base"] == data["merge_base"], "merge the selected base before review"
        )
        self._candidates[candidate_id] = data
        return data

    def target(
        self, location: str, target: str, *, initial: bool = False
    ) -> str | None:
        """The mutable index must identify the unique immutable history tip."""
        matches = {}
        for path in (self.path / "candidates").glob("*/candidate.json"):
            data = read_json(path)
            require(
                digest(data) == path.parent.name,
                "candidate identity or content changed",
            )
            if (data.get("location"), data.get("target")) == (location, target):
                matches[path.parent.name] = data
        index = self.target_path(location, target)
        if not matches:
            require(
                initial and not index.exists(), "missing or mismatched target history"
            )
            return None
        referenced = {data["previous"] for data in matches.values() if data["previous"]}
        require(referenced <= matches.keys(), "candidate history is missing")
        tips = matches.keys() - referenced
        require(len(tips) == 1, "ambiguous candidate history")
        identity = read_json(index).get("candidate")
        require(identity in tips, "target index is empty, stale, or mismatched")
        return identity

    def prepare(self, args: argparse.Namespace) -> str:
        locations = git(
            self.root, "remote", "get-url", "--push", "--all", args.remote
        ).splitlines()
        require(len(locations) == 1, "prepare requires a remote with one push URL")
        target = args.target
        require(
            target.startswith("refs/heads/"), "target must be a full refs/heads/ branch"
        )
        git(self.root, "check-ref-format", target)
        base_ref = git(self.root, "rev-parse", "--symbolic-full-name", args.base)
        require(
            base_ref.startswith("refs/remotes/"),
            "base must name a fetched remote-tracking branch",
        )
        base = git(self.root, "rev-parse", "--verify", base_ref + "^{commit}")
        head = git(self.root, "rev-parse", "--verify", args.head + "^{commit}")
        merge_base = git(self.root, "merge-base", base, head)
        require(
            merge_base == base,
            "merge the fetched base into the candidate before review",
        )
        authors = sorted(
            {text_field(author, "author session") for author in args.author_session}
        )
        target_path = self.target_path(locations[0], target)
        previous = self.target(locations[0], target, initial=True)
        if previous:
            prior = self.candidate(previous)
            require(
                prior["location"] == locations[0] and prior["target"] == target,
                "target history does not match publication destination",
            )
            authors = sorted(set(authors) | set(prior["authors"]))
            if (prior["head"], prior["base_ref"], prior["base"], prior["authors"]) == (
                head,
                base_ref,
                base,
                authors,
            ):
                return previous
        data = {
            "schema_version": SCHEMA_VERSION,
            "location": locations[0],
            "target": target,
            "base_ref": base_ref,
            "base": base,
            "head": head,
            "tree": git(self.root, "rev-parse", head + "^{tree}"),
            "merge_base": merge_base,
            "authors": authors,
            "previous": previous,
        }
        candidate_id = digest(data)
        write_json(self.candidate_dir(candidate_id) / "candidate.json", data)
        write_json(self.candidate_dir(candidate_id) / "reports.json", {"reports": []})
        write_json(target_path, {"candidate": candidate_id})
        return candidate_id

    def reports(self, candidate_id: str) -> list[dict]:
        directory = self.candidate_dir(candidate_id) / "reports"
        names = read_json(directory.parent / "reports.json").get("reports")
        require(
            isinstance(names, list) and all(isinstance(name, str) for name in names),
            "invalid report manifest",
        )
        require(
            names == sorted(path.name for path in directory.glob("*.json")),
            "review report missing or unindexed",
        )
        reports = []
        for number, name in enumerate(names, 1):
            path = directory / name
            report = read_json(path)
            require(
                path.name == f"{number:06d}-{digest(report)}.json",
                "review report missing or changed",
            )
            self.validate_report(report, candidate_id)
            reports.append(report)
        return reports

    def validate_report(self, report: dict, candidate_id: str) -> None:
        candidate = self.candidate(candidate_id)
        require(
            report.get("schema_version") == SCHEMA_VERSION, "unsupported report schema"
        )
        require(
            report.get("candidate") == candidate_id,
            "report reviews a different candidate",
        )
        reviewer = report.get("reviewer")
        require(isinstance(reviewer, dict), "missing reviewer identity")
        session = text_field(reviewer.get("session"), "reviewer session")
        require(
            session not in candidate["authors"],
            "an authoring session cannot review its own change",
        )
        require(
            reviewer.get("kind") in ("human", "agent"),
            "reviewer kind must be human or agent",
        )
        if reviewer["kind"] == "agent":
            for name in ("tool", "model", "effort"):
                text_field(reviewer.get(name), f"effective reviewer {name}")
        require(
            report.get("scope") in ("correctness", "contracts", "non-substantive"),
            "invalid review scope",
        )
        require(
            report.get("status") in ("complete", "incomplete"),
            "invalid completion status",
        )
        require(
            report.get("verdict") in ("clean", "findings"), "invalid review verdict"
        )
        require(
            isinstance(report.get("coverage_complete"), bool),
            "missing acceptance coverage assessment",
        )
        if report["status"] == "complete" and report["verdict"] == "clean":
            require(
                report["coverage_complete"],
                "missing acceptance coverage cannot count as clean",
            )
        text_field(report.get("summary"), "review summary")
        text_field(report.get("report"), "final reviewer report")
        coverage = report.get("coverage")
        require(isinstance(coverage, list) and coverage, "missing reviewed coverage")
        for item in coverage:
            text_field(item, "coverage item")
        findings = report.get("findings")
        dispositions = report.get("dispositions")
        require(
            isinstance(findings, list) and isinstance(dispositions, list),
            "missing finding lists",
        )
        require(
            not findings or report["verdict"] == "findings",
            "a clean report cannot contain open findings",
        )
        for finding in findings:
            require(isinstance(finding, dict), "invalid finding")
            for name in ("id", "summary", "evidence"):
                text_field(finding.get(name), f"finding {name}")
        for disposition in dispositions:
            require(isinstance(disposition, dict), "invalid disposition")
            text_field(disposition.get("id"), "disposition finding ID")
            text_field(disposition.get("evidence"), "disposition evidence")
            require(
                disposition.get("status") in ("fixed", "rejected"),
                "unresolved disposition",
            )
            require(
                report["status"] == "complete",
                "incomplete review cannot close a finding",
            )
        if report["scope"] == "non-substantive":
            require(
                report.get("no_behavior_change") is True,
                "exception must explicitly assess behavior",
            )
            text_field(report.get("reason"), "non-substantive reason")

    def record(self, candidate_id: str, report_path: Path) -> None:
        candidate = self.candidate(candidate_id)
        active = self.target(candidate["location"], candidate["target"])
        cursor = active
        seen = set()
        while cursor and cursor != candidate_id:
            require(cursor not in seen, "invalid candidate history")
            seen.add(cursor)
            cursor = self.candidate(cursor)["previous"]
        require(
            cursor == candidate_id, "report is outside the active candidate history"
        )
        report = read_json(report_path)
        self.validate_report(report, candidate_id)
        reports = self.reports(candidate_id)
        require(report not in reports, "this report is already recorded")
        # Validate semantics before publication. Late ancestor findings remain
        # actionable; an invalid disposition cannot poison the durable journal.
        self.assess(
            active, current=False, extra=(candidate_id, report), require_pass=False
        )
        path = (
            self.candidate_dir(candidate_id)
            / "reports"
            / f"{len(reports) + 1:06d}-{digest(report)}.json"
        )
        write_json(path, report)
        manifest = self.candidate_dir(candidate_id) / "reports.json"
        names = read_json(manifest)["reports"]
        write_json(manifest, {"reports": [*names, path.name]})

    def assert_current(self, candidate_id: str, candidate: dict) -> None:
        require(
            self.target(candidate["location"], candidate["target"]) == candidate_id,
            "a newer candidate supersedes this evidence",
        )
        require(
            git(self.root, "rev-parse", "--verify", candidate["base_ref"] + "^{commit}")
            == candidate["base"],
            "fetched base moved; integrate it and prepare new reviews",
        )

    def assess(
        self,
        candidate_id: str,
        current: bool = True,
        extra: tuple[str, dict] | None = None,
        require_pass: bool = True,
    ) -> str:
        history = []
        seen = set()
        cursor = candidate_id
        while cursor:
            require(
                cursor not in seen and len(seen) < 1000,
                "invalid or excessive candidate history",
            )
            seen.add(cursor)
            candidate = self.candidate(cursor)
            history.append((cursor, candidate))
            cursor = candidate["previous"]
        history.reverse()
        active = history[-1][1]
        if current:
            self.assert_current(candidate_id, active)
        outstanding: dict[str, dict] = {}
        known: dict[str, dict] = {}
        passed: dict[str, dict] = {}
        verdict = "two clean independent reviews are missing"
        for identity, candidate in history:
            require(
                (candidate["location"], candidate["target"])
                == (active["location"], active["target"]),
                "candidate history changed publication target",
            )
            latest = {}
            reports = self.reports(identity)
            if extra and extra[0] == identity:
                reports.append(extra[1])
            for report in reports:
                for finding in report["findings"]:
                    require(
                        finding["id"] not in known or known[finding["id"]] == finding,
                        "a finding ID was reused for a different issue",
                    )
                    known[finding["id"]] = finding
                    outstanding[finding["id"]] = finding
                for disposition in report["dispositions"]:
                    require(
                        disposition["id"] in known,
                        "disposition names an unknown finding",
                    )
                    require(
                        report["status"] == "complete",
                        "incomplete review cannot close a finding",
                    )
                    outstanding.pop(disposition["id"], None)
                latest[(report["reviewer"]["session"], report["scope"])] = report
            clean = [
                report
                for report in latest.values()
                if report["status"] == "complete"
                and report["verdict"] == "clean"
                and report["reviewer"]["session"] not in active["authors"]
            ]
            correctness = {
                r["reviewer"]["session"] for r in clean if r["scope"] == "correctness"
            }
            contracts = {
                r["reviewer"]["session"] for r in clean if r["scope"] == "contracts"
            }
            eligible = any(a != b for a in correctness for b in contracts)
            verdict = "two clean independent reviews are missing"
            if eligible:
                verdict = "two independent local reviews"
            for report in clean:
                if report["scope"] != "non-substantive":
                    continue
                baseline_id = report.get("baseline")
                if baseline_id is None:
                    require(
                        candidate["previous"] is None,
                        "an update exception needs a reviewed baseline",
                    )
                else:
                    require(
                        baseline_id in passed,
                        "exception baseline lacks clean review evidence",
                    )
                    baseline = passed[baseline_id]
                    require(
                        baseline["base"] == candidate["base"],
                        "a changed base needs two new reviews",
                    )
                    require(
                        git(
                            self.root, "merge-base", baseline["head"], candidate["head"]
                        )
                        == baseline["head"],
                        "exception baseline must be an ancestor of the candidate",
                    )
                eligible = True
                verdict = "independent non-substantive assessment"
            if outstanding:
                eligible = False
                verdict = "unresolved findings: " + ", ".join(sorted(outstanding))
            if eligible:
                passed[identity] = candidate
        if require_pass:
            require(candidate_id in passed, verdict)
        return verdict

    def pre_push(self, location: str, payload: bytes) -> None:
        require(len(payload) <= MAX_BYTES, "oversized pre-push input")
        for line in payload.decode("utf-8").splitlines():
            fields = line.split()
            require(len(fields) == 4, "malformed Git pre-push record")
            local_ref, head, target, old = fields
            require(
                bool(OID_RE.fullmatch(head)) and bool(OID_RE.fullmatch(old)),
                "invalid outgoing object ID",
            )
            if set(head) == {"0"} or not target.startswith("refs/heads/"):
                continue
            candidate_id = self.target(location, target)
            candidate = self.candidate(candidate_id)
            require(
                (candidate["location"], candidate["target"]) == (location, target),
                "candidate does not match outgoing destination",
            )
            require(
                candidate["head"] == head,
                f"{target}: outgoing commit differs from reviewed candidate",
            )
            result = self.assess(candidate_id)
            print(f"DART review gate: {target} {head[:12]}: {result}", file=sys.stderr)


def hook_inventory(root: Path) -> dict:
    """Installation diagnosis shared by branch-specific doctors."""
    configured = subprocess.run(
        ["git", "-C", str(root), "config", "--get", "core.hooksPath"],
        capture_output=True,
        text=True,
    ).stdout.strip()
    result = {
        "path": "unavailable",
        "core_hooks_path": configured,
        "installed": False,
        "checker_current": False,
    }
    try:
        hooks = (root / git(root, "rev-parse", "--git-path", "hooks")).resolve()
        hook = hooks / "pre-push"
        checker = hooks / "dart-review-gate.py"
        source = root / "scripts" / "review_gate.py"
        result.update(
            {
                "path": str(hook),
                "installed": hook.is_file()
                and os.access(hook, os.X_OK)
                and source.is_file()
                and hook.read_bytes()
                == pre_push_hook(source.read_bytes()).encode("utf-8"),
                "checker_current": checker.is_file()
                and source.is_file()
                and checker.read_bytes() == source.read_bytes(),
            }
        )
    except HOOK_INVENTORY_ERRORS:
        pass
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    prepare = commands.add_parser(
        "prepare", help="freeze publication identity and carry finding history"
    )
    prepare.add_argument("--base", required=True)
    prepare.add_argument("--head", default="HEAD")
    prepare.add_argument("--remote", required=True)
    prepare.add_argument("--target", required=True)
    prepare.add_argument("--author-session", action="append", required=True)
    record = commands.add_parser(
        "record", help="import a completed reviewer report JSON"
    )
    record.add_argument("candidate")
    record.add_argument("report", type=Path)
    check = commands.add_parser(
        "check", help="check evidence for the current candidate"
    )
    check.add_argument("candidate")
    push = commands.add_parser(
        "pre-push", help="Git hook entrypoint; reads ref updates from stdin"
    )
    push.add_argument("remote")
    push.add_argument("location")
    args = parser.parse_args()
    try:
        store = Store(Path.cwd())
        if args.command == "pre-push":
            payload = sys.stdin.buffer.read(MAX_BYTES + 1)
            require(len(payload) <= MAX_BYTES, "oversized pre-push input")
            with store.lock():
                store.pre_push(args.location, payload)
            # A shell preserves Git's executable/shebang handling on Windows too.
            local = Path(__file__).resolve().parent / "pre-push.local"
            if local.is_file() and os.access(local, os.X_OK):
                return subprocess.run(
                    [
                        "sh",
                        "-c",
                        'exec "$@"',
                        "dart-pre-push",
                        str(local),
                        args.remote,
                        args.location,
                    ],
                    input=payload,
                ).returncode
        else:
            with store.lock():
                if args.command == "prepare":
                    identity = store.prepare(args)
                    print(identity)
                    print(
                        f"Candidate: {store.candidate_dir(identity) / 'candidate.json'}",
                        file=sys.stderr,
                    )
                elif args.command == "record":
                    store.record(args.candidate, args.report)
                    print(
                        "Review report recorded; run review-gate check before publication."
                    )
                else:
                    print("DART review gate passed: " + store.assess(args.candidate))
        return 0
    except (
        GateError,
        OSError,
        ValueError,
        KeyError,
        TypeError,
        subprocess.SubprocessError,
    ) as exc:
        print(f"DART review gate BLOCKED: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
