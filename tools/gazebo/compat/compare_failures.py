#!/usr/bin/env python3
"""Compare a Gazebo compatibility lane run with the lane's expected failures.

Reads the CTest JUnit reports, CTest logs and per-test GoogleTest XML files
that tools/gazebo/compat/lane.sh collects for gz-physics and gz-sim, and
checks that every failure is listed in the lane's expected-failure file,
which is generated from a DART 6.19.4 run. It also enforces `max-seconds`
limits on selected test cases.

A failing CTest test is described by its failing GoogleTest cases when it
exited normally with at least one failing case. A test that crashed, timed
out, did not run, or failed without a failing case is a test-level failure:
its case results are missing, partial or left over from an earlier attempt.
CTest's JUnit report and the GoogleTest XML keep only a repeated test's last
attempt, so the attempts are read from the CTest log: a crash, timeout or
test that did not run on any attempt is a test-level failure as well, and
cases that failed only on an earlier attempt are listed (FLAKY or RETRIED).

Expected-failure file format (one entry per line; `#` starts a comment):

    <suite> <ctest test>                 test-level failure; accepts any
                                         failure of that test
    <suite> <ctest test> <gtest case>    the GoogleTest case fails
    max-seconds <suite> <ctest test> <gtest case> <seconds>
    tests <suite> <count>                the number of CTest tests the
                                         baseline run registered

where <suite> is `gz-physics` or `gz-sim`. Entries carrying an inline
`# accepted: <reason>` comment are reviewed differences from DART 6.19.4;
--write-baseline keeps them when it regenerates the file.

With --base-results (the same lane run on the candidate's base, for example
the release branch before the change), a failure that is not expected but
that the base has too (the identical failure: the same failing case, or for a
test-level failure, a test-level failure of the same test) is reported as BASE
instead of NEW, so a change is gated only on the failures it introduces.

The run must include every test the expected-failure file names, every
case with a max-seconds limit unless its test failed at the test level, at
least as many CTest tests per suite as the baseline run registered, and, with
--base-results, every CTest test the base ran. Otherwise the comparison stops
with an error, as for missing or unreadable results: a run that a test filter,
a renamed test or a test that stopped registering emptied would pass and skip
that coverage. (lane.sh clears GoogleTest's GTEST_* variables, such as
an inherited GTEST_FILTER, before it runs the suites.)
"""

import argparse
import datetime
import math
import os
import pathlib
import platform
import re
import sys
import xml.etree.ElementTree as ET

SUITES = ("gz-physics", "gz-sim")
ACCEPTED = "# accepted:"
# CTest's status for a test that exited with a failure code, in its log and
# JUnit report; crashes, timeouts and tests that did not run carry others.
NORMAL_FAILURE = "Failed"
PASSED = "Passed"
# A test result line of a CTest log, with the attempt's status ("Passed",
# "Failed", "Exception: SegFault", "Timeout", ...); a repeated attempt has no
# "N/M" prefix.
CTEST_RESULT = re.compile(
    r"^\s*(?:\d+/\d+\s+)?Test\s+#\d+:\s+(\S+)\s+\.*\s*(?:\*\*\*)?(.+?)\s+[\d.]+\s+sec\s*$"
)
# CTest's marker after the description of a crash outside its named
# categories ("Subprocess aborted***Exception:", "Bus error***Exception:"),
# reported as "Exception: Subprocess aborted" like "Exception: SegFault".
SIGNAL_EXCEPTION = "***Exception:"
# GoogleTest's line for a failed case in a test's output, which CTest prints
# after each failed attempt: "[  FAILED  ] Suite.Case (12 ms)".
GTEST_FAILED = "[  FAILED  ] "


def describe_test_failure(message):
    """Why a test-level failure has no usable case results."""
    if message == NORMAL_FAILURE:
        return "failed without a failing case"
    return message


class SuiteResults:
    def __init__(self):
        # CTest test -> None if it passed, else CTest's failure message.
        self.tests = {}
        # (CTest test, GoogleTest case) -> True if the case passed.
        self.cases = {}
        self.durations = {}
        # CTest test -> [(status, cases its output reports failed)] for each
        # attempt of a repeated test.
        self.retried = {}

    def failed_cases(self, test):
        return [c for (t, c), ok in self.cases.items() if t == test and not ok]

    def entries(self, suite):
        """The failures of this run as expected-file entries."""
        entries = set()
        for test, message in self.tests.items():
            if message is None:
                continue
            if message != NORMAL_FAILURE or not self.failed_cases(test):
                entries.add((suite, test))
        for (test, case), ok in self.cases.items():
            if not ok:
                entries.add((suite, test, case))
        return entries


def parse_xml(path):
    try:
        return ET.parse(path).getroot()
    except ET.ParseError as error:
        raise ValueError(f"{path}: {error}") from error


def load_suite(results_dir, suite):
    """Return the test, case and retry results of one suite run."""
    junit = results_dir / f"{suite}.junit.xml"
    if not junit.is_file():
        raise FileNotFoundError(f"missing {junit}; run the {suite} tests first")

    results = SuiteResults()
    for case in parse_xml(junit).iter("testcase"):
        problem = case.find("failure")
        if problem is None:
            problem = case.find("error")
        status = case.get("status", "run")
        if problem is not None:
            message = problem.get("message") or status
        elif status != "run":
            message = "Not Run"
        else:
            message = None
        results.tests[case.get("name")] = message

    for xml in sorted((results_dir / f"{suite}-gtest").glob("*.xml")):
        test = xml.stem
        for case in parse_xml(xml).iter("testcase"):
            if (
                case.get("status") == "notrun"
                or case.get("result") in ("skipped", "suppressed")
                or case.find("skipped") is not None
            ):
                continue
            name = f"{case.get('classname')}.{case.get('name')}"
            results.durations[(test, name)] = float(case.get("time", "0"))
            failed = case.find("failure") is not None or case.find("error") is not None
            results.cases[(test, name)] = not failed

    log = results_dir / f"{suite}.log"
    if not log.is_file():
        raise FileNotFoundError(f"missing {log}; run the {suite} tests first")
    attempts = {}
    reported = None  # the failing cases the current attempt's output names
    for line in log.read_text(errors="replace").splitlines():
        match = CTEST_RESULT.match(line)
        if match:
            reported = set()
            status = match.group(2)
            if status.endswith(SIGNAL_EXCEPTION):
                status = f"Exception: {status[: -len(SIGNAL_EXCEPTION)]}"
            attempts.setdefault(match.group(1), []).append((status, reported))
        elif reported is not None and line.startswith(GTEST_FAILED):
            name = line[len(GTEST_FAILED) :].split(" ", 1)[0].rstrip(",")
            if "." in name:  # not the "N tests, listed below" summary
                reported.add(name)
    for test, runs in attempts.items():
        if len(runs) < 2:
            continue
        results.retried[test] = runs
        statuses = [status for status, _ in runs]
        # An earlier attempt crashed, timed out or did not run, and the last
        # attempt, the only one the JUnit report and the XML keep, hides it.
        if results.tests.get(test) in (None, NORMAL_FAILURE) and any(
            status not in (PASSED, NORMAL_FAILURE) for status in statuses
        ):
            results.tests[test] = ", then ".join(statuses)
    return results


class Expected:
    def __init__(self):
        self.entries = set()
        self.max_seconds = {}
        self.accepted = {}
        self.accepted_lines = []
        self.tests = {}


def parse_expected(path):
    expected = Expected()
    if not path.is_file():
        return expected
    for number, raw in enumerate(path.read_text().splitlines(), start=1):
        line = raw.split("#", 1)[0].split()
        if not line:
            continue
        if line[0] == "max-seconds" and len(line) == 5 and line[1] in SUITES:
            expected.max_seconds[(line[1], line[2], line[3])] = float(line[4])
        elif line[0] == "tests" and len(line) == 3 and line[1] in SUITES:
            expected.tests[line[1]] = int(line[2])
        elif line[0] in SUITES and len(line) in (2, 3):
            expected.entries.add(tuple(line))
            if ACCEPTED in raw:
                expected.accepted[tuple(line)] = raw.split(ACCEPTED, 1)[1].strip()
                expected.accepted_lines.append(raw)
        else:
            raise ValueError(f"{path}:{number}: cannot parse '{raw}'")
    return expected


def check_inventory(results, expected, base=None):
    """Raise ValueError if the results lack a test or timed case `expected`
    names, have fewer CTest tests than the baseline run, or lack a CTest test
    that ran on `base`. The cases of a test-level failure are missing or
    stale, so its timed case may be missing; compare reports the failure
    itself."""
    for entry in sorted(expected.entries | expected.max_seconds.keys()):
        suite, test = entry[:2]
        run = results[suite]
        if test not in run.tests:
            missing = entry[:2]
        elif (
            entry in expected.max_seconds
            and entry[1:] not in run.durations
            and entry[:2] not in run.entries(suite)
        ):
            missing = entry
        else:
            continue
        raise ValueError(
            f"{' '.join(missing)} is missing from the results but named in "
            "the expected failures (a test filter such as GTEST_FILTER, or a "
            "renamed test?)"
        )
    for suite, count in sorted(expected.tests.items()):
        if len(results[suite].tests) < count:
            raise ValueError(
                f"{suite} ran {len(results[suite].tests)} CTest tests, fewer "
                f"than the {count} of the baseline run (a test that stopped "
                "registering, or a test filter?)"
            )
    for suite, run in sorted((base or {}).items()):
        missing = sorted(run.tests.keys() - results[suite].tests.keys())
        if missing:
            raise ValueError(
                f"{suite} tests ran on the base but are missing from the "
                f"results: {', '.join(missing)}"
            )


def covers(entries, entry):
    """True if `entries` lists `entry` itself or its test-level entry."""
    return entry in entries or entry[:2] in entries


def passed(run, entry):
    """True if the test or case of `entry` ran and passed in `run`."""
    status = run.tests.get(entry[1], "")
    if len(entry) == 2:
        return status is None
    # The case results of a test-level failure are missing, partial or stale.
    return status in (None, NORMAL_FAILURE) and run.cases.get(entry[1:], False)


class Report:
    def __init__(self):
        self.lines = []
        self.gating = 0

    def add(self, label, text, gating=False):
        self.lines.append(f"{label:<10} {text}")
        self.gating += 1 if gating else 0


def compare(results, expected, max_seconds_scale, factor, base=None):
    """Return a Report of every difference from the expected failures."""
    report = Report()
    found = {s: run.entries(s) for s, run in results.items()}
    on_base = {s: run.entries(s) for s, run in (base or {}).items()}

    def describe(entry):
        text = " ".join(entry)
        if len(entry) == 2:
            message = results[entry[0]].tests[entry[1]]
            text += f" ({describe_test_failure(message)})"
        return text

    accepted_failing = set()
    for suite in results:
        for entry in sorted(found[suite]):
            if len(entry) == 3 and entry[:2] in found[suite]:
                continue  # its test-level entry covers it
            if covers(expected.entries, entry):
                accepted_failing.update(
                    e for e in (entry, entry[:2]) if e in expected.accepted
                )
                if base is not None and passed(base[suite], entry):
                    # An accepted entry is a failure DART 6.19.4 does not have.
                    on_6194 = any(
                        e in expected.entries and e not in expected.accepted
                        for e in (entry, entry[:2])
                    )
                    report.add(
                        "REGRESSED",
                        f"{' '.join(entry)} (passes on the base"
                        + ("; also fails on DART 6.19.4)" if on_6194 else ")"),
                    )
            elif base is not None and entry in on_base[suite]:
                # Only the same failure: a case the base did not fail, or
                # whose result there is unknown, still gates.
                report.add("BASE", f"{describe(entry)} (also fails on the base)")
            else:
                report.add("NEW", describe(entry), gating=True)

    for (suite, test, case), limit in sorted(expected.max_seconds.items()):
        seconds = results[suite].durations.get((test, case))
        limit *= max_seconds_scale
        if seconds is None or seconds <= limit:
            continue
        text = f"{suite} {test} {case} took {seconds:.2f} s (limit {limit:.2f} s"
        base_seconds = base[suite].durations.get((test, case)) if base else None
        # Already over the limit on the base: gate only a further slowdown.
        if base_seconds is not None and limit < base_seconds:
            text += f", base {base_seconds:.2f} s)"
            if seconds <= base_seconds * factor:
                report.add("BASE SLOW", text)
                continue
        else:
            text += ")"
        report.add("SLOW", text, gating=True)

    for suite, run in results.items():
        for test, attempts in sorted(run.retried.items()):
            status = run.tests.get(test, "")
            if status not in (None, NORMAL_FAILURE):
                continue  # a test-level failure, reported above
            earlier = [cases for _, cases in attempts[:-1]]
            hidden = set().union(*earlier) - set(run.failed_cases(test))
            text = f"{suite} {test} ({', then '.join(s for s, _ in attempts)}"
            if hidden:
                text += f"; only an earlier attempt failed {', '.join(sorted(hidden))}"
            elif not all(earlier):
                text += "; an earlier attempt's output names no failing case"
            elif status == NORMAL_FAILURE:
                continue  # every attempt failed the same cases
            report.add("FLAKY" if status is None else "RETRIED", text + ")")

    for entry, reason in sorted(expected.accepted.items()):
        if entry in accepted_failing:
            report.add("ACCEPTED", f"{' '.join(entry)} ({reason})")
        elif passed(results[entry[0]], entry):
            report.add("STALE", f"{' '.join(entry)} (accepted failure now passes)")

    for entry in sorted(expected.entries - expected.accepted.keys()):
        if passed(results[entry[0]], entry):
            report.add("FIXED", f"{' '.join(entry)} (expected failure passed)")
    return report


def host_description():
    model = platform.processor() or platform.machine()
    cpuinfo = pathlib.Path("/proc/cpuinfo")
    if cpuinfo.is_file():
        for line in cpuinfo.read_text().splitlines():
            if line.startswith("model name"):
                model = line.split(":", 1)[1].strip()
                break
    return f"{model}, {platform.system()} {platform.release()}"


def write_baseline(path, results, previous, describe, timed_cases, factor):
    timed = re.compile(timed_cases)
    lines = [
        f"# Expected failures: {describe}",
        f"# Generated {datetime.date.today().isoformat()} on {host_description()}",
        "# by tools/gazebo/compat/compare_failures.py --write-baseline",
        "# (see tools/gazebo/README.md). max-seconds limits are "
        f"{factor:g}x the measured time",
        "# on that host; scale them with GZ_COMPAT_MAX_SECONDS_SCALE elsewhere.",
    ]
    if previous.accepted_lines:
        lines += ["", "# Reviewed differences from DART 6.19.4 (kept on regeneration)"]
        lines += previous.accepted_lines
    for suite, run in results.items():
        lines += ["", f"# {suite}", f"tests {suite} {len(run.tests)}"]
        entries = run.entries(suite)
        for entry in sorted(entries):
            if entry in previous.accepted or (len(entry) == 3 and entry[:2] in entries):
                continue
            line = " ".join(entry)
            if len(entry) == 2:
                line += f"  # {describe_test_failure(run.tests[entry[1]])}"
            lines.append(line)
        for (test, case), seconds in sorted(run.durations.items()):
            if timed.search(f"{test} {case}"):
                limit = math.ceil(seconds * factor * 10) / 10
                lines.append(f"max-seconds {suite} {test} {case} {limit:.1f}")
    path.write_text("\n".join(lines) + "\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--results", required=True, type=pathlib.Path)
    parser.add_argument("--expected", required=True, type=pathlib.Path)
    parser.add_argument(
        "--base-results",
        type=pathlib.Path,
        help="results of the same lane on the candidate's base; failures "
        "that also happen there are reported but do not fail the comparison",
    )
    parser.add_argument(
        "--max-seconds-scale",
        type=float,
        default=float(os.environ.get("GZ_COMPAT_MAX_SECONDS_SCALE", 1)),
        help="multiplier for max-seconds limits (default: "
        "$GZ_COMPAT_MAX_SECONDS_SCALE or 1)",
    )
    parser.add_argument("--write-baseline", action="store_true")
    parser.add_argument("--describe", default="Gazebo compatibility lane")
    parser.add_argument(
        "--timed-cases",
        default=r"_dartsim .*\.StepWorld$",
        help="regex over '<ctest test> <gtest case>' selecting the cases that "
        "get max-seconds limits in a baseline",
    )
    parser.add_argument(
        "--max-seconds-factor",
        type=float,
        default=2.0,
        help="max-seconds limit as a multiple of the baseline time; with "
        "--base-results, also how much slower than the base a case that is "
        "already over its limit there may get",
    )
    args = parser.parse_args(argv)
    if args.write_baseline and args.base_results:
        parser.error("--base-results cannot be combined with --write-baseline")

    try:
        results = {suite: load_suite(args.results, suite) for suite in SUITES}
        base = None
        if args.base_results:
            base = {suite: load_suite(args.base_results, suite) for suite in SUITES}
        expected = parse_expected(args.expected)
        if not args.write_baseline:
            check_inventory(results, expected, base)
    except (FileNotFoundError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    if args.write_baseline:
        write_baseline(
            args.expected,
            results,
            expected,
            args.describe,
            args.timed_cases,
            args.max_seconds_factor,
        )
        print(f"wrote {args.expected}")
        return 0

    report = compare(
        results, expected, args.max_seconds_scale, args.max_seconds_factor, base
    )
    for suite, run in results.items():
        failing = [t for t, message in run.tests.items() if message is not None]
        cases = [c for c, ok in run.cases.items() if not ok]
        print(f"{suite}: {len(failing)} failing tests, {len(cases)} failing cases")
    for line in report.lines:
        print(line)
    against = f"{args.expected}" + (f" and {args.base_results}" if base else "")
    if report.gating:
        print(f"FAIL: {report.gating} new failures or slow cases vs {against}")
        return 1
    print(f"PASS: no new failures or slow cases vs {against}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
