#!/usr/bin/env python3
"""Run the friction-evaluation matrix and summarize it (stdlib only).

  friction_eval.py run --bin B619=PATH --bin B620=PATH --out DIR [-j N]
  friction_eval.py report DIR > summary.md
  friction_eval.py --self-test [--bin PATH]

`run` executes the E1 cells (one friction_eval process per cell, since ERP,
CFM and ERV are process-wide), `report` prints the E1 tables. See README.md.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import contextlib
import csv
import io
import math
import os
import re
import shutil
import statistics
import subprocess
import sys
import tempfile
from collections import defaultdict
from unittest import mock

DETECTORS = ("ode", "dart", "fcl", "bullet")
MU_STAR = 0.3657  # C4 arch, t/R = 0.15
COLUMNS = (
    "label,dart,scene,params,solver,detector,dt,split,deactivation,metric,value"
).split(",")

# (label, solver, extra flags) for each configuration of plan section 6.1.
BASE = ("B620", "dantzig", ())
CONFIGS = {
    "B619": ("B619", "dantzig", ()),
    "B620": BASE,
    "DZ+R": ("B620", "dzr", ()),
    "VA": ("B620", "dantzig", ("--va",)),
    "DZ+R+VA": ("B620", "dzr", ("--va",)),
    "PGS-tight": ("B620", "pgs-tight", ()),
    "PGS30": ("B620", "pgs", ()),
}
L2_CONFIGS = tuple(CONFIGS)
L3_CONFIGS = ("B619", "B620", "DZ+R", "VA", "PGS-tight")


def grid(**axes):
    """Cartesian product of parameter axes as 'k=v,...' strings."""
    rows = [""]
    for key, values in axes.items():
        rows = [f"{r},{key}={v}".lstrip(",") for r in rows for v in values]
    return rows


def e1_cells():
    """The E1 matrix: (scene, params, config, detector, flags) tuples."""
    cells = []

    def add(scene, params, configs, detectors=DETECTORS, flags=()):
        for p in params:
            for c in configs:
                for d in detectors:
                    cells.append((scene, p, c, d, tuple(flags)))

    l2, l3 = L2_CONFIGS, L3_CONFIGS
    add("A1", grid(mu=(0.3, 0.4, 0.45, 0.49, 0.51, 0.55, 0.6, 0.8)), l2)
    add("A2", grid(phi=(15, 30, 45), mu=(0.36, 0.4, 0.45, 0.49)), l2)
    add("A3", grid(mu=(0.3, 0.5, 0.8), phi=(0, 45)), l2)
    bisect_k = ("--bisect", "k=0:2:slides")
    add("A4", grid(phi=(0, 15, 30, 45, 60, 75, 90)), l2, flags=bisect_k)
    add("A4", grid(phi=(15, 30, 45), k=(1.5,)), l2)
    add("A12", grid(mu2=(0, 0.5), phi=(0, 30, 45, 60, 90)), l2, flags=bisect_k)
    add("A5", grid(phi=(0, 22.5, 30, 45), mu=(0.2, 0.5)), l2)
    add("A6", grid(mu=(0.2, 0.5)), l2)
    add("A7", ["v0=4,w0=-200", "v0=4,w0=0", "v0=4,w0=0,phi=30"], l2)
    add("A8", grid(shape=(0, 1), mu=(0.1, 1)), l2)
    add("A10", grid(beta=(0, 22.5, 45)), l2)
    fg = ("--bisect", "Fg=3:15:held")
    add("A11", grid(psi=(0, 22.5, 45), fdir=(0, 1), servo=(0, 1)), l2, flags=fg)
    add("A13", grid(slip=(0.1, 0.5), dir=(0, 1)), l2)
    add("C1", grid(mu=(0.4, 0.45, 0.48, 0.52, 0.55, 0.6)), l2)
    add("C2", grid(muw=(0, 0.5)), l2, flags=("--bisect", "alpha=20:70:slid"))
    arch = [
        f"n={n},mu={round(f * MU_STAR, 4)}"
        for n in (10, 24)
        for f in (0.8, 0.9, 1.2, 1.5, 2)
    ]
    add("C4", arch, l2[1:], detectors=("bullet",))
    add("C5", grid(mu=(0.5, 1.0, 1.5)), l2)
    gz = ("ode", "dart")
    add("R1", grid(n=(5, 10, 20), T=(5,)), l3, gz)
    add("R2", ["rows=10,T=0.5"], l3, ("ode",))
    add("R3", grid(ratio=(1, 10, 100, 1000)), l3, gz)
    add("R5", grid(mu=(0.5, 0.8), proj=(1,)), l3, gz)
    add("R6", ["T=3"], l3[1:], ("bullet",))
    for dt in (0.00025, 0.0005, 0.002, 0.004):
        flags = ("--dt", str(dt))
        add("A1", ["mu=0.3"], l3, gz, flags)
        add("A5", ["phi=30"], l3, gz, flags)
        add("A7", ["v0=4,w0=0"], l3, gz, flags)
        add("R1", ["n=5,T=2"], l3, gz, flags)
    add("A5", grid(mu=(0, 0.000999, 0.001001, 2, 10, 50, 100)), l3, gz)
    add(
        "A4",
        ["mu=0.7,mu2=150,fdir=0,phi=45,k=1.05", "mu=1,mu2=0.0001,fdir=0,phi=45,k=1.05"],
        l3,
        gz,
    )
    add("R9", ["wl=5,wr=10", "wl=10,wr=10", "wl=-5,wr=5"], l3, gz)
    # P1 S5/S4-like generated scenes with the guards' global contact cap:
    # counts in audit rows, wall time and instructions in --perf rows (DZ+R
    # timing is meaningful, VA's is not).
    cap = ("--max-contacts", "20000")
    for n in (90, 900):
        p1 = [f"n={n}"]
        add(
            "P1",
            p1,
            ("B619", "B620", "DZ+R"),
            DETECTORS,
            cap + ("--deactivation", "on"),
        )
        add("P1", p1, ("B620", "DZ+R"), DETECTORS, cap + ("--perf",))
        add("P1", p1, ("B620", "DZ+R"), ("ode", "dart"), cap + ("--perf", "--ir"))
    # Split-impulse cells: the position pass keeps the velocity impulses since
    # #3567.
    add("R1", ["n=5,T=2"], ("B620",), gz, ("--split", "on"))
    return cells


def command(binary, scene, params, config, detector, flags):
    label, solver, extra = CONFIGS[config]
    flags = [f for f in flags if f not in ("--ir",)]
    if "--pending" in flags:
        flags = flags[: flags.index("--pending")]
    cmd = [binary, "--scene", scene, "--solver", solver, "--detector", detector]
    cmd += ["--label", config] + list(extra) + flags
    return cmd + (["--param", params] if params else [])


def check_output(out, flags):
    """Why a cell's CSV is unusable, or "": missing rows or failed measurements.

    Older harnesses emit NaN for unavailable metrics that newer ones omit.
    Keep those markers valid, but reject NaN in measurements with no such case.
    """
    rows = list(csv.DictReader(io.StringIO(out)))
    metrics = {r["metric"]: r["value"] for r in rows}
    scene = rows[0]["scene"] if rows else ""
    if "--bisect" in flags:
        required = ("at_lo", "at_hi")
        if metrics.get("at_lo") != metrics.get("at_hi"):
            required += ("threshold",)
    else:
        required = ("steps", "state_hash")
        if scene == "A10":
            required += ("v_err",)  # No final contact means no velocity measurement.
    missing = [m for m in ("finite",) + required if m not in metrics]
    if missing:
        return f"missing rows {missing}"
    if metrics["finite"] != "1":
        return "non-finite state (finite = 0)"
    # No qualifying samples or an event not reached within the horizon.
    optional = {
        "slip_dir_err_mean_deg",
        "slip_dir_err_max_deg",
        "dilatancy_mean",
    } | set(
        {
            "A3": ("onset_deg",),
            "A4": ("force_ratio", "dir_err_deg"),
            "A5": ("stop_time", "creep"),
            "A6": (
                "alpha_ratio_mean",
                "alpha_ratio_min",
                "alpha_ratio_max",
                "box_err_max",
            ),
            "A7": ("roll_step",),
            "A10": ("sync_time",),
            "A12": ("force_ratio", "dir_err_deg"),
            "C1": ("front_share",),
            "C4": ("collapse_time",),
            "R1": ("rest_time",),
            "R3": ("rest_time",),
            "R5": ("standing_1s",),
            "R6": ("collapse_time",),
        }.get(scene, ())
    )
    if "--bisect" in flags and metrics["at_lo"] == metrics["at_hi"]:
        optional.add("threshold")
    params = dict(p.split("=", 1) for p in rows[0]["params"].split(";") if "=" in p)
    # E1's older A5 reference was undefined at mu=0; zero launch speed also
    # gives no distance reference. A7's relative error needs nonzero v_roll.
    if scene == "A5" and (
        float(params.get("mu", 0.5)) == 0 or float(params.get("v0", 2)) == 0
    ):
        optional.add("dist_ratio")
    if scene == "A7" and float(metrics.get("pred_v_roll", "nan")) == 0:
        optional.add("v_roll_err")
    if scene == "R9" and float(params.get("T", 5)) <= 2:
        optional.add("yaw_rate")  # No interval after spin-up.
    for metric, value in metrics.items():
        if metric == "state_hash":
            continue
        try:
            value = float(value)
        except ValueError:
            return f"invalid measurement {scene}.{metric}: {value}"
        if math.isinf(value) or (
            math.isnan(value)
            and metric not in optional
            and not metric.startswith("pred_")
        ):
            return f"non-finite measurement {scene}.{metric}"
    return ""


def run_cell(binaries, cell, out_dir):
    scene, params, config, detector, flags = cell
    cmd = command(binaries[CONFIGS[config][0]], scene, params, config, detector, flags)
    prefix = []
    ir_file = None
    if "--ir" in flags:
        fd, ir_file = tempfile.mkstemp(suffix=".callgrind", dir=out_dir)
        os.close(fd)
        prefix = [
            "valgrind",
            "--tool=callgrind",
            "--collect-atstart=no",
            "--toggle-collect=dart::simulation::World::step(bool)",
            f"--callgrind-out-file={ir_file}",
        ]
    try:
        proc = subprocess.run(prefix + cmd, capture_output=True, text=True)
    except OSError as e:  # e.g. a missing binary: a failed cell, not a crash
        return "", f"{e}: {' '.join(cmd)}\n"
    out = proc.stdout
    problem = (
        f"exit {proc.returncode}" if proc.returncode != 0 else check_output(out, flags)
    )
    if ir_file:
        # Keep only the instruction count; the --perf cell has the other rows.
        with open(ir_file) as f:
            total = next(
                (int(line.split()[1]) for line in f if line.startswith("summary:")), 0
            )
        os.remove(ir_file)
        rows = list(csv.reader(io.StringIO(out)))[1:]
        steps = next((float(r[-1]) for r in rows if r[-2] == "steps"), 0.0)
        out = (
            ",".join(rows[0][:-2] + ["ir_per_step", f"{total / steps:.6g}"]) + "\n"
            if rows and steps and total
            else ""
        )
        problem = problem or ("" if out else "no Callgrind instruction count")
    err = f"{problem}: {' '.join(cmd)}\n{proc.stderr}" if problem else ""
    return out, err


def plan(cells, have_valgrind):
    """Split cells into a parallel phase, a serial phase for the wall-clock
    (--perf without --ir) cells, and skipped cells with their reasons."""
    parallel, serial, skipped = [], [], []
    for cell in cells:
        flags = cell[4]
        if "--pending" in flags:
            skipped.append((f"pending {flags[flags.index('--pending') + 1]}", cell))
        elif "--ir" in flags and not have_valgrind:
            skipped.append(("valgrind not found", cell))
        elif "--perf" in flags and "--ir" not in flags:
            serial.append(cell)
        else:
            parallel.append(cell)
    return parallel, serial, skipped


COLUMNS_LINE = ",".join(COLUMNS) + "\n"


def preflight(binaries):
    """Why a harness binary cannot start, or "". A cell whose binary never
    starts prints no key, so scores() could not count it as failed."""
    for label, binary in sorted(binaries.items()):
        try:
            proc = subprocess.run(
                [binary, "--list"], capture_output=True, text=True, timeout=120
            )
        except (OSError, subprocess.TimeoutExpired) as e:
            return f"{label} binary {binary} does not start: {e}"
        if proc.returncode:
            return f"{label} binary {binary} does not start: exit {proc.returncode}"
    return ""


def cmd_run(args):
    binaries = dict(b.split("=", 1) for b in args.bin)
    unknown = sorted(set(binaries) - {config[0] for config in CONFIGS.values()})
    if unknown:
        print(f"unknown --bin labels {unknown}", file=sys.stderr)
        return 2
    cells = [c for c in e1_cells() if re.search(args.only, " ".join(map(str, c)))]
    cells = [c for c in cells if CONFIGS[c[2]][0] in binaries]
    if not cells:
        print("no cells selected: check --only and --bin", file=sys.stderr)
        return 2
    # Heavy cells first so the pool stays busy.
    heavy = ("R2", "R6", "C4", "R5", "R1", "P1")
    cells.sort(key=lambda c: heavy.index(c[0]) if c[0] in heavy else len(heavy))
    parallel, serial, skipped = plan(cells, shutil.which("valgrind") is not None)
    if not parallel and not serial:
        reasons = "; ".join(sorted({reason for reason, _ in skipped}))
        print(f"no runnable cells: {reasons}", file=sys.stderr)
        return 2
    used = {CONFIGS[c[2]][0] for c in parallel + serial}
    problem = preflight({k: v for k, v in binaries.items() if k in used})
    if problem:
        print(problem, file=sys.stderr)
        return 2
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, "skipped.txt"), "w") as f:
        f.writelines(f"{reason}: {cell}\n" for reason, cell in skipped)
    errors = []
    done = 0
    total = len(parallel) + len(serial)
    # Rows are written as cells finish, so an interrupted run keeps its data. A
    # failed cell's rows go to failed.csv for diagnosis, never to cells.csv, so
    # they stay out of every table and score; report lists it as failed.
    with open(os.path.join(args.out, "cells.csv"), "w") as f, open(
        os.path.join(args.out, "failed.csv"), "w"
    ) as failed:
        f.write(COLUMNS_LINE)
        failed.write(COLUMNS_LINE)

        def record(result):
            nonlocal done
            out, err = result
            dest = failed if err else f
            dest.writelines(
                line + "\n"
                for line in out.splitlines()
                if not line.startswith("label,")
            )
            dest.flush()
            if err:
                errors.append(err)
            done += 1
            print(f"{done}/{total}", file=sys.stderr, flush=True)

        with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
            futures = [pool.submit(run_cell, binaries, c, args.out) for c in parallel]
            for future in concurrent.futures.as_completed(futures):
                record(future.result())
        # Wall-clock cells run alone, after the CPU-heavy parallel phase.
        for cell in serial:
            record(run_cell(binaries, cell, args.out))
    with open(os.path.join(args.out, "errors.txt"), "w") as f:
        f.writelines(errors)
    print(f"{total} cells run, {len(errors)} failed, {len(skipped)} skipped")
    return 1 if errors else 0


#
# Report
#


def load(path):
    """{(config, scene, params, detector, dt, split, deactivation): {metric: value}}."""
    data = defaultdict(dict)
    with open(path) as f:
        for r in csv.DictReader(f):
            key = (
                r["label"],
                r["scene"],
                r["params"],
                r["detector"],
                r["dt"],
                r["split"],
                r["deactivation"],
            )
            try:
                data[key][r["metric"]] = float(r["value"])
            except ValueError:
                data[key][r["metric"]] = r["value"]
    return data


def score(e_base, e_cand, eps):
    """Per-metric score of D section 11.2: 0 is parity, +1 is 4x better."""
    return max(-2.0, min(2.0, math.log2((e_base + eps) / (e_cand + eps)))) / 2.0


def class_score(base_ok, cand_ok):
    return 0.0 if base_ok == cand_ok else (1.0 if cand_ok else -1.0)


def fmt(v):
    if isinstance(v, str):
        return v
    if v is None:
        return "-"
    if isinstance(v, float) and math.isnan(v):
        return "nan"
    return f"{v:.4g}"


def table(header, rows):
    lines = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    lines += ["| " + " | ".join(fmt(v) for v in row) + " |" for row in rows]
    return "\n".join(lines) + "\n"


# Errors against exact Coulomb: (measured, reference or None for 0, floor);
# reference strings name a per-cell metric.
ACCURACY = {
    "A1": [("accel", "pred_exact_accel", 1e-3)],
    "A2": [("slides", "pred_exact_slides", None)],
    "A3": [("onset_deg", "pred_exact_deg", 0.1)],
    "A4": [("threshold", "pred_exact_cap", 0.02), ("dir_err_deg", None, 0.5)],
    "A12": [("threshold", "pred_exact_cap", 0.02)],
    "A5": [("dist_ratio", 1.0, 1e-3), ("lateral", None, 1e-3)],
    "A6": [("alpha_ratio_mean", 1.0, 1e-2)],
    "A7": [("v_roll_err", None, 1e-6), ("lateral", None, 1e-3)],
    "A10": [("sync_time", "pred_exact_sync_time", 1e-3)],
    "A11": [("threshold", "pred_exact_Fg", 0.2)],
    "A13": [("v_err", None, 1e-5)],
    "C1": [("tipped", "pred_tips", None), ("front_share", "pred_front_share", 1e-3)],
    "C2": [("threshold", "pred_alpha_deg", 0.8)],
}


def accuracy_errors(scene, m):
    out = []
    for metric, ref, eps in ACCURACY.get(scene, []):
        value = m.get(metric)
        target = m.get(ref) if isinstance(ref, str) else (ref or 0.0)
        if (
            value is None
            or target is None
            or any(isinstance(x, str) or math.isnan(x) for x in (value, target))
        ):
            continue
        out.append((metric, abs(value - target), eps))
    return out


def scores(data, config, detectors=("ode",), failed=frozenset()):
    """Mean D-score per scene of a configuration against B620 (accuracy). A
    measurement only one side made (an event the other never reached, a
    failed one, or any of a failed cell, which measures nothing) is scored as
    a classification, so a failure counts -1. failed holds failed.csv's keys."""
    per_scene = defaultdict(list)
    cells = {k[1:] for k in (*data, *failed) if k[0] in ("B620", config)}
    for cell in sorted(cells):
        if cell[2] not in detectors or cell[3] != "0.001":
            continue
        # A cell that one side never ran is no comparison.
        base, m = (
            data[k] if k in data else ({} if k in failed else None)
            for k in (("B620",) + cell, (config,) + cell)
        )
        if base is None or m is None:
            continue
        b = {k: e for k, e, _ in accuracy_errors(cell[0], base)}
        c = {k: e for k, e, _ in accuracy_errors(cell[0], m)}
        for metric, _, eps in ACCURACY.get(cell[0], []):
            if metric in b and metric in c:
                s = (
                    class_score(b[metric] == 0, c[metric] == 0)
                    if eps is None
                    else score(b[metric], c[metric], eps)
                )
            elif metric in b or metric in c:
                s = class_score(metric in b, metric in c)
            else:
                continue
            per_scene[cell[0]].append(s)
    return {s: statistics.mean(v) for s, v in sorted(per_scene.items())}


KEY_METRICS = {
    "A1": ("accel", "creep"),
    "A2": ("slides", "accel", "dir_err_deg"),
    "A3": ("onset_deg",),
    "A4": ("threshold", "force_ratio", "dir_err_deg", "vel_dir_err_deg"),
    "A12": ("threshold",),
    "A5": ("dist_ratio", "dir_deg", "lateral", "creep"),
    "A6": ("alpha_ratio_mean", "alpha_ratio_min", "alpha_ratio_max"),
    "A7": ("v_roll_err", "roll_step", "lateral"),
    "A8": ("speed_loss_per_m",),
    "A10": ("sync_time", "v_err"),
    "A11": ("threshold",),
    "A13": ("v_err",),
    "C1": ("tipped", "max_pitch_deg", "front_share"),
    "C2": ("threshold",),
    "C4": ("collapsed", "max_disp"),
    "C5": ("final_height",),
    "R1": ("top_drift", "top_sink", "rest_time"),
    "R2": ("max_disp", "moved"),
    "R3": ("top_drift", "top_sink", "rest_time"),
    "R5": ("standing_1s", "moved"),
    "R6": ("collapsed", "max_disp"),
    "R9": ("yaw_rate", "speed"),
    "P1": ("resting", "contacts_max", "ir_per_step", "wall_ms_per_step"),
}
# Wall time on a shared host only indicates; instruction counts gate.
INDICATIVE = {"wall_ms_per_step"}


def differs(a, b, rel=1e-6):
    """True unless both values are present and equal to rel. Undefined
    metrics may be omitted or use legacy NaN markers. A NaN is never equal
    to anything, NaN included."""
    if a is None or b is None or isinstance(a, str) or isinstance(b, str):
        return a != b
    if math.isnan(a) or math.isnan(b) or math.isinf(a) or math.isinf(b):
        return not (a == b and math.isinf(a))
    return abs(a - b) > rel * max(1.0, abs(a), abs(b))


def compare(
    data,
    config,
    scenes=None,
    metrics_extra=("nat_res_max", "box_viol_max", "fallbacks"),
):
    """Rows of config vs B620 for every cell both have; a metric that only
    one side reports shows "-" on the other and counts as a difference.
    Params name split and deactivation when on, so P1's count cells
    (deactivation on) and timing cells read apart."""
    rows = []
    for key in sorted(data, key=lambda k: (k[1], k[2], k[3], k[4])):
        if key[0] != config or (scenes and key[1] not in scenes):
            continue
        base = data.get(("B620",) + key[1:])
        if not base:
            continue
        m = data[key]
        params = key[2] + "".join(
            f";{name}=on"
            for name, v in zip(("split", "deactivation"), key[5:])
            if v == "on"
        )
        for metric in KEY_METRICS.get(key[1], ()) + metrics_extra:
            if metric in m or metric in base:
                label = metric + (" (indicative)" if metric in INDICATIVE else "")
                a, b = base.get(metric), m.get(metric)
                rows.append(
                    (key[1], params, key[3], key[4], label, a, b, differs(a, b))
                )
    return rows


# pred_box_<name> -> measured metric when the names differ.
BOX_MEASURED = {
    "cap": "threshold",
    "Fg": "threshold",
    "deg": "onset_deg",
    "ratio_mean_rim": "alpha_ratio_mean",
}

# pred_box_<name> -> pred_exact_<name> when the names differ.
BOX_EXACT = {"ratio_mean_rim": "ratio"}


def report(data, out, failed=frozenset()):
    w = out.write
    w("# E1 tables\n\nGenerated by `tools/friction_eval/friction_eval.py report`.\n\n")
    keys = [k for k in data if k[0] == "B619"]
    drift = compare(data, "B619")
    # Indicative rows carry a suffixed label, so wall time never counts here.
    changed = sorted(
        {r[:4] for r in drift if r[7] and r[4] in KEY_METRICS.get(r[0], ())}
    )
    w(
        f"## B620 vs B619 (Dantzig)\n\n{len(keys)} B619 cells; {len(changed)} differ in a key metric.\n\n"
    )
    w(
        table(
            ("scene", "params", "detector", "dt", "metric", "B620", "B619"),
            [r[:7] for r in drift if r[7]],
        )
    )
    for config in ("DZ+R", "VA", "DZ+R+VA", "PGS-tight", "PGS30"):
        rows = compare(data, config)
        w(f"\n## {config} vs B620\n\n")
        s = scores(data, config, failed=failed)
        if s:
            w(
                "Accuracy score against exact Coulomb (D section 11.2, ODE, dt 1 ms; 0 = parity, +1 = 4x closer):\n\n"
            )
            w(table(("scene", "score"), list(s.items())))
            w(f"\nMean: {fmt(statistics.mean(s.values()))}\n\n")
        w(
            table(
                ("scene", "params", "detector", "dt", "metric", "B620", config),
                [r[:7] for r in rows if r[7]],
            )
        )
    w("\n## Box predictions (B620, all detectors)\n\n")
    rows = []
    for key in sorted(data, key=lambda k: (k[1], k[2], k[3])):
        m = data[key]
        if key[0] != "B620" or key[4] != "0.001":
            continue
        for metric, value in m.items():
            if metric.startswith("pred_box_"):
                measured = metric[len("pred_box_") :]
                measured = BOX_MEASURED.get(measured, measured)
                if measured in m:
                    rows.append(
                        (
                            key[1],
                            key[2],
                            key[3],
                            measured,
                            m[measured],
                            value,
                            m.get(
                                "pred_exact_" + BOX_EXACT.get(metric[9:], metric[9:])
                            ),
                        )
                    )
    w(
        table(
            ("scene", "params", "detector", "metric", "measured", "box", "exact"), rows
        )
    )
    return 0


def l1_report(path, out):
    """Per family and solver: failures (the backend's, or a non-finite
    solution) and the worst residuals of the finite solutions ("-" if none)."""
    groups = defaultdict(list)
    with open(path) as f:
        for r in csv.DictReader(f):
            family = re.sub(r"_s\d+_g\d+$", "", r["scene"])
            family = "check_single" if family.startswith("check_single_") else family
            solves = groups[(family, r["solver"])]
            # runL1 prints each solve's rows together, ok first. Scene names
            # can repeat (dumps from different directories share basenames),
            # so a solve is the rows from one ok to the next.
            if r["metric"] == "ok" or not solves:
                solves.append({})
            solves[-1][r["metric"]] = float(r["value"])
    rows = []
    for (family, solver), ps in sorted(groups.items()):
        # Older files printed zero residuals for non-finite solutions.
        finite = [p for p in ps if p.get("finite") == 1.0]
        res = [p["nat_res"] for p in finite if "nat_res" in p]
        viol = [p["box_viol"] for p in finite if "box_viol" in p]
        rows.append(
            (
                family,
                solver,
                len(ps),
                sum(p.get("ok") != 1.0 or p.get("finite") != 1.0 for p in ps),
                max(res, default=None),
                statistics.median(res) if res else None,
                max(viol, default=None),
                statistics.mean(p["solve_ms"] for p in ps),
                sum(p.get("tight_capped", 0.0) for p in ps),
            )
        )
    out.write(
        table(
            (
                "family",
                "solver",
                "problems",
                "not ok",
                "nat_res max",
                "nat_res median",
                "box_viol max",
                "ms mean",
                "capped",
            ),
            rows,
        )
    )
    return 0


def cmd_report(args):
    path = os.path.join(args.dir, "cells.csv")
    if os.path.exists(path):
        failed_path = os.path.join(args.dir, "failed.csv")
        failed = set(load(failed_path)) if os.path.exists(failed_path) else set()
        report(load(path), sys.stdout, failed)
    for name, title in (
        ("skipped.txt", "Skipped cells"),
        ("errors.txt", "Failed cells"),
    ):
        path = os.path.join(args.dir, name)
        lines = open(path).read().splitlines() if os.path.exists(path) else []
        if lines:
            sys.stdout.write(f"\n## {title}\n\n")
            sys.stdout.writelines(f"- {line}\n" for line in lines if line)
    for name in sorted(os.listdir(args.dir)):
        if name.startswith("l1") and name.endswith(".csv"):
            sys.stdout.write(f"\n## L1 bank: {name}\n\n")
            l1_report(os.path.join(args.dir, name), sys.stdout)
    return 0


#
# Self-test
#


def self_test(binary=None):
    assert score(1.0, 1.0, 1e-9) == 0.0
    assert abs(score(4.0, 1.0, 0.0) - 1.0) < 1e-12  # 4x better saturates at +1
    assert abs(score(1.0, 2.0, 0.0) + 0.5) < 1e-12
    assert score(100.0, 0.0, 1e-9) == 1.0 and score(0.0, 100.0, 1e-9) == -1.0
    assert class_score(True, False) == -1.0 and class_score(False, True) == 1.0
    assert grid(a=(1, 2), b=(3,)) == ["a=1,b=3", "a=2,b=3"]
    nan = float("nan")
    assert differs(1.0, nan) and differs(1.0, math.inf) and differs(nan, nan)
    assert differs(None, 1.0) and not differs(2.0, 2.0)
    text = io.StringIO()
    report(
        {
            ("B620", "A6", "mu=0.5", "ode", "0.001", "off", "off"): {
                "alpha_ratio_mean": 1.2,
                "pred_box_ratio_mean_rim": 1.3,
                "pred_exact_ratio": 1.0,
            }
        },
        text,
    )
    assert "| A6 | mu=0.5 | ode | alpha_ratio_mean | 1.2 | 1.3 | 1 |" in text.getvalue()
    # Cell outputs: a non-finite state or missing rows fail the cell.
    row = "B620,6.20-line,A5,mu=0,dantzig,ode,0.001,off,off,{},{}\n"
    ok = COLUMNS_LINE + "".join(
        row.format(m, v)
        for m, v in (("finite", 1), ("steps", 5), ("state_hash", "0x1"))
    )
    assert check_output(ok, ()) == ""
    assert "non-finite" in check_output(ok.replace("finite,1", "finite,0"), ())
    assert "missing" in check_output(COLUMNS_LINE + row.format("steps", 5), ())
    assert check_output(ok + row.format("slip_dir_err_mean_deg", "nan"), ()) == ""
    assert check_output(ok + row.format("dist_ratio", "nan"), ()) == ""
    assert "dist_ratio" in check_output(
        (ok + row.format("dist_ratio", "nan")).replace("mu=0,", "mu=0.5,"), ()
    )
    a10 = ok.replace(",A5,", ",A10,")
    assert "missing" in check_output(a10, ())
    assert check_output(a10 + row.format("v_err", 0).replace(",A5,", ",A10,"), ()) == ""
    a10_nan = a10 + row.format("v_err", "nan").replace(",A5,", ",A10,")
    assert "A10.v_err" in check_output(a10_nan, ())
    for metric in ("slip_dir_err_mean_deg", "pred_box_dist_ratio"):
        for value in ("inf", "-inf"):
            assert metric in check_output(ok + row.format(metric, value), ())
    assert "steps" in check_output(ok.replace("steps,5", "steps,nan"), ())
    assert "invalid measurement" in check_output(
        ok.replace("steps,5", "steps,broken"), ()
    )
    bisect = COLUMNS_LINE + "".join(
        row.format(m, v) for m, v in (("finite", 1), ("at_lo", 0), ("at_hi", 1))
    )
    assert "threshold" in check_output(bisect, ("--bisect",))
    assert check_output(bisect.replace("at_hi,1", "at_hi,0"), ("--bisect",)) == ""
    legacy_bisect = bisect + row.format("threshold", "nan")
    assert "threshold" in check_output(legacy_bisect, ("--bisect",))
    assert (
        check_output(legacy_bisect.replace("at_hi,1", "at_hi,0"), ("--bisect",)) == ""
    )
    # A failed cell's rows land in failed.csv, never in cells.csv. The process
    # is mocked, so this runs on every platform.
    finite0 = ok.replace("finite,1", "finite,0")
    for code, out, good in (
        (0, ok, 1),
        (0, finite0, 0),
        (1, ok, 0),
        (0, a10_nan, 0),
        (0, ok + row.format("slip_dir_err_mean_deg", "nan"), 1),
    ):
        done = subprocess.CompletedProcess([], code, out, "")
        with tempfile.TemporaryDirectory() as tmp, mock.patch.object(
            subprocess, "run", return_value=done
        ), mock.patch.object(
            sys.modules[__name__], "preflight", return_value=""
        ), contextlib.redirect_stdout(
            io.StringIO()
        ), contextlib.redirect_stderr(
            io.StringIO()
        ):
            run = argparse.Namespace(
                bin=["B620=friction_eval"],
                out=tmp,
                jobs=1,
                only="^A10 beta=0 B620 ode" if ",A10," in out else "^A5 mu=0 B620 ode",
            )
            assert cmd_run(run) == 1 - good
            assert len(load(os.path.join(tmp, "cells.csv"))) == good
            assert len(load(os.path.join(tmp, "failed.csv"))) == 1 - good
    # An empty selection, an unknown label or a binary that cannot start runs
    # nothing and fails, instead of reporting an empty but successful run.
    for bins, only in (
        (["B620=friction_eval"], "^no such cell"),
        (["B62O=friction_eval"], ""),
        (["B620=/nonexistent/friction_eval"], "^A5 mu=0 B620 ode"),
    ):
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stderr(
            io.StringIO()
        ):
            run = argparse.Namespace(bin=bins, out=tmp, jobs=1, only=only)
            assert cmd_run(run) == 2 and not os.listdir(tmp)
    # Scheduling: timing cells run serially; --ir needs Valgrind.
    cells = [
        ("P1", "n=90", "B620", "ode", ("--perf",)),
        ("P1", "n=90", "B620", "ode", ("--perf", "--ir")),
        ("R1", "n=5", "B620", "ode", ("--split", "on", "--pending", "PR-0")),
        ("A5", "mu=0", "B620", "ode", ()),
    ]
    parallel, serial, skipped = plan(cells, have_valgrind=False)
    assert parallel == cells[3:] and serial == cells[:1]
    assert [r for r, _ in skipped] == ["valgrind not found", "pending PR-0"]
    assert plan(cells, have_valgrind=True)[0] == [cells[1], cells[3]]
    # A nonempty selection can still have no runnable cells after planning.
    for only, reasons in (("--ir", ("valgrind not found",)),):
        with tempfile.TemporaryDirectory() as tmp, mock.patch.object(
            shutil, "which", return_value=None
        ), mock.patch.object(subprocess, "run") as process, contextlib.redirect_stderr(
            io.StringIO()
        ) as err:
            run = argparse.Namespace(
                bin=["B620=friction_eval"], out=tmp, jobs=1, only=only
            )
            assert cmd_run(run) == 2 and not os.listdir(tmp)
            assert "no runnable cells" in err.getvalue()
            assert all(reason in err.getvalue() for reason in reasons)
            process.assert_not_called()
    errs = accuracy_errors("A5", {"dist_ratio": 0.707, "lateral": 0.0})
    assert errs[0][0] == "dist_ratio" and abs(errs[0][1] - 0.293) < 1e-12
    assert accuracy_errors(
        "C1",
        {
            "tipped": 0.0,
            "pred_tips": 1.0,
            "front_share": 0.9,
            "pred_front_share": float("nan"),
        },
    ) == [("tipped", 1.0, None)]
    sample = COLUMNS_LINE + "".join(
        f"{c},6.20-line,A5,phi=45,dantzig,ode,0.001,off,off,{m},{v}\n"
        for c, m, v in (
            ("B620", "dist_ratio", 0.7064),
            ("B620", "lateral", 0.0),
            ("VA", "dist_ratio", 1.0),
            ("VA", "lateral", 0.0),
        )
    )
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "cells.csv")
        with open(path, "w") as f:
            f.write(sample)
        data = load(path)
        s = scores(data, "VA")
        # dist_ratio error 0.2936 -> 1e-12: saturated +1; lateral parity 0.
        assert abs(s["A5"] - 0.5) < 1e-12, s
        text = io.StringIO()
        report(data, text)
        assert (
            "| A5 | phi=45 | ode | 0.001 | dist_ratio | 0.7064 | 1 |" in text.getvalue()
        )
        # A metric only one side reports is a difference shown as "-".
        data[("VA", "A5", "phi=45", "ode", "0.001", "off", "off")]["creep"] = 0.0
        rows = [r for r in compare(data, "VA") if r[4] == "creep"]
        assert rows == [("A5", "phi=45", "ode", "0.001", "creep", None, 0.0, True)]
        on = {("B620", "P1", "n=90", "ode", "0.001", "off", "on"): {"resting": 90.0}}
        on[("DZ+R",) + next(iter(on))[1:]] = {"resting": 0.0}
        assert compare(on, "DZ+R")[0][1] == "n=90;deactivation=on"
        # An event only one side reaches scores as a classification.
        a10 = ("A10", "beta=0", "ode", "0.001", "off", "off")
        synced = {"sync_time": 0.17, "pred_exact_sync_time": 0.17}
        never = {"pred_exact_sync_time": 0.17}
        one_sided = {("B620",) + a10: synced, ("VA",) + a10: never}
        assert scores(one_sided, "VA") == {"A10": -1.0}
        one_sided = {("B620",) + a10: never, ("VA",) + a10: synced}
        assert scores(one_sided, "VA") == {"A10": 1.0}
        # A failed cell (failed.csv) measured nothing; one never run is no case.
        base, cand = ("B620",) + a10, ("VA",) + a10
        assert scores({base: synced}, "VA", failed={cand}) == {"A10": -1.0}
        assert scores({cand: synced}, "VA", failed={base}) == {"A10": 1.0}
        assert scores({}, "VA", failed={base, cand}) == {}
        assert scores({base: synced}, "VA") == {}
    # L1: a non-finite solution is not ok and has no residual, even where an
    # older file printed zeros for it; solves sharing a scene name stay apart.
    l1_rows = (
        ("fam_s1_g0", "dantzig", 1, 0.5),
        ("fam_s1_g1", "dantzig", 0, 0.0),
        ("fam_s1_g0", "pgs", 0, 0.0),
        ("fam_s1_g0", "dantzig", 1, 0.5),
    )
    l1 = COLUMNS_LINE + "".join(
        f"B620,6.20-line,{scene},n=3,{solver},-,0.001,off,off,{m},{v}\n"
        for scene, solver, finite, res in l1_rows
        for m, v in (
            ("ok", 1),
            ("finite", finite),
            ("nat_res", res),
            ("box_viol", res),
            ("solve_ms", 1),
        )
    )
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "l1.csv")
        with open(path, "w") as f:
            f.write(l1)
        text = io.StringIO()
        l1_report(path, text)
        assert "| fam | dantzig | 3 | 1 | 0.5 | 0.5 | 0.5 | 1 | 0 |" in text.getvalue()
        assert "| fam | pgs | 1 | 1 | - | - | - | 1 | 0 |" in text.getvalue()
    assert len(e1_cells()) > 1000
    if binary:
        proc = subprocess.run([binary, "--self-test"], capture_output=True, text=True)
        sys.stdout.write(proc.stdout)
        assert proc.returncode == 0, "friction_eval --self-test failed"
    print("friction_eval.py self-test passed")
    return 0


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument(
        "--bin",
        action="append",
        default=[],
        help="LABEL=PATH, or PATH with --self-test",
    )
    sub = parser.add_subparsers(dest="cmd")
    run = sub.add_parser("run")
    run.add_argument(
        "--bin", action="append", required=True, help="LABEL=PATH (B619, B620)"
    )
    run.add_argument("--out", required=True)
    run.add_argument("-j", "--jobs", type=int, default=8)
    run.add_argument("--only", default="", help="regex over the cell tuple")
    rep = sub.add_parser("report")
    rep.add_argument("dir")
    args = parser.parse_args()
    if args.self_test:
        return self_test(args.bin[0] if args.bin else None)
    if args.cmd == "run":
        return cmd_run(args)
    if args.cmd == "report":
        return cmd_report(args)
    parser.print_help()
    return 2


if __name__ == "__main__":
    sys.exit(main())
