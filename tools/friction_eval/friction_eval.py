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
import csv
import io
import math
import os
import re
import statistics
import subprocess
import sys
import tempfile
from collections import defaultdict

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
    # Split-impulse cells wait for PR-0 (the position pass loses the velocity
    # impulses until it merges).
    add("R1", ["n=5,T=2"], ("B620",), gz, ("--split", "on", "--pending", "PR-0"))
    return cells


def command(binary, scene, params, config, detector, flags):
    label, solver, extra = CONFIGS[config]
    flags = [f for f in flags if f not in ("--ir",)]
    if "--pending" in flags:
        flags = flags[: flags.index("--pending")]
    cmd = [binary, "--scene", scene, "--solver", solver, "--detector", detector]
    cmd += ["--label", config] + list(extra) + flags
    return cmd + (["--param", params] if params else [])


def run_cell(binaries, cell, out_dir):
    scene, params, config, detector, flags = cell
    if "--pending" in flags:
        return "", f"pending {flags[flags.index('--pending') + 1]}: {cell}\n"
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
    proc = subprocess.run(prefix + cmd, capture_output=True, text=True)
    out = proc.stdout
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
            if rows and steps
            else ""
        )
    err = (
        ""
        if proc.returncode == 0
        else f"exit {proc.returncode}: {' '.join(cmd)}\n{proc.stderr}"
    )
    return out, err


COLUMNS_LINE = ",".join(COLUMNS) + "\n"


def cmd_run(args):
    binaries = dict(b.split("=", 1) for b in args.bin)
    os.makedirs(args.out, exist_ok=True)
    cells = [c for c in e1_cells() if re.search(args.only, " ".join(map(str, c)))]
    cells = [c for c in cells if CONFIGS[c[2]][0] in binaries]
    # Heavy cells first so the pool stays busy.
    heavy = ("R2", "R6", "C4", "R5", "R1", "P1")
    cells.sort(key=lambda c: heavy.index(c[0]) if c[0] in heavy else len(heavy))
    errors = []
    # Rows are written as cells finish, so an interrupted run keeps its data.
    with open(os.path.join(args.out, "cells.csv"), "w") as f:
        f.write(COLUMNS_LINE)
        with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
            futures = [pool.submit(run_cell, binaries, c, args.out) for c in cells]
            for i, future in enumerate(concurrent.futures.as_completed(futures), 1):
                out, err = future.result()
                f.writelines(
                    line + "\n"
                    for line in out.splitlines()
                    if not line.startswith("label,")
                )
                f.flush()
                if err:
                    errors.append(err)
                print(f"{i}/{len(cells)}", file=sys.stderr, flush=True)
    with open(os.path.join(args.out, "errors.txt"), "w") as f:
        f.writelines(errors)
    failed = sum(not e.startswith("pending") for e in errors)
    print(f"{len(cells)} cells, {failed} failed")
    return 1 if failed else 0


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
    if v is None or (isinstance(v, float) and math.isnan(v)):
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


def scores(data, config, detectors=("ode",)):
    """Mean D-score per scene of a configuration against B620 (accuracy)."""
    per_scene = defaultdict(list)
    for key, m in data.items():
        if key[0] != config or key[3] not in detectors or key[4] != "0.001":
            continue
        base = data.get(("B620",) + key[1:])
        if not base:
            continue
        b = {k: (e, eps) for k, e, eps in accuracy_errors(key[1], base)}
        for metric, e, eps in accuracy_errors(key[1], m):
            if metric not in b:
                continue
            s = (
                class_score(b[metric][0] == 0, e == 0)
                if eps is None
                else score(b[metric][0], e, eps)
            )
            per_scene[key[1]].append(s)
    return {s: statistics.mean(v) for s, v in sorted(per_scene.items())}


KEY_METRICS = {
    "A1": ("accel", "creep"),
    "A2": ("slides", "accel", "dir_err_deg"),
    "A3": ("onset_deg",),
    "A4": ("threshold", "force_ratio", "vel_dir_err_deg"),
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
    "R1": ("top_drift", "top_sink"),
    "R2": ("max_disp", "moved"),
    "R3": ("top_drift", "top_sink"),
    "R5": ("standing_1s", "moved"),
    "R6": ("collapsed", "max_disp"),
    "R9": ("yaw_rate", "speed"),
    "P1": ("resting", "contacts_max"),
}


def differs(a, b, rel=1e-6):
    if isinstance(a, str) or isinstance(b, str):
        return a != b
    if not (math.isfinite(a) and math.isfinite(b)):
        return not (a == b or (math.isnan(a) and math.isnan(b)))
    return abs(a - b) > rel * max(1.0, abs(a), abs(b))


def compare(
    data,
    config,
    scenes=None,
    metrics_extra=("nat_res_max", "box_viol_max", "fallbacks"),
):
    """Rows of config vs B620 for every cell both have."""
    rows = []
    for key in sorted(data, key=lambda k: (k[1], k[2], k[3], k[4])):
        if key[0] != config or (scenes and key[1] not in scenes):
            continue
        base = data.get(("B620",) + key[1:])
        if not base:
            continue
        m = data[key]
        for metric in KEY_METRICS.get(key[1], ()) + metrics_extra:
            if metric in m and metric in base:
                rows.append(
                    (
                        key[1],
                        key[2],
                        key[3],
                        key[4],
                        metric,
                        base[metric],
                        m[metric],
                        differs(base[metric], m[metric]),
                    )
                )
    return rows


# pred_box_<name> -> measured metric when the names differ.
BOX_MEASURED = {
    "cap": "threshold",
    "Fg": "threshold",
    "deg": "onset_deg",
    "ratio_mean_rim": "alpha_ratio_mean",
}


def report(data, out):
    w = out.write
    w("# E1 tables\n\nGenerated by `tools/friction_eval/friction_eval.py report`.\n\n")
    keys = [k for k in data if k[0] == "B619"]
    drift = compare(data, "B619")
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
        s = scores(data, config)
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
                            m.get("pred_exact_" + metric[9:]),
                        )
                    )
    w(
        table(
            ("scene", "params", "detector", "metric", "measured", "box", "exact"), rows
        )
    )
    return 0


def l1_report(path, out):
    """Per family and solver: worst residual, worst box violation, failures."""
    groups = defaultdict(lambda: defaultdict(list))
    with open(path) as f:
        for r in csv.DictReader(f):
            family = re.sub(r"_s\d+_g\d+$", "", r["scene"])
            family = "check_single" if family.startswith("check_single_") else family
            groups[(family, r["solver"])][r["metric"]].append(float(r["value"]))
    rows = []
    for (family, solver), m in sorted(groups.items()):
        rows.append(
            (
                family,
                solver,
                len(m["ok"]),
                len(m["ok"]) - sum(m["ok"]),
                max(m["nat_res"]),
                statistics.median(m["nat_res"]),
                max(m["box_viol"]),
                statistics.mean(m["solve_ms"]),
                sum(m.get("tight_capped", [0])),
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
        report(load(path), sys.stdout)
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
    assert differs(1.0, nan) and differs(1.0, math.inf) and not differs(nan, nan)
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
