"""Regression checks for performance comparison and local execution."""

import copy
import importlib.util
from pathlib import Path

import pytest


def test_compare_classification_and_thresholds():
    path = Path(__file__).resolve().parents[1] / "scripts/perf_regression.py"
    spec = importlib.util.spec_from_file_location("perf_regression", path)
    module = importlib.util.module_from_spec(spec)
    # Dataclasses resolve their defining module when importing a standalone script.
    import sys

    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    env = {
        "valgrind": "3.22.0",
        "compiler": "test",
        "compiler_provenance": "dart-perf-build/1",
        "glibc": "2.39",
        "preset": "perf-1",
        "fingerprint": "test",
    }
    metric = {
        "ir_per_step": 100_000,
        "allocs_per_step": 10,
        "bytes_per_step": 100,
        "guards": {
            "hash": "0x1",
            "contacts": 1,
            "finite": True,
            "cap_hit": True,
            "resting": "0/1",
        },
    }
    base = {
        "schema": "dart-perf/1",
        "run": {"commit": "base", "env": env},
        "results": [
            {
                "row": "a",
                "det": "dart",
                "version": 1,
                "gated": True,
                "status": "ok",
                "method": "slope",
                "head": metric,
            }
        ],
    }

    def check(
        *, ir=100_000, allocs=10, body="", changed=False, gated=True, status="ok"
    ):
        head = copy.deepcopy(base)
        row = head["results"][0]
        row.update(gated=gated, status=status)
        row["head"].update(ir_per_step=ir, allocs_per_step=allocs)
        if changed:
            row["head"]["guards"]["hash"] = "0x2"
        return module.compare(base, head, body)

    same = check()
    assert same["verdict"]["status"] == "PASS"
    assert same["results"][0]["delta"] == {
        "ir": 0,
        "allocs": 0,
        "bytes": 0,
        "guards_equal": True,
        "class": "gated",
    }
    assert check(ir=100_300)["verdict"]["status"] == "WARN"
    assert check(ir=100_500)["verdict"]["status"] == "FAIL"  # geomean boundary
    assert check(ir=101_000)["verdict"]["status"] == "FAIL"  # per-row boundary
    assert check(ir=99_000)["verdict"]["status"] == "PASS"
    assert check(allocs=11)["verdict"]["status"] == "FAIL"
    assert check(allocs=9)["verdict"]["status"] == "PASS"
    assert (
        check(
            ir=102_000,
            allocs=11,
            body="Perf-Regression-Rationale: a/dart: intended work",
        )["verdict"]["status"]
        != "FAIL"
    )
    assert (
        check(ir=102_000, body="Perf-Regression-Rationale: a/ode: intended work")[
            "verdict"
        ]["status"]
        == "FAIL"
    )
    assert check(changed=True)["results"][0]["delta"]["class"] == "behaviour-change"
    assert check(changed=True)["verdict"]["status"] == "FAIL"
    assert (
        check(
            changed=True,
            ir=101_000,
            body="Rebaseline-Rationale: a/dart: intended behaviour",
        )["verdict"]["status"]
        == "PASS"
    )
    for percentage in ("", "2.0%", "-2%", "+2.0%"):
        record = check(
            changed=True,
            ir=102_000,
            body=f"Rebaseline-Rationale: a/dart: {percentage} Ir; intended behaviour",
        )
        assert (record["verdict"]["status"] == "PASS") == percentage.startswith(
            ("+", "-")
        )
        assert record["verdict"]["ir_geomean"] is None
    assert check(status="broken")["results"][0]["delta"]["class"] == "broken"
    assert check(gated=False)["verdict"]["status"] == "FAIL"  # head cannot weaken base
    diagnostic = copy.deepcopy(base)
    diagnostic["results"][0]["gated"] = False
    assert (
        module.compare(diagnostic, diagnostic)["results"][0]["delta"]["class"]
        == "diagnostic"
    )
    new = copy.deepcopy(base)
    new["results"][0]["version"] = 2
    assert module.compare(base, new)["results"][0]["delta"]["class"] == "new"
    assert module.compare({**base, "results": []}, new)["verdict"]["status"] == "PASS"
    missing = copy.deepcopy(base)
    missing["results"][0]["head"].pop("ir_per_step")
    assert module.compare(base, missing)["verdict"]["status"] == "FAIL"
    missing["results"][0].update(expected_ir=False, method="native")
    missing["results"][0]["head"]["guards"]["hash"] = "0x2"
    assert (
        module.compare(
            base, missing, "Rebaseline-Rationale: a/dart: intended behaviour"
        )["verdict"]["status"]
        == "ERROR"
    )
    missing["results"] = []
    assert module.compare(base, missing)["verdict"]["status"] == "FAIL"
    nonfinite = copy.deepcopy(base)
    nonfinite["results"][0]["head"]["guards"]["finite"] = False
    assert (
        module.compare(
            base, nonfinite, "Rebaseline-Rationale: a/dart: intended behaviour"
        )["verdict"]["status"]
        == "FAIL"
    )
    multiple = copy.deepcopy(base)
    second = copy.deepcopy(multiple["results"][0])
    second["row"] = "b"
    multiple["results"].append(second)
    head = copy.deepcopy(multiple)
    for row in head["results"]:
        row["head"]["ir_per_step"] = 100_600
    assert (
        module.compare(
            multiple, head, "Perf-Regression-Rationale: a/dart: intended work"
        )["verdict"]["status"]
        == "FAIL"
    )
    assert (
        module.compare(
            multiple, head, "Perf-Regression-Rationale: a/dart, b/dart: intended work"
        )["verdict"]["status"]
        != "FAIL"
    )
    # A changed row never hides an unchanged row's regression.
    head["results"][0]["head"]["guards"]["hash"] = "0x2"
    assert (
        module.compare(
            multiple, head, "Rebaseline-Rationale: a/dart: intended behaviour"
        )["verdict"]["status"]
        == "FAIL"
    )
    assert (
        module.compare(
            nonfinite, base, "Rebaseline-Rationale: a/dart: intended behaviour"
        )["verdict"]["status"]
        == "FAIL"
    )
    empty = {**base, "results": []}
    assert module.compare(empty, empty)["verdict"]["status"] == "FAIL"
    assert "Wall time" in module.markdown(same)
    assert "-0.0001%" in module.markdown(check(ir=99_999.9))

    # Environment incompatibility rejects the entire comparison before deltas.
    for key in ("compiler", "valgrind", "pixi_lock_sha", "host_cpu", "harness_sha"):
        head = copy.deepcopy(base)
        head["run"]["env"].update({key: "different", "fingerprint": "different"})
        record = module.compare(base, head)
        assert record["verdict"]["status"] == "ERROR"
        assert record["results"] == []
        assert key in module.markdown(record)
    for value in (None, "different"):
        head = copy.deepcopy(base)
        head["run"]["env"]["fingerprint"] = value
        assert module.compare(base, head)["verdict"]["status"] == "ERROR"

    multiple["results"][1]["threads"] = 4
    report = module.markdown(module.compare(multiple, multiple))
    assert "1/4 threads" in report
    assert "| b/dart | 4 |" in report
    assert "base perturbation check passed" in report
    report = module.markdown(module.compare(diagnostic, diagnostic))
    assert "perturbation check not run" in report
    unstable = copy.deepcopy(diagnostic)
    unstable["results"][0]["perturbations"] = {"start4k": {"stable": False}}
    for parent, child in ((unstable, diagnostic), (diagnostic, unstable)):
        record = module.compare(parent, child)
        assert record["verdict"]["status"] == "FAIL"
        assert "perturbation check failed" in module.markdown(record)

    infrastructure = copy.deepcopy(base)
    infrastructure["results"][0].update(
        status="broken", error_kind="infrastructure", error="binary missing"
    )
    for parent, child in ((base, infrastructure), (infrastructure, base)):
        record = module.compare(parent, child)
        assert record["verdict"]["status"] == "ERROR"
        assert "binary missing" in module.markdown(record)


def _load_runner():
    import sys

    path = Path(__file__).resolve().parents[1] / "scripts/perf_regression.py"
    spec = importlib.util.spec_from_file_location("perf_regression", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_fingerprint_uses_installed_build_provenance(monkeypatch, tmp_path):
    module = _load_runner()
    args = module.parser().parse_args(
        [
            "run",
            "--commit",
            "installed",
            "--prefix",
            str(tmp_path),
            "--output-dir",
            str(tmp_path),
            "--rows",
            "pend",
        ]
    )
    args.bin_dir = tmp_path / "bin"
    args.bin_dir.mkdir()
    for name in ("portable_step_bench", "contact_benchmark"):
        (args.bin_dir / name).write_bytes(b"measured driver")
    args.shim = tmp_path / "allocshim.so"
    args.shim.write_bytes(b"measured shim")
    args.heappad = tmp_path / "heappad.so"
    args.heappad.write_bytes(b"heappad")
    (tmp_path / "share/dart").mkdir(parents=True)
    (tmp_path / "lib").mkdir()
    library = tmp_path / "lib/libdart.so"
    library.write_bytes(b"measured artifact")
    stamp = {
        "schema": "dart-perf-build/1",
        "commit": "installed",
        "compiler": "Clang 18.1.8",
        "pixi_lock_sha": "installed lock",
        "preset": "installed preset",
        "libdart_sha": module.sha(library.read_bytes()),
        "libraries": {"lib/libdart.so": module.sha(library.read_bytes())},
        "workload_sources": {module.CB: module.sha(b"installed workload")},
        "binaries": {
            name: module.sha(b"measured driver")
            for name in ("portable_step_bench", "contact_benchmark")
        },
    }
    path = tmp_path / "share/dart/perf-build.json"
    module.write_json(path, stamp)
    monkeypatch.setattr(module, "execute", lambda *args: "Guest CPU: test\n")

    def command_output(command):
        assert command[0] in (module.VALGRIND, "getconf")
        return "valgrind-3.22.0" if command[0] == module.VALGRIND else "glibc 2.39"

    monkeypatch.setattr(module, "command_output", command_output)
    first = module.fingerprint(args)
    for key in ("compiler", "pixi_lock_sha", "preset"):
        assert first[key] == stamp[key]
    stamp["compiler"] = "GNU 13.3.0"
    module.write_json(path, stamp)
    second = module.fingerprint(args)
    assert first["fingerprint"] != second["fingerprint"]
    result = module.compare(
        {"run": {"commit": "base", "env": first}},
        {"run": {"commit": "head", "env": second}},
    )
    assert result["verdict"]["status"] == "ERROR"
    assert "compiler" in result["verdict"]["failures"][0]
    driver = args.bin_dir / "contact_benchmark"
    driver.write_bytes(b"replaced driver")
    with pytest.raises(ValueError, match="driver hash differs"):
        module.fingerprint(args)
    driver.write_bytes(b"measured driver")
    args.shim.write_bytes(b"different shim")
    assert module.fingerprint(args)["fingerprint"] != second["fingerprint"]
    for key, value, reason in (
        ("compiler", "", "invalid installed build provenance"),
        ("compiler", None, "invalid installed build provenance"),
        ("schema", "other", "invalid installed build provenance"),
        ("binaries", None, "invalid installed build provenance"),
        ("workload_sources", None, "invalid installed build provenance"),
        ("workload_sources", {}, "invalid installed build provenance"),
        (
            "workload_sources",
            {module.CB: "invalid"},
            "invalid installed build provenance",
        ),
        ("pixi_lock_sha", "", "invalid installed build provenance"),
        ("preset", "", "invalid installed build provenance"),
        ("commit", "other", "commit differs"),
        ("libdart_sha", "stale", "libdart hash differs"),
    ):
        module.write_json(path, {**stamp, key: value})
        with pytest.raises(ValueError, match=reason):
            module.fingerprint(args)
    module.write_json(path, [])
    with pytest.raises(ValueError, match="invalid installed build provenance"):
        module.fingerprint(args)
    module.write_json(
        path, {key: value for key, value in stamp.items() if key != "workload_sources"}
    )
    with pytest.raises(ValueError, match="invalid installed build provenance"):
        module.fingerprint(args)
    module.write_json(path, stamp)
    library.write_bytes(b"replaced artifact")
    with pytest.raises(ValueError, match="libdart hash differs"):
        module.fingerprint(args)
    path.unlink()
    with pytest.raises(ValueError, match="missing installed compiler provenance"):
        module.fingerprint(args)


@pytest.mark.parametrize("defect", ["changed", "added", "removed", "missing manifest"])
def test_installed_provenance_verifies_all_dart_libraries(tmp_path, defect):
    module = _load_runner()
    args = module.parser().parse_args(
        [
            "run",
            "--commit",
            "installed",
            "--prefix",
            str(tmp_path),
            "--output-dir",
            str(tmp_path),
        ]
    )
    library = tmp_path / "lib/libdart.so"
    library.parent.mkdir()
    library.write_bytes(b"core")
    component = tmp_path / "lib/libdart-utils.so.6.20"
    component.write_bytes(b"utils")
    (tmp_path / "lib/libdart-utils.so").symlink_to(component.name)
    collision = tmp_path / "lib/libdart-collision-ode.so"
    collision.write_bytes(b"collision")
    stamp = {
        "schema": "dart-perf-build/1",
        "commit": "installed",
        "compiler": "GNU 13.3.0",
        "pixi_lock_sha": "installed lock",
        "preset": "perf-1",
        "libdart_sha": module.sha(b"core"),
        "libraries": {
            "lib/libdart.so": module.sha(b"core"),
            "lib/libdart-utils.so": module.sha(b"utils"),
            "lib/libdart-utils.so.6.20": module.sha(b"utils"),
            "lib/libdart-collision-ode.so": module.sha(b"collision"),
        },
        "binaries": {},
        "workload_sources": {},
    }
    path = tmp_path / "share/dart/perf-build.json"
    path.parent.mkdir(parents=True)
    module.write_json(path, stamp)
    assert module.installed_provenance(args) == stamp
    if defect == "changed":
        component.write_bytes(b"stale utils")
    elif defect == "added":
        (tmp_path / "lib/libdart-collision-bullet.so").write_bytes(b"mixed install")
    elif defect == "removed":
        collision.unlink()
    else:
        stamp.pop("libraries")
        module.write_json(path, stamp)
    assert module.sha(library.read_bytes()) == stamp["libdart_sha"]
    reason = (
        "invalid installed build provenance"
        if defect == "missing manifest"
        else "DART library hashes differ"
    )
    with pytest.raises(ValueError, match=reason):
        module.installed_provenance(args)


def test_fingerprint_includes_active_runtime_environment(monkeypatch, tmp_path):
    module = _load_runner()
    args = module.parser().parse_args(
        [
            "run",
            "--commit",
            "installed",
            "--prefix",
            str(tmp_path),
            "--output-dir",
            str(tmp_path),
            "--rows",
            "pend",
        ]
    )
    args.bin_dir = tmp_path / "bin"
    args.bin_dir.mkdir()
    for name in ("portable_step_bench", "contact_benchmark"):
        (args.bin_dir / name).write_bytes(b"driver")
    args.shim = tmp_path / "allocshim.so"
    args.shim.write_bytes(b"shim")
    args.heappad = tmp_path / "heappad.so"
    args.heappad.write_bytes(b"heappad")
    stamp = {
        "schema": "dart-perf-build/1",
        "compiler": "GNU 13.3.0",
        "pixi_lock_sha": "unchanged build lock",
        "preset": "perf-1",
        "binaries": {
            name: module.sha(b"driver")
            for name in ("portable_step_bench", "contact_benchmark")
        },
    }
    monkeypatch.setattr(module, "installed_provenance", lambda args: stamp)
    monkeypatch.setattr(module, "execute", lambda *args: "Guest CPU: test\n")
    monkeypatch.setattr(
        module,
        "command_output",
        lambda command: (
            "valgrind-3.22.0" if command[0] == module.VALGRIND else "glibc 2.39"
        ),
    )
    monkeypatch.setenv("PIXI_PROJECT_ROOT", str(tmp_path))
    monkeypatch.setenv("PIXI_ENVIRONMENT_NAME", "default")
    monkeypatch.setenv("CONDA_PREFIX", str(tmp_path / ".pixi/envs/default"))
    lock = tmp_path / "pixi.lock"
    lock.write_bytes(b"runtime lock")
    first = module.fingerprint(args)
    assert first["runtime_pixi_lock_sha"] == module.sha(b"runtime lock")
    assert first["runtime_environment"] == "default"
    lock.write_bytes(b"different runtime lock")
    changed_lock = module.fingerprint(args)
    lock.write_bytes(b"runtime lock")
    monkeypatch.setenv("PIXI_ENVIRONMENT_NAME", "gazebo")
    monkeypatch.setenv("CONDA_PREFIX", str(tmp_path / ".pixi/envs/gazebo"))
    changed_environment = module.fingerprint(args)
    for changed, field in (
        (changed_lock, "runtime_pixi_lock_sha"),
        (changed_environment, "runtime_environment"),
    ):
        assert changed["pixi_lock_sha"] == first["pixi_lock_sha"]
        assert changed["fingerprint"] != first["fingerprint"]
        result = module.compare(
            {"run": {"commit": "base", "env": first}},
            {"run": {"commit": "head", "env": changed}},
        )
        assert result["verdict"]["status"] == "ERROR"
        assert field in result["verdict"]["failures"][0]


@pytest.mark.parametrize("missing", ["base", "head", "both"])
@pytest.mark.parametrize("field", ["compiler", "compiler_provenance"])
def test_saved_records_require_compiler_provenance(missing, field):
    module = _load_runner()
    base = _micro_record(module, "dyn")
    base["run"]["env"].update(valgrind="test", glibc="test", preset="perf-1")
    head = copy.deepcopy(base)
    for record in (
        [base, head] if missing == "both" else [base if missing == "base" else head]
    ):
        record["run"]["env"].pop(field)
    result = module.compare(base, head)
    assert result["verdict"]["status"] == "ERROR"
    assert "compiler provenance" in result["verdict"]["failures"][0]
    assert "compiler provenance" in module.markdown(result)


@pytest.mark.parametrize(
    "ir,allocs,gated,body,status,summary",
    [
        (100, 0, True, "", "PASS", "neutral"),
        (99, 0, True, "", "PASS", "improved"),
        (100.001, 0, True, "", "PASS", "regressed"),
        (100.3, 0, True, "", "WARN", "regressed"),
        (100.5, 0, True, "", "FAIL", "regressed"),
        (101, 0, True, "", "FAIL", "regressed"),
        (99, 1, True, "", "FAIL", "regressed"),
        (None, 1, True, "", "FAIL", "regressed"),
        (99, 0, False, "", "FAIL", "regressed"),
        (
            101,
            1,
            True,
            "Perf-Regression-Rationale: dyn: intended work",
            "WARN",
            "regressed",
        ),
    ],
)
def test_summary_separates_gated_regressions(ir, allocs, gated, body, status, summary):
    module = _load_runner()
    base = _micro_record(module, "dyn")
    base["run"]["env"].update(valgrind="test", glibc="test", preset="perf-1")
    head = copy.deepcopy(base)
    head["results"][0]["head"].update(ir_per_step=ir, allocs_per_step=allocs)
    head["results"][0]["gated"] = gated
    if ir is None:
        for record in (base, head):
            record["results"][0]["method"] = "native"
            record["results"][0]["head"].pop("ir_per_step")
    result = module.compare(base, head, body)
    assert result["verdict"]["status"] == status
    report = module.markdown(result)
    assert f"1 {summary}." in report
    if summary == "regressed":
        assert "1 neutral" not in report and "1 improved" not in report


def test_allocation_shims_count_each_entry_once(tmp_path):
    import os
    import re
    import subprocess
    import sys

    if sys.platform != "linux" or not Path("/usr/bin/cc").is_file():
        pytest.skip("the harness shims require GNU libc and the system C compiler")
    sources = Path(__file__).resolve().parents[1] / "tools/perf"
    for name in ("allocshim", "heappad", "allocation_probe"):
        command = [
            "/usr/bin/cc",
            "-O2",
            "-o",
            str(tmp_path / name),
            str(sources / f"{name}.c"),
            "-ldl",
        ]
        if name != "allocation_probe":
            command += ["-shared", "-fPIC"]
        subprocess.run(command, check=True, capture_output=True)
    env = os.environ.copy()
    for key in ("LD_PRELOAD", "HEAPPAD", "PERF_WARMUP", "PERF_WINDOW"):
        env.pop(key, None)
    module = _load_runner()
    for mode in ("", "unset", *module.PERTURBATIONS):
        current = {**env, "LD_PRELOAD": str(tmp_path / "allocshim")}
        if mode == "tcache0":
            current["GLIBC_TUNABLES"] = "glibc.malloc.tcache_count=0"
        elif mode:
            current["LD_PRELOAD"] = f"{tmp_path}/allocshim:{tmp_path}/heappad"
            if mode != "unset":
                current["HEAPPAD"] = mode
        for entry in (
            "malloc",
            "calloc",
            "realloc",
            "memalign",
            "aligned_alloc",
            "posix_memalign",
        ):
            result = subprocess.run(
                [str(tmp_path / "allocation_probe"), entry],
                env=current,
                check=True,
                capture_output=True,
                text=True,
            )
            counts = re.search(
                r"STEPALLOC steps=(\d+) measured=(\d+) allocs=(\d+) bytes=(\d+)",
                result.stderr,
            )
            assert counts and tuple(map(int, counts.groups())) == (1, 1, 1, 64), (
                mode,
                entry,
                result.stderr,
            )
    print(
        "Allocation probe: all 6 entry points count once (64 requested bytes), baseline and all 7 perturbations"
    )


@pytest.mark.parametrize("version", [1, 2])
def test_changed_input_requires_rebaseline_without_deltas(version):
    module = _load_runner()
    base = {
        "run": {
            "commit": "base",
            "env": {
                "fingerprint": "same",
                "compiler": "test",
                "compiler_provenance": "dart-perf-build/1",
            },
        },
        "results": [
            {
                "row": "pend",
                "det": "dart",
                "version": 1,
                "input_sha": "base input",
                "gated": True,
                "status": "ok",
                "method": "slope",
                "head": {
                    "ir_per_step": 100,
                    "allocs_per_step": 0,
                    "bytes_per_step": 0,
                    "max_rss_kb": 100,
                    "guards": {
                        "hash": "same",
                        "finite": True,
                        "contacts": 0,
                        "cap_hit": False,
                        "resting": "0/1",
                    },
                },
            }
        ],
    }
    # Report metadata is identical across arms, as is the environment fingerprint.
    base["run"]["env"].update(
        valgrind="test", compiler="test", glibc="test", preset="perf-1"
    )
    head = copy.deepcopy(base)
    head["results"][0]["version"] = version
    head["results"][0]["input_sha"] = "head input"
    head["results"][0]["head"].update(
        ir_per_step=200, allocs_per_step=10, max_rss_kb=200
    )
    for body, status in (
        ("", "FAIL"),
        ("Perf-Regression-Rationale: pend/dart: changed input", "FAIL"),
        ("Rebaseline-Rationale: robot/dart: changed input", "FAIL"),
        ("Rebaseline-Rationale: pend/dart: changed input", "PASS"),
    ):
        record = module.compare(base, head, body)
        assert record["verdict"]["status"] == status
        assert record["verdict"]["ir_geomean"] is None
        assert record["verdict"]["warnings"] == []
        assert record["results"][0]["delta"] == {
            "ir": None,
            "allocs": None,
            "bytes": None,
            "guards_equal": True,
            "class": "behaviour-change",
        }
        assert "input_sha changed" in module.markdown(record)


@pytest.mark.parametrize("name", ["s1p/dart", "dyn", "lcp", "gzb", "robot"])
@pytest.mark.parametrize("changed", [False, True])
def test_run_workload_stamp_controls_rebaseline(monkeypatch, tmp_path, name, changed):
    module = _load_runner()
    row = module.select_rows(name)[0]
    args = module.parser().parse_args(
        [
            "run",
            "--commit",
            "installed",
            "--prefix",
            str(tmp_path),
            "--output-dir",
            str(tmp_path / "run"),
            "--rows",
            name,
            "--no-perturb",
            "--shim",
            str(tmp_path / "allocshim.so"),
        ]
    )
    args.shim.write_bytes(b"shim")
    library = tmp_path / "lib/libdart.so"
    library.parent.mkdir()
    library.write_bytes(b"DART")
    drivers = {module.PB, row.driver}
    stamp = {
        "schema": "dart-perf-build/1",
        "commit": "installed",
        "compiler": "test",
        "pixi_lock_sha": "lock",
        "preset": "perf-1",
        "libdart_sha": module.sha(b"DART"),
        "libraries": {"lib/libdart.so": module.sha(b"DART")},
        "binaries": {driver: module.sha(b"binary") for driver in drivers},
        "workload_sources": {
            driver: module.sha(b"base workload")
            for driver in drivers & module.WORKLOAD_SOURCES.keys()
        },
    }
    path = tmp_path / "share/dart/perf-build.json"
    path.parent.mkdir(parents=True)
    module.write_json(path, stamp)
    monkeypatch.setattr(module, "command_output", lambda command: "installed")
    monkeypatch.setattr(
        module,
        "fingerprint",
        lambda args, provenance: {
            "fingerprint": "same",
            "compiler": "test",
            "compiler_provenance": "dart-perf-build/1",
        },
    )
    metrics = (
        _micro_record(module, name)["results"][0]["head"]
        if not row.det
        else {
            "ir_per_step": 100,
            "allocs_per_step": 0,
            "bytes_per_step": 0,
            "guards": {
                "hash": "same",
                "finite": True,
                "contacts": 0,
                "cap_hit": False,
                "resting": "0/1",
            },
        }
    )
    monkeypatch.setattr(
        module,
        "measure",
        lambda row, args, world: {
            "row": row.row,
            "det": row.det,
            "version": row.version,
            "parity": "",
            "status": "ok",
            "gated": True,
            "method": "slope",
            "input_sha": "scene input",
            "head": copy.deepcopy(metrics),
        },
    )
    base = module.run_arm(args)
    if changed:
        stamp["workload_sources"] = {
            driver: module.sha(b"head workload") for driver in stamp["workload_sources"]
        }
        module.write_json(path, stamp)
    head = module.run_arm(args)
    result = module.compare(base, head)
    workload_changed = changed and row.driver != module.PB
    assert result["verdict"]["status"] == ("FAIL" if workload_changed else "PASS")
    delta = result["results"][0]["delta"]
    assert delta["class"] == ("behaviour-change" if workload_changed else "gated")
    if row.driver == module.PB:
        assert head["results"][0]["input_sha"] == "scene input"
        assert "workload_sha" not in head["results"][0]
    else:
        assert (
            head["results"][0]["workload_sha"] == stamp["workload_sources"][row.driver]
        )
    if workload_changed:
        assert all(delta[key] is None for key in ("ir", "allocs", "bytes"))
        assert "Rebaseline-Rationale required" in result["verdict"]["failures"][0]
        assert (
            module.compare(
                base, head, f"Perf-Regression-Rationale: {row.key}: workload changed"
            )["verdict"]["status"]
            == "FAIL"
        )
        assert (
            module.compare(
                base, head, f"Rebaseline-Rationale: {row.key}: workload changed"
            )["verdict"]["status"]
            == "PASS"
        )
    stamp.pop("workload_sources")
    module.write_json(path, stamp)
    with pytest.raises(ValueError, match="invalid installed build provenance"):
        module.run_arm(args)


@pytest.mark.parametrize(
    "changed_path",
    sorted(
        {path for paths in _load_runner().WORKLOAD_SOURCES.values() for path in paths}
    ),
)
def test_workload_hashes_cover_sources_and_headers(tmp_path, changed_path):
    module = _load_runner()
    for paths in module.WORKLOAD_SOURCES.values():
        for name in paths:
            path = tmp_path / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b"workload")
    drivers = [*module.WORKLOAD_SOURCES, module.PB]
    base = module.workload_hashes(tmp_path, drivers)
    assert module.PB not in base
    # Library implementation changes are what the harness measures.
    library = tmp_path / "dart/library.cpp"
    library.parent.mkdir()
    library.write_bytes(b"changed DART")
    assert module.workload_hashes(tmp_path, drivers) == base
    path = tmp_path / changed_path
    path.write_bytes(b"changed workload")
    head = module.workload_hashes(tmp_path, drivers)
    for driver, paths in module.WORKLOAD_SOURCES.items():
        assert (base[driver] != head[driver]) == (changed_path in paths)
    if path.suffix == ".hpp":
        path.unlink()
        assert module.workload_hashes(tmp_path, drivers) != base
    added = tmp_path / "examples/contact_benchmark/new_case.hpp"
    added.write_bytes(b"new case")
    assert module.workload_hashes(tmp_path, drivers)[module.CB] != head[module.CB]
    # The target globs its sources, so a renamed entry point is a change, not
    # a missing source; a directory without sources is.
    main = tmp_path / "examples/contact_benchmark/main.cpp"
    main.rename(main.with_name("entry.cpp"))
    renamed = module.workload_hashes(tmp_path, drivers)[module.CB]
    assert renamed != head[module.CB]
    main.with_name("entry.cpp").unlink()
    with pytest.raises(ValueError, match="missing workload source"):
        module.workload_hashes(tmp_path, drivers)


def test_local_resets_only_marked_default_output(monkeypatch, tmp_path):
    module = _load_runner()
    monkeypatch.setattr(module, "ROOT", tmp_path)
    monkeypatch.delenv("CONDA_PREFIX", raising=False)
    output = tmp_path / "build/perf-compare"
    args = module.parser().parse_args(["local"])
    # Stop before building; initialization still marks the default directory.
    for _ in range(2):
        with pytest.raises(ValueError, match="active Pixi environment"):
            module.local_arms(args)
        assert (output / ".perf-compare-owned").is_file()
        assert not (output / "stale").exists()
        (output / "stale").write_text("previous run", encoding="utf-8")
    (output / ".perf-compare-owned").unlink()
    with pytest.raises(ValueError, match="must be empty"):
        module.local_arms(args)
    assert (output / "stale").read_text(encoding="utf-8") == "previous run"
    (output / ".perf-compare-owned").symlink_to(output / "stale")
    with pytest.raises(ValueError, match="must be empty"):
        module.local_arms(args)
    assert (output / "stale").is_file()
    (output / ".perf-compare-owned").unlink()
    (output / ".perf-compare-owned").write_text("dart-perf/1\n", encoding="utf-8")
    alias = tmp_path / "alias"
    alias.symlink_to(output, target_is_directory=True)
    args.output_dir = alias
    with pytest.raises(ValueError, match="must be empty"):
        module.local_arms(args)
    assert (output / "stale").is_file()

    custom = tmp_path / "custom"
    custom.mkdir()
    (custom / ".perf-compare-owned").write_text("dart-perf/1\n", encoding="utf-8")
    args.output_dir = custom
    with pytest.raises(ValueError, match="must be empty"):
        module.local_arms(args)
    assert (custom / ".perf-compare-owned").is_file()


def test_run_requires_installed_commit():
    module = _load_runner()
    command = ["run", "--prefix", "/install", "--output-dir", "/output"]
    with pytest.raises(SystemExit) as error:
        module.parser().parse_args(command)
    assert error.value.code == 2
    assert (
        module.parser().parse_args([*command, "--commit", "installed"]).commit
        == "installed"
    )
    assert module.parser().parse_args(["local"]).head == "HEAD"


@pytest.mark.parametrize(
    "changed", [None, "hash", "contacts", "cap_hit", "resting", "finite"]
)
def test_multithread_parity_compares_all_guards(monkeypatch, tmp_path, changed):
    module = _load_runner()
    args = module.parser().parse_args(
        [
            "run",
            "--commit",
            "HEAD",
            "--prefix",
            str(tmp_path),
            "--output-dir",
            str(tmp_path / "run"),
            "--rows",
            "mt4-s3w",
            "--no-perturb",
            "--shim",
            str(tmp_path / "allocshim.so"),
        ]
    )
    args.shim.write_bytes(b"shim")
    monkeypatch.setattr(module, "command_output", lambda command: "HEAD")
    monkeypatch.setattr(module, "fingerprint", lambda args, provenance: {})
    monkeypatch.setattr(
        module,
        "installed_provenance",
        lambda args: {"workload_sources": {module.CB: module.sha(b"workload")}},
    )

    def measure(row, args, world):
        guards = dict(
            hash="same", contacts=1, cap_hit=False, resting="0/1", finite=True
        )
        if row.parity and changed:
            guards[changed] = {
                "hash": "different",
                "contacts": 2,
                "cap_hit": True,
                "resting": "1/1",
                "finite": False,
            }[changed]
        return {
            "row": row.row,
            "det": row.det,
            "parity": row.parity,
            "status": "ok",
            "input_sha": "scene input",
            "head": {"guards": guards},
        }

    monkeypatch.setattr(module, "measure", measure)
    results = module.run_arm(args)["results"]
    assert len(results) == 2  # The serial counterpart is added automatically.
    parallel = next(result for result in results if result["parity"])
    serial = next(result for result in results if not result["parity"])
    assert serial["status"] == "ok"
    assert parallel["status"] == ("broken" if changed else "ok")
    if changed:
        assert parallel["error"] == "mt4 guard parity missing or unequal"


def test_measure_gates_only_on_its_own_perturbation_pass(monkeypatch, tmp_path):
    module = _load_runner()
    args = module.parser().parse_args(
        [
            "run",
            "--commit",
            "HEAD",
            "--prefix",
            str(tmp_path),
            "--output-dir",
            str(tmp_path),
            "--native-only",
            "--no-perturb",
        ]
    )
    row = module.select_rows("pend")[0]
    metrics = {"guards": {"finite": True}, "allocs": 0}
    monkeypatch.setattr(module, "native", lambda *args: copy.deepcopy(metrics))
    # Without this run's heap-layout checks, no row gates.
    assert module.measure(row, args, tmp_path)["gated"] is False
    # The checks run by default.
    assert (
        module.parser()
        .parse_args(["run", "--commit", "HEAD", "--prefix", ".", "--output-dir", "."])
        .perturb
    )
    args.perturb = True
    measured = module.measure(row, args, tmp_path)
    assert measured["gated"] is True
    assert set(measured["perturbations"]) == set(module.PERTURBATIONS)
    monkeypatch.setattr(
        module,
        "native",
        lambda row, args, world, config="": {**metrics, "allocs": int(bool(config))},
    )
    assert module.measure(row, args, tmp_path)["gated"] is False
    # Requested bytes that change with the heap layout also disqualify the row.
    monkeypatch.setattr(
        module,
        "native",
        lambda row, args, world, config="": {**metrics, "bytes": 64 + bool(config)},
    )
    assert module.measure(row, args, tmp_path)["gated"] is False


def test_environment_ignores_inherited_library_path(monkeypatch, tmp_path):
    module = _load_runner()
    monkeypatch.setenv("LD_LIBRARY_PATH", "/elsewhere/lib")
    monkeypatch.setenv("CONDA_PREFIX", str(tmp_path / "env"))
    env = module.environment(tmp_path / "prefix")
    assert env["LD_LIBRARY_PATH"] == f"{tmp_path}/prefix/lib:{tmp_path}/env/lib"


@pytest.mark.parametrize(
    "error",
    [OSError("binary missing"), ValueError("timeout"), ValueError("Valgrind failed")],
)
def test_execution_errors_exit_two_with_reason(monkeypatch, tmp_path, capsys, error):
    module = _load_runner()
    argv = [
        "run",
        "--commit",
        "HEAD",
        "--prefix",
        str(tmp_path),
        "--output-dir",
        str(tmp_path),
    ]
    args = module.parser().parse_args(argv)

    def fail(*args):
        raise error

    monkeypatch.setattr(module, "native", fail)
    broken = module.measure(module.select_rows("pend")[0], args, tmp_path)
    assert broken["status"] == "broken"
    assert broken["error_kind"] == "infrastructure"
    assert broken["error"] == str(error)
    monkeypatch.setattr(module, "run_arm", lambda args: {"results": [broken]})
    assert module.main(argv) == 2
    assert str(error) in capsys.readouterr().out
    broken.pop("error_kind")
    assert module.main(argv) == 1


@pytest.mark.parametrize(
    "returncode, guard_output, correctness",
    [
        (1, "complete", True),
        (1, "missing", False),
        (1, "partial", False),
        (1, "finite", False),
        (1, "invalid", False),
        # contact_benchmark exits 2 after printing complete non-finite guards.
        (2, "complete", True),
        (2, "finite", False),
        (3, "missing", False),
    ],
)
def test_driver_nonfinite_exit_is_correctness_failure(
    monkeypatch, tmp_path, capsys, returncode, guard_output, correctness
):
    module = _load_runner()
    args = module.parser().parse_args(
        [
            "run",
            "--commit",
            "HEAD",
            "--prefix",
            str(tmp_path),
            "--output-dir",
            str(tmp_path),
        ]
    )
    args.bin_dir = tmp_path
    row = module.select_rows("gzb")[0]
    finite = "true"
    output = "complete"
    exit_status = 0

    def popen(command, **kwargs):
        text = (
            "Avg Step Time: 1.0 ms/step\n"
            f"STEPALLOC steps=5 measured=3 allocs=0 bytes=0 libdart={tmp_path}/lib/libdart.so\n"
            "PERFTIME maxrss_kb=100\n"
        )
        if output != "missing":
            text += (
                "Final State Hash: 0x0123456789abcdef\n"
                f"Final State Finite: {finite}\n"
                "Final Contacts: 1\n"
                "Final Contact Cap Hit: false\n"
            )
            if output != "partial":
                text += "Final Resting: 0 / 1\n"
        kwargs["stdout"].write(text)
        return module.argparse.Namespace(
            returncode=exit_status, wait=lambda **kwargs: None
        )

    monkeypatch.setattr(module.subprocess, "Popen", popen)
    monkeypatch.setattr(
        module, "callgrind", lambda row, args, world, steps: {"Ir": steps * 100}
    )
    supported = module.measure(row, args, tmp_path)
    assert supported["status"] == "ok"

    # A completed correctness failure must stop before additional measurements.
    def unexpected_callgrind(*args):
        pytest.fail("Callgrind ran after a non-finite state")

    monkeypatch.setattr(module, "callgrind", unexpected_callgrind)
    output, exit_status = guard_output, returncode
    finite = (
        "true" if output == "finite" else "invalid" if output == "invalid" else "false"
    )
    broken = module.measure(row, args, tmp_path)
    assert broken["status"] == "broken"
    if correctness:
        assert broken["head"]["guards"]["finite"] is False
        assert broken["error"] == "non-finite state"
        assert "error_kind" not in broken
        assert "perturbations" not in broken
    else:
        assert broken["error_kind"] == "infrastructure"
        assert broken["error"].startswith(f"exit {returncode}:")

    env = {
        "fingerprint": "same",
        "valgrind": "test",
        "compiler": "test",
        "compiler_provenance": "dart-perf-build/1",
        "glibc": "test",
        "preset": "perf-1",
    }
    base = {
        "schema": "dart-perf/1",
        "run": {"commit": "base", "env": env},
        "results": [supported],
    }
    head = {
        "schema": "dart-perf/1",
        "run": {"commit": "head", "env": env},
        "results": [broken],
    }
    comparison = module.compare(base, head)
    status = "FAIL" if correctness else "ERROR"
    assert comparison["verdict"]["status"] == status
    assert comparison["results"][0]["delta"]["class"] == "broken"
    base_path, head_path = tmp_path / "base.json", tmp_path / "head.json"
    module.write_json(base_path, base)
    module.write_json(head_path, head)
    assert module.main(
        ["compare", "--base", str(base_path), "--head", str(head_path)]
    ) == (1 if correctness else 2)
    assert status in capsys.readouterr().out


@pytest.mark.parametrize(
    "phase", ["native", "perturb", "callgrind-before", "callgrind-after"]
)
def test_unsupported_driver_rows_are_not_infrastructure_errors(
    monkeypatch, tmp_path, capsys, phase
):
    module = _load_runner()
    argv = [
        "run",
        "--commit",
        "HEAD",
        "--prefix",
        str(tmp_path),
        "--output-dir",
        str(tmp_path),
    ]
    args = module.parser().parse_args(argv)
    args.bin_dir = tmp_path
    row = module.select_rows("gzb")[0]
    metrics = {
        "guards": {
            "hash": "0x1",
            "finite": True,
            "contacts": 1,
            "cap_hit": False,
            "resting": "0/1",
        },
        "allocs": 0,
        "allocs_per_step": 0,
        "bytes_per_step": 0,
    }

    def popen(command, **kwargs):
        kwargs["stdout"].write("UNSUPPORTED: maxNumContactsPerPair\n")
        return module.argparse.Namespace(returncode=3, wait=lambda **kwargs: None)

    monkeypatch.setattr(module.subprocess, "Popen", popen)
    native, callgrind = module.native, module.callgrind

    def run_native(row, args, world, config=""):
        if phase == "native" or phase == "perturb" and config:
            return native(row, args, world, config)
        return copy.deepcopy(metrics)

    def run_callgrind(row, args, world, steps):
        if phase == "callgrind-before" or steps == row.warmup + row.steps:
            return callgrind(row, args, world, steps)
        return {"Ir": 100}

    monkeypatch.setattr(module, "native", run_native)
    monkeypatch.setattr(module, "callgrind", run_callgrind)
    unsupported = module.measure(row, args, tmp_path)
    assert unsupported["status"] == "unsupported"
    assert unsupported["error"] == "maxNumContactsPerPair"
    assert "error_kind" not in unsupported
    assert not unsupported["head"] and not unsupported["perturbations"]
    monkeypatch.setattr(module, "run_arm", lambda args: {"results": [unsupported]})
    assert module.main(argv) == 0
    assert "maxNumContactsPerPair" in capsys.readouterr().out

    env = {
        "fingerprint": "same",
        "compiler": "test",
        "compiler_provenance": "dart-perf-build/1",
    }
    base = {"run": {"commit": "base", "env": env}, "results": [unsupported]}
    supported = copy.deepcopy(unsupported)
    supported.update(status="ok", gated=True, head={**metrics, "ir_per_step": 100})
    head = {"run": {"commit": "head", "env": env}, "results": [supported]}
    comparison = module.compare(base, head)
    assert comparison["verdict"]["status"] == "PASS"
    assert comparison["results"][0]["delta"]["class"] == "new"
    # Losing support in the head remains a policy failure, not an infrastructure error.
    assert module.compare(head, base)["verdict"]["status"] == "FAIL"


@pytest.mark.parametrize(
    "returncode, text", [(3, "API unavailable\n"), (1, "UNSUPPORTED: optional API\n")]
)
def test_unsupported_requires_exit_three_and_marker(
    monkeypatch, tmp_path, returncode, text
):
    module = _load_runner()

    def popen(command, **kwargs):
        kwargs["stdout"].write(text)
        return module.argparse.Namespace(
            returncode=returncode, wait=lambda **kwargs: None
        )

    monkeypatch.setattr(module.subprocess, "Popen", popen)
    with pytest.raises(ValueError, match=f"exit {returncode}") as error:
        module.execute(["driver"], {}, tmp_path / "driver.log", 10)
    assert not isinstance(error.value, module.UnsupportedRow)


@pytest.mark.parametrize(
    "layout",
    ["local", "fallback", "explicit", "missing-shim", "missing-heappad", "no-perturb"],
)
def test_run_resolves_shims_and_reports_missing_options(
    monkeypatch, tmp_path, capsys, layout
):
    module = _load_runner()
    root = tmp_path / "repo"
    world = root / "tests/benchmark/worlds/3k_shapes.sdf.gz"
    world.parent.mkdir(parents=True)
    world.write_bytes(
        (module.ROOT / "tests/benchmark/worlds/3k_shapes.sdf.gz").read_bytes()
    )
    monkeypatch.setattr(module, "ROOT", root)
    monkeypatch.setattr(module, "command_output", lambda command: "HEAD")
    monkeypatch.setattr(module, "fingerprint", lambda args, provenance: {})
    monkeypatch.setattr(module, "installed_provenance", lambda args: {})
    monkeypatch.setattr(
        module,
        "measure",
        lambda row, args, world: {
            "row": row.row,
            "det": row.det,
            "parity": "",
            "status": "ok",
            "gated": True,
        },
    )
    prefix = tmp_path / "output/a"
    argv = [
        "run",
        "--commit",
        "HEAD",
        "--prefix",
        str(prefix),
        "--output-dir",
        str(tmp_path / "run"),
        "--rows",
        "gzb",
    ]
    chosen = {}
    for option, name in (("shim", "allocshim"), ("heappad", "heappad")):
        local = prefix.parent / "shims" / f"{name}.so"
        fallback = root / "build/perf" / f"lib{name}.so"
        path = fallback if layout in ("fallback", f"missing-{option}") else local
        if layout == "explicit":
            local.parent.mkdir(parents=True, exist_ok=True)
            local.write_bytes(b"local shim")
            path = tmp_path / f"custom-{name}.so"
            argv += [f"--{option}", str(path)]
        if layout != f"missing-{option}" and not (
            layout == "no-perturb" and option == "heappad"
        ):
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b"shim")
        chosen[option] = path
    if layout == "no-perturb":
        argv.append("--no-perturb")
    if layout.startswith("missing-"):
        option = layout.removeprefix("missing-")
        assert module.main(argv) == 2
        error = capsys.readouterr().err
        assert f"--{option}" in error and str(chosen[option]) in error
    else:
        args = module.parser().parse_args(argv)
        record = module.run_arm(args)
        assert record["results"][0]["status"] == "ok"
        assert args.shim == chosen["shim"]
        if layout != "no-perturb":
            assert args.heappad == chosen["heappad"]


def test_compare_and_local_infrastructure_exit_two(monkeypatch, tmp_path, capsys):
    module = _load_runner()
    env = {
        "fingerprint": "same",
        "valgrind": "test",
        "compiler": "test",
        "compiler_provenance": "dart-perf-build/1",
        "glibc": "test",
        "preset": "perf-1",
    }
    record = {
        "schema": "dart-perf/1",
        "run": {"commit": "HEAD", "env": env},
        "results": [
            {
                "row": "pend",
                "det": "dart",
                "status": "broken",
                "method": "native",
                "error_kind": "infrastructure",
                "error": "timeout",
                "head": {},
            }
        ],
    }
    monkeypatch.setattr(module, "local_arms", lambda args: (record, record))
    assert module.main(["local"]) == 2
    assert "timeout" in capsys.readouterr().out
    base, head = tmp_path / "base.json", tmp_path / "head.json"
    module.write_json(base, record)
    altered = copy.deepcopy(record)
    altered["run"]["env"].update(fingerprint="different", compiler="other")
    module.write_json(head, altered)
    assert module.main(["compare", "--base", str(base), "--head", str(head)]) == 2
    assert "compiler" in capsys.readouterr().out


def _micro_record(module, name):
    row = module.select_rows(name)[0]
    cases = (
        ["solveNative/boxed_coupled_96", "solveNative/friction_32"]
        if name == "lcp"
        else ["BM_Dynamics/10"]
    )
    return {
        "run": {
            "commit": "HEAD",
            "env": {
                "fingerprint": "same",
                "compiler": "test",
                "compiler_provenance": "dart-perf-build/1",
            },
        },
        "results": [
            {
                "row": name,
                "det": "",
                "version": row.version,
                "status": "ok",
                "gated": True,
                "method": "slope",
                "head": {
                    "ir_per_step": 100,
                    "allocs_per_step": 0,
                    "bytes_per_step": 0,
                    "cases": cases,
                    "guards": {
                        "hash": {case: "0x0123456789abcdef" for case in cases},
                        "finite": True,
                    },
                },
            }
        ],
    }


def test_compare_rejects_mixed_measurement_methods(tmp_path, capsys):
    module = _load_runner()
    slope = {"schema": "dart-perf/1", **_micro_record(module, "dyn")}
    slope["run"]["env"].update(valgrind="test", glibc="test", preset="perf-1")
    native = copy.deepcopy(slope)
    native["results"][0].update(method="native", expected_ir=False)
    native["results"][0]["head"].pop("ir_per_step")
    for base, head in ((native, slope), (slope, native)):
        record = module.compare(base, head)
        assert record["verdict"]["status"] == "ERROR"
        assert record["results"] == []
        assert "incompatible measurement method/expected_ir" in module.markdown(record)
        base_path, head_path = tmp_path / "base.json", tmp_path / "head.json"
        module.write_json(base_path, base)
        module.write_json(head_path, head)
        assert (
            module.main(["compare", "--base", str(base_path), "--head", str(head_path)])
            == 2
        )
        assert "incompatible measurement method/expected_ir" in capsys.readouterr().out
    for field, value in (("method", "native"), ("expected_ir", False)):
        head = copy.deepcopy(slope)
        head["results"][0][field] = value
        assert module.compare(slope, head)["verdict"]["status"] == "ERROR"
    # Rows that are native on both sides collect no Ir and still compare.
    assert module.compare(native, native)["verdict"]["status"] == "PASS"


@pytest.mark.parametrize("name", ["dyn", "lcp"])
def test_micro_checksums_and_allocations_gate_comparison(name):
    module = _load_runner()
    base = _micro_record(module, name)
    same = module.compare(base, copy.deepcopy(base))
    assert same["verdict"]["status"] == "PASS"
    assert same["results"][0]["delta"]["class"] == "gated"
    assert same["results"][0]["delta"]["guards_equal"] is True
    for case in base["results"][0]["head"]["cases"]:
        head = copy.deepcopy(base)
        head["results"][0]["head"]["guards"]["hash"][case] = "0xfedcba9876543210"
        record = module.compare(base, head)
        assert record["verdict"]["status"] == "FAIL"
        assert record["results"][0]["delta"]["class"] == "behaviour-change"
        assert record["results"][0]["delta"]["guards_equal"] is False
    head = copy.deepcopy(base)
    head["results"][0]["head"]["allocs_per_step"] = 1
    record = module.compare(base, head)
    assert record["verdict"]["status"] == "FAIL"
    assert "allocations +1/step" in record["verdict"]["failures"][0]


def test_requested_bytes_gate_report_and_missing_measurements():
    module = _load_runner()
    base = _micro_record(module, "dyn")
    base["run"]["env"].update(valgrind="test", glibc="test", preset="perf-1")
    base["results"][0]["head"]["bytes_per_step"] = 100
    head = copy.deepcopy(base)
    head["results"][0]["head"]["bytes_per_step"] = 128
    result = module.compare(base, head)
    assert result["verdict"]["status"] == "FAIL"
    assert result["results"][0]["delta"]["allocs"] == 0
    assert result["results"][0]["delta"]["bytes"] == 28
    assert result["verdict"]["failures"] == ["dyn: requested bytes +28/step"]
    report = module.markdown(result)
    assert "Bytes/step delta" in report and "| +28 |" in report
    assert "1 regressed." in report
    for row, status in (("dyn", "PASS"), ("lcp", "FAIL")):
        result = module.compare(
            base, head, f"Perf-Regression-Rationale: {row}: intended"
        )
        assert result["verdict"]["status"] == status
        assert "1 regressed." in module.markdown(result)
    head["results"][0]["head"]["bytes_per_step"] = 72
    result = module.compare(base, head)
    assert result["verdict"]["status"] == "PASS"
    assert result["results"][0]["delta"]["bytes"] == -28
    assert "| -28 |" in module.markdown(result)
    head["results"][0]["head"]["bytes_per_step"] = 128
    base["results"][0]["gated"] = False
    assert module.compare(base, head)["verdict"]["status"] == "PASS"
    base["results"][0]["gated"] = True

    # Missing/invalid bytes follow the same policy as allocation counts on either arm.
    for field in ("allocs_per_step", "bytes_per_step"):
        for invalid in ("missing", None, -1, float("nan"), float("inf")):
            for arms in (("base",), ("head",), ("base", "head")):
                records = {"base": copy.deepcopy(base), "head": copy.deepcopy(head)}
                for arm in arms:
                    row = records[arm]["results"][0]
                    if invalid == "missing":
                        row["head"].pop(field)
                    else:
                        row["head"][field] = invalid
                    assert not module.complete(row, row["head"])
                result = module.compare(
                    records["base"],
                    records["head"],
                    "Perf-Regression-Rationale: dyn: intended",
                )
                assert result["verdict"]["status"] == "FAIL"
                assert result["results"][0]["delta"]["class"] == "broken"
                assert result["results"][0]["delta"]["bytes"] is None
                assert "Bytes/step delta" in module.markdown(result)


@pytest.mark.parametrize("name", ["dyn", "lcp"])
def test_changed_workload_needs_rationale_without_micro_instrumentation(name):
    module = _load_runner()
    base = _micro_record(module, name)
    head = copy.deepcopy(base)
    base["results"][0]["head"].update(
        micro_instrumented=False,
        guards=None,
        allocs=None,
        bytes=None,
        allocs_per_step=None,
        bytes_per_step=None,
    )
    head["results"][0]["input_sha"] = "changed workload"
    result = module.compare(base, head)
    row = result["results"][0]
    assert result["verdict"]["status"] == "FAIL"
    assert row["delta"]["class"] == "behaviour-change"
    assert row["failures"] == ["input_sha changed; Rebaseline-Rationale required"]
    acknowledged = module.compare(
        base, head, f"Rebaseline-Rationale: {name}: new workload"
    )
    assert acknowledged["verdict"]["status"] == "PASS"
    assert acknowledged["results"][0]["gated"] is False


@pytest.mark.parametrize("name", ["dyn", "lcp"])
@pytest.mark.parametrize("missing", ["base", "head", "both"])
def test_micro_instrumentation_availability(name, missing):
    module = _load_runner()
    base = _micro_record(module, name)
    head = copy.deepcopy(base)
    head["results"][0]["head"]["ir_per_step"] = 200
    for record in (
        [base, head] if missing == "both" else [base if missing == "base" else head]
    ):
        metrics = record["results"][0]["head"]
        metrics.update(
            micro_instrumented=False,
            guards=None,
            allocs=None,
            bytes=None,
            allocs_per_step=None,
            bytes_per_step=None,
        )
        assert module.complete(record["results"][0], metrics)
    result = module.compare(base, head, f"Rebaseline-Rationale: {name}: +100% intended")
    row = result["results"][0]
    assert row["delta"]["ir"] == 1
    assert row["delta"]["allocs"] is None
    assert row["delta"]["guards_equal"] is None
    assert result["verdict"]["ir_geomean"] is None
    if missing == "head":
        assert result["verdict"]["status"] == "FAIL"
        assert row["delta"]["class"] == "broken"
        assert row["failures"] == ["head lacks micro instrumentation present in base"]
        head["results"][0]["version"] += 1
        assert module.compare(base, head)["verdict"]["status"] == "FAIL"
    else:
        assert result["verdict"]["status"] == "PASS"
        assert row["delta"]["class"] == "diagnostic"
        assert row["gated"] is False
    for arm in (["base", "head"] if missing == "both" else [missing]):
        assert f"{arm} lacks micro instrumentation" in row["gate_reason"]
    result["run"]["env"].update(
        valgrind="test", compiler="test", glibc="test", preset="perf-1"
    )
    report = module.markdown(result)
    assert "unavailable" in report
    assert row["gate_reason"] in report


@pytest.mark.parametrize("name", ["dyn", "lcp"])
@pytest.mark.parametrize("arm", ["base", "head", "both"])
@pytest.mark.parametrize(
    "defect",
    [
        "missing allocation",
        "null allocation",
        "negative allocation",
        "missing guards",
        "null guards",
        "missing checksum",
        "partial checksum",
        "non-finite",
    ],
)
def test_micro_incomplete_metrics_never_pass(name, arm, defect):
    module = _load_runner()
    base = _micro_record(module, name)
    head = copy.deepcopy(base)
    for record in (
        [base, head] if arm == "both" else [base if arm == "base" else head]
    ):
        row = record["results"][0]
        metrics = row["head"]
        if defect == "missing allocation":
            metrics.pop("allocs_per_step")
        elif defect == "null allocation":
            metrics["allocs_per_step"] = None
        elif defect == "negative allocation":
            metrics["allocs_per_step"] = -1
        elif defect == "missing guards":
            metrics.pop("guards")
        elif defect == "null guards":
            metrics["guards"] = None
        elif defect == "missing checksum":
            metrics["guards"].pop("hash")
        elif defect == "partial checksum":
            metrics["guards"]["hash"].pop(metrics["cases"][0])
        else:
            metrics["guards"]["finite"] = False
        assert not module.complete(row, metrics)
    result = module.compare(base, head, f"Rebaseline-Rationale: {name}: intended")
    assert result["verdict"]["status"] == "FAIL"
    assert result["results"][0]["delta"]["class"] == "broken"


@pytest.mark.parametrize("name", ["dyn", "lcp"])
def test_micro_native_slope_and_explicit_windows(monkeypatch, tmp_path, name):
    module = _load_runner()
    row = module.select_rows(name)[0]
    args = module.parser().parse_args(
        [
            "run",
            "--commit",
            "HEAD",
            "--prefix",
            str(tmp_path),
            "--output-dir",
            str(tmp_path),
            "--native-only",
        ]
    )
    args.bin_dir = tmp_path
    cases = _micro_record(module, name)["results"][0]["head"]["cases"]
    defect = ""

    def execute(command, env, log, timeout):
        assert env["PERF_WINDOW"] == "micro"
        assert env["PERF_MICRO"] == "1"
        assert env["PERF_WARMUP"] == "0"
        steps = int(
            next(
                item for item in command if item.startswith("--benchmark_min_time=")
            ).split("=")[1][:-1]
        )
        output = Path(
            next(
                item.split("=", 1)[1]
                for item in command
                if item.startswith("--benchmark_out=")
            )
        )
        module.write_json(
            output,
            {"benchmarks": [{"name": case, "iterations": steps} for case in cases]},
        )
        measured = (
            len(cases) if defect not in ("missing window", "uninstrumented") else 0
        )
        allocs, size = (
            (0, 0) if defect == "uninstrumented" else (50 + 3 * steps, 200 + 12 * steps)
        )
        text = f"STEPALLOC steps={measured} measured={measured} allocs={allocs} bytes={size} libdart={tmp_path}/libdart.so\n"
        if defect not in ("missing checksum", "uninstrumented"):
            text += "".join(
                f"PERFGUARD case={case} hash=0x0123456789abcdef finite=true\n"
                for case in cases
            )
        if defect == "duplicate checksum":
            text += f"PERFGUARD case={cases[0]} hash=0x0123456789abcdef finite=true\n"
        log.write_text(text)
        return text

    monkeypatch.setattr(module, "execute", execute)
    metrics = module.micro_perturb(row, args, tmp_path, "")
    assert metrics["allocs"] == 3 * row.steps
    assert metrics["bytes"] == 12 * row.steps
    assert metrics["allocs_per_step"] == 3
    assert metrics["bytes_per_step"] == 12
    assert metrics["cases"] == cases
    assert metrics["guards"] == module.native_guards(args, row)
    defect = "uninstrumented"
    result = module.measure(row, args, tmp_path)
    assert result["status"] == "ok"
    metrics = result["head"]
    assert metrics["micro_instrumented"] is False
    assert metrics["cases"] == cases
    assert all(
        metrics[key] is None
        for key in ("guards", "allocs", "bytes", "allocs_per_step", "bytes_per_step")
    )
    assert module.native_guards(args, row) is None
    assert module.complete(result, metrics)
    for defect in ("missing window", "missing checksum", "duplicate checksum"):
        with pytest.raises(ValueError):
            module.micro_perturb(row, args, tmp_path, "")


@pytest.mark.parametrize("name", ["dyn", "lcp"])
@pytest.mark.parametrize("changed", ["hash", "allocs"])
def test_micro_perturbations_compare_real_metrics(monkeypatch, tmp_path, name, changed):
    module = _load_runner()
    args = module.parser().parse_args(
        [
            "run",
            "--commit",
            "HEAD",
            "--prefix",
            str(tmp_path),
            "--output-dir",
            str(tmp_path),
            "--native-only",
            "--perturb",
        ]
    )
    baseline = _micro_record(module, name)["results"][0]["head"]
    baseline["allocs"] = 0

    def measure(row, args, world, config):
        metrics = copy.deepcopy(baseline)
        if config:
            if changed == "hash":
                metrics["guards"]["hash"][metrics["cases"][0]] = "0xfedcba9876543210"
            else:
                metrics["allocs"] = 1
        return metrics

    monkeypatch.setattr(module, "micro_perturb", measure)
    result = module.measure(module.select_rows(name)[0], args, tmp_path)
    assert result["status"] == "ok"
    assert result["gated"] is False
    assert all(not sample["stable"] for sample in result["perturbations"].values())


@pytest.mark.parametrize("revision", ["HEAD", "annotated-release"])
def test_local_uses_independent_source_and_cmake_caches(
    monkeypatch, tmp_path, revision
):
    import io
    import tarfile
    from types import SimpleNamespace

    module = _load_runner()
    args = module.parser().parse_args(
        ["local", "--base", revision, "--head", revision, "--output-dir", str(tmp_path)]
    )
    monkeypatch.setenv("CONDA_PREFIX", str(tmp_path / "dependencies"))

    def command_output(command):
        # An annotated tag names a tag object unless explicitly peeled.
        return "tag-object" if command[-1] == "annotated-release" else "commit"

    monkeypatch.setattr(module, "command_output", command_output)
    monkeypatch.setattr(
        module.subprocess, "run", lambda *args, **kwargs: SimpleNamespace(returncode=0)
    )
    archive = io.BytesIO()
    with tarfile.open(fileobj=archive, mode="w") as contents:
        entry = tarfile.TarInfo("CMakeLists.txt")
        entry.size = 0
        contents.addfile(entry, io.BytesIO())
        for name in sorted(
            {path for paths in module.WORKLOAD_SOURCES.values() for path in paths}
        ):
            data = f"archived {name}".encode()
            entry = tarfile.TarInfo(name)
            entry.size = len(data)
            contents.addfile(entry, io.BytesIO(data))
    monkeypatch.setattr(
        module.subprocess, "check_output", lambda *args, **kwargs: archive.getvalue()
    )
    configurations = []

    def execute(command, env, log, timeout):
        if command[:3] == ["cmake", "-G", "Ninja"]:
            source = Path(command[command.index("-S") + 1])
            build = Path(command[command.index("-B") + 1])
            build.mkdir()
            compiler = build / "CMakeFiles/4.0/CMakeCXXCompiler.cmake"
            compiler.parent.mkdir(parents=True)
            compiler.write_text(
                'set(CMAKE_CXX_COMPILER_ID "GNU")\nset(CMAKE_CXX_COMPILER_VERSION "13.3.0")\n'
            )
            if source.name.startswith("src-"):
                assert (source / "CMakeLists.txt").is_file()
                assert not (build / "CMakeCache.txt").exists()
                (build / "CMakeCache.txt").write_text(source.name)
                configurations.append((source, build))
        elif command[:2] == ["cmake", "--build"]:
            build = Path(command[2])
            if build.name.startswith("driver-"):
                (build / "portable_step_bench").write_text("driver")
            else:
                (build / "bin").mkdir()
                for driver in module.WORKLOAD_SOURCES:
                    (build / "bin" / driver).write_bytes(b"archived driver")
        elif command[:2] == ["cmake", "--install"]:
            prefix = Path(command[command.index("--prefix") + 1])
            (prefix / "share/dart").mkdir(parents=True)
            (prefix / "lib").mkdir()
            (prefix / "lib/libdart.so").write_bytes(b"installed DART")
        return ""

    monkeypatch.setattr(module, "execute", execute)

    def run_arm(arm):
        assert arm.commit == "commit"
        stamp = module.installed_provenance(arm)
        assert stamp["compiler"] == "GNU 13.3.0"
        assert stamp["pixi_lock_sha"] == module.sha(
            (module.ROOT / "pixi.lock").read_bytes()
        )
        source = tmp_path / f"src-{arm.prefix.name}"
        assert stamp["workload_sources"] == module.workload_hashes(
            source, module.WORKLOAD_SOURCES
        )
        assert stamp["workload_sources"] != module.workload_hashes(
            module.ROOT, module.WORKLOAD_SOURCES
        )
        return {"commit": arm.commit}

    monkeypatch.setattr(module, "run_arm", run_arm)
    assert len(module.local_arms(args)) == 2
    assert configurations == [
        (tmp_path / f"src-{arm}", tmp_path / f"build-{arm}") for arm in ("a", "b")
    ]
    for source, build in configurations:
        assert (build / "CMakeCache.txt").read_text() == source.name


@pytest.mark.parametrize("name", ["dyn", "lcp"])
def test_micro_callgrind_requires_native_guard_and_library_parity(
    monkeypatch, tmp_path, name
):
    module = _load_runner()
    row = module.select_rows(name)[0]
    args = module.parser().parse_args(
        [
            "run",
            "--commit",
            "HEAD",
            "--prefix",
            str(tmp_path),
            "--output-dir",
            str(tmp_path),
        ]
    )
    args.bin_dir = tmp_path
    cases = module.MICRO_CASES[name]
    native_text = (
        f"STEPALLOC steps=1 measured=1 allocs=0 bytes=0 libdart={tmp_path}/libdart.so\n"
    )
    native_text += "".join(
        f"PERFGUARD case={case} hash=0x0123456789abcdef finite=true\n" for case in cases
    )
    module.native_log(args, row).write_text(native_text)
    defect = ""

    def execute(command, env, log, timeout):
        assert env["PERF_MICRO"] == "1"
        assert f"--toggle-collect={module.COLLECTION_SIGNATURES[row.driver]}" in command
        output = next(
            item.split("=", 1)[1]
            for item in command
            if item.startswith("--callgrind-out-file=")
        )
        library = "other/libdart.so" if defect == "library" else "libdart.so"
        Path(output.replace("%p", "123")).write_text(
            f"events: Ir\nsummary: 1000\nob={tmp_path}/{library}\n"
        )
        return (
            native_text.replace("0x0123456789abcdef", "0xfedcba9876543210")
            if defect == "checksum"
            else native_text
        )

    monkeypatch.setattr(module, "execute", execute)
    assert module.callgrind(row, args, tmp_path, row.warmup + row.steps) == {"Ir": 1000}
    for defect in ("checksum", "library"):
        with pytest.raises(ValueError, match="differs|differ"):
            module.callgrind(row, args, tmp_path, row.warmup + row.steps)
    defect = ""
    native_text = native_text.split("PERFGUARD", 1)[0]
    module.native_log(args, row).write_text(native_text)
    assert module.callgrind(row, args, tmp_path, row.warmup + row.steps) == {"Ir": 1000}


def test_compare_rejects_comparison_reports_as_inputs(tmp_path):
    module = _load_runner()
    record = {"schema": "dart-perf/1", **_micro_record(module, "dyn")}
    path = tmp_path / "record.json"
    module.write_json(path, record)
    assert module.read_record(path)["results"]
    # A comparison report shares the schema but carries the base's
    # qualification, so it must not stand in for a measurement.
    report = module.compare(record, record)
    module.write_json(path, report)
    with pytest.raises(ValueError, match="comparison report"):
        module.read_record(path)
