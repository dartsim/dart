"""The renderer gate uses baseline noise and this process's graphics memory."""

import importlib.util
import json
import os
import subprocess
from pathlib import Path
from types import SimpleNamespace

import pytest


def load_script():
    path = Path(__file__).resolve().parents[1] / "scripts/compare_gui_renderers.py"
    spec = importlib.util.spec_from_file_location("gui_renderer_measurement", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_graphics_memory_ignores_desktop_and_other_gpu():
    script = load_script()
    output = "# gpu pid type fb ccpm command\n0 10 G 2500 0 Xorg\n1 42 G 900 0 other\n0 42 G 123 0 benchmark\n"
    assert script.graphics_memory(output, 42, 0) == 123
    assert script.graphics_memory(output, 99, 0) is None


def test_regression_is_checked_per_metric_against_measured_noise():
    script = load_script()
    baseline = [
        (dict(render_ms=10, peak_rss_kib=100), dict(render_ms=10.2, peak_rss_kib=100))
    ]
    candidate = [
        (dict(render_ms=10, peak_rss_kib=100), dict(render_ms=10.1, peak_rss_kib=101))
    ]
    result = script.compare_samples(baseline, candidate)
    assert result["render_ms"]["passed"]
    assert not result["peak_rss_kib"]["passed"]
    assert result["peak_rss_kib"]["baseline_pair_noise"] == 0


def test_faster_metric_does_not_hide_slow_metric():
    script = load_script()
    baseline = [(dict(render_ms=10, setup_ms=20), dict(render_ms=10, setup_ms=20))]
    candidate = [(dict(render_ms=10, setup_ms=20), dict(render_ms=5, setup_ms=21))]
    result = script.compare_samples(baseline, candidate)
    assert result["render_ms"]["passed"]
    assert not result["setup_ms"]["passed"]


@pytest.mark.parametrize(
    "arguments",
    [
        ["--seconds", "nan"],
        ["--seconds", "inf"],
        ["--cpus", "-1"],
        ["--gpu-index", "-1"],
    ],
)
def test_invalid_measurement_arguments_fail_before_launch(
    monkeypatch, arguments, tmp_path
):
    script = load_script()
    monkeypatch.setattr(
        "sys.argv",
        [
            "compare_gui_renderers",
            "absent-binary",
            "--reference-binary",
            "old-binary",
            "--reference-provenance",
            "old-provenance.json",
            "--output",
            str(tmp_path),
            "--expect-device",
            "RTX 5070 Ti",
            *arguments,
        ],
    )
    with pytest.raises(SystemExit) as error:
        script.main()
    assert error.value.code == 2


def test_expected_device_is_required(monkeypatch, tmp_path):
    script = load_script()
    monkeypatch.setattr(
        "sys.argv",
        [
            "compare_gui_renderers",
            "absent-binary",
            "--reference-binary",
            "old-binary",
            "--reference-provenance",
            "old-provenance.json",
            "--output",
            str(tmp_path),
        ],
    )
    with pytest.raises(SystemExit) as error:
        script.main()
    assert error.value.code == 2


def test_occupied_output_is_rejected_before_launch(monkeypatch, tmp_path):
    script = load_script()
    (tmp_path / "report.json").write_text("old evidence")
    monkeypatch.setattr(
        "sys.argv",
        [
            "compare_gui_renderers",
            "absent-binary",
            "--reference-binary",
            "old-binary",
            "--reference-provenance",
            "old-provenance.json",
            "--output",
            str(tmp_path),
            "--expect-device",
            "RTX",
        ],
    )
    with pytest.raises(SystemExit) as error:
        script.main()
    assert error.value.code == 2
    assert (tmp_path / "report.json").read_text() == "old evidence"


@pytest.mark.skipif(
    not hasattr(os, "sched_getaffinity"), reason="CPU affinity is Linux-only"
)
@pytest.mark.parametrize("cpus", [[0, 1], [0, 0], [0, 9]])
def test_affinity_rejects_smt_siblings_duplicates_and_disallowed_threads(
    monkeypatch, cpus
):
    script = load_script()
    monkeypatch.setattr(script.os, "sched_getaffinity", lambda _: {0, 1, 2})
    monkeypatch.setattr(script, "core_id", lambda cpu: (0, cpu // 2))
    with pytest.raises(ValueError):
        script.validate_cpus(cpus)
    script.validate_cpus([0, 2])
    assert script.physical_cpus() == [0, 2]


def test_gpu_steady_memory_excludes_warmup_and_crossing_samples():
    script = load_script()
    values = [
        (900, False),
        (800, False),
        (100, True),
        (110, True),
        (120, True),
        (130, True),
        (700, False),
    ]
    samples = [dict(mib=value, measured=measured) for value, measured in values]
    result = script.gpu_memory_metrics(samples)
    assert result["gpu_memory_peak_mib"] == 900
    assert result["gpu_memory_steady_mib"] == 125
    assert result["gpu_memory_samples"] == samples
    with pytest.raises(RuntimeError, match="wholly within"):
        script.gpu_memory_metrics([dict(mib=900, measured=False)])


def test_measurement_phase_requires_ordered_markers():
    script = load_script()
    begin = "DART_GUI_BENCHMARK_PHASE measured_begin\n"
    end = "DART_GUI_BENCHMARK_PHASE measured_end\n"
    assert script.measurement_phase("startup\n") == 0
    assert script.measurement_phase("startup\n" + begin) == 1
    assert script.measurement_phase(begin + "a frame\n" + end + "readback\n") == 2
    for invalid in (end, begin + begin, begin + end + begin, begin + end + end):
        with pytest.raises(RuntimeError):
            script.measurement_phase(invalid)


@pytest.mark.parametrize("device", ["RTX 5070 Ti", "llvmpipe (LLVM 23.1.2)"])
def test_hardware_run_cannot_pass_without_gpu_resource_samples(
    tmp_path, monkeypatch, device
):
    script = load_script()
    record = dict(
        renderer="osg",
        scene="boxes",
        device=device,
        width=8,
        height=8,
        shadows=False,
        frames=10,
        guard_pixels=True,
        render_ms=1,
        setup_ms=2,
        readback_full_frame_ms=3,
        peak_rss_kib=100,
    )

    class Process:
        pid = 999999999
        returncode = 0

        def __init__(self, command, stdout, stderr):
            stdout.write(
                "DART_GUI_BENCHMARK_PHASE measured_begin\n"
                "DART_GUI_BENCHMARK_PHASE measured_end\n" + json.dumps(record) + "\n"
            )
            stdout.flush()

        def poll(self):
            return 0

        def wait(self):
            return 0

    monkeypatch.setattr(script.subprocess, "Popen", Process)
    args = SimpleNamespace(
        cpus=[0],
        width=8,
        height=8,
        warmup=1,
        seconds=30,
        shadows=False,
        gpu_index=None,
        expect_device=device,
    )
    if device.startswith("RTX"):
        with pytest.raises(RuntimeError, match="require --gpu-index"):
            script.measure(tmp_path / "binary", "osg", "boxes", args, tmp_path, 0)
    else:
        assert (
            script.measure(tmp_path / "binary", "osg", "boxes", args, tmp_path, 0)[
                "device"
            ]
            == device
        )


def test_live_collector_discards_samples_crossing_measured_window(
    monkeypatch, tmp_path
):
    script = load_script()
    outputs = []
    calls = []
    record = dict(
        renderer="osg",
        scene="boxes",
        device="RTX",
        width=8,
        height=8,
        shadows=False,
        frames=10,
        guard_pixels=True,
        render_ms=1,
        setup_ms=2,
        readback_full_frame_ms=3,
        peak_rss_kib=100,
    )

    class Process:
        pid = 999999999
        returncode = 0

        def __init__(self, command, stdout, stderr):
            outputs.append(stdout)

        def poll(self):
            return None if len(calls) < 3 else 0

        def wait(self):
            return 0

    def run(command, **kwargs):
        calls.append(command)
        if len(calls) == 1:
            outputs[0].write("DART_GUI_BENCHMARK_PHASE measured_begin\n")
            value = 900
        elif len(calls) == 2:
            value = 110
        else:
            outputs[0].write(
                "DART_GUI_BENCHMARK_PHASE measured_end\n" + json.dumps(record) + "\n"
            )
            value = 800
        outputs[0].flush()
        return subprocess.CompletedProcess(
            command, 0, stdout=f"0 {Process.pid} G {value} 0 benchmark\n"
        )

    monkeypatch.setattr(script.subprocess, "Popen", Process)
    monkeypatch.setattr(script.subprocess, "run", run)
    args = SimpleNamespace(
        cpus=[0],
        width=8,
        height=8,
        warmup=1,
        seconds=30,
        shadows=False,
        gpu_index=0,
        expect_device="RTX",
    )
    result = script.measure(tmp_path / "binary", "osg", "boxes", args, tmp_path, 0)
    assert result["gpu_memory_peak_mib"] == 900
    assert result["gpu_memory_steady_mib"] == 110
    assert [sample["measured"] for sample in result["gpu_memory_samples"]] == [
        False,
        True,
        False,
    ]


def test_ldd_provenance_hashes_resolved_library_and_refuses_missing_dependencies(
    monkeypatch, tmp_path
):
    script = load_script()
    library = tmp_path / "libdart-gui.so.6"
    library.write_bytes(b"renderer")
    link = tmp_path / "libdart-gui.so"
    link.symlink_to(library)
    output = f"linux-vdso.so.1 (0x123)\nlibdart-gui.so => {link} (0x456)\n/lib64/ld-linux-x86-64.so.2 (0x789)\n"
    # Keep the loader path local so this test does not depend on the host loader.
    output = output.replace("/lib64/ld-linux-x86-64.so.2", str(library))
    monkeypatch.setattr(
        script.subprocess,
        "run",
        lambda *a, **k: subprocess.CompletedProcess(a[0], 0, stdout=output),
    )
    resolved = script.resolved_libraries(tmp_path / "benchmark")
    assert resolved[str(link)] == script.file_hash(library)
    library.write_bytes(b"changed renderer")
    assert script.resolved_libraries(tmp_path / "benchmark") != resolved
    output = "libdart-gui.so => not found\n"
    with pytest.raises(RuntimeError, match="could not be resolved"):
        script.resolved_libraries(tmp_path / "benchmark")


def test_source_snapshot_hashes_dirty_new_and_deleted_files(monkeypatch, tmp_path):
    script = load_script()
    source = tmp_path / "dart"
    source.mkdir()
    (source / "tracked.cpp").write_text("edited")
    (source / "new.cpp").write_text("new renderer")
    calls = []

    def run(command, **kwargs):
        calls.append(command)
        if command[1] == "ls-files":
            output = b"dart/tracked.cpp\0dart/new.cpp\0dart/deleted.cpp\0"
        elif command[1] == "status":
            output = " M dart/tracked.cpp\n?? dart/new.cpp\n D dart/deleted.cpp\n"
        else:
            output = "source-revision\n"
        return subprocess.CompletedProcess(command, 0, stdout=output)

    monkeypatch.setattr(script.subprocess, "run", run)
    initial = script.source_snapshot(tmp_path)
    assert initial["files"]["dart/new.cpp"] == script.file_hash(source / "new.cpp")
    assert initial["files"]["dart/deleted.cpp"] is None
    assert "--cached" in calls[0] and "--others" in calls[0]
    assert "?? dart/new.cpp" in initial["status"]
    (source / "new.cpp").write_text("changed new renderer")
    assert script.source_snapshot(tmp_path) != initial


def test_provenance_records_build_flags_and_rejects_drift(monkeypatch, tmp_path):
    script = load_script()
    build = tmp_path / "build"
    (build / "bin").mkdir(parents=True)
    binary = build / "bin/benchmark"
    binary.write_bytes(b"benchmark")
    compiler = tmp_path / "c++"
    compiler.write_bytes(b"compiler")
    (tmp_path / "pixi.lock").write_text("pinned dependencies")
    cache = build / "CMakeCache.txt"
    cache.write_text(
        f"CMAKE_HOME_DIRECTORY:INTERNAL={tmp_path}\nCMAKE_CXX_COMPILER:FILEPATH={compiler}\nCMAKE_CXX_FLAGS_RELEASE:STRING=-O3 -DNDEBUG\n"
    )
    (build / "build.ninja").write_text("actual compile and link flags")
    monkeypatch.setattr(script, "resolved_libraries", lambda _: {"SDK.so": "sdk hash"})
    monkeypatch.setattr(
        script, "source_snapshot", lambda _: {"files": {"dart/new.cpp": "source hash"}}
    )
    monkeypatch.setattr(
        script.subprocess,
        "run",
        lambda *a, **k: subprocess.CompletedProcess(
            a[0], 0, stdout="compiler version\n"
        ),
    )
    initial = script.provenance(binary)
    assert (
        initial["cmake"][str(cache)]["values"]["CMAKE_CXX_FLAGS_RELEASE"]
        == "-O3 -DNDEBUG"
    )
    assert initial["compilers"][str(compiler)]["version"] == "compiler version\n"
    assert initial["dependency_lock"]["sha256"] == script.file_hash(
        tmp_path / "pixi.lock"
    )
    script.verify_provenance(binary, initial)
    for file in (
        binary,
        compiler,
        cache,
        build / "build.ninja",
        tmp_path / "pixi.lock",
    ):
        before = file.read_bytes()
        file.write_bytes(before + b"changed")
        with pytest.raises(RuntimeError, match="provenance changed"):
            script.verify_provenance(binary, initial)
        file.write_bytes(before)
    monkeypatch.setattr(
        script, "resolved_libraries", lambda _: {"SDK.so": "changed sdk"}
    )
    with pytest.raises(RuntimeError, match="provenance changed"):
        script.verify_provenance(binary, initial)


def test_preserved_reference_uses_frozen_source_and_rejects_artifact_drift(
    monkeypatch, tmp_path
):
    script = load_script()
    binary = tmp_path / "benchmark"
    binary.write_bytes(b"old OSG binary")
    library = tmp_path / "lib" / "libdart.so.6"
    library.parent.mkdir()
    library.write_bytes(b"old core")
    cache = tmp_path / "CMakeCache.txt"
    cache.write_text("CMAKE_HOME_DIRECTORY:INTERNAL=/now/changed/checkout\n")
    source = tmp_path / "source/tests/benchmark/integration/gui_render_benchmark.cpp"
    source.parent.mkdir(parents=True)
    source.write_text("old OSG scene protocol")
    snapshot = {
        "revision": "old-revision",
        "status": "dirty old source",
        "files": {
            "tests/benchmark/integration/gui_render_benchmark.cpp": script.file_hash(
                source
            )
        },
    }
    freeze = tmp_path / "freeze.json"
    freeze.write_text(json.dumps({"benchmark_source_snapshot": snapshot}))
    manifest = tmp_path / "manifest.json"
    manifest.write_text(
        json.dumps(
            {
                "binary_sha256": script.file_hash(binary),
                "revision": "old-revision",
                "libraries": {library.name: {"sha256": script.file_hash(library)}},
                "source_freeze": {
                    "path": freeze.name,
                    "sha256": script.file_hash(freeze),
                },
                "build_files": {
                    str(file.relative_to(tmp_path)): script.file_hash(file)
                    for file in (cache, source)
                },
            }
        )
    )
    monkeypatch.setenv("LD_LIBRARY_PATH", "/candidate/libs")
    environment = script.reference_environment(library.parent)
    assert environment["LD_LIBRARY_PATH"] == f"{library.parent}:/candidate/libs"
    assert script.os.environ["LD_LIBRARY_PATH"] == "/candidate/libs"
    environments = []

    def libraries(path, env):
        assert path == binary
        environments.append(env)
        return {str(library): script.file_hash(library)}

    monkeypatch.setattr(script, "resolved_libraries", libraries)
    monkeypatch.setattr(
        script,
        "source_snapshot",
        lambda _: pytest.fail("Old source must never be recaptured from live checkout"),
    )
    initial = script.reference_provenance(binary, manifest, environment)
    assert initial["source"] == snapshot
    assert environments == [environment]
    script.verify_reference_provenance(binary, manifest, environment, initial)
    for file in (binary, library, cache, source, freeze, manifest):
        before = file.read_bytes()
        file.write_bytes(before + b" ")
        with pytest.raises(RuntimeError):
            script.verify_reference_provenance(binary, manifest, environment, initial)
        file.write_bytes(before)
    # Updating the copied-source hash cannot replace the original frozen source.
    source.write_text("different scene protocol")
    changed = json.loads(manifest.read_text())
    changed["build_files"][str(source.relative_to(tmp_path))] = script.file_hash(source)
    manifest.write_text(json.dumps(changed))
    with pytest.raises(RuntimeError, match="source does not match its freeze"):
        script.reference_provenance(binary, manifest, environment)


def test_comparison_selects_separate_binaries_and_guards_both_provenances(
    monkeypatch, tmp_path
):
    script = load_script()
    candidate = tmp_path / "filament"
    reference = tmp_path / "osg"
    manifest = tmp_path / "manifest.json"
    library_directory = tmp_path / "old-libraries"
    library_directory.mkdir()
    for file in (candidate, reference, manifest):
        file.write_text("fixture")
    directory = tmp_path / "results"
    monkeypatch.setattr(
        "sys.argv",
        [
            "compare_gui_renderers",
            str(candidate),
            "--reference-binary",
            str(reference),
            "--reference-provenance",
            str(manifest),
            "--reference-library-dir",
            str(library_directory),
            "--output",
            str(directory),
            "--expect-device",
            "test GPU",
        ],
    )
    monkeypatch.setattr(script.platform, "system", lambda: "Linux")
    monkeypatch.setattr(script, "physical_cpus", lambda: [0])
    monkeypatch.setattr(script, "validate_cpus", lambda _: None)
    monkeypatch.setattr(script.shutil, "which", lambda _: None)
    candidate_proof = {"source": {"revision": "new"}, "binary": str(candidate)}
    reference_proof = {"source": {"revision": "old"}, "binary": str(reference)}
    monkeypatch.setattr(script, "provenance", lambda path: candidate_proof)
    monkeypatch.setattr(script, "reference_provenance", lambda *args: reference_proof)
    checked = []

    def verify_candidate(path, expected):
        assert path == candidate and expected == candidate_proof
        checked.append("filament")

    def verify_reference(path, manifest_path, env, expected):
        assert path == reference and manifest_path == manifest
        assert expected == reference_proof
        checked.append("osg")

    monkeypatch.setattr(script, "verify_provenance", verify_candidate)
    monkeypatch.setattr(script, "verify_reference_provenance", verify_reference)
    selected = []

    def measure(path, renderer, scene, args, output, index, environment):
        assert path == {"osg": reference, "filament": candidate}[renderer]
        if renderer == "osg":
            assert environment["LD_LIBRARY_PATH"].split(":")[0] == str(
                library_directory
            )
        else:
            assert environment is None
        selected.append(renderer)
        return {
            "state_hash": "same scene",
            "world_time": 0,
            "render_ms": 1,
            "setup_ms": 2,
            "readback_full_frame_ms": 3,
            "peak_rss_kib": 100,
        }

    monkeypatch.setattr(script, "measure", measure)
    script.main()
    report = json.loads((directory / "report.json").read_text())
    assert report["complete"]
    assert report["provenance"] == {
        "filament": candidate_proof,
        "osg": reference_proof,
    }
    assert "do not establish byte-identical core" in report["comparison_scope"]
    assert selected.count("osg") == 21 and selected.count("filament") == 7
    assert checked[::2] == ["filament"] * (len(checked) // 2)
    assert checked[1::2] == ["osg"] * (len(checked) // 2)
