from __future__ import annotations

import importlib.util
import json
import os
import sys
from pathlib import Path

import pytest

_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_ROOT / "scripts"))
_SPEC = importlib.util.spec_from_file_location(
    "run_figure13_benchmark_under_test",
    _ROOT / "scripts" / "run_figure13_benchmark.py",
)
assert _SPEC is not None
assert _SPEC.loader is not None
runner = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(runner)


def _build_configuration(**overrides: str) -> dict[str, object]:
    values = {key: "<UNDEFINED>" for key in runner.BUILD_CONFIGURATION_KEYS}
    for definition in runner.EVIDENCE_CMAKE_DEFINITIONS:
        name, value = definition.split("=", maxsplit=1)
        if name in values:
            values[name] = value
    values.update(
        {
            "CMAKE_CXX_COMPILER": "/usr/bin/c++",
            "CMAKE_CXX_COMPILER_ID": "GNU",
            "CMAKE_CXX_COMPILER_VERSION": "15",
            "CMAKE_GENERATOR": "Ninja",
            "CMAKE_SYSTEM_NAME": "Linux",
            "CMAKE_SYSTEM_PROCESSOR": "x86_64",
        }
    )
    values.update(overrides)
    record = runner._configuration_record(values)
    return {
        "algorithm": runner.BUILD_CONFIGURATION_ALGORITHM,
        "digest": runner.hashlib.sha256(record.encode("utf-8")).hexdigest(),
        "values": values,
    }


def _write_elf(path: Path, payload: bytes) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b"\x7fELF" + payload)
    return path.resolve()


def _runtime_image_fixture(
    tmp_path: Path, *, binary: Path, library: Path
) -> dict[str, object]:
    paths = [
        binary.resolve(),
        library.resolve(),
        _write_elf(tmp_path / "ld-linux-x86-64.so.2", b"loader"),
        _write_elf(tmp_path / "libbenchmark.so.1", b"benchmark library"),
        _write_elf(tmp_path / "libc.so.6", b"libc"),
        _write_elf(tmp_path / "libm.so.6", b"libm"),
        _write_elf(tmp_path / "libstdc++.so.6", b"libstdc++"),
    ]
    images = [
        {
            "file": path.name,
            "path": str(path),
            "sha256": runner._sha256(path),
            "size_bytes": path.stat().st_size,
        }
        for path in sorted(paths, key=str)
    ]
    payload = {
        "images": images,
        "required_roles": runner._runtime_image_roles(images),
    }
    return {
        "algorithm": runner.RUNTIME_IMAGE_INVENTORY_ALGORITHM,
        **payload,
        "digest": runner._canonical_digest(payload),
    }


def test_build_requires_runner_configured_evidence_tree(tmp_path: Path) -> None:
    with pytest.raises(
        runner.Figure13BenchmarkError,
        match="configured by this runner",
    ):
        runner._build_target(tmp_path / "fresh-release-tree")


def test_configure_uses_fresh_canonical_evidence_tree(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    repo = tmp_path / "repo"
    repo.mkdir()
    build_dir = repo / "build" / "evidence" / "Release"
    commands: list[list[str]] = []

    def fake_run(command, **_kwargs):
        commands.append(list(command))
        build_dir.mkdir(parents=True)
        configuration = _build_configuration()
        cache_values = {
            definition.split("=", maxsplit=1)[0]: definition.split("=", maxsplit=1)[1]
            for definition in runner.EVIDENCE_CMAKE_DEFINITIONS
        }
        cache_values["CMAKE_GENERATOR"] = "Ninja"
        (build_dir / "CMakeCache.txt").write_text(
            "".join(
                f"{name}:{'BOOL' if name == 'DART_FIGURE13_EVIDENCE_BUILD' else 'STRING'}={value}\n"
                for name, value in cache_values.items()
            ),
            encoding="utf-8",
        )
        manifest = build_dir / "generated" / "dart" / "capture_build_configuration.txt"
        manifest.parent.mkdir(parents=True)
        manifest.write_text(
            runner._configuration_record(configuration["values"]), encoding="utf-8"
        )

    monkeypatch.setattr(runner, "REPO_ROOT", repo)
    monkeypatch.setenv("CONDA_PREFIX", str(tmp_path / "prefix"))
    monkeypatch.setenv("CXXFLAGS", "-O0 -fsanitize=address -march=native")
    monkeypatch.setattr(runner.subprocess, "run", fake_run)

    configuration = runner._configure_evidence_build(build_dir)

    assert len(commands) == 1
    command = commands[0]
    assert "-G" in command and "Ninja" in command
    assert "-DCMAKE_BUILD_TYPE=Release" in command
    assert "-DCMAKE_CXX_FLAGS=" in command
    assert "-DCMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF" in command
    assert "-DDART_ENABLE_ASAN=OFF" in command
    assert "-DDART_ENABLE_SIMD=OFF" in command
    assert "-DDART_BUILD_PROFILE=OFF" in command
    assert all("march=native" not in argument for argument in command)
    assert configuration == _build_configuration()

    with pytest.raises(
        runner.Figure13BenchmarkError,
        match="fresh Figure 13 evidence build directory already exists",
    ):
        runner._configure_evidence_build(build_dir)


def test_runtime_inventory_rejects_path_replaced_after_process_mapping(
    tmp_path: Path,
) -> None:
    build_dir = tmp_path / "build"
    library = build_dir / "lib" / "libdart-simulation.so"
    library.parent.mkdir(parents=True)
    library.write_bytes(b"\x7fELFreplacement")
    stat = library.stat()
    wrong_mapping = {
        library.resolve(): (
            os.major(stat.st_dev),
            os.minor(stat.st_dev),
            stat.st_ino + 1,
        )
    }

    with pytest.raises(
        runner.Figure13BenchmarkError,
        match="no longer identifies the running binary image",
    ):
        runner._runtime_binary_image_records(wrong_mapping)


def test_loader_injection_environment_is_rejected(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("LD_PRELOAD", "/tmp/fake-clock.so")

    with pytest.raises(
        runner.Figure13BenchmarkError,
        match="forbids loader-control environment variables.*LD_PRELOAD",
    ):
        runner._reject_loader_injection_environment()


def test_runtime_inventory_requires_google_benchmark_role() -> None:
    images = [
        {"file": "ld-linux-x86-64.so.2", "path": "/lib/ld-linux-x86-64.so.2"},
        {"file": "libc.so.6", "path": "/lib/libc.so.6"},
        {"file": "libm.so.6", "path": "/lib/libm.so.6"},
        {"file": "libstdc++.so.6", "path": "/lib/libstdc++.so.6"},
    ]

    with pytest.raises(
        runner.Figure13BenchmarkError,
        match="missing required runtime roles.*google_benchmark",
    ):
        runner._runtime_image_roles(images)


def test_runtime_inventory_rejects_ambiguous_google_benchmark_role() -> None:
    images = [
        {"file": "ld-linux-x86-64.so.2", "path": "/lib/ld-linux-x86-64.so.2"},
        {"file": "libbenchmark.so.1", "path": "/one/libbenchmark.so.1"},
        {"file": "libbenchmark.so.2", "path": "/two/libbenchmark.so.2"},
        {"file": "libc.so.6", "path": "/lib/libc.so.6"},
        {"file": "libm.so.6", "path": "/lib/libm.so.6"},
        {"file": "libstdc++.so.6", "path": "/lib/libstdc++.so.6"},
    ]

    with pytest.raises(
        runner.Figure13BenchmarkError,
        match="ambiguous required runtime roles.*google_benchmark",
    ):
        runner._runtime_image_roles(images)


def test_runtime_inventory_rejects_substituted_mapped_google_benchmark(
    tmp_path: Path,
) -> None:
    library = _write_elf(tmp_path / "libbenchmark.so.1", b"original")
    mapped = library.stat()
    replacement = _write_elf(tmp_path / "replacement.so", b"replacement")
    replacement.replace(library)

    with pytest.raises(
        runner.Figure13BenchmarkError,
        match="no longer identifies the running binary image",
    ):
        runner._runtime_binary_image_records(
            {
                library: (
                    os.major(mapped.st_dev),
                    os.minor(mapped.st_dev),
                    mapped.st_ino,
                )
            }
        )


def test_build_configuration_rejects_optimized_flag_drift(tmp_path: Path) -> None:
    build_dir = tmp_path / "build"
    build_dir.mkdir()
    configuration = _build_configuration(CMAKE_CXX_FLAGS_RELEASE="-O1 -DNDEBUG")
    cache_values = {
        definition.split("=", maxsplit=1)[0]: definition.split("=", maxsplit=1)[1]
        for definition in runner.EVIDENCE_CMAKE_DEFINITIONS
    }
    cache_values["CMAKE_CXX_FLAGS_RELEASE"] = "-O1 -DNDEBUG"
    cache_values["CMAKE_GENERATOR"] = "Ninja"
    (build_dir / "CMakeCache.txt").write_text(
        "".join(
            f"{name}:{'BOOL' if name == 'DART_FIGURE13_EVIDENCE_BUILD' else 'STRING'}={value}\n"
            for name, value in cache_values.items()
        ),
        encoding="utf-8",
    )
    manifest = build_dir / "generated" / "dart" / "capture_build_configuration.txt"
    manifest.parent.mkdir(parents=True)
    manifest.write_text(
        runner._configuration_record(configuration["values"]), encoding="utf-8"
    )

    with pytest.raises(
        runner.Figure13BenchmarkError,
        match="CMAKE_CXX_FLAGS_RELEASE must be '-O3 -DNDEBUG'",
    ):
        runner._load_build_configuration(build_dir)


def test_evidence_rejects_loaded_library_with_stale_compiled_source(
    tmp_path: Path,
) -> None:
    binary = _write_elf(tmp_path / runner.TARGET, b"benchmark")
    raw_output = tmp_path / "raw.json"
    context = {
        "dart_benchmark_executable_path": str(binary.resolve()),
        "dart_benchmark_source_sha256": "1" * 64,
        "dart_build_configuration_digest": _build_configuration()["digest"],
        "dart_capture_source_git_head": "2" * 40,
        "dart_capture_source_provenance_digest": "3" * 64,
        "dart_cmake_build_type": "Release",
        "dart_compiler_id": "GNU",
        "dart_compiler_version": "15",
        "dart_ndebug": "1",
        "dart_optimization_enabled": "1",
        "date": "2026-08-31T12:00:00Z",
    }
    raw_output.write_text(json.dumps({"context": context}), encoding="utf-8")
    library = _write_elf(tmp_path / "libdart-simulation.so", b"library")
    runtime_inventory = _runtime_image_fixture(tmp_path, binary=binary, library=library)
    stale_identity = {
        "algorithm": runner.DART_LIBRARY_BUILD_IDENTITY_ALGORITHM,
        "build_configuration_digest": _build_configuration()["digest"],
        "build_target": "dart-simulation",
        "cmake_build_type": "Release",
        "compiler_id": "GNU",
        "compiler_version": "15",
        "digest": "4" * 64,
        "ndebug": True,
        "optimization_enabled": True,
        "source_git_head": "2" * 40,
        "source_provenance_digest": "5" * 64,
    }
    host = {"host_token": "6" * 64}

    with pytest.raises(
        runner.Figure13BenchmarkError,
        match="does not match the running benchmark",
    ):
        runner._write_evidence(
            raw_output,
            tmp_path / "final.json",
            binary=binary.resolve(),
            binary_sha256=runner._sha256(binary),
            host_identity=host,
            quiet_host={},
            watchdog={},
            loader_environment={
                "algorithm": runner.LOADER_POLICY_ALGORITHM,
                "forbidden_environment_prefixes": list(
                    runner.LOADER_ENVIRONMENT_PREFIXES
                ),
                "passed": True,
                "present_environment_variables": [],
            },
            runtime_image_inventory=runtime_inventory,
            loaded_dart_libraries=[
                {
                    "build_identity": stale_identity,
                    "file": library.name,
                    "path": str(library),
                    "sha256": runner._sha256(library),
                    "size_bytes": library.stat().st_size,
                }
            ],
            build_configuration=_build_configuration(),
            run_token="123e4567-e89b-42d3-a456-426614174000",
        )


def _write_valid_evidence_fixture(
    tmp_path: Path,
    *,
    context_configuration_digest: str | None = None,
    library_configuration_digest: str | None = None,
    extra_unbound_dart_image: bool = False,
) -> dict[str, object]:
    configuration = _build_configuration()
    configuration_digest = str(configuration["digest"])
    binary = _write_elf(tmp_path / runner.TARGET, b"benchmark executable bytes")
    raw_output = tmp_path / "raw-valid.json"
    context = {
        "dart_benchmark_executable_path": str(binary.resolve()),
        "dart_benchmark_source_sha256": "1" * 64,
        "dart_build_configuration_digest": (
            context_configuration_digest
            if context_configuration_digest is not None
            else configuration_digest
        ),
        "dart_capture_source_git_head": "2" * 40,
        "dart_capture_source_provenance_digest": "3" * 64,
        "dart_cmake_build_type": "Release",
        "dart_compiler_id": "GNU",
        "dart_compiler_version": "15",
        "dart_ndebug": "1",
        "dart_optimization_enabled": "1",
        "date": "2026-08-31T12:00:00Z",
    }
    raw_output.write_text(json.dumps({"context": context}), encoding="utf-8")
    library = _write_elf(tmp_path / "libdart-simulation.so", b"library bytes")
    runtime_inventory = _runtime_image_fixture(tmp_path, binary=binary, library=library)
    if extra_unbound_dart_image:
        extra_library = _write_elf(
            tmp_path / "libdart-collision.so", b"unbound DART image"
        )
        runtime_inventory["images"].append(
            {
                "file": extra_library.name,
                "path": str(extra_library),
                "sha256": runner._sha256(extra_library),
                "size_bytes": extra_library.stat().st_size,
            }
        )
        runtime_inventory["images"].sort(key=lambda image: image["path"])
        inventory_payload = {
            "images": runtime_inventory["images"],
            "required_roles": runtime_inventory["required_roles"],
        }
        runtime_inventory["digest"] = runner._canonical_digest(inventory_payload)
    identity = {
        "algorithm": runner.DART_LIBRARY_BUILD_IDENTITY_ALGORITHM,
        "build_configuration_digest": (
            library_configuration_digest
            if library_configuration_digest is not None
            else configuration_digest
        ),
        "build_target": "dart-simulation",
        "cmake_build_type": "Release",
        "compiler_id": "GNU",
        "compiler_version": "15",
        "digest": "4" * 64,
        "ndebug": True,
        "optimization_enabled": True,
        "source_git_head": "2" * 40,
        "source_provenance_digest": "3" * 64,
    }
    final_output = tmp_path / "final-valid.json"
    runner._write_evidence(
        raw_output,
        final_output,
        binary=binary.resolve(),
        binary_sha256=runner._sha256(binary),
        host_identity={"host_token": "6" * 64},
        quiet_host={"passed": True},
        watchdog={"passed": True},
        loader_environment={
            "algorithm": runner.LOADER_POLICY_ALGORITHM,
            "forbidden_environment_prefixes": list(runner.LOADER_ENVIRONMENT_PREFIXES),
            "passed": True,
            "present_environment_variables": [],
        },
        runtime_image_inventory=runtime_inventory,
        loaded_dart_libraries=[
            {
                "build_identity": identity,
                "file": library.name,
                "path": str(library),
                "sha256": runner._sha256(library),
                "size_bytes": library.stat().st_size,
            }
        ],
        build_configuration=configuration,
        run_token="123e4567-e89b-42d3-a456-426614174000",
    )
    return json.loads(final_output.read_text(encoding="utf-8"))


def test_evidence_persists_exact_executable_and_build_configuration_digests(
    tmp_path: Path,
) -> None:
    data = _write_valid_evidence_fixture(tmp_path)
    build_identity = data["dart_evidence_run"]["build_identity"]

    assert build_identity["executable_sha256"] == runner._sha256(
        tmp_path / runner.TARGET
    )
    assert (
        build_identity["build_configuration"]["digest"]
        == data["context"]["dart_build_configuration_digest"]
    )
    assert (
        build_identity["loaded_dart_libraries"][0]["build_identity"][
            "build_configuration_digest"
        ]
        == data["context"]["dart_build_configuration_digest"]
    )
    assert build_identity["runtime_image_inventory"]["required_roles"][
        "google_benchmark"
    ] == str((tmp_path / "libbenchmark.so.1").resolve())


def test_evidence_rejects_runtime_inventory_without_executable(
    tmp_path: Path,
) -> None:
    data = _write_valid_evidence_fixture(tmp_path)
    inventory = data["dart_evidence_run"]["build_identity"]["runtime_image_inventory"]
    images = [image for image in inventory["images"] if image["file"] != runner.TARGET]
    payload = {
        "images": images,
        "required_roles": runner._runtime_image_roles(images),
    }
    inventory = {
        "algorithm": runner.RUNTIME_IMAGE_INVENTORY_ALGORITHM,
        **payload,
        "digest": runner._canonical_digest(payload),
    }
    binary = (tmp_path / runner.TARGET).resolve()

    with pytest.raises(
        runner.Figure13BenchmarkError,
        match="must contain the exact executable",
    ):
        runner._validate_runtime_image_inventory(inventory, binary=binary)


def test_evidence_rejects_missing_benchmark_build_configuration_digest(
    tmp_path: Path,
) -> None:
    with pytest.raises(
        runner.Figure13BenchmarkError,
        match="does not match its fresh canonical configure record",
    ):
        _write_valid_evidence_fixture(tmp_path, context_configuration_digest="")


def test_evidence_rejects_loaded_library_build_configuration_mismatch(
    tmp_path: Path,
) -> None:
    with pytest.raises(
        runner.Figure13BenchmarkError,
        match="does not match the running benchmark",
    ):
        _write_valid_evidence_fixture(tmp_path, library_configuration_digest="9" * 64)


def test_evidence_rejects_omitted_mapped_dart_library_identity(
    tmp_path: Path,
) -> None:
    with pytest.raises(
        runner.Figure13BenchmarkError,
        match="exactly cover every mapped libdart image",
    ):
        _write_valid_evidence_fixture(tmp_path, extra_unbound_dart_image=True)
