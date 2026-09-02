from __future__ import annotations

import _ctypes
import ctypes
import importlib.util
import os
import shutil
import struct
import subprocess
import types
import zlib
from pathlib import Path

import pytest

_ROOT = Path(__file__).resolve().parents[3]
_SPEC = importlib.util.spec_from_file_location(
    "capture_source_provenance_under_test",
    _ROOT / "scripts" / "capture_source_provenance.py",
)
assert _SPEC is not None
assert _SPEC.loader is not None
provenance = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(provenance)


def _png_chunk(kind: bytes, payload: bytes) -> bytes:
    return (
        struct.pack(">I", len(payload))
        + kind
        + payload
        + struct.pack(">I", zlib.crc32(kind + payload) & 0xFFFFFFFF)
    )


def _write_png(
    path: Path,
    *,
    red: int,
    green: int = 40,
    blue: int = 80,
    width: int = 16,
    height: int = 16,
) -> None:
    row = bytes([red, green, blue]) * width
    raw = b"".join(b"\x00" + row for _ in range(height))
    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + _png_chunk(
            b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
        )
        + _png_chunk(b"IDAT", zlib.compress(raw))
        + _png_chunk(b"IEND", b"")
    )


def _sequence(directory: Path, colors: list[int]) -> list[Path]:
    directory.mkdir()
    result = []
    for index, color in enumerate(colors, start=1):
        path = directory / f"frame_{index:06d}.png"
        _write_png(path, red=color)
        result.append(path)
    return result


def _probe(video: Path, frames: list[Path], *, expected_count: int | None = None):
    manifest = provenance.ordered_png_sequence_provenance(frames)
    return provenance.probe_decoded_video(
        video,
        source_png_frames=frames,
        expected_frame_count=(len(frames) if expected_count is None else expected_count),
        expected_fps=12,
        expected_width=16,
        expected_height=16,
        png_sequence_digest=manifest["digest"],
    )


def test_video_probe_fully_decodes_and_binds_exact_sequence(tmp_path: Path) -> None:
    frames = _sequence(tmp_path / "frames", [10, 30, 60, 90, 120, 150])
    video = tmp_path / "capture.mp4"
    provenance.encode_capture_video(frames, video, 12)

    result = _probe(video, frames)

    assert result == {
        "codec_name": "h264",
        "content_correspondence": {
            "algorithm": provenance.CAPTURE_VIDEO_CONTENT_CORRESPONDENCE_ALGORITHM,
            "encoder": provenance.capture_video_encoder_record(video),
            "expected_reencoded_sha256": provenance.sha256_file(video),
            "passed": True,
            "source_png_sequence_digest": (
                provenance.ordered_png_sequence_provenance(frames)["digest"]
            ),
        },
        "decoded_frame_count": 6,
        "duration_seconds": "1/2",
        "fps": "12/1",
        "height": 16,
        "pixel_format": "yuv420p",
        "probe_algorithm": provenance.CAPTURE_VIDEO_PROBE_ALGORITHM,
        "width": 16,
    }
    encoder = result["content_correspondence"]["encoder"]
    assert encoder["ffmpeg_version"]
    assert encoder["libx264_version"]
    assert {
        key: value
        for key, value in encoder.items()
        if key not in {"ffmpeg_version", "libx264_version"}
    } == provenance.CAPTURE_VIDEO_ENCODER



@pytest.mark.parametrize("mutation", ("corrupt", "truncated", "wrong_count"))
def test_video_probe_rejects_decoder_and_count_failures(
    tmp_path: Path, mutation: str
) -> None:
    frames = _sequence(tmp_path / "frames", [10, 30, 60, 90])
    video = tmp_path / "capture.mp4"
    provenance.encode_capture_video(frames, video, 12)
    expected_count = None
    if mutation == "corrupt":
        video.write_bytes(b"not an mp4")
    elif mutation == "truncated":
        payload = video.read_bytes()
        video.write_bytes(payload[: len(payload) // 2])
    else:
        expected_count = 3

    with pytest.raises(provenance.CaptureSourceProvenanceError):
        _probe(video, frames, expected_count=expected_count)


@pytest.mark.parametrize("mutation", ("swapped_video", "intermediate_frame"))
def test_video_probe_rejects_valid_but_unrelated_full_interval_content(
    tmp_path: Path, mutation: str
) -> None:
    frames = _sequence(tmp_path / "frames", [10, 30, 60, 90, 120, 150])
    video = tmp_path / "capture.mp4"
    provenance.encode_capture_video(frames, video, 12)
    if mutation == "swapped_video":
        other_frames = _sequence(
            tmp_path / "other_frames", [11, 31, 61, 91, 121, 151]
        )
        provenance.encode_capture_video(other_frames, video, 12)
    else:
        _write_png(frames[2], red=200)

    with pytest.raises(
        provenance.CaptureSourceProvenanceError,
        match="complete final PNG sequence",
    ):
        _probe(video, frames)


def test_every_png_and_screenshot_frame_binding_are_decoded(tmp_path: Path) -> None:
    frames = _sequence(tmp_path / "frames", [10, 30, 60])
    screenshot = tmp_path / "screenshot.png"
    _write_png(screenshot, red=60)
    provenance.capture_artifact_provenance(
        scene_metrics_events=None,
        screenshot=screenshot,
        png_frames=frames,
        video=None,
        video_fps=None,
        screenshot_png_frame_index=3,
    )

    _write_png(screenshot, red=10)
    with pytest.raises(
        provenance.CaptureSourceProvenanceError,
        match="screenshot pixels do not match",
    ):
        provenance.capture_artifact_provenance(
            scene_metrics_events=None,
            screenshot=screenshot,
            png_frames=frames,
            video=None,
            video_fps=None,
            screenshot_png_frame_index=3,
        )

    frames[1].write_text("not a PNG", encoding="utf-8")
    with pytest.raises(provenance.CaptureSourceProvenanceError, match="not a PNG"):
        provenance.ordered_png_sequence_provenance(frames)


def test_benchmark_target_cmake_bytes_are_in_capture_source_digest(
    tmp_path: Path,
) -> None:
    target = tmp_path / "tests" / "benchmark" / "simulation" / "CMakeLists.txt"
    target.parent.mkdir(parents=True)
    target.write_text("add_executable(example example.cpp)\n", encoding="utf-8")
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    subprocess.run(["git", "-C", str(tmp_path), "add", "."], check=True)
    subprocess.run(
        [
            "git",
            "-C",
            str(tmp_path),
            "-c",
            "user.name=Test",
            "-c",
            "user.email=test@example.com",
            "commit",
            "-qm",
            "fixture",
        ],
        check=True,
    )
    before = provenance.compute_capture_source_provenance(tmp_path)
    target.write_text("add_executable(example stale.cpp)\n", encoding="utf-8")
    after = provenance.compute_capture_source_provenance(tmp_path)

    assert "tests/benchmark/simulation/CMakeLists.txt" in before["roots"]
    assert before["digest"] != after["digest"]
    assert before["git_head"] == after["git_head"]


def _write_native_image(path: Path, payload: bytes) -> None:
    """Write a stub process image carrying this platform's native magic.

    The runtime-image inventory only records loaded images that
    ``_is_native_binary_image`` accepts, so runtime fixtures must start with
    the same magic number a real loadable image carries.
    """
    if provenance.sys.platform == "darwin":
        magic = b"\xcf\xfa\xed\xfe"
    elif provenance.sys.platform == "win32":
        magic = b"MZ\x90\x00"
    else:
        magic = b"\x7fELF"
    path.write_bytes(magic + payload)


def _runtime_image_inventory(
    images: list[Path], *, extension: Path
) -> dict[str, object]:
    """Build the inventory ``capture_runtime_provenance`` would record."""
    entries = [
        provenance._runtime_binary_metadata(path, "loaded capture runtime image")
        for path in images
    ]
    entries.sort(key=lambda entry: entry["path"])
    payload = {
        "images": entries,
        "required_images": {
            "dartpy_native_extension": str(extension.resolve(strict=True)),
        },
    }
    return {
        "algorithm": provenance.CAPTURE_RUNTIME_IMAGE_INVENTORY_ALGORITHM,
        **payload,
        "digest": provenance._canonical_json_digest(payload),
    }


def test_runtime_binding_uses_embedded_head_but_current_worktree_digest(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    extension = tmp_path / "_dartpy.so"
    _write_native_image(extension, b"native extension")
    library = tmp_path / "libdart-simulation.so"
    _write_native_image(library, b"shared DART library")
    embedded_head = "1" * 40
    current_head = "2" * 40
    source_digest = "3" * 64
    build_configuration_digest = "4" * 64
    module = types.SimpleNamespace(
        __file__=str(extension),
        __capture_build_configuration_digest__=build_configuration_digest,
        __capture_source_git_head__=embedded_head,
        __capture_source_provenance_digest__=source_digest,
    )
    monkeypatch.setattr(provenance.importlib, "import_module", lambda _name: module)
    monkeypatch.setattr(
        provenance, "_loaded_binary_paths", lambda: {extension, library}
    )
    monkeypatch.setattr(
        provenance,
        "_assert_linux_loaded_mapping_matches_file",
        lambda *_args, **_kwargs: None,
    )
    identity_payload = {
        "build_configuration_digest": build_configuration_digest,
        "build_target": "dart-simulation",
        "cmake_build_type": "Release",
        "compiler_id": "GNU",
        "compiler_version": "15",
        "ndebug": True,
        "optimization_enabled": True,
        "source_git_head": embedded_head,
        "source_provenance_digest": source_digest,
    }
    monkeypatch.setattr(
        provenance,
        "dart_library_build_identity",
        lambda *_args, **_kwargs: {
            "algorithm": provenance.DART_LIBRARY_BUILD_IDENTITY_ALGORITHM,
            **identity_payload,
            "digest": provenance._canonical_json_digest(identity_payload),
        },
    )
    monkeypatch.setattr(
        provenance,
        "compute_capture_source_provenance",
        lambda _root: {
            "algorithm": provenance.CAPTURE_SOURCE_PROVENANCE_ALGORITHM,
            "digest": source_digest,
            "file_count": 1,
            "git_head": current_head,
            "roots": list(provenance.CAPTURE_SOURCE_ROOTS),
        },
    )

    runtime = provenance.capture_runtime_provenance(tmp_path)

    assert runtime["source_git_head"] == embedded_head
    assert runtime["native_extension"]["source_git_head"] == embedded_head
    assert runtime["source_provenance_digest"] == source_digest
    assert runtime["dart_library_linkage"] == "shared"

    mismatched_payload = {
        **identity_payload,
        "build_configuration_digest": "5" * 64,
    }
    monkeypatch.setattr(
        provenance,
        "dart_library_build_identity",
        lambda *_args, **_kwargs: {
            "algorithm": provenance.DART_LIBRARY_BUILD_IDENTITY_ALGORITHM,
            **mismatched_payload,
            "digest": provenance._canonical_json_digest(mismatched_payload),
        },
    )
    with pytest.raises(
        provenance.CaptureSourceProvenanceError,
        match="build configuration does not match the imported dartpy extension",
    ):
        provenance.capture_runtime_provenance(tmp_path)


def test_runtime_binding_rejects_static_or_missing_dart_library_inventory(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    extension = tmp_path / "_dartpy.so"
    _write_native_image(extension, b"native extension")
    source_digest = "3" * 64
    module = types.SimpleNamespace(
        __file__=str(extension),
        __capture_build_configuration_digest__="4" * 64,
        __capture_source_git_head__="1" * 40,
        __capture_source_provenance_digest__=source_digest,
    )
    monkeypatch.setattr(provenance.importlib, "import_module", lambda _name: module)
    monkeypatch.setattr(provenance, "_loaded_binary_paths", lambda: {extension})
    monkeypatch.setattr(
        provenance,
        "_assert_linux_loaded_mapping_matches_file",
        lambda *_args, **_kwargs: None,
    )
    monkeypatch.setattr(
        provenance,
        "compute_capture_source_provenance",
        lambda _root: {"digest": source_digest},
    )

    with pytest.raises(
        provenance.CaptureSourceProvenanceError,
        match="shared DART libraries.*static linkage is unsupported",
    ):
        provenance.capture_runtime_provenance(tmp_path)


@pytest.mark.skipif(
    not provenance.sys.platform.startswith("linux"),
    reason="Linux /proc mapping identity is required for this regression",
)
def test_runtime_binding_rejects_replaced_imported_extension_inode(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    extension = tmp_path / "_dartpy.so"
    replacement = tmp_path / "replacement.so"
    source_extension = Path(_ctypes.__file__).resolve()
    shutil.copy2(source_extension, extension)
    shutil.copy2(source_extension, replacement)
    loaded_handle = ctypes.CDLL(str(extension))
    assert loaded_handle is not None
    os.replace(replacement, extension)

    source_digest = "3" * 64
    module = types.SimpleNamespace(
        __file__=str(extension),
        __capture_build_configuration_digest__="4" * 64,
        __capture_source_git_head__="1" * 40,
        __capture_source_provenance_digest__=source_digest,
    )
    monkeypatch.setattr(provenance.importlib, "import_module", lambda _name: module)
    monkeypatch.setattr(
        provenance,
        "compute_capture_source_provenance",
        lambda _root: {"digest": source_digest},
    )

    with pytest.raises(
        provenance.CaptureSourceProvenanceError,
        match="imported dartpy native extension path no longer identifies",
    ):
        provenance.capture_runtime_provenance(tmp_path)


def test_runtime_binding_rejects_stale_extension_source_digest(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    extension = tmp_path / "_dartpy.so"
    extension.write_bytes(b"native extension")
    module = types.SimpleNamespace(
        __file__=str(extension),
        __capture_build_configuration_digest__="4" * 64,
        __capture_source_git_head__="1" * 40,
        __capture_source_provenance_digest__="2" * 64,
    )
    monkeypatch.setattr(provenance.importlib, "import_module", lambda _name: module)
    monkeypatch.setattr(
        provenance,
        "compute_capture_source_provenance",
        lambda _root: {"digest": "3" * 64},
    )

    with pytest.raises(
        provenance.CaptureSourceProvenanceError,
        match="different capture source state",
    ):
        provenance.capture_runtime_provenance(tmp_path)


def test_runtime_validation_rejects_omitted_loaded_dart_library(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source_digest = "3" * 64
    source_head = "4" * 40
    build_configuration_digest = "5" * 64
    extension = tmp_path / "_dartpy.so"
    library_a = tmp_path / "libdart-a.so"
    library_b = tmp_path / "libdart-b.so"
    _write_native_image(extension, b"extension")
    _write_native_image(library_a, b"library-a")
    _write_native_image(library_b, b"library-b")
    identity_payload = {
        "build_configuration_digest": build_configuration_digest,
        "build_target": "dart-a",
        "cmake_build_type": "Release",
        "compiler_id": "GNU",
        "compiler_version": "15",
        "ndebug": True,
        "optimization_enabled": True,
        "source_git_head": source_head,
        "source_provenance_digest": source_digest,
    }
    identity = {
        "algorithm": provenance.DART_LIBRARY_BUILD_IDENTITY_ALGORITHM,
        **identity_payload,
        "digest": provenance._canonical_json_digest(identity_payload),
    }
    extension_entry = {
        **provenance._runtime_binary_metadata(extension, "extension"),
        "build_configuration_digest": build_configuration_digest,
        "module": "dartpy._dartpy",
        "source_git_head": source_head,
        "source_provenance_digest": source_digest,
    }

    def library_entry(path: Path) -> dict[str, object]:
        return {
            **provenance._runtime_binary_metadata(path, "library"),
            "build_identity": identity,
        }

    recorded_payload = {
        "dart_library_linkage": "shared",
        "loaded_dart_libraries": [library_entry(library_a)],
        "loader_environment": provenance._capture_loader_environment_policy(tmp_path),
        "native_extension": extension_entry,
        "runtime_image_inventory": _runtime_image_inventory(
            [extension, library_a], extension=extension
        ),
        "source_git_head": source_head,
        "source_provenance_digest": source_digest,
    }
    recorded = {
        "algorithm": provenance.CAPTURE_RUNTIME_PROVENANCE_ALGORITHM,
        "digest": provenance._canonical_json_digest(recorded_payload),
        **recorded_payload,
    }
    current_payload = {
        **recorded_payload,
        "loaded_dart_libraries": [library_entry(library_a), library_entry(library_b)],
        "runtime_image_inventory": _runtime_image_inventory(
            [extension, library_a, library_b], extension=extension
        ),
    }
    current = {
        "algorithm": provenance.CAPTURE_RUNTIME_PROVENANCE_ALGORITHM,
        "digest": provenance._canonical_json_digest(current_payload),
        **current_payload,
    }
    monkeypatch.setattr(
        provenance, "dart_library_build_identity", lambda *_args, **_kwargs: identity
    )
    monkeypatch.setattr(
        provenance, "capture_runtime_provenance", lambda _root: current
    )

    with pytest.raises(
        provenance.CaptureSourceProvenanceError,
        match="does not match the currently imported",
    ):
        provenance.validate_capture_runtime_provenance(
            recorded,
            expected_source_digest=source_digest,
            expected_source_git_head=source_head,
            repo_root=tmp_path,
        )


def test_shared_library_provenance_keeps_python_optional_for_cpp_only_configure(
    tmp_path: Path,
) -> None:
    source = tmp_path / "example.cpp"
    source.write_text("int example() { return 0; }\n", encoding="utf-8")
    cmake_lists = tmp_path / "CMakeLists.txt"
    cmake_lists.write_text(
        "\n".join(
            (
                "cmake_minimum_required(VERSION 3.28)",
                "project(provenance_optional LANGUAGES CXX)",
                f'list(APPEND CMAKE_MODULE_PATH "{_ROOT / "cmake"}")',
                "include(dart_capture_source_provenance)",
                "add_library(example SHARED example.cpp)",
                "unset(Python3_EXECUTABLE)",
                "dart_embed_capture_source_provenance(example)",
            )
        )
        + "\n",
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            "cmake",
            "-S",
            str(tmp_path),
            "-B",
            str(tmp_path / "build"),
            "-DPython3_EXECUTABLE=",
        ],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    assert result.returncode == 0, result.stdout
