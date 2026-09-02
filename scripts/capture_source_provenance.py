#!/usr/bin/env python3
"""Deterministic source and artifact provenance for visual captures."""

from __future__ import annotations

import argparse
import ctypes
import importlib
import json
import os
import shutil
import struct
import subprocess
import sys
import tempfile
import zlib
from fractions import Fraction
from hashlib import sha256
from pathlib import Path
from typing import Any

CAPTURE_SOURCE_PROVENANCE_ALGORITHM = "sha256-length-prefixed-capture-source-tree-v1"
CAPTURE_RUNTIME_PROVENANCE_ALGORITHM = "sha256-imported-dartpy-runtime-v4"
CAPTURE_RUNTIME_IMAGE_INVENTORY_ALGORITHM = (
    "sha256-complete-capture-mapped-native-image-inventory-v1"
)
CAPTURE_LOADER_POLICY_ALGORITHM = "empty-loader-control-environment-v1"
CAPTURE_LOADER_ENVIRONMENT_PREFIXES = ("DYLD_", "LD_")
DART_LIBRARY_BUILD_IDENTITY_ALGORITHM = (
    "compiled-dart-shared-library-source-and-build-identity-v2"
)
CAPTURE_ARTIFACT_PROVENANCE_ALGORITHM = "sha256-canonical-capture-artifact-manifest-v3"
CAPTURE_PNG_SEQUENCE_PROVENANCE_ALGORITHM = (
    "sha256-length-prefixed-decoded-ordered-png-manifest-v2"
)
CAPTURE_VIDEO_PROBE_ALGORITHM = "ffprobe-metadata-plus-ffmpeg-full-decode-v1"
CAPTURE_VIDEO_CONTENT_CORRESPONDENCE_ALGORITHM = (
    "deterministic-ffmpeg-reencode-byte-match-v1"
)
CAPTURE_VIDEO_ENCODER = {
    "codec": "libx264",
    "crf": 18,
    "pixel_format": "yuv420p",
    "preset": "veryfast",
    "threads": 1,
}
CAPTURE_SCREENSHOT_BINDING_ALGORITHM = "decoded-rgb-exact-png-frame-match-v1"
CAPTURE_SOURCE_ROOTS = (
    "CMakeLists.txt",
    "cmake",
    "dart",
    "pixi.lock",
    "pixi.toml",
    "pyproject.toml",
    "python/CMakeLists.txt",
    "python/dartpy",
    "python/examples/demos",
    "scripts/capture_py_demo.py",
    "scripts/capture_source_provenance.py",
    "scripts/run_figure13_benchmark.py",
    "tests/benchmark/simulation/CMakeLists.txt",
)


class CaptureSourceProvenanceError(RuntimeError):
    """Raised when the capture source snapshot cannot be resolved."""


def sha256_file(path: Path) -> str:
    path = Path(path)
    _validate_artifact_path(path, "capture artifact")
    digest = sha256()
    try:
        if path.is_symlink():
            raise CaptureSourceProvenanceError(
                f"capture artifact must not be a symbolic link: {path}"
            )
        if not path.is_file():
            raise CaptureSourceProvenanceError(
                f"capture artifact must be a regular file: {path}"
            )
        with path.open("rb") as file:
            for chunk in iter(lambda: file.read(1024 * 1024), b""):
                digest.update(chunk)
    except CaptureSourceProvenanceError:
        raise
    except (OSError, ValueError, UnicodeError) as exc:
        raise CaptureSourceProvenanceError(
            f"cannot hash capture artifact {path}: {exc}"
        ) from exc
    return digest.hexdigest()


def _validate_artifact_path(path: Path, label: str) -> None:
    try:
        text = str(path)
        text.encode("utf-8")
    except (TypeError, ValueError, UnicodeError) as exc:
        raise CaptureSourceProvenanceError(
            f"{label} path must be UTF-8-encodable text"
        ) from exc
    if "\x00" in text:
        raise CaptureSourceProvenanceError(f"{label} path must not contain NUL")


def _artifact_metadata(path: Path, label: str) -> tuple[str, int]:
    path = Path(path)
    _validate_artifact_path(path, label)
    try:
        if path.is_symlink():
            raise CaptureSourceProvenanceError(
                f"{label} must not be a symbolic link: {path}"
            )
        if not path.is_file():
            raise CaptureSourceProvenanceError(
                f"{label} must be a regular file: {path}"
            )
        size_bytes = path.stat().st_size
    except CaptureSourceProvenanceError:
        raise
    except (OSError, ValueError, UnicodeError) as exc:
        raise CaptureSourceProvenanceError(
            f"cannot inspect {label} {path}: {exc}"
        ) from exc
    if size_bytes < 0 or size_bytes > 0xFFFFFFFFFFFFFFFF:
        raise CaptureSourceProvenanceError(
            f"{label} size must fit an unsigned 64-bit integer: {size_bytes}"
        )
    return sha256_file(path), size_bytes


def _canonical_json_digest(value: object) -> str:
    try:
        payload = json.dumps(
            value,
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8")
    except (TypeError, ValueError, UnicodeError) as exc:
        raise CaptureSourceProvenanceError(
            f"capture artifact manifest cannot be encoded: {exc}"
        ) from exc
    return sha256(payload).hexdigest()


def _ordered_png_manifest_digest(files: list[dict[str, Any]]) -> str:
    try:
        digest = sha256()
        digest.update(struct.pack("<Q", len(files)))
        for entry in files:
            index = entry["index"]
            name = entry["file"]
            size_bytes = entry["size_bytes"]
            file_digest = entry["sha256"]
            if (
                not isinstance(index, int)
                or isinstance(index, bool)
                or index < 1
                or not isinstance(name, str)
                or not isinstance(size_bytes, int)
                or isinstance(size_bytes, bool)
                or size_bytes < 0
                or not isinstance(file_digest, str)
            ):
                raise ValueError("invalid ordered PNG manifest entry")
            encoded_name = name.encode("utf-8")
            digest.update(struct.pack("<Q", index))
            digest.update(struct.pack("<Q", len(encoded_name)))
            digest.update(encoded_name)
            digest.update(struct.pack("<Q", size_bytes))
            digest.update(bytes.fromhex(file_digest))
        return digest.hexdigest()
    except (KeyError, OverflowError, struct.error, UnicodeError, ValueError) as exc:
        raise CaptureSourceProvenanceError(
            f"cannot encode ordered PNG manifest: {exc}"
        ) from exc


def _paeth_predictor(left: int, above: int, upper_left: int) -> int:
    prediction = left + above - upper_left
    left_distance = abs(prediction - left)
    above_distance = abs(prediction - above)
    upper_left_distance = abs(prediction - upper_left)
    if left_distance <= above_distance and left_distance <= upper_left_distance:
        return left
    if above_distance <= upper_left_distance:
        return above
    return upper_left


def decode_capture_png(path: Path) -> tuple[int, int, bytes]:
    """Decode the 8-bit RGB/RGBA PNG format emitted by capture_py_demo."""
    try:
        with Path(path).open("rb") as file:
            payload = file.read()
    except OSError as exc:
        raise CaptureSourceProvenanceError(
            f"cannot decode final PNG frame {path}: {exc}"
        ) from exc
    if not payload.startswith(b"\x89PNG\r\n\x1a\n"):
        raise CaptureSourceProvenanceError(f"final PNG frame is not a PNG: {path}")

    offset = 8
    ihdr: bytes | None = None
    compressed = bytearray()
    saw_iend = False
    while offset < len(payload):
        if len(payload) - offset < 12:
            raise CaptureSourceProvenanceError(f"truncated PNG chunk in {path}")
        length = struct.unpack(">I", payload[offset : offset + 4])[0]
        chunk_type = payload[offset + 4 : offset + 8]
        data_start = offset + 8
        data_end = data_start + length
        crc_end = data_end + 4
        if crc_end > len(payload):
            raise CaptureSourceProvenanceError(f"truncated PNG chunk in {path}")
        data = payload[data_start:data_end]
        expected_crc = struct.unpack(">I", payload[data_end:crc_end])[0]
        actual_crc = zlib.crc32(chunk_type + data) & 0xFFFFFFFF
        if actual_crc != expected_crc:
            raise CaptureSourceProvenanceError(f"PNG chunk CRC mismatch in {path}")
        if chunk_type == b"IHDR":
            if ihdr is not None or length != 13:
                raise CaptureSourceProvenanceError(f"invalid PNG IHDR in {path}")
            ihdr = data
        elif chunk_type == b"IDAT":
            if ihdr is None or saw_iend:
                raise CaptureSourceProvenanceError(f"invalid PNG IDAT order in {path}")
            compressed.extend(data)
        elif chunk_type == b"IEND":
            if length != 0:
                raise CaptureSourceProvenanceError(f"invalid PNG IEND in {path}")
            saw_iend = True
            offset = crc_end
            break
        offset = crc_end
    if ihdr is None or not compressed or not saw_iend or offset != len(payload):
        raise CaptureSourceProvenanceError(f"incomplete PNG stream in {path}")

    width, height, bit_depth, color_type, compression, filtering, interlace = (
        struct.unpack(">IIBBBBB", ihdr)
    )
    if width < 1 or height < 1:
        raise CaptureSourceProvenanceError(f"PNG dimensions must be positive: {path}")
    if (
        bit_depth != 8
        or color_type not in (2, 6)
        or compression != 0
        or filtering != 0
        or interlace != 0
    ):
        raise CaptureSourceProvenanceError(
            f"final PNG frame must be non-interlaced 8-bit RGB/RGBA: {path}"
        )
    channels = 3 if color_type == 2 else 4
    row_bytes = width * channels
    expected_size = height * (row_bytes + 1)
    try:
        decompressor = zlib.decompressobj()
        filtered = decompressor.decompress(bytes(compressed)) + decompressor.flush()
    except zlib.error as exc:
        raise CaptureSourceProvenanceError(
            f"cannot decompress final PNG frame {path}: {exc}"
        ) from exc
    if (
        not decompressor.eof
        or decompressor.unused_data
        or decompressor.unconsumed_tail
        or len(filtered) != expected_size
    ):
        raise CaptureSourceProvenanceError(
            f"final PNG frame has invalid decoded byte count: {path}"
        )

    decoded = bytearray(height * row_bytes)
    source_offset = 0
    previous = bytearray(row_bytes)
    for row in range(height):
        filter_type = filtered[source_offset]
        source_offset += 1
        encoded_row = filtered[source_offset : source_offset + row_bytes]
        source_offset += row_bytes
        if filter_type > 4:
            raise CaptureSourceProvenanceError(
                f"final PNG frame has invalid row filter: {path}"
            )
        current = bytearray(row_bytes)
        for column, encoded_byte in enumerate(encoded_row):
            left = current[column - channels] if column >= channels else 0
            above = previous[column]
            upper_left = previous[column - channels] if column >= channels else 0
            if filter_type == 0:
                predictor = 0
            elif filter_type == 1:
                predictor = left
            elif filter_type == 2:
                predictor = above
            elif filter_type == 3:
                predictor = (left + above) // 2
            else:
                predictor = _paeth_predictor(left, above, upper_left)
            current[column] = (encoded_byte + predictor) & 0xFF
        start = row * row_bytes
        decoded[start : start + row_bytes] = current
        previous = current

    if channels == 3:
        rgb = bytes(decoded)
    else:
        rgb = bytes(
            channel
            for offset in range(0, len(decoded), 4)
            for channel in decoded[offset : offset + 3]
        )
    return width, height, rgb


def ordered_png_sequence_provenance(
    png_frames: list[Path] | tuple[Path, ...],
) -> dict[str, Any]:
    """Build a deterministic ordered manifest for final PNG frame bytes."""
    ordered = sorted((Path(path) for path in png_frames), key=lambda path: path.name)
    names = [path.name for path in ordered]
    if len(names) != len(set(names)):
        raise CaptureSourceProvenanceError("final PNG frame basenames must be unique")

    files: list[dict[str, Any]] = []
    dimensions: tuple[int, int] | None = None
    for index, path in enumerate(ordered, start=1):
        if path.suffix.lower() != ".png":
            raise CaptureSourceProvenanceError(
                f"final PNG frame must use a .png suffix: {path}"
            )
        width, height, _ = decode_capture_png(path)
        if dimensions is None:
            dimensions = (width, height)
        elif dimensions != (width, height):
            raise CaptureSourceProvenanceError(
                "final PNG frames must all have identical dimensions"
            )
        frame_digest, size_bytes = _artifact_metadata(path, "final PNG frame")
        files.append(
            {
                "file": path.name,
                "index": index,
                "sha256": frame_digest,
                "size_bytes": size_bytes,
            }
        )
    return {
        "algorithm": CAPTURE_PNG_SEQUENCE_PROVENANCE_ALGORITHM,
        "count": len(files),
        "digest": _ordered_png_manifest_digest(files),
        "files": files,
        "height": dimensions[1] if dimensions is not None else None,
        "width": dimensions[0] if dimensions is not None else None,
    }


def _run_git(repo_root: Path, *args: str) -> bytes:
    try:
        result = subprocess.run(
            ["git", "-C", str(repo_root), *args],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
    except (OSError, UnicodeError, ValueError, subprocess.CalledProcessError) as exc:
        detail = ""
        if isinstance(exc, subprocess.CalledProcessError):
            detail = exc.stderr.decode("utf-8", errors="replace").strip()
        suffix = f": {detail}" if detail else ""
        raise CaptureSourceProvenanceError(
            f"cannot resolve capture source state with git{suffix}"
        ) from exc
    return result.stdout


def _capture_source_paths(repo_root: Path) -> list[str]:
    output = _run_git(
        repo_root,
        "ls-files",
        "--cached",
        "--others",
        "--exclude-standard",
        "-z",
        "--",
        *CAPTURE_SOURCE_ROOTS,
    )
    paths = sorted({os.fsdecode(encoded) for encoded in output.split(b"\0") if encoded})
    if not paths:
        raise CaptureSourceProvenanceError(
            "capture source state did not resolve any files"
        )
    for relative in paths:
        _validate_artifact_path(Path(relative), "capture source")
    return paths


def _path_payload(path: Path) -> tuple[bytes, bytes]:
    _validate_artifact_path(path, "capture source")
    if path.is_symlink():
        try:
            return b"L", os.fsencode(os.readlink(path))
        except OSError as exc:
            raise CaptureSourceProvenanceError(
                f"cannot read capture source symlink {path}: {exc}"
            ) from exc
    if not path.exists():
        raise CaptureSourceProvenanceError(
            f"capture source path is listed by git but missing from the "
            f"working tree: {path}"
        )
    if not path.is_file():
        raise CaptureSourceProvenanceError(
            f"capture source path is not a regular file: {path}"
        )
    try:
        return b"F", path.read_bytes()
    except OSError as exc:
        raise CaptureSourceProvenanceError(
            f"cannot read capture source file {path}: {exc}"
        ) from exc


def compute_capture_source_provenance(repo_root: Path) -> dict[str, Any]:
    """Hash the source tree that can affect a Python demo capture."""

    try:
        repo_root = repo_root.resolve()
    except (OSError, RuntimeError, ValueError, UnicodeError) as exc:
        raise CaptureSourceProvenanceError(
            f"cannot resolve capture source repository root: {exc}"
        ) from exc
    _validate_artifact_path(repo_root, "capture source repository root")
    digest = sha256()
    for root in CAPTURE_SOURCE_ROOTS:
        encoded_root = root.encode("utf-8")
        digest.update(struct.pack("<Q", len(encoded_root)))
        digest.update(encoded_root)

    paths = _capture_source_paths(repo_root)
    for relative in paths:
        try:
            encoded_path = relative.encode("utf-8")
        except UnicodeError as exc:
            raise CaptureSourceProvenanceError(
                f"capture source path must be UTF-8 encodable: {relative!r}"
            ) from exc
        kind, payload = _path_payload(repo_root / relative)
        digest.update(struct.pack("<Q", len(encoded_path)))
        digest.update(encoded_path)
        digest.update(kind)
        digest.update(struct.pack("<Q", len(payload)))
        digest.update(payload)

    head = _run_git(repo_root, "rev-parse", "HEAD").decode("ascii").strip()
    if len(head) != 40 or any(
        character not in "0123456789abcdef" for character in head
    ):
        raise CaptureSourceProvenanceError(
            f"git returned an invalid capture source revision: {head!r}"
        )

    return {
        "algorithm": CAPTURE_SOURCE_PROVENANCE_ALGORITHM,
        "digest": digest.hexdigest(),
        "file_count": len(paths),
        "git_head": head,
        "roots": list(CAPTURE_SOURCE_ROOTS),
    }


def _decode_proc_maps_path(value: str) -> str:
    return (
        value.replace(r"\134", "\\")
        .replace(r"\040", " ")
        .replace(r"\011", "\t")
        .replace(r"\012", "\n")
    )


def _loaded_binary_paths() -> set[Path]:
    """Return paths for binary images loaded into this Python process."""
    paths: set[Path] = set()
    if sys.platform.startswith("linux"):
        try:
            lines = Path("/proc/self/maps").read_text(encoding="utf-8").splitlines()
        except OSError as exc:
            raise CaptureSourceProvenanceError(
                f"cannot enumerate loaded process images: {exc}"
            ) from exc
        for line in lines:
            fields = line.split(maxsplit=5)
            if len(fields) != 6 or not fields[5].startswith("/"):
                continue
            text = fields[5]
            if text.endswith(" (deleted)"):
                raise CaptureSourceProvenanceError(
                    f"loaded process image was deleted during capture: {text}"
                )
            paths.add(Path(_decode_proc_maps_path(text)))
        return paths

    if sys.platform == "darwin":
        try:
            lib = ctypes.CDLL(None)
            image_count = lib._dyld_image_count
            image_count.restype = ctypes.c_uint32
            image_name = lib._dyld_get_image_name
            image_name.argtypes = [ctypes.c_uint32]
            image_name.restype = ctypes.c_char_p
            for index in range(image_count()):
                encoded = image_name(index)
                if encoded:
                    paths.add(Path(os.fsdecode(encoded)))
        except (AttributeError, OSError, TypeError, ValueError) as exc:
            raise CaptureSourceProvenanceError(
                f"cannot enumerate loaded process images: {exc}"
            ) from exc
        return paths

    if sys.platform == "win32":
        try:
            from ctypes import wintypes

            process = ctypes.windll.kernel32.GetCurrentProcess()
            modules = (wintypes.HMODULE * 4096)()
            needed = wintypes.DWORD()
            if not ctypes.windll.psapi.EnumProcessModules(
                process,
                ctypes.byref(modules),
                ctypes.sizeof(modules),
                ctypes.byref(needed),
            ):
                raise OSError("EnumProcessModules failed")
            count = min(needed.value // ctypes.sizeof(wintypes.HMODULE), len(modules))
            for module in modules[:count]:
                buffer = ctypes.create_unicode_buffer(32768)
                length = ctypes.windll.psapi.GetModuleFileNameExW(
                    process, module, buffer, len(buffer)
                )
                if length:
                    paths.add(Path(buffer.value))
        except (AttributeError, OSError, TypeError, ValueError) as exc:
            raise CaptureSourceProvenanceError(
                f"cannot enumerate loaded process images: {exc}"
            ) from exc
        return paths

    raise CaptureSourceProvenanceError(
        f"loaded process image enumeration is unsupported on {sys.platform!r}"
    )


def _runtime_binary_metadata(path: Path, label: str) -> dict[str, Any]:
    try:
        resolved = Path(path).resolve(strict=True)
    except (OSError, RuntimeError, ValueError) as exc:
        raise CaptureSourceProvenanceError(
            f"cannot resolve {label} path {path}: {exc}"
        ) from exc
    digest, size_bytes = _artifact_metadata(resolved, label)
    return {
        "file": resolved.name,
        "path": str(resolved),
        "sha256": digest,
        "size_bytes": size_bytes,
    }


def _capture_runtime_trusted_roots(repo_root: Path) -> list[Path]:
    candidates = [
        Path(repo_root),
        Path(sys.prefix),
        Path(sys.base_prefix),
        Path("/lib"),
        Path("/lib64"),
        Path("/usr/lib"),
        Path("/usr/lib64"),
        Path("/usr/local/lib"),
        Path("/System/Library"),
    ]
    system_root = os.environ.get("SystemRoot")
    if system_root:
        candidates.append(Path(system_root))
    roots: set[Path] = set()
    for candidate in candidates:
        try:
            roots.add(candidate.resolve(strict=True))
        except OSError, RuntimeError, ValueError:
            continue
    return sorted(roots, key=str)


def _capture_loader_environment_policy(repo_root: Path) -> dict[str, Any]:
    present = sorted(
        name
        for name, value in os.environ.items()
        if value and name.startswith(CAPTURE_LOADER_ENVIRONMENT_PREFIXES)
    )
    if present:
        raise CaptureSourceProvenanceError(
            "capture evidence forbids loader-control environment variables: "
            f"{present!r}"
        )
    return {
        "algorithm": CAPTURE_LOADER_POLICY_ALGORITHM,
        "forbidden_environment_prefixes": list(CAPTURE_LOADER_ENVIRONMENT_PREFIXES),
        "passed": True,
        "present_environment_variables": [],
        "trusted_image_roots": [
            str(path) for path in _capture_runtime_trusted_roots(repo_root)
        ],
    }


def _is_native_binary_image(path: Path) -> bool:
    try:
        with path.open("rb") as file:
            magic = file.read(4)
    except OSError as exc:
        raise CaptureSourceProvenanceError(
            f"cannot inspect loaded process image {path}: {exc}"
        ) from exc
    if sys.platform.startswith("linux"):
        return magic == b"\x7fELF"
    if sys.platform == "darwin":
        return magic in {
            b"\xca\xfe\xba\xbe",
            b"\xce\xfa\xed\xfe",
            b"\xcf\xfa\xed\xfe",
            b"\xfe\xed\xfa\xce",
            b"\xfe\xed\xfa\xcf",
        }
    if sys.platform == "win32":
        return magic[:2] == b"MZ"
    raise CaptureSourceProvenanceError(
        f"native process-image validation is unsupported on {sys.platform!r}"
    )


def _loaded_native_binary_paths(repo_root: Path) -> set[Path]:
    roots = _capture_runtime_trusted_roots(repo_root)
    paths: set[Path] = set()
    for candidate in _loaded_binary_paths():
        try:
            path = candidate.resolve(strict=True)
        except (OSError, RuntimeError, ValueError) as exc:
            raise CaptureSourceProvenanceError(
                f"cannot resolve loaded process image {candidate}: {exc}"
            ) from exc
        if not _is_native_binary_image(path):
            continue
        if not any(path.is_relative_to(root) for root in roots):
            raise CaptureSourceProvenanceError(
                "capture process loaded a native binary outside trusted roots: "
                f"{path}"
            )
        paths.add(path)
    return paths


def _capture_runtime_image_inventory(
    repo_root: Path, *, extension_path: Path
) -> dict[str, Any]:
    paths = _loaded_native_binary_paths(repo_root)
    extension_path = extension_path.resolve(strict=True)
    if extension_path not in paths:
        raise CaptureSourceProvenanceError(
            "complete capture runtime-image inventory omitted the dartpy extension"
        )
    images: list[dict[str, Any]] = []
    for path in sorted(paths, key=str):
        _assert_linux_loaded_mapping_matches_file(
            path, pid=os.getpid(), label="loaded capture runtime image"
        )
        metadata = _runtime_binary_metadata(path, "loaded capture runtime image")
        _assert_linux_loaded_mapping_matches_file(
            path, pid=os.getpid(), label="loaded capture runtime image"
        )
        if _runtime_binary_metadata(path, "loaded capture runtime image") != metadata:
            raise CaptureSourceProvenanceError(
                f"loaded capture runtime image changed while inspected: {path}"
            )
        images.append(metadata)
    if _loaded_native_binary_paths(repo_root) != paths:
        raise CaptureSourceProvenanceError(
            "capture runtime-image set changed while provenance was collected"
        )
    image_by_path = {image["path"]: image for image in images}
    payload = {
        "images": images,
        "required_images": {
            "dartpy_native_extension": str(extension_path),
        },
    }
    if image_by_path.get(str(extension_path)) is None:
        raise CaptureSourceProvenanceError(
            "capture runtime-image inventory did not bind dartpy extension bytes"
        )
    return {
        "algorithm": CAPTURE_RUNTIME_IMAGE_INVENTORY_ALGORITHM,
        **payload,
        "digest": _canonical_json_digest(payload),
    }


def _assert_linux_loaded_mapping_matches_file(
    path: Path, *, pid: int, label: str = "loaded native binary"
) -> None:
    """Reject a path that no longer names the inode mapped by *pid*."""
    if not sys.platform.startswith("linux"):
        return
    try:
        lines = Path(f"/proc/{pid}/maps").read_text(encoding="utf-8").splitlines()
        stat = path.stat()
    except (FileNotFoundError, OSError, PermissionError, ProcessLookupError) as exc:
        raise CaptureSourceProvenanceError(
            f"cannot verify {label} mapping for {path}: {exc}"
        ) from exc
    identities: set[tuple[int, int, int]] = set()
    for line in lines:
        fields = line.split(maxsplit=5)
        if len(fields) != 6 or not fields[5].startswith("/"):
            continue
        mapped_text = fields[5]
        if mapped_text.endswith(" (deleted)"):
            mapped_text = mapped_text.removesuffix(" (deleted)")
        mapped_text = _decode_proc_maps_path(mapped_text)
        try:
            if Path(mapped_text).resolve() != path.resolve():
                continue
            major_text, minor_text = fields[3].split(":", maxsplit=1)
            identities.add((int(major_text, 16), int(minor_text, 16), int(fields[4])))
        except OSError, RuntimeError, ValueError:
            continue
    current = (os.major(stat.st_dev), os.minor(stat.st_dev), stat.st_ino)
    if not identities or identities != {current}:
        raise CaptureSourceProvenanceError(
            f"{label} path no longer identifies the mapped binary image: {path}"
        )


def dart_library_build_identity(
    path: Path,
    *,
    expected_source_digest: str | None = None,
    expected_source_git_head: str | None = None,
) -> dict[str, Any]:
    """Read the source/build identity exported by one exact DART image."""
    try:
        resolved = Path(path).resolve(strict=True)
        library = ctypes.CDLL(str(resolved))
    except (OSError, RuntimeError, ValueError) as exc:
        raise CaptureSourceProvenanceError(
            f"cannot load DART library provenance from {path}: {exc}"
        ) from exc

    def text_symbol(name: str, label: str) -> str:
        try:
            symbol = getattr(library, name)
            symbol.argtypes = []
            symbol.restype = ctypes.c_char_p
            encoded = symbol()
            value = encoded.decode("utf-8") if encoded is not None else ""
        except (AttributeError, OSError, TypeError, UnicodeError, ValueError) as exc:
            raise CaptureSourceProvenanceError(
                f"DART library {resolved} does not expose {label}: {exc}"
            ) from exc
        if not value:
            raise CaptureSourceProvenanceError(
                f"DART library {resolved} exposes an empty {label}"
            )
        return value

    def flag_symbol(name: str, label: str) -> bool:
        try:
            symbol = getattr(library, name)
            symbol.argtypes = []
            symbol.restype = ctypes.c_int
            value = symbol()
        except (AttributeError, OSError, TypeError, ValueError) as exc:
            raise CaptureSourceProvenanceError(
                f"DART library {resolved} does not expose {label}: {exc}"
            ) from exc
        if value not in (0, 1):
            raise CaptureSourceProvenanceError(
                f"DART library {resolved} exposes invalid {label}: {value!r}"
            )
        return bool(value)

    source_digest = text_symbol(
        "dart_capture_source_provenance_digest_v1", "capture source digest"
    )
    source_git_head = text_symbol(
        "dart_capture_source_git_head_v1", "capture source Git HEAD"
    )
    build_configuration_digest = text_symbol(
        "dart_capture_build_configuration_digest_v1",
        "build configuration digest",
    )
    if len(source_digest) != 64 or any(
        character not in "0123456789abcdef" for character in source_digest
    ):
        raise CaptureSourceProvenanceError(
            f"DART library {resolved} exposes an invalid capture source digest"
        )
    if len(source_git_head) != 40 or any(
        character not in "0123456789abcdef" for character in source_git_head
    ):
        raise CaptureSourceProvenanceError(
            f"DART library {resolved} exposes an invalid capture source Git HEAD"
        )
    if len(build_configuration_digest) != 64 or any(
        character not in "0123456789abcdef" for character in build_configuration_digest
    ):
        raise CaptureSourceProvenanceError(
            f"DART library {resolved} exposes an invalid build configuration digest"
        )
    if expected_source_digest is not None and source_digest != expected_source_digest:
        raise CaptureSourceProvenanceError(
            f"DART library {resolved} was compiled from stale capture source"
        )
    if (
        expected_source_git_head is not None
        and source_git_head != expected_source_git_head
    ):
        raise CaptureSourceProvenanceError(
            f"DART library {resolved} was compiled at a different Git HEAD"
        )
    payload = {
        "build_configuration_digest": build_configuration_digest,
        "build_target": text_symbol(
            "dart_capture_build_target_v1", "CMake build target"
        ),
        "cmake_build_type": text_symbol(
            "dart_capture_cmake_build_type_v1", "CMake build type"
        ),
        "compiler_id": text_symbol("dart_capture_compiler_id_v1", "compiler ID"),
        "compiler_version": text_symbol(
            "dart_capture_compiler_version_v1", "compiler version"
        ),
        "ndebug": flag_symbol("dart_capture_ndebug_v1", "NDEBUG state"),
        "optimization_enabled": flag_symbol(
            "dart_capture_optimization_enabled_v1", "optimization state"
        ),
        "source_git_head": source_git_head,
        "source_provenance_digest": source_digest,
    }
    return {
        "algorithm": DART_LIBRARY_BUILD_IDENTITY_ALGORITHM,
        **payload,
        "digest": _canonical_json_digest(payload),
    }


def capture_runtime_provenance(
    repo_root: Path,
    *,
    module_name: str = "dartpy._dartpy",
) -> dict[str, Any]:
    """Bind the imported dartpy extension and loaded DART shared images."""
    loader_environment = _capture_loader_environment_policy(repo_root)
    try:
        module = importlib.import_module(module_name)
    except (ImportError, OSError, RuntimeError, ValueError) as exc:
        raise CaptureSourceProvenanceError(
            f"cannot import capture runtime module {module_name!r}: {exc}"
        ) from exc

    extension_path = getattr(module, "__file__", None)
    if not isinstance(extension_path, str) or not extension_path:
        raise CaptureSourceProvenanceError(
            f"capture runtime module {module_name!r} has no native file"
        )
    embedded_digest = getattr(module, "__capture_source_provenance_digest__", None)
    if (
        not isinstance(embedded_digest, str)
        or len(embedded_digest) != 64
        or any(character not in "0123456789abcdef" for character in embedded_digest)
    ):
        raise CaptureSourceProvenanceError(
            "imported dartpy extension does not expose a valid compile-time "
            "capture source digest"
        )
    embedded_git_head = getattr(module, "__capture_source_git_head__", None)
    if (
        not isinstance(embedded_git_head, str)
        or len(embedded_git_head) != 40
        or any(character not in "0123456789abcdef" for character in embedded_git_head)
    ):
        raise CaptureSourceProvenanceError(
            "imported dartpy extension does not expose a valid compile-time Git HEAD"
        )
    embedded_build_configuration = getattr(
        module, "__capture_build_configuration_digest__", None
    )
    if (
        not isinstance(embedded_build_configuration, str)
        or len(embedded_build_configuration) != 64
        or any(
            character not in "0123456789abcdef"
            for character in embedded_build_configuration
        )
    ):
        raise CaptureSourceProvenanceError(
            "imported dartpy extension does not expose a valid compile-time "
            "build configuration digest"
        )

    current_source = compute_capture_source_provenance(repo_root)
    if embedded_digest != current_source["digest"]:
        raise CaptureSourceProvenanceError(
            "imported dartpy extension was built from a different capture "
            "source state; rebuild before capturing evidence"
        )
    # The working-tree digest, not the repository's current HEAD, decides
    # whether this binary is stale. Evidence files and documentation may be
    # committed after a build without changing capture-affecting bytes. The
    # embedded HEAD remains cross-bound to the manifest below.

    extension_resolved = Path(extension_path).resolve(strict=True)
    _assert_linux_loaded_mapping_matches_file(
        extension_resolved,
        pid=os.getpid(),
        label="imported dartpy native extension",
    )
    extension = _runtime_binary_metadata(
        extension_resolved, "imported dartpy native extension"
    )
    _assert_linux_loaded_mapping_matches_file(
        extension_resolved,
        pid=os.getpid(),
        label="imported dartpy native extension",
    )
    if _runtime_binary_metadata(
        extension_resolved, "imported dartpy native extension"
    ) != {key: extension[key] for key in ("file", "path", "sha256", "size_bytes")}:
        raise CaptureSourceProvenanceError(
            "imported dartpy native extension changed while it was inspected: "
            f"{extension_resolved}"
        )
    extension["module"] = module_name
    extension["build_configuration_digest"] = embedded_build_configuration
    extension["source_git_head"] = embedded_git_head
    extension["source_provenance_digest"] = embedded_digest

    runtime_image_inventory = _capture_runtime_image_inventory(
        repo_root, extension_path=extension_resolved
    )
    loaded_libraries: list[dict[str, Any]] = []
    extension_resolved = Path(extension["path"])
    for image in runtime_image_inventory["images"]:
        resolved = Path(image["path"])
        if resolved == extension_resolved:
            continue
        name = resolved.name.lower()
        is_dart_library = name.startswith("libdart") or (
            sys.platform == "win32"
            and name.startswith("dart")
            and name.endswith(".dll")
        )
        if is_dart_library:
            _assert_linux_loaded_mapping_matches_file(
                resolved, pid=os.getpid(), label="loaded DART library"
            )
            metadata = _runtime_binary_metadata(resolved, "loaded DART library")
            metadata["build_identity"] = dart_library_build_identity(
                resolved,
                expected_source_digest=embedded_digest,
                expected_source_git_head=embedded_git_head,
            )
            if (
                metadata["build_identity"].get("build_configuration_digest")
                != embedded_build_configuration
            ):
                raise CaptureSourceProvenanceError(
                    "loaded DART library build configuration does not match "
                    f"the imported dartpy extension: {resolved}"
                )
            if _runtime_binary_metadata(resolved, "loaded DART library") != {
                key: metadata[key] for key in ("file", "path", "sha256", "size_bytes")
            }:
                raise CaptureSourceProvenanceError(
                    f"loaded DART library changed while it was inspected: {resolved}"
                )
            loaded_libraries.append(metadata)

    if not loaded_libraries:
        raise CaptureSourceProvenanceError(
            "capture evidence requires shared DART libraries with queryable "
            "compiled source/build identities; static linkage is unsupported"
        )
    if runtime_image_inventory != _capture_runtime_image_inventory(
        repo_root, extension_path=extension_resolved
    ):
        raise CaptureSourceProvenanceError(
            "capture runtime-image bytes or mappings changed while DART "
            "identities were collected"
        )
    runtime_dart_paths = sorted(
        image["path"]
        for image in runtime_image_inventory["images"]
        if image["file"].lower().startswith("libdart")
        or (
            sys.platform == "win32"
            and image["file"].lower().startswith("dart")
            and image["file"].lower().endswith(".dll")
        )
    )
    declared_dart_paths = [library["path"] for library in loaded_libraries]
    if declared_dart_paths != runtime_dart_paths:
        raise CaptureSourceProvenanceError(
            "capture DART library inventory must exactly cover every mapped "
            "DART runtime image"
        )

    payload = {
        "dart_library_linkage": "shared",
        "loader_environment": loader_environment,
        "loaded_dart_libraries": loaded_libraries,
        "native_extension": extension,
        "runtime_image_inventory": runtime_image_inventory,
        "source_git_head": embedded_git_head,
        "source_provenance_digest": current_source["digest"],
    }
    return {
        "algorithm": CAPTURE_RUNTIME_PROVENANCE_ALGORITHM,
        "digest": _canonical_json_digest(payload),
        **payload,
    }


def _validate_recorded_capture_runtime_image_inventory(
    value: object,
    *,
    extension: dict[str, Any],
    repo_root: Path,
) -> dict[str, Any]:
    if not isinstance(value, dict) or set(value) != {
        "algorithm",
        "digest",
        "images",
        "required_images",
    }:
        raise CaptureSourceProvenanceError(
            "capture runtime-image inventory has unexpected fields"
        )
    if value.get("algorithm") != CAPTURE_RUNTIME_IMAGE_INVENTORY_ALGORITHM:
        raise CaptureSourceProvenanceError(
            "capture runtime-image inventory algorithm does not match"
        )
    images_value = value.get("images")
    if not isinstance(images_value, list) or not images_value:
        raise CaptureSourceProvenanceError(
            "capture runtime-image inventory must contain native images"
        )
    trusted_roots = _capture_runtime_trusted_roots(repo_root)
    images: list[dict[str, Any]] = []
    for index, image in enumerate(images_value):
        label = f"capture runtime image {index}"
        if not isinstance(image, dict) or set(image) != {
            "file",
            "path",
            "sha256",
            "size_bytes",
        }:
            raise CaptureSourceProvenanceError(f"{label} has unexpected fields")
        path_value = image.get("path")
        if not isinstance(path_value, str) or not Path(path_value).is_absolute():
            raise CaptureSourceProvenanceError(f"{label}.path must be absolute")
        candidate = Path(path_value)
        try:
            path = candidate.resolve(strict=True)
        except (OSError, RuntimeError, ValueError) as exc:
            raise CaptureSourceProvenanceError(
                f"cannot resolve {label}: {exc}"
            ) from exc
        if candidate.is_symlink() or candidate != path or not path.is_file():
            raise CaptureSourceProvenanceError(
                f"{label} must be canonical, regular, and non-symlink"
            )
        if not _is_native_binary_image(path):
            raise CaptureSourceProvenanceError(f"{label} must be a native binary image")
        if not any(path.is_relative_to(root) for root in trusted_roots):
            raise CaptureSourceProvenanceError(
                f"{label} is outside the trusted runtime-image roots"
            )
        current = _runtime_binary_metadata(path, label)
        if image != current:
            raise CaptureSourceProvenanceError(
                f"{label} does not match current binary bytes"
            )
        images.append(current)
    paths = [image["path"] for image in images]
    if paths != sorted(set(paths)):
        raise CaptureSourceProvenanceError(
            "capture runtime images must be unique and path-sorted"
        )
    required_images = value.get("required_images")
    expected_required = {
        "dartpy_native_extension": extension["path"],
    }
    if required_images != expected_required:
        raise CaptureSourceProvenanceError(
            "capture runtime-image required binding does not match dartpy"
        )
    image_by_path = {image["path"]: image for image in images}
    extension_image = image_by_path.get(extension["path"])
    if extension_image != {
        key: extension[key] for key in ("file", "path", "sha256", "size_bytes")
    }:
        raise CaptureSourceProvenanceError(
            "capture runtime-image inventory does not bind the dartpy extension"
        )
    payload = {"images": images, "required_images": expected_required}
    digest = _canonical_json_digest(payload)
    if value.get("digest") != digest:
        raise CaptureSourceProvenanceError(
            "capture runtime-image inventory digest does not match"
        )
    return {
        "algorithm": CAPTURE_RUNTIME_IMAGE_INVENTORY_ALGORITHM,
        **payload,
        "digest": digest,
    }


def validate_capture_runtime_provenance(
    recorded: object,
    *,
    expected_source_digest: str,
    expected_source_git_head: str,
    repo_root: Path,
) -> dict[str, Any]:
    """Validate and re-hash every runtime binary named by a capture manifest."""
    if not isinstance(recorded, dict):
        raise CaptureSourceProvenanceError(
            "capture runtime provenance must be an object"
        )
    expected_keys = {
        "algorithm",
        "dart_library_linkage",
        "digest",
        "loader_environment",
        "loaded_dart_libraries",
        "native_extension",
        "runtime_image_inventory",
        "source_git_head",
        "source_provenance_digest",
    }
    if set(recorded) != expected_keys:
        raise CaptureSourceProvenanceError(
            "capture runtime provenance has unexpected fields"
        )
    if recorded.get("algorithm") != CAPTURE_RUNTIME_PROVENANCE_ALGORITHM:
        raise CaptureSourceProvenanceError(
            "capture runtime provenance algorithm does not match"
        )
    if recorded.get("source_provenance_digest") != expected_source_digest:
        raise CaptureSourceProvenanceError(
            "capture runtime source digest does not match current source"
        )
    if recorded.get("source_git_head") != expected_source_git_head:
        raise CaptureSourceProvenanceError(
            "capture runtime Git HEAD does not match capture source provenance"
        )
    loader_environment = _capture_loader_environment_policy(repo_root)
    if recorded.get("loader_environment") != loader_environment:
        raise CaptureSourceProvenanceError(
            "capture runtime loader environment does not match the fail-closed "
            "policy"
        )

    def validate_binary(
        value: object, label: str, *, extension: bool
    ) -> dict[str, Any]:
        if not isinstance(value, dict):
            raise CaptureSourceProvenanceError(f"{label} must be an object")
        keys = {"file", "path", "sha256", "size_bytes"}
        if extension:
            keys.update(
                {
                    "build_configuration_digest",
                    "module",
                    "source_git_head",
                    "source_provenance_digest",
                }
            )
        else:
            keys.add("build_identity")
        if set(value) != keys:
            raise CaptureSourceProvenanceError(f"{label} has unexpected fields")
        path_value = value.get("path")
        if not isinstance(path_value, str) or not path_value:
            raise CaptureSourceProvenanceError(f"{label}.path must be non-empty")
        current = _runtime_binary_metadata(Path(path_value), label)
        for key in ("file", "path", "sha256", "size_bytes"):
            if value.get(key) != current[key]:
                raise CaptureSourceProvenanceError(
                    f"{label}.{key} does not match current binary bytes"
                )
        if extension:
            if value.get("module") != "dartpy._dartpy":
                raise CaptureSourceProvenanceError(
                    "capture native extension module must be 'dartpy._dartpy'"
                )
            if value.get("source_provenance_digest") != expected_source_digest:
                raise CaptureSourceProvenanceError(
                    "capture native extension was compiled from stale source"
                )
            if value.get("source_git_head") != expected_source_git_head:
                raise CaptureSourceProvenanceError(
                    "capture native extension was compiled at a different Git HEAD"
                )
            configuration_digest = value.get("build_configuration_digest")
            if (
                not isinstance(configuration_digest, str)
                or len(configuration_digest) != 64
                or any(
                    character not in "0123456789abcdef"
                    for character in configuration_digest
                )
            ):
                raise CaptureSourceProvenanceError(
                    "capture native extension build configuration digest is invalid"
                )
        else:
            current_build_identity = dart_library_build_identity(
                Path(current["path"]),
                expected_source_digest=expected_source_digest,
                expected_source_git_head=expected_source_git_head,
            )
            if value.get("build_identity") != current_build_identity:
                raise CaptureSourceProvenanceError(
                    f"{label}.build_identity does not match the loaded library"
                )
            if (
                current_build_identity.get("build_configuration_digest")
                != extension_build_configuration_digest
            ):
                raise CaptureSourceProvenanceError(
                    f"{label}.build_identity does not match the dartpy build "
                    "configuration"
                )
        return dict(value)

    extension_build_configuration_digest: str | None = None
    extension = validate_binary(
        recorded.get("native_extension"),
        "capture native extension",
        extension=True,
    )
    extension_build_configuration_digest = extension["build_configuration_digest"]
    runtime_image_inventory = _validate_recorded_capture_runtime_image_inventory(
        recorded.get("runtime_image_inventory"),
        extension=extension,
        repo_root=repo_root,
    )
    libraries_value = recorded.get("loaded_dart_libraries")
    if not isinstance(libraries_value, list) or not libraries_value:
        raise CaptureSourceProvenanceError(
            "capture loaded DART libraries must be a non-empty list; static "
            "linkage cannot prove per-library source/build identity"
        )
    libraries = [
        validate_binary(value, "capture loaded DART library", extension=False)
        for value in libraries_value
    ]
    paths = [value["path"] for value in libraries]
    if paths != sorted(set(paths)):
        raise CaptureSourceProvenanceError(
            "capture loaded DART libraries must be unique and path-sorted"
        )
    runtime_dart_paths = sorted(
        image["path"]
        for image in runtime_image_inventory["images"]
        if image["file"].lower().startswith("libdart")
        or (
            sys.platform == "win32"
            and image["file"].lower().startswith("dart")
            and image["file"].lower().endswith(".dll")
        )
    )
    if paths != runtime_dart_paths:
        raise CaptureSourceProvenanceError(
            "capture loaded DART libraries must exactly cover every mapped "
            "DART runtime image"
        )
    linkage = recorded.get("dart_library_linkage")
    if linkage != "shared":
        raise CaptureSourceProvenanceError(
            "capture DART library linkage does not match loaded library inventory"
        )
    payload = {
        "dart_library_linkage": linkage,
        "loader_environment": loader_environment,
        "loaded_dart_libraries": libraries,
        "native_extension": extension,
        "runtime_image_inventory": runtime_image_inventory,
        "source_git_head": expected_source_git_head,
        "source_provenance_digest": expected_source_digest,
    }
    digest = _canonical_json_digest(payload)
    if recorded.get("digest") != digest:
        raise CaptureSourceProvenanceError(
            "capture runtime provenance digest does not match runtime inventory"
        )
    validated = {
        "algorithm": CAPTURE_RUNTIME_PROVENANCE_ALGORITHM,
        "digest": digest,
        **payload,
    }
    current_runtime = capture_runtime_provenance(repo_root)
    core_keys = {
        "algorithm",
        "dart_library_linkage",
        "loaded_dart_libraries",
        "native_extension",
        "source_git_head",
        "source_provenance_digest",
    }
    if any(validated[key] != current_runtime[key] for key in core_keys):
        raise CaptureSourceProvenanceError(
            "capture DART runtime identity does not match the currently "
            "imported dartpy runtime"
        )
    return validated


def _required_tool(name: str) -> str:
    path = shutil.which(name)
    if path is None:
        raise CaptureSourceProvenanceError(
            f"{name} is required to validate encoded capture video"
        )
    return path


def _run_media_tool(command: list[str], label: str) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(
            command,
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
    except (OSError, UnicodeError, ValueError, subprocess.CalledProcessError) as exc:
        detail = ""
        if isinstance(exc, subprocess.CalledProcessError):
            detail = (exc.stderr or exc.stdout or "").strip()
        suffix = f": {detail}" if detail else ""
        raise CaptureSourceProvenanceError(f"{label} failed{suffix}") from exc


def encode_capture_video(
    png_frames: list[Path] | tuple[Path, ...], output: Path, fps: int
) -> None:
    """Encode the exact ordered capture PNG sequence with fixed settings."""
    ordered = sorted((Path(path) for path in png_frames), key=lambda path: path.name)
    expected_names = [f"frame_{index:06d}.png" for index in range(1, len(ordered) + 1)]
    if not ordered or [path.name for path in ordered] != expected_names:
        raise CaptureSourceProvenanceError(
            "video source must be one contiguous frame_%06d.png sequence"
        )
    if len({path.parent.resolve() for path in ordered}) != 1:
        raise CaptureSourceProvenanceError(
            "video source PNG frames must share one directory"
        )
    if not isinstance(fps, int) or isinstance(fps, bool) or fps < 1:
        raise CaptureSourceProvenanceError("encoded capture video fps must be positive")
    ffmpeg = _required_tool("ffmpeg")
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    _run_media_tool(
        [
            ffmpeg,
            "-nostdin",
            "-v",
            "error",
            "-y",
            "-framerate",
            str(fps),
            "-start_number",
            "1",
            "-i",
            str(ordered[0].parent / "frame_%06d.png"),
            "-frames:v",
            str(len(ordered)),
            "-an",
            "-c:v",
            str(CAPTURE_VIDEO_ENCODER["codec"]),
            "-preset",
            str(CAPTURE_VIDEO_ENCODER["preset"]),
            "-crf",
            str(CAPTURE_VIDEO_ENCODER["crf"]),
            "-threads",
            str(CAPTURE_VIDEO_ENCODER["threads"]),
            "-pix_fmt",
            str(CAPTURE_VIDEO_ENCODER["pixel_format"]),
            "-movflags",
            "+faststart",
            str(output),
        ],
        "capture video encoding",
    )


def _video_content_correspondence(
    video: Path,
    png_frames: list[Path] | tuple[Path, ...],
    *,
    fps: int,
    png_sequence_digest: str,
) -> dict[str, Any]:
    with tempfile.TemporaryDirectory(prefix="dart-capture-video-verify-") as temp_dir:
        expected_video = Path(temp_dir) / "expected.mp4"
        encode_capture_video(png_frames, expected_video, fps)
        expected_sha256 = sha256_file(expected_video)
    actual_sha256 = sha256_file(video)
    if actual_sha256 != expected_sha256:
        raise CaptureSourceProvenanceError(
            "encoded capture video bytes do not match deterministic re-encoding "
            "of the complete final PNG sequence"
        )
    return {
        "algorithm": CAPTURE_VIDEO_CONTENT_CORRESPONDENCE_ALGORITHM,
        "encoder": dict(CAPTURE_VIDEO_ENCODER),
        "expected_reencoded_sha256": expected_sha256,
        "passed": True,
        "source_png_sequence_digest": png_sequence_digest,
    }


def _positive_rational(value: object, label: str) -> Fraction:
    if not isinstance(value, str) or not value or value in {"0/0", "N/A"}:
        raise CaptureSourceProvenanceError(f"{label} is unavailable")
    try:
        rational = Fraction(value)
    except (ValueError, ZeroDivisionError) as exc:
        raise CaptureSourceProvenanceError(f"{label} is invalid: {value!r}") from exc
    if rational <= 0:
        raise CaptureSourceProvenanceError(f"{label} must be positive")
    return rational


def probe_decoded_video(
    video: Path,
    *,
    source_png_frames: list[Path] | tuple[Path, ...],
    expected_frame_count: int,
    expected_fps: int,
    expected_width: int,
    expected_height: int,
    png_sequence_digest: str,
) -> dict[str, Any]:
    """Probe and fully decode one MP4, rejecting metadata-only attestations."""
    for value, label in (
        (expected_frame_count, "expected video frame count"),
        (expected_fps, "expected video fps"),
        (expected_width, "expected video width"),
        (expected_height, "expected video height"),
    ):
        if not isinstance(value, int) or isinstance(value, bool) or value < 1:
            raise CaptureSourceProvenanceError(f"{label} must be a positive integer")

    video = Path(video)
    _artifact_metadata(video, "encoded capture video")
    ffprobe = _required_tool("ffprobe")
    ffmpeg = _required_tool("ffmpeg")
    probe = _run_media_tool(
        [
            ffprobe,
            "-v",
            "error",
            "-select_streams",
            "v:0",
            "-count_frames",
            "-show_entries",
            (
                "stream=codec_name,pix_fmt,width,height,avg_frame_rate,"
                "r_frame_rate,nb_frames,nb_read_frames"
            ),
            "-of",
            "json",
            str(video),
        ],
        "encoded capture video probe",
    )
    try:
        payload = json.loads(probe.stdout)
        streams = payload["streams"]
        if not isinstance(streams, list) or len(streams) != 1:
            raise ValueError("expected exactly one selected video stream")
        stream = streams[0]
        if not isinstance(stream, dict):
            raise ValueError("video stream metadata must be an object")
    except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise CaptureSourceProvenanceError(
            f"encoded capture video probe returned invalid metadata: {exc}"
        ) from exc

    try:
        frame_count = int(stream["nb_read_frames"])
        width = int(stream["width"])
        height = int(stream["height"])
    except (KeyError, TypeError, ValueError) as exc:
        raise CaptureSourceProvenanceError(
            "encoded capture video is missing decoded count or dimensions"
        ) from exc
    if frame_count != expected_frame_count:
        raise CaptureSourceProvenanceError(
            "encoded capture video decoded frame count does not match final PNG "
            f"sequence: {frame_count} != {expected_frame_count}"
        )
    if (width, height) != (expected_width, expected_height):
        raise CaptureSourceProvenanceError(
            "encoded capture video dimensions do not match capture contract: "
            f"{width}x{height} != {expected_width}x{expected_height}"
        )

    fps = _positive_rational(stream.get("avg_frame_rate"), "average video fps")
    nominal_fps = _positive_rational(stream.get("r_frame_rate"), "nominal video fps")
    if fps != expected_fps or nominal_fps != fps:
        raise CaptureSourceProvenanceError(
            "encoded capture video frame rate does not match capture contract: "
            f"average={fps}, nominal={nominal_fps}, expected={expected_fps}/1"
        )
    encoded_frame_count = stream.get("nb_frames")
    if encoded_frame_count not in (None, "N/A"):
        try:
            if int(encoded_frame_count) != frame_count:
                raise CaptureSourceProvenanceError(
                    "encoded and decoded video frame counts disagree"
                )
        except (TypeError, ValueError) as exc:
            raise CaptureSourceProvenanceError(
                "encoded capture video frame count metadata is invalid"
            ) from exc

    codec_name = stream.get("codec_name")
    pixel_format = stream.get("pix_fmt")
    if not isinstance(codec_name, str) or not codec_name:
        raise CaptureSourceProvenanceError("encoded capture video codec is unavailable")
    if not isinstance(pixel_format, str) or not pixel_format:
        raise CaptureSourceProvenanceError(
            "encoded capture video pixel format is unavailable"
        )

    _run_media_tool(
        [
            ffmpeg,
            "-nostdin",
            "-v",
            "error",
            "-xerror",
            "-i",
            str(video),
            "-map",
            "0:v:0",
            "-f",
            "null",
            "-",
        ],
        "encoded capture video full decode",
    )

    duration = Fraction(frame_count, 1) / fps
    content_correspondence = _video_content_correspondence(
        video,
        source_png_frames,
        fps=expected_fps,
        png_sequence_digest=png_sequence_digest,
    )
    return {
        "codec_name": codec_name,
        "content_correspondence": content_correspondence,
        "decoded_frame_count": frame_count,
        "duration_seconds": f"{duration.numerator}/{duration.denominator}",
        "fps": f"{fps.numerator}/{fps.denominator}",
        "height": height,
        "pixel_format": pixel_format,
        "probe_algorithm": CAPTURE_VIDEO_PROBE_ALGORITHM,
        "width": width,
    }


def capture_artifact_provenance(
    *,
    scene_metrics_events: Path | None,
    screenshot: Path,
    png_frames: list[Path] | tuple[Path, ...],
    video: Path | None,
    video_fps: int | None,
    video_width: int | None = None,
    video_height: int | None = None,
    screenshot_png_frame_index: int | None = None,
) -> dict[str, Any]:
    """Bind every final PNG plus the optional encoded MP4 to one digest."""
    screenshot_digest, _ = _artifact_metadata(screenshot, "capture screenshot")
    frame_provenance = ordered_png_sequence_provenance(png_frames)
    ordered_frames = sorted(
        (Path(path) for path in png_frames), key=lambda path: path.name
    )
    if not ordered_frames:
        if screenshot_png_frame_index is not None:
            raise CaptureSourceProvenanceError(
                "screenshot PNG-frame index must be absent for an empty sequence"
            )
        screenshot_binding: dict[str, Any] | None = None
    else:
        if (
            not isinstance(screenshot_png_frame_index, int)
            or isinstance(screenshot_png_frame_index, bool)
            or not 1 <= screenshot_png_frame_index <= len(ordered_frames)
        ):
            raise CaptureSourceProvenanceError(
                "screenshot PNG-frame index is outside the final sequence"
            )
        selected_frame = ordered_frames[screenshot_png_frame_index - 1]
        screenshot_width, screenshot_height, screenshot_rgb = decode_capture_png(
            screenshot
        )
        frame_width, frame_height, frame_rgb = decode_capture_png(selected_frame)
        if (screenshot_width, screenshot_height) != (frame_width, frame_height):
            raise CaptureSourceProvenanceError(
                "capture screenshot dimensions do not match its bound PNG frame"
            )
        if screenshot_rgb != frame_rgb:
            raise CaptureSourceProvenanceError(
                "capture screenshot pixels do not match its bound PNG frame"
            )
        screenshot_binding = {
            "algorithm": CAPTURE_SCREENSHOT_BINDING_ALGORITHM,
            "passed": True,
            "png_frame_file": selected_frame.name,
            "png_frame_index": screenshot_png_frame_index,
            "png_frame_sha256": frame_provenance["files"][
                screenshot_png_frame_index - 1
            ]["sha256"],
        }

    metrics_digest: str | None = None
    if scene_metrics_events is not None:
        metrics_digest, _ = _artifact_metadata(
            scene_metrics_events, "scene metrics event log"
        )

    if video is None:
        if any(value is not None for value in (video_fps, video_width, video_height)):
            raise CaptureSourceProvenanceError(
                "video fps and dimensions must be absent when no encoded video is present"
            )
        video_provenance: dict[str, Any] | None = None
    else:
        if (
            not isinstance(video_fps, int)
            or isinstance(video_fps, bool)
            or video_fps < 1
        ):
            raise CaptureSourceProvenanceError(
                "encoded video fps must be a positive integer"
            )
        if (
            not isinstance(video_width, int)
            or isinstance(video_width, bool)
            or video_width < 1
            or not isinstance(video_height, int)
            or isinstance(video_height, bool)
            or video_height < 1
        ):
            raise CaptureSourceProvenanceError(
                "encoded video dimensions must be positive integers"
            )
        if (video_width, video_height) != (
            frame_provenance["width"],
            frame_provenance["height"],
        ):
            raise CaptureSourceProvenanceError(
                "encoded video dimensions must match the decoded PNG sequence"
            )
        decoded = probe_decoded_video(
            video,
            source_png_frames=sorted(
                (Path(path) for path in png_frames), key=lambda path: path.name
            ),
            expected_frame_count=frame_provenance["count"],
            expected_fps=video_fps,
            expected_width=video_width,
            expected_height=video_height,
            png_sequence_digest=frame_provenance["digest"],
        )
        video_digest, video_size_bytes = _artifact_metadata(
            video, "encoded capture video"
        )
        video_provenance = {
            "file": video.name,
            "sha256": video_digest,
            "size_bytes": video_size_bytes,
            **decoded,
        }

    artifact_count = 1 + frame_provenance["count"]
    if metrics_digest is not None:
        artifact_count += 1
    if video_provenance is not None:
        artifact_count += 1
    payload: dict[str, Any] = {
        "artifact_count": artifact_count,
        "png_frames": frame_provenance,
        "scene_metrics_events_sha256": metrics_digest,
        "screenshot_png_frame_binding": screenshot_binding,
        "screenshot_sha256": screenshot_digest,
        "video": video_provenance,
    }
    return {
        "algorithm": CAPTURE_ARTIFACT_PROVENANCE_ALGORITHM,
        "digest": _canonical_json_digest(payload),
        **payload,
    }


def _write_cpp_provenance_header(
    path: Path,
    *,
    provenance: dict[str, Any],
    benchmark_source: Path,
) -> None:
    benchmark_source_sha256 = sha256_file(benchmark_source)
    contents = (
        "// Generated by scripts/capture_source_provenance.py.\n"
        "#pragma once\n\n"
        "#define DART_CAPTURE_SOURCE_PROVENANCE_DIGEST "
        f'"{provenance["digest"]}"\n'
        "#define DART_CAPTURE_SOURCE_GIT_HEAD "
        f'"{provenance["git_head"]}"\n'
        "#define DART_FIGURE13_BENCHMARK_SOURCE_SHA256 "
        f'"{benchmark_source_sha256}"\n'
    )
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        if path.is_file() and path.read_text(encoding="utf-8") == contents:
            return
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            delete=False,
        ) as temporary:
            temporary.write(contents)
            temporary_path = Path(temporary.name)
        temporary_path.replace(path)
    except (OSError, UnicodeError, ValueError) as exc:
        raise CaptureSourceProvenanceError(
            f"cannot write capture source provenance header {path}: {exc}"
        ) from exc


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
    )
    parser.add_argument(
        "--digest-only",
        action="store_true",
        help="print only the capture source digest",
    )
    parser.add_argument(
        "--write-cpp-header",
        type=Path,
        help="write compile-time capture and benchmark source identities",
    )
    parser.add_argument(
        "--benchmark-source",
        type=Path,
        default=(
            Path(__file__).resolve().parents[1]
            / "tests/benchmark/simulation/bm_avbd_rigid_fixed_joint.cpp"
        ),
        help="benchmark translation unit hashed into the generated header",
    )
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)
    if args.digest_only and args.write_cpp_header is not None:
        parser.error("--digest-only and --write-cpp-header are mutually exclusive")
    provenance = compute_capture_source_provenance(args.repo_root)
    if args.write_cpp_header is not None:
        _write_cpp_provenance_header(
            args.write_cpp_header,
            provenance=provenance,
            benchmark_source=args.benchmark_source,
        )
    elif args.digest_only:
        print(provenance["digest"])
    else:
        print(json.dumps(provenance, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
