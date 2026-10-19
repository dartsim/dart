"""PEP 517 backend: scikit-build-core plus the Filament license DART fetches.

scikit-build-core resolves ``project.license-files`` before CMake runs, so the
Filament SDK license that configure fetches cannot join the wheel's dist-info
that way. CMake stages it inside the package (``dartpy/_filament_licenses``)
and this backend moves it to ``dist-info/licenses/filament`` with matching
``License-File`` headers.

``build_wheel`` wraps scikit-build-core's, and ``get_requires_for_build_wheel``
is scikit-build-core's own. ``prepare_metadata_for_build_wheel`` is
deliberately absent: its METADATA could not list the Filament license files,
so frontends read metadata from the built wheel instead. dartpy is wheel-only
(DART 7 decision D20): ``build_sdist`` and ``build_editable`` raise
``UnsupportedOperation``.
"""

from __future__ import annotations

import base64
import csv
import hashlib
import io
import zipfile
from pathlib import Path

from scikit_build_core.build import build_wheel as _build_wheel
from scikit_build_core.build import get_requires_for_build_wheel

__all__ = [
    "UnsupportedOperation",
    "build_editable",
    "build_sdist",
    "build_wheel",
    "get_requires_for_build_wheel",
]

STAGING = "dartpy/_filament_licenses/"
DESTINATION = "licenses/filament/"


class UnsupportedOperation(Exception):
    """PEP 517: the backend cannot perform the requested build."""


def build_wheel(wheel_directory, config_settings=None, metadata_directory=None):
    name = _build_wheel(wheel_directory, config_settings, metadata_directory)
    relocate_filament_licenses(Path(wheel_directory) / name)
    return name


def build_sdist(sdist_directory, config_settings=None):
    raise UnsupportedOperation(
        "dartpy is wheel-only: a source build needs DART's native dependencies, "
        "so no sdist is produced. Build a wheel with `pip wheel .` or "
        "`pixi run -e py312-wheel wheel-build`."
    )


def build_editable(wheel_directory, config_settings=None, metadata_directory=None):
    raise UnsupportedOperation(
        "dartpy does not support editable installs: CMake stages the package "
        "beside the native extension in the build tree. For development, run "
        "`pixi run build-py-dev` and put its python/ build directory on "
        "PYTHONPATH (docs/onboarding/python-bindings.md)."
    )


def _record_row(name: str, data: bytes) -> list[str]:
    digest = base64.urlsafe_b64encode(hashlib.sha256(data).digest()).rstrip(b"=")
    return [name, f"sha256={digest.decode('ascii')}", str(len(data))]


def _add_license_file_headers(metadata: bytes, names: list[str]) -> bytes:
    """Append License-File headers using METADATA's own line ending."""
    first_line = metadata.split(b"\n", 1)[0]
    newline = b"\r\n" if first_line.endswith(b"\r") else b"\n"
    head, separator, body = metadata.partition(newline * 2)
    headers = newline.join(f"License-File: {name}".encode("utf-8") for name in names)
    return head.rstrip(newline) + newline + headers + (separator or newline) + body


def relocate_filament_licenses(wheel: Path) -> list[str]:
    """Move staged Filament license files into dist-info; return their names."""
    with zipfile.ZipFile(wheel) as archive:
        entries = [(info, archive.read(info)) for info in archive.infolist()]
    record_name = next(
        (
            info.filename
            for info, _ in entries
            if info.filename.endswith(".dist-info/RECORD")
        ),
        None,
    )
    if record_name is None:
        raise ValueError(f"{wheel} has no .dist-info/RECORD")
    dist_info = record_name.rpartition("/")[0]
    kept: list[tuple[zipfile.ZipInfo, bytes]] = []
    moved: list[tuple[zipfile.ZipInfo, bytes]] = []
    for info, data in entries:
        if info.filename.startswith(STAGING):
            info.filename = f"{dist_info}/{DESTINATION}{info.filename[len(STAGING):]}"
            moved.append((info, data))
        elif info.filename != record_name:
            kept.append((info, data))
    if not moved:
        return []
    metadata_name = f"{dist_info}/METADATA"
    metadata_index = next(
        (i for i, (info, _) in enumerate(kept) if info.filename == metadata_name), None
    )
    if metadata_index is None:
        raise ValueError(
            f"{wheel} has no {metadata_name} to list the Filament licenses"
        )
    info, data = kept[metadata_index]
    prefix = len(dist_info) + len("/licenses/")
    kept[metadata_index] = (
        info,
        _add_license_file_headers(data, [info.filename[prefix:] for info, _ in moved]),
    )
    record = io.StringIO()
    writer = csv.writer(record, lineterminator="\n")
    writer.writerows(_record_row(info.filename, data) for info, data in kept + moved)
    writer.writerow([record_name, "", ""])
    with zipfile.ZipFile(wheel, "w", zipfile.ZIP_DEFLATED) as archive:
        for info, data in kept + moved:
            archive.writestr(info, data)
        archive.writestr(record_name, record.getvalue())
    return [info.filename for info, _ in moved]
