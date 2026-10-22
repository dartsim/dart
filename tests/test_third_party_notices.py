"""Keep tracked foreign copyright declarations covered by the shipped NOTICE.

The scan misses provenance stated as "based on" or "licensed under" without
a copyright line, and bare "(c) 2014 holder" or "(c)" without the word Copyright.
Non-UTF-8 text may lose declarations during decoding. Binary and compressed
files are covered only by explicit NOTICE globs, not declaration scanning.
"""

import fnmatch
import hashlib
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
COPYRIGHT = "Copyright"
NATIVE_HOLDERS = (
    "the dart development contributors",
    "georgia tech graphics lab",
    "georgia tech research corporation",
    "humanoid robotics lab",
)
DECLARATION = re.compile(
    r"(?:(?i:\bSPDX-FileCopyrightText:)\s*"
    r"|©\s*"
    r"|(?i:\bcopyright\b)\s*(?::\s*|\([cC]\)\s*|©\s*|(?=\d|[A-Z]|the\b)))"
    r"(?P<holders>.+)"
)
ENTRY_SEPARATOR = "-" * 72 + "\n"
COMPILED_IN = "none (compiled into DART binaries)"
# The fixture is never installed. NUL-containing files and compressed/archive
# formats are not text inventories, even when an archive happens to lack NULs.
EXCLUDED_PATHS = {"LICENSE", "NOTICE", "tests/cmake/filament_notices/LICENSE"}
COMPRESSED_SUFFIXES = {
    ".gz",
    ".bz2",
    ".xz",
    ".lzma",
    ".zst",
    ".lz4",
    ".zip",
    ".7z",
    ".rar",
    ".tar",
    ".tgz",
    ".tbz",
    ".tbz2",
    ".txz",
    ".tzst",
}

# These derivatives have no foreign copyright header; dependency-only entries
# are checked by name because their code arrives at build time.
KNOWN_SOURCES = {
    "libccd": "dart/collision/dart/narrow_phase/Mpr.cpp",
    "Tomas Moller triangle intersection": (
        "dart/collision/fcl/tri_tri_intersection_test.hpp"
    ),
    "computational-geometry-toolbox": "dart/math/detail/ConvexHull.hpp",
    # ConvexHull-impl.hpp is covered only by the ConvexHull glob.
    "Dear ImGui": None,
    "tsl::robin_map": None,
    "Eigen": None,
    "Atlas v3 sample model and retained Atlas textures": (
        "data/sdf/atlas/atlas_v3.urdf"
    ),
    "WAM sample model (herb_description BSD files)": "data/urdf/wam/wam.urdf",
}


def entries(notice):
    return [
        section.rstrip()
        for section in notice.split(ENTRY_SEPARATOR)
        if section.startswith("Project: ")
    ]


def patterns(section):
    return re.findall(r"^Files: (.+)$", section, re.MULTILINE)


def normalized(text):
    return re.sub(r"[^a-z0-9]+", " ", text.lower()).strip()


def foreign_declarations(text):
    for line in text.splitlines():
        for match in DECLARATION.finditer(line):
            holders = match.group("holders").rstrip(" */")
            # Yearless declarations start a line/comment, rather than prose.
            if not holders[:1].isdigit() and re.search(
                r"[a-zA-Z]", line[: match.start()]
            ):
                continue
            # License conditions/disclaimers are not holder declarations.
            if re.match(
                r"(?i:holders?\b|owners?\b|notice\b|license\b|headers?\b)", holders
            ):
                continue
            for holder in NATIVE_HOLDERS:
                holders = re.sub(re.escape(holder), "", holders, flags=re.IGNORECASE)
            remaining = re.sub(
                r"(?i:\band\b|\ball rights reserved\b|\bcopyright\b)" r"|\([cC]\)",
                "",
                holders,
            )
            if re.search(r"[a-zA-Z]", remaining):
                yield match.group().rstrip(" */")


def tracked_paths(root=ROOT):
    try:
        top = subprocess.check_output(
            ["git", "rev-parse", "--show-toplevel"], cwd=root, stderr=subprocess.DEVNULL
        )
        if Path(top.decode().strip()).resolve() != root.resolve():
            pytest.skip("Notice inventory requires the source Git checkout")
        return (
            subprocess.check_output(["git", "ls-files", "-z"], cwd=root)
            .decode()
            .split("\0")[:-1]
        )
    except (FileNotFoundError, subprocess.CalledProcessError):
        pytest.skip("Notice inventory requires Git (not available in an sdist)")


def inventory(paths, root=ROOT):
    declarations = []
    for path in paths:
        file = root / path
        if path in EXCLUDED_PATHS or file.suffix.lower() in COMPRESSED_SUFFIXES:
            continue
        # Deleted/unreadable tracked files, symlinks and submodules are not
        # readable source files in this working tree.
        try:
            if file.is_symlink() or not file.is_file():
                continue
            contents = file.read_bytes()
        except OSError:
            continue
        if b"\0" in contents:
            continue
        text = contents.decode("utf-8", errors="replace")
        declarations.extend((path, line) for line in foreign_declarations(text))
    return declarations


def uncovered(notice, declarations):
    sections = entries(notice)
    missing = []
    for path, declaration in declarations:
        if not any(
            any(fnmatch.fnmatchcase(path, pattern) for pattern in patterns(section))
            and normalized(declaration) in normalized(section)
            for section in sections
        ):
            missing.append(f"{path}: {declaration}")
    for project, path in KNOWN_SOURCES.items():
        if not any(
            section.splitlines()[0] == f"Project: {project}"
            and (
                patterns(section) == [COMPILED_IN]
                if path is None
                else any(fnmatch.fnmatchcase(path, p) for p in patterns(section))
            )
            for section in sections
        ):
            missing.append(f"{path or project}: missing {project} entry")
    return missing


def unmatched_patterns(notice, paths):
    return [
        pattern
        for section in entries(notice)
        for pattern in patterns(section)
        if pattern != COMPILED_IN
        and not any(fnmatch.fnmatchcase(path, pattern) for path in paths)
    ]


@pytest.fixture(scope="module")
def paths():
    return tracked_paths()


@pytest.fixture(scope="module")
def declarations(paths):
    return inventory(paths)


def test_tracked_foreign_copyrights_have_notices(declarations):
    assert declarations, "The tracked copyright inventory must not be empty"
    missing = uncovered((ROOT / "NOTICE").read_text(encoding="utf-8"), declarations)
    assert not missing, "Missing third-party notices:\n" + "\n".join(missing)


def test_notice_globs_match_tracked_paths(paths):
    assert not unmatched_patterns((ROOT / "NOTICE").read_text(encoding="utf-8"), paths)


@pytest.mark.parametrize(
    "path, sha256",
    [
        (
            "data/sdf/atlas/LICENSE",
            "cfc7749b96f63bd31c3c42b5c471bf756814053e847c10f3eb003417bc523d30",
        ),
        (
            "data/urdf/wam/LICENSE.bsd",
            "a8fed4eab54a94f01ba8f721c883efa928da5ae3f62c7528b9bee2a24028d31c",
        ),
    ],
)
def test_sample_model_license_matches_upstream(path, sha256):
    # Windows checkouts may convert the upstream LF line endings to CRLF.
    text = (ROOT / path).read_bytes().replace(b"\r\n", b"\n")
    assert hashlib.sha256(text).hexdigest() == sha256


def test_wam_distributes_only_bsd_meshes():
    model = ROOT / "data/urdf/wam"
    meshes = sorted(path for path in (model / "meshes").rglob("*") if path.is_file())
    assert len(meshes) == 16
    assert all(path.suffix == ".STL" for path in meshes)
    robot = ET.parse(model / "wam.urdf").getroot()
    visuals = robot.findall("link/visual/geometry/mesh")
    assert {Path(mesh.attrib["filename"]).name for mesh in visuals} == {
        "wam_base.STL",
        *(f"wam{i}.STL" for i in range(1, 8)),
    }
    for mesh in robot.findall(".//mesh"):
        relative = mesh.attrib["filename"].removeprefix("package://herb_description/")
        assert (model / relative).is_file()


@pytest.mark.parametrize(
    "section",
    entries((ROOT / "NOTICE").read_text(encoding="utf-8")),
    ids=lambda section: section.splitlines()[0].removeprefix("Project: "),
)
def test_removing_any_notice_entry_fails(section, declarations):
    notice = (ROOT / "NOTICE").read_text(encoding="utf-8")
    assert uncovered(notice.replace(section, "", 1), declarations)


@pytest.mark.parametrize("prefix", ["// ", " * ", "# ", "<!-- ", ""])
@pytest.mark.parametrize(
    "declaration",
    [
        f"{COPYRIGHT} (c) 2026 Example Corporation",
        "\N{COPYRIGHT SIGN} 2026 Example Corporation",
        f"SPDX-File{COPYRIGHT}Text: 2026 Example Corporation",
        f"{COPYRIGHT}: Example Corporation",
        f"{COPYRIGHT} Example Corporation",
        f"{COPYRIGHT} (c) 2026 The DART development contributors and Example Corporation",
        f"{COPYRIGHT} (c) 2026 Example Corporation and The DART development contributors",
    ],
)
@pytest.mark.parametrize(
    "path",
    ["dart/simulation/World.cpp", "dart/collision/dart/narrow_phase/Mpr.cpp"],
)
def test_new_copyright_in_an_already_covered_path_is_rejected(
    prefix, declaration, path
):
    declarations = [(path, d) for d in foreign_declarations(prefix + declaration)]
    assert declarations
    missing = uncovered((ROOT / "NOTICE").read_text(encoding="utf-8"), declarations)
    assert any("Example Corporation" in item for item in missing)


@pytest.mark.parametrize(
    "declaration",
    [
        f"SPDX-File{COPYRIGHT}Text: example@example.com",
        f"{COPYRIGHT}: example@example.com",
        f"{COPYRIGHT} (c) 2026 example@example.com",
    ],
)
def test_explicit_declarations_allow_lowercase_holders(declaration):
    assert list(foreign_declarations(declaration)) == [declaration]


def test_prose_is_not_a_holder_declaration():
    assert not list(foreign_declarations(f"Update {COPYRIGHT.lower()}: added a holder"))


@pytest.mark.parametrize("holder", NATIVE_HOLDERS)
def test_native_holders_are_exempt(holder):
    assert not list(foreign_declarations(f"{COPYRIGHT} (c) 2026 {holder}"))


def test_license_body_project_label_does_not_start_entry():
    notice = (
        ENTRY_SEPARATOR + "Project: Example\nFiles: example.cpp\n"
        "Source: example\nLicense body\nProject: part of the license\nMore license\n"
        + ENTRY_SEPARATOR
        + "License text: MIT\nShared text\n"
    )
    assert entries(notice) == [
        "Project: Example\nFiles: example.cpp\nSource: example\n"
        "License body\nProject: part of the license\nMore license"
    ]


def test_unmatched_glob_is_rejected():
    notice = ENTRY_SEPARATOR + "Project: Example\nFiles: missing/*\n"
    assert unmatched_patterns(notice, ["present.cpp"]) == ["missing/*"]
    assert not unmatched_patterns(notice.replace("missing/*", COMPILED_IN), [])


def test_inventory_skips_non_sources(tmp_path, monkeypatch):
    paths = ["missing", "submodule", "binary", "archive.gz", "unreadable", "source.cpp"]
    (tmp_path / "submodule").mkdir()
    line = f"{COPYRIGHT} (c) 2026 Example Corporation"
    (tmp_path / "binary").write_bytes(b"\0" + line.encode())
    (tmp_path / "archive.gz").write_text(line, encoding="utf-8")
    (tmp_path / "unreadable").touch()
    (tmp_path / "source.cpp").write_text(line, encoding="utf-8")
    read_bytes = Path.read_bytes

    def read(path):
        if path.name == "unreadable":
            raise PermissionError("Unreadable fixture")
        return read_bytes(path)

    monkeypatch.setattr(Path, "read_bytes", read)
    assert inventory(paths, tmp_path) == [("source.cpp", line)]


def test_inventory_skips_symlinks(tmp_path):
    (tmp_path / "source.cpp").write_text(
        f"{COPYRIGHT} (c) 2026 Example Corporation", encoding="utf-8"
    )
    try:
        (tmp_path / "link.cpp").symlink_to("source.cpp")
    except OSError:
        pytest.skip("Symlinks unavailable on this platform")
    assert not inventory(["link.cpp"], tmp_path)


def test_inventory_skips_never_installed_fixture():
    assert not inventory(["tests/cmake/filament_notices/LICENSE"])


def test_tracked_paths_skips_without_git(tmp_path):
    with pytest.raises(pytest.skip.Exception, match="Git|checkout"):
        tracked_paths(tmp_path)


def test_wheel_metadata_and_license_files_include_notice(tmp_path):
    source = tmp_path / "source"
    source.mkdir()
    for name in ("pyproject.toml", "README.md", "package.xml", "LICENSE", "NOTICE"):
        shutil.copy2(ROOT / name, source / name)
    backend = source / "scripts/pep517"
    backend.mkdir(parents=True)
    shutil.copy2(ROOT / "scripts/pep517/dartpy_backend.py", backend)
    # Exercise the real wheel configuration without compiling native code.
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.26)\n"
        "project(NoticeFixture NONE)\n"
        "add_custom_target(dartpy)\n",
        encoding="utf-8",
    )
    source_entries = set(ROOT.iterdir())
    python_entries = set((ROOT / "python").iterdir())
    subprocess.run(
        [
            sys.executable,
            "-c",
            "import sys; sys.path.insert(0, 'scripts/pep517'); "
            "from dartpy_backend import build_wheel; "
            "build_wheel('dist')",
        ],
        cwd=source,
        check=True,
        capture_output=True,
        text=True,
    )
    wheel = next((source / "dist").glob("*.whl"))
    assert "-cp312-abi3-" in wheel.name
    with zipfile.ZipFile(wheel) as archive:
        metadata_path = next(
            name for name in archive.namelist() if name.endswith(".dist-info/METADATA")
        )
        dist_info = metadata_path.rpartition("/")[0]
        metadata = archive.read(metadata_path).decode("utf-8").replace("\r\n", "\n")
        for name in ("LICENSE", "NOTICE"):
            assert f"License-File: {name}\n" in metadata
            assert (
                archive.read(f"{dist_info}/licenses/{name}")
                == (ROOT / name).read_bytes()
            )
    assert set(ROOT.iterdir()) == source_entries
    assert set((ROOT / "python").iterdir()) == python_entries


@pytest.mark.parametrize("component", [None, "headers"])
def test_notice_is_installed_with_libraries_and_headers(tmp_path, component):
    source = tmp_path / "source"
    source.mkdir()
    for name in ("LICENSE", "NOTICE"):
        shutil.copy2(ROOT / name, source / name)
    rules = (
        (ROOT / "CMakeLists.txt")
        .read_text()
        .split("# Use the same default install component as libdart.", 1)[1]
        .split("# Variables used in Components.cmake", 1)[0]
    )
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.22.1)\n"
        "project(NoticeInstall NONE)\n"
        "include(GNUInstallDirs)\n" + rules
    )
    build, prefix = tmp_path / "build", tmp_path / "prefix"
    subprocess.run(
        ["cmake", "-S", str(source), "-B", str(build)],
        check=True,
        capture_output=True,
        text=True,
    )
    command = ["cmake", "--install", str(build), "--prefix", str(prefix)]
    if component:
        command += ["--component", component]
    subprocess.run(command, check=True, capture_output=True, text=True)
    for name in ("LICENSE", "NOTICE"):
        assert (prefix / "share/licenses/dart" / name).read_bytes() == (
            ROOT / name
        ).read_bytes()
