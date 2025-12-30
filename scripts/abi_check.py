#!/usr/bin/env python3
"""
Run an ABI compatibility check between two refs or the working tree.

This script builds the from/to refs (or working tree for --to) with identical
options, then compares shared libraries with libabigail (abidiff).
"""

from __future__ import annotations

import argparse
import os
import re
import shlex
import shutil
import subprocess
import sys
import tarfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def run(cmd, cwd=None, env=None):
    print(f"+ {' '.join(cmd)}")
    return subprocess.run(cmd, cwd=cwd, env=env, check=True)


def read_version_from_source(src_dir):
    package_xml = Path(src_dir) / "package.xml"
    text = package_xml.read_text(encoding="utf-8")
    match = re.search(r"<version>([^<]+)</version>", text)
    if not match:
        raise RuntimeError(f"Failed to read <version> from {package_xml}")
    return match.group(1)


def select_baseline_tag(major):
    cmd = [
        "git",
        "tag",
        "--list",
        f"v{major}.*",
        "--sort=-v:refname",
    ]
    result = subprocess.run(cmd, cwd=ROOT, check=True, text=True, stdout=subprocess.PIPE)
    for tag in result.stdout.splitlines():
        tag = tag.strip()
        if re.match(rf"^v{major}\.\d+\.\d+$", tag):
            return tag
    return None


def major_from_version(version):
    return version.split(".")[0]


def sanitize_ref(ref_name):
    return re.sub(r"[^A-Za-z0-9._-]+", "_", ref_name)


def resolve_ref(ref_name):
    result = subprocess.run(
        ["git", "rev-parse", "--verify", f"{ref_name}^{{commit}}"],
        cwd=ROOT,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if result.returncode != 0:
        return None
    commit = result.stdout.strip()
    return commit


def list_refs(pattern):
    cmd = [
        "git",
        "for-each-ref",
        "--sort=refname",
        "--format=%(refname:short)",
        "refs/heads",
        "refs/remotes",
        "refs/tags",
    ]
    result = subprocess.run(cmd, cwd=ROOT, check=True, text=True, stdout=subprocess.PIPE)
    refs = [line.strip() for line in result.stdout.splitlines() if line.strip()]
    refs = [ref for ref in refs if not ref.endswith("/HEAD")]
    if pattern:
        regex = re.compile(pattern)
        refs = [ref for ref in refs if regex.search(ref)]
    for ref in refs:
        print(ref)


def extract_ref(ref_name, dest_dir):
    if dest_dir.exists():
        shutil.rmtree(dest_dir)
    dest_dir.mkdir(parents=True)

    cmd = ["git", "archive", "--format=tar", ref_name]
    proc = subprocess.Popen(cmd, cwd=ROOT, stdout=subprocess.PIPE)
    try:
        with tarfile.open(fileobj=proc.stdout, mode="r|*") as tar:
            tar.extractall(dest_dir)
    finally:
        if proc.stdout:
            proc.stdout.close()
    if proc.wait() != 0:
        raise RuntimeError(f"git archive failed for ref {ref_name}")


def prepare_source(ref_name, ref_commit, work_dir, label):
    short_commit = ref_commit[:12]
    ref_id = f"{sanitize_ref(ref_name)}-{short_commit}"
    ref_root = work_dir / f"{label}-{ref_id}"
    src_dir = ref_root / "src"
    marker = ref_root / "ref.txt"
    marker_value = f"{ref_name}\n{ref_commit}\n"
    if src_dir.exists() and marker.exists() and marker.read_text(encoding="utf-8") == marker_value:
        return src_dir

    ref_root.mkdir(parents=True, exist_ok=True)
    extract_ref(ref_name, src_dir)
    marker.write_text(marker_value, encoding="utf-8")
    return src_dir


def cmake_configure(src_dir, build_dir, build_type, prefix, extra_cmake_args):
    build_dir.mkdir(parents=True, exist_ok=True)
    cmake_cmd = [
        "cmake",
        "-G",
        "Ninja",
        "-S",
        str(src_dir),
        "-B",
        str(build_dir),
        f"-DCMAKE_BUILD_TYPE={build_type}",
        f"-DCMAKE_INSTALL_PREFIX={prefix}",
        f"-DCMAKE_PREFIX_PATH={prefix}",
        "-DBUILD_SHARED_LIBS=ON",
        "-DDART_BUILD_DARTPY=OFF",
        "-DDART_BUILD_DART8=OFF",
        "-DDART_BUILD_GUI=OFF",
        "-DDART_BUILD_GUI_OSG=OFF",
        "-DDART_BUILD_GUI_RAYLIB=OFF",
        "-DDART_BUILD_EXAMPLES=OFF",
        "-DDART_BUILD_TUTORIALS=OFF",
        "-DDART_BUILD_TESTS=OFF",
        "-DDART_BUILD_PROFILE=OFF",
        "-DDART_PROFILE_BUILTIN=OFF",
        "-DDART_PROFILE_TRACY=OFF",
        "-DDART_BUILD_COLLISION_BULLET=ON",
        "-DDART_BUILD_COLLISION_ODE=ON",
        "-DDART_USE_SYSTEM_GOOGLEBENCHMARK=ON",
        "-DDART_USE_SYSTEM_GOOGLETEST=ON",
        "-DDART_USE_SYSTEM_IMGUI=ON",
        "-DDART_USE_SYSTEM_TRACY=ON",
        "-DDART_VERBOSE=OFF",
    ]

    if extra_cmake_args:
        cmake_cmd.extend(extra_cmake_args)

    run(cmake_cmd)


def cmake_build(build_dir, targets, jobs):
    cmd = ["cmake", "--build", str(build_dir)]
    if targets:
        cmd.append("--target")
        cmd.extend(targets)
    if jobs:
        cmd.extend(["-j", str(jobs)])
    run(cmd)


def find_library(build_dir, lib_name):
    if not lib_name.startswith("lib"):
        lib_name = f"lib{lib_name}"
    lib_dir = build_dir / "lib"
    exact = lib_dir / f"{lib_name}.so"
    if exact.exists():
        return exact
    candidates = sorted(lib_dir.glob(f"{lib_name}.so.*"))
    if candidates:
        return candidates[0]

    # Fallback: search the entire build tree for out-of-tree lib outputs.
    fallback = sorted(build_dir.rglob(f"{lib_name}.so"))
    if fallback:
        return fallback[0]
    fallback = sorted(build_dir.rglob(f"{lib_name}.so.*"))
    if fallback:
        return fallback[0]

    raise FileNotFoundError(
        f"Unable to find {lib_name}.so under {build_dir}"
    )


def run_abidiff(baseline_lib, current_lib, suppressions):
    cmd = ["abidiff", "--no-added-syms", "--fail-no-debug-info"]
    if suppressions:
        cmd.extend(["--suppressions", suppressions])
    cmd.extend([str(baseline_lib), str(current_lib)])
    print(f"+ {' '.join(cmd)}")
    return subprocess.run(cmd, check=False).returncode


def main():
    parser = argparse.ArgumentParser(description="Run ABI compatibility checks")
    parser.add_argument(
        "--from",
        dest="from_ref",
        default=os.environ.get("DART_ABI_FROM", ""),
        help="From git ref (tag/branch/commit). Defaults to latest v<major>.<minor>.<patch> tag",
    )
    parser.add_argument(
        "--to",
        dest="to_ref",
        default=os.environ.get("DART_ABI_TO", ""),
        help="To git ref (tag/branch/commit). Defaults to working tree",
    )
    parser.add_argument(
        "--build-type",
        default=os.environ.get("DART_ABI_BUILD_TYPE", "RelWithDebInfo"),
        help="CMake build type for ABI checks",
    )
    parser.add_argument(
        "--libs",
        default=os.environ.get("DART_ABI_LIBS", "dart"),
        help="Comma-separated list of libraries to check (default: dart)",
    )
    parser.add_argument(
        "--work-dir",
        default=os.environ.get("DART_ABI_WORK_DIR", str(ROOT / "build" / "abi")),
        help="Working directory for ABI builds",
    )
    parser.add_argument(
        "--require-baseline",
        action="store_true",
        default=os.environ.get("DART_ABI_REQUIRE_BASELINE", "OFF") == "ON",
        help="Fail if no from tag is found",
    )
    parser.add_argument(
        "--suppressions",
        default=os.environ.get("DART_ABI_SUPPRESSIONS", ""),
        help="Path to abidiff suppressions file",
    )
    parser.add_argument(
        "--allow-cross-major",
        action="store_true",
        default=os.environ.get("DART_ABI_ALLOW_CROSS_MAJOR", "OFF") == "ON",
        help="Allow comparing from/to refs with different major versions",
    )
    parser.add_argument(
        "--list-refs",
        action="store_true",
        default=False,
        help="List available refs and exit",
    )
    parser.add_argument(
        "--list-pattern",
        default=os.environ.get("DART_ABI_LIST_PATTERN", ""),
        help="Regex filter for --list-refs output",
    )

    args = parser.parse_args()

    if not sys.platform.startswith("linux"):
        print("ABI checks are only supported on Linux with libabigail.")
        return 0

    if args.list_refs:
        list_refs(args.list_pattern)
        return 0

    if not shutil.which("abidiff"):
        print("abidiff not found. Install libabigail or set PATH.")
        return 1

    prefix = os.environ.get("DART_ABI_PREFIX", os.environ.get("CONDA_PREFIX", ""))
    if not prefix:
        print("CONDA_PREFIX is not set; run via pixi or set DART_ABI_PREFIX.")
        return 1

    work_dir = Path(args.work_dir)

    if args.to_ref:
        current_commit = resolve_ref(args.to_ref)
        if not current_commit:
            print(f"To ref not found: {args.to_ref}")
            return 1
        current_src = prepare_source(args.to_ref, current_commit, work_dir, "to")
        to_label = f"{sanitize_ref(args.to_ref)}-{current_commit[:12]}"
    else:
        current_src = ROOT
        to_label = "worktree"

    current_version = read_version_from_source(current_src)
    current_major = major_from_version(current_version)

    from_ref = args.from_ref
    if from_ref:
        baseline_commit = resolve_ref(from_ref)
        if not baseline_commit:
            print(f"From ref not found: {from_ref}")
            return 1
    else:
        from_ref = select_baseline_tag(current_major)
        baseline_commit = resolve_ref(from_ref) if from_ref else None

    if not from_ref or not baseline_commit:
        msg = f"No from tag found for major {current_major}. Set --from or DART_ABI_FROM."
        if args.require_baseline:
            print(msg)
            return 1
        print(f"{msg} Skipping ABI check.")
        return 0

    baseline_src = prepare_source(from_ref, baseline_commit, work_dir, "from")
    baseline_version = read_version_from_source(baseline_src)
    baseline_major = major_from_version(baseline_version)
    if baseline_major != current_major and not args.allow_cross_major:
        print(
            "From major does not match to major. "
            "Set DART_ABI_ALLOW_CROSS_MAJOR=ON or pass --allow-cross-major to override."
        )
        return 1

    baseline_build = work_dir / f"from-{sanitize_ref(from_ref)}" / "build"
    current_build = work_dir / f"to-{to_label}" / "build"

    extra_args = shlex.split(os.environ.get("DART_ABI_CMAKE_ARGS", ""))
    libs = [lib.strip() for lib in args.libs.split(",") if lib.strip()]
    targets = libs

    print(f"From ref: {from_ref}")
    print(f"To ref: {args.to_ref or 'WORKTREE'}")
    print(f"Build type: {args.build_type}")
    print(f"Libraries: {', '.join(libs)}")

    cmake_configure(baseline_src, baseline_build, args.build_type, prefix, extra_args)
    try:
        cmake_build(baseline_build, targets, os.environ.get("DART_PARALLEL_JOBS"))
    except subprocess.CalledProcessError as exc:
        print(
            f"Failed to build targets ({', '.join(targets)}) for from ref {from_ref}. "
            "Check that the targets exist in this ref."
        )
        return exc.returncode

    cmake_configure(current_src, current_build, args.build_type, prefix, extra_args)
    try:
        cmake_build(current_build, targets, os.environ.get("DART_PARALLEL_JOBS"))
    except subprocess.CalledProcessError as exc:
        print(
            f"Failed to build targets ({', '.join(targets)}) for to ref {args.to_ref or 'WORKTREE'}. "
            "Check that the targets exist in this ref."
        )
        return exc.returncode

    exit_code = 0
    for lib in libs:
        try:
            baseline_lib = find_library(baseline_build, lib)
        except FileNotFoundError as exc:
            print(f"From ref {from_ref}: {exc}")
            return 1
        try:
            current_lib = find_library(current_build, lib)
        except FileNotFoundError as exc:
            print(f"To ref {args.to_ref or 'WORKTREE'}: {exc}")
            return 1
        result = run_abidiff(baseline_lib, current_lib, args.suppressions)
        if result != 0:
            exit_code = result

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
