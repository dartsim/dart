#!/usr/bin/env python3
"""
Comprehensive testing script for DART before submitting a PR.

This script runs all available tests locally:
- Linting (C++ and Python)
- Building (Release and Debug)
- Unit tests
- Python tests
- Documentation build

Usage:
    python scripts/test_all.py [--skip-build] [--skip-tests] [--skip-lint] [--skip-docs]
"""

import argparse
import os
import subprocess
import sys
from pathlib import Path
from shutil import which
from typing import Dict, Optional, Tuple
from urllib.request import urlopen

from build_helpers import cmake_target_exists, get_build_dir


def supports_unicode() -> bool:
    """Check if the terminal supports Unicode characters"""
    try:
        # Check if stdout encoding supports Unicode
        encoding = sys.stdout.encoding
        if encoding and encoding.lower() in ["utf-8", "utf8"]:
            return True
        # Try to encode a Unicode character
        "\u2713".encode(encoding or "utf-8")
        return True
    except (UnicodeEncodeError, LookupError):
        return False


# Detect Unicode support
USE_UNICODE = supports_unicode()


class Symbols:
    """Terminal symbols with Unicode/ASCII fallback"""

    ARROW = "\u25b6" if USE_UNICODE else ">"
    CHECK = "\u2713" if USE_UNICODE else "[OK]"
    CROSS = "\u2717" if USE_UNICODE else "[ERROR]"
    WARNING = "\u26a0" if USE_UNICODE else "[WARN]"
    SPARKLES = "\u2728" if USE_UNICODE else ""
    ROCKET = "\U0001f680" if USE_UNICODE else ""


PIXI_DEFAULT_DARTPY = "ON"
ROOT_DIR = Path(__file__).resolve().parent.parent


def _resolve_pixi_path() -> Optional[str]:
    """Pick the pixi binary from env override or PATH on demand."""
    return os.environ.get("PIXI_BIN") or which("pixi")


PIXI_BIN = _resolve_pixi_path()


def _env_flag_enabled(name: str, default: str = "ON") -> bool:
    """Helper to treat ON/OFF/0/1 env values as booleans."""
    value = os.environ.get(name, default)
    if value is None:
        return True
    return value.upper() not in {"OFF", "0", "FALSE"}


def _cmake_option_enabled(option: str) -> Optional[bool]:
    """Return bool if option present in CMakeCache, otherwise None."""
    env_name = os.environ.get("PIXI_ENVIRONMENT_NAME", "default")
    build_type = os.environ.get("BUILD_TYPE", "Release")
    cache_path = ROOT_DIR / "build" / env_name / "cpp" / build_type / "CMakeCache.txt"
    if not cache_path.is_file():
        return None

    needle = f"{option}:BOOL="
    with cache_path.open("r", encoding="utf-8", errors="ignore") as cache:
        for line in cache:
            if line.startswith(needle):
                value = line.strip().split("=", maxsplit=1)[-1].upper()
                return value == "ON"
    return None


def pixi_command(task: str, *args: str) -> str:
    pixi_exe = _resolve_pixi_path()
    if pixi_exe is None:
        return f"pixi run {task} {' '.join(args)}".strip()

    joined_args = f" {' '.join(args)}" if args else ""
    return f"{pixi_exe} run {task}{joined_args}"


class Colors:
    """ANSI color codes for terminal output"""

    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKCYAN = "\033[96m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"


def print_header(message: str):
    """Print a formatted header"""
    print(f"\n{Colors.HEADER}{Colors.BOLD}{'=' * 80}{Colors.ENDC}")
    print(f"{Colors.HEADER}{Colors.BOLD}{message}{Colors.ENDC}")
    print(f"{Colors.HEADER}{Colors.BOLD}{'=' * 80}{Colors.ENDC}\n")


def print_step(message: str):
    """Print a formatted step"""
    print(f"{Colors.OKBLUE}{Symbols.ARROW} {message}{Colors.ENDC}")


def print_success(message: str):
    """Print a success message"""
    print(f"{Colors.OKGREEN}{Symbols.CHECK} {message}{Colors.ENDC}")


def print_error(message: str):
    """Print an error message"""
    print(f"{Colors.FAIL}{Symbols.CROSS} {message}{Colors.ENDC}")


def print_warning(message: str):
    """Print a warning message"""
    print(f"{Colors.WARNING}{Symbols.WARNING} {message}{Colors.ENDC}")


def run_command(
    cmd: str,
    description: str,
    stream_output: bool = True,
    env: Optional[Dict[str, str]] = None,
) -> Tuple[bool, str]:
    """
    Run a command and return success status and output.

    Args:
        cmd: Command to run
        description: Description of the command for logging
        stream_output: If True, stream output in real-time; if False, capture and return

    Returns:
        Tuple of (success: bool, output: str)
    """
    print_step(f"Running: {description}")
    print(f"  Command: {cmd}")
    print()  # Add blank line for readability

    env_vars = os.environ.copy()
    if env:
        env_vars.update(env)

    try:
        if stream_output:
            # Stream output in real-time
            process = subprocess.Popen(
                cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                universal_newlines=True,
                env=env_vars,
            )

            output_lines = []
            if process.stdout:
                for line in process.stdout:
                    print(line, end="")  # Print line in real-time
                    output_lines.append(line)

            returncode = process.wait()
            output = "".join(output_lines)

            if returncode == 0:
                print()  # Add blank line
                print_success(f"{description} - PASSED")
                return True, output
            else:
                print()  # Add blank line
                print_error(f"{description} - FAILED")
                print(f"  Return code: {returncode}")
                return False, output
        else:
            # Capture output without streaming
            result = subprocess.run(
                cmd,
                shell=True,
                capture_output=True,
                text=True,
                check=False,
                env=env_vars,
            )

            if result.returncode == 0:
                print_success(f"{description} - PASSED")
                return True, result.stdout
            else:
                print_error(f"{description} - FAILED")
                print(f"  Return code: {result.returncode}")
                if result.stderr:
                    print(f"  Error output:\n{result.stderr}")
                if result.stdout:
                    print(f"  Standard output:\n{result.stdout}")
                return False, result.stderr + "\n" + result.stdout
    except Exception as e:
        print_error(f"{description} - EXCEPTION: {e}")
        return False, str(e)


def patch_nanobind_if_needed() -> None:
    """Work around corrupted nanobind headers in some environments.

    A few conda builds of nanobind 2.9.2 ship with missing escape sequences in
    nanobind sources, which makes compilation fail with missing-terminating-string
    errors. If we detect the broken patterns, rewrite them in-place to match the
    upstream source.
    """

    try:
        import nanobind  # type: ignore
    except Exception:
        return

    nb_root = Path(nanobind.__file__).resolve().parent
    nb_type = nb_root / "src" / "nb_type.cpp"
    nb_func = nb_root / "src" / "nb_func.cpp"
    nb_error = nb_root / "src" / "error.cpp"
    nb_version = getattr(nanobind, "__version__", "main")
    base_url = f"https://raw.githubusercontent.com/wjakob/nanobind/v{nb_version}/src/"

    force_refresh = nb_version.startswith("2.9.2")

    def refresh_from_upstream(
        path: Path, filename: str, bad_markers: Tuple[str, ...], force: bool = False
    ) -> bool:
        if not path.is_file():
            return False

        text = path.read_text(encoding="utf-8")
        if not force and not any(marker in text for marker in bad_markers):
            return False

        try:
            upstream = urlopen(base_url + filename, timeout=10).read().decode("utf-8")
            path.write_text(upstream, encoding="utf-8")
            print_warning(f"Refreshed nanobind source at {path} from upstream.")
            return True
        except Exception:
            return False

    refreshed = False
    refreshed |= refresh_from_upstream(
        nb_type, "nb_type.cpp", ('\\"%s")', '(")', '("['), force=force_refresh
    )
    refreshed |= refresh_from_upstream(
        nb_func,
        "nb_func.cpp",
        ('\\"%s")', 'buf.put(" = \\");', 'buf.put(" = ");'),
        force=force_refresh,
    )
    refreshed |= refresh_from_upstream(
        nb_error,
        "error.cpp",
        ('buf.put("", line ");', 'buf.put("\\\\", line ");'),
        force=force_refresh,
    )

    if refreshed:
        return

    # Fallback: minimal in-place fixes when network refresh fails.
    if nb_type.is_file():
        text = nb_type.read_text(encoding="utf-8")
        fixed = (
            text.replace('\\"%s")', '\\"%s\\")')
            .replace('(\\"(")', '(\\"(\\")')
            .replace('(\\"[")', '(\\"[\\")')
        )
        if fixed != text:
            nb_type.write_text(fixed, encoding="utf-8")
            print_warning(f"Patched nanobind source at {nb_type} to fix bad escapes.")

    if nb_func.is_file():
        func_text = nb_func.read_text(encoding="utf-8")
        func_fixed = (
            func_text.replace(
                '\\"%s"): function not found!', '\\"%s\\"): function not found!'
            )
            .replace('buf.put(" = \\");', 'buf.put(" = \\\\");')
            .replace('buf.put(" = ");', 'buf.put(" = \\\\");')
        )
        if func_fixed != func_text:
            nb_func.write_text(func_fixed, encoding="utf-8")
            print_warning(f"Patched nanobind source at {nb_func} to fix bad escapes.")

    if nb_error.is_file():
        err_text = nb_error.read_text(encoding="utf-8")
        err_fixed = err_text.replace(
            '            buf.put("", line ");',
            '            buf.put("\\", line ");',
        ).replace(
            '            buf.put("\\\\", line ");', '            buf.put("\\", line ");'
        )
        if err_fixed != err_text:
            nb_error.write_text(err_fixed, encoding="utf-8")
            print_warning(f"Patched nanobind source at {nb_error} to fix bad escapes.")


def check_pixi() -> bool:
    """Check if pixi is available"""
    pixi_exe = _resolve_pixi_path()
    if pixi_exe is None:
        print_error("pixi not found. Please install pixi first.")
        return False

    try:
        subprocess.run([pixi_exe, "--version"], capture_output=True, check=True)
        return True
    except subprocess.CalledProcessError:
        print_error("pixi not found. Please install pixi first.")
        return False


def run_lint_tests() -> bool:
    """Run linting (auto-fix formatting issues)"""
    print_header("LINTING (Auto-fixing)")

    # Run all linting tasks (C++, Python, YAML)
    result, _ = run_command(pixi_command("lint"), "Auto-fix formatting (all languages)")

    return result


def run_build_tests(skip_debug: bool = False) -> bool:
    """Run build tests"""
    print_header("BUILD")

    success = True

    # Build Release
    result, _ = run_command(pixi_command("build"), "Build Release")
    success = success and result

    if not skip_debug:
        # Build Debug (for better error messages)
        result, _ = run_command(pixi_command("build-debug"), "Build Debug")
        success = success and result

    return success


def run_unit_tests() -> bool:
    """Run unit tests"""
    print_header("UNIT TESTS")

    success = True

    # Build and run C++ tests
    result, _ = run_command(pixi_command("test", PIXI_DEFAULT_DARTPY), "C++ unit tests")
    success = success and result

    return success


def run_dart8_tests() -> bool:
    """Run dart8-specific tests (ctest filtered to dart8 labels)."""
    print_header("DART8 TESTS")

    if not _env_flag_enabled("DART_BUILD_DART8_OVERRIDE", "ON"):
        print_warning("Skipping dart8 tests because DART_BUILD_DART8_OVERRIDE is OFF")
        return True

    cmake_flag = _cmake_option_enabled("DART_BUILD_DART8")
    if cmake_flag is False:
        print_warning("Skipping dart8 tests because DART_BUILD_DART8 is OFF in build")
        return True

    result, _ = run_command(
        pixi_command("test-dart8", PIXI_DEFAULT_DARTPY), "dart8 C++ tests"
    )
    return result


def run_python_tests() -> bool:
    """Run Python tests"""
    print_header("PYTHON TESTS")

    build_type = os.environ.get("BUILD_TYPE", "Release")
    cmake_flag = _cmake_option_enabled("DART_BUILD_DARTPY")
    if cmake_flag is False:
        print_warning("Skipping python tests because DART_BUILD_DARTPY is OFF in build")
        return True

    if not _env_flag_enabled("DART_BUILD_DARTPY_OVERRIDE", PIXI_DEFAULT_DARTPY):
        print_warning("Skipping python tests because DART_BUILD_DARTPY_OVERRIDE is OFF")
        return True

    build_dir = get_build_dir(build_type)
    if not cmake_target_exists(build_dir, build_type, "pytest"):
        print_warning("Skipping python tests because target 'pytest' was not generated")
        return True

    # Ensure the dartpy bindings are built before running pytest
    build_result, _ = run_command(
        pixi_command("build-py-dev", PIXI_DEFAULT_DARTPY, build_type),
        f"Build dartpy bindings ({build_type})",
    )
    if not build_result:
        return False

    # Check if Python bindings are enabled
    result, _ = run_command(
        pixi_command("test-py", build_type), f"Python tests ({build_type})"
    )

    return result


def run_docs_tests() -> bool:
    """Run documentation build tests"""
    print_header("DOCUMENTATION")

    result, _ = run_command(pixi_command("docs-build"), "Documentation build")

    return result


def generate_report(results: dict):
    """Generate a final test report"""
    print_header("TEST REPORT")

    total_tests = len(results)
    passed_tests = sum(1 for v in results.values() if v)
    failed_tests = total_tests - passed_tests

    print(f"Total Tests: {total_tests}")
    print(f"Passed: {Colors.OKGREEN}{passed_tests}{Colors.ENDC}")

    # Use green for 0 failures, red otherwise
    failed_color = Colors.OKGREEN if failed_tests == 0 else Colors.FAIL
    print(f"Failed: {failed_color}{failed_tests}{Colors.ENDC}")
    print()

    # Find the longest test name for alignment
    max_name_length = max(len(name) for name in results.keys()) if results else 0

    for test_name, passed in results.items():
        status = (
            f"{Colors.OKGREEN}PASSED{Colors.ENDC}"
            if passed
            else f"{Colors.FAIL}FAILED{Colors.ENDC}"
        )
        # Pad the test name to align the status column
        padded_name = test_name.ljust(max_name_length)
        print(f"  {padded_name}: {status}")

    print()
    if failed_tests == 0:
        print_success(f"All tests passed! {Symbols.SPARKLES}")
        print_success(f"Ready to submit PR! {Symbols.ROCKET}")
        return True
    else:
        print_error(f"{failed_tests} test(s) failed!")
        print_warning("Please fix the failures before submitting PR.")
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Run comprehensive tests before submitting a PR"
    )
    parser.add_argument("--skip-build", action="store_true", help="Skip build tests")
    parser.add_argument("--skip-tests", action="store_true", help="Skip unit tests")
    parser.add_argument("--skip-lint", action="store_true", help="Skip linting")
    parser.add_argument(
        "--skip-docs", action="store_true", help="Skip documentation build"
    )
    parser.add_argument("--skip-python", action="store_true", help="Skip Python tests")
    parser.add_argument(
        "--skip-dart8", action="store_true", help="Skip dart8 C++ tests"
    )
    parser.add_argument(
        "--skip-debug",
        action="store_true",
        help="Skip Debug build (only build Release)",
    )
    parser.add_argument(
        "--keep-going",
        action="store_true",
        help="Continue running remaining steps even if a failure occurs (default: fail fast)",
    )

    args = parser.parse_args()

    print_header("DART COMPREHENSIVE TEST SUITE")
    print(
        "This will run all tests to ensure your changes are ready for PR submission.\n"
    )

    # Check if pixi is available
    if not check_pixi():
        return 1

    patch_nanobind_if_needed()

    results = {}
    continue_running = True

    def run_step(name: str, func):
        nonlocal continue_running
        if not continue_running:
            return

        result = func()
        results[name] = result

        if not result and not args.keep_going:
            print_error(
                f"{name} failed. Stopping early (pass --keep-going to continue running tests)."
            )
            continue_running = False

    # Run linting
    if not args.skip_lint:
        run_step("Linting", run_lint_tests)
    else:
        print_warning("Skipping linting tests")

    # Run build
    if not args.skip_build:
        run_step("Build", lambda: run_build_tests(skip_debug=args.skip_debug))
    else:
        print_warning("Skipping build tests")

    # Run unit tests
    if not args.skip_tests:
        run_step("Unit Tests", run_unit_tests)
    else:
        print_warning("Skipping unit tests")

    # Run dart8 tests
    if not args.skip_dart8:
        run_step("DART8 Tests", run_dart8_tests)
    else:
        print_warning("Skipping dart8 tests")

    # Run Python tests
    if not args.skip_python:
        run_step("Python Tests", run_python_tests)
    else:
        print_warning("Skipping Python tests")

    # Run documentation build
    if not args.skip_docs:
        run_step("Documentation", run_docs_tests)
    else:
        print_warning("Skipping documentation tests")

    # Generate report
    success = generate_report(results)

    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
