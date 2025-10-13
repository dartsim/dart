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
import subprocess
import sys
from typing import Tuple


class Colors:
    """ANSI color codes for terminal output"""
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def print_header(message: str):
    """Print a formatted header"""
    print(f"\n{Colors.HEADER}{Colors.BOLD}{'=' * 80}{Colors.ENDC}")
    print(f"{Colors.HEADER}{Colors.BOLD}{message}{Colors.ENDC}")
    print(f"{Colors.HEADER}{Colors.BOLD}{'=' * 80}{Colors.ENDC}\n")


def print_step(message: str):
    """Print a formatted step"""
    print(f"{Colors.OKBLUE}▶ {message}{Colors.ENDC}")


def print_success(message: str):
    """Print a success message"""
    print(f"{Colors.OKGREEN}✓ {message}{Colors.ENDC}")


def print_error(message: str):
    """Print an error message"""
    print(f"{Colors.FAIL}✗ {message}{Colors.ENDC}")


def print_warning(message: str):
    """Print a warning message"""
    print(f"{Colors.WARNING}⚠ {message}{Colors.ENDC}")


def run_command(cmd: str, description: str) -> Tuple[bool, str]:
    """
    Run a command and return success status and output.

    Args:
        cmd: Command to run
        description: Description of the command for logging

    Returns:
        Tuple of (success: bool, output: str)
    """
    print_step(f"Running: {description}")
    print(f"  Command: {cmd}")

    try:
        result = subprocess.run(
            cmd,
            shell=True,
            capture_output=True,
            text=True,
            check=False
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


def check_pixi() -> bool:
    """Check if pixi is available"""
    try:
        subprocess.run(["pixi", "--version"], capture_output=True, check=True)
        return True
    except (subprocess.CalledProcessError, FileNotFoundError):
        print_error("pixi not found. Please install pixi first.")
        return False


def run_lint_tests() -> bool:
    """Run linting tests"""
    print_header("LINTING")

    success = True

    # Check C++ formatting
    result, _ = run_command(
        "pixi run check-lint-cpp",
        "C++ format check"
    )
    success = success and result

    # Check Python formatting
    result, _ = run_command(
        "pixi run check-lint-py",
        "Python format check"
    )
    success = success and result

    return success


def run_build_tests(skip_debug: bool = False) -> bool:
    """Run build tests"""
    print_header("BUILD")

    success = True

    # Build Release
    result, _ = run_command(
        "pixi run build",
        "Build Release"
    )
    success = success and result

    if not skip_debug:
        # Build Debug (for better error messages)
        result, _ = run_command(
            "pixi run build-debug",
            "Build Debug"
        )
        success = success and result

    return success


def run_unit_tests() -> bool:
    """Run unit tests"""
    print_header("UNIT TESTS")

    success = True

    # Build and run C++ tests
    result, _ = run_command(
        "pixi run test",
        "C++ unit tests"
    )
    success = success and result

    return success


def run_python_tests() -> bool:
    """Run Python tests"""
    print_header("PYTHON TESTS")

    # Check if Python bindings are enabled
    result, _ = run_command(
        "pixi run test-py",
        "Python tests"
    )

    return result


def run_docs_tests() -> bool:
    """Run documentation build tests"""
    print_header("DOCUMENTATION")

    result, _ = run_command(
        "pixi run docs-build",
        "Documentation build"
    )

    return result


def generate_report(results: dict):
    """Generate a final test report"""
    print_header("TEST REPORT")

    total_tests = len(results)
    passed_tests = sum(1 for v in results.values() if v)
    failed_tests = total_tests - passed_tests

    print(f"Total Tests: {total_tests}")
    print(f"Passed: {Colors.OKGREEN}{passed_tests}{Colors.ENDC}")
    print(f"Failed: {Colors.FAIL}{failed_tests}{Colors.ENDC}")
    print()

    for test_name, passed in results.items():
        status = f"{Colors.OKGREEN}PASSED{Colors.ENDC}" if passed else f"{Colors.FAIL}FAILED{Colors.ENDC}"
        print(f"  {test_name}: {status}")

    print()
    if failed_tests == 0:
        print_success("All tests passed! ✨")
        print_success("Ready to submit PR! 🚀")
        return True
    else:
        print_error(f"{failed_tests} test(s) failed!")
        print_warning("Please fix the failures before submitting PR.")
        return False


def main():
    parser = argparse.ArgumentParser(
        description='Run comprehensive tests before submitting a PR'
    )
    parser.add_argument(
        '--skip-build',
        action='store_true',
        help='Skip build tests'
    )
    parser.add_argument(
        '--skip-tests',
        action='store_true',
        help='Skip unit tests'
    )
    parser.add_argument(
        '--skip-lint',
        action='store_true',
        help='Skip linting'
    )
    parser.add_argument(
        '--skip-docs',
        action='store_true',
        help='Skip documentation build'
    )
    parser.add_argument(
        '--skip-python',
        action='store_true',
        help='Skip Python tests'
    )
    parser.add_argument(
        '--skip-debug',
        action='store_true',
        help='Skip Debug build (only build Release)'
    )

    args = parser.parse_args()

    print_header("DART COMPREHENSIVE TEST SUITE")
    print("This will run all tests to ensure your changes are ready for PR submission.\n")

    # Check if pixi is available
    if not check_pixi():
        return 1

    results = {}

    # Run linting
    if not args.skip_lint:
        results['Linting'] = run_lint_tests()
    else:
        print_warning("Skipping linting tests")

    # Run build
    if not args.skip_build:
        results['Build'] = run_build_tests(skip_debug=args.skip_debug)
    else:
        print_warning("Skipping build tests")

    # Run unit tests
    if not args.skip_tests:
        results['Unit Tests'] = run_unit_tests()
    else:
        print_warning("Skipping unit tests")

    # Run Python tests
    if not args.skip_python:
        results['Python Tests'] = run_python_tests()
    else:
        print_warning("Skipping Python tests")

    # Run documentation build
    if not args.skip_docs:
        results['Documentation'] = run_docs_tests()
    else:
        print_warning("Skipping documentation tests")

    # Generate report
    success = generate_report(results)

    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
