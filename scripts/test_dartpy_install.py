#!/usr/bin/env python3
"""
Test script for dartpy CMake installation (GitHub issue #1848).

Tests that dartpy can be installed via CMake after the packaging refactor.
Verifies:
1. CMake install to custom prefix works
2. CMake install to virtual environment works
3. Install rules are present in CMakeLists.txt
"""

import shutil
import subprocess
import sys
import tempfile
import venv
from pathlib import Path


class Colors:
    """ANSI color codes for terminal output."""

    RED = "\033[0;31m"
    GREEN = "\033[0;32m"
    YELLOW = "\033[1;33m"
    BLUE = "\033[0;34m"
    NC = "\033[0m"


def print_header(message: str) -> None:
    """Print a formatted header."""
    print(f"\n{Colors.BLUE}{'=' * 40}{Colors.NC}")
    print(f"{Colors.BLUE}{message}{Colors.NC}")
    print(f"{Colors.BLUE}{'=' * 40}{Colors.NC}\n")


def print_success(message: str) -> None:
    """Print a success message."""
    print(f"{Colors.GREEN}✓ {message}{Colors.NC}")


def print_error(message: str) -> None:
    """Print an error message."""
    print(f"{Colors.RED}✗ {message}{Colors.NC}")


def print_info(message: str) -> None:
    """Print an info message."""
    print(f"{Colors.YELLOW}→ {message}{Colors.NC}")


def run_command(cmd: list[str], cwd: Path) -> bool:
    """Run a command and return success status."""
    try:
        subprocess.run(cmd, cwd=cwd, check=True, capture_output=True)
        return True
    except subprocess.CalledProcessError as e:
        print_error(f"Command failed: {' '.join(cmd)}")
        if e.stdout:
            print(e.stdout.decode())
        if e.stderr:
            print(e.stderr.decode())
        return False


def check_prerequisites() -> bool:
    """Check if required tools are available."""
    print_header("Checking Prerequisites")

    # Check CMake
    if not shutil.which("cmake"):
        print_error("CMake not found. Please install CMake.")
        return False

    result = subprocess.run(
        ["cmake", "--version"], capture_output=True, text=True, check=True
    )
    print_success(f"CMake found: {result.stdout.splitlines()[0]}")

    # Check Python
    print_success(f"Python3 found: {sys.version.split()[0]}")

    # Check Ninja (recommended)
    if not shutil.which("ninja"):
        print_info("Ninja not found. Using system default generator.")
    else:
        print_success("Ninja found")

    return True


def cmake_build_and_install(project_root: Path, build_dir: Path, prefix: Path) -> bool:
    """Configure, build, and install DART with dartpy to the given prefix."""
    cmake_args = [
        "cmake",
        "-S",
        str(project_root),
        "-B",
        str(build_dir),
        f"-DCMAKE_INSTALL_PREFIX={prefix}",
        "-DCMAKE_BUILD_TYPE=Release",
        "-DDART_BUILD_DARTPY=ON",
        "-DDART_VERBOSE=OFF",
    ]
    if shutil.which("ninja"):
        cmake_args.extend(["-G", "Ninja"])

    if not run_command(cmake_args, project_root):
        return False
    if not run_command(["cmake", "--build", str(build_dir)], project_root):
        return False
    if not run_command(
        ["cmake", "--build", str(build_dir), "--target", "dartpy"], project_root
    ):
        return False
    return run_command(["cmake", "--install", str(build_dir)], project_root)


def verify_installation(prefix: Path) -> bool:
    """Verify dartpy module and DART libraries are installed in prefix."""
    py_version = f"{sys.version_info.major}.{sys.version_info.minor}"
    site_packages = prefix / "lib" / f"python{py_version}" / "site-packages"

    dartpy_files = list(site_packages.glob("dartpy*.so")) + list(
        site_packages.glob("dartpy*.pyd")
    )
    if not dartpy_files:
        print_error(f"dartpy module not found in {site_packages}")
        return False
    print_success(f"dartpy module found: {dartpy_files[0].name}")

    dart_libs = []
    for lib_dir in [prefix / "lib", prefix / "lib64"]:
        if lib_dir.exists():
            dart_libs.extend(lib_dir.glob("libdart*.so*"))
            dart_libs.extend(lib_dir.glob("dart*.dll"))

    if not dart_libs:
        print_error("DART libraries not found")
        return False
    print_success(f"Found {len(dart_libs)} DART library files")
    return True


def test_install_to_prefix(project_root: Path, build_dir: Path, prefix: Path) -> bool:
    """Test CMake install to custom prefix."""
    print_header("Test 1: CMake Install to Custom Prefix")

    print_info("Creating build directory...")
    build_dir.mkdir(parents=True, exist_ok=True)

    print_info("Configuring and building...")
    if not cmake_build_and_install(project_root, build_dir, prefix):
        return False

    print_info("Verifying installation...")
    if not verify_installation(prefix):
        return False

    print_success("Test 1 PASSED: CMake install to custom prefix works!")
    return True


def test_install_to_venv(project_root: Path, build_dir: Path, venv_dir: Path) -> bool:
    """Test CMake install to virtual environment."""
    print_header("Test 2: CMake Install to Virtual Environment")

    print_info("Creating Python virtual environment...")
    venv.create(venv_dir, with_pip=True)

    print_info("Cleaning previous build...")
    shutil.rmtree(build_dir, ignore_errors=True)
    build_dir.mkdir(parents=True)

    print_info("Configuring and building...")
    if not cmake_build_and_install(project_root, build_dir, venv_dir):
        return False

    print_info("Verifying installation...")
    if not verify_installation(venv_dir):
        return False

    print_success("Test 2 PASSED: CMake install to venv works!")
    return True


def test_install_rules_exist(project_root: Path) -> bool:
    """Verify install rules exist in CMakeLists.txt."""
    print_header("Test 3: Verify Install Rules")

    cmake_file = project_root / "python" / "dartpy" / "CMakeLists.txt"

    print_info("Checking if dartpy install rules exist in CMakeLists.txt...")
    content = cmake_file.read_text()

    if "install(TARGETS" in content and "dartpy" in content:
        print_success("Install rules found in CMakeLists.txt")
    else:
        print_error("Install rules NOT found in CMakeLists.txt")
        return False

    print_info("Checking install destination logic...")
    if "SKBUILD_PLATLIB_DIR" in content or "PYTHON_SITE_PACKAGES" in content:
        print_success("Install destination logic is present")
    else:
        print_error("Install destination logic NOT found")
        return False

    return True


def main() -> int:
    """Run all tests."""
    project_root = Path(__file__).parent.parent

    # Check prerequisites
    if not check_prerequisites():
        return 1

    # Create temporary directories for testing
    with tempfile.TemporaryDirectory(prefix="dart_test_") as tmpdir:
        tmpdir_path = Path(tmpdir)
        build_dir = tmpdir_path / "build"
        venv_dir = tmpdir_path / "venv"
        install_prefix = tmpdir_path / "prefix"

        # Run tests
        test_results = []

        test_results.append(
            test_install_to_prefix(project_root, build_dir, install_prefix)
        )
        test_results.append(test_install_to_venv(project_root, build_dir, venv_dir))
        test_results.append(test_install_rules_exist(project_root))

        # Print summary
        print_header("Test Summary")

        if all(test_results):
            print_success("All tests passed!")
            print()
            print("Verified:")
            print("  1. ✓ CMake install to custom prefix works")
            print("  2. ✓ CMake install to virtual environment works")
            print("  3. ✓ Install rules are present in CMakeLists.txt")
            print()
            print(f"{Colors.GREEN}GitHub issue #1848 is RESOLVED!{Colors.NC}")
            print()
            print("The fix will be available in DART v7.0.0+")
            print("Users can build from main branch for immediate access.")
            return 0
        else:
            print_error("Some tests failed!")
            return 1


if __name__ == "__main__":
    sys.exit(main())
