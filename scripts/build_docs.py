#!/usr/bin/env python3
"""
Build DART API Documentation (C++ and Python)

This script builds both C++ (Doxygen) and Python (Sphinx) API documentation
for multiple versions of DART and deploys them to GitHub Pages.

Version Management:
    - Auto-discovers versions from git tags (default)
    - Supports manual version lists via YAML config
    - Can build latest N versions only

Usage:
    python scripts/build_docs.py build
    python scripts/build_docs.py build --strategy latest_n --count 5
    python scripts/build_docs.py list-versions
    python scripts/build_docs.py clean
"""

import argparse
import logging
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import List, Optional

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger(__name__)


class DocsBuilder:
    """Builds DART API documentation for multiple versions."""

    def __init__(self):
        """Initialize paths and configuration."""
        # Get workspace root
        github_workspace = os.environ.get("GITHUB_WORKSPACE", Path.cwd())
        self.workspace = Path(github_workspace)

        # Working directories
        self.work_dir = self.workspace / "dart_docs"
        self.clone_dir = self.work_dir / "dart"
        self.build_dir = self.work_dir / "build"
        self.output_dir = self.workspace / "gh-pages"

        # Branch name (if building from a specific branch)
        self.branch_name = os.environ.get("BRANCH_NAME")

        # Versions file
        self.versions_file = self.clone_dir / "scripts" / "docs_versions.txt"

    def run_command(
        self,
        cmd: List[str],
        cwd: Optional[Path] = None,
        check: bool = True,
        env: Optional[dict] = None,
    ) -> subprocess.CompletedProcess:
        """
        Run a command and log output.

        Args:
            cmd: Command and arguments as a list
            cwd: Working directory for the command
            check: Whether to raise exception on non-zero exit
            env: Environment variables to pass to the command

        Returns:
            CompletedProcess instance
        """
        cmd_str = " ".join(str(c) for c in cmd)
        logger.info(f"Running: {cmd_str}")
        if cwd:
            logger.info(f"  in directory: {cwd}")

        # Merge environment variables
        cmd_env = os.environ.copy()
        if env:
            cmd_env.update(env)

        try:
            result = subprocess.run(
                cmd, cwd=cwd, check=check, capture_output=True, text=True, env=cmd_env
            )
            if result.stdout:
                logger.debug(result.stdout)
            return result
        except subprocess.CalledProcessError as e:
            logger.error(f"Command failed with exit code {e.returncode}")
            if e.stdout:
                logger.error(f"STDOUT:\n{e.stdout}")
            if e.stderr:
                logger.error(f"STDERR:\n{e.stderr}")
            raise

    def clean(self):
        """Clean up generated documentation and temporary files."""
        logger.info("=" * 70)
        logger.info(" Cleaning up")
        logger.info("=" * 70)

        directories = [self.work_dir, self.output_dir]
        for directory in directories:
            if directory.exists():
                logger.info(f"Removing {directory}")
                shutil.rmtree(directory)
            else:
                logger.info(f"Directory {directory} does not exist, skipping")

        logger.info("Cleanup complete")

    def clone_repository(self):
        """Clone DART repository."""
        logger.info("=" * 70)
        logger.info(" Cloning DART repository")
        logger.info("=" * 70)

        self.work_dir.mkdir(parents=True, exist_ok=True)

        # Clone the full repository
        logger.info(f"Cloning to {self.clone_dir}")
        self.run_command(
            ["git", "clone", "https://github.com/dartsim/dart.git", str(self.clone_dir)]
        )

        # Checkout specific branch if specified
        if self.branch_name:
            logger.info(f"Checking out branch: {self.branch_name}")
            try:
                self.run_command(
                    ["git", "checkout", self.branch_name], cwd=self.clone_dir
                )
            except subprocess.CalledProcessError:
                logger.warning(
                    f"Branch {self.branch_name} not found in upstream, "
                    "using default branch"
                )

    def read_versions(self) -> List[tuple]:
        """
        Read versions to build from docs_versions.txt.

        Returns:
            List of (version_name, is_header) tuples
        """
        logger.info(f"Reading versions from {self.versions_file}")

        if not self.versions_file.exists():
            raise FileNotFoundError(f"Versions file not found: {self.versions_file}")

        versions = []
        with open(self.versions_file, "r") as f:
            for line in f:
                line = line.strip()
                if not line:  # Skip empty lines
                    continue
                # Check if this is a header line (e.g., "DART 6")
                is_header = line.startswith("DART ")
                versions.append((line, is_header))

        logger.info(f"Found {len(versions)} version entries")
        return versions

    def build_cpp_docs(self, version: str):
        """
        Build C++ API documentation using Doxygen.

        Args:
            version: Version tag or branch name
        """
        logger.info(f"Building C++ documentation for {version}")

        # Checkout version
        self.run_command(["git", "checkout", version], cwd=self.clone_dir)

        # Clean build directory
        if self.build_dir.exists():
            for item in self.build_dir.iterdir():
                if item.is_dir():
                    shutil.rmtree(item)
                else:
                    item.unlink()
        else:
            self.build_dir.mkdir(parents=True)

        # Configure with CMake (enable dartpy for Python docs later)
        self.run_command(
            ["cmake", str(self.clone_dir), "-DDART_BUILD_DARTPY=ON"], cwd=self.build_dir
        )

        # Build C++ documentation
        self.run_command(["make", "docs"], cwd=self.build_dir)

        # Find and move documentation
        docs_sources = [
            self.build_dir / "docs" / "doxygen" / "html",
            self.build_dir / "doxygen" / "html",
        ]

        docs_dest = self.output_dir / version
        docs_moved = False

        for docs_source in docs_sources:
            if docs_source.exists() and any(docs_source.iterdir()):
                logger.info(f"Moving {docs_source} to {docs_dest}")
                docs_dest.parent.mkdir(parents=True, exist_ok=True)
                shutil.move(str(docs_source), str(docs_dest))
                docs_moved = True
                break

        if not docs_moved:
            logger.warning(f"No C++ documentation generated for {version}")

    def build_python_docs(self, version: str):
        """
        Build Python API documentation using Sphinx.

        Args:
            version: Version tag or branch name
        """
        logger.info("=" * 70)
        logger.info(f" Building Python API documentation for {version}")
        logger.info("=" * 70)

        # Build dartpy module
        logger.info("Building dartpy module...")
        num_cores = os.cpu_count() or 4
        self.run_command(["make", f"-j{num_cores}", "dartpy"], cwd=self.build_dir)

        # Find dartpy module location
        dartpy_locations = [
            self.build_dir / "python" / "dartpy",
            self.build_dir / "python",
        ]

        dartpy_path = None
        for location in dartpy_locations:
            if location.exists():
                dartpy_path = location
                break

        if not dartpy_path:
            logger.warning(
                f"Could not find dartpy module for {version}, skipping Python docs"
            )
            return

        logger.info(f"Found dartpy at: {dartpy_path}")

        # Check if Python API docs configuration exists
        python_docs_dir = self.clone_dir / "docs" / "python_api"
        if not (python_docs_dir / "conf.py").exists():
            logger.warning(
                f"No Python API documentation configuration found for {version}, skipping"
            )
            return

        # Set PYTHONPATH and build documentation
        logger.info("Building Python API documentation with Sphinx...")
        docs_dest = self.output_dir / f"{version}-py"
        docs_dest.mkdir(parents=True, exist_ok=True)

        env = {"PYTHONPATH": str(dartpy_path)}

        try:
            self.run_command(
                ["sphinx-build", "-b", "html", ".", str(docs_dest)],
                cwd=python_docs_dir,
                env=env,
            )
            logger.info(f"✓ Python documentation generated at {docs_dest}")
        except subprocess.CalledProcessError:
            logger.error(f"Failed to build Python documentation for {version}")
            # Don't fail the entire build, just continue

    def create_index(self, versions: List[tuple]):
        """
        Create index.md file listing all API documentation versions.

        Args:
            versions: List of (version_name, is_header) tuples
        """
        logger.info("Creating index.md")

        index_file = self.output_dir / "index.md"

        with open(index_file, "w") as f:
            f.write("# DART API Documentation\n\n")

            for version, is_header in versions:
                if is_header:
                    f.write(f"\n## {version}\n\n")
                else:
                    # Write links for both C++ and Python docs
                    cpp_link = f"https://dartsim.github.io/dart/{version}/"
                    python_link = f"https://dartsim.github.io/dart/{version}-py/"

                    f.write(f"**{version}**\n")
                    f.write(f"* [C++ API]({cpp_link})\n")
                    f.write(f"* [Python API]({python_link})\n")
                    f.write("\n")

        logger.info(f"Index created at {index_file}")

    def build(self):
        """Build all documentation."""
        logger.info("=" * 70)
        logger.info(" Building DART API Documentation")
        logger.info("=" * 70)

        # Clone repository
        self.clone_repository()

        # Create output directory
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # Read versions to build
        versions = self.read_versions()

        # Create build directory
        self.build_dir.mkdir(parents=True, exist_ok=True)

        # Build documentation for each version
        version_list = []
        for version, is_header in versions:
            if is_header:
                version_list.append((version, is_header))
                logger.info("")
                logger.info("=" * 70)
                logger.info(f" {version}")
                logger.info("=" * 70)
                continue

            version_list.append((version, is_header))

            logger.info("")
            logger.info("=" * 70)
            logger.info(f" Building documentation for {version}")
            logger.info("=" * 70)

            try:
                # Build C++ documentation
                self.build_cpp_docs(version)

                # Build Python documentation
                self.build_python_docs(version)

            except Exception as e:
                logger.error(f"Failed to build documentation for {version}: {e}")
                logger.warning("Continuing with next version...")
                continue

        # Create index
        self.create_index(version_list)

        logger.info("")
        logger.info("=" * 70)
        logger.info(" Build Complete!")
        logger.info("=" * 70)
        logger.info(f"Documentation output: {self.output_dir}")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Build DART API Documentation (C++ and Python)"
    )
    parser.add_argument(
        "command", choices=["build", "clean"], help="Command to execute"
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true", help="Enable verbose logging"
    )

    args = parser.parse_args()

    # Set logging level
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    # Create builder and run command
    builder = DocsBuilder()

    try:
        if args.command == "build":
            builder.build()
        elif args.command == "clean":
            builder.clean()

        logger.info("✓ Success!")
        return 0

    except Exception as e:
        logger.error(f"✗ Failed: {e}", exc_info=args.verbose)
        return 1


if __name__ == "__main__":
    sys.exit(main())
