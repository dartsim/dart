# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.

# References:
# - https://pybind11.readthedocs.io/en/stable/compiling.html

import distutils.log
import os
import re
import subprocess
import sys
from codecs import open  # To use a consistent encoding
from glob import glob
from pathlib import Path

from setuptools import Extension, find_packages, setup
from setuptools.command.build_ext import build_ext

# Convert distutils Windows platform specifiers to CMake -A arguments
# PLAT_TO_CMAKE = {
#     "win32": "Win32",
#     "win-amd64": "x64",
#     "win-arm32": "ARM",
#     "win-arm64": "ARM64",
# }

# Get the current directory path.
dart_root = os.path.abspath(os.path.dirname(__file__))
dartpy_root = os.path.join(dart_root, "python")

with open(os.path.join(dart_root, "README.md"), encoding="utf-8") as f:
    long_description = f.read()
description = "Python API of Dynamic Animation and Robotics Toolkit."

distutils.log.set_verbosity(distutils.log.DEBUG)  # Set DEBUG level


# A CMakeExtension needs a sourcedir instead of a file list.
# The name must be the _single_ output extension from the CMake build.
# If you need multiple extensions, see scikit-build.
class CMakeExtension(Extension):
    def __init__(self, name: str, sourcedir: str = "", sources=[]) -> None:
        super().__init__(name, sources=sources)
        self.sourcedir = os.fspath(Path(sourcedir).resolve())


class CMakeBuild(build_ext):
    """Wrapper class that builds the extension using CMake."""

    def build_extension(self, ext: CMakeExtension) -> None:
        # Must be in this form due to bug in .resolve() only fixed in Python 3.10+
        # type: ignore[no-untyped-call]
        ext_fullpath = Path.cwd() / self.get_ext_fullpath(ext.name)
        extdir = ext_fullpath.parent.resolve()

        # Using this requires trailing slash for auto-detection & inclusion of
        # auxiliary "native" libs

        debug = int(os.environ.get("DEBUG", 0)) if self.debug is None else self.debug
        cfg = "Debug" if debug else "Release"

        # CMake lets you override the generator - we need to check this.
        # Can be set with Conda-Build, for example.
        cmake_generator = os.environ.get("CMAKE_GENERATOR", "")

        # Set Python_EXECUTABLE instead if you use PYBIND11_FINDPYTHON
        # DARTPY_VERSION_INFO shows you how to pass a value into the C++ code
        # from Python.
        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}{os.sep}",
            f"-DPYTHON_EXECUTABLE={sys.executable}",
            f"-DCMAKE_BUILD_TYPE={cfg}",  # not used on MSVC, but no harm
            f"-DBUILD_SHARED_LIBS=OFF",
            f"-DDART_DBUILD_TESTING=OFF",
            f"-DDART_ENABLE_SIMD=OFF",
            f"-DDART_BUILD_WHEELS=ON",
            f"-DDART_DOWNLOAD_DEPENDENT_PACKAGES=ON",
            f"-DDART_TREAT_WARNINGS_AS_ERRORS=OFF",
        ]
        build_args = []
        # Adding CMake arguments set as environment variable
        # (needed e.g. to build for ARM OSx on conda-forge)
        if "CMAKE_ARGS" in os.environ:
            cmake_args += [item for item in os.environ["CMAKE_ARGS"].split(" ") if item]

        # In this example, we pass in the version to C++. You might not need to.
        # type: ignore[attr-defined]
        cmake_args += [f"-DDARTPY_VERSION_INFO={self.distribution.get_version()}"]

        if self.compiler.compiler_type != "msvc":
            # Using Ninja-build since it a) is available as a wheel and b)
            # multithread automatically. MSVC would require all variables be
            # exported for Ninja to pick it up, which is a little tricky to do.
            # Users can override the generator with CMAKE_GENERATOR in CMake
            # 3.15+.
            if not cmake_generator or cmake_generator == "Ninja":
                try:
                    import ninja  # noqa: F401

                    ninja_executable_path = Path(ninja.BIN_DIR) / "ninja"
                    cmake_args += [
                        "-GNinja",
                        f"-DCMAKE_MAKE_PROGRAM:FILEPATH={ninja_executable_path}",
                    ]
                except ImportError:
                    pass

        else:
            # Single config generators are handled "normally"
            single_config = any(x in cmake_generator for x in {"NMake", "Ninja"})

            # CMake allows an arch-in-generator style for backward compatibility
            contains_arch = any(x in cmake_generator for x in {"ARM", "Win64"})

            # Specify the arch if using MSVC generator, but only if it doesn't
            # contain a backward-compatibility arch spec already in the
            # generator name.
            # if not single_config and not contains_arch:
            #     cmake_args += ["-A", PLAT_TO_CMAKE[self.plat_name]]

            if sys.maxsize > 2**32:
                cmake_args += ["-A", "x64"]

            # Multi-config generators have a different way to specify configs
            if not single_config:
                cmake_args += [
                    f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{cfg.upper()}={extdir}"
                ]
                build_args += ["--config", cfg]

        if sys.platform.startswith("darwin"):
            # Enable cross-compilation support for macOS and respect ARCHFLAGS if set
            archs = re.findall(r"-arch (\S+)", os.environ.get("ARCHFLAGS", ""))
            if archs:
                cmake_args.append(
                    "-DCMAKE_OSX_ARCHITECTURES={}".format(";".join(archs))
                )

            # Get the location of Homebrew prefix and specify the location of OpenCL headers for macOS
            homebrew_prefix = (
                subprocess.check_output(["brew", "--prefix"]).decode().strip()
            )
            opencl_headers_dir = (
                f"{homebrew_prefix}/opt/opencl-headers/share/cmake/OpenCLHeaders"
            )
            opencl_headerscpp_dir = f"{homebrew_prefix}/opt/opencl-clhpp-headers/share/cmake/OpenCLHeadersCpp"
            cmake_args.extend(
                [
                    f"-DOpenCLHeaders_DIR={opencl_headers_dir}",
                    f"-DOpenCLHeadersCpp_DIR={opencl_headerscpp_dir}",
                    f"-DCMAKE_PREFIX_PATH={homebrew_prefix}",
                ]
            )

            print(f"[DEBUG] cmake_args: {cmake_args}")

        # Set CMAKE_BUILD_PARALLEL_LEVEL to control the parallel build level
        # across all generators.
        elif "CMAKE_BUILD_PARALLEL_LEVEL" not in os.environ:
            # self.parallel is a Python 3 only way to set parallel jobs by hand
            # using -j in the build_ext call, not supported by pip or PyPA-build.
            if hasattr(self, "parallel") and self.parallel:
                # CMake 3.12+ only.
                build_args += [f"-j{self.parallel}"]

        cmake_args += [
            f"-DDART_IN_CI={os.environ.get('DART_IN_CI', 'OFF')}",
        ]

        build_temp = Path(self.build_temp) / ext.name
        if not build_temp.exists():
            build_temp.mkdir(parents=True)

        env = os.environ.copy()

        subprocess.run(
            ["cmake", ext.sourcedir] + cmake_args, cwd=build_temp, env=env, check=True
        )
        subprocess.run(
            ["cmake", "--build", "."] + ["--target", "dartpy"] + build_args,
            cwd=build_temp,
            check=True,
        )


sources = ["CMakeLists.txt", "package.xml"]
sources.extend(glob("cmake/**/*", recursive=True))
sources.extend(glob("dart/**/*", recursive=True))
sources.extend(glob("python/**/*", recursive=True))
sources.extend(glob("doxygen/**/*", recursive=True))
sources.extend(glob("examples/**/*", recursive=True))
sources.extend(glob("tools/**/*", recursive=True))
sources.extend(glob("tutorials/**/*", recursive=True))


def get_new_patch_number(package_name, default: int):
    import requests

    try:
        url = f"https://pypi.org/pypi/{package_name}/json"
        response = requests.get(url)
        if response.status_code == 200:
            data = response.json()
            version = data["info"]["version"]
            patch_number = version.split(".")[-1].split("post")[-1]
            return str(max(default, int(patch_number) + 1))
        else:
            return str(default)
    except requests.exceptions.RequestException:
        return str(default)


# Set up the python package wrapping this extension.
setup(
    name="dartpy",
    version="0.1.0.post" + get_new_patch_number("dartpy", 31),
    description=description,
    long_description=long_description,
    long_description_content_type="text/markdown",
    author="Jeongseok Lee",
    author_email="jslee02@gmail.com",
    license="BSD 2-Clause",
    keywords="dartsim robotics",
    classifiers=[
        "Development Status :: 2 - Pre-Alpha",
        "Framework :: Robot Framework",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: BSD License",
        "Topic :: Scientific/Engineering",
        "Operating System :: Microsoft :: Windows",
        "Operating System :: MacOS",
        "Operating System :: POSIX :: Linux",
    ],
    ext_modules=[CMakeExtension("dartpy_math", sources=sources)],
    url="https://github.com/dartsim/dart.git",
    cmdclass={"build_ext": CMakeBuild},
    zip_safe=False,
    extras_require={"test": ["pytest>=6.0"]},
    python_requires=">=3.7",
    install_requires=[
        "numpy",
    ],
    packages=find_packages(where="python", exclude=["__pycache__"]),
    package_dir={"": "python"},
)
