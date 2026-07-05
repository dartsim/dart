import shutil
import subprocess
import textwrap
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]


def test_msvc_posix_math_links_pruned_from_nested_imported_targets(tmp_path):
    cmake = shutil.which("cmake")
    if cmake is None:
        pytest.skip("cmake is not available")

    source = tmp_path / "source"
    source.mkdir()
    (source / "CMakeLists.txt").write_text(
        textwrap.dedent(
            f"""
            cmake_minimum_required(VERSION 3.22)
            project(msvc_link_sanitizer NONE)

            include("{(ROOT / "cmake" / "dart_defs.cmake").as_posix()}")
            set(MSVC TRUE)

            add_library(Leaf INTERFACE IMPORTED)
            set_target_properties(
              Leaf
              PROPERTIES INTERFACE_LINK_LIBRARIES "icuuc;icudt;m;-lm;$<LINK_ONLY:m>;kernel32"
            )

            add_library(Nested INTERFACE IMPORTED)
            set_target_properties(
              Nested
              PROPERTIES INTERFACE_LINK_LIBRARIES "m;z"
            )

            add_library(CURL::libcurl INTERFACE IMPORTED)
            set_target_properties(
              CURL::libcurl
              PROPERTIES INTERFACE_LINK_LIBRARIES "Leaf;$<LINK_ONLY:Nested>"
            )

            dart_prune_msvc_posix_link_libraries(CURL::libcurl)

            get_target_property(leaf_links Leaf INTERFACE_LINK_LIBRARIES)
            get_target_property(nested_links Nested INTERFACE_LINK_LIBRARIES)
            get_target_property(curl_links CURL::libcurl INTERFACE_LINK_LIBRARIES)

            if(NOT leaf_links STREQUAL "icuuc;icudt;kernel32")
              message(FATAL_ERROR "Leaf links were not sanitized: ${{leaf_links}}")
            endif()
            if(NOT nested_links STREQUAL "z")
              message(FATAL_ERROR "Nested links were not sanitized: ${{nested_links}}")
            endif()
            if(NOT curl_links STREQUAL "Leaf;$<LINK_ONLY:Nested>")
              message(FATAL_ERROR "CURL links were changed unexpectedly: ${{curl_links}}")
            endif()
            """
        ),
        encoding="utf-8",
    )

    result = subprocess.run(
        [cmake, "-S", str(source), "-B", str(tmp_path / "build")],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stdout + result.stderr
