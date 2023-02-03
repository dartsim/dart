# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

# pybind11
if(DART_DOWNLOAD_DEPENDENT_PACKAGES)
  # Fetch Pybind11
  dart_fetch_git_repo(
    PROJECT_NAME pybind11
    WORKING_DIR ${CMAKE_CURRENT_BINARY_DIR}/.deps
    GIT_URL https://github.com/pybind/pybind11
    GIT_TAG v2.10.3
  )

  # Set options
  set(PYBIND11_TEST OFF CACHE BOOL "" FORCE)

  # To suppress warning, Policy CMP0042 is not set
  set(CMAKE_MACOSX_RPATH 1)

  # Adds the targers: gtest, gtest_main, gmock, gmock_main
  add_subdirectory(
    ${pybind11_SOURCE_DIR}
    ${pybind11_BINARY_DIR}
    EXCLUDE_FROM_ALL
  )

  set(pybind11_FOUND TRUE)
else()
  # Find pybind11, including PythonInterp and PythonLibs
  if(WIN32)
    set(PYBIND11_FINDPYTHON TRUE)
  endif()
  find_package(pybind11 2.2.0 QUIET)
  if(NOT pybind11_FOUND)
    message(WARNING "Disabling [dartpy] due to missing pybind11 >= 2.2.0.")
    return()
  endif()
endif()

execute_process(COMMAND ${PYTHON_EXECUTABLE} -c
  "from distutils.sysconfig import get_python_lib;\
  print(get_python_lib(plat_specific=True))"
  OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NOT IS_ABSOLUTE ${PYTHON_SITE_PACKAGES})
  set(PYTHON_SITE_PACKAGES "${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE_PACKAGES}")
endif()
set(PYTHON_SITE_PACKAGES ${PYTHON_SITE_PACKAGES} PARENT_SCOPE)

add_custom_target(_copy_submodules)

add_subdirectory(common)
add_subdirectory(math)
add_subdirectory(optimization)
add_subdirectory(collision)
add_subdirectory(dynamics)
add_subdirectory(simulation)
add_subdirectory(io)
add_subdirectory(gui)
