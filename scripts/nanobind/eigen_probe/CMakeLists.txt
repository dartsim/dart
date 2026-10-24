cmake_minimum_required(VERSION 3.28)
project(nanobind_eigen_probe LANGUAGES CXX)

find_package(Python 3.12 REQUIRED COMPONENTS Interpreter Development.Module)
execute_process(
  COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_VARIABLE nanobind_cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE
  COMMAND_ERROR_IS_FATAL ANY
)
list(APPEND CMAKE_PREFIX_PATH "${nanobind_cmake_dir}")
find_package(nanobind 3.1 CONFIG REQUIRED)
if(
  NOT NB_DIR
  OR NOT EXISTS "${NB_DIR}/include/nanobind/nanobind.h"
  OR NOT EXISTS "${NB_DIR}/src/nb_internals.h"
)
  message(
    FATAL_ERROR
    "The Eigen probe requires a complete nanobind 3.1.x package: NB_DIR must contain "
    "include/nanobind/nanobind.h and src/nb_internals.h. "
    "Install the complete nanobind package."
  )
endif()
file(
  STRINGS "${NB_DIR}/include/nanobind/nanobind.h"
  probe_nanobind_version_header
  REGEX "^# *define NB_VERSION_(MAJOR|MINOR|PATCH) "
)
foreach(part MAJOR MINOR PATCH)
  set(CMAKE_MATCH_1 "")
  string(
    REGEX MATCH
    "NB_VERSION_${part} ([0-9]+)"
    _
    "${probe_nanobind_version_header}"
  )
  set(probe_nanobind_${part} "${CMAKE_MATCH_1}")
endforeach()
set(
  probe_nanobind_version
  "${probe_nanobind_MAJOR}.${probe_nanobind_MINOR}.${probe_nanobind_PATCH}"
)
# Mirrors the nanobind 3.1.x and internals checks in python/CMakeLists.txt;
# update both on a nanobind bump.
if(NOT probe_nanobind_version MATCHES "^3[.]1[.][0-9]+$")
  message(
    FATAL_ERROR
    "The Eigen probe requires nanobind 3.1.x (found ${probe_nanobind_version}). "
    "Use a compatible nanobind package."
  )
endif()
file(
  STRINGS "${NB_DIR}/src/nb_internals.h"
  probe_nanobind_internals
  REGEX "^# *define NB_INTERNALS_VERSION "
)
if(NOT probe_nanobind_internals MATCHES "NB_INTERNALS_VERSION 22([^0-9]|$)")
  message(
    FATAL_ERROR
    "The Eigen probe requires nanobind 3.1.x with NB_INTERNALS_VERSION=22. "
    "Audit python/dartpy/detail/construction.cpp before changing nanobind."
  )
endif()
find_package(Eigen3 CONFIG REQUIRED)
if(NOT Eigen3_VERSION OR Eigen3_VERSION VERSION_LESS 3.3)
  if(NOT Eigen3_VERSION)
    set(Eigen3_VERSION "unknown")
  endif()
  message(
    FATAL_ERROR
    "Eigen3 >= 3.3 is required; found version ${Eigen3_VERSION}"
  )
endif()

get_filename_component(
  DART_SOURCE_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../.."
  ABSOLUTE
)
nanobind_add_module(numpy_export_probe NB_STATIC PROTECT_STACK probe.cpp)
target_compile_features(numpy_export_probe PRIVATE cxx_std_17)
target_include_directories(
  numpy_export_probe
  PRIVATE "${DART_SOURCE_ROOT}/python/dartpy"
)
target_link_libraries(numpy_export_probe PRIVATE Eigen3::Eigen)
if(MSVC)
  target_compile_options(numpy_export_probe PRIVATE /W4 /WX)
else()
  target_compile_options(numpy_export_probe PRIVATE -Wall -Wextra -Werror)
endif()

add_custom_target(
  check
  COMMAND
    "${Python_EXECUTABLE}" -I "${CMAKE_CURRENT_SOURCE_DIR}/../probe_eigen.py"
    --check-dir "$<TARGET_FILE_DIR:numpy_export_probe>"
  DEPENDS numpy_export_probe
  VERBATIM
)
