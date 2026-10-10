cmake_minimum_required(VERSION 3.20)
project(nanobind_eigen_probe LANGUAGES CXX)

find_package(Python 3.10 REQUIRED COMPONENTS Interpreter Development.Module)
execute_process(
  COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_VARIABLE nanobind_cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE
  COMMAND_ERROR_IS_FATAL ANY
)
list(APPEND CMAKE_PREFIX_PATH "${nanobind_cmake_dir}")
find_package(nanobind 3.1 CONFIG REQUIRED)
find_package(Eigen3 3.3 CONFIG REQUIRED)

get_filename_component(
  DART_SOURCE_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../.."
  ABSOLUTE
)
nanobind_add_module(numpy_export_probe NB_STATIC PROTECT_STACK probe.cpp)
target_compile_features(numpy_export_probe PRIVATE cxx_std_17)
target_include_directories(
  numpy_export_probe
  PRIVATE "${DART_SOURCE_ROOT}/python/dartpy_nanobind"
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
