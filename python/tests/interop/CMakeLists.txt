# A pybind11 extension that exchanges DART objects with the nanobind dartpy
# through its C API. test_pybind11_interop.py skips when it is not built.
# Both modules must load the same shared DART libraries.
if(NOT BUILD_SHARED_LIBS)
  message(STATUS "Skipping the dartpy pybind11 interop test: static DART")
  return()
endif()
execute_process(
  COMMAND
    "${Python_EXECUTABLE}" -c "import pybind11; print(pybind11.get_cmake_dir())"
  OUTPUT_VARIABLE dartpy_pybind11_cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET
)
set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG QUIET HINTS "${dartpy_pybind11_cmake_dir}")
if(NOT pybind11_FOUND)
  message(STATUS "Skipping the dartpy pybind11 interop test: no pybind11")
  return()
endif()

pybind11_add_module(
  dartpy_pybind11_interop_test
  MODULE
  pybind11_interop_test.cpp
)
set_target_properties(
  dartpy_pybind11_interop_test
  PROPERTIES
    EXCLUDE_FROM_ALL TRUE
    LIBRARY_OUTPUT_DIRECTORY "${DART_PYTHON_BUILD_DIR}/dartpy"
)
target_include_directories(
  dartpy_pybind11_interop_test
  PRIVATE "${PROJECT_SOURCE_DIR}/python/include"
)
target_link_libraries(dartpy_pybind11_interop_test PRIVATE dart)
add_dependencies(pytest dartpy_pybind11_interop_test)
if(TARGET py_test_pybind11_interop)
  add_dependencies(py_test_pybind11_interop dartpy_pybind11_interop_test)
endif()
dart_format_add(pybind11_interop_test.cpp)
