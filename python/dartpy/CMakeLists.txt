if(NOT DARTPY_PYTHON_VERSION)
  set(DARTPY_PYTHON_VERSION 3.4 CACHE STRING "Choose the target Python version (e.g., 3.4, 2.7)" FORCE)
endif()

find_package(PythonInterp ${DARTPY_PYTHON_VERSION} QUIET)
execute_process(COMMAND ${PYTHON_EXECUTABLE} -c
  "from distutils.sysconfig import get_python_lib;\
  print(get_python_lib(plat_specific=True, prefix=''))"
  OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NOT PythonInterp_FOUND)
  message(STATUS "DART_BUILD_DARTPY is ON, but failed to find PythonInterp. "
    "Disabling dartpy."
  )
  return()
endif()

find_package(PythonLibs ${DARTPY_PYTHON_VERSION} QUIET)
if(NOT PythonLibs_FOUND)
  message(STATUS "DART_BUILD_DARTPY is ON, but failed to find PythonLibs. "
    "Disabling dartpy."
  )
  return()
endif()

# Find pybind11
# Needs to set PYBIND11_PYTHON_VERSION before finding pybind11
set(PYBIND11_PYTHON_VERSION ${DARTPY_PYTHON_VERSION})
find_package(pybind11 2.2.0 QUIET)
if(NOT pybind11_FOUND)
  message(STATUS "DART_BUILD_DARTPY is ON, but failed to find pybind11 >= "
    "2.2.0. Disabling dartpy."
  )
  return()
endif()

file(GLOB_RECURSE dartpy_headers "*.hpp")
file(GLOB_RECURSE dartpy_sources "*.cpp")

# Build a Python extension module:
# pybind11_add_module(<name> [MODULE | SHARED] [EXCLUDE_FROM_ALL]
#                     [NO_EXTRAS] [SYSTEM] [THIN_LTO] source1 [source2 ...])
#
pybind11_add_module(dartpy
  MODULE
  ${dartpy_headers}
  ${dartpy_sources}
)

target_include_directories(dartpy
  SYSTEM PUBLIC
    ${PYTHON_INCLUDE_DIRS}
    ${pybind11_INCLUDE_DIRS}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(dartpy
  PUBLIC
    dart
    dart-utils
    dart-utils-urdf
    dart-gui
    dart-gui-osg
    ${PYTHON_LIBRARIES}
)
if(TARGET dart-collision-bullet)
  target_link_libraries(dartpy PUBLIC dart-collision-bullet)
endif()

set_target_properties(dartpy
  PROPERTIES
    PREFIX ""
    SUFFIX ".so"  # python uses '.so' extension even on macOS
    DEBUG_POSTFIX ""
)

install(TARGETS dartpy
  LIBRARY DESTINATION "${PYTHON_SITE_PACKAGES}"
)

list(REMOVE_ITEM dartpy_headers
  eigen_geometry_pybind.h
  eigen_pybind.h
)
list(REMOVE_ITEM dartpy_sources
  eigen_geometry_pybind.cpp
)
dart_format_add(${dartpy_headers} ${dartpy_sources})
