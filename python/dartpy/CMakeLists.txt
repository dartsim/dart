if(NOT DARTPY_PYTHON_VERSION)
  set(DARTPY_PYTHON_VERSION 3.4 CACHE STRING "Choose the target Python version (e.g., 3.4, 2.7)" FORCE)
endif()

find_package(PythonInterp ${DARTPY_PYTHON_VERSION} REQUIRED)
execute_process(COMMAND ${PYTHON_EXECUTABLE} -c
  "from distutils.sysconfig import get_python_lib;\
  print(get_python_lib(plat_specific=True, prefix=''))"
  OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

find_package(PythonLibs ${DARTPY_PYTHON_VERSION} REQUIRED)

# Find pybind11
# Needs to set PYBIND11_PYTHON_VERSION before finding pybind11
set(PYBIND11_PYTHON_VERSION ${DARTPY_PYTHON_VERSION})
find_package(pybind11 2.2.0 REQUIRED)

set(dartpy_headers
  eigen_geometry_pybind.h
  eigen_pybind.h
)
set(dartpy_sources
  eigen_geometry_pybind.cpp
  dartpy.cpp
  common/common_module.cpp
  common/Uri.cpp
  dynamics/dynamics_module.cpp
  dynamics/Skeleton.cpp
  simulation/simulation_module.cpp
  simulation/World.cpp
  utils/utils_module.cpp
  utils/DartLoader.cpp
)

# Build a Python extension module:
# pybind11_add_module(<name> [MODULE | SHARED] [EXCLUDE_FROM_ALL]
#                     [NO_EXTRAS] [SYSTEM] [THIN_LTO] source1 [source2 ...])
#
pybind11_add_module(dartpy
  MODULE
  EXCLUDE_FROM_ALL
  ${dartpy_headers}
  ${dartpy_sources}
)

target_include_directories(dartpy
  SYSTEM PUBLIC
    ${PYTHON_INCLUDE_DIRS}
    ${pybind11_INCLUDE_DIRS}
)

target_link_libraries(dartpy
  PUBLIC
    dart
    dart-collision-bullet
    dart-utils
    dart-utils-urdf
    ${PYTHON_LIBRARIES}
)

set_target_properties(dartpy
  PROPERTIES
    PREFIX ""
    SUFFIX ".so"  # python uses '.so' extension even on macOS
    DEBUG_POSTFIX ""
)

install(TARGETS dartpy
  LIBRARY DESTINATION "${PYTHON_SITE_PACKAGES}"
)

dart_format_add(${dartpy_headers} ${dartpy_sources})
