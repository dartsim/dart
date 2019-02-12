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

# Find boost with python components. The name of python component varies
# depending on the platform, boost version, and python version.
if(APPLE)
  find_package(Boost REQUIRED
    COMPONENTS
      python${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR} thread
  )
else() # LINUX assumed
  if(${PYTHON_VERSION_MAJOR} EQUAL 3)
    find_package(Boost
      COMPONENTS
        python-py${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR} thread
    )
    if(NOT Boost_FOUND)
      find_package(Boost REQUIRED COMPONENTS python3 thread)
    endif()
  else() # Python 2 assumed
    find_package(Boost REQUIRED COMPONENTS python thread)
  endif()
endif()

include_directories(SYSTEM
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}/include
  ${PYTHON_INCLUDE_DIRS}
)

#===================
# Chimera Settings
#===================

find_package(chimera QUIET)

# Check whether the CXX compiler and boost::python support get_pointer for
# std::shared_ptr<T> references.
include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_DEFINITIONS "")
set(CMAKE_REQUIRED_FLAGS "-std=c++11")
set(CMAKE_REQUIRED_INCLUDES
  ${Boost_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)
set(CMAKE_REQUIRED_LIBRARIES
  ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES}
)
set(CMAKE_REQUIRED_QUIET FALSE)

check_cxx_source_compiles(
  "
  #include <memory>
  #include <boost/get_pointer.hpp>
  int main() { std::shared_ptr<int> ptr; int *p = boost::get_pointer(ptr); }
  "
  DARTPY_HAS_STD_SHARED_GET_POINTER
)

unset(CMAKE_REQUIRED_QUIET)
unset(CMAKE_REQUIRED_DEFINITIONS)
unset(CMAKE_REQUIRED_FLAGS)
unset(CMAKE_REQUIRED_INCLUDES)
unset(CMAKE_REQUIRED_LIBRARIES)

# Generate a header file for the DARTPY_HAS_STD_SHARED_GET_POINTER flag.
configure_file("cmake/config.h.in"
  "include/dartpy/config.h"
)

# Check if DART is built with octomap so that it supports VoxelGridShape
include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_DEFINITIONS "")
set(CMAKE_REQUIRED_FLAGS "-std=c++11")
set(CMAKE_REQUIRED_INCLUDES ${DART_INCLUDE_DIRS})
set(CMAKE_REQUIRED_LIBRARIES ${DART_LIBRARIES})
check_cxx_source_compiles(
  "
  #include <dart/dart.hpp>
  int main()
  {
    auto voxel = new dart::dynamics::VoxelGridShape();
    delete voxel;
    return 0;
  }
  "
  DART_HAS_VOXELGRIDSHAPE
)

set(CHIMERA_CONFIG "${CMAKE_CURRENT_SOURCE_DIR}/chimera/chimera.yml")
# if(DART_HAS_VOXELGRIDSHAPE)
#   set(CHIMERA_CONFIG "${CMAKE_CURRENT_SOURCE_DIR}/chimera/chimera.yml")
# else()
#   set(CHIMERA_CONFIG "${CMAKE_CURRENT_SOURCE_DIR}/chimera/chimera_without_voxel.yml")
# endif()
if(chimera_FOUND)
  add_chimera_binding(TARGET "dartpy_CHIMERA"
    MODULE "dartpy"
    CONFIGURATION "${CHIMERA_CONFIG}"
    SOURCES "chimera/chimera.hpp"
    DESTINATION "${CMAKE_CURRENT_SOURCE_DIR}/src/generated"
    DEBUG EXCLUDE_FROM_ALL
  )
  target_include_directories("dartpy_CHIMERA"
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
  )
  target_link_libraries("dartpy_CHIMERA"
    PUBLIC
      ${Boost_LIBRARIES}
      ${PYTHON_LIBRARIES}
      dart
      dart-collision-bullet
      dart-collision-ode
      dart-optimizer-nlopt
      dart-planning
      dart-utils
      dart-utils-urdf
      dart-gui
  )
  add_custom_target(binding DEPENDS "dartpy_CHIMERA_REBUILD")
else()
  add_custom_target(binding
    COMMAND ${CMAKE_COMMAND} -E cmake_echo_color --red "ERROR: Cannot generate bindings without \\'chimera\\' installed."
    COMMAND false
  )
endif()

# Create a target to build generated bindings.
set(SOURCES_TXT "${CMAKE_CURRENT_SOURCE_DIR}/src/generated/sources.txt")
if(EXISTS "${SOURCES_TXT}")
  file(STRINGS "${SOURCES_TXT}" SOURCES_GENERATED_RELATIVE NO_HEX_CONVERSION)

  set(SOURCES_GENERATED)
  foreach(relative_path ${SOURCES_GENERATED_RELATIVE})
    list(APPEND SOURCES_GENERATED "${CMAKE_CURRENT_SOURCE_DIR}/src/generated/${relative_path}")
  endforeach()

  add_library("dartpy" MODULE
    ${SOURCES_GENERATED}
    src/BodyNode.cpp
    src/Skeleton.cpp
    src/skel_parser.cpp
    src/template_registry.cpp
  )
  target_include_directories("dartpy"
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
  )
  target_link_libraries("dartpy"
    PUBLIC
      ${Boost_LIBRARIES}
      ${PYTHON_LIBRARIES}
      dart
      dart-collision-bullet
      dart-collision-ode
      dart-optimizer-nlopt
      dart-planning
      dart-utils
      dart-utils-urdf
      dart-gui
  )
  set_target_properties("dartpy" PROPERTIES
    PREFIX ""
    SUFFIX ".so"  # python uses '.so' extension even on macOS
  )
  if(CMAKE_COMPILER_IS_GNUCXX)
    target_compile_options("dartpy"
      PRIVATE -fabi-version=6
    )
  endif()
  target_compile_options("dartpy" PRIVATE -fvisibility=hidden)

  # dart_format_add(
  #   ${SOURCES_GENERATED}
  #   src/BodyNode.cpp
  #   src/Skeleton.cpp
  #   src/skel_parser.cpp
  #   src/template_registry.cpp
  # )

  install(TARGETS "dartpy"
    LIBRARY DESTINATION "${PYTHON_SITE_PACKAGES}"
  )
else()
  add_custom_target(dartpy ALL
    COMMAND ${CMAKE_COMMAND} -E cmake_echo_color --red "ERROR: Generate bindings using \\'make binding\\' before attempting to build."
    COMMAND false
  )
endif()
