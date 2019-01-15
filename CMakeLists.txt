#===============================================================================
# CMake settings
#===============================================================================
if(MSVC)
  cmake_minimum_required(VERSION 3.8.0)
else()
  cmake_minimum_required(VERSION 3.5.1)
endif()

project(dart)

string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPERCASE)

# Use MACOSX_RPATH by default on OS X. This was added in CMake 2.8.12 and
# became default in CMake 3.0. Explicitly setting this policy is necessary to
# suppress a warning in CMake 3.0 and above.
if(POLICY CMP0042)
  cmake_policy(SET CMP0042 NEW)
endif()

# Simplify variable reference and escape sequence evaluation. This was added in
# CMake 3.1. Explicitly setting this policy is necessary to suppress a warning
# in CMake 3.1 and above.
if(POLICY CMP0053)
  cmake_policy(SET CMP0053 NEW)
endif()

# Variables used in Components.cmake
set(INCLUDE_INSTALL_DIR "include")
set(LIBRARY_INSTALL_DIR "lib")
set(CONFIG_INSTALL_DIR "share/${PROJECT_NAME}/cmake")

# Set relative location to install additional documentation (sample data,
# examples, and tutorials)
set(DART_ADDITIONAL_DOCUMENTATION_INSTALL_PATH "share/doc/${PROJECT_NAME}")

set(CMAKE_DEBUG_POSTFIX "d")
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

include(DARTMacros)

# CMake component helper. It uses the following variables:
# - LIBRARY_INSTALL_DIR
# - CONFIG_INSTALL_DIR
include(Components)
initialize_component_helpers(${PROJECT_NAME})

# System install paths on Windows
if(WIN32)
  set(CMAKE_INSTALL_PREFIX "C:/Golems" CACHE PATH "Install prefix" FORCE)
endif()

#===============================================================================
# Project settings
#===============================================================================

# If you change the version, please update the <version> tag in package.xml.
set(DART_MAJOR_VERSION "7")
set(DART_MINOR_VERSION "0")
set(DART_PATCH_VERSION "0")
set(DART_VERSION "${DART_MAJOR_VERSION}.${DART_MINOR_VERSION}.${DART_PATCH_VERSION}")
set(DART_PKG_DESC "Dynamic Animation and Robotics Toolkit.")
set(DART_PKG_EXTERNAL_DEPS "eigen, ccd, fcl, assimp, boost")

#===============================================================================
# Build options
#===============================================================================
option(DART_VERBOSE "Whether print detailed information in CMake process" OFF)
option(ENABLE_OPENMP "Build with OpenMP parallaization enabled" ON)
if(MSVC)
  set(DART_RUNTIME_LIBRARY "/MD" CACHE STRING "BaseName chosen by the user at CMake configure time")
  set_property(CACHE DART_RUNTIME_LIBRARY PROPERTY STRINGS /MD /MT)
  option(DART_MSVC_DEFAULT_OPTIONS "Build DART with default Visual Studio options" OFF)
else()
  option(BUILD_SHARED_LIBS "Build shared libraries" ON)
endif()
# Warning: DART_ENABLE_SIMD should be ON only when you build DART and the DART
# dependent projects on the same machine. If this option is on, then compile
# option `-march=native` is added to the target `dart` that enables all
# instruction subsets supported by the local machine. If the architecture of
# local machines are different then the projects will be built with different
# compile options, which may cause runtime errors especially memory alignment
# errors.
option(DART_ENABLE_SIMD
  "Build DART with all SIMD instructions on the current local machine" OFF)
option(DART_BUILD_GUI_OSG "Build osgDart library" ON)
option(DART_BUILD_EXTRAS "Build extra projects" OFF)
option(DART_CODECOV "Turn on codecov support" OFF)
option(DART_TREAT_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)
option(DART_FAST_DEBUG "Add -O1 option for DEBUG mode build" OFF)

#===============================================================================
# Print intro
#===============================================================================
message(STATUS "")
message(STATUS "============================================")
message(STATUS "                DART ${DART_VERSION}")
message(STATUS "============================================")

#===============================================================================
# CodeCov settings
#===============================================================================
if(DART_CODECOV)
  include(CodeCoverage)
  setup_target_for_coverage(codecov ctest coverage)
  set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -g -O0 -fprofile-arcs -ftest-coverage"
  )
endif()

#===============================================================================
# Build type settings
#===============================================================================
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: Debug | Release | RelWithDebInfo | MinSizeRel" FORCE)
endif()
string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE_UPPERCASE)

set(BUILD_TYPE_DEBUG FALSE)
set(BUILD_TYPE_RELEASE FALSE)
set(BUILD_TYPE_RELWITHDEBINFO FALSE)
set(BUILD_TYPE_MINSIZEREL FALSE)

if("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "DEBUG")
  set(BUILD_TYPE_DEBUG TRUE)
elseif("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "RELEASE")
  set(BUILD_TYPE_RELEASE TRUE)
elseif("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "RELWITHDEBINFO")
  set(BUILD_TYPE_RELWITHDEBINFO TRUE)
elseif("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "MINSIZEREL")
  set(BUILD_TYPE_MINSIZEREL TRUE)
else()
  message(STATUS "CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} unknown. Valid options are: Debug | Release | RelWithDebInfo | MinSizeRel")
endif()

#===============================================================================
# Find dependencies
#===============================================================================
include(DARTFindDependencies)

#===============================================================================
# Check for non-case-sensitive filesystems
#===============================================================================
execute_process(COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/tools/case_sensitive_filesystem
                RESULT_VARIABLE FILESYSTEM_CASE_SENSITIVE_RETURN)
if(${FILESYSTEM_CASE_SENSITIVE_RETURN} EQUAL 0)
  set(FILESYSTEM_CASE_SENSITIVE TRUE)
else()
  set(FILESYSTEM_CASE_SENSITIVE FALSE)
endif()

#===============================================================================
# Compiler flags
#===============================================================================
if(MSVC)

  # Visual Studio enables c++11 support by default
  if(MSVC_VERSION VERSION_LESS 1900)
    message(FATAL_ERROR "${PROJECT_NAME} requires VS2017 or greater.")
  endif()
  if(DART_TREAT_WARNINGS_AS_ERRORS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /WX")
  endif()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /permissive-")
  #set(CMAKE_EXE_LINKER_FLAGS_RELEASE "/LTCG /INCREMENTAL:NO")
  if(NOT DART_MSVC_DEFAULT_OPTIONS)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${DART_RUNTIME_LIBRARY}d /Zi /Gy /W1 /EHsc")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${DART_RUNTIME_LIBRARY} /Zi /GL /Gy /W1 /EHsc")
  endif(NOT DART_MSVC_DEFAULT_OPTIONS)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
  add_compile_options(/wd4334)
  add_compile_options(/wd4267)
  add_compile_options(/wd4244)
  add_compile_options(/wd4250)
  add_compile_options(/wd4996)
  add_compile_options(/wd4099)
  add_compile_options(/wd4305)
  add_compile_options(/wd4838)
  add_compile_options(/bigobj)

elseif(CMAKE_COMPILER_IS_GNUCXX)

  if(DART_TREAT_WARNINGS_AS_ERRORS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")
  endif()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -fPIC")
  execute_process(
    COMMAND ${CMAKE_CXX_COMPILER} -dumpfullversion -dumpversion OUTPUT_VARIABLE GCC_VERSION)
  set(CXX_COMPILER_VERSION ${GCC_VERSION})
  if(GCC_VERSION VERSION_LESS 4.8)
    message(FATAL_ERROR "The installed g++ version is ${GCC_VERSION}. ${PROJECT_NAME} requires g++ 4.8 or greater.")
  endif()
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
  set(CMAKE_CXX_FLAGS_DEBUG "-g -fno-omit-frame-pointer -fno-inline-functions -fno-inline-functions-called-once -fno-optimize-sibling-calls")
  if(DART_FAST_DEBUG)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O1")
  endif()
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS_DEBUG}")
  set(CMAKE_CXX_FLAGS_PROFILE "${CMAKE_CXX_FLAGS_DEBUG} -pg")
  set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined")

elseif("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")

  if(DART_TREAT_WARNINGS_AS_ERRORS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Wno-error=deprecated-declarations")
    # Turn warning "deprecated-declarations" into an warning even if -Werror is
    # specified until we abandon glut.
  endif()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
  execute_process(
    COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE CLANG_VERSION)
  set(CXX_COMPILER_VERSION ${CLANG_VERSION})
  if(CLANG_VERSION VERSION_LESS 3.3)
    message(FATAL_ERROR "The installed Clang version is ${CLANG_VERSION}. ${PROJECT_NAME} requires clang 3.3 or greater.")
  endif()
  if("${CMAKE_SYSTEM_NAME}" MATCHES "Darwin")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
  endif()
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
  set(CMAKE_CXX_FLAGS_DEBUG "-g -fno-omit-frame-pointer -fno-inline-functions -fno-optimize-sibling-calls")
  if(DART_FAST_DEBUG)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O1")
  endif()
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS_DEBUG}")
  set(CMAKE_CXX_FLAGS_PROFILE "${CMAKE_CXX_FLAGS_DEBUG} -pg")
  set(CMAKE_OSX_DEPLOYMENT_TARGET "10.8")

else()

  message(SEND_ERROR "Compiler[${CMAKE_CXX_COMPILER_ID}] not supported.")

endif()

#===============================================================================
# Print build summary
#===============================================================================
if(DART_VERBOSE)
  message(STATUS "")
  message(STATUS "[ Build summary ]")
  message(STATUS "CMAKE_GENERATOR  : ${CMAKE_GENERATOR}")
  message(STATUS "Compiler ID      : ${CMAKE_CXX_COMPILER_ID}")
  message(STATUS "Compiler version : ${CXX_COMPILER_VERSION}")
  message(STATUS "Build type       : ${CMAKE_BUILD_TYPE}")
  message(STATUS "BUILD_SHARED_LIBS: ${BUILD_SHARED_LIBS}")
  message(STATUS "ENABLE_OPENMP    : ${ENABLE_OPENMP}")
  message(STATUS "Build gui::osg   : ${DART_BUILD_GUI_OSG}")
  message(STATUS "Install path     : ${CMAKE_INSTALL_PREFIX}")
  message(STATUS "CXX_FLAGS        : ${CMAKE_CXX_FLAGS}")
  if(${CMAKE_BUILD_TYPE_UPPERCASE} STREQUAL "RELEASE")
    message(STATUS "CXX_FLAGS_RELEASE: ${CMAKE_CXX_FLAGS_RELEASE}")
  elseif(${CMAKE_BUILD_TYPE_UPPERCASE} STREQUAL "DEBUG")
    message(STATUS "CXX_FLAGS_DEBUG  : ${CMAKE_CXX_FLAGS_DEBUG}")
  elseif(${CMAKE_BUILD_TYPE_UPPERCASE} STREQUAL "RELWITHDEBINFO")
    message(STATUS "CXX_FLAGS_RELWITHDEBINFO: ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
  elseif(${CMAKE_BUILD_TYPE_UPPERCASE} STREQUAL "PROFILE")
    message(STATUS "CXX_FLAGS_PROFILE: ${CMAKE_CXX_FLAGS_PROFILE}")
  endif()
  message(STATUS "CMAKE_SOURCE_DIR : ${CMAKE_SOURCE_DIR}")
  message(STATUS "CMAKE_BINARY_DIR : ${CMAKE_BINARY_DIR}")
endif(DART_VERBOSE)

#===============================================================================
# Add sub-directories
#===============================================================================

add_subdirectory(dart)

set(DART_IN_SOURCE_BUILD TRUE)

if(TARGET dart)

  # Add a "tests" target to build unit tests.
  enable_testing()
  add_subdirectory(unittests EXCLUDE_FROM_ALL)

  # Add example subdirectories and an "examples" target.
  add_subdirectory(examples EXCLUDE_FROM_ALL)
  get_property(examples GLOBAL PROPERTY DART_EXAMPLES)
  add_custom_target(examples DEPENDS ${examples})

  if(DART_VERBOSE)
    message(STATUS "")
    message(STATUS "[ Examples ]")
    foreach(example ${examples})
      message(STATUS "Adding example: ${example}")
    endforeach(example ${examples})
  else(DART_VERBOSE)
    list(LENGTH examples examples_length)
    message(STATUS "Adding ${examples_length} examples")
  endif(DART_VERBOSE)

  # Add a "tutorials" target to build tutorials.
  add_subdirectory(tutorials EXCLUDE_FROM_ALL)
  get_property(tutorials GLOBAL PROPERTY DART_TUTORIALS)
  add_custom_target(tutorials DEPENDS ${tutorials})

  if(DART_VERBOSE)
    message(STATUS "")
    message(STATUS "[ Tutorials ]")
    foreach(tutorial ${tutorials})
      message(STATUS "Adding tutorial: ${tutorial}")
    endforeach(tutorial ${tutorials})
  else(DART_VERBOSE)
    list(LENGTH tutorials tutorials_length)
    message(STATUS "Adding ${tutorials_length} tutorials")
  endif(DART_VERBOSE)

endif()

if(DART_BUILD_EXTRAS)
  add_subdirectory(extras)
endif()

#===============================================================================
# CMake configuration files for components and targets
#===============================================================================
# Generate and install CMake configuration files for each component <C>:
# - <C>Component.cmake, which defines:
#   - dart_<C>_DEPENDENCIES: list of component dependencies
#   - dart_<C>_LIBRARIES: list of library targets in this component
# - <C>Targets.cmake, which creates IMPORTED targets
install_component_exports(${PROJECT_NAME})

#===============================================================================
# Configure files
#===============================================================================
if(DART_VERBOSE)
  message(STATUS "")
  message(STATUS "[ Configured files ]")
endif()

# Generate and install a Config.cmake file. This file includes the
# <C>Component.cmake and <C>Targets.cmake created above. It also uses the
# following variables:
#
# - PACKAGE_INCLUDE_INSTALL_DIR
# - PACKAGE_INCLUDE_DIRS
get_property(PACKAGE_INCLUDE_DIRS GLOBAL
  PROPERTY "${PROJECT_NAME_UPPERCASE}_INCLUDE_DIRS")

# Generate the DART CMake Config and version files
include(CMakePackageConfigHelpers)
set(DART_CONFIG_IN ${CMAKE_SOURCE_DIR}/cmake/${PROJECT_NAME_UPPERCASE}Config.cmake.in)
set(DART_CONFIG_OUT ${CMAKE_BINARY_DIR}/${PROJECT_NAME_UPPERCASE}Config.cmake)
set(DART_VERSION_OUT ${CMAKE_BINARY_DIR}/cmake/${PROJECT_NAME_UPPERCASE}ConfigVersion.cmake)
if(DART_VERBOSE)
  message(STATUS ${DART_CONFIG_OUT})
  message(STATUS ${DART_VERSION_OUT})
endif()
configure_package_config_file(
  ${DART_CONFIG_IN}
  ${DART_CONFIG_OUT}
  INSTALL_DESTINATION "${CONFIG_INSTALL_DIR}"
  PATH_VARS INCLUDE_INSTALL_DIR
)
write_basic_config_version_file(
  ${DART_VERSION_OUT}
  VERSION ${${PROJECT_NAME_UPPERCASE}_VERSION}
  COMPATIBILITY SameMajorVersion
)
install(
  FILES ${DART_CONFIG_OUT} ${DART_VERSION_OUT}
  DESTINATION "${CONFIG_INSTALL_DIR}")

# Generate the DART pkg-config
set(PC_CONFIG_IN ${CMAKE_SOURCE_DIR}/cmake/dart.pc.in)
set(PC_CONFIG_OUT ${CMAKE_BINARY_DIR}/cmake/dart.pc)
if(DART_VERBOSE)
  message(STATUS ${PC_CONFIG_OUT})
endif()
configure_file(${PC_CONFIG_IN} ${PC_CONFIG_OUT} @ONLY)
install(FILES ${PC_CONFIG_OUT} DESTINATION lib/pkgconfig)

# Install a Catkin 'package.xml' file. This is required by REP-136.
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

#===============================================================================
# Install sample data, examples, and tutorials
#===============================================================================

# Sample data
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/data"
  DESTINATION ${DART_ADDITIONAL_DOCUMENTATION_INSTALL_PATH})

# Examples source
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/examples"
  DESTINATION ${DART_ADDITIONAL_DOCUMENTATION_INSTALL_PATH})

# Tutorials source
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tutorials"
  DESTINATION ${DART_ADDITIONAL_DOCUMENTATION_INSTALL_PATH})

#===============================================================================
# Uninstall
#===============================================================================

# Add an "uninstall" target
# Ref: http://www.cmake.org/Wiki/CMake_FAQ#Can_I_do_.22make_uninstall.22_with_CMake.3F
configure_file("${PROJECT_SOURCE_DIR}/cmake/uninstall_target.cmake.in" "${PROJECT_BINARY_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P "${PROJECT_BINARY_DIR}/uninstall_target.cmake")

#===============================================================================
# Code Formatting
#===============================================================================
if(DART_VERBOSE)
  message(STATUS "")
  message(STATUS "[ Code Formatting ]")
endif()

find_program(
  CLANG_FORMAT_EXECUTABLE
  NAMES clang-format-6.0
)

get_property(formatting_files GLOBAL PROPERTY DART_FORMAT_FILES)
list(LENGTH formatting_files formatting_files_length)

if(CLANG_FORMAT_EXECUTABLE)

  if(DART_VERBOSE)
    message(STATUS "Looking for clang-format - found")
  endif()

  message(STATUS "Formatting on ${formatting_files_length} source files.")

  if(formatting_files)
    add_custom_target(format
                      COMMAND ${CMAKE_COMMAND} -E echo "Formatting ${formatting_files_length} files... "
                      COMMAND ${CLANG_FORMAT_EXECUTABLE} -style=file -i ${formatting_files}
                      COMMAND ${CMAKE_COMMAND} -E echo "Done."
                      DEPENDS ${CLANG_FORMAT_EXECUTABLE}
                      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/dart)

    add_custom_target(check-format
                      COMMAND ${CMAKE_COMMAND} -E echo "Checking ${formatting_files_length} files... "
                      COMMAND ${CMAKE_SOURCE_DIR}/tools/check_format.sh ${CLANG_FORMAT_EXECUTABLE} ${formatting_files}
                      COMMAND ${CMAKE_COMMAND} -E echo "Done."
                      DEPENDS ${CLANG_FORMAT_EXECUTABLE}
                      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/dart)
  else()
    add_custom_target(format
                      COMMAND ${CMAKE_COMMAND} -E echo "Warning: Not found any source files to format.")

    add_custom_target(check-format
                      COMMAND ${CMAKE_COMMAND} -E echo "Warning: Not found any source files to check.")
  endif()

else()

  if(DART_VERBOSE)
    message(STATUS "Looking for clang-format - NOT found, please install clang-format to enable automatic code formatting")
  endif()

endif()

#===============================================================================
# API Document using Doxygen
# References:
#   http://mementocodex.wordpress.com/2013/01/19/how-to-generate-code-documentation-with-doxygen-and-cmake-a-slightly-improved-approach/
#   http://www.cmake.org/pipermail/cmake/2007-February/012796.html
#===============================================================================
if(DOXYGEN_FOUND)

  set(DOXYGEN_DOXYFILE_IN      "${PROJECT_SOURCE_DIR}/doxygen/Doxyfile.in"    )
  set(DOXYGEN_DOXYFILE         "${PROJECT_BINARY_DIR}/doxygen/Doxyfile"       )
  set(DOXYGEN_HTML_INDEX       "${PROJECT_BINARY_DIR}/doxygen/html/index.html")
  set(DOXYGEN_OUTPUT_ROOT      "${PROJECT_BINARY_DIR}/doxygen/html"           )
  set(DOXYGEN_GENERATE_TAGFILE "${DOXYGEN_OUTPUT_ROOT}/${PROJECT_NAME}.tag"   )
  set(DOXYGEN_INCLUDE_PATH     "${PROJECT_SOURCE_DIR}"                        )
  set(DOXYGEN_INPUT_ROOT       "${PROJECT_SOURCE_DIR}/dart"                   )
  set(DOXYGEN_EXTRA_INPUTS     "${PROJECT_SOURCE_DIR}/doxygen/mainpage.dox"   )
  set(DOXYGEN_EXCLUDE          "${PROJECT_SOURCE_DIR}/dart/external"          )
  set(DOXYGEN_STRIP_FROM_PATH  "${CMAKE_CURRENT_SOURCE_DIR}"                  )

  # Generate a Doxyfile. This uses the variables:
  #
  # - DOXYGEN_OUTPUT_ROOT
  # - DOXYGEN_GENERATE_TAGFILE
  # - DOXYGEN_EXTRA_INPUTS
  # - DOXYGEN_INPUT_ROOT
  # - DOXYGEN_EXCLUDE
  # - DOXYGEN_STRIP_FROM_PATH
  configure_file(${DOXYGEN_DOXYFILE_IN} ${DOXYGEN_DOXYFILE} @ONLY)
  file(
    COPY "${PROJECT_SOURCE_DIR}/doxygen/DART logo.png"
    DESTINATION ${DOXYGEN_OUTPUT_ROOT}
  )
  add_custom_command(
    OUTPUT ${DOXYGEN_HTML_INDEX}
    COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
    COMMAND ${DOXYGEN_EXECUTABLE} -u ${DOXYGEN_DOXYFILE}
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_DOXYFILE}
    # Strip path prefix from all paths in dart.tag
    COMMAND ${CMAKE_COMMAND} -E echo "Stripping paths from"
        "${DOXYGEN_GENERATE_TAGFILE}"
    COMMAND sed -i s:${DOXYGEN_STRIP_FROM_PATH}::g ${DOXYGEN_GENERATE_TAGFILE}
    # Strip all doxygen="path" HTML tags
    COMMAND ${CMAKE_COMMAND} -E echo "Stripping Doxygen HTML tags"
    COMMAND find "${DOXYGEN_OUTPUT_ROOT}" -type f -name "*.html"
        -exec sed -i 's: doxygen=\"[^\"]*\"::g' {} \\$<SEMICOLON>
    COMMAND ${CMAKE_COMMAND} -E echo "Done."
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/doxygen
    DEPENDS ${DOXYGEN_DOXYFILE}
  )
  # add_custom_target(docs ALL DEPENDS ${DOXYGEN_HTML_INDEX})
  add_custom_target(docs DEPENDS ${DOXYGEN_HTML_INDEX})
  add_custom_target(
    docs_forced
    COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
    COMMAND ${DOXYGEN_EXECUTABLE} -u ${DOXYGEN_DOXYFILE}
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_DOXYFILE}
    COMMAND ${CMAKE_COMMAND} -E echo "Done."
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/doxygen
  )

  # Add the "view_docs" target that opens the generated API documentation.
  if(APPLE)
    set(OPEN_COMMAND "open")
  else()
    set(OPEN_COMMAND "xdg-open")
  endif()

  add_custom_target(view_docs "${OPEN_COMMAND}" "${DOXYGEN_HTML_INDEX}"
    DEPENDS "${DOXYGEN_HTML_INDEX}"
    COMMENT "Opening documentation in a web browser.")

endif()

#===============================================================================
# Coloring build outputs using gccfilter if appliciable
# Ref: http://stackoverflow.com/questions/14399984/make-cmake-use-gccfilter
#===============================================================================
if("${PERLMODULES_FOUND}" STREQUAL "TRUE")
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_GNUCPP)
    option(COLOR_GCC "Use GCCFilter to color compiler output messages" ON)
    set(COLOR_GCC_OPTIONS "-c -r -w" CACHE STRING "Arguments that are passed to gccfilter when output coloring is switchend on. Defaults to -c -r -w.")
    if(COLOR_GCC)
      set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${PROJECT_SOURCE_DIR}/tools/gccfilter ${COLOR_GCC_OPTIONS}")
    endif()
  endif()
endif()

#===============================================================================
# Build Instructions
#===============================================================================
message(STATUS "")
message(STATUS "Run 'make' to build all the components")
message(STATUS "Run 'make tests' to build all the unittests")
message(STATUS "Run 'make examples' to build all the examples")
message(STATUS "Run 'make tutorials' to build all the tutorials")
message(STATUS "Run 'make view_docs' to see the API documentation")

#===============================================================================
# END
#===============================================================================
message(STATUS "")
