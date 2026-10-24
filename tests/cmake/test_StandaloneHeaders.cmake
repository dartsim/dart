# Child mode: stage one install component and keep its manifest. The build
# tree's own manifest is shared with test_HeadersComponent, which can rewrite
# it while CTest runs both tests at once.
if(DEFINED CMAKE_INSTALL_COMPONENT)
  include("${DART_BINARY_DIR}/cmake_install.cmake")
  file(WRITE "${MANIFEST}" "${CMAKE_INSTALL_MANIFEST_FILES}")
  return()
endif()

cmake_minimum_required(VERSION 3.22.1)

# These headers hold definitions for the parent that includes them at its end
# (common/Signal.hpp for detail/Signal.hpp, detail/AllocatorMemoryLayout.hpp
# for the two layout files, and each -impl.hpp for its namesake), so compiling
# that parent covers them.
set(
  parent_only_headers
  dart/common/detail/FrameAllocatorMemoryLayout-impl.hpp
  dart/common/detail/FreeListAllocatorMemoryLayout-impl.hpp
  dart/common/detail/Signal.hpp
  dart/common/detail/StlAllocator-impl.hpp
  dart/common/detail/Stopwatch-impl.hpp
  dart/dynamics/detail/HeightmapShape-impl.hpp
  dart/lcpsolver/dantzig/DantzigMatrix-impl.hpp
  dart/math/detail/Icosphere-impl.hpp
  dart/math/detail/Random-impl.hpp
)

file(REMOVE_RECURSE "${TEST_BINARY_DIR}")
set(prefix "${TEST_BINARY_DIR}/install")
# A DESTDIR in the environment would stage the files somewhere else.
unset(ENV{DESTDIR})
# The default component is the C++ package without dartpy; the headers
# component installs every header.
foreach(component Unspecified headers)
  execute_process(
    COMMAND
      "${CMAKE_COMMAND}" "-DDART_BINARY_DIR=${DART_BINARY_DIR}"
      "-DCMAKE_INSTALL_PREFIX=${prefix}"
      "-DCMAKE_INSTALL_CONFIG_NAME=${TEST_CONFIG}"
      "-DCMAKE_INSTALL_COMPONENT=${component}"
      "-DMANIFEST=${TEST_BINARY_DIR}/${component}.txt" -P
      "${CMAKE_CURRENT_LIST_FILE}"
    RESULT_VARIABLE result
    OUTPUT_VARIABLE output
    ERROR_VARIABLE error
  )
  if(NOT result EQUAL 0)
    message(FATAL_ERROR "Installing ${component} failed:\n${output}\n${error}")
  endif()
endforeach()

file(READ "${TEST_BINARY_DIR}/headers.txt" installed_files)
set(headers)
foreach(file IN LISTS installed_files)
  file(RELATIVE_PATH header "${prefix}/include" "${file}")
  if(NOT header MATCHES "^\\.\\./")
    list(APPEND headers "${header}")
  endif()
endforeach()
foreach(header IN LISTS parent_only_headers)
  if(NOT header IN_LIST headers)
    message(
      FATAL_ERROR
      "${header} is no longer installed; remove it from parent_only_headers"
    )
  endif()
endforeach()
list(REMOVE_ITEM headers ${parent_only_headers})

set(consumer "${TEST_BINARY_DIR}/consumer")
foreach(header IN LISTS headers)
  file(WRITE "${consumer}/src/${header}.cpp" "#include <${header}>\n")
endforeach()

file(READ "${TEST_BINARY_DIR}/Unspecified.txt" package_files)
set(components)
foreach(file IN LISTS package_files)
  if(file MATCHES "/dart_(.+)Component\\.cmake$")
    list(APPEND components "${CMAKE_MATCH_1}")
  endif()
endforeach()
# This component only reports that DART 7 removed the OSG GUI.
list(REMOVE_ITEM components gui-osg)

# Each header is its own translation unit with the usage requirements of every
# installed component. DART's include directories use -I, as for pkg-config
# consumers, so diagnostics in DART headers are not hidden as system-header
# ones. GCC and Clang also compile as C++20 with warnings as errors.
file(
  WRITE "${consumer}/CMakeLists.txt"
  [=[
cmake_minimum_required(VERSION 3.22.1)
project(DARTStandaloneHeaders LANGUAGES CXX)
find_package(DART CONFIG REQUIRED COMPONENTS ${DART_COMPONENTS}
  PATHS "${DART_PREFIX}" NO_DEFAULT_PATH)
set_target_properties(${DART_LIBRARIES} PROPERTIES SYSTEM FALSE)
file(GLOB_RECURSE sources "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
set(standards 17)
if(NOT MSVC)
  list(APPEND standards 20)
endif()
foreach(standard IN LISTS standards)
  add_library(cxx${standard} OBJECT ${sources})
  target_link_libraries(cxx${standard} PRIVATE ${DART_LIBRARIES})
  set_target_properties(cxx${standard} PROPERTIES
    CXX_STANDARD ${standard} CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
endforeach()
if(TARGET cxx20)
  target_compile_options(cxx20 PRIVATE -Werror)
endif()
if(CMAKE_GENERATOR MATCHES "^Visual Studio")
  # Visual Studio compiles a target's sources in one compiler call.
  target_compile_options(cxx17 PRIVATE /MP)
endif()
]=]
)

# The consumer uses the DART build's toolchain and finds the dependencies a
# static install leaves to its consumers, including fetched ones staged here.
load_cache(
  "${DART_BINARY_DIR}"
  READ_WITH_PREFIX dart_
  CMAKE_GENERATOR
  CMAKE_GENERATOR_PLATFORM
  CMAKE_GENERATOR_TOOLSET
  CMAKE_TOOLCHAIN_FILE
  CMAKE_SYSROOT
  CMAKE_OSX_SYSROOT
  CMAKE_OSX_ARCHITECTURES
  CMAKE_OSX_DEPLOYMENT_TARGET
  CMAKE_CXX_COMPILER
  CMAKE_CXX_COMPILER_TARGET
  CMAKE_CXX_COMPILER_LAUNCHER
  CMAKE_PREFIX_PATH
)
execute_process(
  COMMAND
    "${CMAKE_COMMAND}" -S "${consumer}" -B "${TEST_BINARY_DIR}/build" -G
    "${dart_CMAKE_GENERATOR}" -A "${dart_CMAKE_GENERATOR_PLATFORM}" -T
    "${dart_CMAKE_GENERATOR_TOOLSET}"
    "-DCMAKE_TOOLCHAIN_FILE=${dart_CMAKE_TOOLCHAIN_FILE}"
    "-DCMAKE_SYSROOT=${dart_CMAKE_SYSROOT}"
    "-DCMAKE_OSX_SYSROOT=${dart_CMAKE_OSX_SYSROOT}"
    "-DCMAKE_OSX_ARCHITECTURES=${dart_CMAKE_OSX_ARCHITECTURES}"
    "-DCMAKE_OSX_DEPLOYMENT_TARGET=${dart_CMAKE_OSX_DEPLOYMENT_TARGET}"
    "-DCMAKE_CXX_COMPILER=${dart_CMAKE_CXX_COMPILER}"
    "-DCMAKE_CXX_COMPILER_TARGET=${dart_CMAKE_CXX_COMPILER_TARGET}"
    "-DCMAKE_CXX_COMPILER_LAUNCHER=${dart_CMAKE_CXX_COMPILER_LAUNCHER}"
    "-DCMAKE_PREFIX_PATH=${prefix};${dart_CMAKE_PREFIX_PATH}"
    "-DDART_PREFIX=${prefix}" "-DDART_COMPONENTS=${components}"
  RESULT_VARIABLE result
)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "Configuring the header consumer failed")
endif()

# CMAKE_BUILD_PARALLEL_LEVEL caps the parallel compiles.
set(parallel)
if("$ENV{CMAKE_BUILD_PARALLEL_LEVEL}" STREQUAL "")
  cmake_host_system_information(RESULT jobs QUERY NUMBER_OF_LOGICAL_CORES)
  set(parallel --parallel ${jobs})
endif()
set(keep_going)
if(dart_CMAKE_GENERATOR MATCHES "Ninja")
  # Report every failing header, not only the first.
  set(keep_going -- -k 0)
endif()
execute_process(
  COMMAND
    "${CMAKE_COMMAND}" --build "${TEST_BINARY_DIR}/build" --config
    "${TEST_CONFIG}" ${parallel} ${keep_going}
  RESULT_VARIABLE result
)
if(NOT result EQUAL 0)
  message(
    FATAL_ERROR
    "Some installed headers do not compile on their own; see the errors above"
  )
endif()
list(LENGTH headers count)
message(STATUS "${count} installed headers compile on their own")
