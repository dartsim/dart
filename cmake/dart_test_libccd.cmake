# Optional libccd support for collision experimental tests/benchmarks.

if(TARGET dart_tests_libccd)
  set(DART_TESTS_LIBCCD_FOUND TRUE)
  return()
endif()

set(DART_TESTS_LIBCCD_FOUND FALSE)
set(DART_TESTS_LIBCCD_ROOT "" CACHE PATH "Path to libccd (source or install) for tests/benchmarks")

if(DART_TESTS_LIBCCD_ROOT)
  if(EXISTS "${DART_TESTS_LIBCCD_ROOT}/CMakeLists.txt")
    set(_dart_prev_build_shared_libs ${BUILD_SHARED_LIBS})
    set(_dart_prev_enable_double ${ENABLE_DOUBLE_PRECISION})
    set(_dart_prev_build_testing ${BUILD_TESTING})

    set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
    set(ENABLE_DOUBLE_PRECISION ON CACHE BOOL "" FORCE)
    set(BUILD_TESTING OFF CACHE BOOL "" FORCE)

    add_subdirectory(
      ${DART_TESTS_LIBCCD_ROOT}
      ${CMAKE_BINARY_DIR}/_deps/libccd
      EXCLUDE_FROM_ALL
    )

    if(TARGET ccd)
      add_library(dart_tests_libccd INTERFACE)
      target_link_libraries(dart_tests_libccd INTERFACE ccd)
      set(DART_TESTS_LIBCCD_FOUND TRUE)
    endif()

    if(DEFINED _dart_prev_build_shared_libs)
      set(BUILD_SHARED_LIBS ${_dart_prev_build_shared_libs} CACHE BOOL "" FORCE)
    else()
      unset(BUILD_SHARED_LIBS CACHE)
    endif()

    if(DEFINED _dart_prev_enable_double)
      set(ENABLE_DOUBLE_PRECISION ${_dart_prev_enable_double} CACHE BOOL "" FORCE)
    else()
      unset(ENABLE_DOUBLE_PRECISION CACHE)
    endif()

    if(DEFINED _dart_prev_build_testing)
      set(BUILD_TESTING ${_dart_prev_build_testing} CACHE BOOL "" FORCE)
    else()
      unset(BUILD_TESTING CACHE)
    endif()
  else()
    find_path(
      LIBCCD_INCLUDE_DIR
      ccd/ccd.h
      PATHS "${DART_TESTS_LIBCCD_ROOT}/include"
      NO_DEFAULT_PATH
    )
    find_library(
      LIBCCD_LIBRARY
      NAMES ccd libccd
      PATHS "${DART_TESTS_LIBCCD_ROOT}/lib" "${DART_TESTS_LIBCCD_ROOT}/lib64"
      NO_DEFAULT_PATH
    )

    if(LIBCCD_INCLUDE_DIR AND LIBCCD_LIBRARY)
      add_library(dart_tests_libccd INTERFACE)
      target_include_directories(dart_tests_libccd INTERFACE ${LIBCCD_INCLUDE_DIR})
      target_link_libraries(dart_tests_libccd INTERFACE ${LIBCCD_LIBRARY})
      set(DART_TESTS_LIBCCD_FOUND TRUE)
    endif()
  endif()
endif()

if(NOT DART_TESTS_LIBCCD_FOUND)
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(PC_LIBCCD QUIET ccd)
  endif()

  find_path(
    LIBCCD_INCLUDE_DIR
    ccd/ccd.h
    HINTS ${PC_LIBCCD_INCLUDEDIR} ${PC_LIBCCD_INCLUDE_DIRS}
  )
  find_library(
    LIBCCD_LIBRARY
    NAMES ccd libccd
    HINTS ${PC_LIBCCD_LIBDIR} ${PC_LIBCCD_LIBRARY_DIRS}
  )

  if(LIBCCD_INCLUDE_DIR AND LIBCCD_LIBRARY)
    add_library(dart_tests_libccd INTERFACE)
    target_include_directories(dart_tests_libccd INTERFACE ${LIBCCD_INCLUDE_DIR})
    target_link_libraries(dart_tests_libccd INTERFACE ${LIBCCD_LIBRARY})
    set(DART_TESTS_LIBCCD_FOUND TRUE)
  endif()
endif()
