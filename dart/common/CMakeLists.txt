# Copyright (c) 2011-2021, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

# Set basic component settings
set(project_name ${PROJECT_NAME})
set(component_name common)
set(link_libraries_public
  Boost::boost
  Boost::system
  Boost::filesystem
  Eigen3::Eigen
)

# Build DART with all available SIMD instructions
if(DART_ENABLE_SIMD)
  if(MSVC)
    list(APPEND compile_options_public /arch:SSE2 /arch:AVX /arch:AVX2)
    # /arch:SSE2 option is effective only for x86 but not for x64 since it's
    # enabled for x64 by default. The option will be ignored on x64 with a
    # warning message. If anyone knows how to detect the system's architecture
    # in CMake time, then I would gratefully apply these options differently
    # depending on the architectures.
  elseif(CMAKE_COMPILER_IS_GNUCXX)
    list(APPEND compile_options_public -march=native -faligned-new)
  elseif("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    list(APPEND compile_options_public -march=native -faligned-new)
  endif()
endif()

# CodeCov settings
# Reference: https://github.com/codecov/example-cpp11-cmake
if(DART_CODECOV AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.13)
    list(APPEND link_options_public --coverage)
  else()
    list(APPEND link_libraries_public_skip_checking --coverage)
  endif()
  list(APPEND compile_options_public
    -O0         # no optimization
    -g          # generate debug info
    --coverage  # sets all required flags
  )
  list(APPEND link_libraries_public_skip_checking gcov)
endif()

# Add component
dart_add_component(
  PROJECT_NAME
    ${PROJECT_NAME}
  PROJECT_VERSION_MAJOR
    ${DART_MAJOR_VERSION}
  PROJECT_SOURCE_DIR
    ${DART_SOURCE_DIR}
  PROJECT_BINARY_DIR
    ${DART_BINARY_DIR}
  COMPONENT_NAME
    ${component_name}
  COMPONENT_DEPENDENCY_PACKAGES
    Boost Eigen3
  TARGET_LINK_LIBRARIES_PUBLIC
    ${link_libraries_public}
  TARGET_LINK_LIBRARIES_PUBLIC_SKIP_CHECKING
    ${CMAKE_DL_LIBS}
    $<$<PLATFORM_ID:Linux>:stdc++fs>
    ${link_libraries_public_skip_checking}
  TARGET_LINK_OPTIONS_PUBLIC
    ${link_options_public}
  TARGET_COMPILE_FEATURES_PUBLIC
    cxx_std_17
  TARGET_COMPILE_OPTIONS_PUBLIC
    ${compile_options_public}
  SUB_DIRECTORIES
    detail
  GENERATE_META_HEADER
  FORMAT_CODE
)
