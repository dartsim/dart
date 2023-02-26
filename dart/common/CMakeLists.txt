# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

# Set basic component settings
set(project_name ${PROJECT_NAME})
set(component_name common)

# Component dependency packages
set(component_dependency_packages
  Eigen3
)

# Link libraries
set(link_libraries_public
  Eigen3::Eigen
)

# fmt settings
if(TARGET fmt::fmt-header-only)
  list(APPEND component_dependency_packages fmt)
  list(APPEND link_libraries_public fmt::fmt-header-only)
elseif(TARGET fmt::fmt)
  list(APPEND component_dependency_packages fmt)
  list(APPEND link_libraries_public fmt::fmt)
else()
  message(FATAL_ERROR "Failed to find fmt targets")
endif()

# spdlog settings
if(spdlog_FOUND)
  # Prefer header only target
  if(TARGET spdlog::spdlog_header_only)
    list(APPEND component_dependency_packages_optional spdlog)
    list(APPEND link_libraries_public spdlog::spdlog_header_only)
  elseif(TARGET spdlog::spdlog)
    list(APPEND component_dependency_packages_optional spdlog)
    list(APPEND link_libraries_public spdlog::spdlog)
  else()
    message(WARNING "spdlog found, but no target is found. Expected spdlog::spdlog or target_link_library_public spdlog::spdlog_header_only")
    list(APPEND compile_definitions_public -DDART_HAS_SPDLOG=0)
  endif()
else()
  list(APPEND compile_definitions_public -DDART_HAS_SPDLOG=0)
endif()

# Profile backend settings
if(DART_PROFILE_BACKEND_REMOTERY)
  list(APPEND link_libraries_public dart${DART_MAJOR_VERSION}-profile)
  list(APPEND compile_definitions_public -DDART_PROFILE_BACKEND_REMOTERY=1)
endif()

# Explicit template instantiation options
if(DART_BUILD_TEMPLATE_CODE_FOR_DOUBLE)
  list(APPEND compile_definitions_public -DDART_BUILD_TEMPLATE_CODE_FOR_DOUBLE=1)
else()
  list(APPEND compile_definitions_public -DDART_BUILD_TEMPLATE_CODE_FOR_DOUBLE=0)
endif()
if(DART_BUILD_TEMPLATE_CODE_FOR_FLOAT)
  list(APPEND compile_definitions_public -DDART_BUILD_TEMPLATE_CODE_FOR_FLOAT=1)
else()
  list(APPEND compile_definitions_public -DDART_BUILD_TEMPLATE_CODE_FOR_FLOAT=0)
endif()

# Active log level
if(DART_ACTIVE_LOG_LEVEL STREQUAL "TRACE")
  list(APPEND compile_definitions_public -DDART_ACTIVE_LOG_LEVEL=0)
elseif(DART_ACTIVE_LOG_LEVEL STREQUAL "DEBUG")
  list(APPEND compile_definitions_public -DDART_ACTIVE_LOG_LEVEL=1)
elseif(DART_ACTIVE_LOG_LEVEL STREQUAL "INFO")
  list(APPEND compile_definitions_public -DDART_ACTIVE_LOG_LEVEL=2)
elseif(DART_ACTIVE_LOG_LEVEL STREQUAL "WARN")
  list(APPEND compile_definitions_public -DDART_ACTIVE_LOG_LEVEL=3)
elseif(DART_ACTIVE_LOG_LEVEL STREQUAL "ERROR")
  list(APPEND compile_definitions_public -DDART_ACTIVE_LOG_LEVEL=4)
elseif(DART_ACTIVE_LOG_LEVEL STREQUAL "FATAL")
  list(APPEND compile_definitions_public -DDART_ACTIVE_LOG_LEVEL=5)
elseif(DART_ACTIVE_LOG_LEVEL STREQUAL "OFF")
  list(APPEND compile_definitions_public -DDART_ACTIVE_LOG_LEVEL=6)
else()
  message(FATAL_ERROR "Invalid active log level: ${DART_ACTIVE_LOG_LEVEL}")
endif()

# Build DART with all available SIMD instructions
if(DART_ENABLE_SIMD)

  if(MSVC)
    # /arch:AVX2 option is effective only for x64 but not for x86 since it's
    # enabled for x86 by default. The option will be ignored on x86 with a
    # warning message. If anyone knows how to detect the system's architecture
    # in CMake time, then I would gratefully apply these options differently
    # depending on the architectures.
    if(CMAKE_SIZEOF_VOID_P EQUAL 8) # x64
        list(APPEND compile_options_public /arch:AVX2)
    else()  # x86
        list(APPEND compile_options_public /arch:SSE2 /arch:AVX /arch:AVX2)
    endif()

  elseif(CMAKE_COMPILER_IS_GNUCXX OR "${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang" OR "${CMAKE_CXX_COMPILER_ID}" MATCHES "AppleClang")

    if(APPLE)
      if(CMAKE_SYSTEM_PROCESSOR MATCHES "^arm")
          list(APPEND compile_options_public -march=armv8.3-a+simd -faligned-new)
      else()
          if(NOT "${CMAKE_CXX_COMPILER_ID}" MATCHES "AppleClang")
              list(APPEND compile_options_public -march=native -faligned-new)
          endif()
      endif()

    else()
      list(APPEND compile_options_public -march=native -faligned-new)
    endif()

  endif()

endif()

# Build DART with coverage settings
if(DART_CODECOV)
  list(APPEND link_libraries_public coverage_config)
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
  DEPENDENT_PACKAGES_REQUIRED
    ${component_dependency_packages}
  DEPENDENT_PACKAGES_OPTIONAL
    ${component_dependency_packages_optional}
  TARGET_LINK_LIBRARIES_PUBLIC
    ${link_libraries_public}
  TARGET_LINK_LIBRARIES_PUBLIC_SKIP_CHECKING
    ${CMAKE_DL_LIBS}
    $<$<PLATFORM_ID:Linux>:stdc++fs>
    ${link_libraries_public_skip_checking}
  TARGET_LINK_OPTIONS_PUBLIC
    ${link_options_public}
  TARGET_COMPILE_DEFINITIONS_PUBLIC
    ${compile_definitions_public}
  TARGET_COMPILE_FEATURES_PUBLIC
    cxx_std_17
  TARGET_COMPILE_OPTIONS_PUBLIC
    ${compile_options_public}
  SUB_DIRECTORIES
    detail allocator ecs gpu
  GENERATE_EXPORT_HEADER
  GENERATE_META_HEADER
  FORMAT_CODE
)
