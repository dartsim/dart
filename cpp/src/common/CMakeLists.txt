# Copyright (c) 2011-2021, The DART development contributors

# Target link libraries
set(target_link_library_public
  Eigen3::Eigen
)
if(TARGET fmt::fmt-header-only)
  list(APPEND target_link_library_public fmt::fmt-header-only)
else()
  list(APPEND target_link_library_public fmt::fmt)
endif()
if(spdlog_FOUND)
  if(TARGET spdlog::spdlog_header_only)
    list(APPEND target_link_library_public spdlog::spdlog_header_only)
  else()
    list(APPEND target_link_library_public spdlog::spdlog)
  endif()
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

set(compile_options_public -fPIC)

# Code coverage settings
if(DART_ENABLE_CODECOV)
  set(CMAKE_CXX_FLAGS "-fprofile-arcs -ftest-coverage -O0")
  if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.13)
    list(APPEND link_options_public --coverage)
  else()
    list(APPEND target_link_library_public --coverage)
  endif()
  list(APPEND compile_options_public
    -O0         # no optimization
    -g          # generate debug info
    --coverage  # sets all required flags
  )
  list(APPEND target_link_library_public gcov)
endif()

# Set up component
dart_component_setup(
  COMPONENT_NAME common
  DEPENDENT_PACKAGES_REQUIRED
    fmt Eigen3
  DEPENDENT_PACKAGES_OPTIONAL
    spdlog
  TARGET_LINK_LIBRARIES_PUBLIC
    ${target_link_library_public}
  TARGET_COMPILE_FEATURES_PUBLIC
    cxx_std_17
  TARGET_COMPILE_OPTIONS_PUBLIC
    ${compile_options_public}
  TARGET_COMPILE_DEFINITIONS_PUBLIC
    ${compile_definitions_public}
  TARGET_LINK_OPTIONS_PUBLIC
    ${link_options_public}
  GENERATE_EXPORT_HEADER
  GENERATE_COMPONENT_ALL_HEADER
)
