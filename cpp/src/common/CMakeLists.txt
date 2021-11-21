# Copyright (c) 2011-2021, The DART development contributors

# Target link libraries
set(target_link_library_public
  Eigen3::Eigen
  ${CMAKE_DL_LIBS}
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
if(OpenMP_FOUND)
  list(APPEND target_link_library_public OpenMP::OpenMP_CXX)
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

if(MSVC)
  set(compile_options_public )
else()
  set(compile_options_public -fPIC)
endif()

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

if(CMAKE_BUILD_TYPE STREQUAL Debug)
  list(APPEND compile_definitions_public -DDART_ACTIVE_LOG_LEVEL=1)
else()
  list(APPEND compile_definitions_public -DDART_ACTIVE_LOG_LEVEL=2)
endif()

# Set up component
dart_component_setup(
  COMPONENT_NAME common
  DEPENDENT_PACKAGES_REQUIRED
    fmt Eigen3
  DEPENDENT_PACKAGES_OPTIONAL
    OpenMP spdlog
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
  SUB_DIRECTORIES
    memory_allocator resource
)

# TODO(JS): Fix this
message("[DEBUG] TBB_INCLUDE_DIRS: ${TBB_INCLUDE_DIRS}")
message("[DEBUG] TBB_LIBRARIES   : ${TBB_LIBRARIES}")
# TODO(JS): Make TBB imported target
if(DART_ENABLE_TBB)
  target_include_directories(dart8-common PUBLIC ${TBB_INCLUDE_DIRS})
  target_link_libraries(dart8-common PUBLIC ${TBB_LIBRARIES})
  target_compile_definitions(dart8-common PUBLIC -DDART_ENABLE_TBB=1)
else()
  target_compile_definitions(dart8-common PUBLIC -DDART_ENABLE_TBB=0)
endif()
