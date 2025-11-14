if(NOT DART_BUILD_USD)
  return()
endif()

if(NOT pxr_FOUND)
  message(WARNING "USD support is enabled but the pxr package could not be located. Skipping dart-utils-usd.")
  return()
endif()

file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

set(target_name ${PROJECT_NAME}-utils-usd)
set(component_name utils-usd)

dart_add_library(${target_name} ${hdrs} ${srcs})
set_source_files_properties(${srcs} PROPERTIES COMPILE_FLAGS "-Wno-error=cpp")
target_link_libraries(${target_name}
  PUBLIC
    dart-utils
    pxr::usd
    pxr::usdGeom
    pxr::usdPhysics
    pxr::sdf
    pxr::tf
)
if(TARGET Python3::Python)
  target_link_libraries(${target_name} PUBLIC Python3::Python)
endif()

target_compile_definitions(${target_name} PUBLIC DART_HAS_USD=1)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(${target_name} PRIVATE -Wno-error=cpp)
endif()

target_compile_features(${target_name} PUBLIC cxx_std_17)

add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} utils)
add_component_dependency_packages(${PROJECT_NAME} ${component_name} pxr)

dart_get_filename_components(header_names "utils_usd headers" ${hdrs})
dart_generate_component_headers(
  COMPONENT_NAME usd
  TARGET_DIR "dart/utils/usd/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs}
)

install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/all.hpp ${CMAKE_CURRENT_BINARY_DIR}/usd.hpp
  DESTINATION include/dart/utils/usd
  COMPONENT headers
)

dart_format_add(${hdrs} ${srcs})
