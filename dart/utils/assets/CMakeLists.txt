# Dependency checks
dart_find_package(CURL)
dart_find_package(OpenSSL)
if(NOT CURL_FOUND OR NOT TARGET CURL::libcurl)
  message(FATAL_ERROR "dart-utils-assets requires libcurl >= 7.85.0")
endif()
if(NOT OPENSSL_FOUND OR NOT TARGET OpenSSL::Crypto)
  message(FATAL_ERROR "dart-utils-assets requires OpenSSL >= 1.1.1 (Crypto)")
endif()

set(hdrs ModelResourceRetriever.hpp)
set(srcs ModelResourceRetriever.cpp)
set(target_name ${PROJECT_NAME}-utils-assets)
set(component_name utils-assets)

target_sources(dart-io PRIVATE ${hdrs} ${srcs})
# Other dart-io unity sources leave Windows' far macro undefined, which breaks
# the winsock headers that curl includes.
set_source_files_properties(
  ${srcs}
  TARGET_DIRECTORY dart-io
  PROPERTIES SKIP_UNITY_BUILD_INCLUSION ON
)
target_link_libraries(dart-io PRIVATE CURL::libcurl OpenSSL::Crypto)
add_component_dependency_packages(${PROJECT_NAME} io CURL OpenSSL)

add_library(${target_name} INTERFACE ${hdrs})
target_link_libraries(${target_name} INTERFACE dart-io)
add_dependencies(${target_name} dart-io)

add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} io)

install(FILES ${hdrs} DESTINATION include/dart/utils/assets COMPONENT headers)
dart_format_add(${hdrs} ${srcs})
