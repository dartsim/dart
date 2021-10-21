# Copyright (c) 2011-2021, The DART development contributors

add_subdirectory(common)
add_subdirectory(math)
add_subdirectory(collision)
add_subdirectory(simulation)
add_subdirectory(rendering)
add_subdirectory(gui)
add_subdirectory(application)

# Generate "all" header file
dart_property_get(
  DART_GLOBAL_PROPERTY_COMPONENT_LIST
  component_list
)
dart_property_get(
  DART_GLOBAL_PROPERTY_PROJECT_INCLUDE_BINARY_BASE_DIR
  project_include_binary_base_dir
)
set(output_filepath ${project_include_binary_base_dir}/dart/all.hpp)
file(WRITE ${output_filepath} "// Automatically generated file by cmake\n\n")
foreach(comp ${component_list})
  file(APPEND ${output_filepath} "#include \"${comp}/all.hpp\"\n")
endforeach()
install(
  FILES ${output_filepath}
  DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}${DART_VERSION_MAJOR}
)
