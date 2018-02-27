get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)

if(NOT TARGET dart-io-urdf OR NOT TARGET dart-gui-osg)
  return()
endif()

file(GLOB srcs "*.cpp" "*.hpp")

add_library(example_wamIk
  "${CMAKE_CURRENT_SOURCE_DIR}/ikfast/ikfast71.Transform6D.4_6_9_10_11_12_f8.cpp"
)
target_link_libraries(example_wamIk PUBLIC dart)
target_compile_definitions(example_wamIk PUBLIC IKFAST_NO_MAIN IKFAST_CLIBRARY)
target_compile_options(example_wamIk PRIVATE -w)

add_executable(${example_name} ${srcs})
target_link_libraries(${example_name}
  dart dart-io-urdf dart-gui-osg example_wamIk
)
set_target_properties(${example_name}
  PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

dart_add_example(${example_name})
