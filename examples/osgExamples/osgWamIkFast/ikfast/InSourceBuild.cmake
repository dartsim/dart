add_library(wamIk SHARED ikfast71.Transform6D.4_6_9_10_11_12_f8.cpp)
target_link_libraries(wamIk PUBLIC dart)
target_compile_definitions(wamIk PUBLIC IKFAST_NO_MAIN IKFAST_CLIBRARY)
target_compile_options(wamIk PRIVATE -w)
set_target_properties(wamIk
  PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
