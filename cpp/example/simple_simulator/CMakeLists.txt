# Copyright (c) 2011-2021, The DART development contributors

get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)

dart_build_executable(
  TARGET_NAME
    EXAMPLE_${example_name}
  DEPENDENT_COMPONENTS
    application
)
