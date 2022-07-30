# Copyright (c) 2011-2022, The DART development contributors

# Set up component
dart_component_setup(
  COMPONENT_NAME
    collision
  DEPENDENT_COMPONENTS
    common math
  TARGET_LINK_LIBRARIES_PUBLIC
  TARGET_COMPILE_FEATURES_PUBLIC
  GENERATE_EXPORT_HEADER
  GENERATE_COMPONENT_ALL_HEADER
  SUB_DIRECTORIES
    dart fcl
)
