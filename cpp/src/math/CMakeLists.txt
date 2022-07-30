# Copyright (c) 2011-2022, The DART development contributors

# Set up component
dart_component_setup(
  COMPONENT_NAME math
  DEPENDENT_COMPONENTS common
  DEPENDENT_PACKAGES_OPTIONAL
  TARGET_LINK_LIBRARIES_PUBLIC
    convhull_3d::convhull_3d
  TARGET_COMPILE_FEATURES_PUBLIC
  GENERATE_EXPORT_HEADER
  GENERATE_COMPONENT_ALL_HEADER
  SUB_DIRECTORIES
    bounding_volume bvh geometry lie_group
)
