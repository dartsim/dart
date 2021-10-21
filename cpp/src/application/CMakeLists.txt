# Copyright (c) 2011-2021, The DART development contributors

# Set up component
dart_component_setup(
  COMPONENT_NAME
    application
  DEPENDENT_COMPONENTS
    common math simulation rendering gui
  DEPENDENT_PACKAGES_OPTIONAL
  TARGET_LINK_LIBRARIES_PUBLIC
  TARGET_COMPILE_FEATURES_PUBLIC
  GENERATE_EXPORT_HEADER
  GENERATE_COMPONENT_ALL_HEADER
)
