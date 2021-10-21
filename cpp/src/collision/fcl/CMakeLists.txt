# Copyright (c) 2011-2021, The DART development contributors

# Set up component
dart_component_setup_subdirectory(
  SUB_COMPONENT_NAME
    fcl
  DEPENDENT_PACKAGES_REQUIRED
    ccd fcl
  TARGET_LINK_LIBRARIES_PUBLIC
    ccd fcl
)
