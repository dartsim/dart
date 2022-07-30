# Copyright (c) 2011-2022, The DART development contributors

# Set up component
dart_component_setup_subdirectory(
  SUB_COMPONENT_NAME
    dart
  DEPENDENT_PACKAGES_REQUIRED
  TARGET_LINK_LIBRARIES_PUBLIC
  SUB_DIRECTORIES
    broad_phase geometry narrow_phase
)
