# Copyright (c) 2011-2021, The DART development contributors

# Set up component
dart_component_setup(
  COMPONENT_NAME
    rendering
  DEPENDENT_COMPONENTS
    common math simulation
  DEPENDENT_PACKAGES_REQUIRED
    OpenSceneGraph
  TARGET_LINK_LIBRARIES_PUBLIC
    osg::osg
  TARGET_COMPILE_FEATURES_PUBLIC
  GENERATE_EXPORT_HEADER
  GENERATE_COMPONENT_ALL_HEADER
)
