# Copyright (c) 2011-2021, The DART development contributors

set(target_link_libraries_public osg::osg)
if(APPLE)
  list(APPEND target_link_libraries_public "-framework OpenGL")
endif()

# Set up component
dart_component_setup(
  COMPONENT_NAME
    gui
  DEPENDENT_COMPONENTS
    common math simulation
  DEPENDENT_PACKAGES_REQUIRED
    OpenSceneGraph
  TARGET_LINK_LIBRARIES_PUBLIC
    ${target_link_libraries_public}
  TARGET_COMPILE_FEATURES_PUBLIC
  GENERATE_EXPORT_HEADER
  GENERATE_COMPONENT_ALL_HEADER
)
