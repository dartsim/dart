# Note: The dart-gui component has been removed since GLUT has been deprecated.
# OpenSceneGraph (OSG) is now the only GUI backend, provided through the
# dart-gui-osg component.

# Add subdirectories
add_subdirectory(osg)
if(DART_BUILD_RERUN)
  add_subdirectory(rerun)
endif()
