# DART 7 removed the OpenSceneGraph GUI. This stub only reports the migration;
# it never forwards gui-osg to the Filament GUI.
set(
  _dart_gui_osg_migration
  "DART 7 removed the OpenSceneGraph GUI component gui-osg (dart-gui-osg); use the gui component and the dart-gui target, which provide the Filament GUI."
)
if(DART_FIND_REQUIRED_gui-osg)
  message(FATAL_ERROR "${_dart_gui_osg_migration}")
elseif(NOT DART_FIND_QUIETLY)
  message(STATUS "${_dart_gui_osg_migration}")
endif()
unset(_dart_gui_osg_migration)
set(dart_gui-osg_DEPENDENCIES)
set_property(DIRECTORY PROPERTY DART_gui-osg_FOUND FALSE)
