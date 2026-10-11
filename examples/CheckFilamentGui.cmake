# Inspect the selected target rather than another installation on the search path.
set(DART_EXAMPLES_HAVE_FILAMENT_GUI FALSE)
if(TARGET dart-gui)
  if(DART_IN_SOURCE_BUILD)
    set(DART_EXAMPLES_HAVE_FILAMENT_GUI TRUE)
  else()
    get_target_property(
      _dart_gui_include_dirs
      dart-gui
      INTERFACE_INCLUDE_DIRECTORIES
    )
    foreach(_dart_gui_include_dir IN LISTS _dart_gui_include_dirs)
      if(
        NOT _dart_gui_include_dir MATCHES "^\\$<"
        AND EXISTS "${_dart_gui_include_dir}/dart/gui/Viewer.hpp"
      )
        set(DART_EXAMPLES_HAVE_FILAMENT_GUI TRUE)
        break()
      endif()
    endforeach()
    unset(_dart_gui_include_dir)
    unset(_dart_gui_include_dirs)
  endif()
endif()
