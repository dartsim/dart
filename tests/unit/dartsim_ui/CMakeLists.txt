# Copyright (c) 2011, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License.

dart_add_test("unit" UNIT_dartsim_ui_ProjectActions test_project_actions.cpp)
target_link_libraries(UNIT_dartsim_ui_ProjectActions dartsim_ui)

dart_add_test("unit" UNIT_dartsim_ui_ConsoleActions test_console_actions.cpp)
target_link_libraries(UNIT_dartsim_ui_ConsoleActions dartsim_ui)

dart_add_test("unit" UNIT_dartsim_ui_HistoryActions test_history_actions.cpp)
target_link_libraries(UNIT_dartsim_ui_HistoryActions dartsim_ui)

dart_add_test("unit" UNIT_dartsim_ui_InspectorActions test_inspector_actions.cpp)
target_link_libraries(UNIT_dartsim_ui_InspectorActions dartsim_ui)

dart_add_test("unit" UNIT_dartsim_ui_OutlinerActions test_outliner_actions.cpp)
target_link_libraries(UNIT_dartsim_ui_OutlinerActions dartsim_ui)

dart_add_test("unit" UNIT_dartsim_ui_PaletteActions test_palette_actions.cpp)
target_link_libraries(UNIT_dartsim_ui_PaletteActions dartsim_ui)

dart_add_test(
  "unit" UNIT_dartsim_ui_RelationshipActions test_relationship_actions.cpp
)
target_link_libraries(UNIT_dartsim_ui_RelationshipActions dartsim_ui)

dart_add_test("unit" UNIT_dartsim_ui_ViewportActions test_viewport_actions.cpp)
target_link_libraries(UNIT_dartsim_ui_ViewportActions dartsim_ui)

dart_add_test("unit" UNIT_dartsim_ui_SimulationActions test_simulation_actions.cpp)
target_link_libraries(UNIT_dartsim_ui_SimulationActions dartsim_ui)

dart_add_test("unit" UNIT_dartsim_ui_WatchActions test_watch_actions.cpp)
target_link_libraries(UNIT_dartsim_ui_WatchActions dartsim_ui)

if(NOT APPLE AND TARGET dart-simulation-experimental)
  if(WIN32)
    set(_dartsim_ui_path_var "PATH")
  else()
    set(_dartsim_ui_path_var "LD_LIBRARY_PATH")
  endif()
  foreach(
    _dartsim_ui_test
    UNIT_dartsim_ui_ProjectActions
    UNIT_dartsim_ui_ConsoleActions
    UNIT_dartsim_ui_HistoryActions
    UNIT_dartsim_ui_InspectorActions
    UNIT_dartsim_ui_OutlinerActions
    UNIT_dartsim_ui_PaletteActions
    UNIT_dartsim_ui_RelationshipActions
    UNIT_dartsim_ui_ViewportActions
    UNIT_dartsim_ui_SimulationActions
    UNIT_dartsim_ui_WatchActions
  )
    if(TEST ${_dartsim_ui_test})
      set_property(
        TEST ${_dartsim_ui_test}
        APPEND
        PROPERTY
          ENVIRONMENT_MODIFICATION
            "${_dartsim_ui_path_var}=path_list_prepend:$<TARGET_FILE_DIR:dart-simulation-experimental>"
      )
    endif()
  endforeach()
  unset(_dartsim_ui_path_var)
endif()
