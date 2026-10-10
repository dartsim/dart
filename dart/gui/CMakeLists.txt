set(target_name ${PROJECT_NAME}-gui)
dart_add_library(${target_name})
target_link_libraries(${target_name} PUBLIC dart)
add_component(${PROJECT_NAME} gui)
add_component_targets(${PROJECT_NAME} gui ${target_name})
add_component_dependencies(${PROJECT_NAME} gui dart)

# Configure ImGui before the OSG sources that consume its headers.
add_subdirectory(imgui)
add_subdirectory(osg)

# Legacy names forward to the complete GUI library for rebuilt consumers.
foreach(legacy_component external-imgui gui-osg)
  set(legacy_target ${PROJECT_NAME}-${legacy_component})
  add_library(${legacy_target} INTERFACE osg/Viewer.hpp)
  target_link_libraries(${legacy_target} INTERFACE ${target_name})
  add_dependencies(${legacy_target} ${target_name})
  add_component(${PROJECT_NAME} ${legacy_component})
  add_component_targets(${PROJECT_NAME} ${legacy_component} ${legacy_target})
endforeach()
add_component_dependencies(${PROJECT_NAME} external-imgui gui)
add_component_dependencies(${PROJECT_NAME} gui-osg external-imgui)
