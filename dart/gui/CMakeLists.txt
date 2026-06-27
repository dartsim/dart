# The legacy dart-gui component has been removed. OpenSceneGraph remains
# available through the dart-gui-osg component.

# The bundled ImGui (the external-imgui compatibility component) is consumed by
# the OSG GUI, so configure it before the osg subdirectory.
add_subdirectory(imgui)
add_subdirectory(osg)
