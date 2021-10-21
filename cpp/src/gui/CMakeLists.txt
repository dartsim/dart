# Copyright (c) 2011-2021, The DART development contributors

# Set up component
dart_component_setup(
  COMPONENT_NAME
    gui
  DEPENDENT_COMPONENTS
    common math rendering
  DEPENDENT_PACKAGES_REQUIRED
    OpenGL GLEW glfw3
  TARGET_LINK_LIBRARIES_PUBLIC
    OpenGL::GL GLEW::glew glfw imgui::imgui
  TARGET_COMPILE_FEATURES_PUBLIC
  GENERATE_EXPORT_HEADER
  GENERATE_COMPONENT_ALL_HEADER
)
