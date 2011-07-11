###############################################
# apps/exampleUI
PROJECT(exampleUI)
FILE(GLOB example_ui_srcs "*.cpp")
FILE(GLOB example_ui_hdrs "*.h")
ADD_EXECUTABLE(exampleUI ${example_ui_srcs} ${example_ui_hdrs})
TARGET_LINK_LIBRARIES(exampleUI YUI renderer)
TARGET_LINK_LIBRARIES(exampleUI ${OPENGL_LIBRARIES} ${GLUT_glut_LIBRARY})

SET_TARGET_PROPERTIES(exampleUI PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

