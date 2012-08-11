###############################################
# apps/exampleUI
if(DART_UI)
	PROJECT(exampleUI)
	FILE(GLOB example_ui_srcs "*.cpp")
	FILE(GLOB example_ui_hdrs "*.h")
	ADD_EXECUTABLE(exampleUI ${example_ui_srcs} ${example_ui_hdrs})
	TARGET_LINK_LIBRARIES(exampleUI yui renderer)
	LINK_THIRDPARTY_FREE_IMAGE(exampleUI)
	LINK_THIRDPARTY_GLUT(exampleUI)

	SET_TARGET_PROPERTIES(exampleUI PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
endif(DART_UI)