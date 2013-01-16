###############################################
# apps/examplePlanning
if(DART_UI AND PLANNING)
	project(examplePlanning)
	file(GLOB srcs "*.cpp")
	file(GLOB hdrs "*.h")
	add_executable(examplePlanning ${srcs} ${hdrs})
	target_link_libraries(examplePlanning collision kinematics dynamics yui renderer integration planning robotics parser optimized fcl debug fcld ccd)
	link_thirdparty_glut(examplePlanning)
	
	set_target_properties(examplePlanning PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
endif()
