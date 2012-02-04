cmake_minimum_required(VERSION 2.6)
project (fcl)

option (BUILD_ALL_TESTS "OFF")
option (USE_BOOST "OFF")

if (USE_BOOST)
	add_definitions (-DUSEBOOST)
endif (USE_BOOST)

###################################
# Build Library
file (GLOB srcs "src/*.cpp")
file (GLOB hdrs "include/fcl/*.h")

set (CMAKE_CXX_FLAGS "-DUSE_PQP=0 -DUSE_SVMLIGHT=0")
include_directories (include ../include ../ann_1.1.2/include ../libccd-1.0/src)
link_directories (fcl "${CMAKE_SOURCE_DIR}/lib" "${CMAKE_SOURCE_DIR}/../lib")
add_library (fcl ${srcs} ${hdrs})
set_target_properties (fcl PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")

###################################
# Build Test Cases
add_executable (test_core_collision test/test_core_collision.cpp)
target_link_libraries (test_core_collision fcl ccd ann)
set_target_properties (test_core_collision PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")

if (BUILD_ALL_TESTS)
	add_executable (test_core_distance test/test_core_distance.cpp)
	target_link_libraries (test_core_distance fcl ccd ann)
	
	add_executable (test_core_collision_point test/test_core_collision_point.cpp)
	target_link_libraries (test_core_collision_point fcl ccd ann)
	
	add_executable (test_core_geometric_shapes test/test_core_geometric_shapes.cpp)
	target_link_libraries(test_core_geometric_shapes fcl)
	
	add_executable (test_core_collision_shape_mesh_consistency test/test_core_collision_shape_mesh_consistency.cpp)
	target_link_libraries(test_core_collision_shape_mesh_consistency fcl)
	
	add_executable (test_core_broad_phase test/test_core_broad_phase.cpp)
	target_link_libraries(test_core_broad_phase fcl)
	
	add_executable (test_core_front_list test/test_core_front_list.cpp)
	target_link_libraries(test_core_front_list fcl)
	
	add_executable (test_core_continuous_collision test/test_core_continuous_collision.cpp)
	target_link_libraries(test_core_continuous_collision fcl)
	
	add_executable(test_core_deformable_object test/test_core_deformable_object.cpp)
	target_link_libraries(test_core_deformable_object fcl)
	
	add_executable(test_core_conservative_advancement test/test_core_conservative_advancement.cpp)
	target_link_libraries(test_core_conservative_advancement fcl)
endif (BUILD_ALL_TESTS)                                               

###################################
# Install files 
install (FILES ${hdrs} DESTINATION include/fcl)
install (TARGETS fcl ARCHIVE DESTINATION lib)
