###############################################
# apps/bipedCMA

message(STATUS "Reconfigure the Boost for this app")
FIND_PACKAGE( Boost 1.45.0 COMPONENTS system date_time filesystem program_options signals serialization thread unit_test_framework REQUIRED )
include_directories ( ${Boost_INCLUDE_DIR} ) # CI: changed Boost_INCLUDE_DIRS to Boost_INCLUDE_DIR ... hope that this is always works
message(STATUS "Boost_LIBRARIES = ${Boost_LIBRARIES}")
include_directories(${SHARK_INCLUDE_DIRS})

file(GLOB bipedCMA_srcs "*.cpp")
file(GLOB bipedCMA_hdrs "*.h")
add_executable(bipedCMA ${bipedCMA_srcs} ${bipedCMA_hdrs})
target_link_libraries(bipedCMA dart)
target_link_libraries(bipedCMA ${SHARK_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(bipedCMA PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
