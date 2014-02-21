###############################################
# apps/vehicle
file(GLOB vehicle_srcs "*.cpp")
file(GLOB vehicle_hdrs "*.h")
add_executable(vehicle ${vehicle_srcs} ${vehicle_hdrs})
target_link_libraries(vehicle dart)
set_target_properties(vehicle PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
