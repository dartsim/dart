###############################################
# Unittests
project(unittests)

# Depends on snopt
if(NOT SNOPT)
  message(STATUS "Disabling some unittests. Cannot build all tests without snopt")
endif()

# Search all header and source files
file(GLOB unittests_srcs "*.cpp")
file(GLOB unittests_hdrs "*.h")

if(SNOPT)
    file(GLOB unittests_snopt "*.cpp" "*.h")
    set(snopt_lib snopt gfortran)
endif(SNOPT)

add_executable(unittests ${unittests_srcs} ${unittests_hdrs} ${unittests_snopt})
target_link_libraries(unittests utils kinematics dynamics optimizer ${snopt_lib})

link_thirdparty_gtest(unittests)
link_thirdparty_tinyxml(unittests)

if(APPLE OR UNIX)
  target_link_libraries(unittests pthread)
endif()

set_target_properties(unittests PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")