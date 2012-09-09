###############################################
# apps/exampleOpt
<<<<<<< HEAD

# Depends on snopt
if(NOT SNOPT)
  MESSAGE(STATUS "Disabling exampleOpt. Cannot build without snopt")
  RETURN()
endif()

=======
>>>>>>> karen
PROJECT(exampleOpt)
FILE(GLOB exampleOpt_srcs "*.cpp")
FILE(GLOB exampleOpt_hdrs "*.h")
ADD_EXECUTABLE(exampleOpt ${exampleOpt_srcs} ${exampleOpt_hdrs})
TARGET_LINK_LIBRARIES(exampleOpt optimizer)
<<<<<<< HEAD
TARGET_LINK_LIBRARIES(exampleOpt gfortran snopt)
=======
TARGET_LINK_LIBRARIES(exampleOpt gfortran snopt_c snprint_c blas_c f2c)
>>>>>>> karen
LINK_THIRDPARTY_GFLAGS(exampleOpt)
LINK_THIRDPARTY_GLOG(exampleOpt)

SET_TARGET_PROPERTIES(exampleOpt PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
