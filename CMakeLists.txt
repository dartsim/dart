CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

message(STATUS "Generate makefile/project file for GroundZero")

###############################################
# SETTING: Global settings
INCLUDE_DIRECTORIES("src")
SET(CMAKE_CXX_WARNING_LEVEL 4)
if (WIN32)
  message(STATUS "Setup Visual Studio Specifig Flags")
  SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd /Zi /GL /Gy")
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT /Zi /GL /Gy")
elseif (APPLE)
elseif (UNIX)
  # SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -msse2")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2")
endif()

###############################################
# SETTING: User specific setting
SET(user_info "Sehoon")
message(STATUS "User profile information = [" ${user_info} "]")

####
if (user_info STREQUAL "Sehoon")
  # If user is Sehoon
  message(STATUS "Retrieve configuration for Sehoon")
  # Detect OS
  if (UNIX)
    message(STATUS "Operating system = UNIX")
    INCLUDE_DIRECTORIES(/usr/local/include/eigen3)
    INCLUDE_DIRECTORIES(/usr/local/include/ticpp)
    INCLUDE_DIRECTORIES(~/dev/packages/gtest-1.6.0/include)
    LINK_DIRECTORIES(~/dev/packages/gtest-1.6.0/lib)
  elseif (WIN32)
    message(STATUS "Operating system = WIN32")
    INCLUDE_DIRECTORIES(c:/dev/external/include)
    INCLUDE_DIRECTORIES(c:/dev/boost_1_46_1)
    LINK_DIRECTORIES(c:/dev/external/lib)
  elseif (APPLE)
    message(STATUS "Operating system = APPLE")
  else()
    message(FATAL_ERROR "Invalid operating system")
  endif()

  ####
elseif (user_info EQUAL "Yunfei")
  # If user is Yunfei
elseif (user_info EQUAL "Kristin")
  # If user is Yunfei
  ####
else ()
  # Invalid user info?
  message(FATAL_ERROR "Invalid user information")
endif ()


###############################################
# Utils
PROJECT(utils)

# Search all header and source files
FILE(GLOB utils_srcs "src/utils/*.cpp")
FILE(GLOB utils_hdrs "src/utils/*.h")

# Library 
ADD_LIBRARY(utils ${utils_srcs} ${utils_hdrs})
SET_TARGET_PROPERTIES(utils PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "lib")

###############################################
# Model3d
PROJECT(model3d)

# Search all header and source files
FILE(GLOB model3d_srcs "src/model3d/*.cpp")
FILE(GLOB model3d_hdrs "src/model3d/*.h")

# Library 
ADD_LIBRARY(model3d ${model3d_srcs} ${model3d_hdrs})
TARGET_LINK_LIBRARIES(model3d utils)
SET_TARGET_PROPERTIES(model3d PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "lib")


###############################################
# optimizer
PROJECT(optimizer)

# Search all header and source files
FILE(GLOB optimizer_srcs "src/optimizer/*.cpp")
FILE(GLOB optimizer_hdrs "src/optimizer/*.h")
FILE(GLOB optimizer_snopt_srcs "src/optimizer/snopt/*.cpp")
FILE(GLOB optimizer_snopt_hdrs "src/optimizer/snopt/*.h")

# Library 
ADD_LIBRARY(optimizer ${optimizer_srcs} ${optimizer_hdrs}
  ${optimizer_snopt_srcs} ${optimizer_snopt_hdrs})
TARGET_LINK_LIBRARIES(optimizer utils)
SET_TARGET_PROPERTIES(optimizer PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "lib")

###############################################
# Unittests
PROJECT(unittests)
# Search all header and source files
FILE(GLOB unittests_srcs "src/unittests/*.cpp")
FILE(GLOB unittests_hdrs "src/unittests/*.h")

ADD_EXECUTABLE(unittests ${unittests_srcs} ${unittests_hdrs})
TARGET_LINK_LIBRARIES(unittests utils model3d optimizer ticpp)
TARGET_LINK_LIBRARIES(unittests gflags glog gfortran snopt)
if (UNIX)
  TARGET_LINK_LIBRARIES(unittests gtest pthread glut)
endif()
if (WIN32)
  TARGET_LINK_LIBRARIES(unittests debug gtestd optimized gtest)
  TARGET_LINK_LIBRARIES(unittests debug gtest_maind optimized gtest_main)
endif()

###############################################
# YUI
PROJECT(YUI)
# Search all header and source files
FILE(GLOB YUI_srcs "src/YUI/*.cpp")
FILE(GLOB YUI_hdrs "src/YUI/*.h")
# Library 
ADD_LIBRARY(YUI ${YUI_srcs} ${YUI_hdrs})
SET_TARGET_PROPERTIES(YUI PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "lib")

###############################################
# renderer
PROJECT(renderer)
# Search all header and source files
FILE(GLOB renderer_srcs "src/renderer/*.cpp")
FILE(GLOB renderer_hdrs "src/renderer/*.h")
# Library 
ADD_LIBRARY(renderer ${renderer_srcs} ${renderer_hdrs})
SET_TARGET_PROPERTIES(renderer PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "lib")

###############################################
# apps/exampleUI
PROJECT(exampleUI)
FILE(GLOB example_ui_srcs "src/apps/exampleUI/*.cpp")
FILE(GLOB example_ui_hdrs "src/apps/exampleUI/*.h")
ADD_EXECUTABLE(exampleUI ${example_ui_srcs} ${example_ui_hdrs})
TARGET_LINK_LIBRARIES(exampleUI YUI renderer)
TARGET_LINK_LIBRARIES(exampleUI glut GL GLU)

###############################################
# apps/loadModel
PROJECT(loadModel)
FILE(GLOB load_model_srcs "src/apps/loadModel/*.cpp")
FILE(GLOB load_model_hdrs "src/apps/loadModel/*.h")
ADD_EXECUTABLE(loadModel ${load_model_srcs} ${load_model_hdrs})
TARGET_LINK_LIBRARIES(loadModel model3d ticpp YUI renderer gflags glog)
TARGET_LINK_LIBRARIES(loadModel glut GL GLU)

###############################################
# apps/viewer
PROJECT(viewer)
FILE(GLOB viewer_srcs "src/apps/viewer/*.cpp")
FILE(GLOB viewer_hdrs "src/apps/viewer/*.h")
ADD_EXECUTABLE(viewer ${viewer_srcs} ${viewer_hdrs})
TARGET_LINK_LIBRARIES(viewer model3d ticpp YUI renderer)
TARGET_LINK_LIBRARIES(viewer glut GL GLU glog gflags)


###############################################
# apps/sampleOpt
PROJECT(sampleOpt)
FILE(GLOB sampleOpt_srcs "src/apps/sampleOpt/*.cpp")
FILE(GLOB sampleOpt_hdrs "src/apps/sampleOpt/*.h")
ADD_EXECUTABLE(sampleOpt ${sampleOpt_srcs} ${sampleOpt_hdrs})
TARGET_LINK_LIBRARIES(sampleOpt optimizer)
TARGET_LINK_LIBRARIES(sampleOpt glog gflags gfortran snopt)

###############################################
# apps/simpleik
PROJECT(simpleik)
FILE(GLOB simpleik_srcs "src/apps/simpleik/*.cpp")
FILE(GLOB simpleik_hdrs "src/apps/simpleik/*.h")
ADD_EXECUTABLE(simpleik ${simpleik_srcs} ${simpleik_hdrs})
TARGET_LINK_LIBRARIES(simpleik model3d optimizer ticpp)
TARGET_LINK_LIBRARIES(simpleik glut GL GLU)
TARGET_LINK_LIBRARIES(simpleik glog gflags gfortran snopt)


###############################################
# Doxygen
FIND_PACKAGE(Doxygen)
IF (DOXYGEN_FOUND)

  SET(DOXYGEN_INPUT "Doxyfile")
  SET(DOXYGEN_OUTPUT "doc")

  ADD_CUSTOM_COMMAND(
    OUTPUT ${DOXYGEN_OUTPUT}
    COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_INPUT}
    COMMAND ${CMAKE_COMMAND} -E echo "Done."
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    DEPENDS ${DOXYGEN_INPUT}
    )

  ADD_CUSTOM_TARGET(docs DEPENDS ${DOXYGEN_OUTPUT})
  # ADD_CUSTOM_TARGET(apidoc ALL DEPENDS ${DOXYGEN_OUTPUT})

  ADD_CUSTOM_TARGET(docs_forced
    COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_INPUT}
    COMMAND ${CMAKE_COMMAND} -E echo "Done."
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

ENDIF (DOXYGEN_FOUND)
