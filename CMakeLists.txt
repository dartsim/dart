CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

message(STATUS "Generate makefile/project file for GroundZero")

message(STATUS "CMAKE_BINARY_DIR = ${CMAKE_BINARY_DIR}")

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
    LINK_DIRECTORIES(/home/sehoon/dev/packages/gtest-1.6.0/lib/.libs/)
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

ADD_SUBDIRECTORY(src)

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
