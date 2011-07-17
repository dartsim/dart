CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

message(STATUS "Generate makefile/project file for GroundZero")

message(STATUS "CMAKE_BINARY_DIR = ${CMAKE_BINARY_DIR}")

###############################################
# SETTING: Global settings
INCLUDE_DIRECTORIES("src")

SET(CMAKE_CXX_WARNING_LEVEL 4)

if (WIN32)
#  message(STATUS "Setup Visual Studio Specifig Flags")
#  SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd /Zi /GL /Gy")
#  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT /Zi /GL /Gy")
#elseif (APPLE)
#elseif (UNIX AND NOT APPLE)
elseif (UNIX)  
  # SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -msse2")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2")
endif()

###############################################
# User specific settings
INCLUDE(Directories.cmake)

###############################################
# Find important packages
FIND_PACKAGE(OpenGL)
IF (OPENGL_FOUND)
MESSAGE("OPENGL FOUND")
MESSAGE(${OPENGL_LIBRARIES})
ELSE (OPENGL_FOUND)
MESSAGE("OPENGL NOT FOUND")
ENDIF ()

if (WIN32)
    SET(GLUT_glut_LIBRARY glut32)
    MESSAGE("GLUT FOUND")
    MESSAGE(${GLUT_glut_LIBRARY})
else()
    FIND_PACKAGE(GLUT)
    IF (GLUT_FOUND)
        MESSAGE("GLUT FOUND")
        MESSAGE(${GLUT_glut_LIBRARY})
    ELSE (GLUT_FOUND)
        MESSAGE("GLUT NOT FOUND")
    ENDIF ()
endif()

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
