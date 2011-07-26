message (STATUS "Setting up dependecies configurations")

# third party libraries
if (UNIX)
    message(STATUS "Operating system = Unix")
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/eigen-3.0.1)
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/gflags-1.5/src)
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/glog-0.3.1/src)
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/gtest-1.6.0/include)
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/ticpp)
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/FreeImage-3.15.0/Dist)

    LINK_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/lib)
    LINK_DIRECTORIES(${CMAKE_SOURCE_DIR}/bin)

else()
    message(STATUS "Operating system = WIN32")
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/eigen-3.0.1)
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/gflags-1.5/src/windows)
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/glog-0.3.1/src/windows)
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/glut-3.7.6/include)
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/ticpp)

    LINK_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/lib)
endif()

# glut
if (WIN32)
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/glut-3.7.6/include)
endif()
