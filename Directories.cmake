message (STATUS "Setting up dependecies configurations")

# third party libraries
if (UNIX)
    message(STATUS "Operating system = Unix")
else()
    message(STATUS "Operating system = WIN32")
endif()

    INCLUDE_DIRECTORIES(${THIRDPARTY_INSTALL_PREFIX}/include)
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/eigen-3.0.5)
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/gtest-1.6.0/include)
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/ann_1.1.2/include)
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/libccd/src)
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/fcl/include)

    LINK_DIRECTORIES(${THIRDPARTY_INSTALL_PREFIX}/lib)
    LINK_DIRECTORIES(${CMAKE_INSTALL_PREFIX}/lib)
    LINK_DIRECTORIES(${CMAKE_SOURCE_DIR}/bin)

# glut
if (WIN32)
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/glut-3.7.6/include)
endif()
