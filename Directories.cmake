message(STATUS "Retrieve configuration for Karen")

if (UNIX AND NOT APPLE)
    message(STATUS "Operating system = Linux")
    INCLUDE_DIRECTORIES(/usr/local/include/eigen3)
    INCLUDE_DIRECTORIES(/usr/local/include/ticpp)
    INCLUDE_DIRECTORIES(~/dev/packages/gtest-1.6.0/include)
    LINK_DIRECTORIES(/home/sehoon/dev/packages/gtest-1.6.0/lib/.libs/)
elseif (WIN32)
    message(STATUS "Operating system = WIN32")
    INCLUDE_DIRECTORIES(c:/dev/external/include)
    INCLUDE_DIRECTORIES(c:/dev/boost_1_46_1)
    LINK_DIRECTORIES(c:/dev/external/lib)

    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/eigen-3.0.1)
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/gflags-1.5/src/windows)
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/glog-0.3.1/src/windows)
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/glut-3.7.6/include)
    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/ticpp)

    LINK_DIRECTORIES(${CMAKE_SOURCE_DIR}/thirdparty/lib)

elseif (APPLE)
    message(STATUS "Operating system = APPLE")
    INCLUDE_DIRECTORIES(/Users/karenliu/Research/eigen-3.0)
    INCLUDE_DIRECTORIES(/Users/karenliu/Research/glog-0.3.1/src)
    LINK_DIRECTORIES   (/usr/local/lib)

    INCLUDE_DIRECTORIES(/Users/karenliu/Research/ticpp)
    LINK_DIRECTORIES   (/Users/karenliu/Research/ticpp/lib)

    INCLUDE_DIRECTORIES(/Users/karenliu/Research/RTQL8/thirdparty/gtest-1.6.0/include)
    LINK_DIRECTORIES   (/Users/karenliu/Research/RTQL8/thirdparty/gtest-1.6.0)

    INCLUDE_DIRECTORIES(/Users/karenliu/Research/RTQL8/thirdparty/gflags-1.5/src)
    LINK_DIRECTORIES   (/Users/karenliu/Research/RTQL8/thirdparty/gflags-1.5)

    LINK_DIRECTORIES   (/Users/karenliu/Research/snopt)
else()
    message(FATAL_ERROR "Invalid operating system")
endif()
