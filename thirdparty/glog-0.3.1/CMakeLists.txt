PROJECT(glog-wrapper)

include(ExternalProject)

FUNCTION(INCLUDE_GLOG_VS PJEXT)
    INCLUDE_EXTERNAL_MSPROJECT( glog-ext ${CMAKE_CURRENT_SOURCE_DIR}/vsprojects/libglog/libglog.${PJEXT} )
    FILE(COPY ${CMAKE_CURRENT_SOURCE_DIR}/src/windows/glog
         DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/include/glog)
    Set(GLOG_INCLUDE_PATH ${CMAKE_CURRENT_BINARY_DIR}/include/glog/ PARENT_SCOPE)
ENDFUNCTION(INCLUDE_GLOG_VS)

INCLUDE_DIRECTORIES( ${GFLAGS_INCLUDE_PATH} )

if(CMAKE_GENERATOR MATCHES "Visual Studio 7")
    MESSAGE(FATAL_ERROR "This generator (${CMAKE_GENERATOR}) is not supported by glog.)
elseif(CMAKE_GENERATOR MATCHES "Visual Studio 8")
    MESSAGE(FATAL_ERROR "This generator (${CMAKE_GENERATOR}) is not supported by glog.)
elseif(CMAKE_GENERATOR MATCHES "Visual Studio 9")
    INCLUDE_GLOG_VS(vcproj)
elseif(CMAKE_GENERATOR MATCHES "Visual Studio 10")
    INCLUDE_GLOG_VS(vcxproj)
else()
    ExternalProject_Add(
        glog-ext
        SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}
        CONFIGURE_COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/configure --prefix=${THIRDPARTY_INSTALL_PREFIX}
    #	INSTALL_COMMAND ""
        )
    Set(GLOG_INCLUDE_PATH ${CMAKE_CURRENT_BINARY_DIR}/glog-ext-prefix/src/glog-ext-build/src/)
endif()

Set(GLOG_INCLUDE_PATH ${GLOG_INCLUDE_PATH} PARENT_SCOPE)


