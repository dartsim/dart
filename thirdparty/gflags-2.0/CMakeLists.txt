PROJECT(gflags-wrapper)

include(ExternalProject)

FUNCTION(INCLUDE_GFLAGS_VS)
    INCLUDE_EXTERNAL_MSPROJECT( gflags-ext ${CMAKE_CURRENT_SOURCE_DIR}/vsprojects/libgflags/libgflags.vcproj )
    SET_TARGET_PROPERTIES( gflags-ext PROPERTIES FOLDER ThirdParty/gflags )
    FILE(COPY ${CMAKE_CURRENT_SOURCE_DIR}/src/windows/gflags
         DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/include/gflags)
    Set(GFLAGS_INCLUDE_PATH ${CMAKE_CURRENT_BINARY_DIR}/include/gflags/ PARENT_SCOPE)
ENDFUNCTION(INCLUDE_GFLAGS_VS)

if(CMAKE_GENERATOR MATCHES "Visual Studio 7")
    INCLUDE_GFLAGS_VS()
elseif(CMAKE_GENERATOR MATCHES "Visual Studio 8")
    INCLUDE_GFLAGS_VS()
elseif(CMAKE_GENERATOR MATCHES "Visual Studio 9")
    INCLUDE_GFLAGS_VS()
elseif(CMAKE_GENERATOR MATCHES "Visual Studio 10")
    INCLUDE_GFLAGS_VS()
else()
    ExternalProject_Add(
        gflags-ext
        SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}
        CONFIGURE_COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/configure --prefix=${THIRDPARTY_INSTALL_PREFIX}
    #	INSTALL_COMMAND ""
        )
    Set(GFLAGS_INCLUDE_PATH ${CMAKE_CURRENT_BINARY_DIR}/gflags-ext-prefix/src/gflags-ext-build/src/)
endif()

Set(GFLAGS_INCLUDE_PATH ${GFLAGS_INCLUDE_PATH} PARENT_SCOPE)


