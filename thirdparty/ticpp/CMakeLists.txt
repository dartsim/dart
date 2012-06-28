PROJECT(ticpp-wrapper)

include(ExternalProject)

set(PREMAKE_GENERATOR gmake)
if(CMAKE_GENERATOR STREQUAL "Unix Makefiles")
    set(PREMAKE_GENERATOR gmake)
elseif(CMAKE_GENERATOR STREQUAL "MinGW Makefiles")
    set(PREMAKE_GENERATOR gmake)
elseif(CMAKE_GENERATOR STREQUAL "Visual Studio 7")
    set(PREMAKE_GENERATOR vs2002)
elseif(CMAKE_GENERATOR STREQUAL "Visual Studio 7 .NET 2003")
    set(PREMAKE_GENERATOR vs2003)
elseif(CMAKE_GENERATOR MATCHES "Visual Studio 8 2005.*")
    set(PREMAKE_GENERATOR vs2005)
elseif(CMAKE_GENERATOR MATCHES "Visual Studio 9 2008.*")
    set(PREMAKE_GENERATOR vs2008)
elseif(CMAKE_GENERATOR MATCHES "Visual Studio 10 2010.*")
    set(PREMAKE_GENERATOR vs2010)
elseif(CMAKE_GENERATOR MATCHES "CodeBlocks.*")
    set(PREMAKE_GENERATOR codeblocks)
else()
    MESSAGE(FATAL_ERROR "No mapping to premake generator for: ${CMAKE_GENERATOR}")
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(TICPP_LIBRARY libticppd)
    set(TICPP_BUILD_CMD make config=debug)
else()
    set(TICPP_LIBRARY libticpp)
    set(TICPP_BUILD_CMD make config=release)
endif()

set(TICPP_INSTALL_CMD
    ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/lib/${TICPP_LIBRARY}.a ${THIRDPARTY_INSTALL_PREFIX}/lib/)

ExternalProject_Add(
	ticpp-ext
	SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}
	CONFIGURE_COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/premake4 ${PREMAKE_GENERATOR}
	BUILD_COMMAND ${TICPP_BUILD_CMD}
	BUILD_IN_SOURCE 1
	INSTALL_COMMAND ${TICPP_INSTALL_CMD}
	)

#Set(TICPP_INCLUDE_PATH ${CMAKE_CURRENT_BINARY_DIR}/ticpp-ext-prefix/src/ticpp-ext-build/src/ PARENT_SCOPE)

