PROJECT(gflags-wrapper)

include(ExternalProject)

ExternalProject_Add(
	gflags-ext
	SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}
	CONFIGURE_COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/configure --prefix=${THIRDPARTY_INSTALL_PREFIX}
#	INSTALL_COMMAND ""
	)

Set(GFLAGS_INCLUDE_PATH ${CMAKE_CURRENT_BINARY_DIR}/gflags-ext-prefix/src/gflags-ext-build/src/ PARENT_SCOPE)

