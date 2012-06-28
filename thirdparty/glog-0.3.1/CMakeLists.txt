PROJECT(glog-wrapper)

include(ExternalProject)

ExternalProject_Add(
	glog-ext
	SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}
	CONFIGURE_COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/configure --prefix=${THIRDPARTY_INSTALL_PREFIX}
#	INSTALL_COMMAND ""
	)

Set(GLOG_INCLUDE_PATH ${CMAKE_CURRENT_BINARY_DIR}/glog-ext-prefix/src/glog-ext-build/src/ PARENT_SCOPE)

