message (STATUS "Setting up dependecies configurations")

include_directories(BEFORE ${THIRDPARTY_INSTALL_PREFIX}/include)
include_directories(BEFORE ${THIRDPARTY_INSTALL_PREFIX}/gtest-1.6.0/include)
include_directories(BEFORE ${THIRDPARTY_INSTALL_PREFIX}/tinyxml2)
include_directories(BEFORE ${THIRDPARTY_INSTALL_PREFIX}/tinyxml)

link_directories(${THIRDPARTY_INSTALL_PREFIX}/lib)
link_directories(${CMAKE_INSTALL_PREFIX}/lib)
link_directories(${CMAKE_SOURCE_DIR}/bin)
