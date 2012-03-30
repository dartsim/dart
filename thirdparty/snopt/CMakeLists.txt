cmake_minimum_required (VERSION 2.8)

project (snopt)
enable_language (Fortran)

FILE(GLOB snopt_srcs "*.F")

ADD_LIBRARY(snopt STATIC ${snopt_srcs})
SET_TARGET_PROPERTIES(snopt PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
INSTALL(TARGETS snopt ARCHIVE DESTINATION lib)
