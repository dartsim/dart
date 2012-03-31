###############################################
# optimizer
PROJECT(optimizer)

# Search all header and source files
FILE(GLOB optimizer_srcs "*.cpp")
FILE(GLOB optimizer_hdrs "*.h")
FILE(GLOB optimizer_snopt_srcs "snopt/*.cpp")
FILE(GLOB optimizer_snopt_hdrs "snopt/*.h")

# Library 
ADD_LIBRARY(optimizer ${optimizer_srcs} ${optimizer_hdrs} ${optimizer_snopt_srcs} ${optimizer_snopt_hdrs})
TARGET_LINK_LIBRARIES(optimizer utils)
SET_TARGET_PROPERTIES(optimizer PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
