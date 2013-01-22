###############################################
# Utils
project(utils)

# Search all header and source files
file(GLOB_RECURSE utils_srcs "*.cpp")
file(GLOB_RECURSE utils_hdrs "*.h")

# Library 
add_library_to_dart(utils "${utils_srcs}" "${utils_hdrs}" "")