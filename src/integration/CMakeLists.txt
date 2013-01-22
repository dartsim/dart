###############################################
# integration
project(integration)

# Search all header and source files
file(GLOB integration_srcs "*.cpp")
file(GLOB integration_hdrs "*.h")

# Library 
add_library_to_dart(integration "${integration_srcs}" "${integration_hdrs}" "")