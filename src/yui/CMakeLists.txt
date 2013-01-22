###############################################
# YUI
project(yui)

# Search all header and source files
file(GLOB yui_srcs "*.cpp")
file(GLOB yui_hdrs "*.h")

# Library 
add_library_to_dart(yui "${yui_srcs}" "${yui_hdrs}" "")