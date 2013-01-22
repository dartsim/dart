###############################################
# renderer
project(renderer)

# Search all header and source files
file(GLOB renderer_srcs "*.cpp")
file(GLOB renderer_hdrs "*.h")

# Library 
add_library_to_dart(renderer "${renderer_srcs}" "${renderer_hdrs}" "assimp")