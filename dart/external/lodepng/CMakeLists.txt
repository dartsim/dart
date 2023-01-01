# Search all header and source files
file(GLOB hdrs "*.h")
file(GLOB srcs "*.cpp")

# Set local target name
set(target_name ${PROJECT_NAME}-external-lodepng)

dart_add_library(${target_name} STATIC ${hdrs} ${srcs})
target_include_directories(${target_name}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
