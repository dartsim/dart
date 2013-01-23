###############################################
# Collision
project(collision)

# Search all header and source files
file(GLOB collision_srcs "*.cpp")
file(GLOB collision_hdrs "*.h")

# Library
add_library_to_dart(collision "${collision_srcs}" "${collision_hdrs}" "kinematics ${Boost_LIBRARIES}")
