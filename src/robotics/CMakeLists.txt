###############################################
# robotics
project(robotics)

# Search all header and source files
file(GLOB robotics_srcs "*.cpp")
file(GLOB robotics_hdrs "*.h")

# Library
add_library_to_dart(robotics "${robotics_srcs}" "${robotics_hdrs}" "utils integration lcpsolver dynamics ${Boost_LIBRARIES}")