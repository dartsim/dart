###############################################
# Dynamics
project(dynamics)

# Search all header and source files
file(GLOB dynamics_srcs "*.cpp")
file(GLOB dynamics_hdrs "*.h")

# Library 
add_library_to_dart(dynamics "${dynamics_srcs}" "${dynamics_hdrs}" "")