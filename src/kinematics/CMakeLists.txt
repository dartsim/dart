###############################################
# Kinematics
project(kinematics)

# Search all header and source files
file(GLOB kinematics_srcs "*.cpp")
file(GLOB kinematics_hdrs "*.h")
file(GLOB kinematics_hpps "*.hpp")

list(APPEND kinematics_hdrs ${kinematics_hpps})

# Library 
add_library_to_dart(kinematics "${kinematics_srcs}" "${kinematics_hdrs}" "utils")