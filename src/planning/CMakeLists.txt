###############################################
# planning
project(planning)

# Search all header and source files
file(GLOB planning_srcs "*.cpp")
file(GLOB planning_hdrs "*.h")
file(GLOB planning_hpps "*.hpp")

list(APPEND planning_hdrs ${planning_hpps})

# Library
add_library_to_dart(planning "${planning_srcs}" "${planning_hdrs}" "utils")