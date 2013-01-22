###############################################
# lcpsolver
project(lcpsolver)

# Search all header and source files
file(GLOB lcpsolver_srcs "*.cpp")
file(GLOB lcpsolver_hdrs "*.h")

# Library 
add_library_to_dart(lcpsolver "${lcpsolver_srcs}" "${lcpsolver_hdrs}" "")