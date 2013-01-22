###############################################
# optimizer
project(optimizer)

# Search all header and source files
file(GLOB optimizer_srcs "*.cpp")
file(GLOB optimizer_hdrs "*.h")
file(GLOB optimizer_snopt_srcs "snopt/*.cpp")
file(GLOB optimizer_snopt_hdrs "snopt/*.h")

list(APPEND optimizer_srcs ${optimizer_snopt_srcs})
list(APPEND optimizer_hdrs ${optimizer_snopt_hdrs})

# Library 
add_library_to_dart(optimizer "${optimizer_srcs}" "${optimizer_hdrs}" "utils")