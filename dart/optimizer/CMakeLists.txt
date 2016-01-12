# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

# Add subdirectories
if(HAVE_IPOPT)
  add_subdirectory(ipopt)
endif()
if(HAVE_NLOPT)
  add_subdirectory(nlopt)
endif()
if(HAVE_SNOPT)
  add_subdirectory(snopt)
endif()

set(kido_optimizer_hdrs ${hdrs} ${kido_optimizer_hdrs} PARENT_SCOPE)
set(kido_optimizer_srcs ${srcs} ${kido_optimizer_srcs} PARENT_SCOPE)

# Library
#kido_add_library(kido_optimizer ${srcs} ${hdrs})
#target_link_libraries(
#  kido_optimizer
#  kido_common
#  kido_math
#  ${KIDO_DEPENDENCIES}
#)
#if(HAVE_IPOPT)
#  target_link_libraries(kido_optimizer kido_optimizer_ipopt)
#endif()
#if(HAVE_NLOPT)
#  target_link_libraries(kido_optimizer kido_optimizer_nlopt)
#endif()
#if(HAVE_SNOPT)
#  target_link_libraries(kido_optimizer kido_optimizer_snopt)
#endif()

# Generate header for this namespace
kido_get_filename_components(header_names "optimizer headers" ${hdrs})
if(HAVE_IPOPT)
  set(header_names ${header_names} ipopt/ipopt.h)
endif()
if(HAVE_NLOPT)
  set(header_names ${header_names} nlopt/nlopt.h)
endif()
if(HAVE_SNOPT)
  set(header_names ${header_names} snopt/snopt.h)
endif()
kido_generate_include_header_list(
  optimizer_headers
  "dart/optimizer/"
  "optimizer headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/optimizer.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/optimizer.h
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/optimizer.h
  DESTINATION include/dart/optimizer
  COMPONENT headers
)
#install(TARGETS kido_optimizer EXPORT KIDOTargets DESTINATION lib)

