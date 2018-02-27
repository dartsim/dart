# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")
dart_add_core_headers(${hdrs} ${detail_hdrs})
dart_add_core_sources(${srcs} ${detail_srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "dynamics headers" ${hdrs})
# TODO: remove below line once the files are completely removed.
list(REMOVE_ITEM header_names "MultiSphereShape.hpp")
dart_generate_include_header_list(
  dynamics_headers
  "dart/dynamics/"
  "dynamics headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamics.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/dynamics.hpp
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/dynamics.hpp
  DESTINATION include/dart/dynamics
  COMPONENT headers
)
install(
  FILES ${detail_hdrs}
  DESTINATION include/dart/dynamics/detail
  COMPONENT headers
)

<<<<<<< HEAD
get_filename_component(formatting_file "BodyNodeDerivatives.hpp" ABSOLUTE)
dart_format_add(${formatting_file})
get_filename_component(formatting_file "BodyNodeDerivatives.cpp" ABSOLUTE)
dart_format_add(${formatting_file})
get_filename_component(formatting_file "BodyNodeSecondDerivatives.hpp" ABSOLUTE)
dart_format_add(${formatting_file})
get_filename_component(formatting_file "BodyNodeSecondDerivatives.cpp" ABSOLUTE)
dart_format_add(${formatting_file})
get_filename_component(formatting_file "SkeletonDerivatives.hpp" ABSOLUTE)
dart_format_add(${formatting_file})
get_filename_component(formatting_file "SkeletonDerivatives.cpp" ABSOLUTE)
dart_format_add(${formatting_file})
get_filename_component(formatting_file "SkeletonSecondDerivatives.hpp" ABSOLUTE)
dart_format_add(${formatting_file})
get_filename_component(formatting_file "SkeletonSecondDerivatives.cpp" ABSOLUTE)
dart_format_add(${formatting_file})
=======
dart_format_add(
  TranslationalJoint2D.hpp
  TranslationalJoint2D.cpp
  detail/TranslationalJoint2DAspect.hpp
  detail/TranslationalJoint2DAspect.cpp
)
>>>>>>> origin/release-6.4
