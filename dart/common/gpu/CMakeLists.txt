# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

if(APPLE)
  # -DOpenCLHeaders_DIR=$(brew --prefix opencl-headers)/share/cmake/OpenCLHeaders -DOpenCLHeadersCpp_DIR=$(brew --prefix opencl-clhpp-headers)/share/cmake/OpenCLHeadersCpp
  if(DART_ENABLE_GPU AND OpenCL_FOUND AND OpenCLHeaders_FOUND AND OpenCLHeadersCpp_FOUND)
    list(APPEND dependent_packages_required OpenCL OpenCLHeaders OpenCLHeadersCpp)
    list(APPEND target_link_libraries_public OpenCL::OpenCL OpenCL::Headers OpenCL::HeadersCpp)
    list(APPEND target_link_libraries_public_skip_checking "-framework OpenCL")
  endif()
else()
  if(DART_ENABLE_GPU AND OpenCL_FOUND)
    list(APPEND dependent_packages_required OpenCL)
    list(APPEND target_link_libraries_public OpenCL::OpenCL)
  endif()
endif()

dart_add_component_sub_directory(
  DEPENDENT_PACKAGES_REQUIRED
    ${dependent_packages_required}
  TARGET_LINK_LIBRARIES_PUBLIC
    ${target_link_libraries_public}
  TARGET_LINK_LIBRARIES_PUBLIC_SKIP_CHECKING
    ${target_link_libraries_public_skip_checking}
)
