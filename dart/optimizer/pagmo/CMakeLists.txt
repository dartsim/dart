# Copyright (c) 2011-2022, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

if(THREADS_HAVE_PTHREAD_ARG)
  set(target_compile_options_public "-pthread")
endif()
if(CMAKE_THREAD_LIBS_INIT)
  set(target_link_libraries_public ${CMAKE_THREAD_LIBS_INIT})
endif()

dart_add_component_sub_directory(
  COMPONENT_DEPENDENCY_PACKAGES
    NLOPT pagmo
  TARGET_LINK_LIBRARIES_PUBLIC
    Pagmo::pagmo
    ${target_link_libraries_public}
  TARGET_COMPILE_OPTIONS_PUBLIC
    ${target_compile_options_public}
)
