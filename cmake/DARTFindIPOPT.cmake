# Copyright (c) 2011-2021, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

find_package(IPOPT 3.11.9 QUIET MODULE)

# HAVE_CSTDDEF is necessary to workaround this bug:
# https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=684062
if(IPOPT_FOUND AND NOT TARGET IPOPT::ipopt)
  add_library(IPOPT::ipopt INTERFACE IMPORTED)
  set_target_properties(IPOPT::ipopt PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${IPOPT_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${IPOPT_LIBRARIES}"
    INTERFACE_COMPILE_DEFINITIONS HAVE_CSTDDEF
  )
endif()
