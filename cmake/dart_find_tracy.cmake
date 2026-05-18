# Copyright (c) 2011, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

find_package(Tracy QUIET CONFIG)

if(Tracy_FOUND OR TARGET Tracy::TracyClient)
  set(TRACY_FOUND TRUE)
  set(Tracy_FOUND TRUE)
else()
  set(TRACY_FOUND FALSE)
  set(Tracy_FOUND FALSE)
endif()
