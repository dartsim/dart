# Copyright (c) 2011, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

find_package(sdformat QUIET CONFIG)

if(sdformat_FOUND)
  set(SDFORMAT_FOUND TRUE)
  if(TARGET sdformat::sdformat)
    set(SDFORMAT_LIBRARIES sdformat::sdformat)
  endif()
  if(DEFINED sdformat_INCLUDE_DIRS)
    set(SDFORMAT_INCLUDE_DIRS ${sdformat_INCLUDE_DIRS})
  endif()
else()
  set(SDFORMAT_FOUND FALSE)
endif()
