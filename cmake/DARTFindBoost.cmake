# Copyright (c) 2011-2019, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

set(DART_MIN_BOOST_VERSION 1.58.0 CACHE INTERNAL "Boost min version requirement" FORCE)
if(MSVC)
  add_definitions(-DBOOST_ALL_NO_LIB)
endif()
add_definitions(-DBOOST_TEST_DYN_LINK)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
if(MSVC)
  set(BOOST_REQUIRED_COMPONENTS system filesystem)
else()
  set(BOOST_REQUIRED_COMPONENTS regex system filesystem)
endif()
if(DART_VERBOSE)
  find_package(Boost ${DART_MIN_BOOST_VERSION} REQUIRED COMPONENTS ${BOOST_REQUIRED_COMPONENTS})
else()
  find_package(Boost ${DART_MIN_BOOST_VERSION} QUIET REQUIRED COMPONENTS ${BOOST_REQUIRED_COMPONENTS})
endif()
