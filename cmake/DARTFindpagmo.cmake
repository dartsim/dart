# Copyright (c) 2011-2021, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

find_package(pagmo QUIET CONFIG)

dart_find_package(NLOPT)
dart_find_package(IPOPT)

if(TARGET Pagmo::pagmo)

  # Check for pagmo optional dependencies
  include(CheckCXXSourceCompiles)
  set(CMAKE_REQUIRED_DEFINITIONS "")
  set(CMAKE_REQUIRED_LIBRARIES Pagmo::pagmo)
  set(CMAKE_REQUIRED_FLAGS "-std=c++11 -w")

  check_cxx_source_compiles(
    "
    #include <pagmo/config.hpp>
    int main()
    {
    #if defined(PAGMO_WITH_NLOPT)
      static_assert(true, \"Pagmo is build with NLOPT\");
    #else
      static_assert(false, \"Pagmo is NOT build with NLOPT\");
    #endif
      return 0;
    }
    "
    PAGMO_BUILT_WITH_NLOPT
  )

  check_cxx_source_compiles(
    "
    #include <pagmo/config.hpp>
    int main()
    {
    #if defined(PAGMO_WITH_IPOPT)
      static_assert(true, \"Pagmo is build with IPOPT\");
    #else
      static_assert(false, \"Pagmo is NOT build with IPOPT\");
    #endif
      return 0;
    }
    "
    PAGMO_BUILT_WITH_IPOPT
  )

  unset(CMAKE_REQUIRED_FLAGS)
  unset(CMAKE_REQUIRED_LIBRARIES)
  unset(CMAKE_REQUIRED_DEFINITIONS)

  if(PAGMO_BUILT_WITH_NLOPT)
    dart_find_package(NLOPT)
    if(NOT TARGET NLOPT::nlopt)
      message(WARNING
        "The installed version of pagmo is built with nlopt, but nlopt is not "
        "found. Please install nlopt to use dart-optimization-pagmo."
      )
      set(pagmo_FOUND FALSE)
    endif()
  endif()

  if(PAGMO_BUILT_WITH_IPOPT)
    dart_find_package(IPOPT)
    if(NOT TARGET IPOPT::ipopt)
      message(WARNING
        "The installed version of pagmo is built with ipopt, but ipopt is not "
        "found. Please install ipopt to use dart-optimization-pagmo."
      )
      set(pagmo_FOUND FALSE)
    endif()
  endif()

endif()
