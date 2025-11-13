/*
 * Copyright (c) 2011-2025, The DART development contributors
 * All rights reserved.
 *
 * This file validates that dart/config.hpp exposes the new DART_HAVE_* feature
 * macros so downstream code and these unit tests can rely on the definitions.
 */

#include <dart/config.hpp>

#include <gtest/gtest.h>

#ifndef DART_HAVE_BULLET
  #error "DART_HAVE_BULLET must be defined"
#endif
#ifndef DART_HAVE_ODE
  #error "DART_HAVE_ODE must be defined"
#endif
#ifndef DART_HAVE_OCTOMAP
  #error "DART_HAVE_OCTOMAP must be defined"
#endif
#ifndef DART_HAVE_IPOPT
  #error "DART_HAVE_IPOPT must be defined"
#endif
#ifndef DART_HAVE_NLOPT
  #error "DART_HAVE_NLOPT must be defined"
#endif
#ifndef DART_HAVE_PAGMO
  #error "DART_HAVE_PAGMO must be defined"
#endif
#ifndef DART_HAVE_SNOPT
  #error "DART_HAVE_SNOPT must be defined"
#endif
#ifndef DART_HAVE_GUI_OSG
  #error "DART_HAVE_GUI_OSG must be defined"
#endif

namespace dart::unittest {

TEST(ConfigMacros, FeatureFlagsAreDefined)
{
  // The static_asserts above ensure the macros exist; the runtime body is only
  // here so the test suite reports a clear pass.
  SUCCEED();
}

} // namespace dart::unittest
