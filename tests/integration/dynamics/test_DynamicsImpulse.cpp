#include "DynamicsTestFixture.hpp"

TEST_F(DynamicsTest, ConstraintImpulse)
{
  for (const auto& uri : getList()) {
#if DART_BUILD_MODE_DEBUG
    DART_DEBUG("{}", uri.toString());
#endif
    testConstraintImpulse(uri);
  }
}

TEST_F(DynamicsTest, ImpulseBasedDynamics)
{
  for (const auto& uri : getList()) {
#if DART_BUILD_MODE_DEBUG
    DART_DEBUG("{}", uri.toString());
#endif
    testImpulseBasedDynamics(uri);
  }
}
