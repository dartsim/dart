#include "dynamics_test_fixture.hpp"

TEST_F(DynamicsTest, ConstraintImpulse)
{
  for (const auto& uri : getList()) {
#if !defined(NDEBUG)
    DART_DEBUG("{}", uri.toString());
#endif
    testConstraintImpulse(uri);
  }
}

TEST_F(DynamicsTest, ImpulseBasedDynamics)
{
  for (const auto& uri : getList()) {
#if !defined(NDEBUG)
    DART_DEBUG("{}", uri.toString());
#endif
    testImpulseBasedDynamics(uri);
  }
}
