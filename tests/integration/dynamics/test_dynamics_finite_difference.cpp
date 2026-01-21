#include "dynamics_test_fixture.hpp"

TEST_F(DynamicsTest, FiniteDifference)
{
  for (const auto& uri : getList()) {
#if !defined(NDEBUG)
    DART_DEBUG("{}", uri.toString());
#endif
    testFiniteDifferenceGeneralizedCoordinates(uri);
    testFiniteDifferenceBodyNodeVelocity(uri);
    testFiniteDifferenceBodyNodeAcceleration(uri);
  }
}
