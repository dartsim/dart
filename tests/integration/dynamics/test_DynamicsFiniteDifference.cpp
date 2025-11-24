#include "DynamicsTestFixture.hpp"

TEST_F(DynamicsTest, FiniteDifference)
{
  for (const auto& uri : getList()) {
#if DART_BUILD_MODE_DEBUG
    DART_DEBUG("{}", uri.toString());
#endif
    testFiniteDifferenceGeneralizedCoordinates(uri);
    testFiniteDifferenceBodyNodeVelocity(uri);
    testFiniteDifferenceBodyNodeAcceleration(uri);
  }
}
