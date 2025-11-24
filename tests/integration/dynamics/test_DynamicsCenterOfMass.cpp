#include "DynamicsTestFixture.hpp"

TEST_F(DynamicsTest, CenterOfMass)
{
  for (const auto& uri : getList()) {
#if DART_BUILD_MODE_DEBUG
    DART_DEBUG("{}", uri.toString());
#endif
    testCenterOfMass(uri);
  }
}

TEST_F(DynamicsTest, CenterOfMassFreeFall)
{
  for (const auto& uri : getList()) {
#if DART_BUILD_MODE_DEBUG
    DART_DEBUG("{}", uri.toString());
#endif
    testCenterOfMassFreeFall(uri);
  }
}
