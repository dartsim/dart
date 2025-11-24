#include "DynamicsTestFixture.hpp"

TEST_F(DynamicsTest, Jacobians)
{
  for (const auto& uri : getList()) {
#if DART_BUILD_MODE_DEBUG
    DART_DEBUG("{}", uri.toString());
#endif
    testJacobians(uri);
  }
}

TEST_F(DynamicsTest, ForwardKinematics)
{
  for (const auto& uri : getList()) {
#if DART_BUILD_MODE_DEBUG
    DART_DEBUG("{}", uri.toString());
#endif
    testForwardKinematics(uri);
  }
}
