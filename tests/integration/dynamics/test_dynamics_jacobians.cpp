#include "dynamics_test_fixture.hpp"

TEST_F(DynamicsTest, Jacobians)
{
  for (const auto& uri : getList()) {
#if !defined(NDEBUG)
    DART_DEBUG("{}", uri.toString());
#endif
    testJacobians(uri);
  }
}

TEST_F(DynamicsTest, ForwardKinematics)
{
  for (const auto& uri : getList()) {
#if !defined(NDEBUG)
    DART_DEBUG("{}", uri.toString());
#endif
    testForwardKinematics(uri);
  }
}
