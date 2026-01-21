#include "dynamics_test_fixture.hpp"

TEST_F(DynamicsTest, CenterOfMass)
{
  for (const auto& uri : getList()) {
#if !defined(NDEBUG)
    DART_DEBUG("{}", uri.toString());
#endif
    testCenterOfMass(uri);
  }
}

TEST_F(DynamicsTest, CenterOfMassFreeFall)
{
  for (const auto& uri : getList()) {
#if !defined(NDEBUG)
    DART_DEBUG("{}", uri.toString());
#endif
    testCenterOfMassFreeFall(uri);
  }
}
