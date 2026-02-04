#include "dynamics_test_fixture.hpp"

TEST_F(DynamicsTest, InverseDynamics)
{
  for (const auto& uri : getList()) {
#if !defined(NDEBUG)
    DART_DEBUG("{}", uri.toString());
#endif
    testInverseDynamics(uri);
  }
}

TEST_F(DynamicsTest, EquationsOfMotion)
{
  for (const auto& uri : getList()) {
    ////////////////////////////////////////////////////////////////////////////
    // TODO(JS): Following skel files, which contain euler joints couldn't
    //           pass EQUATIONS_OF_MOTION, are disabled.
    if (uri.toString() == "dart://sample/skel/test/double_pendulum_euler_joint.skel"
        || uri.toString() == "dart://sample/skel/test/chainwhipa.skel"
        || uri.toString() == "dart://sample/skel/test/serial_chain_eulerxyz_joint.skel"
        || uri.toString() == "dart://sample/skel/test/simple_tree_structure_euler_joint.skel"
        || uri.toString() == "dart://sample/skel/test/tree_structure_euler_joint.skel"
        || uri.toString() == "dart://sample/skel/fullbody1.skel")
    {
      continue;
    }
    ////////////////////////////////////////////////////////////////////////////

#if !defined(NDEBUG)
    DART_DEBUG("{}", uri.toString());
#endif
    compareEquationsOfMotion(uri);
  }
}
