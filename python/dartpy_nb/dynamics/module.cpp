#include "dynamics/module.hpp"

#include "dynamics/ball_joint.hpp"
#include "dynamics/body_node.hpp"
#include "dynamics/chain.hpp"
#include "dynamics/degree_of_freedom.hpp"
#include "dynamics/entity.hpp"
#include "dynamics/frame.hpp"
#include "dynamics/jacobian_node.hpp"
#include "dynamics/free_joint.hpp"
#include "dynamics/inertia.hpp"
#include "dynamics/inverse_kinematics.hpp"
#include "dynamics/joint.hpp"
#include "dynamics/end_effector.hpp"
#include "dynamics/meta_skeleton.hpp"
#include "dynamics/euler_joint.hpp"
#include "dynamics/prismatic_joint.hpp"
#include "dynamics/linkage.hpp"
#include "dynamics/revolute_joint.hpp"
#include "dynamics/screw_joint.hpp"
#include "dynamics/shape.hpp"
#include "dynamics/shape_frame.hpp"
#include "dynamics/shape_node.hpp"
#include "dynamics/simple_frame.hpp"
#include "dynamics/hierarchical_ik.hpp"
#include "dynamics/skeleton.hpp"
#include "dynamics/planar_joint.hpp"
#include "dynamics/translational_joint.hpp"
#include "dynamics/translational_joint2d.hpp"
#include "dynamics/universal_joint.hpp"
#include "dynamics/weld_joint.hpp"
#include "dynamics/zero_dof_joint.hpp"

#include <cstdio>
#include <cstdlib>

namespace {

bool shouldTrace()
{
  static const bool enabled = std::getenv("DARTPY_NB_TRACE") != nullptr;
  return enabled;
}

void traceScope(const char* name)
{
  if (shouldTrace())
    std::fprintf(stderr, "[dartpy_nb][dynamics] %s\n", name);
}

} // namespace

namespace dart::python_nb {

void defDynamicsModule(nanobind::module_& m)
{
  traceScope("entity");
  defEntity(m);
  traceScope("joint");
  defJoint(m);
  traceScope("zero_dof_joint");
  defZeroDofJoint(m);
  traceScope("dof");
  defDegreeOfFreedom(m);
  traceScope("free_joint");
  defFreeJoint(m);
  traceScope("revolute_joint");
  defRevoluteJoint(m);
  traceScope("translational_joint2d");
  defTranslationalJoint2D(m);
  traceScope("prismatic_joint");
  defPrismaticJoint(m);
  traceScope("planar_joint");
  defPlanarJoint(m);
  traceScope("translational_joint");
  defTranslationalJoint(m);
  traceScope("euler_joint");
  defEulerJoint(m);
  traceScope("screw_joint");
  defScrewJoint(m);
  traceScope("universal_joint");
  defUniversalJoint(m);
  traceScope("ball_joint");
  defBallJoint(m);
  traceScope("weld_joint");
  defWeldJoint(m);
  traceScope("shape");
  defShape(m);
  traceScope("frame");
  defFrame(m);
  traceScope("jacobian_node");
  defJacobianNode(m);
  traceScope("end_effector");
  defEndEffector(m);
  traceScope("body_node");
  defBodyNode(m);
  traceScope("meta_skeleton");
  defMetaSkeleton(m);
  traceScope("linkage");
  defLinkage(m);
  traceScope("chain");
  defChain(m);
  traceScope("shape_frame");
  defShapeFrame(m);
  traceScope("shape_node");
  defShapeNode(m);
  traceScope("simple_frame");
  defSimpleFrame(m);
  traceScope("inverse_kinematics");
  defInverseKinematics(m);
  traceScope("hierarchical_ik");
  defHierarchicalIK(m);
  traceScope("skeleton");
  defSkeleton(m);
  traceScope("inertia");
  defInertia(m);
}

} // namespace dart::python_nb
