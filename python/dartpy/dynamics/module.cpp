#include "dynamics/module.hpp"

#include "dynamics/ball_joint.hpp"
#include "dynamics/body_node.hpp"
#include "dynamics/chain.hpp"
#include "dynamics/degree_of_freedom.hpp"
#include "dynamics/end_effector.hpp"
#include "dynamics/entity.hpp"
#include "dynamics/euler_joint.hpp"
#include "dynamics/frame.hpp"
#include "dynamics/free_joint.hpp"
#include "dynamics/hierarchical_ik.hpp"
#include "dynamics/inertia.hpp"
#include "dynamics/inverse_kinematics.hpp"
#include "dynamics/jacobian_node.hpp"
#include "dynamics/joint.hpp"
#include "dynamics/linkage.hpp"
#include "dynamics/meta_skeleton.hpp"
#include "dynamics/planar_joint.hpp"
#include "dynamics/prismatic_joint.hpp"
#include "dynamics/revolute_joint.hpp"
#include "dynamics/screw_joint.hpp"
#include "dynamics/shape.hpp"
#include "dynamics/shape_frame.hpp"
#include "dynamics/shape_node.hpp"
#include "dynamics/simple_frame.hpp"
#include "dynamics/skeleton.hpp"
#include "dynamics/translational_joint.hpp"
#include "dynamics/translational_joint2d.hpp"
#include "dynamics/universal_joint.hpp"
#include "dynamics/weld_joint.hpp"
#include "dynamics/zero_dof_joint.hpp"

namespace dart::python_nb {

void defDynamicsModule(nanobind::module_& m)
{
  defEntity(m);
  defJoint(m);
  defZeroDofJoint(m);
  defDegreeOfFreedom(m);
  defFreeJoint(m);
  defRevoluteJoint(m);
  defTranslationalJoint2D(m);
  defPrismaticJoint(m);
  defPlanarJoint(m);
  defTranslationalJoint(m);
  defEulerJoint(m);
  defScrewJoint(m);
  defUniversalJoint(m);
  defBallJoint(m);
  defWeldJoint(m);
  defShape(m);
  defFrame(m);
  defJacobianNode(m);
  defEndEffector(m);
  defBodyNode(m);
  defMetaSkeleton(m);
  defLinkage(m);
  defChain(m);
  defShapeFrame(m);
  defShapeNode(m);
  defSimpleFrame(m);
  defInverseKinematics(m);
  defHierarchicalIK(m);
  defSkeleton(m);
  defInertia(m);
}

} // namespace dart::python_nb
