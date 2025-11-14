#include "dynamics/module.hpp"

#include "dynamics/ball_joint.hpp"
#include "dynamics/body_node.hpp"
#include "dynamics/chain.hpp"
#include "dynamics/entity.hpp"
#include "dynamics/free_joint.hpp"
#include "dynamics/frame.hpp"
#include "dynamics/inertia.hpp"
#include "dynamics/inverse_kinematics.hpp"
#include "dynamics/joint.hpp"
#include "dynamics/meta_skeleton.hpp"
#include "dynamics/prismatic_joint.hpp"
#include "dynamics/revolute_joint.hpp"
#include "dynamics/shape.hpp"
#include "dynamics/shape_frame.hpp"
#include "dynamics/shape_node.hpp"
#include "dynamics/simple_frame.hpp"
#include "dynamics/skeleton.hpp"

namespace dart::python_nb {

void defDynamicsModule(nanobind::module_& m)
{
  defEntity(m);
  defJoint(m);
  defFreeJoint(m);
  defRevoluteJoint(m);
  defPrismaticJoint(m);
  defBallJoint(m);
  defShape(m);
  defFrame(m);
  defBodyNode(m);
  defMetaSkeleton(m);
  defShapeFrame(m);
  defShapeNode(m);
  defSimpleFrame(m);
  defInverseKinematics(m);
  defChain(m);
  defSkeleton(m);
  defInertia(m);
}

} // namespace dart::python_nb
