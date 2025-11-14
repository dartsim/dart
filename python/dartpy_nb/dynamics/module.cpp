#include "dynamics/module.hpp"

#include "dynamics/body_node.hpp"
#include "dynamics/entity.hpp"
#include "dynamics/free_joint.hpp"
#include "dynamics/frame.hpp"
#include "dynamics/inertia.hpp"
#include "dynamics/joint.hpp"
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
  defShape(m);
  defFrame(m);
  defBodyNode(m);
  defShapeFrame(m);
  defShapeNode(m);
  defSimpleFrame(m);
  defSkeleton(m);
  defInertia(m);
}

} // namespace dart::python_nb
