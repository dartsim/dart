#include "simulation_experimental/module.hpp"

#include "simulation_experimental/frame.hpp"
#include "simulation_experimental/joint_type.hpp"
#include "simulation_experimental/multi_body.hpp"
#include "simulation_experimental/rigid_body.hpp"
#include "simulation_experimental/shape_node.hpp"
#include "simulation_experimental/state_space.hpp"
#include "simulation_experimental/world.hpp"

namespace dart::python_nb {

void defSimulationExperimentalModule(nanobind::module_& m)
{
  defJointType(m);
  defExpFrame(m);
  defExpJoint(m);
  defExpShapeNode(m);
  defExpLink(m);
  defExpMultiBody(m);
  defRigidBody(m);
  defStateSpace(m);
  defExperimentalWorld(m);
}

} // namespace dart::python_nb
