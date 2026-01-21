#include "gui/gui.hpp"
#include "gui/utils.hpp"

#include <dart/gui/real_time_world_node.hpp>
#include <dart/gui/viewer.hpp>
#include <dart/gui/world_node.hpp>

#include <dart/simulation/world.hpp>

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/trampoline.h>

#include <memory>

namespace nb = nanobind;

namespace dart::python_nb {

namespace {

using WorldPtr = std::shared_ptr<dart::simulation::World>;
using ShadowTechniquePtr = osgShadow::ShadowTechnique*;

class PyWorldNode : public dart::gui::WorldNode
{
public:
  NB_TRAMPOLINE(dart::gui::WorldNode, 5);
  void refresh() override
  {
    NB_OVERRIDE(refresh);
  }
  void customPreRefresh() override
  {
    NB_OVERRIDE(customPreRefresh);
  }
  void customPostRefresh() override
  {
    NB_OVERRIDE(customPostRefresh);
  }
  void customPreStep() override
  {
    NB_OVERRIDE(customPreStep);
  }
  void customPostStep() override
  {
    NB_OVERRIDE(customPostStep);
  }
};

class PyRealTimeWorldNode : public dart::gui::RealTimeWorldNode
{
public:
  NB_TRAMPOLINE(dart::gui::RealTimeWorldNode, 5);
  void refresh() override
  {
    NB_OVERRIDE(refresh);
  }
  void customPreRefresh() override
  {
    NB_OVERRIDE(customPreRefresh);
  }
  void customPostRefresh() override
  {
    NB_OVERRIDE(customPostRefresh);
  }
  void customPreStep() override
  {
    NB_OVERRIDE(customPreStep);
  }
  void customPostStep() override
  {
    NB_OVERRIDE(customPostStep);
  }
};

} // namespace

::osg::ref_ptr<osgShadow::ShadowTechnique> toShadowRef(
    ShadowTechniquePtr shadow)
{
  if (!shadow)
    return {};
  return ::osg::ref_ptr<osgShadow::ShadowTechnique>(shadow);
}
void defWorldNode(nb::module_& m)
{
  using dart::gui::WorldNode;

  nb::class_<WorldNode, PyWorldNode>(m, "WorldNode", nb::dynamic_attr())
      .def(nb::new_([]() { return makeOsgShared<WorldNode>(); }))
      .def(
          nb::new_([](const WorldPtr& world, ShadowTechniquePtr shadow) {
            return makeOsgShared<WorldNode>(world, toShadowRef(shadow));
          }),
          nb::arg("world"),
          nb::arg("shadow_technique") = nullptr)
      .def("setWorld", &WorldNode::setWorld, nb::arg("new_world"))
      .def("getWorld", &WorldNode::getWorld)
      .def("refresh", &WorldNode::refresh)
      .def("customPreRefresh", &WorldNode::customPreRefresh)
      .def("customPostRefresh", &WorldNode::customPostRefresh)
      .def("customPreStep", &WorldNode::customPreStep)
      .def("customPostStep", &WorldNode::customPostStep)
      .def("isSimulating", &WorldNode::isSimulating)
      .def("simulate", &WorldNode::simulate, nb::arg("on"))
      .def(
          "setNumStepsPerCycle",
          &WorldNode::setNumStepsPerCycle,
          nb::arg("steps"))
      .def("getNumStepsPerCycle", &WorldNode::getNumStepsPerCycle)
      .def("isShadowed", &WorldNode::isShadowed)
      .def(
          "setShadowTechnique",
          [](WorldNode& self, const ShadowTechniquePtr& shadow) {
            self.setShadowTechnique(toShadowRef(shadow));
          },
          nb::arg("shadow_technique") = nullptr)
      .def(
          "getShadowTechnique",
          [](const WorldNode& self) {
            return osgRefToShared(self.getShadowTechnique());
          })
      .def_static(
          "createDefaultShadowTechnique",
          [](const dart::gui::Viewer* viewer) {
            return osgRefToShared(
                dart::gui::WorldNode::createDefaultShadowTechnique(viewer));
          },
          nb::arg("viewer"));
}

void defRealTimeWorldNode(nb::module_& m)
{
  using dart::gui::RealTimeWorldNode;

  nb::class_<RealTimeWorldNode, dart::gui::WorldNode, PyRealTimeWorldNode>(
      m, "RealTimeWorldNode", nb::dynamic_attr())
      .def(nb::new_([]() { return makeOsgShared<RealTimeWorldNode>(); }))
      .def(
          nb::new_([](const WorldPtr& world,
                      ShadowTechniquePtr shadower,
                      double targetFrequency,
                      double targetRealTimeFactor) {
            return makeOsgShared<RealTimeWorldNode>(
                world,
                toShadowRef(shadower),
                targetFrequency,
                targetRealTimeFactor);
          }),
          nb::arg("world"),
          nb::arg("shadower") = nullptr,
          nb::arg("target_frequency") = 60.0,
          nb::arg("target_real_time_factor") = 1.0)
      .def(
          "setTargetFrequency",
          &RealTimeWorldNode::setTargetFrequency,
          nb::arg("target_frequency"))
      .def("getTargetFrequency", &RealTimeWorldNode::getTargetFrequency)
      .def(
          "setTargetRealTimeFactor",
          &RealTimeWorldNode::setTargetRealTimeFactor,
          nb::arg("target_rtf"))
      .def(
          "getTargetRealTimeFactor",
          &RealTimeWorldNode::getTargetRealTimeFactor)
      .def("getLastRealTimeFactor", &RealTimeWorldNode::getLastRealTimeFactor)
      .def(
          "getLowestRealTimeFactor",
          &RealTimeWorldNode::getLowestRealTimeFactor)
      .def(
          "getHighestRealTimeFactor",
          &RealTimeWorldNode::getHighestRealTimeFactor)
      .def("refresh", &RealTimeWorldNode::refresh);
}

} // namespace dart::python_nb
