#include "gui/osg/osg.hpp"

#include "gui/osg/utils.hpp"

#include <dart/gui/osg/WorldNode.hpp>
#include <dart/gui/osg/RealTimeWorldNode.hpp>
#include <dart/gui/osg/Viewer.hpp>
#include <dart/simulation/World.hpp>

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/trampoline.h>

#include <memory>

namespace nb = nanobind;

namespace dart::python_nb {

namespace {

using WorldPtr = std::shared_ptr<dart::simulation::World>;
using ShadowTechniquePtr = osgShadow::ShadowTechnique*;

class PyWorldNode : public dart::gui::osg::WorldNode
{
public:
  NB_TRAMPOLINE(dart::gui::osg::WorldNode, 1);
  void refresh() override { NB_OVERRIDE(refresh); }
  void customPreRefresh() override { NB_OVERRIDE(customPreRefresh); }
  void customPostRefresh() override { NB_OVERRIDE(customPostRefresh); }
  void customPreStep() override { NB_OVERRIDE(customPreStep); }
  void customPostStep() override { NB_OVERRIDE(customPostStep); }
};

class PyRealTimeWorldNode : public dart::gui::osg::RealTimeWorldNode
{
public:
  NB_TRAMPOLINE(dart::gui::osg::RealTimeWorldNode, 1);
  void refresh() override { NB_OVERRIDE(refresh); }
  void customPreRefresh() override { NB_OVERRIDE(customPreRefresh); }
  void customPostRefresh() override { NB_OVERRIDE(customPostRefresh); }
  void customPreStep() override { NB_OVERRIDE(customPreStep); }
  void customPostStep() override { NB_OVERRIDE(customPostStep); }
};

} // namespace

::osg::ref_ptr<osgShadow::ShadowTechnique> toShadowRef(ShadowTechniquePtr shadow)
{
  if (!shadow)
    return {};
  return ::osg::ref_ptr<osgShadow::ShadowTechnique>(shadow);
}
void defWorldNode(nb::module_& m)
{
  using dart::gui::osg::WorldNode;

  nb::class_<WorldNode, PyWorldNode>(m, "WorldNode", nb::dynamic_attr())
      .def(
          "__init__",
          [](PyWorldNode* self) {
            new (self) PyWorldNode();
          })
      .def(
          "__init__",
          [](PyWorldNode* self,
             dart::simulation::World* world,
             const ::osg::ref_ptr<osgShadow::ShadowTechnique>& shadow) {
            // Create a non-owning shared_ptr from the raw pointer
            WorldPtr wptr = world ? std::shared_ptr<dart::simulation::World>(
                world, [](dart::simulation::World*){}) : nullptr;
            new (self) PyWorldNode(wptr, shadow);
          },
          nb::arg("world") = nullptr,
          nb::arg("shadowTechnique") = ::osg::ref_ptr<osgShadow::ShadowTechnique>())
      .def("setWorld", &WorldNode::setWorld, nb::arg("newWorld"))
      .def("getWorld", &WorldNode::getWorld)
      .def("refresh", &WorldNode::refresh)
      .def("customPreRefresh", &WorldNode::customPreRefresh)
      .def("customPostRefresh", &WorldNode::customPostRefresh)
      .def("customPreStep", &WorldNode::customPreStep)
      .def("customPostStep", &WorldNode::customPostStep)
      .def("isSimulating", &WorldNode::isSimulating)
      .def("simulate", &WorldNode::simulate, nb::arg("on"))
      .def("setNumStepsPerCycle", &WorldNode::setNumStepsPerCycle, nb::arg("steps"))
      .def("getNumStepsPerCycle", &WorldNode::getNumStepsPerCycle)
      .def("isShadowed", &WorldNode::isShadowed)
      .def(
          "setShadowTechnique",
          [](WorldNode& self, const ShadowTechniquePtr& shadow) {
            self.setShadowTechnique(toShadowRef(shadow));
          },
          nb::arg("shadowTechnique") = nb::none())
      .def(
          "getShadowTechnique",
          [](const WorldNode& self) {
            return osgRefToShared(self.getShadowTechnique());
          })
      .def_static(
          "createDefaultShadowTechnique",
          [](const dart::gui::osg::Viewer* viewer) {
            return osgRefToShared(
                dart::gui::osg::WorldNode::createDefaultShadowTechnique(viewer));
          },
          nb::arg("viewer"));
}

void defRealTimeWorldNode(nb::module_& m)
{
  using dart::gui::osg::RealTimeWorldNode;

  nb::class_<RealTimeWorldNode,
      dart::gui::osg::WorldNode,
      PyRealTimeWorldNode>(
      m, "RealTimeWorldNode", nb::dynamic_attr())
      .def(
          "__init__",
          [](PyRealTimeWorldNode* self) {
            new (self) PyRealTimeWorldNode();
          })
      .def(
          "__init__",
          [](PyRealTimeWorldNode* self,
             dart::simulation::World* world,
             const ::osg::ref_ptr<osgShadow::ShadowTechnique>& shadower,
             double targetFrequency,
             double targetRealTimeFactor) {
            // Create a non-owning shared_ptr from the raw pointer
            WorldPtr wptr = world ? std::shared_ptr<dart::simulation::World>(
                world, [](dart::simulation::World*){}) : nullptr;
            new (self) PyRealTimeWorldNode(
                wptr,
                shadower,
                targetFrequency,
                targetRealTimeFactor);
          },
          nb::arg("world") = nullptr,
          nb::arg("shadower") = ::osg::ref_ptr<osgShadow::ShadowTechnique>(),
          nb::arg("targetFrequency") = 60.0,
          nb::arg("targetRealTimeFactor") = 1.0)
      .def("setTargetFrequency", &RealTimeWorldNode::setTargetFrequency, nb::arg("targetFrequency"))
      .def("getTargetFrequency", &RealTimeWorldNode::getTargetFrequency)
      .def("setTargetRealTimeFactor", &RealTimeWorldNode::setTargetRealTimeFactor, nb::arg("targetRTF"))
      .def("getTargetRealTimeFactor", &RealTimeWorldNode::getTargetRealTimeFactor)
      .def("getLastRealTimeFactor", &RealTimeWorldNode::getLastRealTimeFactor)
      .def("getLowestRealTimeFactor", &RealTimeWorldNode::getLowestRealTimeFactor)
      .def("getHighestRealTimeFactor", &RealTimeWorldNode::getHighestRealTimeFactor)
      .def("refresh", &RealTimeWorldNode::refresh);
}

} // namespace dart::python_nb
