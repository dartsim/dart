#include "detail/dart_nb.hpp"

#include <dart/gui/osg/RealTimeWorldNode.hpp>

#include <nanobind/trampoline.h>

/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "gui/osg/ownership.hpp"

#include <dart/gui/osg/Viewer.hpp>
#include <dart/gui/osg/WorldNode.hpp>

#include <dart/simulation/World.hpp>

#include <osgShadow/ShadowTechnique>

#include <memory>

#include <cstddef>

namespace dart {
namespace python {

namespace gui_trampolines {
using WorldNode = dart::gui::osg::WorldNode;
using RealTimeWorldNode = dart::gui::osg::RealTimeWorldNode;
using World = dart::simulation::World;
using Viewer = dart::gui::osg::Viewer;
class PyWorldNode : public WorldNode
{
public:
  NB_TRAMPOLINE(WorldNode);

  PyWorldNode(
      std::shared_ptr<World> world = nullptr,
      ::osg::ref_ptr<osgShadow::ShadowTechnique> shadow = nullptr)
    : WorldNode(std::move(world), std::move(shadow))
  {
    ref();
  }

  ~PyWorldNode() override
  {
    unref_nodelete();
  }
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

} // namespace gui_trampolines

void bindWorldNode(nb::module_& m)
{
  dartnb::dart_class<dart::gui::osg::WorldNode, gui_trampolines::PyWorldNode>(
      m, "WorldNode")
      .def(dartnb::gui::init<>())
      .def(
          dartnb::gui::init<std::shared_ptr<dart::simulation::World>>(),
          nb::arg("world").none())
      .def(
          dartnb::gui::init<
              std::shared_ptr<dart::simulation::World>,
              osg::ref_ptr<osgShadow::ShadowTechnique>>(),
          nb::arg("world").none(),
          nb::arg("shadowTechnique"))
      .def(
          "setWorld",
          +[](dart::gui::osg::WorldNode* self,
              std::shared_ptr<dart::simulation::World> newWorld) {
            self->setWorld(newWorld);
          },
          nb::arg("newWorld").none())
      .def(
          "getWorld",
          +[](const dart::gui::osg::WorldNode* self)
              -> std::shared_ptr<dart::simulation::World> {
            return self->getWorld();
          })
      .def(
          "refresh", +[](dart::gui::osg::WorldNode* self) { self->refresh(); })
      .def(
          "customPreRefresh",
          +[](dart::gui::osg::WorldNode* self) { self->customPreRefresh(); })
      .def(
          "customPostRefresh",
          +[](dart::gui::osg::WorldNode* self) { self->customPostRefresh(); })
      .def(
          "customPreStep",
          +[](dart::gui::osg::WorldNode* self) { self->customPreStep(); })
      .def(
          "customPostStep",
          +[](dart::gui::osg::WorldNode* self) { self->customPostStep(); })
      .def(
          "isSimulating",
          +[](const dart::gui::osg::WorldNode* self) -> bool {
            return self->isSimulating();
          })
      .def(
          "simulate",
          +[](dart::gui::osg::WorldNode* self, bool on) { self->simulate(on); },
          nb::arg("on"))
      .def(
          "setNumStepsPerCycle",
          +[](dart::gui::osg::WorldNode* self, std::size_t steps) {
            self->setNumStepsPerCycle(steps);
          },
          nb::arg("steps"))
      .def(
          "getNumStepsPerCycle",
          +[](const dart::gui::osg::WorldNode* self) -> std::size_t {
            return self->getNumStepsPerCycle();
          })
      .def(
          "isShadowed",
          +[](const dart::gui::osg::WorldNode* self) -> bool {
            return self->isShadowed();
          })
      .def(
          "setShadowTechnique",
          +[](dart::gui::osg::WorldNode* self) { self->setShadowTechnique(); })
      .def(
          "setShadowTechnique",
          +[](dart::gui::osg::WorldNode* self,
              osg::ref_ptr<osgShadow::ShadowTechnique> shadowTechnique) {
            self->setShadowTechnique(shadowTechnique);
          },
          nb::arg("shadowTechnique"))
      .def(
          "getShadowTechnique",
          +[](const dart::gui::osg::WorldNode* self)
              -> osg::ref_ptr<osgShadow::ShadowTechnique> {
            return self->getShadowTechnique();
          })
      .def_static(
          "createDefaultShadowTechnique",
          +[](const dart::gui::osg::Viewer* viewer)
              -> osg::ref_ptr<osgShadow::ShadowTechnique> {
            return dart::gui::osg::WorldNode::createDefaultShadowTechnique(
                viewer);
          },
          nb::arg("viewer").none());
}

} // namespace python
} // namespace dart
