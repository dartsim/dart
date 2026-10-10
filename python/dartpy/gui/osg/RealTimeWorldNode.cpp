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

#include <dart/gui/osg/RealTimeWorldNode.hpp>
#include <dart/gui/osg/WorldNode.hpp>

#include <dart/simulation/World.hpp>

#include <osgShadow/ShadowTechnique>

#include <memory>

namespace dart {
namespace python {

namespace gui_trampolines {
using WorldNode = dart::gui::osg::WorldNode;
using RealTimeWorldNode = dart::gui::osg::RealTimeWorldNode;
using World = dart::simulation::World;
using Viewer = dart::gui::osg::Viewer;
class PyRealTimeWorldNode : public RealTimeWorldNode
{
public:
  NB_TRAMPOLINE(RealTimeWorldNode);

  PyRealTimeWorldNode(
      const std::shared_ptr<World>& world = nullptr,
      const ::osg::ref_ptr<osgShadow::ShadowTechnique>& shadow = nullptr,
      double frequency = 60.0,
      double factor = 1.0)
    : RealTimeWorldNode(world, shadow, frequency, factor)
  {
    ref();
  }

  ~PyRealTimeWorldNode() override
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

void bindRealTimeWorldNode(nb::module_& m)
{
  dartnb::dart_class<
      dart::gui::osg::RealTimeWorldNode,
      dart::gui::osg::WorldNode,
      gui_trampolines::PyRealTimeWorldNode>(m, "RealTimeWorldNode")
      .def(dartnb::gui::init<>())
      .def(
          dartnb::gui::init<const std::shared_ptr<dart::simulation::World>&>(),
          nb::arg("world").none())
      .def(
          dartnb::gui::init<
              const std::shared_ptr<dart::simulation::World>&,
              const osg::ref_ptr<osgShadow::ShadowTechnique>&>(),
          nb::arg("world").none(),
          nb::arg("shadower"))
      .def(
          dartnb::gui::init<
              const std::shared_ptr<dart::simulation::World>&,
              const osg::ref_ptr<osgShadow::ShadowTechnique>&,
              double>(),
          nb::arg("world").none(),
          nb::arg("shadower"),
          nb::arg("targetFrequency"))
      .def(
          dartnb::gui::init<
              const std::shared_ptr<dart::simulation::World>&,
              const osg::ref_ptr<osgShadow::ShadowTechnique>&,
              double,
              double>(),
          nb::arg("world").none(),
          nb::arg("shadower"),
          nb::arg("targetFrequency"),
          nb::arg("targetRealTimeFactor"))
      .def(
          "setTargetFrequency",
          +[](dart::gui::osg::RealTimeWorldNode* self, double targetFrequency) {
            self->setTargetFrequency(targetFrequency);
          },
          nb::arg("targetFrequency"))
      .def(
          "getTargetFrequency",
          +[](const dart::gui::osg::RealTimeWorldNode* self) -> double {
            return self->getTargetFrequency();
          })
      .def(
          "setTargetRealTimeFactor",
          +[](dart::gui::osg::RealTimeWorldNode* self, double targetRTF) {
            self->setTargetRealTimeFactor(targetRTF);
          },
          nb::arg("targetRTF"))
      .def(
          "getTargetRealTimeFactor",
          +[](const dart::gui::osg::RealTimeWorldNode* self) -> double {
            return self->getTargetRealTimeFactor();
          })
      .def(
          "getLastRealTimeFactor",
          +[](const dart::gui::osg::RealTimeWorldNode* self) -> double {
            return self->getLastRealTimeFactor();
          })
      .def(
          "getLowestRealTimeFactor",
          +[](const dart::gui::osg::RealTimeWorldNode* self) -> double {
            return self->getLowestRealTimeFactor();
          })
      .def(
          "getHighestRealTimeFactor",
          +[](const dart::gui::osg::RealTimeWorldNode* self) -> double {
            return self->getHighestRealTimeFactor();
          })
      .def(
          "refresh",
          +[](dart::gui::osg::RealTimeWorldNode* self) { self->refresh(); });
}

} // namespace python
} // namespace dart
