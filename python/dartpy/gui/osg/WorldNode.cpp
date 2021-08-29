/*
 * Copyright (c) 2011-2021, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>

PYBIND11_DECLARE_HOLDER_TYPE(T, ::osg::ref_ptr<T>, true);

namespace py = pybind11;

namespace dart {
namespace python {

class PyWorldNode : public dart::gui::osg::WorldNode
{
public:
  // Inherit the constructors
  using WorldNode::WorldNode;

  // Trampoline for virtual function
  void refresh() override
  {
    PYBIND11_OVERLOAD(
        void,      // Return type
        WorldNode, // Parent class
        refresh,   // Name of function in C++ (must match Python name)
    );
  }

  // Trampoline for virtual function
  void customPreRefresh() override
  {
    PYBIND11_OVERLOAD(
        void,             // Return type
        WorldNode,        // Parent class
        customPreRefresh, // Name of function in C++ (must match Python name)
    );
  }

  // Trampoline for virtual function
  void customPostRefresh() override
  {
    PYBIND11_OVERLOAD(
        void,              // Return type
        WorldNode,         // Parent class
        customPostRefresh, // Name of function in C++ (must match Python name)
    );
  }

  // Trampoline for virtual function
  void customPreStep() override
  {
    PYBIND11_OVERLOAD(
        void,          // Return type
        WorldNode,     // Parent class
        customPreStep, // Name of function in C++ (must match Python name)
    );
  }

  // Trampoline for virtual function
  void customPostStep() override
  {
    PYBIND11_OVERLOAD(
        void,           // Return type
        WorldNode,      // Parent class
        customPostStep, // Name of function in C++ (must match Python name)
    );
  }
};

void WorldNode(py::module& m)
{
  ::py::class_<
      dart::gui::osg::WorldNode,
      PyWorldNode,
      ::osg::ref_ptr<dart::gui::osg::WorldNode>>(m, "WorldNode")
      .def(::py::init<>())
      .def(
          ::py::init<std::shared_ptr<dart::simulation::World>>(),
          ::py::arg("world"))
      .def(
          ::py::init<
              std::shared_ptr<dart::simulation::World>,
              osg::ref_ptr<osgShadow::ShadowTechnique>>(),
          ::py::arg("world"),
          ::py::arg("shadowTechnique"))
      .def(
          "setWorld",
          +[](dart::gui::osg::WorldNode* self,
              std::shared_ptr<dart::simulation::World> newWorld) {
            self->setWorld(newWorld);
          },
          ::py::arg("newWorld"))
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
          ::py::arg("on"))
      .def(
          "setNumStepsPerCycle",
          +[](dart::gui::osg::WorldNode* self, std::size_t steps) {
            self->setNumStepsPerCycle(steps);
          },
          ::py::arg("steps"))
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
          ::py::arg("shadowTechnique"))
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
          ::py::arg("viewer"));
}

} // namespace python
} // namespace dart
