/*
 * Copyright (c) 2011-2019, The DART development contributors
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
#include <pybind11/pybind11.h>

PYBIND11_DECLARE_HOLDER_TYPE(T, ::osg::ref_ptr<T>, true);

namespace py = pybind11;

namespace dart {
namespace python {

class PyPyRealTimeWorldNodeNode : public dart::gui::osg::RealTimeWorldNode
{
public:
  // Inherit the constructors
  using RealTimeWorldNode::RealTimeWorldNode;

  // Trampoline for virtual function
  void refresh() override
  {
    PYBIND11_OVERLOAD(
        void,              // Return type
        RealTimeWorldNode, // Parent class
        refresh,           // Name of function in C++ (must match Python name)
    );
  }

  // Trampoline for virtual function
  void customPreRefresh() override
  {
    PYBIND11_OVERLOAD(
        void,              // Return type
        RealTimeWorldNode, // Parent class
        customPreRefresh,  // Name of function in C++ (must match Python name)
    );
  }

  // Trampoline for virtual function
  void customPostRefresh() override
  {
    PYBIND11_OVERLOAD(
        void,              // Return type
        RealTimeWorldNode, // Parent class
        customPostRefresh, // Name of function in C++ (must match Python name)
    );
  }

  // Trampoline for virtual function
  void customPreStep() override
  {
    PYBIND11_OVERLOAD(
        void,              // Return type
        RealTimeWorldNode, // Parent class
        customPreStep,     // Name of function in C++ (must match Python name)
    );
  }

  // Trampoline for virtual function
  void customPostStep() override
  {
    PYBIND11_OVERLOAD(
        void,              // Return type
        RealTimeWorldNode, // Parent class
        customPostStep,    // Name of function in C++ (must match Python name)
    );
  }
};

void RealTimeWorldNode(pybind11::module& m)
{
  ::pybind11::class_<
      dart::gui::osg::RealTimeWorldNode,
      dart::gui::osg::WorldNode,
      PyPyRealTimeWorldNodeNode,
      ::osg::ref_ptr<dart::gui::osg::RealTimeWorldNode>>(m, "RealTimeWorldNode")
      .def(::pybind11::init<>())
      .def(
          ::pybind11::init<const std::shared_ptr<dart::simulation::World>&>(),
          ::pybind11::arg("world"))
      .def(
          ::pybind11::init<
              const std::shared_ptr<dart::simulation::World>&,
              const osg::ref_ptr<osgShadow::ShadowTechnique>&>(),
          ::pybind11::arg("world"),
          ::pybind11::arg("shadower"))
      .def(
          ::pybind11::init<
              const std::shared_ptr<dart::simulation::World>&,
              const osg::ref_ptr<osgShadow::ShadowTechnique>&,
              double>(),
          ::pybind11::arg("world"),
          ::pybind11::arg("shadower"),
          ::pybind11::arg("targetFrequency"))
      .def(
          ::pybind11::init<
              const std::shared_ptr<dart::simulation::World>&,
              const osg::ref_ptr<osgShadow::ShadowTechnique>&,
              double,
              double>(),
          ::pybind11::arg("world"),
          ::pybind11::arg("shadower"),
          ::pybind11::arg("targetFrequency"),
          ::pybind11::arg("targetRealTimeFactor"))
      .def(
          "setTargetFrequency",
          +[](dart::gui::osg::RealTimeWorldNode* self, double targetFrequency) {
            self->setTargetFrequency(targetFrequency);
          },
          ::pybind11::arg("targetFrequency"))
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
          ::pybind11::arg("targetRTF"))
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
      .def("refresh", +[](dart::gui::osg::RealTimeWorldNode* self) {
        self->refresh();
      });
}

} // namespace python
} // namespace dart
