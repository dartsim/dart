/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include <dart/gui/osg/all.hpp>

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>

PYBIND11_DECLARE_HOLDER_TYPE(T, ::osg::ref_ptr<T>, true);

namespace py = pybind11;

namespace dart {
namespace python {

// Trampoline class for GUIEventHandler to allow Python subclassing
class PyGUIEventHandler : public osgGA::GUIEventHandler
{
public:
  using osgGA::GUIEventHandler::GUIEventHandler;

  bool handle(
      const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) override
  {
    // Use const reference to prevent copying non-copyable OSG objects
    PYBIND11_OVERRIDE(
        bool, osgGA::GUIEventHandler, handle, std::cref(ea), std::ref(aa));
  }
};

void GUIEventHandler(py::module& m)
{
  // Bind osg::Object base class first
  py::class_<osg::Object, ::osg::ref_ptr<osg::Object>>(m, "Object");

  // Bind GUIEventAdapter for event info with proper reference handling
  py::class_<
      osgGA::GUIEventAdapter,
      osg::Object,
      ::osg::ref_ptr<osgGA::GUIEventAdapter>>(m, "GUIEventAdapter")
      .def("getEventType", &osgGA::GUIEventAdapter::getEventType)
      .def("getKey", &osgGA::GUIEventAdapter::getKey)
      .def("getModKeyMask", &osgGA::GUIEventAdapter::getModKeyMask)
      .def("getX", &osgGA::GUIEventAdapter::getX)
      .def("getY", &osgGA::GUIEventAdapter::getY)
      .def("getButton", &osgGA::GUIEventAdapter::getButton)
      .def("getButtonMask", &osgGA::GUIEventAdapter::getButtonMask);

  // Bind EventType enum
  py::enum_<osgGA::GUIEventAdapter::EventType>(m, "EventType")
      .value("NONE", osgGA::GUIEventAdapter::NONE)
      .value("PUSH", osgGA::GUIEventAdapter::PUSH)
      .value("RELEASE", osgGA::GUIEventAdapter::RELEASE)
      .value("DOUBLECLICK", osgGA::GUIEventAdapter::DOUBLECLICK)
      .value("DRAG", osgGA::GUIEventAdapter::DRAG)
      .value("MOVE", osgGA::GUIEventAdapter::MOVE)
      .value("KEYDOWN", osgGA::GUIEventAdapter::KEYDOWN)
      .value("KEYUP", osgGA::GUIEventAdapter::KEYUP)
      .value("SCROLL", osgGA::GUIEventAdapter::SCROLL)
      .value("FRAME", osgGA::GUIEventAdapter::FRAME);

  // Bind GUIActionAdapter (minimal binding - mostly opaque, abstract class)
  py::class_<osgGA::GUIActionAdapter>(m, "GUIActionAdapter");

  // Bind GUIEventHandler with trampoline
  // Note: The trampoline class PyGUIEventHandler allows Python to override the
  // handle() method. The handle() method is automatically available through
  // the trampoline and doesn't need to be explicitly bound.
  py::class_<
      osgGA::GUIEventHandler,
      PyGUIEventHandler,
      osg::Object,
      ::osg::ref_ptr<osgGA::GUIEventHandler>>(m, "GUIEventHandler")
      .def(py::init<>());
}

} // namespace python
} // namespace dart
