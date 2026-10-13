// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

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

#include "demo_models/OperationalSpaceController.hpp"
#include "detail/eigen.hpp"

#include <dart/external/imgui/imgui.h>

namespace dart::python {
namespace {

struct DemoOscController
{
  nb::object robotWrapper;
  nb::object targetWrapper;
  std::unique_ptr<dart_demos::OperationalSpaceController> native;

  DemoOscController(nb::object robot, nb::object target)
    : robotWrapper(std::move(robot)), targetWrapper(std::move(target))
  {
    if (!nb::isinstance<dart::dynamics::Skeleton>(robotWrapper))
      throw nb::type_error("robot must be a Skeleton");
    if (!nb::isinstance<dart::dynamics::SimpleFrame>(targetWrapper))
      throw nb::type_error("target must be a SimpleFrame");
    native = std::make_unique<dart_demos::OperationalSpaceController>(
        nb::cast<dart::dynamics::SkeletonPtr>(robotWrapper),
        nb::cast<dart::dynamics::SimpleFramePtr>(targetWrapper));
  }

  dart_demos::OperationalSpaceController& get() const
  {
    if (!native)
      throw nb::value_error("controller has been cleared");
    return *native;
  }

  void clear()
  {
    native.reset();
    robotWrapper.reset();
    targetWrapper.reset();
  }
};

int oscTraverse(PyObject* self, visitproc visit, void* arg) noexcept
{
  Py_VISIT(Py_TYPE(self));
  if (!nb::inst_ready(self))
    return 0;
  try {
    auto* owner = nb::inst_ptr<DemoOscController>(self);
    Py_VISIT(owner->robotWrapper.ptr());
    Py_VISIT(owner->targetWrapper.ptr());
    if (owner->native) {
      dartnb::GcEdges edges;
      edges.add(owner->native->getTarget(), [] {});
      return edges.traverse(visit, arg);
    }
    return 0;
  } catch (...) {
    PyErr_SetString(
        PyExc_RuntimeError, "operational-space controller GC traversal failed");
    return -1;
  }
}

int oscClear(PyObject* self) noexcept
{
  if (nb::inst_ready(self))
    nb::inst_ptr<DemoOscController>(self)->clear();
  return 0;
}

} // namespace
} // namespace dart::python

namespace dartnb {
template <>
const PyType_Slot* gc_slots<dart::python::DemoOscController>()
{
  static const PyType_Slot slots[]
      = {{Py_tp_traverse, reinterpret_cast<void*>(&dart::python::oscTraverse)},
         {Py_tp_clear, reinterpret_cast<void*>(&dart::python::oscClear)},
         {0, nullptr}};
  return slots;
}
} // namespace dartnb

namespace dart::python {

void bind_operational_space_control(nb::module_& demos)
{
  demos.def(
      "keyboard_capture_active",
      [] {
        return ImGui::GetCurrentContext() && ImGui::GetIO().WantCaptureKeyboard;
      },
      "Whether scene input should yield to the native panel's keyboard "
      "capture.");
  dartnb::dart_class<DemoOscController>(
      demos,
      "OperationalSpaceController",
      "The native KR5 task-space controller.")
      .def(
          nb::init<nb::object, nb::object>(),
          nb::arg("robot"),
          nb::arg("target"))
      .def_prop_ro(
          "robot",
          [](const DemoOscController& self) {
            self.get();
            return self.robotWrapper;
          })
      .def_prop_rw(
          "target",
          [](const DemoOscController& self) {
            self.get();
            return self.targetWrapper;
          },
          [](DemoOscController& self, nb::object target) {
            if (!nb::isinstance<dart::dynamics::SimpleFrame>(target))
              throw nb::type_error("target must be a SimpleFrame");
            self.get().setTarget(
                nb::cast<dart::dynamics::SimpleFramePtr>(target));
            self.targetWrapper = std::move(target);
          })
      .def_prop_rw(
          "offset",
          [](const DemoOscController& self) -> Eigen::Vector3d {
            return self.get().getOffset();
          },
          [](DemoOscController& self, const Eigen::Vector3d& offset) {
            self.get().setOffset(offset);
          })
      .def_prop_rw(
          "kp",
          [](const DemoOscController& self) -> Eigen::Matrix3d {
            return self.get().getKp();
          },
          [](DemoOscController& self, const Eigen::Matrix3d& kp) {
            self.get().setKp(kp);
          })
      .def_prop_rw(
          "kd",
          [](const DemoOscController& self) -> Eigen::MatrixXd {
            return self.get().getKd();
          },
          [](DemoOscController& self, const Eigen::MatrixXd& kd) {
            self.get().setKd(kd);
          })
      .def(
          "compute_forces",
          [](const DemoOscController& self) {
            return self.get().computeForces();
          })
      .def("update", [](DemoOscController& self) { self.get().update(); });
}

} // namespace dart::python
