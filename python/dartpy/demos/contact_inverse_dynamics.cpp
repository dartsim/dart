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

#include "demo_models/ContactInverseDynamicsModel.hpp"
#include "detail/eigen.hpp"

#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <memory>

namespace dart::python {
namespace {

struct DemoContactInverseDynamicsModel
{
  nb::object worldWrapper;
  std::unique_ptr<dart_demos::ContactInverseDynamicsModel> native;

  DemoContactInverseDynamicsModel(nb::object world)
    : worldWrapper(std::move(world))
  {
    if (worldWrapper.is_none())
      throw nb::value_error("world must not be None");
    if (!nb::isinstance<dart::simulation::World>(worldWrapper))
      throw nb::type_error("world must be a World");
    native = std::make_unique<dart_demos::ContactInverseDynamicsModel>(
        nb::cast<dart::simulation::WorldPtr>(worldWrapper));
  }

  dart_demos::ContactInverseDynamicsModel& get() const
  {
    if (!native)
      throw nb::value_error("model has been cleared");
    return *native;
  }

  void clear()
  {
    native.reset();
    worldWrapper.reset();
  }
};

int contactInverseDynamicsModelTraverse(
    PyObject* self, visitproc visit, void* arg) noexcept
{
  Py_VISIT(Py_TYPE(self));
  if (!nb::inst_ready(self))
    return 0;
  try {
    auto* owner = nb::inst_ptr<DemoContactInverseDynamicsModel>(self);
    Py_VISIT(owner->worldWrapper.ptr());
    dartnb::GcEdges edges;
    if (owner->native)
      edges.add(owner->native->world, [] {});
    return edges.traverse(visit, arg);
  } catch (...) {
    PyErr_SetString(
        PyExc_RuntimeError, "contact inverse dynamics GC traversal failed");
    return -1;
  }
}

int contactInverseDynamicsModelClear(PyObject* self) noexcept
{
  if (nb::inst_ready(self))
    nb::inst_ptr<DemoContactInverseDynamicsModel>(self)->clear();
  return 0;
}

} // namespace
} // namespace dart::python

namespace dartnb {
template <>
const PyType_Slot* gc_slots<dart::python::DemoContactInverseDynamicsModel>()
{
  static const PyType_Slot slots[]
      = {{Py_tp_traverse,
          reinterpret_cast<void*>(
              &dart::python::contactInverseDynamicsModelTraverse)},
         {Py_tp_clear,
          reinterpret_cast<void*>(
              &dart::python::contactInverseDynamicsModelClear)},
         {0, nullptr}};
  return slots;
}
} // namespace dartnb

namespace dart::python {

void bind_contact_inverse_dynamics(nb::module_& demos)
{
  using Owner = DemoContactInverseDynamicsModel;
  dartnb::dart_class<Owner>(
      demos,
      "ContactInverseDynamicsModel",
      "Native squat keyframes, contact inverse dynamics, arrows, and "
      "histories.")
      .def(nb::init<nb::object>(), nb::arg("world").none())
      .def_prop_ro("world", [](const Owner& self) { return self.worldWrapper; })
      .def_prop_ro("biped", [](const Owner& self) { return self.get().biped; })
      .def_prop_ro("time", [](const Owner& self) { return self.get().time; })
      .def_prop_ro(
          "keyframe_count",
          [](const Owner& self) { return self.get().keyframes.size(); })
      .def_prop_ro(
          "regeneration_requested",
          [](const Owner& self) { return self.get().regenerateRequested; })
      .def_prop_rw(
          "playing",
          [](const Owner& self) { return self.get().playing; },
          [](Owner& self, bool value) { self.get().playing = value; })
      .def_prop_rw(
          "playback_speed",
          [](const Owner& self) { return self.get().playbackSpeed; },
          [](Owner& self, float value) { self.get().setPlaybackSpeed(value); })
      .def_prop_rw(
          "squat_depth",
          [](const Owner& self) { return self.get().squatDepth; },
          [](Owner& self, float value) { self.get().setSquatDepth(value); })
      .def_prop_rw(
          "friction_coeff",
          [](const Owner& self) { return self.get().frictionCoeff; },
          [](Owner& self, float value) { self.get().setFrictionCoeff(value); })
      .def_prop_rw(
          "force_scale",
          [](const Owner& self) { return self.get().forceScale; },
          [](Owner& self, float value) { self.get().setForceScale(value); })
      .def_prop_ro(
          "feasible", [](const Owner& self) { return self.get().feasible; })
      .def_prop_ro(
          "residual_norm",
          [](const Owner& self) { return self.get().residualNorm; })
      .def_prop_ro(
          "contact_forces",
          [](const Owner& self) { return self.get().contactForces; })
      .def_prop_ro(
          "total_vertical_force",
          [](const Owner& self) { return self.get().totalVerticalForce; })
      .def_prop_ro(
          "torque_names",
          [](const Owner& self) { return self.get().torqueNames; })
      .def_prop_ro(
          "torque_plots",
          [](const Owner& self) { return self.get().torquePlots; })
      .def_prop_ro(
          "plot_samples",
          [](const Owner& self) { return self.get().plotSamples; })
      .def_prop_ro(
          "sample_index",
          [](const Owner& self) { return self.get().sampleIndex; })
      .def(
          "request_regeneration",
          [](Owner& self) { self.get().requestRegeneration(); })
      .def("update_frame", [](Owner& self) { self.get().updateFrame(); });
}

} // namespace dart::python
