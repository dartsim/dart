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

#include "demo_models/BipedStandController.hpp"
#include "detail/eigen.hpp"

#include <stdexcept>

#include <cmath>

namespace dart::python {
namespace biped_stand_binding {

void requireTimestep(double timestep)
{
  if (!std::isfinite(timestep) || timestep <= 0)
    throw std::invalid_argument("timestep must be finite and positive");
}

struct Controller
{
  nb::object skeletonWrapper;
  dart::dynamics::SkeletonPtr skeleton;
  dart_demos::BipedStandController native;

  Controller(nb::object skeletonObject, double timestep)
    : skeletonWrapper(std::move(skeletonObject))
  {
    if (skeletonWrapper.is_none())
      throw nb::value_error("skeleton must not be None");
    if (!nb::isinstance<dart::dynamics::Skeleton>(skeletonWrapper))
      throw nb::type_error("skeleton must be a Skeleton");
    requireTimestep(timestep);
    skeleton = nb::cast<dart::dynamics::SkeletonPtr>(skeletonWrapper);
    native = dart_demos::makeBipedStandController(skeleton, timestep);
  }

  Eigen::VectorXd computeTorques()
  {
    if (!skeleton)
      throw nb::value_error("controller has been cleared");
    if (skeleton->getNumDofs()
            != static_cast<std::size_t>(native.desiredDofs.size())
        || skeleton->getBodyNode("h_heel_left") != native.leftHeel)
      throw nb::value_error("controlled skeleton structure has changed");
    return dart_demos::computeTorques(skeleton, native);
  }

  void clear()
  {
    native.leftHeel = nullptr;
    skeleton.reset();
    skeletonWrapper.reset();
  }
};

int traverse(PyObject* self, visitproc visit, void* arg) noexcept
{
  Py_VISIT(Py_TYPE(self));
  if (nb::inst_ready(self))
    Py_VISIT(nb::inst_ptr<Controller>(self)->skeletonWrapper.ptr());
  return 0;
}

int clear(PyObject* self) noexcept
{
  if (nb::inst_ready(self))
    nb::inst_ptr<Controller>(self)->clear();
  return 0;
}

} // namespace biped_stand_binding
} // namespace dart::python

namespace dartnb {
template <>
const PyType_Slot* gc_slots<dart::python::biped_stand_binding::Controller>()
{
  static const PyType_Slot slots[] = {
      {Py_tp_traverse,
       reinterpret_cast<void*>(&dart::python::biped_stand_binding::traverse)},
      {Py_tp_clear,
       reinterpret_cast<void*>(&dart::python::biped_stand_binding::clear)},
      {0, nullptr}};
  return slots;
}
} // namespace dartnb

namespace dart::python {

void bind_biped_stand(nb::module_& demos)
{
  using Controller = biped_stand_binding::Controller;
  dartnb::dart_class<Controller>(
      demos,
      "BipedStandController",
      "Private native SPD and ankle-strategy controller retaining its "
      "skeleton.")
      .def(
          nb::init<nb::object, double>(),
          nb::arg("skeleton").none(),
          nb::arg("timestep"))
      .def_prop_ro(
          "skeleton",
          [](const Controller& self) { return self.skeletonWrapper; })
      .def_prop_rw(
          "timestep",
          [](const Controller& self) { return self.native.timestep; },
          [](Controller& self, double timestep) {
            biped_stand_binding::requireTimestep(timestep);
            self.native.timestep = timestep;
          })
      .def("compute_torques", &Controller::computeTorques);
}

} // namespace dart::python
