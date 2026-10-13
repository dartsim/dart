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

#include "demo_models/WorldHistory.hpp"

#include <memory>

namespace dart::python {
namespace {

struct DemoWorldHistory
{
  nb::object worldWrapper;
  std::unique_ptr<dart_demos::WorldHistory> native;

  DemoWorldHistory(nb::object world, std::size_t maxFrames)
    : worldWrapper(std::move(world))
  {
    if (worldWrapper.is_none())
      throw nb::value_error("world must not be None");
    if (!nb::isinstance<dart::simulation::World>(worldWrapper))
      throw nb::type_error("world must be a World");
    native = std::make_unique<dart_demos::WorldHistory>(
        nb::cast<dart::simulation::WorldPtr>(worldWrapper), maxFrames);
  }

  dart_demos::WorldHistory& get() const
  {
    if (!native)
      throw nb::value_error("history has been cleared");
    return *native;
  }

  void clear()
  {
    native.reset();
    worldWrapper.reset();
  }
};

int historyTraverse(PyObject* self, visitproc visit, void* arg) noexcept
{
  Py_VISIT(Py_TYPE(self));
  if (!nb::inst_ready(self))
    return 0;
  try {
    auto* owner = nb::inst_ptr<DemoWorldHistory>(self);
    Py_VISIT(owner->worldWrapper.ptr());
    dartnb::GcEdges edges;
    if (owner->native)
      edges.add(owner->native->getWorld(), [] {});
    return edges.traverse(visit, arg);
  } catch (...) {
    PyErr_SetString(PyExc_RuntimeError, "world history GC traversal failed");
    return -1;
  }
}

int historyClear(PyObject* self) noexcept
{
  if (nb::inst_ready(self))
    nb::inst_ptr<DemoWorldHistory>(self)->clear();
  return 0;
}

} // namespace
} // namespace dart::python

namespace dartnb {
template <>
const PyType_Slot* gc_slots<dart::python::DemoWorldHistory>()
{
  static const PyType_Slot slots[] = {
      {Py_tp_traverse, reinterpret_cast<void*>(&dart::python::historyTraverse)},
      {Py_tp_clear, reinterpret_cast<void*>(&dart::python::historyClear)},
      {0, nullptr}};
  return slots;
}
} // namespace dartnb

namespace dart::python {

void bind_world_history(nb::module_& demos)
{
  dartnb::dart_class<DemoWorldHistory>(
      demos, "WorldHistory", "Bounded native world-state playback history.")
      .def(
          nb::init<nb::object, std::size_t>(),
          nb::arg("world"),
          nb::kw_only(),
          nb::arg("max_frames") = dart_demos::WorldHistory::kMaxHistoryFrames,
          "Record frame zero and retain the World for subsequent playback.")
      .def_prop_ro(
          "world",
          [](const DemoWorldHistory& self) { return self.worldWrapper; })
      .def_prop_ro(
          "current_index",
          [](const DemoWorldHistory& self) {
            return self.get().getCurrentIndex();
          })
      .def_prop_ro(
          "size",
          [](const DemoWorldHistory& self) { return self.get().getSize(); })
      .def_prop_ro(
          "frames_recorded",
          [](const DemoWorldHistory& self) {
            return self.get().getFramesRecorded();
          })
      .def_prop_ro(
          "max_frames",
          [](const DemoWorldHistory& self) {
            return self.get().getMaxFrames();
          })
      .def("record", [](DemoWorldHistory& self) { self.get().record(); })
      .def(
          "move_to",
          [](DemoWorldHistory& self, std::size_t index) {
            self.get().moveTo(index);
          },
          nb::arg("index"))
      .def(
          "move_forward",
          [](DemoWorldHistory& self, std::size_t delta) {
            self.get().moveForward(delta);
          },
          nb::arg("delta") = 1)
      .def(
          "move_backward",
          [](DemoWorldHistory& self, std::size_t delta) {
            self.get().moveBackward(delta);
          },
          nb::arg("delta") = 1);
}

} // namespace dart::python
