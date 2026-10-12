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

#ifdef DARTPY_HAS_SIMBICON_DEMOS

  #include "demo_models/atlas_simbicon/Controller.hpp"
  #include "demo_models/atlas_simbicon/State.hpp"
  #include "demo_models/atlas_simbicon/StateMachine.hpp"

namespace dart::python {
namespace {
using NativeController = dart_demos::atlas_simbicon::Controller;

} // namespace

// Python drops controllers in any order relative to their World. When the last
// owner of a controller goes, remove its pelvis harness from the World's
// solver, or the solver keeps a weld to bodies that may be freed. A World that
// is gone took its solver and harness with it.
std::shared_ptr<dart_demos::atlas_simbicon::Controller> unharnessingOwner(
    std::shared_ptr<dart_demos::atlas_simbicon::Controller> controller,
    const dart::simulation::WorldPtr& world)
{
  auto* raw = controller.get();
  return std::shared_ptr<dart_demos::atlas_simbicon::Controller>(
      raw,
      [owned = std::move(controller),
       weak = std::weak_ptr<dart::simulation::World>(world)](auto*) mutable {
        if (weak.lock())
          owned->unharnessPelvis();
        owned.reset();
      });
}

namespace {

// The native controller borrows its solver; release it before its World.
struct DemoController
{
  nb::object atlasWrapper;
  nb::object worldWrapper;
  dart::simulation::WorldPtr world;
  std::shared_ptr<NativeController> native;

  DemoController(nb::object atlasObject, nb::object worldObject)
    : atlasWrapper(std::move(atlasObject)), worldWrapper(std::move(worldObject))
  {
    if (atlasWrapper.is_none() || worldWrapper.is_none())
      throw nb::value_error("atlas and world must not be None");
    if (!nb::isinstance<dart::dynamics::Skeleton>(atlasWrapper))
      throw nb::type_error("atlas must be a Skeleton");
    if (!nb::isinstance<dart::simulation::World>(worldWrapper))
      throw nb::type_error("world must be a World");
    world = nb::cast<dart::simulation::WorldPtr>(worldWrapper);
    native = unharnessingOwner(
        std::make_shared<NativeController>(
            nb::cast<dart::dynamics::SkeletonPtr>(atlasWrapper),
            world->getConstraintSolver()),
        world);
  }

  DemoController(
      std::shared_ptr<NativeController> controller,
      dart::simulation::WorldPtr owner)
    : atlasWrapper(nb::cast(controller->getAtlasRobot())),
      worldWrapper(nb::cast(owner)),
      world(std::move(owner)),
      native(std::move(controller))
  {
  }

  NativeController& get() const
  {
    if (!native)
      throw nb::value_error("controller has been cleared");
    return *native;
  }

  void clear()
  {
    native.reset();
    world.reset();
    atlasWrapper.reset();
    worldWrapper.reset();
  }
};

int controllerTraverse(PyObject* self, visitproc visit, void* arg) noexcept
{
  Py_VISIT(Py_TYPE(self));
  if (!nb::inst_ready(self))
    return 0;
  try {
    auto* owner = nb::inst_ptr<DemoController>(self);
    Py_VISIT(owner->atlasWrapper.ptr());
    Py_VISIT(owner->worldWrapper.ptr());
    dartnb::GcEdges edges;
    edges.add(owner->world, [] {});
    return edges.traverse(visit, arg);
  } catch (...) {
    PyErr_SetString(PyExc_RuntimeError, "demo controller GC traversal failed");
    return -1;
  }
}

int controllerClear(PyObject* self) noexcept
{
  if (nb::inst_ready(self))
    nb::inst_ptr<DemoController>(self)->clear();
  return 0;
}
} // namespace
} // namespace dart::python

namespace dartnb {
template <>
const PyType_Slot* gc_slots<dart::python::DemoController>()
{
  static const PyType_Slot slots[]
      = {{Py_tp_traverse,
          reinterpret_cast<void*>(&dart::python::controllerTraverse)},
         {Py_tp_clear, reinterpret_cast<void*>(&dart::python::controllerClear)},
         {0, nullptr}};
  return slots;
}
} // namespace dartnb

namespace dart::python {
nb::object make_simbicon_controller(
    std::shared_ptr<dart_demos::atlas_simbicon::Controller> controller,
    dart::simulation::WorldPtr world)
{
  return nb::cast(DemoController(std::move(controller), std::move(world)));
}

void bind_simbicon(nb::module_& demos)
{
  auto m = demos.def_submodule(
      "simbicon", "Private GUI-free Atlas SIMBICON controller bindings.");
  dartnb::dart_class<DemoController>(
      m, "Controller", "Atlas controller retaining its robot and World.")
      .def(
          nb::init<nb::object, nb::object>(),
          nb::arg("atlas"),
          nb::arg("world"),
          "Build a controller for an Atlas skeleton and its owning World.")
      .def_prop_ro(
          "atlas",
          [](const DemoController& self) { return self.atlasWrapper; },
          "The Atlas skeleton controlled by this instance.")
      .def_prop_ro(
          "world",
          [](const DemoController& self) { return self.worldWrapper; },
          "The World whose constraint solver this controller uses.")
      .def_prop_ro(
          "current_state_machine",
          [](DemoController& self) {
            return self.get().getCurrentState()->getName();
          },
          "Name of the active gait state machine.")
      .def_prop_ro(
          "current_state",
          [](DemoController& self) {
            return self.get().getCurrentState()->getCurrentState()->getName();
          },
          "Name of the active state within the gait state machine.")
      .def(
          "update",
          [](DemoController& self) { self.get().update(); },
          "Apply control immediately before advancing the World.")
      .def(
          "change_state_machine",
          [](DemoController& self, const std::string& name, double time) {
            self.get().changeStateMachine(name, time);
          },
          nb::arg("name"),
          nb::arg("time"),
          "Select a gait by name, beginning it at the given simulation time.")
      .def(
          "is_allowing_control",
          [](const DemoController& self) {
            return self.get().isAllowingControl();
          },
          "Whether pelvis height permits the controller to apply forces.")
      .def(
          "harness_pelvis",
          [](DemoController& self) { self.get().harnessPelvis(); },
          "Attach the pelvis harness to the World solver.")
      .def(
          "unharness_pelvis",
          [](DemoController& self) { self.get().unharnessPelvis(); },
          "Remove the pelvis harness.")
      .def(
          "harness_left_foot",
          [](DemoController& self) { self.get().harnessLeftFoot(); },
          "Create the native left-foot harness and mark it enabled.")
      .def(
          "unharness_left_foot",
          [](DemoController& self) { self.get().unharnessLeftFoot(); },
          "Remove the left-foot harness.")
      .def(
          "harness_right_foot",
          [](DemoController& self) { self.get().harnessRightFoot(); },
          "Create the native right-foot harness and mark it enabled.")
      .def(
          "unharness_right_foot",
          [](DemoController& self) { self.get().unharnessRightFoot(); },
          "Remove the right-foot harness.")
      .def(
          "reset_robot",
          [](DemoController& self) { self.get().resetRobot(); },
          "Restore the robot's initial positions and velocities.")
      .def(
          "set_verbosity",
          [](DemoController& self, bool enabled) {
            self.get().setVerbosity(enabled);
          },
          nb::arg("enabled"),
          "Enable or disable native controller debug output.")
      .def(
          "print_debug_info",
          [](const DemoController& self) { self.get().printDebugInfo(); },
          "Print the native controller's current debug information.");
}
} // namespace dart::python

#endif
