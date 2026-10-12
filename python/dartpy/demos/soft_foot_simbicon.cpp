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
  #include "demo_models/soft_foot_simbicon/SoftFootSimbiconModel.hpp"
  #include "detail/eigen.hpp"

  #include <stdexcept>
  #include <string>

namespace dart::python {
nb::object make_simbicon_controller(
    std::shared_ptr<dart_demos::atlas_simbicon::Controller> controller,
    dart::simulation::WorldPtr world);

void bind_soft_foot_simbicon(nb::module_& demos)
{
  namespace model = dart_demos::soft_foot_simbicon_model;
  auto m = demos.def_submodule(
      "soft_foot_simbicon", "Private rigid/soft-foot Atlas model bindings.");
  m.attr("time_step") = model::kTimeStep;
  m.attr("push_steps") = model::kPushSteps;
  m.attr("default_push_magnitude") = model::kDefaultPushMagnitude;
  m.attr("upright_gap") = model::kUprightGap;
  m.attr("push_sweep_start") = model::kPushSweepStart;
  m.attr("push_sweep_end") = model::kPushSweepEnd;
  m.attr("push_sweep_step") = model::kPushSweepStep;

  nb::enum_<model::Feet>(m, "Feet", "Foot geometry used by the Atlas model.")
      .value("RIGID", model::Feet::Rigid)
      .value("SOFT", model::Feet::Soft);

  // Native children share a simulation graph; borrowed views retain its owner.
  dartnb::dart_class<model::Model>(
      m, "Model", "Owning state for an Atlas-on-ground SIMBICON simulation.")
      .def_prop_ro(
          "world",
          [](const model::Model& self) { return self.world; },
          "The model's owning World.")
      .def_prop_ro(
          "atlas",
          [](const model::Model& self) { return self.atlas; },
          "The Atlas skeleton.")
      .def_prop_ro(
          "controller",
          [](const model::Model& self) {
            return make_simbicon_controller(self.controller, self.world);
          },
          "A controller handle retaining this model's World and Atlas.")
      .def_prop_ro(
          "pelvis",
          [](const model::Model& self) { return self.pelvis; },
          nb::rv_policy::reference_internal,
          nb::keep_alive<0, 1>(),
          "The pelvis body; retains this Model.")
      .def_prop_ro(
          "left_foot",
          [](const model::Model& self) { return self.leftFoot; },
          nb::rv_policy::reference_internal,
          nb::keep_alive<0, 1>(),
          "The left foot body; retains this Model.")
      .def_prop_ro(
          "right_foot",
          [](const model::Model& self) { return self.rightFoot; },
          nb::rv_policy::reference_internal,
          nb::keep_alive<0, 1>(),
          "The right foot body; retains this Model.")
      .def_prop_ro(
          "external_force",
          [](const model::Model& self) { return self.externalForce; },
          "The scheduled pelvis push in world coordinates, in newtons.")
      .def_prop_ro(
          "force_duration",
          [](const model::Model& self) { return self.forceDuration; },
          "Remaining steps in the scheduled pelvis push.")
      .def_prop_ro(
          "feet",
          [](const model::Model& self) { return self.feet; },
          "The model's foot geometry.");

  m.def(
      "create_model",
      [](model::Feet feet) {
        try {
          return model::createModel(feet);
        } catch (const std::runtime_error& error) {
          throw std::runtime_error(
              std::string(error.what())
              + "; these demo models load DART's sample data (dart://sample),"
                " which a DART source checkout provides");
        }
      },
      nb::arg("feet"),
      "Build the Y-up Atlas-on-ground World with the requested foot geometry. "
      "It loads DART's sample data, which a source checkout provides.");
  m.def(
      "apply_push",
      [](model::Model& self, const Eigen::Vector3d& force, int steps) {
        if (!force.allFinite())
          throw nb::value_error("force must be finite");
        if (steps < 0)
          throw nb::value_error("steps must not be negative");
        model::applyPush(self, force, steps);
      },
      nb::arg("model"),
      nb::arg("force"),
      nb::arg("steps"),
      "Schedule a world-frame pelvis force for the given number of steps.");
  m.def(
      "prepare_step",
      &model::prepareStep,
      nb::arg("model"),
      "Apply the pending push and update control before World.step().");
  m.def(
      "step",
      &model::step,
      nb::arg("model"),
      "Prepare control and advance the model by one simulation step.");
  m.def(
      "pelvis_height_y",
      &model::pelvisHeightY,
      nb::arg("model"),
      "Return the pelvis world-frame Y coordinate in meters.");
  m.def(
      "mean_foot_height_y",
      &model::meanFootHeightY,
      nb::arg("model"),
      "Return the mean world-frame Y coordinate of the two feet.");
  m.def(
      "is_upright",
      &model::isUpright,
      nb::arg("model"),
      "Whether the pelvis remains at least upright_gap above the feet.");
  m.def(
      "foot_contact_count",
      &model::footContactCount,
      nb::arg("model"),
      "Count last-step contact points involving either foot.");
  m.def(
      "is_finite",
      &model::isFinite,
      nb::arg("model"),
      "Whether all skeleton and soft-point state is finite.");
  m.def(
      "state_vector",
      &model::stateVector,
      nb::arg("model"),
      "Return skeleton positions/velocities, then soft-point state.");
  m.def(
      "reset_model",
      &model::resetModel,
      nb::arg("model"),
      "Restore initial robot and soft-point state, gait, and pending push.");
  m.def(
      "max_recoverable_push",
      &model::maxRecoverablePush,
      nb::arg("feet"),
      // A sweep of a dozen simulations; other Python threads keep running.
      nb::call_guard<nb::gil_scoped_release>(),
      "Sweep lateral pushes and return the largest recoverable force in N.");
}
} // namespace dart::python

#endif
