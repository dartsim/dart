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
 *     copyright notice, this list of conditions and the following disclaimer
 *     in the documentation and/or other materials provided with the
 *     distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "demo_models/AdaptiveSoftContactModel.hpp"
#include "detail/eigen.hpp"

#include <vector>

namespace dart::python {
namespace adaptive_soft_contact_binding {

using AdaptiveModel = dart_demos::adaptive_soft_contact_model::Model;
using PointPositions
    = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;
using ActiveFlags = Eigen::Array<bool, Eigen::Dynamic, 1>;

PointPositions pointPositions(const AdaptiveModel& model)
{
  const std::size_t count = model.softBody->getNumPointMasses();
  PointPositions positions(static_cast<Eigen::Index>(count), 3);
  for (std::size_t i = 0; i < count; ++i) {
    positions.row(static_cast<Eigen::Index>(i))
        = model.softBody->getPointMass(i)->getWorldPosition().transpose();
  }
  return positions;
}

ActiveFlags activeFlags(const AdaptiveModel& model)
{
  const std::vector<bool> active
      = dart_demos::adaptive_soft_contact_model::estimateActivePoints(model);
  ActiveFlags flags(static_cast<Eigen::Index>(active.size()));
  for (std::size_t i = 0; i < active.size(); ++i)
    flags[static_cast<Eigen::Index>(i)] = active[i];
  return flags;
}

} // namespace adaptive_soft_contact_binding

void bind_adaptive_soft_contact(nb::module_& demos)
{
  namespace model = dart_demos::adaptive_soft_contact_model;
  auto m = demos.def_submodule(
      "adaptive_soft_contact", "Private GUI-free adaptive soft-contact model.");

  m.attr("time_step") = model::kTimeStep;
  m.attr("soft_body_radius") = model::kSoftBodyRadius;
  m.attr("compare_tolerance") = model::kCompareTolerance;
  m.attr("default_ring_count") = model::kDefaultRingCount;
  m.attr("default_linger_steps") = model::kDefaultLingerSteps;

  // Factories supply native shared owners. Read-only properties cannot inject
  // Python pins; borrowed graph wrappers retain their Model and skeleton.
  dartnb::dart_class<model::Model>(
      m,
      "Model",
      "Owns the simulation world and the soft-body/pusher skeletons.")
      .def_prop_ro(
          "world",
          [](const model::Model& self) { return self.world; },
          "The owned simulation world.")
      .def_prop_ro(
          "soft_skeleton",
          [](const model::Model& self) { return self.softSkeleton; },
          "The owned soft-ellipsoid skeleton.")
      .def_prop_ro(
          "pusher_skeleton",
          [](const model::Model& self) { return self.pusherSkeleton; },
          "The owned periodic-pusher skeleton.")
      .def_prop_ro(
          "soft_body",
          [](const model::Model& self) { return self.softBody; },
          nb::rv_policy::reference_internal,
          nb::keep_alive<0, 1>(),
          "The soft body; its wrapper retains this Model.")
      .def_prop_ro(
          "pusher_joint",
          [](const model::Model& self) { return self.pusherJoint; },
          nb::rv_policy::reference_internal,
          nb::keep_alive<0, 1>(),
          "The pusher's free joint; its wrapper retains this Model.");

  dartnb::dart_class<model::Checksum>(
      m, "Checksum", "Deterministic scalar summaries of a model's state.")
      .def_ro(
          "skeleton_position_l1",
          &model::Checksum::skeletonPositionL1,
          "Sum of absolute skeleton position components.")
      .def_ro(
          "skeleton_position_squared",
          &model::Checksum::skeletonPositionSquared,
          "Squared norm of skeleton positions.")
      .def_ro(
          "point_world_position_l1",
          &model::Checksum::pointWorldPositionL1,
          "Sum of absolute soft-point world-position components.")
      .def_ro(
          "point_world_position_squared",
          &model::Checksum::pointWorldPositionSquared,
          "Sum of squared soft-point world-position norms.")
      .def_ro(
          "active", &model::Checksum::active, "Number of active soft points.")
      .def_ro("total", &model::Checksum::total, "Total number of soft points.")
      .def_ro(
          "contacts",
          &model::Checksum::contacts,
          "Number of last-step contacts.")
      .def_ro(
          "finite", &model::Checksum::finite, "Whether the state is finite.");

  m.def(
      "create_model",
      &model::createModel,
      nb::kw_only(),
      nb::arg("adaptive_enabled") = true,
      nb::arg("ring_count") = model::kDefaultRingCount,
      nb::arg("linger_steps") = model::kDefaultLingerSteps,
      "Build a complete adaptive soft-contact world and its owned model.");
  m.def(
      "configure",
      &model::configure,
      nb::arg("model"),
      nb::kw_only(),
      nb::arg("adaptive_enabled"),
      nb::arg("ring_count"),
      nb::arg("linger_steps"),
      "Update adaptive activation settings without rebuilding the world.");
  m.def(
      "set_pusher_pose",
      &model::setPusherPose,
      nb::arg("model"),
      nb::arg("time"),
      "Set the periodic pusher pose for the supplied simulation time.");
  m.def(
      "prepare_step",
      &model::prepareStep,
      nb::arg("model"),
      "Update the pusher immediately before stepping the world.");
  m.def(
      "step",
      &model::step,
      nb::arg("model"),
      "Prepare and advance one step; return whether the state is finite.");
  m.def(
      "compute_checksum",
      &model::computeChecksum,
      nb::arg("model"),
      "Return deterministic scalar summaries of the current model state.");
  m.def(
      "position_checksum",
      &model::positionChecksum,
      nb::arg("model"),
      "Return a weighted checksum of skeleton and soft-point positions.");
  m.def(
      "is_finite",
      &model::isFinite,
      nb::arg("model"),
      "Return whether the skeleton and soft-point state is finite.");
  m.def(
      "surface_pose_delta",
      &model::surfacePoseDelta,
      nb::arg("lhs"),
      nb::arg("rhs"),
      "Return the translation/rotation surface-pose delta between two models.");
  m.def(
      "point_positions",
      &adaptive_soft_contact_binding::pointPositions,
      nb::arg("model"),
      "Return an owned float64 (N, 3) array of soft-point world positions.");
  m.def(
      "active_flags",
      &adaptive_soft_contact_binding::activeFlags,
      nb::arg("model"),
      "Return owned bool (N,) marker flags from estimateActivePoints(): the "
      "points nearest the last contacts, as many as are active.");
}

} // namespace dart::python
