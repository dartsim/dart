/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License
 */

#pragma once

#include <dart/simulation/comps/component_category.hpp>

#include <vector>

#include <cstddef>
#include <cstdint>

namespace dart::simulation::comps {

/// Persistent augmented-Lagrangian dual state for variational-integrator ground
/// contact (PLAN-084 Phase C, rung C3), paired with `VariationalContact` on the
/// same multibody. Holds the per-contact-point duals (`lambda >= 0`, the
/// accumulated AL contact forces) and the cadence counter that gates the
/// outer-loop dual ascent.
///
/// Serialized as a `State` component so an AL contact scene resumes
/// bit-identically across binary save/load -- the duals are dynamic state,
/// warm-started across steps. It exists only when
/// `VariationalContact::dualUpdateCadence > 0`; the C2 compliant default never
/// creates it, so the robust path carries no extra state.
///
/// **Internal Implementation Detail** - Not exposed in public API.
struct VariationalContactDualState
{
  DART_SIMULATION_STATE_COMPONENT(VariationalContactDualState);

  /// Per-contact-point duals, parallel to
  /// `VariationalContact::pointLinkIndices` (and
  /// `VariationalContact::pointLocalPositions`).
  std::vector<double> duals;

  /// Steps since the last dual ascent. The variational-integration stage
  /// advances the duals and resets this to 0 once it reaches
  /// `VariationalContact::dualUpdateCadence`.
  std::uint64_t stepsSinceDualUpdate = 0;
};

} // namespace dart::simulation::comps
