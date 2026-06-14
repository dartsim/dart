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

#include <dart/common/stl_allocator.hpp>

#include <Eigen/Core>

#include <vector>

#include <cstddef>

namespace dart::simulation::comps {

/// Per-multibody ground-contact configuration for the variational integrator.
///
/// Stores an analytic ground half-space `{x : n . (x - p0) >= 0}` plus a set of
/// body-fixed contact points (by link index, parallel arrays) with
/// compliant-penalty, lagged-Coulomb-friction, and Kelvin-Voigt-damping
/// parameters. When present on a multibody, the variational-integration stage
/// builds the in-loop compliant-contact hook (PLAN-082 Phase C rungs C1/C2)
/// from it and folds it into the forced-DEL root-find each step; absent (the
/// default), the integrator is contact-free.
///
/// Stored as a serialized `Property`-category component: it is configuration
/// (set once via `Multibody::setGroundContact`, read each step, never mutated
/// by the sim), so it persists across binary save/load like the sibling
/// `LoopClosure` config. Registered in `io/serializer.cpp`; the contact-point
/// link indices are positions in the multibody's link order (not entity
/// references), so no entity-remap pass entry is needed.
///
/// **Internal Implementation Detail** - Not exposed in public API.
struct VariationalContact
{
  DART_SIMULATION_PROPERTY_COMPONENT(VariationalContact);

  using LinkIndexVector
      = std::vector<std::size_t, dart::common::StlAllocator<std::size_t>>;
  using PointVector = std::
      vector<Eigen::Vector3d, dart::common::StlAllocator<Eigen::Vector3d>>;

  Eigen::Vector3d planeNormal = Eigen::Vector3d::UnitZ(); ///< out of the ground
  Eigen::Vector3d planePoint = Eigen::Vector3d::Zero();   ///< a point on plane
  double stiffness = 0.0;                 ///< penalty k (N/m), >= 0
  double frictionCoefficient = 0.0;       ///< Coulomb mu (>= 0); 0 disables
  double frictionRegularization = 1.0e-4; ///< friction velocity scale (m/s)
  double dampingCoefficient = 0.0;        ///< Kelvin-Voigt normal damping

  /// Augmented-Lagrangian (C3) dual-update cadence. `0` keeps the C2 compliant
  /// penalty (the robust default); `N > 0` enables the drift-free AL rung,
  /// advancing the per-point duals every `N` steps -- an outer-loop cadence
  /// slower than the primal, which the undamped symplectic step needs for
  /// stability. The duals live in the `VariationalContactDualState` component.
  std::size_t dualUpdateCadence = 0;

  /// Contact-point link indices (into the multibody structure's link order).
  LinkIndexVector pointLinkIndices;
  /// Contact-point body-frame positions, parallel to `pointLinkIndices`.
  PointVector pointLocalPositions;
};

} // namespace dart::simulation::comps
