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

#include <cstddef>

namespace dart::simulation {

/// Integration family used for multibody dynamics on the default
/// `World::step()` path.
///
/// This is a typed, documented selector on the public facade surface (mirroring
/// `RigidBodySolver` and `ContactSolverMethod`): it names the discretization
/// family without exposing any solver, stage, registry, or backend type, and it
/// is resolved once when the configuration is applied so the per-step path
/// carries no parsing cost.
enum class MultibodyIntegrationFamily
{
  /// Semi-implicit joint-space integration. The long-standing default.
  SemiImplicit,

  /// The linear-time discrete-mechanics (variational) integrator.
  Variational,
};

/// World-level solver/integration configuration for the multibody domain.
///
/// A value object set as a whole via `World::setMultibodyOptions()` rather than
/// one setter/getter per setting, so new capabilities are added as fields here
/// instead of accumulating World methods. Configuration is by typed
/// method-family selector (the public-facade "Solver And Execution Policy"
/// capability matrix); no solver, stage, or backend types are exposed.
///
/// This is the multibody **solver** configuration, distinct from any future
/// per-multibody construction options.
struct MultibodyOptions
{
  /// Integration family used for multibody dynamics on the default `step()`
  /// path. Defaults to `SemiImplicit` (semi-implicit joint-space integration);
  /// the other documented value is `Variational` (the linear-time discrete-
  /// mechanics integrator).
  MultibodyIntegrationFamily integrationFamily
      = MultibodyIntegrationFamily::SemiImplicit;

  /// Maximum RIQN iterations for the variational integrator on the default
  /// `World::step()` path. Must be positive.
  std::size_t variationalMaxIterations = 100;

  /// Per-coordinate convergence tolerance for the variational integrator on
  /// the default `World::step()` path. Must be positive and finite.
  double variationalTolerance = 1e-10;
};

} // namespace dart::simulation
