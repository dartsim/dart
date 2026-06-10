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

#include <string>

#include <cstddef>

namespace dart::simulation {

/// World-level solver/integration configuration for the multibody domain.
///
/// A value object set as a whole via `World::setMultibodyOptions()` rather than
/// one setter/getter per setting, so new capabilities are added as fields here
/// instead of accumulating World methods. Configuration is by documented
/// method-family name (the public-facade "Solver And Execution Policy"
/// capability matrix); no solver, stage, or backend types are exposed.
///
/// This is the multibody **solver** configuration, distinct from any future
/// per-multibody construction options.
struct MultibodyOptions
{
  /// Integration family used for multibody dynamics on the default `step()`
  /// path. Documented values: `"semi-implicit"` (default; articulated-body
  /// forward dynamics) and `"variational integrator"` (the linear-time
  /// discrete-mechanics integrator). `World::setMultibodyOptions()` throws
  /// InvalidArgumentException for unknown names.
  std::string integrationFamily = "semi-implicit";

  /// Maximum RIQN iterations for the variational integrator on the default
  /// `World::step()` path. Must be positive.
  std::size_t variationalMaxIterations = 100;

  /// Per-coordinate convergence tolerance for the variational integrator on
  /// the default `World::step()` path. Must be positive and finite.
  double variationalTolerance = 1e-10;
};

} // namespace dart::simulation
