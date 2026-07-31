/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#ifndef DART_EXAMPLES_DEMOS_MEMORY_DIAGNOSTICS_HPP_
#define DART_EXAMPLES_DEMOS_MEMORY_DIAGNOSTICS_HPP_

#include "memory_diagnostics_model.hpp"

#include <dart/gui/panel.hpp>

#include <functional>
#include <string>

#include <cstdint>

namespace dart::simulation {
class World;
}

namespace dart::examples::demos {

/// Collects one opt-in memory/layout sample for a DART 7 World.
DiagnosticSnapshot collectMemoryDiagnostics(
    const simulation::World& world, std::uint64_t generation);

/// Creates the scene-local, default-off diagnostics panel used by dart-demos.
gui::Panel createMemoryDiagnosticsPanel(
    std::string sceneLabel,
    std::function<const simulation::World*()> worldProvider);

} // namespace dart::examples::demos

#endif // DART_EXAMPLES_DEMOS_MEMORY_DIAGNOSTICS_HPP_
