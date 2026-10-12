/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#ifndef DART_GUI_APP_MEMORY_DIAGNOSTICS_HPP_
#define DART_GUI_APP_MEMORY_DIAGNOSTICS_HPP_

#include "memory_diagnostics_model.hpp"

#include <dart/simulation/World.hpp>

#include <cstdint>

namespace dart::gui::app {

/// Collects one DART 6 diagnostic snapshot without retaining graph pointers.
DiagnosticSnapshot collectMemoryDiagnostics(
    const simulation::WorldPtr& world,
    std::uint64_t generation,
    const ProcessMemoryReading& processMemory = collectProcessMemory());

/// Renders the raw-ImGui DART 6 view for a renderer-neutral session.
void renderMemoryDiagnostics(
    DiagnosticSession& session, double monotonicNowSeconds, double guiScale);

} // namespace dart::gui::app

#endif // DART_GUI_APP_MEMORY_DIAGNOSTICS_HPP_
