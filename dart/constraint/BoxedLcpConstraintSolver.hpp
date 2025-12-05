/*
 * Copyright (c) 2011-2025, The DART development contributors
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
 *   * Redistributions in binary forms, with or
 *   modification, are permitted provided that the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
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

#pragma once

#include <dart/common/Deprecated.hpp>
#include <dart/constraint/BoxedLcpSolver.hpp>
#include <dart/constraint/ConstraintSolver.hpp>

namespace dart {
namespace constraint {

/// Compatibility subclass that preserves the legacy boxed LCP solver API used
/// by external integrations (e.g., gz-physics) while delegating to the unified
/// math::LcpSolver pipeline.
class DART_DEPRECATED(7.0) DART_API BoxedLcpConstraintSolver
    : public ConstraintSolver
{
public:
  BoxedLcpConstraintSolver();
  explicit BoxedLcpConstraintSolver(BoxedLcpSolverPtr solver);
  ~BoxedLcpConstraintSolver() override = default;

  /// Set the boxed LCP solver (also updates the underlying math::LcpSolver).
  void setBoxedLcpSolver(BoxedLcpSolverPtr solver);

  /// Get the current boxed LCP solver wrapper.
  BoxedLcpSolverPtr getBoxedLcpSolver() const;

private:
  void applySolver(BoxedLcpSolverPtr solver);

  BoxedLcpSolverPtr mBoxedLcpSolver;
};

} // namespace constraint
} // namespace dart
