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
 *     copyright notice and this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
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

#include <dart/simulation/experimental/body/deformable_body.hpp>
#include <dart/simulation/experimental/export.hpp>

#include <filesystem>
#include <iosfwd>
#include <string>
#include <vector>

#include <cstddef>

namespace dart::simulation::experimental {
class World;
} // namespace dart::simulation::experimental

namespace dart::simulation::experimental::io {

/// Options for loading an experimental deformable scene text file.
///
/// This loader covers the audited contact-free tutorial-style subset used by
/// PLAN-081 bring-up. It creates DART deformable bodies, scripted boundary
/// conditions, and diagnostics scaffolding; it does not enable FEM elasticity,
/// mesh contact, projected Newton, CCD line search, or friction.
struct DeformableSceneLoadOptions
{
  /// Optional root used to resolve asset paths that are not relative to the
  /// scene file.
  std::filesystem::path assetRoot;

  /// Prefix used for generated deformable-body names.
  std::string bodyNamePrefix = "deformable_scene";

  /// Build distance-spring edges from loaded mesh topology so contact-free
  /// mesh scenes can move through the existing point-mass/spring stage.
  bool addStructuralSprings = true;

  /// Spring stiffness used for generated structural edges. This is independent
  /// of Young's modulus until FEM material models land in a later PLAN-081
  /// slice.
  double structuralSpringStiffness = 100.0;

  /// Velocity damping assigned to generated spring models.
  double damping = 0.01;

  /// Ignore reference-scene contact/friction directives and report them as
  /// warnings. Contact-free replay is the only behavior covered by this slice.
  bool ignoreContactDirectives = true;
};

struct DeformableSceneBodyInfo
{
  std::string name;
  DeformableBody body;
  std::size_t nodeCount = 0;
  std::size_t tetrahedronCount = 0;
  std::size_t surfaceTriangleCount = 0;
  std::size_t dirichletConditionCount = 0;
  std::size_t neumannConditionCount = 0;
};

struct DeformableSceneInfo
{
  double duration = 0.0;
  double timeStep = 0.0;
  bool gravityEnabled = true;
  std::vector<DeformableSceneBodyInfo> bodies;
  std::vector<std::string> warnings;
};

struct DeformableSceneDiagnostics
{
  std::size_t frame = 0;
  double time = 0.0;
  std::size_t bodyCount = 0;
  std::size_t nodeCount = 0;
  std::size_t tetrahedronCount = 0;
  std::size_t surfaceTriangleCount = 0;
  std::size_t dirichletConditionCount = 0;
  std::size_t neumannConditionCount = 0;
  double totalMass = 0.0;
  double maxDisplacement = 0.0;
  double minZ = 0.0;
  double maxZ = 0.0;
};

/// Load a contact-free deformable scene into a World.
DART_EXPERIMENTAL_API DeformableSceneInfo loadDeformableScene(
    World& world,
    const std::filesystem::path& scenePath,
    const DeformableSceneLoadOptions& options = DeformableSceneLoadOptions{});

/// Collect replay-oriented diagnostics from all deformable bodies in a World.
DART_EXPERIMENTAL_API DeformableSceneDiagnostics
collectDeformableSceneDiagnostics(const World& world);

/// Write diagnostics as a compact JSON object.
DART_EXPERIMENTAL_API void writeDeformableSceneDiagnosticsJson(
    std::ostream& output, const DeformableSceneDiagnostics& diagnostics);

/// Save a restart image for deformable scene replay.
DART_EXPERIMENTAL_API void saveDeformableSceneRestart(
    const World& world, std::ostream& output);

/// Load a restart image for deformable scene replay.
DART_EXPERIMENTAL_API void loadDeformableSceneRestart(
    World& world, std::istream& input);

} // namespace dart::simulation::experimental::io
