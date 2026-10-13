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

#ifndef DART_PYTHON_NATIVE_DEMO_MODELS_CONTACTINVERSEDYNAMICSMODEL_HPP_
#define DART_PYTHON_NATIVE_DEMO_MODELS_CONTACTINVERSEDYNAMICSMODEL_HPP_

#include <dart/dart.hpp>

#include <memory>
#include <string>
#include <vector>

namespace dart_demos {

/// GUI-free kinematic squat replay, contact solver, arrows, and plot histories.
class ContactInverseDynamicsModel
{
public:
  explicit ContactInverseDynamicsModel(dart::simulation::WorldPtr world);

  void updateFrame();
  void requestRegeneration();
  void setPlaybackSpeed(float value);
  void setSquatDepth(float value);
  void setFrictionCoeff(float value);
  void setForceScale(float value);

  dart::simulation::WorldPtr world;
  dart::dynamics::SkeletonPtr biped;
  std::unique_ptr<dart::dynamics::ContactInverseDynamics> solver;

  dart::dynamics::BodyNode* leftHeel = nullptr;
  dart::dynamics::BodyNode* leftToe = nullptr;
  dart::dynamics::BodyNode* rightHeel = nullptr;
  dart::dynamics::BodyNode* rightToe = nullptr;

  std::vector<dart::dynamics::SimpleFramePtr> contactFrames;
  std::vector<std::shared_ptr<dart::dynamics::ArrowShape>> arrows;

  std::vector<Eigen::VectorXd> keyframes;
  Eigen::VectorXd initialPositions;

  std::vector<std::string> torqueNames;
  std::vector<std::size_t> torqueDofs;

  double time = 0.0;

  // Panel-owned controls (ImGuiWidget state in the original).
  bool playing = true;
  float playbackSpeed = 1.0f;
  float squatDepth = 0.15f;
  float frictionCoeff = 0.8f;
  float forceScale = 0.002f;
  // Set when the Squat Depth slider is released or Regenerate is pressed;
  // consumed at the top of the next updateFrame().
  bool regenerateRequested = false;

  // Status readout.
  bool feasible = true;
  double residualNorm = 0.0;
  std::vector<Eigen::Vector3d> contactForces;
  double totalVerticalForce = 0.0;

  std::vector<float> plotSamples = std::vector<float>(240, 0.0f);
  std::vector<std::vector<float>> torquePlots
      = std::vector<std::vector<float>>(3, std::vector<float>(240, 0.0f));
  std::size_t sampleIndex = 0;
};

} // namespace dart_demos

#endif // DART_PYTHON_NATIVE_DEMO_MODELS_CONTACTINVERSEDYNAMICSMODEL_HPP_
