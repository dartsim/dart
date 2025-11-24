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

#ifndef DART_EXAMPLES_DEMO_APP_CONTROLLEDPENDULUMSCENE_HPP_
#define DART_EXAMPLES_DEMO_APP_CONTROLLEDPENDULUMSCENE_HPP_

#include "core/Scene.hpp"
#include "core/WorldNodeHooks.hpp"

#include <dart/dynamics/Fwd.hpp>

#include <Eigen/Geometry>

namespace dart::demo {

class ControlledPendulumScene : public Scene
{
public:
  ControlledPendulumScene();

  const SceneMetadata& metadata() const override;
  dart::simulation::WorldPtr world() const override;
  osg::ref_ptr<dart::gui::osg::RealTimeWorldNode> node() override;
  void reset() override;
  void step(double dt) override;
  void renderImGui() override;

private:
  void buildScene();
  void updateTargetFrame();
  void createGround();

  SceneMetadata mMetadata;
  dart::simulation::WorldPtr mWorld;
  osg::ref_ptr<HookedWorldNode> mNode;
  dart::dynamics::SkeletonPtr mPendulum;
  dart::dynamics::RevoluteJoint* mShoulder{nullptr};
  dart::dynamics::SimpleFramePtr mTargetFrame;
  double mTargetAngle{0.35};
  double mHomeAngle{0.35};
  double mKp{45.0};
  double mKd{4.5};
  double mArmLength{1.0};
};

std::shared_ptr<Scene> createControlledPendulumScene();

} // namespace dart::demo

#endif // DART_EXAMPLES_DEMO_APP_CONTROLLEDPENDULUMSCENE_HPP_
