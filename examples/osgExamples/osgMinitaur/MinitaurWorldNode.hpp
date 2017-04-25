/*
 * Copyright (c) 2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2017, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#ifndef DART_EXAMPLE_OSG_OSGMINITAUR_MINITAURWORLDNODE_HPP_
#define DART_EXAMPLE_OSG_OSGMINITAUR_MINITAURWORLDNODE_HPP_

#include <dart/dart.hpp>
#include <dart/utils/utils.hpp>
#include <dart/gui/osg/osg.hpp>

#include "Controller.hpp"

class MinitaurWorldNode : public dart::gui::osg::WorldNode
{
public:
  /// Constructor
  MinitaurWorldNode(const dart::simulation::WorldPtr& world,
                         const dart::dynamics::SkeletonPtr& atlas);

  // Documentation inherited
  void customPreStep() override;

  void reset();

  void pushForwardAtlas(double force = 500, int frames = 100);
  void pushBackwardAtlas(double force = 500, int frames = 100);
  void pushLeftAtlas(double force = 500, int frames = 100);
  void pushRightAtlas(double force = 500, int frames = 100);

  void switchToNormalStrideWalking();
  void switchToWarmingUp();
  void switchToNoControl();

protected:
  std::unique_ptr<Controller> mController;
  Eigen::Vector3d mExternalForce;
  int mForceDuration;
};

#endif // DART_EXAMPLE_OSG_OSGMINITAUR_MINITAURWORLDNODE_HPP_
