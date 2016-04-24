/*
 * Copyright (c) 2014-2016, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#ifndef EXAMPLES_ATLASSIMBICON_HUMANOID_HPP_
#define EXAMPLES_ATLASSIMBICON_HUMANOID_HPP_

#include <vector>
#include <string>

#include <Eigen/Dense>

#include "dart/dart.hpp"

class State;

//==============================================================================
/// \brief class Humanoid
/// \warning This class is not used now.
class Humanoid
{
public:
  /// \brief Constructor
  Humanoid(dart::dynamics::Skeleton* _skeleton);

  /// \brief Destructor
  virtual ~Humanoid();

  /// \brief Get skeleton
  dart::dynamics::Skeleton* getSkeleton();

  /// \brief Get pelvis
  dart::dynamics::BodyNode* getPelvis();

  /// \brief Get left thigh
  dart::dynamics::BodyNode* getLeftThigh();

  /// \brief Get right thigh
  dart::dynamics::BodyNode* getRightThigh();

  /// \brief Get left foot
  dart::dynamics::BodyNode* getLeftFoot();

  /// \brief Get right foot
  dart::dynamics::BodyNode* getRightFoot();

protected:
  /// \brief State
  dart::dynamics::Skeleton* mSkeleton;

  /// \brief Pelvis
  dart::dynamics::BodyNode* mPelvis;

  /// \brief LeftThigh
  dart::dynamics::BodyNode* mLeftThigh;

  /// \brief LeftThigh
  dart::dynamics::BodyNode* mRightThigh;

  /// \brief Left foot
  dart::dynamics::BodyNode* mLeftFoot;

  /// \brief Right foot
  dart::dynamics::BodyNode* mRightFoot;

  /// \brief Left ankle on sagital plane
  dart::dynamics::Joint* mLeftAnkleSagital;

  /// \brief Left ankle on coronal plane
  dart::dynamics::Joint* mLeftAnkleCoronal;

  /// \brief Right ankle on sagital plane
  dart::dynamics::Joint* mRightAnkleSagital;

  /// \brief Right ankle on coronal plane
  dart::dynamics::Joint* mRightAnkleCoronal;
};

//==============================================================================
/// \brief class AtlasRobot
/// \warning This class is not used now.
class AtlasRobot : public Humanoid
{
public:
  /// \brief Constructor
  AtlasRobot(dart::dynamics::Skeleton* _skeleton);

  /// \brief Destructor
  virtual ~AtlasRobot();

protected:
};

#endif  // EXAMPLES_ATLASSIMBICON_HUMANOID_HPP_
