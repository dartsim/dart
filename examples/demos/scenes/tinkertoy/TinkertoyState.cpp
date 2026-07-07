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

#include "TinkertoyState.hpp"

#include <algorithm>

namespace dart_demos {
namespace tinkertoy {

//==============================================================================
void TinkertoyState::handlePick(const dart::gui::osg::PickInfo& pick)
{
  auto* bn
      = dynamic_cast<dart::dynamics::BodyNode*>(pick.frame->getParentFrame());
  if (!bn)
    return;

  pickedNode = bn;
  pickedPoint = bn->getWorldTransform().inverse() * pick.position;

  Eigen::Isometry3d tf = bn->getWorldTransform();
  tf.translation()
      = pick.position + pick.normal.normalized() * kDefaultBlockWidth / 2.0;
  target->setTransform(tf);
}

//==============================================================================
void TinkertoyState::clearPick()
{
  pickedNode = nullptr;
  target->setTransform(Eigen::Isometry3d::Identity());
}

//==============================================================================
void TinkertoyState::reorientTarget()
{
  Eigen::Isometry3d tf = target->getWorldTransform();
  tf.linear() = Eigen::Matrix3d::Identity();
  target->setTransform(tf);
}

//==============================================================================
void TinkertoyState::incrementForceCoeff()
{
  forceCoeff = std::min(forceCoeff + kForceIncrement, kMaxForceCoeff);
}

//==============================================================================
void TinkertoyState::decrementForceCoeff()
{
  forceCoeff = std::max(forceCoeff - kForceIncrement, kMinForceCoeff);
}

//==============================================================================
void TinkertoyMouseHandler::update()
{
  auto* defaultHandler = mViewer->getDefaultEventHandler();
  if (defaultHandler->getButtonEvent(dart::gui::osg::LEFT_MOUSE)
      != dart::gui::osg::BUTTON_PUSH)
    return;

  const auto& picks = defaultHandler->getButtonPicks(
      dart::gui::osg::LEFT_MOUSE, dart::gui::osg::BUTTON_PUSH);
  if (picks.empty())
    return;

  mState->handlePick(picks.front());
}

} // namespace tinkertoy
} // namespace dart_demos
