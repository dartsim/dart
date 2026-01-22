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

#include "hardcoded_event_handler.hpp"

#include <iostream>

HardcodedEventHandler::HardcodedEventHandler(dart::dynamics::SkeletonPtr _skel)
  : mSkel(_skel), mInverse(false)
{
}

bool HardcodedEventHandler::handle(
    const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& /*aa*/)
{
  if (ea.getEventType() != osgGA::GUIEventAdapter::KEYDOWN) {
    return false;
  }

  switch (ea.getKey()) {
    case '-': {
      mInverse = !mInverse;
      std::cout << "Direction inverted: "
                << (mInverse ? "negative" : "positive") << std::endl;
      return true;
    }
    case '1':
    case '2':
    case '3': {
      std::size_t dofIdx = ea.getKey() - '1';
      if (dofIdx < mSkel->getNumDofs()) {
        Eigen::VectorXd pose = mSkel->getPositions();
        pose(dofIdx) = pose(dofIdx) + (mInverse ? -mDOF : mDOF);
        mSkel->setPositions(pose);
        std::cout << "Updated pose DOF " << dofIdx << ": " << pose.transpose()
                  << std::endl;
      }
      return true;
    }
    default:
      return false;
  }
}
