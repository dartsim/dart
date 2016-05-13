/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
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

#include "dart/dynamics/FixedFrame.hpp"

namespace dart {
namespace dynamics {

namespace detail {

//==============================================================================
FixedFrameProperties::FixedFrameProperties(const Eigen::Isometry3d& relativeTf)
  : mRelativeTf(relativeTf)
{
  // Do nothing
}

} // namespace detail

//==============================================================================
const Eigen::Vector6d FixedFrame::mZero = Eigen::Vector6d::Zero();

//==============================================================================
FixedFrame::FixedFrame(Frame* refFrame,
                       const Eigen::Isometry3d& relativeTransform)
  : Entity(refFrame, false),
    Frame(refFrame)
{
  createAspect<Aspect>(AspectProperties(relativeTransform));
}

//==============================================================================
FixedFrame::~FixedFrame()
{
  // Do nothing. The inherited destructors will do all the necessary cleanup.
}

//==============================================================================
void FixedFrame::setAspectProperties(const AspectProperties& properties)
{
  setRelativeTransform(properties.mRelativeTf);
}

//==============================================================================
void FixedFrame::setRelativeTransform(const Eigen::Isometry3d& transform)
{
  if(transform.matrix() == mAspectProperties.mRelativeTf.matrix())
    return;

  mAspectProperties.mRelativeTf = transform;
  notifyTransformUpdate();
  incrementVersion();
}

//==============================================================================
const Eigen::Isometry3d& FixedFrame::getRelativeTransform() const
{
  return mAspectProperties.mRelativeTf;
}

//==============================================================================
const Eigen::Vector6d& FixedFrame::getRelativeSpatialVelocity() const
{
  return mZero;
}

//==============================================================================
const Eigen::Vector6d& FixedFrame::getRelativeSpatialAcceleration() const
{
  return mZero;
}

//==============================================================================
const Eigen::Vector6d& FixedFrame::getPrimaryRelativeAcceleration() const
{
  return mZero;
}

//==============================================================================
const Eigen::Vector6d& FixedFrame::getPartialAcceleration() const
{
  return mZero;
}

//==============================================================================
FixedFrame::FixedFrame()
  : FixedFrame(ConstructAbstract)
{
  // Delegates to the abstract constructor
}

//==============================================================================
FixedFrame::FixedFrame(ConstructAbstractTag)
{
  dterr << "[FixedFrame::FixedFrame] Attempting to construct a pure abstract "
        << "FixedFrame object. This is not allowed!\n";
  assert(false);
}

} // namespace dynamics
} // namespace dart
