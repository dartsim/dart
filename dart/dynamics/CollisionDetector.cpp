/*
 * Copyright (c) 2016, Georgia Tech Research Corporation
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

#include "dart/dynamics/CollisionDetector.h"

namespace dart {
namespace dynamics {

//==============================================================================
ShapeNodeCollisionObject::ShapeNodeCollisionObject(
    const collision::EnginePtr& engine,
    const dynamics::ShapePtr& shape,
    const dynamics::BodyNodePtr& bodyNode)
  : CollisionObject(engine, shape),
    mBodyNode(bodyNode)
{
  auto found = false;
  auto numShapes = mBodyNode->getNumCollisionShapes();
  for (auto i = 0u; i < numShapes; ++i)
  {
    auto shapeIt = mBodyNode->getCollisionShape(i);
    if (shape == shapeIt)
    {
      found = true;
      break;
    }
  }

  if (!found)
  {
    dtwarn << "[ShapeNodeCollisionObject::constructor] Attempting to create "
           << "ShapeNodeCollisionObject with invalid pair of Shape and "
           << "BodyNode.\n";
    assert(false);
  }
}

//==============================================================================
const Eigen::Isometry3d ShapeNodeCollisionObject::getTransform() const
{
  return mBodyNode->getWorldTransform() * mShape->getLocalTransform();
}

//==============================================================================
BodyNodePtr ShapeNodeCollisionObject::getBodyNode() const
{
  return mBodyNode;
}

//==============================================================================
std::shared_ptr<ShapeNodeCollisionObject>
CollisionDetector::createCollisionNode(
    const collision::EnginePtr engine,
    const ShapePtr& shape,
    const BodyNodePtr& bodyNode)
{
  return std::make_shared<ShapeNodeCollisionObject>(engine, shape, bodyNode);
}

} // namespace dynamics
} // namespace dart
