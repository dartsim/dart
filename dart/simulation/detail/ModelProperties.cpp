/*
 * Copyright (c) 2026, The DART development contributors
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

#include "dart/common/Composite.hpp"
#include "dart/common/detail/ModelMutation.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/ShapeFrame.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"

#include <algorithm>

namespace dart {
namespace common {
namespace detail {

//==============================================================================
std::size_t incrementAspectVersion(VersionCounter* object, const Aspect* aspect)
{
  if (auto* shape = dynamic_cast<dynamics::ShapeFrame*>(object)) {
    if (dynamic_cast<const dynamics::VisualAspect*>(aspect))
      return object->incrementLocalVersion();

    if (dynamic_cast<const dynamics::CollisionAspect*>(aspect)
        || dynamic_cast<const dynamics::DynamicsAspect*>(aspect)) {
      const std::size_t version = object->incrementParameterVersion();
      notifyParameterMutation(shape);
      if (auto* node = shape->asShapeNode()) {
        if (const auto skeleton = node->getSkeleton())
          skeleton->incrementDeactivationStateVersion();
      }
      return version;
    }
  }

  return object->incrementVersion();
}

//==============================================================================
void checkTopologyProperties(
    const Composite* object, const Composite::Properties& properties)
{
  if (const auto* shape = dynamic_cast<const dynamics::ShapeFrame*>(object)) {
    if (object->has<dynamics::ShapeFrame::Aspect>()) {
      if (const auto* incoming = properties.get<dynamics::ShapeFrame>()) {
        if (shape->getShape().get() != incoming->mShape.get())
          checkTopologyMutation(object);
      }
    }
  }

  if (const auto* joint = dynamic_cast<const dynamics::Joint*>(object)) {
    if (object->has<dynamics::Joint::Aspect>()) {
      if (const auto* incoming = properties.get<dynamics::Joint>()) {
        const auto& current = joint->getJointProperties();
        if (current.mActuatorType != incoming->mActuatorType
            || current.mActuatorTypes != incoming->mActuatorTypes)
          checkTopologyMutation(object);

        const std::size_t currentCount
            = std::min(joint->getNumDofs(), current.mMimicDofProps.size());
        const std::size_t incomingCount
            = std::min(joint->getNumDofs(), incoming->mMimicDofProps.size());
        if (currentCount != incomingCount)
          checkTopologyMutation(object);
        const std::size_t count = std::min(
            current.mMimicDofProps.size(), incoming->mMimicDofProps.size());
        for (std::size_t i = 0; i < count; ++i) {
          if (current.mMimicDofProps[i].mConstraintType
              != incoming->mMimicDofProps[i].mConstraintType)
            checkTopologyMutation(object);
        }
      }
    }
  }

  if (const auto* soft = dynamic_cast<const dynamics::SoftBodyNode*>(object)) {
    if (object->has<dynamics::SoftBodyNode::Aspect>()) {
      if (const auto* incoming = properties.get<dynamics::SoftBodyNode>()) {
        const auto& current = soft->getAspectProperties();
        if (current.mPointProps.size() != incoming->mPointProps.size()
            || current.mFaces != incoming->mFaces)
          checkTopologyMutation(object);
        const std::size_t count = std::min(
            current.mPointProps.size(), incoming->mPointProps.size());
        for (std::size_t i = 0; i < count; ++i) {
          if (current.mPointProps[i].mConnectedPointMassIndices
              != incoming->mPointProps[i].mConnectedPointMassIndices)
            checkTopologyMutation(object);
        }
      }
    }
  }

  if (const auto* skeleton = dynamic_cast<const dynamics::Skeleton*>(object)) {
    using Bodies = dynamics::detail::BodyNodeVectorProxyAspect;
    if (object->has<Bodies>()) {
      if (const auto* incoming = properties.get<Bodies>()) {
        const auto entries = incoming->get();
        const std::size_t count
            = std::min(skeleton->getNumBodyNodes(), entries.size());
        for (std::size_t i = 0; i < count; ++i)
          checkTopologyProperties(skeleton->getBodyNode(i), entries[i]);
      }
    }

    using Joints = dynamics::detail::JointVectorProxyAspect;
    if (object->has<Joints>()) {
      if (const auto* incoming = properties.get<Joints>()) {
        const auto entries = incoming->get();
        const std::size_t count
            = std::min(skeleton->getNumJoints(), entries.size());
        for (std::size_t i = 0; i < count; ++i)
          checkTopologyProperties(skeleton->getJoint(i), entries[i]);
      }
    }
  }

  if (const auto* body = dynamic_cast<const dynamics::BodyNode*>(object)) {
    using Nodes = dynamics::detail::NodeVectorProxyAspect;
    if (object->has<Nodes>()) {
      if (const auto* incoming = properties.get<Nodes>()) {
        const auto entries = incoming->get();
        const auto nodes = body->getNodes();
        for (const auto& [type, data] : entries.getMap()) {
          if (!data)
            continue;

          const auto& values = data->getVector();
          std::size_t index = 0;
          for (const auto* node : nodes) {
            if (std::type_index(typeid(*node)) != type)
              continue;
            if (index >= values.size())
              break;

            const auto* nodeObject = dynamic_cast<const Composite*>(node);
            const auto* value = dynamic_cast<const Composite::Properties*>(
                values[index++].get());
            if (nodeObject && value)
              checkTopologyProperties(nodeObject, *value);
          }
        }
      }
    }
  }
}

} // namespace detail
} // namespace common
} // namespace dart
