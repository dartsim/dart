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

#include <dart/gui/experimental/renderable.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/shape.hpp>
#include <dart/dynamics/shape_frame.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <algorithm>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <cstddef>

namespace dart::gui::experimental {

RenderableId makeRenderableId(const dynamics::ShapeFrame& shapeFrame)
{
  return reinterpret_cast<RenderableId>(&shapeFrame);
}

RenderableId makeRenderableId(const dynamics::ShapeNode& shapeNode)
{
  return makeRenderableId(static_cast<const dynamics::ShapeFrame&>(shapeNode));
}

namespace {

void hashCombine(std::size_t& seed, std::size_t value)
{
  constexpr auto kHashMixConstant
      = static_cast<std::size_t>(0x9e3779b97f4a7c15ULL);
  seed ^= value + kHashMixConstant + (seed << 6) + (seed >> 2);
}

void hashVector3d(std::size_t& seed, const Eigen::Vector3d& value)
{
  for (int axis = 0; axis < 3; ++axis) {
    hashCombine(seed, std::hash<double>{}(value[axis]));
  }
}

void hashVector4d(std::size_t& seed, const Eigen::Vector4d& value)
{
  for (int axis = 0; axis < 4; ++axis) {
    hashCombine(seed, std::hash<double>{}(value[axis]));
  }
}

std::size_t computeRenderResourceVersion(
    const GeometryDescriptor& geometry,
    const MaterialDescriptor& material,
    std::size_t shapeVersion)
{
  std::size_t seed = shapeVersion;
  hashCombine(seed, static_cast<std::size_t>(geometry.kind));
  hashCombine(seed, std::hash<double>{}(material.rgba.w()));
  if (geometry.kind == ShapeKind::Mesh) {
    hashCombine(seed, geometry.meshUsesMaterialColors ? 1u : 0u);
    hashCombine(seed, static_cast<std::size_t>(geometry.meshAlphaMode));
  }
  if (geometry.kind == ShapeKind::PointCloud) {
    hashCombine(seed, std::hash<double>{}(geometry.pointSize));
    hashCombine(seed, geometry.pointCloudColors.size());
    for (const Eigen::Vector4d& color : geometry.pointCloudColors) {
      hashVector4d(seed, color);
    }
  }
  if (geometry.kind == ShapeKind::SoftMesh) {
    hashCombine(seed, geometry.triangleVertices.size());
    for (const Eigen::Vector3d& vertex : geometry.triangleVertices) {
      hashVector3d(seed, vertex);
    }
    hashCombine(seed, geometry.triangleIndices.size());
    for (const Eigen::Vector3i& triangle : geometry.triangleIndices) {
      for (int axis = 0; axis < 3; ++axis) {
        hashCombine(seed, std::hash<int>{}(triangle[axis]));
      }
    }
  }
  return seed;
}

} // namespace

std::optional<RenderableDescriptor> makeRenderableDescriptor(
    const dynamics::ShapeFrame& shapeFrame,
    const dynamics::VisualAspect& visualAspect,
    const std::string& skeletonName,
    const std::string& bodyName,
    const std::string& shapeNodeName,
    const dynamics::WeakConstSimpleFramePtr& simpleFrame = {})
{
  const auto shape = shapeFrame.getShape();
  if (!shape) {
    return std::nullopt;
  }

  auto geometry = describeShape(*shape);
  if (!geometry) {
    return std::nullopt;
  }

  RenderableDescriptor descriptor;
  descriptor.id = makeRenderableId(shapeFrame);
  descriptor.shapeFrame = &shapeFrame;
  descriptor.shapeNode = shapeFrame.asShapeNode();
  descriptor.shape = shape.get();
  if (descriptor.shapeNode != nullptr) {
    descriptor.skeleton = descriptor.shapeNode->getSkeleton();
  }
  descriptor.simpleFrame = simpleFrame;
  descriptor.skeletonName = skeletonName;
  descriptor.bodyName = bodyName;
  descriptor.shapeFrameName = shapeFrame.getName();
  descriptor.shapeNodeName = shapeNodeName;
  descriptor.geometry = std::move(*geometry);
  descriptor.material.rgba = visualAspect.getRGBA();
  descriptor.material.visible = !visualAspect.isHidden();
  descriptor.material.castsShadows = visualAspect.getShadowed();
  descriptor.material.receivesShadows = visualAspect.getShadowed();
  descriptor.worldTransform = shapeFrame.getWorldTransform();
  descriptor.shapeFrameVersion = shapeFrame.getVersion();
  descriptor.shapeNodeVersion = descriptor.shapeNode != nullptr
                                    ? descriptor.shapeNode->getVersion()
                                    : 0;
  descriptor.shapeVersion = shape->getVersion();
  descriptor.renderResourceVersion = computeRenderResourceVersion(
      descriptor.geometry, descriptor.material, descriptor.shapeVersion);
  return descriptor;
}

std::vector<RenderableDescriptor> extractRenderables(
    const simulation::World& world)
{
  std::vector<RenderableDescriptor> renderables;

  for (std::size_t skeletonIndex = 0; skeletonIndex < world.getNumSkeletons();
       ++skeletonIndex) {
    const auto skeleton = world.getSkeleton(skeletonIndex);
    if (!skeleton) {
      continue;
    }

    skeleton->eachBodyNode([&](const dynamics::BodyNode* bodyNode) {
      bodyNode->eachShapeNodeWith<dynamics::VisualAspect>(
          [&](const dynamics::ShapeNode* shapeNode) {
            const auto* visualAspect = shapeNode->getVisualAspect();
            auto descriptor = makeRenderableDescriptor(
                *shapeNode,
                *visualAspect,
                skeleton->getName(),
                bodyNode->getName(),
                shapeNode->getName());
            if (descriptor) {
              renderables.push_back(std::move(*descriptor));
            }
          });
    });
  }

  for (std::size_t i = 0; i < world.getNumSimpleFrames(); ++i) {
    const auto simpleFrame = world.getSimpleFrame(i);
    if (simpleFrame == nullptr || !simpleFrame->hasVisualAspect()) {
      continue;
    }

    auto descriptor = makeRenderableDescriptor(
        *simpleFrame, *simpleFrame->getVisualAspect(), {}, {}, {}, simpleFrame);
    if (descriptor) {
      renderables.push_back(std::move(*descriptor));
    }
  }

  return renderables;
}

} // namespace dart::gui::experimental
