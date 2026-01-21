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

#include "dart/gui/vsg/collision_scene_builder.hpp"

#include "dart/gui/vsg/conversions.hpp"
#include "dart/gui/vsg/debug_draw.hpp"
#include "dart/gui/vsg/geometry_builders.hpp"

#include <dart/collision/experimental/aabb.hpp>
#include <dart/collision/experimental/collision_object.hpp>
#include <dart/collision/experimental/types.hpp>

namespace dart::gui::vsg {

CollisionSceneBuilder::CollisionSceneBuilder() = default;
CollisionSceneBuilder::~CollisionSceneBuilder() = default;

void CollisionSceneBuilder::addObject(
    const collision::experimental::CollisionObject& obj,
    const Eigen::Vector4d& color)
{
  const auto* shape = obj.getShape();
  if (!shape) {
    return;
  }

  auto geometry = createFromShape(*shape, GeometryOptions{color, false, false});
  auto transform = createTransform(obj.getTransform());
  transform->addChild(geometry);
  m_nodes.push_back(transform);
}

void CollisionSceneBuilder::addContacts(
    const collision::experimental::CollisionResult& result,
    double normalLength,
    double pointSize)
{
  std::vector<Eigen::Vector3d> contactPositions;
  std::vector<Eigen::Vector3d> contactNormals;

  for (size_t i = 0; i < result.numManifolds(); ++i) {
    const auto& manifold = result.getManifold(i);
    for (size_t j = 0; j < manifold.numContacts(); ++j) {
      const auto& contact = manifold.getContact(j);
      contactPositions.push_back(contact.position);
      contactNormals.push_back(contact.normal);
    }
  }

  if (!contactPositions.empty()) {
    m_nodes.push_back(createPoints(contactPositions, pointSize, colors::Red));
    m_nodes.push_back(createArrows(
        contactPositions, contactNormals, normalLength, colors::Yellow));
  }
}

void CollisionSceneBuilder::addSphereCast(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    double radius,
    const collision::experimental::CcdResult* hit)
{
  auto startSphere
      = createSphere(radius, GeometryOptions{colors::Cyan, true, false});
  auto startTransform = createTransform(
      Eigen::Translation3d(start) * Eigen::Isometry3d::Identity());
  startTransform->addChild(startSphere);
  m_nodes.push_back(startTransform);

  m_nodes.push_back(createLine(start, end, colors::Cyan));

  Eigen::Vector3d endPos = end;
  Eigen::Vector4d endColor = colors::Cyan;

  if (hit && hit->hit) {
    endPos = start + (end - start) * hit->timeOfImpact;
    endColor = colors::Red;
    m_nodes.push_back(createPoint(hit->point, 0.03, colors::Red));
    m_nodes.push_back(
        createArrow(hit->point, hit->normal, 0.15, colors::Yellow));
  }

  auto endSphere = createSphere(radius, GeometryOptions{endColor, true, false});
  auto endTransform = createTransform(
      Eigen::Translation3d(endPos) * Eigen::Isometry3d::Identity());
  endTransform->addChild(endSphere);
  m_nodes.push_back(endTransform);
}

void CollisionSceneBuilder::addAabb(
    const collision::experimental::Aabb& aabb, const Eigen::Vector4d& color)
{
  m_nodes.push_back(createWireframeBox(aabb.min, aabb.max, color));
}

void CollisionSceneBuilder::addDistanceResult(
    const collision::experimental::DistanceResult& result,
    const Eigen::Vector4d& lineColor,
    const Eigen::Vector4d& pointColor)
{
  if (!result.isValid()) {
    return;
  }

  m_nodes.push_back(
      createLine(result.pointOnObject1, result.pointOnObject2, lineColor));
  m_nodes.push_back(createPoint(result.pointOnObject1, 0.04, pointColor));
  m_nodes.push_back(createPoint(result.pointOnObject2, 0.04, pointColor));
}

void CollisionSceneBuilder::addRaycast(
    const collision::experimental::Ray& ray,
    const collision::experimental::RaycastResult* hit,
    const Eigen::Vector4d& rayColor,
    const Eigen::Vector4d& hitColor)
{
  Eigen::Vector3d endPoint = ray.pointAt(ray.maxDistance);
  if (hit && hit->hit) {
    endPoint = hit->point;
  }

  m_nodes.push_back(createLine(ray.origin, endPoint, rayColor));
  m_nodes.push_back(createPoint(ray.origin, 0.03, rayColor));

  if (hit && hit->hit) {
    m_nodes.push_back(createPoint(hit->point, 0.05, hitColor));
    m_nodes.push_back(createArrow(hit->point, hit->normal, 0.2, hitColor));
  }
}

::vsg::ref_ptr<::vsg::Node> CollisionSceneBuilder::build()
{
  auto group = ::vsg::Group::create();
  for (auto& node : m_nodes) {
    group->addChild(node);
  }
  return group;
}

void CollisionSceneBuilder::clear()
{
  m_nodes.clear();
}

} // namespace dart::gui::vsg
