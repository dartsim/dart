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

#pragma once

#include <dart/gui/vsg/export.hpp>
#include <dart/gui/vsg/materials.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vsg/all.h>

#include <vector>

namespace dart::collision::experimental {
class Aabb;
class CollisionObject;
class CollisionResult;
struct CcdResult;
struct DistanceResult;
struct Ray;
struct RaycastResult;
} // namespace dart::collision::experimental

namespace dart::gui::vsg {

class DART_GUI_VSG_API CollisionSceneBuilder
{
public:
  CollisionSceneBuilder();
  ~CollisionSceneBuilder();

  void addObject(
      const collision::experimental::CollisionObject& obj,
      const Eigen::Vector4d& color = colors::Gray);

  void addContacts(
      const collision::experimental::CollisionResult& result,
      double normalLength = 0.1,
      double pointSize = 0.02);

  void addSphereCast(
      const Eigen::Vector3d& start,
      const Eigen::Vector3d& end,
      double radius,
      const collision::experimental::CcdResult* hit = nullptr);

  void addAabb(
      const collision::experimental::Aabb& aabb,
      const Eigen::Vector4d& color = colors::Yellow);

  void addDistanceResult(
      const collision::experimental::DistanceResult& result,
      const Eigen::Vector4d& lineColor = colors::Cyan,
      const Eigen::Vector4d& pointColor = colors::Magenta);

  void addRaycast(
      const collision::experimental::Ray& ray,
      const collision::experimental::RaycastResult* hit = nullptr,
      const Eigen::Vector4d& rayColor = colors::Cyan,
      const Eigen::Vector4d& hitColor = colors::Red);

  ::vsg::ref_ptr<::vsg::Node> build();

  void clear();

private:
  std::vector<::vsg::ref_ptr<::vsg::Node>> m_nodes;
};

} // namespace dart::gui::vsg
