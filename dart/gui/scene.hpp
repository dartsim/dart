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

#ifndef DART_GUI_SCENE_HPP_
#define DART_GUI_SCENE_HPP_

#include <Eigen/Geometry>

#include <optional>
#include <utility>
#include <variant>
#include <vector>

#include <cstdint>

namespace dart {
namespace gui {

/// Material for a renderable node
struct Material
{
  Eigen::Vector4d color{0.5, 0.5, 0.5, 1.0}; // RGBA
  bool wireframe = false;
};

/// Shape data variants — one per shape type
/// Use std::variant to avoid virtual dispatch
struct BoxData
{
  Eigen::Vector3d size;
};

struct SphereData
{
  double radius;
};

struct CylinderData
{
  double radius;
  double height;
};

struct CapsuleData
{
  double radius;
  double height;
};

struct ConeData
{
  double radius;
  double height;
};

struct EllipsoidData
{
  Eigen::Vector3d radii;
};

struct PlaneData
{
  Eigen::Vector3d normal;
  double offset;
};

struct MeshData
{
  std::vector<Eigen::Vector3f> vertices;
  std::vector<Eigen::Vector3f> normals;
  std::vector<uint32_t> indices;
};

struct LineData
{
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> segments;
};

using ShapeData = std::variant<
    BoxData,
    SphereData,
    CylinderData,
    CapsuleData,
    ConeData,
    EllipsoidData,
    PlaneData,
    MeshData,
    LineData>;

/// A single renderable node in the scene
struct SceneNode
{
  uint64_t id = 0; // Stable ID for picking
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  ShapeData shape;
  Material material;
  bool visible = true;
};

/// Debug drawing primitives
struct DebugLine
{
  Eigen::Vector3d start, end;
  Eigen::Vector4d color{1, 0, 0, 1};
};

struct DebugPoint
{
  Eigen::Vector3d position;
  Eigen::Vector4d color{1, 1, 0, 1};
  double size = 3.0;
};

/// Result of a ray-cast pick operation
struct HitResult
{
  uint64_t node_id = 0;
  double distance = 0.0;
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
};

/// Camera configuration
struct Camera
{
  Eigen::Vector3d position{2.0, -2.0, 2.0};
  Eigen::Vector3d target{0.0, 0.0, 0.0};
  Eigen::Vector3d up{0.0, 0.0, 1.0};
  double fovy = 45.0; // degrees
};

/// Light configuration
struct Light
{
  Eigen::Vector3d direction{-0.5, -0.5, -1.0};
  Eigen::Vector4d color{1.0, 1.0, 1.0, 1.0};
};

/// The complete scene snapshot — pure data, no GPU resources
struct Scene
{
  std::vector<SceneNode> nodes;
  std::vector<DebugLine> debug_lines;
  std::vector<DebugPoint> debug_points;
  Camera camera;
  std::vector<Light> lights;
  bool show_grid = true;
  bool show_axes = true;
  bool paused = false;
  double sim_time = 0.0;
  std::optional<uint64_t> selected_node_id;
};

} // namespace gui
} // namespace dart

#endif // DART_GUI_SCENE_HPP_
