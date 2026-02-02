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
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include <cstdint>

namespace dart {
namespace gui {

/// Material for a renderable node
struct Material
{
  Eigen::Vector4d color{0.5, 0.5, 0.5, 1.0};
  Eigen::Vector4d ambient{0.2, 0.2, 0.2, 1.0};
  Eigen::Vector4d specular{1.0, 1.0, 1.0, 1.0};
  double shininess = 32.0;
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
  std::vector<Eigen::Vector2f> texcoords;
  std::string texture_path;
};

struct LineData
{
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> segments;
};

struct PyramidData
{
  double base_width;
  double base_depth;
  double height;
};

enum class PointShapeType
{
  Box,
  BillboardSquare,
  BillboardCircle,
  Point
};
enum class PointCloudColorMode
{
  ShapeColor,
  Overall,
  PerPoint
};

struct PointCloudData
{
  std::vector<Eigen::Vector3d> points;
  double visual_size = 0.01;
  PointShapeType point_shape_type = PointShapeType::Box;
  PointCloudColorMode color_mode = PointCloudColorMode::ShapeColor;
  Eigen::Vector4d overall_color{0.5, 0.5, 0.5, 1.0};
  std::vector<Eigen::Vector4d> colors;
};

struct MultiSphereData
{
  /// Each pair: (radius, center_position)
  std::vector<std::pair<double, Eigen::Vector3d>> spheres;
};

struct HeightmapData
{
  std::vector<float> heights; // row-major
  std::size_t width = 0;
  std::size_t depth = 0;
  Eigen::Vector3d scale{1.0, 1.0, 1.0};
};

struct VoxelGridData
{
  /// Each voxel: (center_position, half_size)
  std::vector<std::pair<Eigen::Vector3d, double>> voxels;
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
    LineData,
    PyramidData,
    PointCloudData,
    MultiSphereData,
    HeightmapData,
    VoxelGridData>;

/// Grid configuration for the viewer
struct GridConfig
{
  enum class Plane
  {
    XY,
    YZ,
    ZX
  };

  Plane plane = Plane::ZX;
  std::size_t num_cells = 20;
  double cell_size = 1.0;
  std::size_t minor_per_major = 5;
  Eigen::Vector4d major_color{0.5, 0.5, 0.5, 1.0};
  Eigen::Vector4d minor_color{0.7, 0.7, 0.7, 0.5};
  float major_width = 2.0f;
  float minor_width = 1.0f;
  Eigen::Vector3d offset{0, 0, 0};

  /// Create default ZX grid
  static GridConfig defaultZX()
  {
    return GridConfig{};
  }
};

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

/// Support polygon debug visualization
struct DebugPolygon
{
  std::vector<Eigen::Vector3d> vertices; // polygon vertices in world space
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  Eigen::Vector3d com = Eigen::Vector3d::Zero();
  Eigen::Vector4d fill_color{0.0, 0.8, 0.0, 0.3};
  Eigen::Vector4d outline_color{0.0, 1.0, 0.0, 1.0};
  Eigen::Vector4d centroid_color{0.0, 0.0, 1.0, 1.0};
  Eigen::Vector4d com_color{1.0, 0.0, 0.0, 1.0};
};

/// Debug wireframe polyhedron
struct DebugPolyhedron
{
  std::vector<Eigen::Vector3d> vertices;
  std::vector<std::pair<std::size_t, std::size_t>> edges; // index pairs
  Eigen::Vector4d color{1.0, 1.0, 0.0, 0.8};
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
  std::vector<DebugPolygon> debug_polygons;
  std::vector<DebugPolyhedron> debug_polyhedra;
  Camera camera;
  std::vector<Light> lights;
  bool headlight = true;
  std::optional<GridConfig> grid_config = GridConfig::defaultZX();
  bool show_axes = true;
  bool paused = false;
  double sim_time = 0.0;
  std::optional<uint64_t> selected_node_id;
};

} // namespace gui
} // namespace dart

#endif // DART_GUI_SCENE_HPP_
