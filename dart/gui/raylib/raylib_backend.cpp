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

#include "dart/gui/raylib/raylib_backend.hpp"

#include "dart/common/logging.hpp"

#include <raylib.h>
#include <rlgl.h>

#include <algorithm>
#include <array>
#include <limits>
#include <optional>
#include <type_traits>

#include <cmath>
#include <cstring>

namespace dart {
namespace gui {
namespace {

Vector3 toVector3(const Eigen::Vector3d& value)
{
  return Vector3{
      static_cast<float>(value.x()),
      static_cast<float>(value.y()),
      static_cast<float>(value.z())};
}

Color toColor(const Eigen::Vector4d& value)
{
  const auto clamp = [](double v) {
    return static_cast<unsigned char>(std::clamp(v, 0.0, 1.0) * 255.0);
  };

  return Color{
      clamp(value[0]), clamp(value[1]), clamp(value[2]), clamp(value[3])};
}

Matrix eigenToRaylib(const Eigen::Isometry3d& transform)
{
  Eigen::Matrix4f matrix = transform.matrix().cast<float>();
  Matrix result;
  std::memcpy(&result, matrix.data(), sizeof(Matrix));
  return result;
}

Camera3D toRaylibCamera(const Camera& camera)
{
  Camera3D result{};
  result.position = toVector3(camera.position);
  result.target = toVector3(camera.target);
  result.up = toVector3(camera.up);
  result.fovy = static_cast<float>(camera.fovy);
  result.projection = CAMERA_PERSPECTIVE;
  return result;
}

Matrix planeLocalTransform(const PlaneData& plane)
{
  Eigen::Vector3d normal = plane.normal;
  if (normal.norm() < 1e-9) {
    normal = Eigen::Vector3d::UnitZ();
  }
  normal.normalize();

  Eigen::Vector3d tangent = normal.cross(Eigen::Vector3d::UnitZ());
  if (tangent.norm() < 1e-6) {
    tangent = normal.cross(Eigen::Vector3d::UnitX());
  }
  tangent.normalize();
  Eigen::Vector3d bitangent = normal.cross(tangent);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear().col(0) = tangent;
  transform.linear().col(1) = normal;
  transform.linear().col(2) = bitangent;
  transform.translation() = normal * plane.offset;
  return eigenToRaylib(transform);
}

void drawMeshData(const MeshData& data, const Color& color)
{
  if (data.vertices.empty() || data.indices.empty()) {
    return;
  }

  const std::size_t vertexCount = data.vertices.size();
  const std::size_t indexCount = data.indices.size();

  if (indexCount % 3 != 0) {
    DART_WARN("Mesh indices are not a multiple of 3. Skipping mesh draw.");
    return;
  }

  const uint32_t maxIndex
      = *std::max_element(data.indices.begin(), data.indices.end());
  if (maxIndex > std::numeric_limits<unsigned short>::max()) {
    DART_WARN("Mesh indices exceed 16-bit range. Skipping mesh draw.");
    return;
  }

  Mesh mesh{};
  mesh.vertexCount = static_cast<int>(vertexCount);
  mesh.triangleCount = static_cast<int>(indexCount / 3);
  mesh.vertices = static_cast<float*>(
      MemAlloc(static_cast<int>(vertexCount * 3 * sizeof(float))));

  for (std::size_t i = 0; i < vertexCount; ++i) {
    mesh.vertices[i * 3 + 0] = data.vertices[i].x();
    mesh.vertices[i * 3 + 1] = data.vertices[i].y();
    mesh.vertices[i * 3 + 2] = data.vertices[i].z();
  }

  if (data.normals.size() == vertexCount) {
    mesh.normals = static_cast<float*>(
        MemAlloc(static_cast<int>(vertexCount * 3 * sizeof(float))));
    for (std::size_t i = 0; i < vertexCount; ++i) {
      mesh.normals[i * 3 + 0] = data.normals[i].x();
      mesh.normals[i * 3 + 1] = data.normals[i].y();
      mesh.normals[i * 3 + 2] = data.normals[i].z();
    }
  }

  mesh.indices = static_cast<unsigned short*>(
      MemAlloc(static_cast<int>(indexCount * sizeof(unsigned short))));
  for (std::size_t i = 0; i < indexCount; ++i) {
    mesh.indices[i] = static_cast<unsigned short>(data.indices[i]);
  }

  UploadMesh(&mesh, false);

  ::Material rlMaterial = LoadMaterialDefault();
  rlMaterial.maps[MATERIAL_MAP_DIFFUSE].color = color;
  ::Matrix identity = eigenToRaylib(Eigen::Isometry3d::Identity());
  DrawMesh(mesh, rlMaterial, identity);
  UnloadMaterial(rlMaterial);
  UnloadMesh(mesh);
}

std::optional<Key> mapKey(int key)
{
  switch (key) {
    case KEY_SPACE:
      return Key::Space;
    case KEY_ESCAPE:
      return Key::Escape;
    case KEY_ENTER:
      return Key::Enter;
    case KEY_TAB:
      return Key::Tab;
    case KEY_LEFT:
      return Key::Left;
    case KEY_RIGHT:
      return Key::Right;
    case KEY_UP:
      return Key::Up;
    case KEY_DOWN:
      return Key::Down;
    case KEY_A:
      return Key::A;
    case KEY_B:
      return Key::B;
    case KEY_C:
      return Key::C;
    case KEY_D:
      return Key::D;
    case KEY_E:
      return Key::E;
    case KEY_F:
      return Key::F;
    case KEY_G:
      return Key::G;
    case KEY_H:
      return Key::H;
    case KEY_I:
      return Key::I;
    case KEY_J:
      return Key::J;
    case KEY_K:
      return Key::K;
    case KEY_L:
      return Key::L;
    case KEY_M:
      return Key::M;
    case KEY_N:
      return Key::N;
    case KEY_O:
      return Key::O;
    case KEY_P:
      return Key::P;
    case KEY_Q:
      return Key::Q;
    case KEY_R:
      return Key::R;
    case KEY_S:
      return Key::S;
    case KEY_T:
      return Key::T;
    case KEY_U:
      return Key::U;
    case KEY_V:
      return Key::V;
    case KEY_W:
      return Key::W;
    case KEY_X:
      return Key::X;
    case KEY_Y:
      return Key::Y;
    case KEY_Z:
      return Key::Z;
    case KEY_ZERO:
      return Key::Num0;
    case KEY_ONE:
      return Key::Num1;
    case KEY_TWO:
      return Key::Num2;
    case KEY_THREE:
      return Key::Num3;
    case KEY_FOUR:
      return Key::Num4;
    case KEY_FIVE:
      return Key::Num5;
    case KEY_SIX:
      return Key::Num6;
    case KEY_SEVEN:
      return Key::Num7;
    case KEY_EIGHT:
      return Key::Num8;
    case KEY_NINE:
      return Key::Num9;
    default:
      return std::nullopt;
  }
}

std::optional<MouseButton> mapMouseButton(int button)
{
  switch (button) {
    case MOUSE_BUTTON_LEFT:
      return MouseButton::Left;
    case MOUSE_BUTTON_RIGHT:
      return MouseButton::Right;
    case MOUSE_BUTTON_MIDDLE:
      return MouseButton::Middle;
    default:
      return std::nullopt;
  }
}

} // namespace

bool RaylibBackend::initialize(const ViewerConfig& config)
{
  if (initialized_) {
    return true;
  }

  config_ = config;
  if (config_.headless) {
    SetConfigFlags(FLAG_WINDOW_HIDDEN);
  }

  InitWindow(config_.width, config_.height, config_.title.c_str());
  if (config_.target_fps > 0) {
    SetTargetFPS(config_.target_fps);
  }

  initialized_ = true;
  return true;
}

bool RaylibBackend::shouldClose() const
{
  if (!initialized_) {
    return false;
  }

  return WindowShouldClose();
}

void RaylibBackend::beginFrame() {}

void RaylibBackend::render(const Scene& scene)
{
  if (!initialized_) {
    return;
  }

  const Camera3D camera = toRaylibCamera(scene.camera);

  BeginDrawing();
  ClearBackground(RAYWHITE);
  BeginMode3D(camera);

  if (scene.show_grid) {
    DrawGrid(20, 1.0f);
  }

  if (scene.show_axes) {
    const Vector3 origin{0.0f, 0.0f, 0.0f};
    DrawLine3D(origin, Vector3{1.0f, 0.0f, 0.0f}, RED);
    DrawLine3D(origin, Vector3{0.0f, 1.0f, 0.0f}, GREEN);
    DrawLine3D(origin, Vector3{0.0f, 0.0f, 1.0f}, BLUE);
  }

  for (const auto& node : scene.nodes) {
    if (!node.visible) {
      continue;
    }

    const Color color = toColor(node.material.color);
    rlPushMatrix();
    const Matrix transform = eigenToRaylib(node.transform);
    rlMultMatrixf(reinterpret_cast<const float*>(&transform));

    std::visit(
        [&](const auto& shape) {
          using ShapeT = std::decay_t<decltype(shape)>;
          if constexpr (std::is_same_v<ShapeT, BoxData>) {
            Vector3 size{
                static_cast<float>(shape.size.x()),
                static_cast<float>(shape.size.y()),
                static_cast<float>(shape.size.z())};
            DrawCubeV(Vector3{0.0f, 0.0f, 0.0f}, size, color);
            if (node.material.wireframe) {
              DrawCubeWiresV(Vector3{0.0f, 0.0f, 0.0f}, size, color);
            }
          } else if constexpr (std::is_same_v<ShapeT, SphereData>) {
            DrawSphere(
                Vector3{0.0f, 0.0f, 0.0f},
                static_cast<float>(shape.radius),
                color);
          } else if constexpr (std::is_same_v<ShapeT, CylinderData>) {
            rlPushMatrix();
            rlRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
            DrawCylinder(
                Vector3{0.0f, 0.0f, 0.0f},
                static_cast<float>(shape.radius),
                static_cast<float>(shape.radius),
                static_cast<float>(shape.height),
                24,
                color);
            rlPopMatrix();
          } else if constexpr (std::is_same_v<ShapeT, CapsuleData>) {
            rlPushMatrix();
            rlRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
            const float halfHeight = static_cast<float>(shape.height) * 0.5f;
            DrawCapsule(
                Vector3{0.0f, -halfHeight, 0.0f},
                Vector3{0.0f, halfHeight, 0.0f},
                static_cast<float>(shape.radius),
                16,
                16,
                color);
            rlPopMatrix();
          } else if constexpr (std::is_same_v<ShapeT, ConeData>) {
            rlPushMatrix();
            rlRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
            DrawCylinder(
                Vector3{0.0f, 0.0f, 0.0f},
                0.0f,
                static_cast<float>(shape.radius),
                static_cast<float>(shape.height),
                24,
                color);
            rlPopMatrix();
          } else if constexpr (std::is_same_v<ShapeT, EllipsoidData>) {
            rlPushMatrix();
            rlScalef(
                static_cast<float>(shape.radii.x()),
                static_cast<float>(shape.radii.y()),
                static_cast<float>(shape.radii.z()));
            DrawSphere(Vector3{0.0f, 0.0f, 0.0f}, 1.0f, color);
            rlPopMatrix();
          } else if constexpr (std::is_same_v<ShapeT, PlaneData>) {
            rlPushMatrix();
            const Matrix planeTransform = planeLocalTransform(shape);
            rlMultMatrixf(reinterpret_cast<const float*>(&planeTransform));
            DrawPlane(Vector3{0.0f, 0.0f, 0.0f}, Vector2{20.0f, 20.0f}, color);
            rlPopMatrix();
          } else if constexpr (std::is_same_v<ShapeT, MeshData>) {
            drawMeshData(shape, color);
          } else if constexpr (std::is_same_v<ShapeT, LineData>) {
            for (const auto& segment : shape.segments) {
              DrawLine3D(
                  toVector3(segment.first), toVector3(segment.second), color);
            }
          }
        },
        node.shape);

    rlPopMatrix();
  }

  for (const auto& line : scene.debug_lines) {
    DrawLine3D(toVector3(line.start), toVector3(line.end), toColor(line.color));
  }

  for (const auto& point : scene.debug_points) {
    DrawSphere(
        toVector3(point.position),
        static_cast<float>(point.size),
        toColor(point.color));
  }

  // Selection highlight: draw yellow wireframe overlay on selected node
  if (scene.selected_node_id.has_value()) {
    const uint64_t selectedId = scene.selected_node_id.value();
    for (const auto& node : scene.nodes) {
      if (node.id != selectedId || !node.visible) {
        continue;
      }

      rlPushMatrix();
      const Matrix selTransform = eigenToRaylib(node.transform);
      rlMultMatrixf(reinterpret_cast<const float*>(&selTransform));

      std::visit(
          [&](const auto& shape) {
            using ShapeT = std::decay_t<decltype(shape)>;
            if constexpr (std::is_same_v<ShapeT, BoxData>) {
              Vector3 size{
                  static_cast<float>(shape.size.x()),
                  static_cast<float>(shape.size.y()),
                  static_cast<float>(shape.size.z())};
              DrawCubeWiresV(Vector3{0.0f, 0.0f, 0.0f}, size, YELLOW);
            } else if constexpr (std::is_same_v<ShapeT, SphereData>) {
              DrawSphereWires(
                  Vector3{0.0f, 0.0f, 0.0f},
                  static_cast<float>(shape.radius),
                  12,
                  12,
                  YELLOW);
            } else if constexpr (std::is_same_v<ShapeT, CylinderData>) {
              rlPushMatrix();
              rlRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
              DrawCylinderWires(
                  Vector3{0.0f, 0.0f, 0.0f},
                  static_cast<float>(shape.radius),
                  static_cast<float>(shape.radius),
                  static_cast<float>(shape.height),
                  24,
                  YELLOW);
              rlPopMatrix();
            } else if constexpr (std::is_same_v<ShapeT, CapsuleData>) {
              rlPushMatrix();
              rlRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
              const float halfH = static_cast<float>(shape.height) * 0.5f;
              DrawCapsuleWires(
                  Vector3{0.0f, -halfH, 0.0f},
                  Vector3{0.0f, halfH, 0.0f},
                  static_cast<float>(shape.radius),
                  16,
                  16,
                  YELLOW);
              rlPopMatrix();
            } else if constexpr (std::is_same_v<ShapeT, ConeData>) {
              rlPushMatrix();
              rlRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
              DrawCylinderWires(
                  Vector3{0.0f, 0.0f, 0.0f},
                  0.0f,
                  static_cast<float>(shape.radius),
                  static_cast<float>(shape.height),
                  24,
                  YELLOW);
              rlPopMatrix();
            } else if constexpr (std::is_same_v<ShapeT, EllipsoidData>) {
              rlPushMatrix();
              rlScalef(
                  static_cast<float>(shape.radii.x()),
                  static_cast<float>(shape.radii.y()),
                  static_cast<float>(shape.radii.z()));
              DrawSphereWires(Vector3{0.0f, 0.0f, 0.0f}, 1.0f, 12, 12, YELLOW);
              rlPopMatrix();
            }
          },
          node.shape);

      rlPopMatrix();
      break;
    }
  }

  EndMode3D();

  DrawText(TextFormat("FPS: %d", GetFPS()), 20, 20, 20, DARKGRAY);
  DrawText(TextFormat("Sim: %.2f", scene.sim_time), 20, 45, 20, DARKBLUE);
  DrawText(scene.paused ? "Paused" : "Running", 20, 70, 20, DARKGREEN);

  if (scene.selected_node_id.has_value()) {
    DrawText(
        TextFormat("Selected: %llu", scene.selected_node_id.value()),
        20,
        95,
        20,
        ORANGE);
  }

  if (!pending_screenshot_.empty()) {
    Image image = LoadImageFromScreen();
    ExportImage(image, pending_screenshot_.c_str());
    UnloadImage(image);
    pending_screenshot_.clear();
  }

  EndDrawing();
}

void RaylibBackend::endFrame() {}

void RaylibBackend::shutdown()
{
  if (!initialized_) {
    return;
  }

  CloseWindow();
  initialized_ = false;
}

std::vector<InputEvent> RaylibBackend::pollEvents()
{
  std::vector<InputEvent> events;
  if (!initialized_) {
    return events;
  }

  static const std::array<int, 44> kKeys
      = {KEY_SPACE, KEY_ESCAPE, KEY_ENTER, KEY_TAB,  KEY_LEFT, KEY_RIGHT,
         KEY_UP,    KEY_DOWN,   KEY_A,     KEY_B,    KEY_C,    KEY_D,
         KEY_E,     KEY_F,      KEY_G,     KEY_H,    KEY_I,    KEY_J,
         KEY_K,     KEY_L,      KEY_M,     KEY_N,    KEY_O,    KEY_P,
         KEY_Q,     KEY_R,      KEY_S,     KEY_T,    KEY_U,    KEY_V,
         KEY_W,     KEY_X,      KEY_Y,     KEY_Z,    KEY_ZERO, KEY_ONE,
         KEY_TWO,   KEY_THREE,  KEY_FOUR,  KEY_FIVE, KEY_SIX,  KEY_SEVEN,
         KEY_EIGHT, KEY_NINE};

  for (int key : kKeys) {
    auto mapped = mapKey(key);
    if (!mapped) {
      continue;
    }

    if (IsKeyPressed(key)) {
      events.push_back(KeyEvent{*mapped, true});
    }

    if (IsKeyReleased(key)) {
      events.push_back(KeyEvent{*mapped, false});
    }
  }

  const Vector2 mouseDelta = GetMouseDelta();
  if (mouseDelta.x != 0.0f || mouseDelta.y != 0.0f) {
    const Vector2 mousePos = GetMousePosition();
    events.push_back(
        MouseMoveEvent{mousePos.x, mousePos.y, mouseDelta.x, mouseDelta.y});
  }

  static const std::array<int, 3> kButtons
      = {MOUSE_BUTTON_LEFT, MOUSE_BUTTON_RIGHT, MOUSE_BUTTON_MIDDLE};
  for (int button : kButtons) {
    auto mapped = mapMouseButton(button);
    if (!mapped) {
      continue;
    }

    if (IsMouseButtonPressed(button)) {
      const Vector2 mousePos = GetMousePosition();
      events.push_back(MouseButtonEvent{*mapped, true, mousePos.x, mousePos.y});
    }

    if (IsMouseButtonReleased(button)) {
      const Vector2 mousePos = GetMousePosition();
      events.push_back(
          MouseButtonEvent{*mapped, false, mousePos.x, mousePos.y});
    }
  }

  const Vector2 wheelMove = GetMouseWheelMoveV();
  if (wheelMove.x != 0.0f || wheelMove.y != 0.0f) {
    events.push_back(ScrollEvent{wheelMove.x, wheelMove.y});
  }

  return events;
}

namespace {

BoundingBox computeAABB(
    const SceneNode& node, const Eigen::Isometry3d& transform)
{
  Eigen::Vector3d halfExtent = Eigen::Vector3d::Zero();

  std::visit(
      [&halfExtent](const auto& shape) {
        using ShapeT = std::decay_t<decltype(shape)>;
        if constexpr (std::is_same_v<ShapeT, BoxData>) {
          halfExtent = shape.size * 0.5;
        } else if constexpr (std::is_same_v<ShapeT, SphereData>) {
          halfExtent.setConstant(shape.radius);
        } else if constexpr (std::is_same_v<ShapeT, CylinderData>) {
          halfExtent
              = Eigen::Vector3d(shape.radius, shape.radius, shape.height * 0.5);
        } else if constexpr (std::is_same_v<ShapeT, CapsuleData>) {
          const double r = shape.radius;
          const double h = shape.height * 0.5 + r;
          halfExtent = Eigen::Vector3d(r, r, h);
        } else if constexpr (std::is_same_v<ShapeT, ConeData>) {
          halfExtent
              = Eigen::Vector3d(shape.radius, shape.radius, shape.height * 0.5);
        } else if constexpr (std::is_same_v<ShapeT, EllipsoidData>) {
          halfExtent = shape.radii;
        } else if constexpr (std::is_same_v<ShapeT, MeshData>) {
          Eigen::Vector3f minV
              = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
          Eigen::Vector3f maxV
              = Eigen::Vector3f::Constant(std::numeric_limits<float>::lowest());
          for (const auto& v : shape.vertices) {
            minV = minV.cwiseMin(v);
            maxV = maxV.cwiseMax(v);
          }
          halfExtent = ((maxV - minV) * 0.5f).cast<double>();
        }
        // PlaneData and LineData: skip picking
      },
      node.shape);

  if (halfExtent.isZero()) {
    return BoundingBox{Vector3{0.0f, 0.0f, 0.0f}, Vector3{0.0f, 0.0f, 0.0f}};
  }

  const Eigen::Vector3d center = transform.translation();
  const Eigen::Matrix3d rot = transform.rotation();

  // Compute axis-aligned extent from oriented bounding box
  Eigen::Vector3d aaExtent = Eigen::Vector3d::Zero();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      aaExtent[i] += std::abs(rot(i, j)) * halfExtent[j];
    }
  }

  const Eigen::Vector3d minCorner = center - aaExtent;
  const Eigen::Vector3d maxCorner = center + aaExtent;
  return BoundingBox{toVector3(minCorner), toVector3(maxCorner)};
}

} // namespace

std::optional<HitResult> RaylibBackend::pickNode(
    const Scene& scene, float screen_x, float screen_y)
{
  if (!initialized_) {
    return std::nullopt;
  }

  const Camera3D camera = toRaylibCamera(scene.camera);
  const Ray ray = GetScreenToWorldRay(Vector2{screen_x, screen_y}, camera);

  std::optional<HitResult> nearest;
  double nearestDist = std::numeric_limits<double>::max();

  for (const auto& node : scene.nodes) {
    if (!node.visible) {
      continue;
    }

    const BoundingBox aabb = computeAABB(node, node.transform);

    // Skip zero-size AABBs (planes, lines, etc.)
    if (aabb.min.x == aabb.max.x && aabb.min.y == aabb.max.y
        && aabb.min.z == aabb.max.z) {
      continue;
    }

    const RayCollision collision = GetRayCollisionBox(ray, aabb);
    if (collision.hit) {
      const double dist = static_cast<double>(collision.distance);
      if (dist < nearestDist) {
        nearestDist = dist;
        HitResult hit;
        hit.node_id = node.id;
        hit.distance = dist;
        hit.point = Eigen::Vector3d(
            collision.point.x, collision.point.y, collision.point.z);
        hit.normal = Eigen::Vector3d(
            collision.normal.x, collision.normal.y, collision.normal.z);
        nearest = hit;
      }
    }
  }

  return nearest;
}

void RaylibBackend::captureScreenshot(const std::string& filename)
{
  pending_screenshot_ = filename;
}

} // namespace gui
} // namespace dart
