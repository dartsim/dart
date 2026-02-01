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

// clang-format off
constexpr const char* kLightingVS =
    "#version 330\n"
    "in vec3 vertexPosition;\n"
    "in vec2 vertexTexcoord;\n"
    "in vec3 vertexNormal;\n"
    "in vec4 vertexColor;\n"
    "uniform mat4 mvp;\n"
    "uniform mat4 matModel;\n"
    "uniform mat4 matNormal;\n"
    "out vec3 fragPosition;\n"
    "out vec2 fragTexCoord;\n"
    "out vec3 fragNormal;\n"
    "void main() {\n"
    "    fragPosition = vec3(matModel * vec4(vertexPosition, 1.0));\n"
    "    fragTexCoord = vertexTexcoord;\n"
    "    fragNormal = normalize(vec3(matNormal * vec4(vertexNormal, 0.0)));\n"
    "    gl_Position = mvp * vec4(vertexPosition, 1.0);\n"
    "}\n";

constexpr const char* kLightingFS =
    "#version 330\n"
    "in vec3 fragPosition;\n"
    "in vec2 fragTexCoord;\n"
    "in vec3 fragNormal;\n"
    "uniform sampler2D texture0;\n"
    "uniform vec4 colDiffuse;\n"
    "uniform vec3 viewPos;\n"
    "uniform vec4 ambientColor;\n"
    "uniform vec4 specularColor;\n"
    "uniform float shininess;\n"
    "uniform int numLights;\n"
    "uniform vec3 lightDirs[4];\n"
    "uniform vec4 lightColors[4];\n"
    "out vec4 finalColor;\n"
    "void main() {\n"
    "    vec4 texelColor = texture(texture0, fragTexCoord);\n"
    "    vec3 normal = normalize(fragNormal);\n"
    "    vec3 viewDir = normalize(viewPos - fragPosition);\n"
    "    vec3 ambient = ambientColor.rgb;\n"
    "    vec3 diffuse = vec3(0.0);\n"
    "    vec3 specular = vec3(0.0);\n"
    "    for (int i = 0; i < numLights && i < 4; i++) {\n"
    "        vec3 ld = normalize(-lightDirs[i]);\n"
    "        float NdotL = max(dot(normal, ld), 0.0);\n"
    "        diffuse += lightColors[i].rgb * NdotL;\n"
    "        if (NdotL > 0.0) {\n"
    "            vec3 halfDir = normalize(ld + viewDir);\n"
    "            float spec = pow(max(dot(normal, halfDir), 0.0), shininess);\n"
    "            specular += specularColor.rgb * spec * lightColors[i].rgb;\n"
    "        }\n"
    "    }\n"
    "    vec3 lighting = ambient + diffuse;\n"
    "    finalColor = vec4(texelColor.rgb * colDiffuse.rgb * lighting + specular,"
    "                      colDiffuse.a * texelColor.a);\n"
    "}\n";
// clang-format on

constexpr int kMaxLights = 4;

struct LightingState
{
  Shader shader{};
  bool loaded = false;
  int loc_view_pos = -1;
  int loc_ambient = -1;
  int loc_specular = -1;
  int loc_shininess = -1;
  int loc_num_lights = -1;
  int loc_light_dirs[kMaxLights] = {-1, -1, -1, -1};
  int loc_light_colors[kMaxLights] = {-1, -1, -1, -1};
};

struct MeshCache
{
  Mesh unit_cube{};
  Mesh unit_sphere{};
  Mesh unit_cylinder{};
  Mesh unit_cone{};
  Mesh unit_hemisphere{};
  Mesh unit_plane{};
  Mesh unit_pyramid{};
  bool loaded = false;
};

static LightingState g_lighting;
static MeshCache g_meshes;

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

Matrix eigenMatToRaylib(const Eigen::Matrix4d& mat)
{
  Eigen::Matrix4f matf = mat.cast<float>();
  Matrix result;
  std::memcpy(&result, matf.data(), sizeof(Matrix));
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

Mesh genPyramidMesh()
{
  constexpr int kTriCount = 6;
  constexpr int kVertCount = kTriCount * 3;

  Mesh mesh{};
  mesh.vertexCount = kVertCount;
  mesh.triangleCount = kTriCount;
  mesh.vertices = static_cast<float*>(
      MemAlloc(static_cast<int>(kVertCount * 3 * sizeof(float))));
  mesh.normals = static_cast<float*>(
      MemAlloc(static_cast<int>(kVertCount * 3 * sizeof(float))));
  mesh.texcoords = static_cast<float*>(
      MemAlloc(static_cast<int>(kVertCount * 2 * sizeof(float))));
  mesh.indices = static_cast<unsigned short*>(
      MemAlloc(static_cast<int>(kVertCount * sizeof(unsigned short))));

  const float hw = 0.5f;
  const float hd = 0.5f;
  const float hh = 0.5f;

  const float bx[4] = {-hw, hw, hw, -hw};
  const float by[4] = {-hd, -hd, hd, hd};
  const float bz = -hh;
  const float az = hh;

  int vi = 0;
  auto addVert = [&](float x, float y, float z, float nx, float ny, float nz) {
    mesh.vertices[vi * 3 + 0] = x;
    mesh.vertices[vi * 3 + 1] = y;
    mesh.vertices[vi * 3 + 2] = z;
    mesh.normals[vi * 3 + 0] = nx;
    mesh.normals[vi * 3 + 1] = ny;
    mesh.normals[vi * 3 + 2] = nz;
    mesh.texcoords[vi * 2 + 0] = 0.0f;
    mesh.texcoords[vi * 2 + 1] = 0.0f;
    mesh.indices[vi] = static_cast<unsigned short>(vi);
    ++vi;
  };

  const float sn = 1.0f / std::sqrt(1.25f);
  const float un = 0.5f / std::sqrt(1.25f);

  const float nx[4] = {0.0f, sn, 0.0f, -sn};
  const float ny[4] = {-sn, 0.0f, sn, 0.0f};

  for (int i = 0; i < 4; ++i) {
    const int j = (i + 1) % 4;
    addVert(bx[i], by[i], bz, nx[i], ny[i], un);
    addVert(bx[j], by[j], bz, nx[i], ny[i], un);
    addVert(0.0f, 0.0f, az, nx[i], ny[i], un);
  }

  addVert(bx[0], by[0], bz, 0.0f, 0.0f, -1.0f);
  addVert(bx[2], by[2], bz, 0.0f, 0.0f, -1.0f);
  addVert(bx[1], by[1], bz, 0.0f, 0.0f, -1.0f);
  addVert(bx[0], by[0], bz, 0.0f, 0.0f, -1.0f);
  addVert(bx[3], by[3], bz, 0.0f, 0.0f, -1.0f);
  addVert(bx[2], by[2], bz, 0.0f, 0.0f, -1.0f);

  UploadMesh(&mesh, false);
  return mesh;
}

void initLighting()
{
  g_lighting.shader = LoadShaderFromMemory(kLightingVS, kLightingFS);
  const int testLoc = GetShaderLocation(g_lighting.shader, "numLights");
  if (testLoc < 0) {
    DART_WARN("Lighting shader failed to compile; using flat rendering");
    g_lighting.loaded = false;
    return;
  }

  g_lighting.loaded = true;
  g_lighting.loc_view_pos = GetShaderLocation(g_lighting.shader, "viewPos");
  g_lighting.loc_ambient = GetShaderLocation(g_lighting.shader, "ambientColor");
  g_lighting.loc_specular
      = GetShaderLocation(g_lighting.shader, "specularColor");
  g_lighting.loc_shininess = GetShaderLocation(g_lighting.shader, "shininess");
  g_lighting.loc_num_lights = testLoc;

  for (int i = 0; i < kMaxLights; ++i) {
    g_lighting.loc_light_dirs[i]
        = GetShaderLocation(g_lighting.shader, TextFormat("lightDirs[%d]", i));
    g_lighting.loc_light_colors[i] = GetShaderLocation(
        g_lighting.shader, TextFormat("lightColors[%d]", i));
  }
}

void shutdownLighting()
{
  if (g_lighting.loaded) {
    UnloadShader(g_lighting.shader);
    g_lighting = LightingState{};
  }
}

void initMeshes()
{
  g_meshes.unit_cube = GenMeshCube(1.0f, 1.0f, 1.0f);
  g_meshes.unit_sphere = GenMeshSphere(1.0f, 16, 16);
  g_meshes.unit_cylinder = GenMeshCylinder(1.0f, 1.0f, 24);
  g_meshes.unit_cone = GenMeshCone(1.0f, 1.0f, 24);
  g_meshes.unit_hemisphere = GenMeshHemiSphere(1.0f, 16, 16);
  g_meshes.unit_plane = GenMeshPlane(1.0f, 1.0f, 1, 1);
  g_meshes.unit_pyramid = genPyramidMesh();
  g_meshes.loaded = true;
}

void shutdownMeshes()
{
  if (!g_meshes.loaded) {
    return;
  }
  UnloadMesh(g_meshes.unit_cube);
  UnloadMesh(g_meshes.unit_sphere);
  UnloadMesh(g_meshes.unit_cylinder);
  UnloadMesh(g_meshes.unit_cone);
  UnloadMesh(g_meshes.unit_hemisphere);
  UnloadMesh(g_meshes.unit_plane);
  UnloadMesh(g_meshes.unit_pyramid);
  g_meshes = MeshCache{};
}

void updateLightUniforms(const Scene& scene)
{
  if (!g_lighting.loaded) {
    return;
  }

  const float viewPos[3]
      = {static_cast<float>(scene.camera.position.x()),
         static_cast<float>(scene.camera.position.y()),
         static_cast<float>(scene.camera.position.z())};
  SetShaderValue(
      g_lighting.shader, g_lighting.loc_view_pos, viewPos, SHADER_UNIFORM_VEC3);

  int numLights = 0;

  if (scene.headlight) {
    Eigen::Vector3d dir = (scene.camera.target - scene.camera.position);
    if (dir.norm() > 1e-9) {
      dir.normalize();
    } else {
      dir = Eigen::Vector3d(0, 0, -1);
    }
    const float ld[3]
        = {static_cast<float>(dir.x()),
           static_cast<float>(dir.y()),
           static_cast<float>(dir.z())};
    const float lc[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    SetShaderValue(
        g_lighting.shader,
        g_lighting.loc_light_dirs[0],
        ld,
        SHADER_UNIFORM_VEC3);
    SetShaderValue(
        g_lighting.shader,
        g_lighting.loc_light_colors[0],
        lc,
        SHADER_UNIFORM_VEC4);
    numLights = 1;
  }

  for (const auto& light : scene.lights) {
    if (numLights >= kMaxLights) {
      break;
    }
    const float ld[3]
        = {static_cast<float>(light.direction.x()),
           static_cast<float>(light.direction.y()),
           static_cast<float>(light.direction.z())};
    const float lc[4]
        = {static_cast<float>(light.color[0]),
           static_cast<float>(light.color[1]),
           static_cast<float>(light.color[2]),
           static_cast<float>(light.color[3])};
    SetShaderValue(
        g_lighting.shader,
        g_lighting.loc_light_dirs[numLights],
        ld,
        SHADER_UNIFORM_VEC3);
    SetShaderValue(
        g_lighting.shader,
        g_lighting.loc_light_colors[numLights],
        lc,
        SHADER_UNIFORM_VEC4);
    ++numLights;
  }

  SetShaderValue(
      g_lighting.shader,
      g_lighting.loc_num_lights,
      &numLights,
      SHADER_UNIFORM_INT);
}

void setPerNodeUniforms(const dart::gui::Material& mat)
{
  if (!g_lighting.loaded) {
    return;
  }

  const float ambient[4]
      = {static_cast<float>(mat.ambient[0]),
         static_cast<float>(mat.ambient[1]),
         static_cast<float>(mat.ambient[2]),
         static_cast<float>(mat.ambient[3])};
  const float specular[4]
      = {static_cast<float>(mat.specular[0]),
         static_cast<float>(mat.specular[1]),
         static_cast<float>(mat.specular[2]),
         static_cast<float>(mat.specular[3])};
  const float shininess = static_cast<float>(mat.shininess);

  SetShaderValue(
      g_lighting.shader, g_lighting.loc_ambient, ambient, SHADER_UNIFORM_VEC4);
  SetShaderValue(
      g_lighting.shader,
      g_lighting.loc_specular,
      specular,
      SHADER_UNIFORM_VEC4);
  SetShaderValue(
      g_lighting.shader,
      g_lighting.loc_shininess,
      &shininess,
      SHADER_UNIFORM_FLOAT);
}

::Material makeLitMaterial(const Color& diffuse)
{
  ::Material mat = LoadMaterialDefault();
  if (g_lighting.loaded) {
    mat.shader = g_lighting.shader;
  }
  mat.maps[MATERIAL_MAP_DIFFUSE].color = diffuse;
  return mat;
}

void freeMaterial(::Material& mat)
{
  // Detach the shared lighting shader so UnloadMaterial won't free it.
  // Also detach texture references that belong to the cache.
  mat.shader = {0, nullptr};
  // Free the maps array allocated by LoadMaterialDefault().
  RL_FREE(mat.maps);
  mat.maps = nullptr;
}

void drawCachedMesh(
    Mesh mesh, const Color& diffuse, const Eigen::Matrix4d& transform)
{
  ::Material mat = makeLitMaterial(diffuse);
  DrawMesh(mesh, mat, eigenMatToRaylib(transform));
  freeMaterial(mat);
}

Eigen::Matrix3d rotMinusX90()
{
  Eigen::Matrix3d r;
  r << 1, 0, 0, 0, 0, 1, 0, -1, 0;
  return r;
}

void drawMeshDataLit(
    const MeshData& data,
    const Color& diffuse,
    const Eigen::Isometry3d& nodeTransform)
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

  ::Material mat = makeLitMaterial(diffuse);
  DrawMesh(mesh, mat, eigenToRaylib(nodeTransform));
  freeMaterial(mat);
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

  initLighting();
  initMeshes();

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

  updateLightUniforms(scene);

  const Eigen::Matrix3d rMinusX90 = rotMinusX90();

  for (const auto& node : scene.nodes) {
    if (!node.visible) {
      continue;
    }

    const Color color = toColor(node.material.color);
    setPerNodeUniforms(node.material);

    std::visit(
        [&](const auto& shape) {
          using ShapeT = std::decay_t<decltype(shape)>;
          if constexpr (std::is_same_v<ShapeT, BoxData>) {
            Eigen::Matrix4d m = node.transform.matrix();
            m.col(0) *= shape.size.x();
            m.col(1) *= shape.size.y();
            m.col(2) *= shape.size.z();
            drawCachedMesh(g_meshes.unit_cube, color, m);
          } else if constexpr (std::is_same_v<ShapeT, SphereData>) {
            Eigen::Matrix4d m = node.transform.matrix();
            const double r = shape.radius;
            m.col(0) *= r;
            m.col(1) *= r;
            m.col(2) *= r;
            drawCachedMesh(g_meshes.unit_sphere, color, m);
          } else if constexpr (std::is_same_v<ShapeT, CylinderData>) {
            Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
            m.block<3, 3>(0, 0) = node.transform.rotation() * rMinusX90;
            m.col(0).head<3>() *= shape.radius;
            m.col(1).head<3>() *= shape.height;
            m.col(2).head<3>() *= shape.radius;
            m.col(3).head<3>() = node.transform.translation();
            drawCachedMesh(g_meshes.unit_cylinder, color, m);
          } else if constexpr (std::is_same_v<ShapeT, CapsuleData>) {
            rlPushMatrix();
            const Matrix tf = eigenToRaylib(node.transform);
            rlMultMatrixf(reinterpret_cast<const float*>(&tf));
            rlRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
            const float halfH = static_cast<float>(shape.height) * 0.5f;
            DrawCapsule(
                Vector3{0.0f, -halfH, 0.0f},
                Vector3{0.0f, halfH, 0.0f},
                static_cast<float>(shape.radius),
                16,
                16,
                color);
            rlPopMatrix();
          } else if constexpr (std::is_same_v<ShapeT, ConeData>) {
            Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
            m.block<3, 3>(0, 0) = node.transform.rotation() * rMinusX90;
            m.col(0).head<3>() *= shape.radius;
            m.col(1).head<3>() *= shape.height;
            m.col(2).head<3>() *= shape.radius;
            m.col(3).head<3>() = node.transform.translation();
            drawCachedMesh(g_meshes.unit_cone, color, m);
          } else if constexpr (std::is_same_v<ShapeT, EllipsoidData>) {
            Eigen::Matrix4d m = node.transform.matrix();
            m.col(0) *= shape.radii.x();
            m.col(1) *= shape.radii.y();
            m.col(2) *= shape.radii.z();
            drawCachedMesh(g_meshes.unit_sphere, color, m);
          } else if constexpr (std::is_same_v<ShapeT, PlaneData>) {
            Eigen::Vector3d normal = shape.normal;
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
            Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
            m.block<3, 1>(0, 0) = tangent * 20.0;
            m.block<3, 1>(0, 1) = normal;
            m.block<3, 1>(0, 2) = bitangent * 20.0;
            m.block<3, 1>(0, 3) = normal * shape.offset;
            drawCachedMesh(g_meshes.unit_plane, color, m);
          } else if constexpr (std::is_same_v<ShapeT, MeshData>) {
            drawMeshDataLit(shape, color, node.transform);
          } else if constexpr (std::is_same_v<ShapeT, LineData>) {
            rlPushMatrix();
            const Matrix tf = eigenToRaylib(node.transform);
            rlMultMatrixf(reinterpret_cast<const float*>(&tf));
            for (const auto& segment : shape.segments) {
              DrawLine3D(
                  toVector3(segment.first), toVector3(segment.second), color);
            }
            rlPopMatrix();
          } else if constexpr (std::is_same_v<ShapeT, PyramidData>) {
            Eigen::Matrix4d m = node.transform.matrix();
            m.col(0) *= shape.base_width;
            m.col(1) *= shape.base_depth;
            m.col(2) *= shape.height;
            drawCachedMesh(g_meshes.unit_pyramid, color, m);
          }
        },
        node.shape);
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
            } else if constexpr (std::is_same_v<ShapeT, PyramidData>) {
              const float hw = static_cast<float>(shape.base_width) * 0.5f;
              const float hd = static_cast<float>(shape.base_depth) * 0.5f;
              const float hh = static_cast<float>(shape.height) * 0.5f;
              const Vector3 b0{-hw, -hd, -hh};
              const Vector3 b1{hw, -hd, -hh};
              const Vector3 b2{hw, hd, -hh};
              const Vector3 b3{-hw, hd, -hh};
              const Vector3 apex{0, 0, hh};
              DrawLine3D(b0, b1, YELLOW);
              DrawLine3D(b1, b2, YELLOW);
              DrawLine3D(b2, b3, YELLOW);
              DrawLine3D(b3, b0, YELLOW);
              DrawLine3D(b0, apex, YELLOW);
              DrawLine3D(b1, apex, YELLOW);
              DrawLine3D(b2, apex, YELLOW);
              DrawLine3D(b3, apex, YELLOW);
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

  shutdownMeshes();
  shutdownLighting();
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
        } else if constexpr (std::is_same_v<ShapeT, PyramidData>) {
          halfExtent = Eigen::Vector3d(
              shape.base_width * 0.5,
              shape.base_depth * 0.5,
              shape.height * 0.5);
        }
      },
      node.shape);

  if (halfExtent.isZero()) {
    return BoundingBox{Vector3{0.0f, 0.0f, 0.0f}, Vector3{0.0f, 0.0f, 0.0f}};
  }

  const Eigen::Vector3d center = transform.translation();
  const Eigen::Matrix3d rot = transform.rotation();

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
