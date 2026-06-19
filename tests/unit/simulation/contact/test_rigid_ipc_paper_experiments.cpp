/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License
 */

// Paper-parity experiments for the rigid-body implicit-barrier (rigid IPC)
// contact solver tracked by PLAN-082. Each test reproduces the qualitative
// behavior of a figure from the rigid IPC paper (Ferguson et al.,
// "Intersection-free Rigid Body Dynamics", SIGGRAPH 2021,
// https://ipc-sim.github.io/rigid-ipc/) that the current DART-owned rigid IPC
// runtime stage can express with free dynamic and static rigid bodies plus
// lagged Coulomb friction. Scenes that require articulation (chains, hinges,
// gears) or scripted kinematic drivers are out of scope here and tracked
// separately.
//
// The two invariants every rigid IPC scene must hold are encoded as assertions:
//   1. Intersection-free: no body ever penetrates another (the barrier and the
//      conservative CCD line search guarantee this), and the solve never fails.
//   2. Coulomb friction threshold: a body slides iff the down-slope pull
//      exceeds the friction cone, i.e. tan(theta) > mu (Fig. 18).
//
// Friction note: the rigid IPC stage combines per-body friction coefficients as
// the geometric mean (effective mu = sqrt(mu_a * mu_b)). Setting BOTH bodies in
// a contact to the same value `mu` therefore yields an effective coefficient of
// exactly `mu`, which is what these tests rely on.

#include <dart/simulation/body/collision_shape.hpp>
#include <dart/simulation/body/contact.hpp>
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/body/rigid_body_options.hpp>
#include <dart/simulation/compute/detail/world_step_stages.hpp>
#include <dart/simulation/compute/sequential_executor.hpp>
#include <dart/simulation/world.hpp>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <array>
#include <limits>
#include <numbers>
#include <string>
#include <vector>

#include <cmath>

namespace {

namespace sx = dart::simulation;

// Build a triangulated thin cylinder ("coin"/"disk") collision mesh centered at
// the body origin, with its symmetry axis along local +z. `segments` rim
// samples produce a closed surface: two cap fans plus a quad side wall.
sx::CollisionShape makeDiskMesh(double radius, double halfHeight, int segments)
{
  std::vector<Eigen::Vector3d> vertices;
  std::vector<Eigen::Vector3i> triangles;
  vertices.reserve(static_cast<std::size_t>(2 * segments + 2));
  triangles.reserve(static_cast<std::size_t>(4 * segments));

  const int topCenter = 0;
  const int bottomCenter = 1;
  vertices.emplace_back(0.0, 0.0, halfHeight);
  vertices.emplace_back(0.0, 0.0, -halfHeight);

  const int topRing = 2;
  const int bottomRing = 2 + segments;
  for (int i = 0; i < segments; ++i) {
    const double angle
        = 2.0 * std::numbers::pi * static_cast<double>(i) / segments;
    vertices.emplace_back(
        radius * std::cos(angle), radius * std::sin(angle), halfHeight);
  }
  for (int i = 0; i < segments; ++i) {
    const double angle
        = 2.0 * std::numbers::pi * static_cast<double>(i) / segments;
    vertices.emplace_back(
        radius * std::cos(angle), radius * std::sin(angle), -halfHeight);
  }

  for (int i = 0; i < segments; ++i) {
    const int n = (i + 1) % segments;
    // Top cap (outward normal +z).
    triangles.emplace_back(topCenter, topRing + i, topRing + n);
    // Bottom cap (outward normal -z).
    triangles.emplace_back(bottomCenter, bottomRing + n, bottomRing + i);
    // Side wall (two triangles per quad).
    triangles.emplace_back(topRing + i, bottomRing + i, bottomRing + n);
    triangles.emplace_back(topRing + i, bottomRing + n, topRing + n);
  }

  return sx::CollisionShape::makeMesh(
      std::move(vertices), std::move(triangles));
}

sx::CollisionShape makeConeMesh(double radius, double halfHeight, int segments)
{
  std::vector<Eigen::Vector3d> vertices;
  std::vector<Eigen::Vector3i> triangles;
  vertices.reserve(static_cast<std::size_t>(segments + 2));
  triangles.reserve(static_cast<std::size_t>(2 * segments));

  const int apex = 0;
  const int baseCenter = 1;
  vertices.emplace_back(0.0, 0.0, halfHeight);
  vertices.emplace_back(0.0, 0.0, -halfHeight);

  const int baseRing = 2;
  for (int i = 0; i < segments; ++i) {
    const double angle
        = 2.0 * std::numbers::pi * static_cast<double>(i) / segments;
    vertices.emplace_back(
        radius * std::cos(angle), radius * std::sin(angle), -halfHeight);
  }

  for (int i = 0; i < segments; ++i) {
    const int n = (i + 1) % segments;
    triangles.emplace_back(apex, baseRing + i, baseRing + n);
    triangles.emplace_back(baseCenter, baseRing + n, baseRing + i);
  }

  return sx::CollisionShape::makeMesh(
      std::move(vertices), std::move(triangles));
}

sx::CollisionShape makeEllipsoidMesh(
    const Eigen::Vector3d& radii, int stacks, int slices)
{
  std::vector<Eigen::Vector3d> vertices;
  std::vector<Eigen::Vector3i> triangles;
  vertices.reserve(static_cast<std::size_t>((stacks - 1) * slices + 2));
  triangles.reserve(static_cast<std::size_t>(2 * slices * (stacks - 1)));

  const int top = 0;
  vertices.emplace_back(0.0, 0.0, radii.z());
  for (int i = 1; i < stacks; ++i) {
    const double theta = std::numbers::pi * static_cast<double>(i) / stacks;
    const double ringRadius = std::sin(theta);
    const double z = std::cos(theta);
    for (int j = 0; j < slices; ++j) {
      const double phi
          = 2.0 * std::numbers::pi * static_cast<double>(j) / slices;
      vertices.emplace_back(
          radii.x() * ringRadius * std::cos(phi),
          radii.y() * ringRadius * std::sin(phi),
          radii.z() * z);
    }
  }
  const int bottom = static_cast<int>(vertices.size());
  vertices.emplace_back(0.0, 0.0, -radii.z());

  auto ringVertex = [slices](int ring, int index) {
    return 1 + ring * slices + (index % slices);
  };

  for (int j = 0; j < slices; ++j) {
    const int n = (j + 1) % slices;
    triangles.emplace_back(top, ringVertex(0, j), ringVertex(0, n));
  }

  for (int ring = 0; ring < stacks - 2; ++ring) {
    for (int j = 0; j < slices; ++j) {
      const int n = (j + 1) % slices;
      const int a = ringVertex(ring, j);
      const int b = ringVertex(ring, n);
      const int c = ringVertex(ring + 1, j);
      const int d = ringVertex(ring + 1, n);
      triangles.emplace_back(a, c, d);
      triangles.emplace_back(a, d, b);
    }
  }

  const int lastRing = stacks - 2;
  for (int j = 0; j < slices; ++j) {
    const int n = (j + 1) % slices;
    triangles.emplace_back(
        bottom, ringVertex(lastRing, n), ringVertex(lastRing, j));
  }

  return sx::CollisionShape::makeMesh(
      std::move(vertices), std::move(triangles));
}

void appendBoxMesh(
    std::vector<Eigen::Vector3d>& vertices,
    std::vector<Eigen::Vector3i>& triangles,
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& halfExtents)
{
  const int base = static_cast<int>(vertices.size());
  for (double dx : {-halfExtents.x(), halfExtents.x()}) {
    for (double dy : {-halfExtents.y(), halfExtents.y()}) {
      for (double dz : {-halfExtents.z(), halfExtents.z()}) {
        vertices.push_back(center + Eigen::Vector3d(dx, dy, dz));
      }
    }
  }

  const std::vector<Eigen::Vector3i> boxTriangles = {
      {0, 2, 3},
      {0, 3, 1},
      {4, 5, 7},
      {4, 7, 6},
      {0, 1, 5},
      {0, 5, 4},
      {2, 6, 7},
      {2, 7, 3},
      {0, 4, 6},
      {0, 6, 2},
      {1, 3, 7},
      {1, 7, 5},
  };
  for (const Eigen::Vector3i& t : boxTriangles) {
    triangles.emplace_back(base + t.x(), base + t.y(), base + t.z());
  }
}

sx::CollisionShape makeWingNutLikeMesh()
{
  std::vector<Eigen::Vector3d> vertices;
  std::vector<Eigen::Vector3i> triangles;
  vertices.reserve(24);
  triangles.reserve(36);

  appendBoxMesh(
      vertices, triangles, Eigen::Vector3d::Zero(), {0.18, 0.18, 0.12});
  appendBoxMesh(
      vertices,
      triangles,
      Eigen::Vector3d(-0.65, 0.0, 0.0),
      {0.47, 0.14, 0.06});
  appendBoxMesh(
      vertices, triangles, Eigen::Vector3d(0.65, 0.0, 0.0), {0.47, 0.14, 0.06});

  return sx::CollisionShape::makeMesh(
      std::move(vertices), std::move(triangles));
}

sx::CollisionShape makeTwoTrianglePlaneMesh(double halfExtent)
{
  std::vector<Eigen::Vector3d> vertices = {
      {-halfExtent, -halfExtent, 0.0},
      {halfExtent, -halfExtent, 0.0},
      {-halfExtent, halfExtent, 0.0},
      {halfExtent, halfExtent, 0.0},
  };
  std::vector<Eigen::Vector3i> triangles = {{1, 2, 0}, {1, 3, 2}};
  return sx::CollisionShape::makeMesh(
      std::move(vertices), std::move(triangles));
}

sx::CollisionShape makeTessellatedPlaneMesh(double halfExtent, int cellsPerAxis)
{
  std::vector<Eigen::Vector3d> vertices;
  vertices.reserve((cellsPerAxis + 1) * (cellsPerAxis + 1));

  const double step = 2.0 * halfExtent / static_cast<double>(cellsPerAxis);
  for (int y = 0; y <= cellsPerAxis; ++y) {
    const double py = -halfExtent + step * static_cast<double>(y);
    for (int x = 0; x <= cellsPerAxis; ++x) {
      const double px = -halfExtent + step * static_cast<double>(x);
      vertices.emplace_back(px, py, 0.0);
    }
  }

  std::vector<Eigen::Vector3i> triangles;
  triangles.reserve(cellsPerAxis * cellsPerAxis * 2);
  const auto index = [cellsPerAxis](int x, int y) {
    return y * (cellsPerAxis + 1) + x;
  };
  for (int y = 0; y < cellsPerAxis; ++y) {
    for (int x = 0; x < cellsPerAxis; ++x) {
      const int lowerLeft = index(x, y);
      const int lowerRight = index(x + 1, y);
      const int upperLeft = index(x, y + 1);
      const int upperRight = index(x + 1, y + 1);
      triangles.emplace_back(lowerRight, upperLeft, lowerLeft);
      triangles.emplace_back(lowerRight, upperRight, upperLeft);
    }
  }

  return sx::CollisionShape::makeMesh(
      std::move(vertices), std::move(triangles));
}

sx::CollisionShape makeTetPyramidMesh()
{
  std::vector<Eigen::Vector3d> vertices = {
      {-0.5, 0.288675, -0.176777},
      {0.0, 0.0, 0.530330},
      {0.5, 0.288675, -0.176777},
      {0.0, -0.577350, -0.176777},
  };
  std::vector<Eigen::Vector3i> triangles
      = {{2, 3, 1}, {2, 1, 0}, {3, 0, 1}, {2, 0, 3}};
  return sx::CollisionShape::makeMesh(
      std::move(vertices), std::move(triangles));
}

sx::CollisionShape makeTetCornerMesh(double scale = 1.0)
{
  std::vector<Eigen::Vector3d> vertices = {
      scale * Eigen::Vector3d(0.0, 0.0, 0.0),
      scale * Eigen::Vector3d(1.0, 0.0, 0.0),
      scale * Eigen::Vector3d(0.0, 0.0, 1.0),
      scale * Eigen::Vector3d(0.0, 1.0, 0.0),
  };
  std::vector<Eigen::Vector3i> triangles
      = {{0, 2, 1}, {0, 3, 2}, {1, 2, 3}, {0, 1, 3}};
  return sx::CollisionShape::makeMesh(
      std::move(vertices), std::move(triangles));
}

sx::CollisionShape makeErlebenCliffMesh()
{
  std::vector<Eigen::Vector3d> vertices = {
      {-0.75, 0.75, -0.5},
      {-0.5, 0.5, 0.5},
      {-0.75, -0.75, -0.5},
      {-0.5, -0.5, 0.5},
      {0.75, 0.75, -0.5},
      {0.5, 0.5, 0.5},
      {0.75, -0.75, -0.5},
      {0.5, -0.5, 0.5},
  };
  std::vector<Eigen::Vector3i> triangles
      = {{1, 2, 0},
         {3, 6, 2},
         {7, 4, 6},
         {5, 0, 4},
         {6, 0, 2},
         {3, 5, 7},
         {1, 3, 2},
         {3, 7, 6},
         {7, 5, 4},
         {5, 1, 0},
         {6, 4, 0},
         {3, 1, 5}};
  return sx::CollisionShape::makeMesh(
      std::move(vertices), std::move(triangles));
}

sx::CollisionShape makeErlebenInternalEdgesMesh()
{
  std::vector<Eigen::Vector3d> vertices = {
      {-1.0, 1.0, -0.5},
      {-1.0, 1.0, 0.5},
      {-1.0, -1.0, -0.5},
      {-1.0, -1.0, 0.5},
      {1.0, 1.0, -0.5},
      {1.0, 1.0, 0.5},
      {1.0, -1.0, -0.5},
      {1.0, -1.0, 0.5},
      {-1.0, 0.0, 0.5},
      {0.0, -1.0, 0.5},
      {1.0, 0.0, 0.5},
      {0.0, 1.0, 0.5},
      {-0.5, 0.5, 0.5},
      {0.5, 0.5, 0.5},
  };
  std::vector<Eigen::Vector3i> triangles
      = {{8, 3, 2},   {9, 7, 6},   {10, 5, 4},  {11, 1, 0},  {6, 0, 2},
         {1, 11, 12}, {11, 5, 13}, {3, 8, 9},   {7, 9, 10},  {5, 10, 13},
         {12, 8, 1},  {10, 9, 13}, {13, 9, 11}, {12, 11, 9}, {8, 12, 9},
         {2, 0, 8},   {0, 1, 8},   {6, 2, 9},   {2, 3, 9},   {4, 6, 10},
         {6, 7, 10},  {0, 4, 11},  {4, 5, 11},  {6, 4, 0}};
  return sx::CollisionShape::makeMesh(
      std::move(vertices), std::move(triangles));
}

sx::CollisionShape makeErlebenSpikeMesh()
{
  std::vector<Eigen::Vector3d> vertices = {
      {0.5, -0.5, 0.0},
      {0.5, 0.5, 0.0},
      {-0.5, 0.5, 0.0},
      {-0.5, -0.5, 0.0},
      {0.0, 0.0, 3.0},
  };
  std::vector<Eigen::Vector3i> triangles
      = {{0, 4, 1}, {1, 4, 2}, {2, 4, 3}, {3, 4, 0}, {1, 3, 0}, {1, 2, 3}};
  return sx::CollisionShape::makeMesh(
      std::move(vertices), std::move(triangles));
}

sx::CollisionShape makeErlebenWedgeMesh()
{
  std::vector<Eigen::Vector3d> vertices = {
      {-0.5, 1.0, 0.0},
      {0.0, 1.0, 3.0},
      {-0.5, -1.0, 0.0},
      {0.0, -1.0, 3.0},
      {0.5, 1.0, 0.0},
      {0.5, -1.0, 0.0},
  };
  std::vector<Eigen::Vector3i> triangles
      = {{1, 2, 0},
         {1, 5, 3},
         {5, 0, 2},
         {1, 0, 4},
         {2, 3, 5},
         {1, 3, 2},
         {1, 4, 5},
         {5, 4, 0}};
  return sx::CollisionShape::makeMesh(
      std::move(vertices), std::move(triangles));
}

sx::CollisionShape makeErlebenCrackMesh()
{
  std::vector<Eigen::Vector3d> vertices = {
      {-1.0, -2.0, -0.5}, {-0.5, -2.0, -0.5}, {0.0, -2.0, -0.5},
      {0.5, -2.0, -0.5},  {1.0, -2.0, -0.5},  {-1.0, -2.0, 0.5},
      {-0.5, -2.0, 0.5},  {0.0, -2.0, 0.0},   {0.5, -2.0, 0.5},
      {1.0, -2.0, 0.5},   {-1.0, 2.0, 0.5},   {-1.0, 2.0, -0.5},
      {-0.5, 2.0, -0.5},  {-0.5, 2.0, 0.5},   {0.0, 2.0, -0.5},
      {0.0, 2.0, 0.0},    {0.5, 2.0, 0.5},    {0.5, 2.0, -0.5},
      {1.0, 2.0, -0.5},   {1.0, 2.0, 0.5},
  };
  std::vector<Eigen::Vector3i> triangles
      = {{0, 5, 6},    {3, 7, 8},    {7, 1, 6},    {4, 3, 8},    {2, 7, 3},
         {7, 2, 1},    {4, 8, 9},    {0, 6, 1},    {11, 13, 10}, {17, 16, 15},
         {15, 13, 12}, {18, 16, 17}, {14, 17, 15}, {15, 12, 14}, {18, 19, 16},
         {11, 12, 13}, {9, 18, 4},   {1, 11, 0},   {8, 19, 9},   {7, 6, 13},
         {8, 7, 15},   {5, 13, 6},   {4, 17, 3},   {0, 10, 5},   {2, 3, 17},
         {1, 2, 14},   {9, 19, 18},  {1, 12, 11},  {8, 16, 19},  {7, 13, 15},
         {8, 15, 16},  {5, 10, 13},  {4, 18, 17},  {0, 11, 10},  {2, 17, 14},
         {1, 14, 12}};
  return sx::CollisionShape::makeMesh(
      std::move(vertices), std::move(triangles));
}

sx::CollisionShape makeErlebenHoleMesh()
{
  std::vector<Eigen::Vector3d> vertices = {
      {-1.0, -0.5, -0.5}, {-1.0, -0.5, 0.5},  {-1.0, -1.0, -0.5},
      {-1.0, -1.0, 0.5},  {-0.5, -0.5, -0.5}, {-0.5, -0.5, 0.5},
      {-0.5, -1.0, -0.5}, {-0.5, -1.0, 0.5},  {-1.0, 0.5, -0.5},
      {-1.0, 0.5, 0.5},   {-0.5, 0.5, 0.5},   {-0.5, 0.5, -0.5},
      {-1.0, 1.0, -0.5},  {-1.0, 1.0, 0.5},   {-0.5, 1.0, 0.5},
      {-0.5, 1.0, -0.5},  {0.5, 0.5, 0.5},    {0.5, 0.5, -0.5},
      {0.5, 1.0, 0.5},    {0.5, 1.0, -0.5},   {1.0, 0.5, 0.5},
      {1.0, 0.5, -0.5},   {1.0, 1.0, 0.5},    {1.0, 1.0, -0.5},
      {0.5, -0.5, 0.5},   {0.5, -0.5, -0.5},  {1.0, -0.5, 0.5},
      {1.0, -0.5, -0.5},  {0.5, -1.0, 0.5},   {0.5, -1.0, -0.5},
      {1.0, -1.0, 0.5},   {1.0, -1.0, -0.5},  {1.0, -0.0, -0.5},
      {1.0, -0.0, 0.5},   {-1.0, -0.0, -0.5}, {-1.0, -0.0, 0.5},
      {-0.0, 1.0, 0.5},   {-0.0, 1.0, -0.5},  {0.0, -1.0, 0.5},
      {0.0, -1.0, -0.5},  {0.0, 0.0, 0.0},
  };
  std::vector<Eigen::Vector3i> triangles
      = {{1, 2, 0},    {2, 3, 7},    {34, 9, 35},  {2, 4, 0},    {3, 1, 7},
         {11, 12, 8},  {39, 25, 4},  {14, 12, 15}, {8, 12, 9},   {13, 14, 10},
         {17, 23, 19}, {37, 18, 36}, {11, 8, 34},  {20, 23, 21}, {21, 32, 20},
         {18, 20, 16}, {19, 23, 18}, {30, 28, 24}, {16, 10, 36}, {25, 27, 32},
         {28, 31, 29}, {27, 33, 32}, {27, 29, 31}, {17, 19, 37}, {11, 25, 17},
         {27, 31, 26}, {7, 39, 6},   {15, 37, 14}, {0, 35, 1},   {1, 3, 2},
         {6, 2, 7},    {34, 8, 9},   {2, 6, 4},    {7, 1, 5},    {11, 15, 12},
         {4, 6, 39},   {39, 29, 25}, {35, 9, 10},  {14, 13, 12}, {12, 13, 9},
         {10, 9, 13},  {36, 18, 16}, {35, 10, 5},  {17, 21, 23}, {37, 19, 18},
         {0, 4, 34},   {4, 11, 34},  {20, 22, 23}, {33, 20, 32}, {18, 22, 20},
         {24, 26, 30}, {38, 5, 24},  {16, 33, 24}, {33, 26, 24}, {20, 33, 16},
         {32, 21, 17}, {17, 25, 32}, {28, 30, 31}, {27, 26, 33}, {27, 25, 29},
         {37, 15, 11}, {11, 17, 37}, {5, 38, 7},   {28, 38, 24}, {1, 35, 5},
         {11, 4, 25},  {10, 14, 36}, {30, 26, 31}, {7, 38, 39},  {36, 14, 37},
         {0, 34, 35},  {29, 39, 28}, {28, 39, 38}, {24, 5, 40},  {24, 40, 16},
         {10, 16, 40}, {5, 10, 40},  {22, 18, 23}};
  return sx::CollisionShape::makeMesh(
      std::move(vertices), std::move(triangles));
}

// Build one arch voussoir (wedge block) spanning the angular range
// [theta0, theta1] in the world x-z plane, between inner and outer radii, with
// half-width halfW along y. Writes the world centroid (the body position) into
// `center` and returns the body-frame collision mesh.
sx::CollisionShape makeVoussoirMesh(
    double theta0,
    double theta1,
    double innerRadius,
    double rOut,
    double halfW,
    Eigen::Vector3d& center)
{
  std::array<Eigen::Vector3d, 8> world;
  int k = 0;
  for (double theta : {theta0, theta1}) {
    for (double r : {innerRadius, rOut}) {
      for (double y : {-halfW, halfW}) {
        world[static_cast<std::size_t>(k++)]
            = Eigen::Vector3d(r * std::cos(theta), y, r * std::sin(theta));
      }
    }
  }
  center = Eigen::Vector3d::Zero();
  for (const auto& v : world) {
    center += v;
  }
  center /= 8.0;
  std::vector<Eigen::Vector3d> vertices;
  vertices.reserve(8);
  for (const auto& v : world) {
    vertices.push_back(v - center);
  }
  const std::vector<Eigen::Vector3i> triangles
      = {{0, 1, 3},
         {0, 3, 2},
         {4, 6, 7},
         {4, 7, 5},
         {0, 4, 5},
         {0, 5, 1},
         {2, 3, 7},
         {2, 7, 6},
         {0, 2, 6},
         {0, 6, 4},
         {1, 5, 7},
         {1, 7, 3}};
  return sx::CollisionShape::makeMesh(std::move(vertices), triangles);
}

struct InclineRun
{
  double downSlopeDisplacement = 0.0; // signed, +x is the down-slope direction
  double downSlopeSpeed = 0.0;
  double minCenterZ = 0.0;
  bool everFailed = false;
};

// Reproduce Fig. 18 ("High school physics friction test"): a flat block resting
// on an inclined plane of slope `inclineRad`, with Coulomb coefficient `mu` on
// both surfaces. Instead of rotating the contact geometry, we tilt gravity so
// the block sits on a flat, axis-aligned ground (robust contact) while feeling
// a down-slope pull g*sin(theta) along +x and a normal load g*cos(theta) along
// -z. This is mechanically identical to the inclined plane and isolates the
// friction threshold tan(theta) vs mu.
InclineRun runInclinedFrictionBlock(double inclineRad, double mu, int steps)
{
  sx::World world;
  constexpr double g = 9.81;
  world.setGravity(
      Eigen::Vector3d(
          g * std::sin(inclineRad), 0.0, -g * std::cos(inclineRad)));
  world.setTimeStep(0.005);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("incline_ground", groundOptions);
  ground.setCollisionShape(sx::CollisionShape::makeBox({5.0, 5.0, 0.5}));
  ground.setFriction(mu);

  // A flat, low block so the friction force does not generate a tipping torque.
  sx::RigidBodyOptions blockOptions;
  blockOptions.mass = 1.0;
  blockOptions.position = Eigen::Vector3d(0.0, 0.0, 0.1 + 1e-3);
  auto block = world.addRigidBody("incline_block", blockOptions);
  block.setCollisionShape(sx::CollisionShape::makeBox({0.4, 0.4, 0.1}));
  block.setFriction(mu);

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  InclineRun run;
  run.minCenterZ = block.getTranslation().z();
  const double startX = block.getTranslation().x();
  for (int s = 0; s < steps; ++s) {
    world.step(executor, pipeline);
    if (ipcStage.getLastStats().failed) {
      run.everFailed = true;
    }
    run.minCenterZ = std::min(run.minCenterZ, block.getTranslation().z());
  }
  run.downSlopeDisplacement = block.getTranslation().x() - startX;
  run.downSlopeSpeed = block.getLinearVelocity().x();
  return run;
}

struct HorizontalSlideRun
{
  double displacement = 0.0;
  double speed = 0.0;
  double minCenterZ = 0.0;
  bool everFailed = false;
};

// Reproduce the 3D friction/sliding fixture mechanism: a cube with a large
// initial tangential velocity slides over a frictional plane. The fixture uses
// mu = 0.05, so the expected effect is modest but observable deceleration
// relative to a frictionless run while the cube remains intersection-free.
HorizontalSlideRun runHorizontalSlidingCube(double mu, int steps)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.8));
  world.setTimeStep(0.005);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("sliding_ground", groundOptions);
  ground.setCollisionShape(sx::CollisionShape::makeBox({30.0, 5.0, 0.5}));
  ground.setFriction(mu);

  sx::RigidBodyOptions blockOptions;
  blockOptions.mass = 1.0;
  blockOptions.position = Eigen::Vector3d(-3.0, 0.0, 0.5 + 1e-3);
  blockOptions.linearVelocity = Eigen::Vector3d(10.0, 0.0, 0.0);
  auto block = world.addRigidBody("sliding_block", blockOptions);
  block.setCollisionShape(sx::CollisionShape::makeBox({0.5, 0.5, 0.5}));
  block.setFriction(mu);

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  HorizontalSlideRun run;
  run.minCenterZ = block.getTranslation().z();
  const double startX = block.getTranslation().x();
  for (int s = 0; s < steps; ++s) {
    world.step(executor, pipeline);
    if (ipcStage.getLastStats().failed) {
      run.everFailed = true;
    }
    run.minCenterZ = std::min(run.minCenterZ, block.getTranslation().z());
  }

  run.displacement = block.getTranslation().x() - startX;
  run.speed = block.getLinearVelocity().x();
  return run;
}

struct TurntableRun
{
  double maxY = 0.0;
  double finalYVelocity = 0.0;
  double minCenterZ = 0.0;
  int failCount = 0;
};

// Reproduce the 3D turntable fixture mechanism: a cube starts on a rotating
// cylinder, and high friction lets the kinematic surface motion carry it
// counter-clockwise.
TurntableRun runKinematicTurntableRider(double mu, int steps)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.8));
  world.setTimeStep(0.005);

  constexpr double tableHalfHeight = 0.08;
  sx::RigidBodyOptions tableOptions;
  tableOptions.position = Eigen::Vector3d(0.0, 0.0, -tableHalfHeight);
  tableOptions.angularVelocity = Eigen::Vector3d(0.0, 0.0, 1.0);
  auto table = world.addRigidBody("turntable", tableOptions);
  table.setCollisionShape(
      makeDiskMesh(/*radius=*/0.6, tableHalfHeight, /*segments=*/24));
  table.setKinematic(true);
  table.setFriction(mu);

  constexpr double cubeHalfExtent = 0.1;
  sx::RigidBodyOptions boxOptions;
  boxOptions.mass = 1.0;
  boxOptions.position = Eigen::Vector3d(0.3, 0.0, cubeHalfExtent + 1e-3);
  auto box = world.addRigidBody("rider", boxOptions);
  box.setCollisionShape(
      sx::CollisionShape::makeBox(
          {cubeHalfExtent, cubeHalfExtent, cubeHalfExtent}));
  box.setFriction(mu);

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  TurntableRun run;
  run.minCenterZ = box.getTranslation().z();
  for (int s = 0; s < steps; ++s) {
    world.step(executor, pipeline);
    if (ipcStage.getLastStats().failed) {
      ++run.failCount;
    }
    run.minCenterZ = std::min(run.minCenterZ, box.getTranslation().z());
    run.maxY = std::max(run.maxY, box.getTranslation().y());
  }
  run.finalYVelocity = box.getLinearVelocity().y();
  return run;
}

void expectFreeEllipsoidAdvancesWithoutContact(const Eigen::Vector3d& angular)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.01);

  sx::RigidBodyOptions bodyOptions;
  bodyOptions.mass = 1.0;
  bodyOptions.position = Eigen::Vector3d::Zero();
  bodyOptions.angularVelocity = angular;
  auto body = world.addRigidBody("rotating_ellipsoid", bodyOptions);
  body.setCollisionShape(makeEllipsoidMesh(
      Eigen::Vector3d(2.0, 1.0, 0.5), /*stacks=*/8, /*slices=*/16));

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  const Eigen::Matrix3d startRotation = body.getRotation();
  world.step(executor, pipeline);

  const auto& stats = ipcStage.getLastStats();
  EXPECT_FALSE(stats.failed);
  EXPECT_TRUE(stats.resultApplied);
  EXPECT_NEAR(body.getTranslation().norm(), 0.0, 1e-12);
  EXPECT_TRUE(body.getRotation().allFinite());
  EXPECT_TRUE(body.getAngularVelocity().allFinite());
  EXPECT_GT((body.getRotation() - startRotation).norm(), 1e-3);
}

} // namespace

// Fig. 18: with a low coefficient (mu = 0.2 well below tan(theta) = 0.5) the
// block slides down the slope and keeps accelerating.
TEST(RigidIpcPaperExperiments, FrictionTestBelowThresholdSlides)
{
  const double theta = std::atan(0.5); // tan(theta) = 0.5
  const InclineRun run
      = runInclinedFrictionBlock(theta, /*mu=*/0.2, /*steps=*/200);

  EXPECT_FALSE(run.everFailed);
  EXPECT_GT(run.minCenterZ, 0.1 - 5e-3); // never penetrated the ground
  // Slid forward and is still moving down-slope.
  EXPECT_GT(run.downSlopeDisplacement, 0.05);
  EXPECT_GT(run.downSlopeSpeed, 0.2);
}

// Fig. 18: with a high coefficient (mu = 0.8 well above tan(theta) = 0.5) the
// friction cone holds the block: it does not slide.
TEST(RigidIpcPaperExperiments, FrictionTestAboveThresholdSticks)
{
  const double theta = std::atan(0.5);
  const InclineRun run
      = runInclinedFrictionBlock(theta, /*mu=*/0.8, /*steps=*/200);

  EXPECT_FALSE(run.everFailed);
  EXPECT_GT(run.minCenterZ, 0.1 - 5e-3); // never penetrated the ground
  // Essentially stationary: negligible down-slope drift and speed.
  EXPECT_LT(std::abs(run.downSlopeDisplacement), 5e-3);
  EXPECT_LT(std::abs(run.downSlopeSpeed), 0.05);
}

// Fig. 18 fidelity: Coulomb friction monotonically resists the slide as mu
// increases past the analytic threshold tan(theta) = 0.5. We discriminate a
// sustained slide from a settle-then-rest by the FINAL down-slope speed (a
// sliding block is still moving; a stuck block has come to rest). Below the
// threshold (mu = 0.3) the block is still sliding at the end; above it (mu =
// 0.7) the block has come to rest, and both the residual speed and the total
// drift are markedly smaller. (The barrier+lagged-friction scaffold allows a
// few centimeters of transient creep near the threshold before resting, so this
// asserts the slide/rest dichotomy rather than a razor-thin mu threshold.)
TEST(RigidIpcPaperExperiments, FrictionMonotonicallyResistsSlideAcrossThreshold)
{
  const double theta = std::atan(0.5);

  const InclineRun below
      = runInclinedFrictionBlock(theta, /*mu=*/0.3, /*steps=*/200);
  EXPECT_FALSE(below.everFailed);
  EXPECT_GT(below.downSlopeSpeed, 0.12) << "mu=0.3 should still be sliding";
  EXPECT_GT(below.downSlopeDisplacement, 0.05);

  const InclineRun above
      = runInclinedFrictionBlock(theta, /*mu=*/0.7, /*steps=*/200);
  EXPECT_FALSE(above.everFailed);
  EXPECT_LT(above.downSlopeSpeed, 0.03) << "mu=0.7 should have come to rest";

  // Friction monotonically reduces both the residual speed and the drift.
  EXPECT_GT(below.downSlopeSpeed, above.downSlopeSpeed + 0.05);
  EXPECT_GT(below.downSlopeDisplacement, above.downSlopeDisplacement);
}

// Fig. 18 fixture row: the upstream corpus pins the below-threshold
// high-school physics example at mu = 0.49 for tan(theta) = 0.5. DART uses the
// same threshold condition in a robust flat-ground formulation and preserves
// the expected sliding behavior for that row.
TEST(RigidIpcPaperExperiments, FrictionThresholdBelowFixtureRowSlides)
{
  const double theta = std::atan(0.5);

  const InclineRun below
      = runInclinedFrictionBlock(theta, /*mu=*/0.49, /*steps=*/200);
  EXPECT_FALSE(below.everFailed);
  EXPECT_GT(below.minCenterZ, 0.1 - 5e-3);
  EXPECT_GT(below.downSlopeDisplacement, 0.01);
  EXPECT_GT(below.downSlopeSpeed, 0.03);
}

// 3D friction fixture row: the upstream high-school physics corpus also keeps a
// high-friction mu = 1.0 scene. With tan(theta) = 0.5, that row should settle
// without a sustained down-slope slide.
TEST(RigidIpcPaperExperiments, FrictionThresholdHighFixtureRowSticks)
{
  const double theta = std::atan(0.5);

  const InclineRun above
      = runInclinedFrictionBlock(theta, /*mu=*/1.0, /*steps=*/200);
  EXPECT_FALSE(above.everFailed);
  EXPECT_GT(above.minCenterZ, 0.1 - 5e-3);
  EXPECT_LT(std::abs(above.downSlopeDisplacement), 5e-3);
  EXPECT_LT(std::abs(above.downSlopeSpeed), 0.05);
}

TEST(RigidIpcPaperExperiments, SlidingCubeFixtureRowIsBrakedByFriction)
{
  const HorizontalSlideRun frictionless
      = runHorizontalSlidingCube(/*mu=*/0.0, /*steps=*/100);
  const HorizontalSlideRun frictional
      = runHorizontalSlidingCube(/*mu=*/0.05, /*steps=*/100);

  EXPECT_FALSE(frictionless.everFailed);
  EXPECT_FALSE(frictional.everFailed);
  EXPECT_GT(frictionless.minCenterZ, 0.5 - 5e-3);
  EXPECT_GT(frictional.minCenterZ, 0.5 - 5e-3);

  EXPECT_GT(frictionless.displacement, 4.0);
  EXPECT_GT(frictional.displacement, 1.0);
  EXPECT_LT(frictional.displacement, frictionless.displacement);
  EXPECT_LT(frictional.speed, frictionless.speed - 0.05);
  EXPECT_GT(frictional.speed, 0.0);
}

TEST(RigidIpcPaperExperiments, RollingConeFixtureRowAdvancesWithContact)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.8));
  world.setTimeStep(0.005);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.05);
  auto ground = world.addRigidBody("rolling_cone_ground", groundOptions);
  ground.setCollisionShape(sx::CollisionShape::makeBox({3.0, 1.0, 0.05}));
  ground.setFriction(0.5);

  sx::RigidBodyOptions coneOptions;
  coneOptions.mass = 1.0;
  coneOptions.position = Eigen::Vector3d(-0.1, 0.0, 0.05);
  coneOptions.orientation = Eigen::AngleAxisd(
      117.0 * std::numbers::pi / 180.0, Eigen::Vector3d::UnitX());
  coneOptions.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  auto cone = world.addRigidBody("rolling_cone", coneOptions);
  cone.setCollisionShape(
      makeConeMesh(/*radius=*/0.1, /*halfHeight=*/0.1, /*segments=*/48));
  cone.setFriction(0.5);

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  const double startX = cone.getTranslation().x();
  bool sawActiveContact = false;
  double maxOverlapDepth = 0.0;
  for (int s = 0; s < 24; ++s) {
    world.step(executor, pipeline);
    const auto& stats = ipcStage.getLastStats();
    EXPECT_FALSE(stats.failed) << "step " << s;
    sawActiveContact = sawActiveContact || stats.activeConstraints > 0u;

    for (const auto& contact : world.collide()) {
      maxOverlapDepth = std::max(maxOverlapDepth, contact.depth);
    }
  }

  EXPECT_TRUE(sawActiveContact);
  EXPECT_LT(maxOverlapDepth, 5e-3);
  EXPECT_GT(cone.getTranslation().x(), startX + 0.05);
  EXPECT_GT(cone.getAngularVelocity().norm(), 0.01);
  EXPECT_TRUE(cone.getTranslation().allFinite());
  EXPECT_TRUE(cone.getLinearVelocity().allFinite());
  EXPECT_TRUE(cone.getAngularVelocity().allFinite());
}

TEST(RigidIpcPaperExperiments, CardTentFixtureRowStaysUprightWithFriction)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.8));
  world.setTimeStep(0.005);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.05);
  auto ground = world.addRigidBody("card_tent_ground", groundOptions);
  ground.setCollisionShape(sx::CollisionShape::makeBox({3.0, 1.0, 0.05}));
  ground.setFriction(0.5);

  constexpr double cardAngle = 65.0 * std::numbers::pi / 180.0;
  constexpr double cardHalfLength = 1.25;
  constexpr double cardHalfWidth = 0.625;
  constexpr double cardHalfThickness = 0.01;
  std::vector<sx::RigidBody> cards;
  for (const auto& spec :
       {std::pair<double, double>{-0.55, cardAngle},
        std::pair<double, double>{0.55, -cardAngle}}) {
    sx::RigidBodyOptions cardOptions;
    cardOptions.mass = 1.0;
    cardOptions.position = Eigen::Vector3d(spec.first, 0.0, 1.20830308488593);
    cardOptions.orientation
        = Eigen::AngleAxisd(spec.second, Eigen::Vector3d::UnitY());
    auto card = world.addRigidBody(
        std::string("card_tent_card_") + std::to_string(cards.size()),
        cardOptions);
    card.setCollisionShape(
        sx::CollisionShape::makeBox(
            {cardHalfLength, cardHalfWidth, cardHalfThickness}));
    card.setFriction(0.5);
    cards.push_back(card);
  }

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  const double initialAverageZ
      = 0.5 * (cards[0].getTranslation().z() + cards[1].getTranslation().z());
  bool sawActiveContact = false;
  double maxOverlapDepth = 0.0;
  for (int s = 0; s < 80; ++s) {
    world.step(executor, pipeline);
    const auto& stats = ipcStage.getLastStats();
    EXPECT_FALSE(stats.failed) << "step " << s;
    sawActiveContact = sawActiveContact || stats.activeConstraints > 0u;

    for (const auto& contact : world.collide()) {
      maxOverlapDepth = std::max(maxOverlapDepth, contact.depth);
    }
  }

  const double finalAverageZ
      = 0.5 * (cards[0].getTranslation().z() + cards[1].getTranslation().z());
  EXPECT_TRUE(sawActiveContact);
  EXPECT_LT(maxOverlapDepth, 5e-3);
  EXPECT_GT(finalAverageZ, initialAverageZ - 0.25);
  for (const auto& card : cards) {
    EXPECT_TRUE(card.getTranslation().allFinite());
    EXPECT_TRUE(card.getLinearVelocity().allFinite());
    EXPECT_TRUE(card.getAngularVelocity().allFinite());
  }
}

// Fig. 7 ("Spolling coin"): a coin spinning on a frictional surface is braked
// by friction. We use a thin triangulated disk resting flat on the ground, spun
// about its symmetry (z) axis. The contact patch friction must dissipate the
// spin while the coin stays intersection-free.
TEST(RigidIpcPaperExperiments, SpinningCoinIsBrakedByFrictionWithoutPenetration)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.005);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("coin_ground", groundOptions);
  ground.setCollisionShape(sx::CollisionShape::makeBox({5.0, 5.0, 0.5}));
  ground.setFriction(0.5);

  constexpr double radius = 0.4;
  constexpr double halfHeight = 0.04;
  sx::RigidBodyOptions coinOptions;
  coinOptions.mass = 1.0;
  // Disk inertia about the symmetry axis (z) and the diameters.
  const double izz = 0.5 * coinOptions.mass * radius * radius;
  const double ixx = 0.25 * coinOptions.mass * radius * radius
                     + (1.0 / 3.0) * coinOptions.mass * halfHeight * halfHeight;
  coinOptions.inertia = Eigen::Vector3d(ixx, ixx, izz).asDiagonal();
  coinOptions.position = Eigen::Vector3d(0.0, 0.0, halfHeight + 1e-3);
  coinOptions.angularVelocity = Eigen::Vector3d(0.0, 0.0, 8.0); // fast spin
  auto coin = world.addRigidBody("coin", coinOptions);
  coin.setCollisionShape(makeDiskMesh(radius, halfHeight, /*segments=*/16));
  coin.setFriction(0.5);

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  const double initialSpin = std::abs(coin.getAngularVelocity().z());
  double minCenterZ = coin.getTranslation().z();
  for (int s = 0; s < 60; ++s) {
    world.step(executor, pipeline);
    EXPECT_FALSE(ipcStage.getLastStats().failed) << "step " << s;
    minCenterZ = std::min(minCenterZ, coin.getTranslation().z());
  }

  // Intersection-free: the coin never sinks into the ground.
  EXPECT_GT(minCenterZ, halfHeight - 5e-3);
  // Friction dissipated a meaningful fraction of the spin.
  const double finalSpin = std::abs(coin.getAngularVelocity().z());
  EXPECT_LT(finalSpin, initialSpin)
      << "friction should brake the spin (init " << initialSpin << ")";
}

TEST(RigidIpcPaperExperiments, TurntableHighFrictionFixtureRowCarriesRider)
{
  const TurntableRun run
      = runKinematicTurntableRider(/*mu=*/1.0, /*steps=*/120);

  EXPECT_GT(run.minCenterZ, 0.1 - 5e-3);
  EXPECT_GT(run.maxY, 0.05);
  EXPECT_GT(run.finalYVelocity, 0.0);
  EXPECT_LE(run.failCount, 3);
}

TEST(RigidIpcPaperExperiments, TurntableModerateFrictionFixtureRowCarriesRider)
{
  const TurntableRun run
      = runKinematicTurntableRider(/*mu=*/0.5, /*steps=*/120);

  EXPECT_GT(run.minCenterZ, 0.1 - 5e-3);
  EXPECT_GT(run.maxY, 0.05);
  EXPECT_GT(run.finalYVelocity, 0.0);
  EXPECT_LE(run.failCount, 3);
}

TEST(RigidIpcPaperExperiments, TurntableLowFrictionFixtureRowCarriesRider)
{
  const TurntableRun run
      = runKinematicTurntableRider(/*mu=*/0.1, /*steps=*/120);

  EXPECT_GT(run.minCenterZ, 0.1 - 5e-3);
  EXPECT_GT(run.maxY, 0.05);
  EXPECT_GT(run.finalYVelocity, 0.0);
  EXPECT_LE(run.failCount, 3);
}

TEST(RigidIpcPaperExperiments, HighSpeedCubeDoesNotTunnelThroughWall)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.01);

  constexpr double wallHalfThickness = 0.05;
  sx::RigidBodyOptions wallOptions;
  wallOptions.isStatic = true;
  wallOptions.position = Eigen::Vector3d(0.0, 0.0, 0.0);
  auto wall = world.addRigidBody("tunneling_wall", wallOptions);
  wall.setCollisionShape(
      sx::CollisionShape::makeBox({wallHalfThickness, 5.0, 5.0}));

  constexpr double cubeHalfExtent = 0.5;
  constexpr double deg = std::numbers::pi / 180.0;
  sx::RigidBodyOptions cubeOptions;
  cubeOptions.mass = 1.0;
  cubeOptions.position = Eigen::Vector3d(-5.0, 0.0, 0.0);
  cubeOptions.linearVelocity = Eigen::Vector3d(1000.0, 0.0, 0.0);
  cubeOptions.orientation
      = Eigen::AngleAxisd(32.0 * deg, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(247.0 * deg, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(53.0 * deg, Eigen::Vector3d::UnitZ());
  auto cube = world.addRigidBody("tunneling_cube", cubeOptions);
  cube.setCollisionShape(
      sx::CollisionShape::makeBox(
          {cubeHalfExtent, cubeHalfExtent, cubeHalfExtent}));

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  world.step(executor, pipeline);

  const Eigen::Matrix3d R = cube.getRotation();
  double maxCubeX = -std::numeric_limits<double>::infinity();
  for (double dx : {-cubeHalfExtent, cubeHalfExtent}) {
    for (double dy : {-cubeHalfExtent, cubeHalfExtent}) {
      for (double dz : {-cubeHalfExtent, cubeHalfExtent}) {
        const Eigen::Vector3d v
            = cube.getTranslation() + R * Eigen::Vector3d(dx, dy, dz);
        maxCubeX = std::max(maxCubeX, v.x());
      }
    }
  }

  EXPECT_TRUE(cube.getTranslation().allFinite());
  EXPECT_LT(cube.getTranslation().x(), 0.0);
  EXPECT_LE(maxCubeX, -wallHalfThickness + 1e-3);
  EXPECT_GT(ipcStage.getLastStats().lineSearchHits, 0u);
}

TEST(RigidIpcPaperExperiments, CubeSettlesOnTwoTrianglePlaneFixtureRow)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.8));
  world.setTimeStep(0.005);

  sx::RigidBodyOptions planeOptions;
  planeOptions.isStatic = true;
  planeOptions.position = Eigen::Vector3d::Zero();
  auto plane = world.addRigidBody("two_triangle_plane", planeOptions);
  plane.setCollisionShape(makeTwoTrianglePlaneMesh(/*halfExtent=*/5.0));

  constexpr double cubeHalfExtent = 0.5;
  sx::RigidBodyOptions cubeOptions;
  cubeOptions.mass = 1.0;
  cubeOptions.position = Eigen::Vector3d(0.0, 0.0, 0.75);
  auto cube = world.addRigidBody("two_triangle_plane_cube", cubeOptions);
  cube.setCollisionShape(
      sx::CollisionShape::makeBox(
          {cubeHalfExtent, cubeHalfExtent, cubeHalfExtent}));

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  double minBottomZ = cube.getTranslation().z() - cubeHalfExtent;
  bool sawActiveContact = false;
  for (int s = 0; s < 120; ++s) {
    world.step(executor, pipeline);
    const auto& stats = ipcStage.getLastStats();
    EXPECT_FALSE(stats.failed) << "step " << s;
    sawActiveContact = sawActiveContact || stats.activeConstraints > 0u;

    const Eigen::Matrix3d R = cube.getRotation();
    double lowest = std::numeric_limits<double>::infinity();
    for (double dx : {-cubeHalfExtent, cubeHalfExtent}) {
      for (double dy : {-cubeHalfExtent, cubeHalfExtent}) {
        for (double dz : {-cubeHalfExtent, cubeHalfExtent}) {
          const Eigen::Vector3d v
              = cube.getTranslation() + R * Eigen::Vector3d(dx, dy, dz);
          lowest = std::min(lowest, v.z());
        }
      }
    }
    minBottomZ = std::min(minBottomZ, lowest);
  }

  EXPECT_GT(minBottomZ, -5e-3);
  EXPECT_TRUE(sawActiveContact);
  EXPECT_TRUE(cube.getTranslation().allFinite());
  EXPECT_TRUE(cube.getLinearVelocity().allFinite());
}

TEST(RigidIpcPaperExperiments, CubeContactsEightKTrianglePlaneFixtureRow)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.8));
  world.setTimeStep(0.005);

  sx::RigidBodyOptions planeOptions;
  planeOptions.isStatic = true;
  planeOptions.position = Eigen::Vector3d::Zero();
  auto plane = world.addRigidBody("eight_k_triangle_plane", planeOptions);
  plane.setCollisionShape(
      makeTessellatedPlaneMesh(/*halfExtent=*/5.0, /*cellsPerAxis=*/64));

  constexpr double cubeHalfExtent = 0.5;
  sx::RigidBodyOptions cubeOptions;
  cubeOptions.mass = 1.0;
  cubeOptions.position = Eigen::Vector3d(0.0, 0.0, 0.525);
  cubeOptions.linearVelocity = Eigen::Vector3d(0.0, 0.0, -0.2);
  auto cube = world.addRigidBody("eight_k_triangle_plane_cube", cubeOptions);
  cube.setCollisionShape(
      sx::CollisionShape::makeBox(
          {cubeHalfExtent, cubeHalfExtent, cubeHalfExtent}));

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  double minBottomZ = cube.getTranslation().z() - cubeHalfExtent;
  bool sawActiveContact = false;
  for (int s = 0; s < 8; ++s) {
    world.step(executor, pipeline);
    const auto& stats = ipcStage.getLastStats();
    EXPECT_FALSE(stats.failed) << "step " << s;
    sawActiveContact = sawActiveContact || stats.activeConstraints > 0u;

    const Eigen::Matrix3d R = cube.getRotation();
    double lowest = std::numeric_limits<double>::infinity();
    for (double dx : {-cubeHalfExtent, cubeHalfExtent}) {
      for (double dy : {-cubeHalfExtent, cubeHalfExtent}) {
        for (double dz : {-cubeHalfExtent, cubeHalfExtent}) {
          const Eigen::Vector3d v
              = cube.getTranslation() + R * Eigen::Vector3d(dx, dy, dz);
          lowest = std::min(lowest, v.z());
        }
      }
    }
    minBottomZ = std::min(minBottomZ, lowest);
  }

  EXPECT_GT(minBottomZ, -5e-3);
  EXPECT_TRUE(sawActiveContact);
  EXPECT_TRUE(cube.getTranslation().allFinite());
  EXPECT_TRUE(cube.getLinearVelocity().allFinite());
}

TEST(RigidIpcPaperExperiments, LargeMassRatioFixtureRowStaysSeparated)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.8));
  world.setTimeStep(0.005);

  sx::RigidBodyOptions planeOptions;
  planeOptions.isStatic = true;
  planeOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto plane = world.addRigidBody("large_mass_ratio_plane", planeOptions);
  plane.setCollisionShape(makeTwoTrianglePlaneMesh(/*halfExtent=*/8.0));

  constexpr double smallHalfExtent = 0.5;
  sx::RigidBodyOptions smallOptions;
  smallOptions.mass = 1.0;
  smallOptions.position = Eigen::Vector3d(0.0, 0.0, 0.1);
  auto smallCube
      = world.addRigidBody("large_mass_ratio_small_cube", smallOptions);
  smallCube.setCollisionShape(
      sx::CollisionShape::makeBox(
          {smallHalfExtent, smallHalfExtent, smallHalfExtent}));

  constexpr double largeHalfExtent = 5.0;
  sx::RigidBodyOptions largeOptions;
  largeOptions.mass = 1000.0;
  largeOptions.position = Eigen::Vector3d(0.0, 0.0, 6.0);
  auto largeCube
      = world.addRigidBody("large_mass_ratio_large_cube", largeOptions);
  largeCube.setCollisionShape(
      sx::CollisionShape::makeBox(
          {largeHalfExtent, largeHalfExtent, largeHalfExtent}));

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  double minSmallPlaneClearance
      = smallCube.getTranslation().z() - smallHalfExtent + 0.5;
  double minLargeSmallClearance
      = largeCube.getTranslation().z() - largeHalfExtent
        - (smallCube.getTranslation().z() + smallHalfExtent);
  for (int s = 0; s < 80; ++s) {
    world.step(executor, pipeline);
    const auto& stats = ipcStage.getLastStats();
    EXPECT_FALSE(stats.failed) << "step " << s;

    minSmallPlaneClearance = std::min(
        minSmallPlaneClearance,
        smallCube.getTranslation().z() - smallHalfExtent + 0.5);
    minLargeSmallClearance = std::min(
        minLargeSmallClearance,
        largeCube.getTranslation().z() - largeHalfExtent
            - (smallCube.getTranslation().z() + smallHalfExtent));
  }

  EXPECT_GT(minSmallPlaneClearance, -5e-3);
  EXPECT_GT(minLargeSmallClearance, -5e-3);
  EXPECT_LT(minLargeSmallClearance, 0.05);
  EXPECT_TRUE(smallCube.getTranslation().allFinite());
  EXPECT_TRUE(largeCube.getTranslation().allFinite());
  EXPECT_TRUE(smallCube.getLinearVelocity().allFinite());
  EXPECT_TRUE(largeCube.getLinearVelocity().allFinite());
}

TEST(RigidIpcPaperExperiments, FiveCubesFixtureRowStacksWithoutPenetration)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.8));
  world.setTimeStep(0.01);

  sx::RigidBodyOptions planeOptions;
  planeOptions.isStatic = true;
  planeOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto plane = world.addRigidBody("five_cubes_plane", planeOptions);
  plane.setCollisionShape(sx::CollisionShape::makeBox({8.0, 8.0, 0.5}));

  constexpr double cubeHalfExtent = 0.5;
  constexpr double initialGap = 0.1;
  std::vector<sx::RigidBody> cubes;
  cubes.reserve(5);
  for (int i = 0; i < 5; ++i) {
    sx::RigidBodyOptions cubeOptions;
    cubeOptions.mass = 1.0;
    cubeOptions.position = Eigen::Vector3d(
        0.0,
        0.0,
        cubeHalfExtent + initialGap
            + (2.0 * cubeHalfExtent + initialGap) * static_cast<double>(i));
    auto cube = world.addRigidBody(
        std::string("five_cubes_") + std::to_string(i), cubeOptions);
    cube.setCollisionShape(
        sx::CollisionShape::makeBox(
            {cubeHalfExtent, cubeHalfExtent, cubeHalfExtent}));
    cubes.push_back(cube);
  }

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  auto zBounds = [](const sx::RigidBody& cube) {
    const Eigen::Matrix3d R = cube.getRotation();
    double minZ = std::numeric_limits<double>::infinity();
    double maxZ = -std::numeric_limits<double>::infinity();
    for (double dx : {-cubeHalfExtent, cubeHalfExtent}) {
      for (double dy : {-cubeHalfExtent, cubeHalfExtent}) {
        for (double dz : {-cubeHalfExtent, cubeHalfExtent}) {
          const Eigen::Vector3d v
              = cube.getTranslation() + R * Eigen::Vector3d(dx, dy, dz);
          minZ = std::min(minZ, v.z());
          maxZ = std::max(maxZ, v.z());
        }
      }
    }
    return std::array<double, 2>{minZ, maxZ};
  };

  double minPlaneClearance = std::numeric_limits<double>::infinity();
  std::array<double, 4> minCubeClearances;
  minCubeClearances.fill(std::numeric_limits<double>::infinity());
  bool sawActiveContact = false;
  for (int s = 0; s < 40; ++s) {
    world.step(executor, pipeline);
    const auto& stats = ipcStage.getLastStats();
    EXPECT_FALSE(stats.failed) << "step " << s;
    sawActiveContact = sawActiveContact || stats.activeConstraints > 0u;

    std::vector<std::array<double, 2>> bounds;
    bounds.reserve(cubes.size());
    for (const auto& cube : cubes) {
      bounds.push_back(zBounds(cube));
      minPlaneClearance = std::min(minPlaneClearance, bounds.back()[0]);
    }
    for (std::size_t i = 1; i < bounds.size(); ++i) {
      minCubeClearances[i - 1]
          = std::min(minCubeClearances[i - 1], bounds[i][0] - bounds[i - 1][1]);
    }
  }

  EXPECT_TRUE(sawActiveContact);
  EXPECT_GT(minPlaneClearance, -5e-3);
  for (const double clearance : minCubeClearances) {
    EXPECT_GT(clearance, -5e-3);
    EXPECT_LT(clearance, 0.05);
  }
  for (const auto& cube : cubes) {
    EXPECT_TRUE(cube.getTranslation().allFinite());
    EXPECT_TRUE(cube.getLinearVelocity().allFinite());
  }
}

TEST(RigidIpcPaperExperiments, CubeFallingOnEdgeFixtureRowStaysSeparated)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.8));
  world.setTimeStep(0.01);

  constexpr double deg = std::numbers::pi / 180.0;
  sx::RigidBodyOptions supportOptions;
  supportOptions.isStatic = true;
  supportOptions.position = Eigen::Vector3d(0.0, 0.0, 0.7);
  supportOptions.orientation
      = Eigen::AngleAxisd(-30.0 * deg, Eigen::Vector3d::UnitY());
  auto support = world.addRigidBody("falling_on_edge_support", supportOptions);
  support.setCollisionShape(sx::CollisionShape::makeBox({1.0, 0.5, 0.5}));

  constexpr double cubeHalfExtent = 0.5;
  sx::RigidBodyOptions cubeOptions;
  cubeOptions.mass = 1.0;
  cubeOptions.position = Eigen::Vector3d(0.0, 0.0, 2.2);
  cubeOptions.orientation
      = Eigen::AngleAxisd(30.0 * deg, Eigen::Vector3d::UnitY());
  auto cube = world.addRigidBody("falling_on_edge_cube", cubeOptions);
  cube.setCollisionShape(
      sx::CollisionShape::makeBox(
          {cubeHalfExtent, cubeHalfExtent, cubeHalfExtent}));

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  const double startZ = cube.getTranslation().z();
  bool sawActiveContact = false;
  double maxOverlapDepth = 0.0;
  for (int s = 0; s < 120; ++s) {
    world.step(executor, pipeline);
    const auto& stats = ipcStage.getLastStats();
    EXPECT_FALSE(stats.failed) << "step " << s;
    sawActiveContact = sawActiveContact || stats.activeConstraints > 0u;

    for (const auto& contact : world.collide()) {
      maxOverlapDepth = std::max(maxOverlapDepth, contact.depth);
    }
  }

  EXPECT_TRUE(sawActiveContact);
  EXPECT_LT(maxOverlapDepth, 5e-3);
  EXPECT_LT(cube.getTranslation().z(), startZ - 0.2);
  EXPECT_TRUE(cube.getTranslation().allFinite());
  EXPECT_TRUE(cube.getLinearVelocity().allFinite());
}

TEST(RigidIpcPaperExperiments, FaceVertexFixtureRowStaysSeparated)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.8));
  world.setTimeStep(0.01);

  sx::RigidBodyOptions fixedOptions;
  fixedOptions.isStatic = true;
  fixedOptions.position = Eigen::Vector3d::Zero();
  auto fixed = world.addRigidBody("face_vertex_fixed_tet", fixedOptions);
  fixed.setCollisionShape(makeTetPyramidMesh());

  sx::RigidBodyOptions movingOptions;
  movingOptions.mass = 1.0;
  movingOptions.position = Eigen::Vector3d(0.0, 0.0, 1.15);
  auto moving = world.addRigidBody("face_vertex_moving_tet", movingOptions);
  moving.setCollisionShape(makeTetPyramidMesh());

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  const double startZ = moving.getTranslation().z();
  bool sawActiveContact = false;
  double maxOverlapDepth = 0.0;
  for (int s = 0; s < 120; ++s) {
    world.step(executor, pipeline);
    const auto& stats = ipcStage.getLastStats();
    EXPECT_FALSE(stats.failed) << "step " << s;
    sawActiveContact = sawActiveContact || stats.activeConstraints > 0u;

    for (const auto& contact : world.collide()) {
      maxOverlapDepth = std::max(maxOverlapDepth, contact.depth);
    }
  }

  EXPECT_TRUE(sawActiveContact);
  EXPECT_LT(maxOverlapDepth, 5e-3);
  EXPECT_LT(moving.getTranslation().z(), startZ - 0.2);
  EXPECT_TRUE(moving.getTranslation().allFinite());
  EXPECT_TRUE(moving.getLinearVelocity().allFinite());
}

TEST(RigidIpcPaperExperiments, VertexFaceFixtureRowStaysSeparated)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.8));
  world.setTimeStep(0.01);

  const Eigen::AngleAxisd flip(std::numbers::pi, Eigen::Vector3d::UnitY());

  sx::RigidBodyOptions fixedOptions;
  fixedOptions.isStatic = true;
  fixedOptions.position = Eigen::Vector3d::Zero();
  fixedOptions.orientation = flip;
  auto fixed = world.addRigidBody("vertex_face_fixed_tet", fixedOptions);
  fixed.setCollisionShape(makeTetPyramidMesh());

  sx::RigidBodyOptions movingOptions;
  movingOptions.mass = 1.0;
  movingOptions.position = Eigen::Vector3d(0.0, 0.0, 1.15);
  movingOptions.orientation = flip;
  auto moving = world.addRigidBody("vertex_face_moving_tet", movingOptions);
  moving.setCollisionShape(makeTetPyramidMesh());

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  const double startZ = moving.getTranslation().z();
  bool sawActiveContact = false;
  double maxOverlapDepth = 0.0;
  for (int s = 0; s < 120; ++s) {
    world.step(executor, pipeline);
    const auto& stats = ipcStage.getLastStats();
    EXPECT_FALSE(stats.failed) << "step " << s;
    sawActiveContact = sawActiveContact || stats.activeConstraints > 0u;

    for (const auto& contact : world.collide()) {
      maxOverlapDepth = std::max(maxOverlapDepth, contact.depth);
    }
  }

  EXPECT_TRUE(sawActiveContact);
  EXPECT_LT(maxOverlapDepth, 5e-3);
  EXPECT_LT(moving.getTranslation().z(), startZ - 0.2);
  EXPECT_TRUE(moving.getTranslation().allFinite());
  EXPECT_TRUE(moving.getLinearVelocity().allFinite());
}

TEST(RigidIpcPaperExperiments, VertexVertexFixtureRowStaysSeparated)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.8));
  world.setTimeStep(0.01);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.55);
  auto ground = world.addRigidBody("vertex_vertex_ground", groundOptions);
  ground.setCollisionShape(sx::CollisionShape::makeBox({5.0, 5.0, 0.05}));

  sx::RigidBodyOptions fixedOptions;
  fixedOptions.isStatic = true;
  fixedOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto fixed = world.addRigidBody("vertex_vertex_fixed_tet", fixedOptions);
  fixed.setCollisionShape(makeTetCornerMesh());

  sx::RigidBodyOptions movingOptions;
  movingOptions.mass = 1.0;
  movingOptions.position = Eigen::Vector3d(0.0, 0.0, 0.7);
  auto moving = world.addRigidBody("vertex_vertex_moving_tet", movingOptions);
  moving.setCollisionShape(makeTetCornerMesh());

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  const double startZ = moving.getTranslation().z();
  bool sawActiveContact = false;
  double maxOverlapDepth = 0.0;
  for (int s = 0; s < 120; ++s) {
    world.step(executor, pipeline);
    const auto& stats = ipcStage.getLastStats();
    EXPECT_FALSE(stats.failed) << "step " << s;
    sawActiveContact = sawActiveContact || stats.activeConstraints > 0u;

    for (const auto& contact : world.collide()) {
      maxOverlapDepth = std::max(maxOverlapDepth, contact.depth);
    }
  }

  EXPECT_TRUE(sawActiveContact);
  EXPECT_LT(maxOverlapDepth, 5e-3);
  EXPECT_LT(moving.getTranslation().z(), startZ - 0.1);
  EXPECT_TRUE(moving.getTranslation().allFinite());
  EXPECT_TRUE(moving.getLinearVelocity().allFinite());
}

TEST(RigidIpcPaperExperiments, TetCornerFixtureRowStaysSeparated)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.01);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.55);
  auto ground = world.addRigidBody("tet_corner_ground", groundOptions);
  ground.setCollisionShape(sx::CollisionShape::makeBox({5.0, 5.0, 0.05}));

  for (int i = 0; i < 3; ++i) {
    sx::RigidBodyOptions wallOptions;
    wallOptions.isStatic = true;
    if (i == 0) {
      wallOptions.position = Eigen::Vector3d(-1.0, 0.0, 0.0);
    } else if (i == 1) {
      wallOptions.position = Eigen::Vector3d(0.0, -1.0, 0.0);
    } else {
      wallOptions.position = Eigen::Vector3d(1.0, 0.0, 0.0);
    }
    auto wall = world.addRigidBody(
        "tet_corner_wall_" + std::to_string(i), wallOptions);
    wall.setCollisionShape(sx::CollisionShape::makeBox({0.5, 0.5, 0.5}));
  }

  sx::RigidBodyOptions tetOptions;
  tetOptions.mass = 1.0;
  tetOptions.position = Eigen::Vector3d(-0.4995, -0.4995, 0.6);
  auto tet = world.addRigidBody("tet_corner_moving", tetOptions);
  tet.setCollisionShape(makeTetCornerMesh(0.999));

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  const double startZ = tet.getTranslation().z();
  bool sawActiveContact = false;
  double maxOverlapDepth = 0.0;
  for (int s = 0; s < 120; ++s) {
    world.step(executor, pipeline);
    const auto& stats = ipcStage.getLastStats();
    EXPECT_FALSE(stats.failed) << "step " << s;
    sawActiveContact = sawActiveContact || stats.activeConstraints > 0u;

    for (const auto& contact : world.collide()) {
      maxOverlapDepth = std::max(maxOverlapDepth, contact.depth);
    }
  }

  EXPECT_TRUE(sawActiveContact);
  EXPECT_LT(maxOverlapDepth, 5e-3);
  EXPECT_LT(tet.getTranslation().z(), startZ - 0.1);
  EXPECT_TRUE(tet.getTranslation().allFinite());
  EXPECT_TRUE(tet.getLinearVelocity().allFinite());
}

TEST(RigidIpcPaperExperiments, TetCornerFallsOnTwoTrianglePlaneFixtureRow)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.8));
  world.setTimeStep(0.01);

  sx::RigidBodyOptions planeOptions;
  planeOptions.isStatic = true;
  planeOptions.position = Eigen::Vector3d::Zero();
  auto plane = world.addRigidBody("two_triangle_tet_plane", planeOptions);
  plane.setCollisionShape(makeTwoTrianglePlaneMesh(/*halfExtent=*/5.0));

  sx::RigidBodyOptions tetOptions;
  tetOptions.mass = 1.0;
  tetOptions.position = Eigen::Vector3d(0.0, 0.0, 0.75);
  tetOptions.orientation
      = Eigen::AngleAxisd(std::numbers::pi, Eigen::Vector3d::UnitZ());
  auto tet = world.addRigidBody("two_triangle_tet", tetOptions);
  tet.setCollisionShape(makeTetCornerMesh());

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  const double startZ = tet.getTranslation().z();
  bool sawActiveContact = false;
  double maxOverlapDepth = 0.0;
  for (int s = 0; s < 120; ++s) {
    world.step(executor, pipeline);
    const auto& stats = ipcStage.getLastStats();
    EXPECT_FALSE(stats.failed) << "step " << s;
    sawActiveContact = sawActiveContact || stats.activeConstraints > 0u;

    for (const auto& contact : world.collide()) {
      maxOverlapDepth = std::max(maxOverlapDepth, contact.depth);
    }
  }

  EXPECT_TRUE(sawActiveContact);
  EXPECT_LT(maxOverlapDepth, 5e-3);
  EXPECT_LT(tet.getTranslation().z(), startZ - 0.2);
  EXPECT_TRUE(tet.getTranslation().allFinite());
  EXPECT_TRUE(tet.getLinearVelocity().allFinite());
}

TEST(RigidIpcPaperExperiments, ErlebenCliffEdgesFixtureRowStaysSeparated)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.8));
  world.setTimeStep(0.01);

  sx::RigidBodyOptions cliffOptions;
  cliffOptions.isStatic = true;
  cliffOptions.position = Eigen::Vector3d(0.0, 0.0, -1.0);
  auto cliff = world.addRigidBody("erleben_cliff_edges", cliffOptions);
  cliff.setCollisionShape(makeErlebenCliffMesh());

  constexpr double cubeHalfExtent = 0.5;
  sx::RigidBodyOptions cubeOptions;
  cubeOptions.mass = 1.0;
  cubeOptions.position = Eigen::Vector3d(0.0, 0.0, 0.05);
  auto cube = world.addRigidBody("erleben_cliff_cube", cubeOptions);
  cube.setCollisionShape(
      sx::CollisionShape::makeBox(
          {cubeHalfExtent, cubeHalfExtent, cubeHalfExtent}));

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  const double startZ = cube.getTranslation().z();
  bool sawActiveContact = false;
  double maxOverlapDepth = 0.0;
  for (int s = 0; s < 20; ++s) {
    world.step(executor, pipeline);
    const auto& stats = ipcStage.getLastStats();
    EXPECT_FALSE(stats.failed) << "step " << s;
    sawActiveContact = sawActiveContact || stats.activeConstraints > 0u;

    for (const auto& contact : world.collide()) {
      maxOverlapDepth = std::max(maxOverlapDepth, contact.depth);
    }
  }

  EXPECT_TRUE(sawActiveContact);
  EXPECT_LT(maxOverlapDepth, 5e-3);
  EXPECT_LT(cube.getTranslation().z(), startZ - 0.02);
  EXPECT_TRUE(cube.getTranslation().allFinite());
  EXPECT_TRUE(cube.getLinearVelocity().allFinite());
}

TEST(RigidIpcPaperExperiments, ErlebenInternalEdgesFixtureRowStaysSeparated)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.8));
  world.setTimeStep(0.01);

  sx::RigidBodyOptions supportOptions;
  supportOptions.isStatic = true;
  supportOptions.position = Eigen::Vector3d(0.0, 0.0, -1.0);
  auto support = world.addRigidBody("erleben_internal_edges", supportOptions);
  support.setCollisionShape(makeErlebenInternalEdgesMesh());

  constexpr double cubeHalfExtent = 0.5;
  sx::RigidBodyOptions cubeOptions;
  cubeOptions.mass = 1.0;
  cubeOptions.position = Eigen::Vector3d(0.0, 0.0, 0.05);
  auto cube = world.addRigidBody("erleben_internal_edges_cube", cubeOptions);
  cube.setCollisionShape(
      sx::CollisionShape::makeBox(
          {cubeHalfExtent, cubeHalfExtent, cubeHalfExtent}));

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  const double startZ = cube.getTranslation().z();
  bool sawActiveContact = false;
  double maxOverlapDepth = 0.0;
  for (int s = 0; s < 20; ++s) {
    world.step(executor, pipeline);
    const auto& stats = ipcStage.getLastStats();
    EXPECT_FALSE(stats.failed) << "step " << s;
    sawActiveContact = sawActiveContact || stats.activeConstraints > 0u;

    for (const auto& contact : world.collide()) {
      maxOverlapDepth = std::max(maxOverlapDepth, contact.depth);
    }
  }

  EXPECT_TRUE(sawActiveContact);
  EXPECT_LT(maxOverlapDepth, 5e-3);
  EXPECT_LT(cube.getTranslation().z(), startZ - 0.02);
  EXPECT_TRUE(cube.getTranslation().allFinite());
  EXPECT_TRUE(cube.getLinearVelocity().allFinite());
}

TEST(RigidIpcPaperExperiments, ErlebenSlidingSpikeFixtureRowStaysSeparated)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.01);

  sx::RigidBodyOptions planeOptions;
  planeOptions.isStatic = true;
  planeOptions.position = Eigen::Vector3d(0.0, 0.0, -1.0);
  auto plane = world.addRigidBody("erleben_sliding_spike_plane", planeOptions);
  plane.setCollisionShape(makeTwoTrianglePlaneMesh(5.0));

  sx::RigidBodyOptions spikeOptions;
  spikeOptions.mass = 1.0;
  spikeOptions.position = Eigen::Vector3d(-4.9, 0.0, 2.0001);
  spikeOptions.orientation
      = Eigen::AngleAxisd(std::numbers::pi, Eigen::Vector3d::UnitY());
  spikeOptions.linearVelocity = Eigen::Vector3d(10.0, 0.0, 0.0);
  auto spike = world.addRigidBody("erleben_sliding_spike", spikeOptions);
  spike.setCollisionShape(makeErlebenSpikeMesh());

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  const double startX = spike.getTranslation().x();
  bool sawActiveContact = false;
  double maxOverlapDepth = 0.0;
  for (int s = 0; s < 20; ++s) {
    world.step(executor, pipeline);
    const auto& stats = ipcStage.getLastStats();
    EXPECT_FALSE(stats.failed) << "step " << s;
    sawActiveContact = sawActiveContact || stats.activeConstraints > 0u;

    for (const auto& contact : world.collide()) {
      maxOverlapDepth = std::max(maxOverlapDepth, contact.depth);
    }
  }

  EXPECT_TRUE(sawActiveContact);
  EXPECT_LT(maxOverlapDepth, 5e-3);
  EXPECT_GT(spike.getTranslation().x(), startX + 0.1);
  EXPECT_TRUE(spike.getTranslation().allFinite());
  EXPECT_TRUE(spike.getLinearVelocity().allFinite());
}

TEST(RigidIpcPaperExperiments, ErlebenSlidingWedgeFixtureRowStaysSeparated)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.01);

  sx::RigidBodyOptions planeOptions;
  planeOptions.isStatic = true;
  planeOptions.position = Eigen::Vector3d(0.0, 0.0, -1.0);
  auto plane = world.addRigidBody("erleben_sliding_wedge_plane", planeOptions);
  plane.setCollisionShape(makeTwoTrianglePlaneMesh(5.0));

  sx::RigidBodyOptions wedgeOptions;
  wedgeOptions.mass = 1.0;
  wedgeOptions.position = Eigen::Vector3d(4.9, 0.0, 2.0001);
  wedgeOptions.orientation
      = Eigen::AngleAxisd(std::numbers::pi, Eigen::Vector3d::UnitY());
  wedgeOptions.linearVelocity = Eigen::Vector3d(-10.0, 0.0, 0.0);
  auto wedge = world.addRigidBody("erleben_sliding_wedge", wedgeOptions);
  wedge.setCollisionShape(makeErlebenWedgeMesh());

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  const double startX = wedge.getTranslation().x();
  bool sawActiveContact = false;
  double maxOverlapDepth = 0.0;
  for (int s = 0; s < 20; ++s) {
    world.step(executor, pipeline);
    const auto& stats = ipcStage.getLastStats();
    EXPECT_FALSE(stats.failed) << "step " << s;
    sawActiveContact = sawActiveContact || stats.activeConstraints > 0u;

    for (const auto& contact : world.collide()) {
      maxOverlapDepth = std::max(maxOverlapDepth, contact.depth);
    }
  }

  EXPECT_TRUE(sawActiveContact);
  EXPECT_LT(maxOverlapDepth, 5e-3);
  EXPECT_LT(wedge.getTranslation().x(), startX - 0.1);
  EXPECT_TRUE(wedge.getTranslation().allFinite());
  EXPECT_TRUE(wedge.getLinearVelocity().allFinite());
}

TEST(RigidIpcPaperExperiments, ErlebenSpikesFixtureRowStaysSeparated)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.8));
  world.setTimeStep(0.01);

  sx::RigidBodyOptions fixedOptions;
  fixedOptions.isStatic = true;
  fixedOptions.position = Eigen::Vector3d(0.0, 0.0, -3.0);
  auto fixed = world.addRigidBody("erleben_fixed_spike", fixedOptions);
  fixed.setCollisionShape(makeErlebenSpikeMesh());

  sx::RigidBodyOptions movingOptions;
  movingOptions.mass = 1.0;
  movingOptions.position = Eigen::Vector3d(0.0, 0.0, 3.0001);
  movingOptions.orientation
      = Eigen::AngleAxisd(std::numbers::pi, Eigen::Vector3d::UnitY());
  auto moving = world.addRigidBody("erleben_moving_spike", movingOptions);
  moving.setCollisionShape(makeErlebenSpikeMesh());

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  bool sawActiveContact = false;
  double maxOverlapDepth = 0.0;
  for (int s = 0; s < 20; ++s) {
    world.step(executor, pipeline);
    const auto& stats = ipcStage.getLastStats();
    EXPECT_FALSE(stats.failed) << "step " << s;
    sawActiveContact = sawActiveContact || stats.activeConstraints > 0u;

    for (const auto& contact : world.collide()) {
      maxOverlapDepth = std::max(maxOverlapDepth, contact.depth);
    }
  }

  EXPECT_TRUE(sawActiveContact);
  EXPECT_LT(maxOverlapDepth, 5e-3);
  EXPECT_TRUE(moving.getTranslation().allFinite());
  EXPECT_TRUE(moving.getLinearVelocity().allFinite());
}

TEST(RigidIpcPaperExperiments, ErlebenWedgesFixtureRowStaysSeparated)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.8));
  world.setTimeStep(0.01);

  sx::RigidBodyOptions fixedOptions;
  fixedOptions.isStatic = true;
  fixedOptions.position = Eigen::Vector3d(0.0, 0.0, -3.0);
  auto fixed = world.addRigidBody("erleben_fixed_wedge", fixedOptions);
  fixed.setCollisionShape(makeErlebenWedgeMesh());

  sx::RigidBodyOptions movingOptions;
  movingOptions.mass = 1.0;
  movingOptions.position = Eigen::Vector3d(0.0, 0.0, 3.0001);
  movingOptions.orientation
      = Eigen::AngleAxisd(std::numbers::pi, Eigen::Vector3d::UnitY());
  auto moving = world.addRigidBody("erleben_moving_wedge", movingOptions);
  moving.setCollisionShape(makeErlebenWedgeMesh());

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  bool sawActiveContact = false;
  double maxOverlapDepth = 0.0;
  for (int s = 0; s < 20; ++s) {
    world.step(executor, pipeline);
    const auto& stats = ipcStage.getLastStats();
    EXPECT_FALSE(stats.failed) << "step " << s;
    sawActiveContact = sawActiveContact || stats.activeConstraints > 0u;

    for (const auto& contact : world.collide()) {
      maxOverlapDepth = std::max(maxOverlapDepth, contact.depth);
    }
  }

  EXPECT_TRUE(sawActiveContact);
  EXPECT_LT(maxOverlapDepth, 5e-3);
  EXPECT_TRUE(moving.getTranslation().allFinite());
  EXPECT_TRUE(moving.getLinearVelocity().allFinite());
}

TEST(RigidIpcPaperExperiments, ErlebenSpikeAndWedgeFixtureRowStaysSeparated)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.8));
  world.setTimeStep(0.01);

  sx::RigidBodyOptions wedgeOptions;
  wedgeOptions.isStatic = true;
  wedgeOptions.position = Eigen::Vector3d(0.0, 0.0, -3.0);
  auto wedge
      = world.addRigidBody("erleben_fixed_wedge_for_spike", wedgeOptions);
  wedge.setCollisionShape(makeErlebenWedgeMesh());

  sx::RigidBodyOptions spikeOptions;
  spikeOptions.mass = 1.0;
  spikeOptions.position = Eigen::Vector3d(0.0, 0.0, 3.0001);
  spikeOptions.orientation
      = Eigen::AngleAxisd(std::numbers::pi, Eigen::Vector3d::UnitY());
  auto spike = world.addRigidBody("erleben_spike_on_wedge", spikeOptions);
  spike.setCollisionShape(makeErlebenSpikeMesh());

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  bool sawActiveContact = false;
  double maxOverlapDepth = 0.0;
  for (int s = 0; s < 20; ++s) {
    world.step(executor, pipeline);
    const auto& stats = ipcStage.getLastStats();
    EXPECT_FALSE(stats.failed) << "step " << s;
    sawActiveContact = sawActiveContact || stats.activeConstraints > 0u;

    for (const auto& contact : world.collide()) {
      maxOverlapDepth = std::max(maxOverlapDepth, contact.depth);
    }
  }

  EXPECT_TRUE(sawActiveContact);
  EXPECT_LT(maxOverlapDepth, 5e-3);
  EXPECT_TRUE(spike.getTranslation().allFinite());
  EXPECT_TRUE(spike.getLinearVelocity().allFinite());
}

TEST(RigidIpcPaperExperiments, ErlebenSpikeInCrackFixtureRowStaysSeparated)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.8));
  world.setTimeStep(0.01);

  sx::RigidBodyOptions crackOptions;
  crackOptions.isStatic = true;
  crackOptions.position = Eigen::Vector3d(0.0, 0.0, -1.0);
  auto crack = world.addRigidBody("erleben_crack_for_spike", crackOptions);
  crack.setCollisionShape(makeErlebenCrackMesh());

  sx::RigidBodyOptions spikeOptions;
  spikeOptions.mass = 1.0;
  spikeOptions.position = Eigen::Vector3d(0.0, 0.0, 2.0001);
  spikeOptions.orientation
      = Eigen::AngleAxisd(std::numbers::pi, Eigen::Vector3d::UnitY());
  spikeOptions.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  auto spike = world.addRigidBody("erleben_spike_in_crack", spikeOptions);
  spike.setCollisionShape(makeErlebenSpikeMesh());
  spike.setForce(Eigen::Vector3d(1.0, 0.0, 0.0));

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  const double startX = spike.getTranslation().x();
  bool sawActiveContact = false;
  double maxOverlapDepth = 0.0;
  for (int s = 0; s < 20; ++s) {
    world.step(executor, pipeline);
    const auto& stats = ipcStage.getLastStats();
    EXPECT_FALSE(stats.failed) << "step " << s;
    sawActiveContact = sawActiveContact || stats.activeConstraints > 0u;

    for (const auto& contact : world.collide()) {
      maxOverlapDepth = std::max(maxOverlapDepth, contact.depth);
    }
  }

  EXPECT_TRUE(sawActiveContact);
  EXPECT_LT(maxOverlapDepth, 5e-3);
  EXPECT_GT(spike.getTranslation().x(), startX + 0.005);
  EXPECT_TRUE(spike.getTranslation().allFinite());
  EXPECT_TRUE(spike.getLinearVelocity().allFinite());
}

TEST(RigidIpcPaperExperiments, ErlebenWedgeInCrackFixtureRowStaysSeparated)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.8));
  world.setTimeStep(0.01);

  sx::RigidBodyOptions crackOptions;
  crackOptions.isStatic = true;
  crackOptions.position = Eigen::Vector3d(0.0, 0.0, -1.0);
  auto crack = world.addRigidBody("erleben_crack_for_wedge", crackOptions);
  crack.setCollisionShape(makeErlebenCrackMesh());

  sx::RigidBodyOptions wedgeOptions;
  wedgeOptions.mass = 1.0;
  wedgeOptions.position = Eigen::Vector3d(0.0, 0.0, 2.001);
  wedgeOptions.orientation
      = Eigen::AngleAxisd(std::numbers::pi, Eigen::Vector3d::UnitY());
  wedgeOptions.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  auto wedge = world.addRigidBody("erleben_wedge_in_crack", wedgeOptions);
  wedge.setCollisionShape(makeErlebenWedgeMesh());
  wedge.setForce(Eigen::Vector3d(10.0, 0.0, 0.0));

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  const double startX = wedge.getTranslation().x();
  bool sawActiveContact = false;
  double maxOverlapDepth = 0.0;
  for (int s = 0; s < 20; ++s) {
    world.step(executor, pipeline);
    const auto& stats = ipcStage.getLastStats();
    EXPECT_FALSE(stats.failed) << "step " << s;
    sawActiveContact = sawActiveContact || stats.activeConstraints > 0u;

    for (const auto& contact : world.collide()) {
      maxOverlapDepth = std::max(maxOverlapDepth, contact.depth);
    }
  }

  EXPECT_TRUE(sawActiveContact);
  EXPECT_LT(maxOverlapDepth, 5e-3);
  EXPECT_GT(wedge.getTranslation().x(), startX + 0.005);
  EXPECT_TRUE(wedge.getTranslation().allFinite());
  EXPECT_TRUE(wedge.getLinearVelocity().allFinite());
}

TEST(RigidIpcPaperExperiments, ErlebenSpikeInHoleFixtureRowStaysSeparated)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.8));
  world.setTimeStep(0.01);

  sx::RigidBodyOptions holeOptions;
  holeOptions.isStatic = true;
  holeOptions.position = Eigen::Vector3d(0.0, 0.0, -1.0);
  auto hole = world.addRigidBody("erleben_hole_for_spike", holeOptions);
  hole.setCollisionShape(makeErlebenHoleMesh());

  sx::RigidBodyOptions spikeOptions;
  spikeOptions.mass = 1.0;
  spikeOptions.position = Eigen::Vector3d(0.0, 0.0, 2.0001);
  spikeOptions.orientation
      = Eigen::AngleAxisd(std::numbers::pi, Eigen::Vector3d::UnitY());
  spikeOptions.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  auto spike = world.addRigidBody("erleben_spike_in_hole", spikeOptions);
  spike.setCollisionShape(makeErlebenSpikeMesh());

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  const double startX = spike.getTranslation().x();
  bool sawActiveContact = false;
  double maxOverlapDepth = 0.0;
  for (int s = 0; s < 20; ++s) {
    world.step(executor, pipeline);
    const auto& stats = ipcStage.getLastStats();
    EXPECT_FALSE(stats.failed) << "step " << s;
    sawActiveContact = sawActiveContact || stats.activeConstraints > 0u;

    for (const auto& contact : world.collide()) {
      maxOverlapDepth = std::max(maxOverlapDepth, contact.depth);
    }
  }

  EXPECT_TRUE(sawActiveContact);
  EXPECT_LT(maxOverlapDepth, 5e-3);
  EXPECT_GT(spike.getTranslation().x(), startX + 0.005);
  EXPECT_TRUE(spike.getTranslation().allFinite());
  EXPECT_TRUE(spike.getLinearVelocity().allFinite());
}

TEST(RigidIpcPaperExperiments, RotatingCubeFixtureRowAdvancesWithoutContact)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.01);

  sx::RigidBodyOptions cubeOptions;
  cubeOptions.mass = 1.0;
  cubeOptions.position = Eigen::Vector3d::Zero();
  cubeOptions.angularVelocity = Eigen::Vector3d(0.1, 180.0, 0.0);
  auto cube = world.addRigidBody("rotating_cube", cubeOptions);
  cube.setCollisionShape(sx::CollisionShape::makeBox({0.5, 0.5, 0.5}));

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  const Eigen::Matrix3d startRotation = cube.getRotation();
  world.step(executor, pipeline);

  const auto& stats = ipcStage.getLastStats();
  EXPECT_FALSE(stats.failed);
  EXPECT_TRUE(stats.resultApplied);
  EXPECT_NEAR(cube.getTranslation().norm(), 0.0, 1e-12);
  EXPECT_TRUE(cube.getRotation().allFinite());
  EXPECT_TRUE(cube.getAngularVelocity().allFinite());
  EXPECT_GT((cube.getRotation() - startRotation).norm(), 1e-3);
}

TEST(RigidIpcPaperExperiments, SpinningCubeOverPlaneFixtureRowAdvancesSafely)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.01);

  constexpr double planeTopZ = -0.5001;
  constexpr double planeHalfThickness = 0.05;
  sx::RigidBodyOptions planeOptions;
  planeOptions.isStatic = true;
  planeOptions.position
      = Eigen::Vector3d(2.0, -2.0, planeTopZ - planeHalfThickness);
  auto plane = world.addRigidBody("spinning_cube_plane", planeOptions);
  plane.setCollisionShape(
      sx::CollisionShape::makeBox({5.0, 5.0, planeHalfThickness}));

  constexpr double cubeHalfExtent = 0.5;
  sx::RigidBodyOptions cubeOptions;
  cubeOptions.mass = 1.0;
  cubeOptions.position = Eigen::Vector3d::Zero();
  cubeOptions.angularVelocity = Eigen::Vector3d(0.0, 0.0, 180.0);
  auto cube = world.addRigidBody("spinning_cube_over_plane", cubeOptions);
  cube.setCollisionShape(
      sx::CollisionShape::makeBox(
          {cubeHalfExtent, cubeHalfExtent, cubeHalfExtent}));

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  const Eigen::Matrix3d startRotation = cube.getRotation();
  double minClearance = std::numeric_limits<double>::infinity();
  for (int s = 0; s < 20; ++s) {
    world.step(executor, pipeline);
    EXPECT_FALSE(ipcStage.getLastStats().failed) << "step " << s;

    const Eigen::Matrix3d R = cube.getRotation();
    double lowest = std::numeric_limits<double>::infinity();
    for (double dx : {-cubeHalfExtent, cubeHalfExtent}) {
      for (double dy : {-cubeHalfExtent, cubeHalfExtent}) {
        for (double dz : {-cubeHalfExtent, cubeHalfExtent}) {
          const Eigen::Vector3d v
              = cube.getTranslation() + R * Eigen::Vector3d(dx, dy, dz);
          lowest = std::min(lowest, v.z());
        }
      }
    }
    minClearance = std::min(minClearance, lowest - planeTopZ);
  }

  EXPECT_GT(minClearance, -5e-4);
  EXPECT_LT(cube.getTranslation().head<2>().norm(), 1e-2);
  EXPECT_TRUE(cube.getTranslation().allFinite());
  EXPECT_TRUE(cube.getRotation().allFinite());
  EXPECT_TRUE(cube.getAngularVelocity().allFinite());
  EXPECT_GT((cube.getRotation() - startRotation).norm(), 1e-3);
}

TEST(
    RigidIpcPaperExperiments,
    RotatingScaledSphereFixtureRowAdvancesWithoutContact)
{
  expectFreeEllipsoidAdvancesWithoutContact(Eigen::Vector3d(0.0, 0.0, -180.0));
}

TEST(
    RigidIpcPaperExperiments,
    RotatingEllipsoidMajorFixtureRowAdvancesWithoutContact)
{
  expectFreeEllipsoidAdvancesWithoutContact(Eigen::Vector3d(180.0, 0.0, 0.0));
}

TEST(
    RigidIpcPaperExperiments,
    RotatingEllipsoidIntermediateFixtureRowAdvancesWithoutContact)
{
  expectFreeEllipsoidAdvancesWithoutContact(Eigen::Vector3d(0.0, 180.0, 0.0));
}

TEST(
    RigidIpcPaperExperiments,
    RotatingEllipsoidMinorFixtureRowAdvancesWithoutContact)
{
  expectFreeEllipsoidAdvancesWithoutContact(Eigen::Vector3d(0.0, 0.0, 180.0));
}

TEST(RigidIpcPaperExperiments, TorqueFixtureRowAcceleratesFreeBody)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.01);

  sx::RigidBodyOptions bodyOptions;
  bodyOptions.mass = 1.0;
  bodyOptions.position = Eigen::Vector3d::Zero();
  bodyOptions.orientation
      = Eigen::AngleAxisd(std::numbers::pi / 2.0, Eigen::Vector3d::UnitX());
  bodyOptions.inertia = Eigen::Vector3d(1.0, 1.0, 1.0).asDiagonal();
  auto body = world.addRigidBody("torqued_gear", bodyOptions);
  body.setCollisionShape(makeDiskMesh(/*radius=*/0.5, /*halfHeight=*/0.1, 24));
  body.setTorque(Eigen::Vector3d(0.0, 90.0, 0.0));

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  world.step(executor, pipeline);

  const auto& stats = ipcStage.getLastStats();
  EXPECT_FALSE(stats.failed);
  EXPECT_TRUE(stats.resultApplied);
  EXPECT_TRUE(body.getTranslation().isApprox(Eigen::Vector3d::Zero(), 1e-12));
  EXPECT_TRUE(body.getAngularVelocity().allFinite());
  EXPECT_GT(body.getAngularVelocity().y(), 0.1);
  EXPECT_TRUE(body.getRotation().allFinite());
}

TEST(RigidIpcPaperExperiments, DzhanibekovWingNutFixtureRowAdvancesSafely)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.01);

  constexpr double deg = std::numbers::pi / 180.0;
  sx::RigidBodyOptions bodyOptions;
  bodyOptions.mass = 1.0;
  bodyOptions.position = Eigen::Vector3d::Zero();
  bodyOptions.orientation
      = Eigen::AngleAxisd(-3.0 * deg, Eigen::Vector3d::UnitX());
  bodyOptions.angularVelocity = Eigen::Vector3d(0.0, 0.0, 180.0);
  bodyOptions.inertia = Eigen::Vector3d(1.0, 4.0, 6.0).asDiagonal();
  auto body = world.addRigidBody("dzhanibekov_wing_nut", bodyOptions);
  body.setCollisionShape(makeWingNutLikeMesh());

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  const Eigen::Matrix3d startRotation = body.getRotation();
  for (int s = 0; s < 20; ++s) {
    world.step(executor, pipeline);
    EXPECT_FALSE(ipcStage.getLastStats().failed) << "step " << s;
  }

  EXPECT_TRUE(ipcStage.getLastStats().resultApplied);
  EXPECT_NEAR(body.getTranslation().norm(), 0.0, 1e-12);
  EXPECT_TRUE(body.getRotation().allFinite());
  EXPECT_TRUE(body.getAngularVelocity().allFinite());
  EXPECT_GT((body.getRotation() - startRotation).norm(), 1e-3);
}

// Figs. 16/17 (unit tests / Erleben degenerate cases): a box dropped onto its
// edge (rotated 45 degrees about x) lands in a degenerate edge-on-face contact.
// The solver must handle the degenerate configuration robustly: it settles to
// rest without ever penetrating the ground and without a failed solve.
TEST(RigidIpcPaperExperiments, DegenerateEdgeDropSettlesWithoutPenetration)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.005);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("erleben_ground", groundOptions);
  ground.setCollisionShape(sx::CollisionShape::makeBox({5.0, 5.0, 0.5}));
  ground.setFriction(0.6);

  // A unit cube rotated 45 degrees about x: it descends onto a bottom edge. The
  // half-diagonal of the y-z face is 0.25 * sqrt(2) ~ 0.3536, so we place the
  // center just above that so the edge starts a hair above the ground.
  constexpr double half = 0.25;
  const double edgeHeight = half * std::numbers::sqrt2;
  sx::RigidBodyOptions boxOptions;
  boxOptions.mass = 1.0;
  boxOptions.position = Eigen::Vector3d(0.0, 0.0, edgeHeight + 2e-3);
  boxOptions.orientation = Eigen::Quaterniond(
      Eigen::AngleAxisd(std::numbers::pi / 4.0, Eigen::Vector3d::UnitX()));
  auto box = world.addRigidBody("erleben_box", boxOptions);
  box.setCollisionShape(sx::CollisionShape::makeBox({half, half, half}));
  box.setFriction(0.6);

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  double minBottomZ = 0.0;
  for (int s = 0; s < 120; ++s) {
    world.step(executor, pipeline);
    EXPECT_FALSE(ipcStage.getLastStats().failed) << "step " << s;
    // Lowest world-space vertex of the rotated cube (an axis-aligned bound on
    // penetration). The center height minus the rotated half-extent.
    const Eigen::Matrix3d R = box.getRotation();
    double lowest = box.getTranslation().z();
    for (double dx : {-half, half}) {
      for (double dy : {-half, half}) {
        for (double dz : {-half, half}) {
          const Eigen::Vector3d v
              = box.getTranslation() + R * Eigen::Vector3d(dx, dy, dz);
          lowest = std::min(lowest, v.z());
        }
      }
    }
    minBottomZ = std::min(minBottomZ, lowest);
  }

  // Never penetrated the ground top (z = 0) beyond the barrier tolerance.
  EXPECT_GT(minBottomZ, -5e-3);
  // Came to rest (no explosion): the cube is no longer falling fast.
  EXPECT_LT(box.getLinearVelocity().norm(), 0.5);
  // All quantities stayed finite.
  EXPECT_TRUE(box.getTranslation().allFinite());
  EXPECT_TRUE(box.getLinearVelocity().allFinite());
}

// Figs. 16/17 (unit tests / Erleben degenerate cases), point-contact variant: a
// cube released with its body diagonal pointing straight down lands on a single
// vertex -- the most degenerate box-on-ground contact. The barrier and CCD keep
// it intersection-free as it tips off the unstable point onto a face and
// settles, with no failed solve or divergence.
TEST(RigidIpcPaperExperiments, DegenerateVertexDropStaysIntersectionFree)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.005);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("vertex_ground", groundOptions);
  ground.setCollisionShape(sx::CollisionShape::makeBox({5.0, 5.0, 0.5}));
  ground.setFriction(0.6);

  constexpr double half = 0.25;
  // Rotate the cube so its (+,+,+) body diagonal points to -z: the corner is
  // the lowest feature, a distance half*sqrt(3) below the center.
  const Eigen::Quaterniond tilt = Eigen::Quaterniond::FromTwoVectors(
      Eigen::Vector3d(1.0, 1.0, 1.0).normalized(),
      Eigen::Vector3d(0.0, 0.0, -1.0));
  sx::RigidBodyOptions boxOptions;
  boxOptions.mass = 1.0;
  boxOptions.position
      = Eigen::Vector3d(0.0, 0.0, half * std::numbers::sqrt3 + 0.03);
  boxOptions.orientation = tilt;
  auto box = world.addRigidBody("vertex_box", boxOptions);
  box.setCollisionShape(sx::CollisionShape::makeBox({half, half, half}));
  box.setFriction(0.6);

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  double minBottomZ = 0.0;
  for (int s = 0; s < 120; ++s) {
    world.step(executor, pipeline);
    EXPECT_FALSE(ipcStage.getLastStats().failed) << "step " << s;
    const Eigen::Matrix3d R = box.getRotation();
    double lowest = box.getTranslation().z();
    for (double dx : {-half, half}) {
      for (double dy : {-half, half}) {
        for (double dz : {-half, half}) {
          const Eigen::Vector3d v
              = box.getTranslation() + R * Eigen::Vector3d(dx, dy, dz);
          lowest = std::min(lowest, v.z());
        }
      }
    }
    minBottomZ = std::min(minBottomZ, lowest);
  }

  EXPECT_GT(minBottomZ, -5e-3);                   // never penetrated the ground
  EXPECT_LT(box.getLinearVelocity().norm(), 0.5); // came to rest
  EXPECT_TRUE(box.getTranslation().allFinite());
  EXPECT_TRUE(box.getLinearVelocity().allFinite());
}

// Fig. 11 (Arch): a five-voussoir semicircular arch stands in equilibrium under
// gravity, held by friction at the wedge interfaces and the ground (no
// abutments). The keystone (top wedge) is wedged between the springers;
// friction resists the outward thrust at the ground. The keystone must not
// collapse, nothing may penetrate the ground, and the dense exact-contact
// plateau must not report a persistent failed solve.
TEST(RigidIpcPaperExperiments, FiveVoussoirFrictionArchStandsInEquilibrium)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.005);

  constexpr int kBlocks = 5; // odd: the middle wedge is the keystone
  constexpr double innerRadius = 1.0;
  constexpr double rOut = 1.3;
  constexpr double halfW = 0.15;
  constexpr double lift = 0.02; // start just above the ground
  const double dTheta = std::numbers::pi / kBlocks;
  constexpr double angularGap = 4e-3;

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5 + lift);
  auto ground = world.addRigidBody("arch_ground", groundOptions);
  ground.setCollisionShape(sx::CollisionShape::makeBox({3.0, 1.0, 0.5}));
  ground.setFriction(1.0);

  std::vector<sx::RigidBody> voussoirs;
  std::vector<std::vector<Eigen::Vector3d>> localVertices;
  for (int i = 0; i < kBlocks; ++i) {
    const double t0 = i * dTheta + 0.5 * angularGap;
    const double t1 = (i + 1) * dTheta - 0.5 * angularGap;
    Eigen::Vector3d center;
    auto shape = makeVoussoirMesh(t0, t1, innerRadius, rOut, halfW, center);
    localVertices.push_back(shape.vertices); // body-frame wedge vertices
    sx::RigidBodyOptions options;
    options.mass = 1.0;
    options.position = center + Eigen::Vector3d(0.0, 0.0, lift);
    auto v = world.addRigidBody(
        std::string("arch_voussoir_") + std::to_string(i), options);
    v.setCollisionShape(shape);
    v.setFriction(1.0);
    voussoirs.push_back(v);
  }

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  const double keystoneStartZ = voussoirs[kBlocks / 2].getTranslation().z();
  double minVoussoirZ = lift;
  for (int s = 0; s < 80; ++s) {
    world.step(executor, pipeline);
    const auto& stats = ipcStage.getLastStats();
    EXPECT_FALSE(stats.failed)
        << "step " << s << " status=" << static_cast<int>(stats.status)
        << " accepted=" << stats.acceptedSteps
        << " kappa=" << stats.barrierStiffness
        << " kappaIncreases=" << stats.barrierStiffnessIncreases
        << " lsZero=" << stats.lineSearchZeroStepCount;
    EXPECT_TRUE(stats.resultApplied)
        << "step " << s << " status=" << static_cast<int>(stats.status)
        << " accepted=" << stats.acceptedSteps
        << " kappa=" << stats.barrierStiffness
        << " kappaIncreases=" << stats.barrierStiffnessIncreases
        << " lsZero=" << stats.lineSearchZeroStepCount;
    for (std::size_t i = 0; i < voussoirs.size(); ++i) {
      const Eigen::Matrix3d R = voussoirs[i].getRotation();
      const Eigen::Vector3d t = voussoirs[i].getTranslation();
      for (const Eigen::Vector3d& localVertex : localVertices[i]) {
        minVoussoirZ = std::min(minVoussoirZ, (t + R * localVertex).z());
      }
    }
  }
  const double keystoneEndZ = voussoirs[kBlocks / 2].getTranslation().z();

  // The arch holds: the keystone barely settles (does not collapse).
  EXPECT_GT(keystoneEndZ, keystoneStartZ - 0.05);
  EXPECT_TRUE(voussoirs[kBlocks / 2].getTranslation().allFinite());
  // Nothing fell through the ground (top at z = lift) beyond the barrier band.
  EXPECT_GT(minVoussoirZ, lift - 5e-3);
}

TEST(RigidIpcPaperExperiments, TwentyFiveVoussoirFrictionArchFixtureRowStands)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.005);

  constexpr int kBlocks = 25;
  constexpr double innerRadius = 1.0;
  constexpr double rOut = 1.3;
  constexpr double halfW = 0.15;
  constexpr double lift = 0.02;
  const double dTheta = std::numbers::pi / kBlocks;
  constexpr double angularGap = 1e-3;

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5 + lift);
  auto ground = world.addRigidBody("arch_25_ground", groundOptions);
  ground.setCollisionShape(sx::CollisionShape::makeBox({3.0, 1.0, 0.5}));
  ground.setFriction(1.0);

  std::vector<sx::RigidBody> voussoirs;
  std::vector<std::vector<Eigen::Vector3d>> localVertices;
  for (int i = 0; i < kBlocks; ++i) {
    const double t0 = i * dTheta + 0.5 * angularGap;
    const double t1 = (i + 1) * dTheta - 0.5 * angularGap;
    Eigen::Vector3d center;
    auto shape = makeVoussoirMesh(t0, t1, innerRadius, rOut, halfW, center);
    localVertices.push_back(shape.vertices);
    sx::RigidBodyOptions options;
    options.mass = 1.0;
    options.position = center + Eigen::Vector3d(0.0, 0.0, lift);
    auto v = world.addRigidBody(
        std::string("arch_25_voussoir_") + std::to_string(i), options);
    v.setCollisionShape(shape);
    v.setFriction(1.0);
    voussoirs.push_back(v);
  }

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  const double keystoneStartZ = voussoirs[kBlocks / 2].getTranslation().z();
  double minVoussoirZ = lift;
  bool sawActiveContact = false;
  for (int s = 0; s < 20; ++s) {
    world.step(executor, pipeline);
    const auto& stats = ipcStage.getLastStats();
    EXPECT_FALSE(stats.failed) << "step " << s;
    sawActiveContact = sawActiveContact || stats.activeConstraints > 0u;

    for (std::size_t i = 0; i < voussoirs.size(); ++i) {
      const Eigen::Matrix3d R = voussoirs[i].getRotation();
      const Eigen::Vector3d t = voussoirs[i].getTranslation();
      for (const Eigen::Vector3d& localVertex : localVertices[i]) {
        minVoussoirZ = std::min(minVoussoirZ, (t + R * localVertex).z());
      }
    }
  }
  const double keystoneEndZ = voussoirs[kBlocks / 2].getTranslation().z();

  EXPECT_TRUE(sawActiveContact);
  EXPECT_GT(keystoneEndZ, keystoneStartZ - 0.02);
  EXPECT_TRUE(voussoirs[kBlocks / 2].getTranslation().allFinite());
  EXPECT_GT(minVoussoirZ, lift - 5e-3);
}
