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

// WP-ASV.4 camera-control coverage: the canonical view presets, the
// modify-not-replace per-view resolver (azimuth/elevation degrees -> orbit
// yaw/pitch), the scene bounding-sphere helper, and the auto-frame distance
// formula. These are pure value-type helpers on the public dart::gui surface.

#include <dart/gui/renderable.hpp>
#include <dart/gui/viewer.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <vector>

#include <cmath>

namespace {

constexpr double kPi = 3.14159265358979323846;

double deg2rad(double degrees)
{
  return degrees * kPi / 180.0;
}

dart::gui::RenderableDescriptor makeBox(
    const Eigen::Vector3d& size, const Eigen::Vector3d& translation)
{
  dart::gui::RenderableDescriptor descriptor;
  descriptor.geometry.kind = dart::gui::ShapeKind::Box;
  descriptor.geometry.size = size;
  descriptor.worldTransform = Eigen::Isometry3d::Identity();
  descriptor.worldTransform.translation() = translation;
  return descriptor;
}

} // namespace

TEST(CameraControl, PresetsMatchHouseDefaults)
{
  double azimuth = 0.0;
  double elevation = 0.0;

  EXPECT_TRUE(
      dart::gui::orbitCameraViewPreset("three-quarter", azimuth, elevation));
  EXPECT_DOUBLE_EQ(azimuth, -45.0);
  EXPECT_DOUBLE_EQ(elevation, 25.0);

  EXPECT_TRUE(dart::gui::orbitCameraViewPreset("front", azimuth, elevation));
  EXPECT_DOUBLE_EQ(azimuth, -90.0);
  EXPECT_DOUBLE_EQ(elevation, 0.0);

  EXPECT_TRUE(dart::gui::orbitCameraViewPreset("side", azimuth, elevation));
  EXPECT_DOUBLE_EQ(azimuth, 0.0);
  EXPECT_DOUBLE_EQ(elevation, 0.0);

  EXPECT_TRUE(dart::gui::orbitCameraViewPreset("top", azimuth, elevation));
  EXPECT_DOUBLE_EQ(azimuth, -90.0);
  EXPECT_DOUBLE_EQ(elevation, 89.0);

  EXPECT_FALSE(dart::gui::orbitCameraViewPreset("bogus", azimuth, elevation));
}

TEST(CameraControl, ExplicitAnglesMapToYawPitch)
{
  dart::gui::OrbitCamera base;
  base.yaw = 1.23;
  base.pitch = 0.45;
  base.distance = 3.0;
  base.target = Eigen::Vector3d(1.0, 2.0, 3.0);

  dart::gui::OrbitCameraViewOptions view;
  view.azimuthDegrees = 30.0;
  view.elevationDegrees = -15.0;
  const dart::gui::OrbitCamera resolved
      = dart::gui::applyOrbitCameraView(base, view);

  EXPECT_NEAR(resolved.yaw, deg2rad(30.0), 1e-12);
  EXPECT_NEAR(resolved.pitch, deg2rad(-15.0), 1e-12);
  // Modify-not-replace: unspecified fields keep the base values.
  EXPECT_DOUBLE_EQ(resolved.distance, 3.0);
  EXPECT_TRUE(resolved.target.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
}

TEST(CameraControl, PresetSeedsAnglesAndExplicitOverridesWin)
{
  dart::gui::OrbitCamera base;
  base.distance = 5.0;

  dart::gui::OrbitCameraViewOptions presetOnly;
  presetOnly.preset = std::string("front");
  const dart::gui::OrbitCamera preset
      = dart::gui::applyOrbitCameraView(base, presetOnly);
  EXPECT_NEAR(preset.yaw, deg2rad(-90.0), 1e-12);
  EXPECT_NEAR(preset.pitch, deg2rad(0.0), 1e-12);
  EXPECT_DOUBLE_EQ(preset.distance, 5.0);

  dart::gui::OrbitCameraViewOptions overridden;
  overridden.preset = std::string("front");
  overridden.azimuthDegrees = 10.0;
  overridden.distance = 2.5;
  overridden.target = Eigen::Vector3d(0.0, 0.0, 1.0);
  const dart::gui::OrbitCamera resolved
      = dart::gui::applyOrbitCameraView(base, overridden);
  // Explicit azimuth overrides the preset azimuth; preset elevation stays.
  EXPECT_NEAR(resolved.yaw, deg2rad(10.0), 1e-12);
  EXPECT_NEAR(resolved.pitch, deg2rad(0.0), 1e-12);
  EXPECT_DOUBLE_EQ(resolved.distance, 2.5);
  EXPECT_TRUE(resolved.target.isApprox(Eigen::Vector3d(0.0, 0.0, 1.0)));
}

TEST(CameraControl, BoundingSphereUnionsDescriptorBoxes)
{
  std::vector<dart::gui::RenderableDescriptor> descriptors;
  descriptors.push_back(
      makeBox(Eigen::Vector3d(2.0, 2.0, 2.0), Eigen::Vector3d::Zero()));

  const dart::gui::BoundingSphere single
      = dart::gui::sceneBoundingSphere(descriptors);
  EXPECT_TRUE(single.center.isApprox(Eigen::Vector3d::Zero()));
  // Half-diagonal of a [-1,1]^3 box = 0.5 * |(2,2,2)| = sqrt(3).
  EXPECT_NEAR(single.radius, std::sqrt(3.0), 1e-9);

  descriptors.push_back(
      makeBox(Eigen::Vector3d(2.0, 2.0, 2.0), Eigen::Vector3d(10.0, 0.0, 0.0)));
  const dart::gui::BoundingSphere pair
      = dart::gui::sceneBoundingSphere(descriptors);
  // x in [-1, 11], y,z in [-1, 1] -> center (5,0,0).
  EXPECT_TRUE(pair.center.isApprox(Eigen::Vector3d(5.0, 0.0, 0.0)));
  EXPECT_NEAR(pair.radius, 0.5 * Eigen::Vector3d(12.0, 2.0, 2.0).norm(), 1e-9);
}

TEST(CameraControl, BoundingSphereEmptyIsFinite)
{
  const dart::gui::BoundingSphere sphere = dart::gui::sceneBoundingSphere({});
  EXPECT_TRUE(sphere.center.isApprox(Eigen::Vector3d::Zero()));
  EXPECT_GT(sphere.radius, 0.0);
  EXPECT_TRUE(std::isfinite(sphere.radius));
}

TEST(CameraControl, FitDistanceMatchesFovFormula)
{
  dart::gui::BoundingSphere sphere;
  sphere.center = Eigen::Vector3d(0.0, 0.0, 0.5);
  sphere.radius = 2.0;

  const double fovYDegrees = 45.0;
  const dart::gui::OrbitCamera camera
      = dart::gui::fitOrbitCamera(sphere, fovYDegrees, -45.0, 25.0);

  EXPECT_TRUE(camera.target.isApprox(sphere.center));
  EXPECT_NEAR(camera.yaw, deg2rad(-45.0), 1e-12);
  EXPECT_NEAR(camera.pitch, deg2rad(25.0), 1e-12);
  const double expectedDistance
      = sphere.radius / std::sin(0.5 * deg2rad(fovYDegrees));
  EXPECT_NEAR(camera.distance, expectedDistance, 1e-9);
}
