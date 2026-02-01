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

#include "dart/gui/orbit_camera_controller.hpp"

#include <algorithm>
#include <type_traits>

#include <cmath>

namespace dart {
namespace gui {

namespace {
constexpr double kDegToRad = 0.01745329251994329;
constexpr double kPi = 3.14159265358979323846;
} // namespace

OrbitCameraController::OrbitCameraController(
    double rotate_sensitivity,
    double pan_sensitivity,
    double zoom_sensitivity,
    double min_distance,
    double max_distance)
  : rotate_sensitivity_(rotate_sensitivity),
    pan_sensitivity_(pan_sensitivity),
    zoom_sensitivity_(zoom_sensitivity),
    min_distance_(min_distance),
    max_distance_(max_distance)
{
}

void OrbitCameraController::handleEvent(const InputEvent& event, Camera& camera)
{
  std::visit(
      [this, &camera](const auto& e) {
        using EventType = std::decay_t<decltype(e)>;
        if constexpr (std::is_same_v<EventType, MouseButtonEvent>) {
          if (e.button == MouseButton::Right) {
            right_button_down_ = e.pressed;
          }
          if (e.button == MouseButton::Middle) {
            middle_button_down_ = e.pressed;
          }
          return;
        }

        if constexpr (std::is_same_v<EventType, MouseMoveEvent>) {
          if (right_button_down_) {
            Eigen::Vector3d view = camera.position - camera.target;
            const double distance = view.norm();
            if (distance < 1e-9) {
              return;
            }

            const double yaw_angle = -e.dx * rotate_sensitivity_ * kDegToRad;
            const double pitch_angle = -e.dy * rotate_sensitivity_ * kDegToRad;

            Eigen::Vector3d right = view.cross(camera.up);
            if (right.norm() < 1e-9) {
              right = Eigen::Vector3d::UnitX();
            } else {
              right.normalize();
            }

            const Eigen::AngleAxisd yaw(yaw_angle, camera.up.normalized());
            Eigen::Vector3d rotated_view = yaw * view;
            const Eigen::AngleAxisd pitch(pitch_angle, right);
            const Eigen::Vector3d pitched_view = pitch * rotated_view;

            const double dot = std::clamp(
                pitched_view.normalized().dot(camera.up.normalized()),
                -1.0,
                1.0);
            const double angle = std::acos(dot);
            if (angle >= 0.1 && angle <= kPi - 0.1) {
              rotated_view = pitched_view;
            }

            camera.position = camera.target + rotated_view;
          }

          if (middle_button_down_) {
            const Eigen::Vector3d forward = camera.target - camera.position;
            const double distance = forward.norm();
            if (distance < 1e-9) {
              return;
            }

            const Eigen::Vector3d right = forward.cross(camera.up).normalized();
            const Eigen::Vector3d local_up = right.cross(forward).normalized();
            const double scale = pan_sensitivity_ * distance;
            const Eigen::Vector3d delta
                = -e.dx * scale * right + e.dy * scale * local_up;
            camera.position += delta;
            camera.target += delta;
          }

          return;
        }

        if constexpr (std::is_same_v<EventType, ScrollEvent>) {
          const Eigen::Vector3d view = camera.position - camera.target;
          const double distance = view.norm();
          if (distance < 1e-9) {
            return;
          }

          const double new_distance = std::clamp(
              distance - e.dy * zoom_sensitivity_,
              min_distance_,
              max_distance_);
          camera.position = camera.target + view.normalized() * new_distance;
          return;
        }

        if constexpr (std::is_same_v<EventType, KeyEvent>) {
          return;
        }
      },
      event);
}

void OrbitCameraController::handleEvents(
    const std::vector<InputEvent>& events, Camera& camera)
{
  for (const auto& event : events) {
    handleEvent(event, camera);
  }
}

} // namespace gui
} // namespace dart
