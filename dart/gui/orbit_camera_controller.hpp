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

#ifndef DART_GUI_ORBIT_CAMERA_CONTROLLER_HPP_
#define DART_GUI_ORBIT_CAMERA_CONTROLLER_HPP_

#include <dart/gui/export.hpp>
#include <dart/gui/input_event.hpp>
#include <dart/gui/scene.hpp>

#include <vector>

namespace dart {
namespace gui {

class DART_GUI_API OrbitCameraController
{
public:
  explicit OrbitCameraController(
      double rotate_sensitivity = 0.3,
      double pan_sensitivity = 0.005,
      double zoom_sensitivity = 0.5,
      double min_distance = 0.1,
      double max_distance = 100.0);

  void handleEvent(const InputEvent& event, Camera& camera);
  void handleEvents(const std::vector<InputEvent>& events, Camera& camera);

private:
  bool right_button_down_ = false;
  bool middle_button_down_ = false;
  double rotate_sensitivity_;
  double pan_sensitivity_;
  double zoom_sensitivity_;
  double min_distance_;
  double max_distance_;
};

} // namespace gui
} // namespace dart

#endif // DART_GUI_ORBIT_CAMERA_CONTROLLER_HPP_
