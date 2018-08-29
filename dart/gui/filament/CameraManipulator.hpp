/*
 * Copyright (c) 2011-2018, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

/*
 * Copyright (C) 2016 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef DART_GUI_FILAMENT_CAMERAMANIPULATOR_HPP_
#define DART_GUI_FILAMENT_CAMERAMANIPULATOR_HPP_

#include <math/mat4.h>
#include <math/vec2.h>
#include <math/vec3.h>
#include <math/vec4.h>

#include <filament/Camera.h>

namespace dart {
namespace gui {
namespace flmt {

class CameraManipulator
{
public:
  using Camera = filament::Camera;
  using CameraChangedCallback = std::function<void(Camera const* c)>;

  CameraManipulator();
  CameraManipulator(Camera* camera, size_t width, size_t height);

  void setCameraChangedCallback(CameraChangedCallback cb);

  void setCamera(Camera* camera);
  Camera const* getCamera() const
  {
    return mCamera;
  }

  void setViewport(size_t w, size_t h);

  void lookAt(const ::math::double3& eye, const ::math::double3& at);
  void track(const ::math::double2& delta);
  void dolly(double delta, double dollySpeed = 5.0);
  void rotate(const ::math::double2& delta, double rotateSpeed = 7.0);

  void updateCameraTransform();

private:
  void updateCameraProjection();

  Camera* mCamera = nullptr;

  ::math::double3 mRotation;
  ::math::double3 mTranslation;

  double mCenterOfInterest = 10.0;
  double mFovx = 65.0;
  double mClipNear = 0.1;
  double mClipFar = 11.0;
  size_t mWidth;
  size_t mHeight;
  double mAspect = 1.0;

  std::function<void(Camera const* c)> mCameraChanged;
};

} // namespace flmt
} // namespace gui
} // namespace dart

#endif // DART_GUI_FILAMENT_CAMERAMANIPULATOR_HPP_
