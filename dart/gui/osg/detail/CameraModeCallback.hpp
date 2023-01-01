/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#ifndef DART_GUI_OSG_DETAIL_CAMERAMODECALLBACK_HPP_
#define DART_GUI_OSG_DETAIL_CAMERAMODECALLBACK_HPP_

#include "dart/gui/osg/Viewer.hpp"

#include <osg/NodeCallback>

#include <mutex>

namespace dart::gui::osg::detail {

class CameraModeCallback : public ::osg::NodeCallback
{
public:
  /// Default constructor
  CameraModeCallback();

  // Documentation inherited
  void operator()(::osg::Node* node, ::osg::NodeVisitor* nv) override;

  /// Sets the camera mode of the primary camera.
  ///
  /// \note Thread safe
  void setCameraMode(CameraMode mode);

  /// Returns the camera mode of the primary camera.
  ///
  /// \note Thread safe
  CameraMode getCameraMode() const;

  /// Sets the scene to render the depth
  ///
  /// \note Thread safe
  void setSceneData(::osg::Node* scene);

private:
  ::osg::ref_ptr<::osg::Camera> mDepthRrtCam;
  ::osg::ref_ptr<::osg::Camera> mDepthHudCam;
  CameraMode mCameraMode;
  bool mCameraModeChanged;
  ::osg::ref_ptr<::osg::Node> mScene;
  ::osg::ref_ptr<::osg::Node> mSceneToChange;

  /// Mutex for all the member variables
  mutable std::mutex mMutex;
};

} // namespace dart::gui::osg::detail

#endif // DART_GUI_OSG_DETAIL_CAMERAMODECALLBACK_HPP_
