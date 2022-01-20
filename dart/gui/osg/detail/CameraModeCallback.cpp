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

#include "dart/gui/osg/detail/CameraModeCallback.hpp"

#include "dart/common/Logging.hpp"
#include "dart/common/Macros.hpp"
#include "dart/gui/osg/Utils.hpp"

namespace dart::gui::osg::detail {

//==============================================================================
CameraModeCallback::CameraModeCallback()
  : mCameraMode(CameraMode::RGBA), mCameraModeChanged(false)
{
  // Do nothing
}

//==============================================================================
void CameraModeCallback::operator()(::osg::Node* node, ::osg::NodeVisitor* nv)
{
  ::osg::ref_ptr<::osg::Group> group = dynamic_cast<::osg::Group*>(node);

  if (group)
  {
    std::lock_guard<std::mutex> lock(mMutex);

    if (mSceneToChange)
    {
      mScene = mSceneToChange;
      if (mDepthRrtCam)
      {
        mDepthRrtCam->removeChildren(0, mDepthHudCam->getNumChildren());
        mDepthRrtCam->addChild(mScene);
      }
      mSceneToChange = nullptr;
    }

    if (mCameraModeChanged)
    {
      if (mCameraMode == CameraMode::RGBA)
      {
        if (mDepthRrtCam)
        {
          group->removeChild(mDepthRrtCam);
          mDepthRrtCam.release();
        }

        if (mDepthHudCam)
        {
          group->removeChild(mDepthHudCam);
          mDepthHudCam.release();
        }
      }
      else if (mCameraMode == CameraMode::DEPTH)
      {
        // Allocate an empty texture by specifying its size for RTT operation
        ::osg::ref_ptr<::osg::Texture2D> tex2d = new ::osg::Texture2D;
        tex2d->setTextureSize(1024, 1024);
        tex2d->setInternalFormat(GL_DEPTH_COMPONENT24);
        tex2d->setSourceFormat(GL_DEPTH_COMPONENT);
        tex2d->setSourceType(GL_FLOAT);
        // TODO(JS): Make these configurable

        // Create a RTT camera and apply the required buffer value
        // (DEPTH_BUFFER) and the texture object to it
        mDepthRrtCam = createRttCamera(::osg::Camera::DEPTH_BUFFER, tex2d);

        // Create a HUD camera over the scene, and add a quad shown over the
        // screen
        mDepthHudCam = createHudCamera(0, 1, 0, 1);
        mDepthHudCam->addChild(createScreenQuad(1.0, 1.0));
        mDepthHudCam->getOrCreateStateSet()->setTextureAttributeAndModes(
            0, tex2d);
        mDepthRrtCam->setComputeNearFarMode(
            ::osg::CullSettings::COMPUTE_NEAR_FAR_USING_PRIMITIVES);
        // TODO(JS): Make these configurable

        if (mScene)
        {
          mDepthRrtCam->addChild(mScene);
        }

        group->addChild(mDepthRrtCam);
        group->addChild(mDepthHudCam);
      }
      else
      {
        DART_ASSERT(false);
      }

      mCameraModeChanged = false;
    }
  }

  traverse(node, nv);
}

//==============================================================================
void CameraModeCallback::setCameraMode(CameraMode mode)
{
  if (mode != CameraMode::RGBA && mode != CameraMode::DEPTH)
  {
    DART_WARN(
        "Unsupported camera mode '{}'. Use RGBA or DEPTH.",
        static_cast<int>(mode));
    return;
  }

  std::lock_guard<std::mutex> lock(mMutex);
  if (mode == mCameraMode)
  {
    return;
  }

  mCameraMode = mode;
  mCameraModeChanged = true;
}

//==============================================================================
CameraMode CameraModeCallback::getCameraMode() const
{
  std::lock_guard<std::mutex> lock(mMutex);
  return mCameraMode;
}

//==============================================================================
void CameraModeCallback::setSceneData(::osg::Node* scene)
{
  std::lock_guard<std::mutex> lock(mMutex);
  if (scene == mScene)
  {
    return;
  }
  mSceneToChange = scene;
}

} // namespace dart::gui::osg::detail
