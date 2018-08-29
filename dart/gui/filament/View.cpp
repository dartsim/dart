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

#include "dart/gui/filament/View.hpp"

#include "stb_image.h"

#include <fstream>
#include <string>

#include <filament/Engine.h>
#include <filament/IndirectLight.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/Skybox.h>
#include <filament/Texture.h>

namespace dart {
namespace gui {
namespace flmt {

CView::CView(filament::Renderer& renderer, std::string name)
  : engine(*renderer.getEngine()), mName(name)
{
  view = engine.createView();
  view->setClearColor({0});
  view->setName(name.c_str());
}

CView::~CView()
{
  engine.destroy(view);
}

void CView::setViewport(filament::Viewport const& viewport)
{
  mViewport = viewport;
  view->setViewport(viewport);
  if (mCameraManipulator)
  {
    mCameraManipulator->setViewport(viewport.width, viewport.height);
  }
}

void CView::mouseDown(int button, ssize_t x, ssize_t y)
{
  mLastMousePosition = math::double2(x, y);
  if (button == 1)
  {
    mMode = Mode::ROTATE;
  }
  else if (button == 3)
  {
    mMode = Mode::TRACK;
  }
}

void CView::mouseUp(ssize_t /*x*/, ssize_t /*y*/)
{
  mMode = Mode::NONE;
}

void CView::mouseMoved(ssize_t x, ssize_t y)
{
  if (mCameraManipulator)
  {
    math::double2 delta = math::double2(x, y) - mLastMousePosition;
    mLastMousePosition = math::double2(x, y);
    switch (mMode)
    {
      case Mode::NONE:
        break;
      case Mode::ROTATE:
        mCameraManipulator->rotate(delta);
        break;
      case Mode::TRACK:
        mCameraManipulator->track(delta);
        break;
    }
  }
}

void CView::mouseWheel(ssize_t x)
{
  if (mCameraManipulator)
  {
    mCameraManipulator->dolly(x);
  }
}

bool CView::intersects(ssize_t x, ssize_t y)
{
  if (x >= mViewport.left && x < mViewport.left + mViewport.width)
    if (y >= mViewport.bottom && y < mViewport.bottom + mViewport.height)
      return true;

  return false;
}

void CView::setCameraManipulator(CameraManipulator* cm)
{
  mCameraManipulator = cm;
}

void CView::setCamera(filament::Camera* camera)
{
  view->setCamera(camera);
}

void GodView::setGodCamera(filament::Camera* camera)
{
  getView()->setDebugCamera(camera);
}

} // namespace flmt
} // namespace gui
} // namespace dart
