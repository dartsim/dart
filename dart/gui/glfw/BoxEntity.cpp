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

#include "dart/gui/glfw/BoxEntity.hpp"

namespace dart {
namespace gui {
namespace glfw {

//==============================================================================
BoxEntity::BoxEntity()
{
  // Do nothing
}

//==============================================================================
BoxEntity::~BoxEntity()
{
  // Do nothing
}

//==============================================================================
void BoxEntity::initialize()
{
  mVAO.initialize();

  mVAO.bind();

  mVAO.mVBO.bind();
  mVAO.mIBO.bind();

  mVAO.mVBO.mData.resize(4);
  mVAO.mIBO.mData.resize(6);

  mVAO.mVBO.mData[0] = {0.5f, 0.5f, 0.0f};   // top right
  mVAO.mVBO.mData[1] = {0.5f, -0.5f, 0.0f};  // bottom right
  mVAO.mVBO.mData[2] = {-0.5f, -0.5f, 0.0f}; // bottom left
  mVAO.mVBO.mData[3] = {-0.5f, 0.5f, 0.0f};  // top left

  mVAO.mIBO.mData[0] = 0; // first triangle
  mVAO.mIBO.mData[1] = 1;
  mVAO.mIBO.mData[2] = 3;
  mVAO.mIBO.mData[3] = 1; // second triangl
  mVAO.mIBO.mData[4] = 2;
  mVAO.mIBO.mData[5] = 3;
}

//==============================================================================
void BoxEntity::release()
{
  mVAO.release();
}

//==============================================================================
void BoxEntity::render()
{
  mVAO.draw();
}

} // namespace glfw
} // namespace gui
} // namespace dart
