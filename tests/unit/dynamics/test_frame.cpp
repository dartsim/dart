/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#include <gtest/gtest.h>

#include "dart/dynamics/dynamics.hpp"

using namespace Eigen;
using namespace dart;
using namespace dart::common;
using namespace dart::dynamics;

//==============================================================================
void frameChangeCallback(
    const Entity* _entity,
    const Frame* _oldParentFrame,
    const Frame* _newParentFrame) {
  assert(_entity);

  std::string oldFrameName
      = _oldParentFrame == nullptr ? "(empty)" : _oldParentFrame->getName();
  std::string newFrameName
      = _newParentFrame == nullptr ? "(empty)" : _newParentFrame->getName();

  if (_newParentFrame)
    std::cout << "[" << _entity->getName() << "]: " << oldFrameName << " --> "
              << newFrameName << std::endl;
  else
    std::cout << "Entity (" << _entity << ") has been destroyed" << std::endl;
}

//==============================================================================
void nameChangedCallback(
    const Entity* entity,
    const std::string& oldName,
    const std::string& newName) {
  assert(entity);

  std::cout << "[" << entity->getName() << "]: Name changed: '" << oldName
            << "' --> '" << newName << "'.\n";
}

//==============================================================================
TEST(FrameTest, FrameSignals) {
  Isometry3d tf1(Isometry3d::Identity());
  tf1.translate(Vector3d(0.1, -0.1, 0));

  Isometry3d tf2(Isometry3d::Identity());
  tf2.translate(Vector3d(0, 0.1, 0));
  tf2.rotate(AngleAxisd(dart::math::toRadian(45.0), Vector3d(1, 0, 0)));

  Isometry3d tf3(Isometry3d::Identity());
  tf3.translate(Vector3d(0, 0, 0.1));
  tf3.rotate(AngleAxisd(dart::math::toRadian(60.0), Vector3d(0, 1, 0)));

  SimpleFrame F1(Frame::World(), "F1", tf1);
  SimpleFrame F2(&F1, "F2", tf2);
  SimpleFrame F3(&F2, "F3", tf3);

  Connection cf1 = F1.onFrameChanged.connect(frameChangeCallback);
  Connection cf2 = F2.onFrameChanged.connect(frameChangeCallback);
  ScopedConnection cf3(F3.onFrameChanged.connect(frameChangeCallback));

  Connection cv1 = F1.onNameChanged.connect(nameChangedCallback);
  Connection cv2 = F2.onNameChanged.connect(nameChangedCallback);
  ScopedConnection cv3(F3.onNameChanged.connect(nameChangedCallback));

  F1.setName("new " + F1.getName());
  F2.setName("new " + F2.getName());
  F3.setName("new " + F3.getName());

  F3.setParentFrame(&F1);
}
