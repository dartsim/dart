/*
 * Copyright (c) 2011-2017, The DART development contributors
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

#include <gtest/gtest.h>
#include "dart/dart.hpp"
#include "dart/utils/urdf/DartLoader.hpp"
#include "TestHelpers.hpp"

using namespace dart;
using namespace dart::math;
using namespace dart::collision;
using namespace dart::dynamics;
using namespace dart::simulation;

//==============================================================================
TEST(Sensor, ImuSensor)
{
  // This is a regression test for pull request #683
  const double tolerance = 1e-8;

  dart::utils::DartLoader loader;
  SkeletonPtr skeleton1
      = loader.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf");

  BodyNode* lastBn1 = skeleton1->getBodyNode(skeleton1->getNumBodyNodes() - 1);
  Sensor* imuSensor1 = lastBn1->createSensor<Sensor>();
  Eigen::Isometry3d tf1 = imuSensor1->getTransform();

  EXPECT_TRUE(equals(tf1, lastBn1->getTransform()));

  SkeletonPtr skeleton2 = skeleton1->clone();
  BodyNode* lastBn2 = skeleton2->getBodyNode(skeleton2->getNumBodyNodes() - 1);
  Sensor* imuSensor2 = lastBn2->createSensor<Sensor>();

  Eigen::Isometry3d tf2 = imuSensor2->getTransform();

  EXPECT_TRUE(equals(tf2, lastBn2->getTransform()));

  std::vector<std::size_t> active_indices;
  for (std::size_t i = 0; i < 3; ++i)
    active_indices.push_back(i);

  Eigen::VectorXd q = Eigen::VectorXd::Random(active_indices.size());
}
