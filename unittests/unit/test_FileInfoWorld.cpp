/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#include <iostream>
#include <fstream>
#include <gtest/gtest.h>
#include "TestHelpers.hpp"

#include "dart/math/Geometry.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/simulation/World.hpp"
#include "dart/io/SkelParser.hpp"
#include "dart/io/FileInfoWorld.hpp"

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace simulation;
using namespace io;

//==============================================================================
TEST(FileInfoWorld, Basic)
{
  const std::size_t numFrames = 100;
  const std::string fileName = "testWorld.txt";
  double tol = 1e-6;
  bool result = false;
  FileInfoWorld worldFile;

  WorldPtr world = SkelParser::readWorld(
      "dart://sample/skel/test/file_info_world_test.skel");
  EXPECT_TRUE(world != nullptr);

  Recording* recording1 = nullptr;
  Recording* recording2 = nullptr;

  // Do some simulation with recording
  for (std::size_t i = 0; i < numFrames; ++i)
  {
    world->step();
    world->bake();
  }

  recording1 = world->getRecording();
  EXPECT_TRUE(recording1 != nullptr);

  // Save the recording to a file
  result = worldFile.saveFile(fileName.c_str(), world->getRecording());
  EXPECT_TRUE(result);

  // Load the file
  result = worldFile.loadFile(fileName.c_str());
  EXPECT_TRUE(result);
  recording2 = worldFile.getRecording();
  EXPECT_TRUE(recording2 != nullptr);

  // Check number of frames
  EXPECT_EQ(recording1->getNumFrames(), (int)numFrames);
  EXPECT_EQ(recording2->getNumFrames(), (int)numFrames);

  // Check number of skeletons
  std::size_t numSkeletons = recording1->getNumSkeletons();
  EXPECT_EQ(recording1->getNumSkeletons(), recording2->getNumSkeletons());

  // Check number of dofs of the skeletons
  for (std::size_t i = 0; i < numSkeletons; ++i)
    EXPECT_EQ(recording1->getNumDofs(i), recording2->getNumDofs(i));

  // Check generalized positions and contact info
  for (std::size_t i = 0; i < numFrames; ++i)
  {
    // Check generalized positions
    for (std::size_t j = 0; j < numSkeletons; ++j)
    {
      std::size_t dofs = recording1->getNumDofs(j);

      for (std::size_t k = 0; k < dofs; ++k)
      {
        EXPECT_NEAR(recording1->getGenCoord(i, j, k),
                    recording2->getGenCoord(i, j, k), tol);
      }
    }

    // Check contact info
    tol = 1e-3;
    std::size_t numContacts = recording1->getNumContacts(i);
    EXPECT_EQ(recording1->getNumContacts(i), recording2->getNumContacts(i));

    for (std::size_t j = 0; j < numContacts; ++j)
    {
      for (std::size_t k = 0; k < 3; ++k)
      {
        EXPECT_NEAR(recording1->getContactForce(i, j)[k],
                    recording2->getContactForce(i, j)[k], tol);

        EXPECT_NEAR(recording1->getContactPoint(i, j)[k],
                    recording2->getContactPoint(i, j)[k], tol);
      }
    }
  }
}
