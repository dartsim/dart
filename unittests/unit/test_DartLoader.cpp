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
#include <gtest/gtest.h>
#include "dart/io/urdf/DartLoader.hpp"

using dart::common::Uri;
using dart::io::DartLoader;

TEST(DartLoader, parseSkeleton_NonExistantPathReturnsNull)
{
  DartLoader loader;
  EXPECT_EQ(nullptr,
    loader.parseSkeleton("dart://sample/skel/test/does_not_exist.urdf"));
}

TEST(DartLoader, parseSkeleton_InvalidUrdfReturnsNull)
{
  DartLoader loader;
  EXPECT_EQ(nullptr,
    loader.parseSkeleton("dart://sample/urdf/test/invalid.urdf)"));
}

TEST(DartLoader, parseSkeleton_MissingMeshReturnsNull)
{
  DartLoader loader;
  EXPECT_EQ(nullptr,
    loader.parseSkeleton("dart://sample/urdf/test/missing_mesh.urdf"));
}

TEST(DartLoader, parseSkeleton_InvalidMeshReturnsNull)
{
  DartLoader loader;
  EXPECT_EQ(nullptr,
    loader.parseSkeleton("dart://sample/urdf/test/invalid_mesh.urdf"));
}

TEST(DartLoader, parseSkeleton_MissingPackageReturnsNull)
{
  DartLoader loader;
  EXPECT_EQ(nullptr,
    loader.parseSkeleton("dart://sample/urdf/test/missing_package.urdf"));
}

TEST(DartLoader, parseSkeleton_LoadsPrimitiveGeometry)
{
  DartLoader loader;
  EXPECT_TRUE(nullptr !=
    loader.parseSkeleton("dart://sample/urdf/test/primitive_geometry.urdf"));
}

TEST(DartLoader, parseWorld)
{
  DartLoader loader;
  EXPECT_TRUE(nullptr !=
      loader.parseWorld("dart://sample/urdf/test/testWorld.urdf"));
}

TEST(DartLoader, parseJointProperties)
{
  std::string urdfStr =
    "<robot name=\"testRobot\">                                       "
    "  <link name=\"link_0\">                                         "
    "    <visual>                                                     "
    "      <origin rpy=\"0 0 0\" xyz=\"0 0 -0.087\"/>                 "
    "      <geometry>                                                 "
    "        <box size=\"1 1 1\"/>                                    "
    "      </geometry>                                                "
    "    </visual>                                                    "
    "    <visual>                                                     "
    "      <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>                      "
    "      <geometry>                                                 "
    "        <box size=\"1 1 1\"/>                                    "
    "      </geometry>                                                "
    "    </visual>                                                    "
    "  </link>                                                        "
    "  <joint name=\"0_to_1\" type=\"revolute\">                      "
    "    <parent link=\"link_0\"/>                                    "
    "    <child link=\"link_1\"/>                                     "
    "    <limit effort=\"2.5\" lower=\"-3.14159265359\"               "
    "           upper=\"3.14159265359\" velocity=\"3.00545697193\"/>  "
    "    <axis xyz=\"0 0 1\"/>                                        "
    "    <dynamics damping=\"1.2\" friction=\"2.3\"/>                 "
    "  </joint>                                                       "
    "  <link name=\"link_1\">                                         "
    "    <visual>                                                     "
    "      <origin rpy=\"0 0 0\" xyz=\"0 0 -0.087\"/>                 "
    "      <geometry>                                                 "
    "        <box size=\"1 1 1\"/>                                    "
    "      </geometry>                                                "
    "    </visual>                                                    "
    "    <visual>                                                     "
    "      <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>                      "
    "      <geometry>                                                 "
    "        <box size=\"1 1 1\"/>                                    "
    "      </geometry>                                                "
    "    </visual>                                                    "
    "  </link>                                                        "
    "  <joint name=\"1_to_2\" type=\"continuous\">                    "
    "    <parent link=\"link_1\"/>                                    "
    "    <child link=\"link_2\"/>                                     "
    "    <limit effort=\"2.5\" velocity=\"3.00545697193\"/>           "
    "    <axis xyz=\"0 0 1\"/>                                        "
    "    <dynamics damping=\"1.2\" friction=\"2.3\"/>                 "
    "  </joint>                                                       "
    "  <link name=\"link_2\">                                         "
    "    <visual>                                                     "
    "      <origin rpy=\"0 0 0\" xyz=\"0 0 -0.087\"/>                 "
    "      <geometry>                                                 "
    "        <box size=\"1 1 1\"/>                                    "
    "      </geometry>                                                "
    "    </visual>                                                    "
    "    <visual>                                                     "
    "      <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>                      "
    "      <geometry>                                                 "
    "        <box size=\"1 1 1\"/>                                    "
    "      </geometry>                                                "
    "    </visual>                                                    "
    "  </link>                                                        "
    "</robot>                                                         ";

  DartLoader loader;
  auto robot = loader.parseSkeletonString(urdfStr, "");
  EXPECT_TRUE(nullptr != robot);

  auto joint1 = robot->getJoint(1);
  EXPECT_TRUE(nullptr != joint1);
  EXPECT_NEAR(joint1->getDampingCoefficient(0), 1.2, 1e-12);
  EXPECT_NEAR(joint1->getCoulombFriction(0), 2.3, 1e-12);

  auto joint2 = robot->getJoint(2);
  EXPECT_DOUBLE_EQ(joint2->getPositionLowerLimit(0), -dart::math::constantsd::inf());
  EXPECT_DOUBLE_EQ(joint2->getPositionUpperLimit(0), dart::math::constantsd::inf());
  EXPECT_TRUE(joint2->isCyclic(0));
}

TEST(DartLoader, mimicJoint)
{
  std::string urdfStr =
    "<robot name=\"testRobot\">                                       "
    "  <link name=\"link_0\">                                         "
    "    <visual>                                                     "
    "      <origin rpy=\"0 0 0\" xyz=\"0 0 -0.087\"/>                 "
    "      <geometry>                                                 "
    "        <box size=\"1 1 1\"/>                                    "
    "      </geometry>                                                "
    "    </visual>                                                    "
    "    <visual>                                                     "
    "      <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>                      "
    "      <geometry>                                                 "
    "        <box size=\"1 1 1\"/>                                    "
    "      </geometry>                                                "
    "    </visual>                                                    "
    "  </link>                                                        "
    "  <joint name=\"0_to_1\" type=\"revolute\">                      "
    "    <parent link=\"link_0\"/>                                    "
    "    <child link=\"link_1\"/>                                     "
    "    <limit effort=\"2.5\" lower=\"-3.14159265359\"               "
    "           upper=\"3.14159265359\" velocity=\"3.00545697193\"/>  "
    "    <axis xyz=\"0 0 1\"/>                                        "
    "    <dynamics damping=\"1.2\" friction=\"2.3\"/>                 "
    "  </joint>                                                       "
    "  <link name=\"link_1\">                                         "
    "    <visual>                                                     "
    "      <origin rpy=\"0 0 0\" xyz=\"0 0 -0.087\"/>                 "
    "      <geometry>                                                 "
    "        <box size=\"1 1 1\"/>                                    "
    "      </geometry>                                                "
    "    </visual>                                                    "
    "    <visual>                                                     "
    "      <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>                      "
    "      <geometry>                                                 "
    "        <box size=\"1 1 1\"/>                                    "
    "      </geometry>                                                "
    "    </visual>                                                    "
    "  </link>                                                        "
    "  <joint name=\"1_to_2\" type=\"continuous\">                    "
    "    <parent link=\"link_1\"/>                                    "
    "    <child link=\"link_2\"/>                                     "
    "    <limit effort=\"2.5\" velocity=\"3.00545697193\"/>           "
    "    <axis xyz=\"0 0 1\"/>                                        "
    "    <dynamics damping=\"1.2\" friction=\"2.3\"/>                 "
    "    <mimic joint=\"0_to_1\" multiplier=\"2.\" offset=\"0.1\"/>   "
    "  </joint>                                                       "
    "  <link name=\"link_2\">                                         "
    "    <visual>                                                     "
    "      <origin rpy=\"0 0 0\" xyz=\"0 0 -0.087\"/>                 "
    "      <geometry>                                                 "
    "        <box size=\"1 1 1\"/>                                    "
    "      </geometry>                                                "
    "    </visual>                                                    "
    "    <visual>                                                     "
    "      <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>                      "
    "      <geometry>                                                 "
    "        <box size=\"1 1 1\"/>                                    "
    "      </geometry>                                                "
    "    </visual>                                                    "
    "  </link>                                                        "
    "</robot>                                                         ";

  DartLoader loader;
  auto robot = loader.parseSkeletonString(urdfStr, "");
  EXPECT_TRUE(nullptr != robot);

  auto joint1 = robot->getJoint(1);
  EXPECT_TRUE(nullptr != joint1);
  EXPECT_NEAR(joint1->getDampingCoefficient(0), 1.2, 1e-12);
  EXPECT_NEAR(joint1->getCoulombFriction(0), 2.3, 1e-12);

  auto joint2 = robot->getJoint(2);
  EXPECT_DOUBLE_EQ(joint2->getPositionLowerLimit(0), -dart::math::constantsd::inf());
  EXPECT_DOUBLE_EQ(joint2->getPositionUpperLimit(0), dart::math::constantsd::inf());
  EXPECT_TRUE(joint2->isCyclic(0));

  EXPECT_TRUE(joint2->getActuatorType() == dart::dynamics::Joint::MIMIC);
  EXPECT_TRUE(nullptr != joint2->getMimicJoint());
  EXPECT_DOUBLE_EQ(joint2->getMimicMultiplier(), 2.);
  EXPECT_DOUBLE_EQ(joint2->getMimicOffset(), 0.1);
}


TEST(DartLoader, badMimicJoint)
{
  std::string urdfStr =
    "<robot name=\"testRobot\">                                       "
    "  <link name=\"link_0\">                                         "
    "    <visual>                                                     "
    "      <origin rpy=\"0 0 0\" xyz=\"0 0 -0.087\"/>                 "
    "      <geometry>                                                 "
    "        <box size=\"1 1 1\"/>                                    "
    "      </geometry>                                                "
    "    </visual>                                                    "
    "    <visual>                                                     "
    "      <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>                      "
    "      <geometry>                                                 "
    "        <box size=\"1 1 1\"/>                                    "
    "      </geometry>                                                "
    "    </visual>                                                    "
    "  </link>                                                        "
    "  <joint name=\"0_to_1\" type=\"revolute\">                      "
    "    <parent link=\"link_0\"/>                                    "
    "    <child link=\"link_1\"/>                                     "
    "    <limit effort=\"2.5\" lower=\"-3.14159265359\"               "
    "           upper=\"3.14159265359\" velocity=\"3.00545697193\"/>  "
    "    <axis xyz=\"0 0 1\"/>                                        "
    "    <dynamics damping=\"1.2\" friction=\"2.3\"/>                 "
    "  </joint>                                                       "
    "  <link name=\"link_1\">                                         "
    "    <visual>                                                     "
    "      <origin rpy=\"0 0 0\" xyz=\"0 0 -0.087\"/>                 "
    "      <geometry>                                                 "
    "        <box size=\"1 1 1\"/>                                    "
    "      </geometry>                                                "
    "    </visual>                                                    "
    "    <visual>                                                     "
    "      <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>                      "
    "      <geometry>                                                 "
    "        <box size=\"1 1 1\"/>                                    "
    "      </geometry>                                                "
    "    </visual>                                                    "
    "  </link>                                                        "
    "  <joint name=\"1_to_2\" type=\"continuous\">                    "
    "    <parent link=\"link_1\"/>                                    "
    "    <child link=\"link_2\"/>                                     "
    "    <limit effort=\"2.5\" velocity=\"3.00545697193\"/>           "
    "    <axis xyz=\"0 0 1\"/>                                        "
    "    <dynamics damping=\"1.2\" friction=\"2.3\"/>                 "
    "    <mimic joint=\"mjoint\" multiplier=\"2.\" offset=\"0.1\"/>   "
    "  </joint>                                                       "
    "  <link name=\"link_2\">                                         "
    "    <visual>                                                     "
    "      <origin rpy=\"0 0 0\" xyz=\"0 0 -0.087\"/>                 "
    "      <geometry>                                                 "
    "        <box size=\"1 1 1\"/>                                    "
    "      </geometry>                                                "
    "    </visual>                                                    "
    "    <visual>                                                     "
    "      <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>                      "
    "      <geometry>                                                 "
    "        <box size=\"1 1 1\"/>                                    "
    "      </geometry>                                                "
    "    </visual>                                                    "
    "  </link>                                                        "
    "</robot>                                                         ";

  DartLoader loader;
  auto robot = loader.parseSkeletonString(urdfStr, "");
  EXPECT_TRUE(nullptr != robot);

  auto joint1 = robot->getJoint(1);
  EXPECT_TRUE(nullptr != joint1);
  EXPECT_NEAR(joint1->getDampingCoefficient(0), 1.2, 1e-12);
  EXPECT_NEAR(joint1->getCoulombFriction(0), 2.3, 1e-12);

  auto joint2 = robot->getJoint(2);
  EXPECT_DOUBLE_EQ(joint2->getPositionLowerLimit(0), -dart::math::constantsd::inf());
  EXPECT_DOUBLE_EQ(joint2->getPositionUpperLimit(0), dart::math::constantsd::inf());
  EXPECT_TRUE(joint2->isCyclic(0));

  EXPECT_TRUE(joint2->getActuatorType() != dart::dynamics::Joint::MIMIC);
  EXPECT_TRUE(nullptr == joint2->getMimicJoint());
}
