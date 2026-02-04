// Copyright (c) 2011, The DART development contributors
// All rights reserved.
//
// The list of contributors can be found at:
//   https://github.com/dartsim/dart/blob/main/LICENSE
//
// This file is provided under the following "BSD-style" License:
//   Redistribution and use in source and binary forms, with or
//   without modification, are permitted provided that the following
//   conditions are met:
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above
//     copyright notice, this list of conditions and the following
//     disclaimer in the documentation and/or other materials provided
//     with the distribution.
//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
//   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
//   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
//   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
//   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
//   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//   POSSIBILITY OF SUCH DAMAGE.

#include <dart/config.hpp>

#if DART_HAVE_ODE

  #include <dart/collision/collision_group.hpp>
  #include <dart/collision/collision_option.hpp>
  #include <dart/collision/collision_result.hpp>
  #include <dart/collision/ode/ode_collision_detector.hpp>

  #include <dart/dynamics/box_shape.hpp>
  #include <dart/dynamics/cylinder_shape.hpp>
  #include <dart/dynamics/mesh_shape.hpp>
  #include <dart/dynamics/simple_frame.hpp>
  #include <dart/dynamics/sphere_shape.hpp>

  #include <dart/math/tri_mesh.hpp>

  #include <gtest/gtest.h>

using dart::collision::CollisionOption;
using dart::collision::CollisionResult;
using dart::collision::OdeCollisionDetector;
using dart::dynamics::BoxShape;
using dart::dynamics::CylinderShape;
using dart::dynamics::Frame;
using dart::dynamics::MeshShape;
using dart::dynamics::SimpleFrame;
using dart::dynamics::SphereShape;
using dart::math::TriMeshd;

namespace {

std::shared_ptr<SimpleFrame> makeFrame(
    const std::shared_ptr<dart::dynamics::Shape>& shape,
    const Eigen::Vector3d& translation)
{
  auto frame = SimpleFrame::createShared(Frame::World());
  frame->setShape(shape);
  frame->setTranslation(translation);
  return frame;
}

bool collidePair(
    const std::shared_ptr<OdeCollisionDetector>& ode,
    const std::shared_ptr<SimpleFrame>& first,
    const std::shared_ptr<SimpleFrame>& second,
    CollisionResult* result)
{
  auto group = ode->createCollisionGroup(first.get(), second.get());
  CollisionOption option;
  option.enableContact = true;
  return group->collide(option, result);
}

} // namespace

//==============================================================================
TEST(OdeCylinderMesh, CylinderSphereCollision)
{
  auto ode = OdeCollisionDetector::create();
  ASSERT_TRUE(ode);

  auto cylinder = std::make_shared<CylinderShape>(0.5, 1.0);
  auto sphere = std::make_shared<SphereShape>(0.4);

  auto cylinderFrame = makeFrame(cylinder, Eigen::Vector3d::Zero());
  auto sphereFrame = makeFrame(sphere, Eigen::Vector3d(0.0, 0.0, 0.0));

  CollisionResult result;
  EXPECT_TRUE(collidePair(ode, cylinderFrame, sphereFrame, &result));
  EXPECT_GT(result.getNumContacts(), 0u);
}

//==============================================================================
TEST(OdeCylinderMesh, CylinderCylinderCollision)
{
  auto ode = OdeCollisionDetector::create();
  ASSERT_TRUE(ode);

  auto cylinder = std::make_shared<CylinderShape>(0.5, 1.0);

  auto firstFrame = makeFrame(cylinder, Eigen::Vector3d::Zero());
  auto secondFrame = makeFrame(cylinder, Eigen::Vector3d(0.2, 0.0, 0.0));

  CollisionResult result;
  EXPECT_TRUE(collidePair(ode, firstFrame, secondFrame, &result));
  EXPECT_GT(result.getNumContacts(), 0u);
}

//==============================================================================
TEST(OdeCylinderMesh, CylinderMeshCollision)
{
  auto ode = OdeCollisionDetector::create();
  ASSERT_TRUE(ode);

  auto cylinder = std::make_shared<CylinderShape>(0.5, 1.0);

  auto triMesh = std::make_shared<TriMeshd>();
  TriMeshd::Vertices vertices;
  vertices.emplace_back(-0.5, -0.5, -0.5);
  vertices.emplace_back(0.5, -0.5, -0.5);
  vertices.emplace_back(0.5, 0.5, -0.5);
  vertices.emplace_back(-0.5, 0.5, -0.5);
  vertices.emplace_back(-0.5, -0.5, 0.5);
  vertices.emplace_back(0.5, -0.5, 0.5);
  vertices.emplace_back(0.5, 0.5, 0.5);
  vertices.emplace_back(-0.5, 0.5, 0.5);
  TriMeshd::Triangles triangles;
  // Front face
  triangles.emplace_back(0, 1, 2);
  triangles.emplace_back(0, 2, 3);
  // Back face
  triangles.emplace_back(4, 6, 5);
  triangles.emplace_back(4, 7, 6);
  // Bottom face
  triangles.emplace_back(0, 5, 1);
  triangles.emplace_back(0, 4, 5);
  // Top face
  triangles.emplace_back(2, 7, 3);
  triangles.emplace_back(2, 6, 7);
  // Left face
  triangles.emplace_back(0, 3, 7);
  triangles.emplace_back(0, 7, 4);
  // Right face
  triangles.emplace_back(1, 5, 6);
  triangles.emplace_back(1, 6, 2);
  triMesh->setTriangles(vertices, triangles);

  auto meshShape
      = std::make_shared<MeshShape>(Eigen::Vector3d::Ones(), triMesh);

  auto cylinderFrame = makeFrame(cylinder, Eigen::Vector3d::Zero());
  auto meshFrame = makeFrame(meshShape, Eigen::Vector3d(0.1, 0.0, 0.0));

  CollisionResult result;
  EXPECT_TRUE(collidePair(ode, cylinderFrame, meshFrame, &result));
  EXPECT_GT(result.getNumContacts(), 0u);
}

//==============================================================================
TEST(OdeCylinderMesh, CylinderSeparated)
{
  auto ode = OdeCollisionDetector::create();
  ASSERT_TRUE(ode);

  auto cylinder = std::make_shared<CylinderShape>(0.5, 1.0);
  auto sphere = std::make_shared<SphereShape>(0.4);

  auto cylinderFrame = makeFrame(cylinder, Eigen::Vector3d::Zero());
  auto sphereFrame = makeFrame(sphere, Eigen::Vector3d(5.0, 0.0, 0.0));

  CollisionResult result;
  EXPECT_FALSE(collidePair(ode, cylinderFrame, sphereFrame, &result));
  EXPECT_EQ(result.getNumContacts(), 0u);
}

#endif // DART_HAVE_ODE
