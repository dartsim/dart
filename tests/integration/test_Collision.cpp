/*
 * Copyright (c) 2011, The DART development contributors
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

#include "dart/collision/collision.hpp"
#include "dart/collision/dart/DARTCollide.hpp"
#include "dart/collision/fcl/fcl.hpp"
#include "dart/common/common.hpp"
#include "dart/config.hpp"
#include "dart/dynamics/dynamics.hpp"
#include "dart/math/math.hpp"

#include <gtest/gtest.h>

#include <iostream>
#if HAVE_ODE
  #include "dart/collision/ode/ode.hpp"
#endif
#if HAVE_BULLET
  #include "dart/collision/bullet/bullet.hpp"
#endif
#include "TestHelpers.hpp"
#include "dart/simulation/simulation.hpp"
#include "dart/utils/utils.hpp"

#include <algorithm>
#include <limits>
#include <new>

using namespace dart;
using namespace common;
using namespace math;
using namespace collision;
using namespace dynamics;
using namespace simulation;
using namespace utils;

namespace {

class TestCollisionObject final : public CollisionObject
{
public:
  TestCollisionObject(
      CollisionDetector* collisionDetector, const ShapeFrame* shapeFrame)
    : CollisionObject(collisionDetector, shapeFrame)
  {
    // Do nothing
  }

private:
  void updateEngineData() override
  {
    // Do nothing
  }
};

class ToggleBodyNodeCollisionFilter : public BodyNodeCollisionFilter
{
public:
  ToggleBodyNodeCollisionFilter(
      const BodyNode* body1, const BodyNode* body2, bool ignorePair)
    : mBody1(body1), mBody2(body2), mIgnorePair(ignorePair)
  {
  }

  void setIgnorePair(bool ignorePair)
  {
    mIgnorePair = ignorePair;
  }

  bool ignoresCollision(
      const CollisionObject* object1,
      const CollisionObject* object2) const override
  {
    if (BodyNodeCollisionFilter::ignoresCollision(object1, object2))
      return true;

    const auto* body1 = object1->getBodyNode();
    const auto* body2 = object2->getBodyNode();
    const bool targetPair = (body1 == mBody1 && body2 == mBody2)
                            || (body1 == mBody2 && body2 == mBody1);
    return mIgnorePair && targetPair;
  }

private:
  const BodyNode* mBody1;
  const BodyNode* mBody2;
  bool mIgnorePair;
};

struct alignas(TestCollisionObject) TestCollisionObjectStorage
{
  unsigned char data[sizeof(TestCollisionObject)];
};

} // namespace

class Collision : public testing::Test
{
public:
  void unrotatedTest(
      dart::collision::fcl::CollisionGeometry* _coll1,
      dart::collision::fcl::CollisionGeometry* _coll2,
      double expectedContactPoint,
      int _idxAxis);
  void dropWithRotation(
      dart::collision::fcl::CollisionGeometry* _object,
      double EulerZ,
      double EulerY,
      double EulerX);
  void printResult(const dart::collision::fcl::CollisionResult& _result);
};

void Collision::unrotatedTest(
    dart::collision::fcl::CollisionGeometry* _coll1,
    dart::collision::fcl::CollisionGeometry* _coll2,
    double expectedContactPoint,
    int _idxAxis)
{
  dart::collision::fcl::CollisionResult result;
  dart::collision::fcl::CollisionRequest request;
  request.enable_contact = true;
  request.num_max_contacts = 100;

  dart::collision::fcl::Vector3 position(0, 0, 0);

  dart::collision::fcl::Transform3 coll1_transform;
  dart::collision::fcl::Transform3 coll2_transform;

  //==========================================================================
  // Approaching test
  //==========================================================================
  result.clear();
  double dpos = -0.001;
  double pos = 10.0;

  coll1_transform.setIdentity();
  dart::collision::fcl::setTranslation(
      coll1_transform, dart::collision::fcl::Vector3(0, 0, 0));
  coll2_transform.setIdentity();

  // Let's drop box2 until it collide with box1
  do {
    position[_idxAxis] = pos;
    dart::collision::fcl::setTranslation(coll2_transform, position);

    ::fcl::collide(
        _coll1, coll1_transform, _coll2, coll2_transform, request, result);

    pos += dpos;
  } while (result.numContacts() == 0);

  //
  if (_idxAxis == 0)
    std::cout << "The object is collided when its x-axis position is: "
              << (pos - dpos) << std::endl;
  if (_idxAxis == 1)
    std::cout << "The object is collided when its y-axis position is: "
              << (pos - dpos) << std::endl;
  if (_idxAxis == 2)
    std::cout << "The object is collided when its z-axis position is: "
              << (pos - dpos) << std::endl;

  // printResult(result);

  for (std::size_t i = 0; i < result.numContacts(); ++i) {
    EXPECT_GE(result.getContact(i).penetration_depth, 0.0);
    //		EXPECT_NEAR(result.getContact(i).normal[_idxAxis], -1.0);
    EXPECT_EQ(dart::collision::fcl::length(result.getContact(i).normal), 1.0);
    EXPECT_NEAR(
        result.getContact(i).pos[_idxAxis], expectedContactPoint, -dpos * 2.0);
  }
}

void Collision::dropWithRotation(
    dart::collision::fcl::CollisionGeometry* _object,
    double EulerZ,
    double EulerY,
    double EulerX)
{
  // Collision test setting
  dart::collision::fcl::CollisionResult result;
  dart::collision::fcl::CollisionRequest request;
  request.enable_contact = true;
  request.num_max_contacts = 100;

  // Ground like box setting
  dart::collision::fcl::Box groundObject(100, 100, 0.1);
  dart::collision::fcl::Transform3 groundTransf;
  groundTransf.setIdentity();
  dart::collision::fcl::Vector3 ground_position(0, 0, 0);
  dart::collision::fcl::setTranslation(groundTransf, ground_position);

  // Dropping object setting
  dart::collision::fcl::Transform3 objectTransf;
  dart::collision::fcl::Matrix3 rot;
  dart::collision::fcl::setEulerZYX(rot, EulerZ, EulerY, EulerX);
  dart::collision::fcl::setRotation(objectTransf, rot);
  dart::collision::fcl::Vector3 dropping_position(0, 0, 0);
  dart::collision::fcl::setTranslation(objectTransf, dropping_position);

  //==========================================================================
  // Dropping test in x, y, z aixs each.
  //==========================================================================
  for (int _idxAxis = 0; _idxAxis < 3; ++_idxAxis) {
    result.clear();

    groundObject.side = dart::collision::fcl::Vector3(100, 100, 100);
    groundObject.side[_idxAxis] = 0.1;
    ground_position = dart::collision::fcl::Vector3(0, 0, 0);
    ground_position[_idxAxis] = -0.05;
    dart::collision::fcl::setTranslation(groundTransf, ground_position);

    // Let's drop the object until it collide with ground
    double posDelta = -0.0001;
    double initPos = 10.0;
    dropping_position = dart::collision::fcl::Vector3(0, 0, 0);
    do {
      dropping_position[_idxAxis] = initPos;
      dart::collision::fcl::setTranslation(objectTransf, dropping_position);

      ::fcl::collide(
          _object, objectTransf, &groundObject, groundTransf, request, result);

      initPos += posDelta;
    } while (result.numContacts() == 0);

    std::cout << "Current position of the object: "
              << dart::collision::fcl::getTranslation(objectTransf) << std::endl
              << "Number of contacts: " << result.numContacts() << std::endl;

    dart::collision::fcl::Transform3 objectTransfInv = objectTransf;
    objectTransfInv.inverse();
    for (std::size_t i = 0; i < result.numContacts(); ++i) {
      dart::collision::fcl::Vector3 posWorld = dart::collision::fcl::transform(
          objectTransfInv, result.getContact(i).pos);
      std::cout << "----- CONTACT " << i << " --------" << std::endl;
      std::cout << "contact_points: " << result.getContact(i).pos << std::endl;
      std::cout << "contact_points(w): " << posWorld << std::endl;
      std::cout << "norm: "
                << dart::collision::fcl::length(result.getContact(i).pos)
                << std::endl;
      std::cout << "penetration_depth: "
                << result.getContact(i).penetration_depth << std::endl;
      std::cout << "normal: " << result.getContact(i).normal << std::endl;
    }

    std::cout << std::endl;
  }
}

void Collision::printResult(
    const dart::collision::fcl::CollisionResult& _result)
{
  std::cout << "====== [ RESULT ] ======" << std::endl;
  std::cout << "The number of contacts: " << _result.numContacts() << std::endl;

  for (std::size_t i = 0; i < _result.numContacts(); ++i) {
    std::cout << "----- CONTACT " << i << " --------" << std::endl;
    std::cout << "contact_points: " << _result.getContact(i).pos << std::endl;
    std::cout << "penetration_depth: "
              << _result.getContact(i).penetration_depth << std::endl;
    std::cout << "normal: " << _result.getContact(i).normal << std::endl;
    // std::cout << std::endl;
  }
  std::cout << std::endl;
}

TEST_F(Collision, DROP)
{
  dtdbg << "Unrotated box\n";
  dart::collision::fcl::Box box1(0.5, 0.5, 0.5);
  dropWithRotation(&box1, 0, 0, 0);

  dtdbg << "Rotated box\n";
  dart::collision::fcl::Box box2(0.5, 0.5, 0.5);
  dropWithRotation(
      &box2,
      dart::math::Random::uniform(-3.14, 3.14),
      dart::math::Random::uniform(-3.14, 3.14),
      dart::math::Random::uniform(-3.14, 3.14));

  dropWithRotation(&box2, 0.0, 0.1, 0.0);
}

TEST_F(Collision, FCL_BOX_BOX)
{
  double EulerZ = 1;
  double EulerY = 2;
  double EulerX = 3;

  // Collision test setting
  dart::collision::fcl::CollisionResult result;
  dart::collision::fcl::CollisionRequest request;
  request.enable_contact = true;
  request.num_max_contacts = 100;

  // Ground like box setting
  dart::collision::fcl::Box groundObject(100, 100, 0.1);
  dart::collision::fcl::Transform3 groundTransf;
  groundTransf.setIdentity();
  dart::collision::fcl::Vector3 ground_position(0.0, 0.0, -0.05);
  dart::collision::fcl::setTranslation(groundTransf, ground_position);

  // Dropping box object setting
  dart::collision::fcl::Box box(0.5, 0.5, 0.5);
  dart::collision::fcl::Transform3 objectTransf;
  dart::collision::fcl::Matrix3 rot;
  dart::collision::fcl::setEulerZYX(rot, EulerZ, EulerY, EulerX);
  dart::collision::fcl::setRotation(objectTransf, rot);
  dart::collision::fcl::Vector3 dropping_position(0.0, 0.0, 5.0);
  dart::collision::fcl::setTranslation(objectTransf, dropping_position);

  // Let's drop the object until it collide with ground
  do {
    dart::collision::fcl::setTranslation(objectTransf, dropping_position);

    ::fcl::collide(
        &box, objectTransf, &groundObject, groundTransf, request, result);

    dropping_position[2] -= 0.00001;
  } while (result.numContacts() == 0);

  std::cout << "Current position of the object: "
            << dart::collision::fcl::getTranslation(objectTransf) << std::endl
            << "Number of contacts: " << result.numContacts() << std::endl;

  for (std::size_t i = 0; i < result.numContacts(); ++i) {
    std::cout << "----- CONTACT " << i << " --------" << std::endl;
    std::cout << "contact_points: " << result.getContact(i).pos << std::endl;
    std::cout << "penetration_depth: " << result.getContact(i).penetration_depth
              << std::endl;
    std::cout << "normal: " << result.getContact(i).normal << std::endl;
  }
}

//==============================================================================
void testSimpleFrames(const std::shared_ptr<CollisionDetector>& cd)
{
  auto simpleFrame1 = SimpleFrame::createShared(Frame::World());
  auto simpleFrame2 = SimpleFrame::createShared(Frame::World());
  auto simpleFrame3 = SimpleFrame::createShared(Frame::World());

  ShapePtr shape1(new BoxShape(Eigen::Vector3d(1.0, 1.0, 1.0)));
  ShapePtr shape2(new BoxShape(Eigen::Vector3d(1.0, 1.0, 1.0)));
  ShapePtr shape3(new BoxShape(Eigen::Vector3d(1.0, 1.0, 1.0)));

  simpleFrame1->setShape(shape1);
  simpleFrame2->setShape(shape2);
  simpleFrame3->setShape(shape3);

  auto group1 = cd->createCollisionGroup(simpleFrame1.get());
  auto group2 = cd->createCollisionGroup(simpleFrame2.get());
  auto group3 = cd->createCollisionGroup(simpleFrame3.get());

  auto groupAll
      = cd->createCollisionGroup(group1.get(), group2.get(), group3.get());

  EXPECT_EQ(group1->getNumShapeFrames(), 1u);
  EXPECT_EQ(group2->getNumShapeFrames(), 1u);
  EXPECT_EQ(group3->getNumShapeFrames(), 1u);
  EXPECT_EQ(
      groupAll->getNumShapeFrames(),
      group1->getNumShapeFrames() + group2->getNumShapeFrames()
          + group3->getNumShapeFrames());

  for (std::size_t i = 0; i < group1->getNumShapeFrames(); ++i)
    EXPECT_EQ(groupAll->getShapeFrame(i), group1->getShapeFrame(i));

  std::size_t start = group1->getNumShapeFrames();
  std::size_t end = start + group2->getNumShapeFrames();
  for (std::size_t i = start; i < end; ++i)
    EXPECT_EQ(groupAll->getShapeFrame(i), group2->getShapeFrame(i - start));

  start = start + group2->getNumShapeFrames();
  end = start + group3->getNumShapeFrames();
  for (std::size_t i = start; i < end; ++i)
    EXPECT_EQ(groupAll->getShapeFrame(i), group3->getShapeFrame(i - start));

  collision::CollisionOption option;
  collision::CollisionResult result;

  simpleFrame1->setTranslation(Eigen::Vector3d::Zero());
  simpleFrame2->setTranslation(Eigen::Vector3d(1.1, 0.0, 0.0));
  simpleFrame3->setTranslation(Eigen::Vector3d(2.2, 0.0, 0.0));
  EXPECT_FALSE(group1->collide(option, &result));
  EXPECT_FALSE(group2->collide(option, &result));
  EXPECT_FALSE(group3->collide(option, &result));
  EXPECT_FALSE(groupAll->collide(option, &result));

  simpleFrame1->setTranslation(Eigen::Vector3d::Zero());
  simpleFrame2->setTranslation(Eigen::Vector3d(0.5, 0.0, 0.0));
  simpleFrame3->setTranslation(Eigen::Vector3d(1.0, 0.0, 0.0));
  EXPECT_TRUE(group1->collide(group2.get(), option, &result));
  EXPECT_TRUE(group1->collide(group2.get(), option, &result));
  EXPECT_TRUE(group2->collide(group3.get(), option, &result));
  EXPECT_TRUE(groupAll->collide(option, &result));

  auto group23
      = cd->createCollisionGroup(simpleFrame2.get(), simpleFrame3.get());
  simpleFrame1->setTranslation(Eigen::Vector3d::Zero());
  simpleFrame2->setTranslation(Eigen::Vector3d(1.1, 0.0, 0.0));
  simpleFrame3->setTranslation(Eigen::Vector3d(1.6, 0.0, 0.0));
  EXPECT_FALSE(group1->collide(group2.get()));
  EXPECT_FALSE(group1->collide(group3.get()));
  EXPECT_TRUE(group2->collide(group3.get()));
  EXPECT_TRUE(group23->collide());
  EXPECT_FALSE(group1->collide(group23.get()));
}

//==============================================================================
TEST_F(Collision, SimpleFrames)
{
  auto fcl_mesh_dart = FCLCollisionDetector::create();
  fcl_mesh_dart->setPrimitiveShapeType(FCLCollisionDetector::MESH);
  fcl_mesh_dart->setContactPointComputationMethod(FCLCollisionDetector::DART);
  testSimpleFrames(fcl_mesh_dart);

  // auto fcl_prim_fcl = FCLCollisionDetector::create();
  // fcl_prim_fcl->setPrimitiveShapeType(FCLCollisionDetector::MESH);
  // fcl_prim_fcl->setContactPointComputationMethod(FCLCollisionDetector::FCL);
  // testSimpleFrames(fcl_prim_fcl);

  // auto fcl_mesh_fcl = FCLCollisionDetector::create();
  // fcl_mesh_fcl->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  // fcl_mesh_fcl->setContactPointComputationMethod(FCLCollisionDetector::DART);
  // testSimpleFrames(fcl_mesh_fcl);

  // auto fcl_mesh_fcl = FCLCollisionDetector::create();
  // fcl_mesh_fcl->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  // fcl_mesh_fcl->setContactPointComputationMethod(FCLCollisionDetector::FCL);
  // testSimpleFrames(fcl_mesh_fcl);

#if HAVE_BULLET
  auto bullet = BulletCollisionDetector::create();
  testSimpleFrames(bullet);
#endif

  auto dart = DARTCollisionDetector::create();
  testSimpleFrames(dart);
}

//==============================================================================
TEST_F(Collision, DARTCollisionDetectorRefreshesChangedShapeGeometry)
{
  auto cd = DARTCollisionDetector::create();
  auto simpleFrame1 = SimpleFrame::createShared(Frame::World());
  auto simpleFrame2 = SimpleFrame::createShared(Frame::World());

  auto shape1 = std::make_shared<BoxShape>(Eigen::Vector3d::Ones());
  auto shape2 = std::make_shared<BoxShape>(Eigen::Vector3d::Ones());
  simpleFrame1->setShape(shape1);
  simpleFrame2->setShape(shape2);

  simpleFrame1->setTranslation(Eigen::Vector3d::Zero());
  simpleFrame2->setTranslation(Eigen::Vector3d(1.2, 0.0, 0.0));

  auto group = cd->createCollisionGroup(simpleFrame1.get(), simpleFrame2.get());

  collision::CollisionOption option;
  option.enableContact = true;

  collision::CollisionResult result;
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_EQ(0u, result.getNumContacts());

  shape2->setSize(Eigen::Vector3d(1.6, 1.0, 1.0));

  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_GT(result.getNumContacts(), 0u);
}

//==============================================================================
TEST_F(Collision, DARTCollisionDetectorRefreshesShapeNodeRelativeTransform)
{
  auto cd = DARTCollisionDetector::create();
  auto planeFrame = SimpleFrame::createShared(Frame::World());
  planeFrame->setShape(
      std::make_shared<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0));

  auto sphereSkeleton = Skeleton::create("shape_node_relative_transform");
  auto pair = sphereSkeleton->createJointAndBodyNodePair<FreeJoint>();
  auto* sphereJoint = pair.first;
  auto* sphereBody = pair.second;
  auto* sphereNode = sphereBody->createShapeNodeWith<CollisionAspect>(
      std::make_shared<SphereShape>(0.5));

  Eigen::Isometry3d bodyPose = Eigen::Isometry3d::Identity();
  bodyPose.translation() = Eigen::Vector3d(0.0, 0.0, 0.45);
  sphereJoint->setRelativeTransform(bodyPose);

  auto planeGroup = cd->createCollisionGroup(planeFrame.get());
  auto sphereGroup = cd->createCollisionGroup(sphereNode);

  CollisionOption option;
  option.enableContact = true;

  CollisionResult result;
  EXPECT_TRUE(planeGroup->collide(sphereGroup.get(), option, &result));
  EXPECT_GT(result.getNumContacts(), 0u);

  Eigen::Isometry3d shapeOffset = Eigen::Isometry3d::Identity();
  shapeOffset.translation() = Eigen::Vector3d(0.0, 0.0, 2.0);
  sphereNode->setRelativeTransform(shapeOffset);

  result.clear();
  EXPECT_FALSE(planeGroup->collide(sphereGroup.get(), option, &result));
  EXPECT_EQ(0u, result.getNumContacts());
}

//==============================================================================
void testSphereSphere(
    const std::shared_ptr<CollisionDetector>& cd, double tol = 1e-12)
{
  auto simpleFrame1 = SimpleFrame::createShared(Frame::World());
  auto simpleFrame2 = SimpleFrame::createShared(Frame::World());

  ShapePtr shape1(new SphereShape(1.0));
  ShapePtr shape2(new SphereShape(0.5));
  simpleFrame1->setShape(shape1);
  simpleFrame2->setShape(shape2);

  auto group = cd->createCollisionGroup(simpleFrame1.get(), simpleFrame2.get());

  EXPECT_EQ(group->getNumShapeFrames(), 2u);

  collision::CollisionOption option;
  option.enableContact = true;

  collision::CollisionResult result;

  //----------------------------------------------------------------------------
  // Test 1: No contact
  //----------------------------------------------------------------------------

  simpleFrame1->setTranslation(Eigen::Vector3d::Zero());
  simpleFrame2->setTranslation(Eigen::Vector3d(2.0, 0.0, 0.0));
  result.clear();
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_TRUE(result.getNumContacts() == 0u);

  //----------------------------------------------------------------------------
  // Test 2: Point contact
  //----------------------------------------------------------------------------

  simpleFrame1->setTranslation(Eigen::Vector3d::Zero());
  simpleFrame2->setTranslation(Eigen::Vector3d(1.5 - tol, 0.0, 0.0));
  result.clear();
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_TRUE(result.getNumContacts() == 1u);

  const auto& contact = result.getContact(0);

  // Test contact location
  EXPECT_TRUE(contact.point.isApprox(Eigen::Vector3d::UnitX(), 2.0 * tol));

  // Test normal
  Eigen::Vector3d expectedNormal;
  if (result.getContact(0).collisionObject1->getShapeFrame()
      == simpleFrame1.get())
    expectedNormal << -1, 0, 0;
  else
    expectedNormal << 1, 0, 0;
  double tol2 = tol;
  if (cd->getType() == FCLCollisionDetector::getStaticType()
      && static_cast<FCLCollisionDetector*>(cd.get())->getPrimitiveShapeType()
             == FCLCollisionDetector::MESH) {
    tol2 *= 1e+12;
    // FCL returns less accurate contact normals for sphere-sphere since we're
    // using sphere-like rough meshes instead of analytical sphere shapes.
  }
  EXPECT_TRUE(contact.normal.isApprox(expectedNormal, tol2));

  //----------------------------------------------------------------------------
  // Test 3: Corner case of that the bigger sphere completely encloses the
  // smaller sphere
  //----------------------------------------------------------------------------

  simpleFrame1->setTranslation(Eigen::Vector3d::Zero());
  simpleFrame2->setTranslation(Eigen::Vector3d::Zero());
  result.clear();
  if (cd->getType() == FCLCollisionDetector::getStaticType()) {
    EXPECT_FALSE(group->collide(option, &result));
    // FCL is not able to detect collisions when an object completely (strictly)
    // contains the other object (no collisions between the hulls)
  } else {
    EXPECT_TRUE(group->collide(option, &result));
    // TODO(JS): BulletCollisionDetector includes a bug related to this.
    // (see #876)

    constexpr auto hasBullet = (HAVE_BULLET == 1);
    if constexpr (!hasBullet) {
      EXPECT_EQ(result.getNumContacts(), 1u);
    }
    for (auto i = 0u; i < result.getNumContacts(); ++i) {
      std::cout << "point: " << result.getContact(i).point.transpose()
                << std::endl;
    }
  }
  // The positions of contact point are different depending on the collision
  // detector. More integration tests need to be added.
}

//==============================================================================
TEST_F(Collision, SphereSphere)
{
  {
    SCOPED_TRACE("FCLCollisionDetector");
    auto fcl_mesh_dart = FCLCollisionDetector::create();
    fcl_mesh_dart->setPrimitiveShapeType(FCLCollisionDetector::MESH);
    fcl_mesh_dart->setContactPointComputationMethod(FCLCollisionDetector::DART);
    testSphereSphere(fcl_mesh_dart);
  }

  // auto fcl_prim_fcl = FCLCollisionDetector::create();
  // fcl_prim_fcl->setPrimitiveShapeType(FCLCollisionDetector::MESH);
  // fcl_prim_fcl->setContactPointComputationMethod(FCLCollisionDetector::FCL);
  // testSphereSphere(fcl_prim_fcl);

  // auto fcl_mesh_fcl = FCLCollisionDetector::create();
  // fcl_mesh_fcl->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  // fcl_mesh_fcl->setContactPointComputationMethod(FCLCollisionDetector::DART);
  // testSphereSphere(fcl_mesh_fcl);

  // auto fcl_mesh_fcl = FCLCollisionDetector::create();
  // fcl_mesh_fcl->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  // fcl_mesh_fcl->setContactPointComputationMethod(FCLCollisionDetector::FCL);
  // testSphereSphere(fcl_mesh_fcl);

#if HAVE_ODE
  {
    SCOPED_TRACE("OdeCollisionDetector");
    auto ode = OdeCollisionDetector::create();
    testSphereSphere(ode);
  }
#endif

#if HAVE_BULLET
  {
    SCOPED_TRACE("BulletCollisionDetector");
    auto bullet = BulletCollisionDetector::create();
    testSphereSphere(bullet);
  }
#endif

  {
    SCOPED_TRACE("BulletCollisionDetector");
    auto dart = DARTCollisionDetector::create();
    testSphereSphere(dart);
  }
}

//==============================================================================
bool checkBoundingBox(
    const Eigen::Vector3d& min,
    const Eigen::Vector3d& max,
    const Eigen::Vector3d& point,
    double tol = 1e-12)
{
  for (auto i = 0u; i < 3u; ++i) {
    if (min[i] - tol > point[i] || point[i] > max[i] + tol)
      return false;
  }

  return true;
}

//==============================================================================
void testBoxBox(
    const std::shared_ptr<CollisionDetector>& cd, double tol = 1e-12)
{
  auto simpleFrame1 = SimpleFrame::createShared(Frame::World());
  auto simpleFrame2 = SimpleFrame::createShared(Frame::World());

  ShapePtr shape1(new BoxShape(Eigen::Vector3d(1.0, 1.0, 1.0)));
  ShapePtr shape2(new BoxShape(Eigen::Vector3d(0.5, 0.5, 0.5)));
  simpleFrame1->setShape(shape1);
  simpleFrame2->setShape(shape2);

  Eigen::Vector3d pos1 = Eigen::Vector3d(0.0, 0.0, -0.5);
  Eigen::Vector3d pos2 = Eigen::Vector3d(0.0, 0.5, 0.25);
  simpleFrame1->setTranslation(pos1);
  simpleFrame2->setTranslation(pos2);

  auto group1 = cd->createCollisionGroup(simpleFrame1.get());
  auto group2 = cd->createCollisionGroup(simpleFrame2.get());
  auto groupAll = cd->createCollisionGroup(group1.get(), group2.get());

  EXPECT_EQ(group1->getNumShapeFrames(), 1u);
  EXPECT_EQ(group2->getNumShapeFrames(), 1u);
  EXPECT_EQ(
      groupAll->getNumShapeFrames(),
      group1->getNumShapeFrames() + group2->getNumShapeFrames());

  collision::CollisionOption option;
  collision::CollisionResult result;

  result.clear();
  EXPECT_TRUE(group1->collide(group2.get(), option, &result));

  result.clear();
  EXPECT_TRUE(groupAll->collide(option, &result));

  Eigen::Vector3d min = Eigen::Vector3d(-0.25, 0.25, 0.0);
  Eigen::Vector3d max = Eigen::Vector3d(0.25, 0.5, 0.0);

  const auto numContacts = result.getNumContacts();

  const auto checkNumContacts = (numContacts <= 4u);
  EXPECT_TRUE(checkNumContacts);
  if (!checkNumContacts)
    std::cout << "# of contants: " << numContacts << "\n";

  for (const auto& contact : result.getContacts()) {
    const auto& point = contact.point;

    const auto result = checkBoundingBox(min, max, point, tol);
    EXPECT_TRUE(result);

    if (!result)
      std::cout << "point: " << point.transpose() << "\n";
  }
}

//==============================================================================
TEST_F(Collision, BoxBox)
{
  auto fcl_mesh_dart = FCLCollisionDetector::create();
  fcl_mesh_dart->setPrimitiveShapeType(FCLCollisionDetector::MESH);
  fcl_mesh_dart->setContactPointComputationMethod(FCLCollisionDetector::DART);
  testBoxBox(fcl_mesh_dart);

  // auto fcl_prim_fcl = FCLCollisionDetector::create();
  // fcl_prim_fcl->setPrimitiveShapeType(FCLCollisionDetector::MESH);
  // fcl_prim_fcl->setContactPointComputationMethod(FCLCollisionDetector::FCL);
  // testBoxBox(fcl_prim_fcl);

  // auto fcl_mesh_fcl = FCLCollisionDetector::create();
  // fcl_mesh_fcl->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  // fcl_mesh_fcl->setContactPointComputationMethod(FCLCollisionDetector::DART);
  // testBoxBox(fcl_mesh_fcl);

  // auto fcl_mesh_fcl = FCLCollisionDetector::create();
  // fcl_mesh_fcl->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  // fcl_mesh_fcl->setContactPointComputationMethod(FCLCollisionDetector::FCL);
  // testBoxBox(fcl_mesh_fcl);

#if HAVE_ODE
  auto ode = OdeCollisionDetector::create();
  testBoxBox(ode);
#endif

#if HAVE_BULLET
  auto bullet = BulletCollisionDetector::create();
  testBoxBox(bullet);
#endif

  auto dart = DARTCollisionDetector::create();
  testBoxBox(dart);
}

//==============================================================================
TEST_F(Collision, DartPlanePrimitiveContacts)
{
  auto cd = DARTCollisionDetector::create();

  auto planeFrame = SimpleFrame::createShared(Frame::World());
  auto sphereFrame = SimpleFrame::createShared(Frame::World());
  auto boxFrame = SimpleFrame::createShared(Frame::World());
  auto cylinderFrame = SimpleFrame::createShared(Frame::World());
  auto farSphereFrame = SimpleFrame::createShared(Frame::World());

  planeFrame->setShape(
      std::make_shared<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0));
  sphereFrame->setShape(std::make_shared<SphereShape>(0.5));
  boxFrame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  cylinderFrame->setShape(std::make_shared<CylinderShape>(0.5, 1.0));
  farSphereFrame->setShape(std::make_shared<SphereShape>(0.5));

  sphereFrame->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.45));
  boxFrame->setTranslation(Eigen::Vector3d(2.0, 0.0, 0.49));
  cylinderFrame->setTranslation(Eigen::Vector3d(4.0, 0.0, 0.49));
  farSphereFrame->setTranslation(Eigen::Vector3d(6.0, 0.0, 0.6));

  auto group = cd->createCollisionGroup(
      planeFrame.get(),
      sphereFrame.get(),
      boxFrame.get(),
      cylinderFrame.get(),
      farSphereFrame.get());

  CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 16u;

  CollisionResult result;
  ASSERT_TRUE(group->collide(option, &result));
  ASSERT_EQ(result.getNumContacts(), 5u);

  bool sawSphere = false;
  auto boxContacts = 0u;
  bool sawCylinder = false;
  for (const auto& contact : result.getContacts()) {
    EXPECT_EQ(contact.collisionObject1->getShapeFrame(), planeFrame.get());
    EXPECT_TRUE(contact.normal.isApprox(-Eigen::Vector3d::UnitZ(), 1e-12));

    const auto* shapeFrame = contact.collisionObject2->getShapeFrame();
    if (shapeFrame == sphereFrame.get()) {
      sawSphere = true;
      EXPECT_NEAR(contact.penetrationDepth, 0.05, 1e-12);
    } else if (shapeFrame == boxFrame.get()) {
      ++boxContacts;
      EXPECT_NEAR(contact.penetrationDepth, 0.01, 1e-12);
    } else if (shapeFrame == cylinderFrame.get()) {
      sawCylinder = true;
      EXPECT_NEAR(contact.penetrationDepth, 0.01, 1e-12);
    } else {
      FAIL() << "Unexpected contact shape";
    }
  }
  EXPECT_TRUE(sawSphere);
  EXPECT_EQ(boxContacts, 3u);
  EXPECT_TRUE(sawCylinder);

  auto sphereGroup = cd->createCollisionGroup(sphereFrame.get());
  auto planeGroup = cd->createCollisionGroup(planeFrame.get());
  result.clear();
  ASSERT_TRUE(sphereGroup->collide(planeGroup.get(), option, &result));
  ASSERT_EQ(result.getNumContacts(), 1u);
  EXPECT_EQ(
      result.getContact(0).collisionObject1->getShapeFrame(),
      sphereFrame.get());
  EXPECT_EQ(
      result.getContact(0).collisionObject2->getShapeFrame(), planeFrame.get());
  EXPECT_TRUE(
      result.getContact(0).normal.isApprox(Eigen::Vector3d::UnitZ(), 1e-12));

  auto horizontalCylinderFrame = SimpleFrame::createShared(Frame::World());
  horizontalCylinderFrame->setShape(std::make_shared<CylinderShape>(0.5, 1.0));

  Eigen::Isometry3d horizontalCylinderTf = Eigen::Isometry3d::Identity();
  horizontalCylinderTf.linear()
      = Eigen::AngleAxisd(0.5 * constantsd::pi(), Eigen::Vector3d::UnitY())
            .toRotationMatrix();
  horizontalCylinderTf.translation() = Eigen::Vector3d(8.0, 0.0, 0.49);
  horizontalCylinderFrame->setTransform(horizontalCylinderTf);

  auto horizontalCylinderGroup
      = cd->createCollisionGroup(horizontalCylinderFrame.get());
  result.clear();
  ASSERT_TRUE(
      planeGroup->collide(horizontalCylinderGroup.get(), option, &result));
  ASSERT_EQ(result.getNumContacts(), 1u);
  const auto& planeFirstCylinderContact = result.getContact(0);
  EXPECT_TRUE(planeFirstCylinderContact.normal.isApprox(
      -Eigen::Vector3d::UnitZ(), 1e-12));
  EXPECT_NEAR(planeFirstCylinderContact.penetrationDepth, 0.01, 1e-12);
  EXPECT_NEAR(planeFirstCylinderContact.point.x(), 8.0, 1e-12);
  EXPECT_NEAR(planeFirstCylinderContact.point.y(), 0.0, 1e-12);

  result.clear();
  ASSERT_TRUE(
      horizontalCylinderGroup->collide(planeGroup.get(), option, &result));
  ASSERT_EQ(result.getNumContacts(), 1u);
  const auto& cylinderFirstPlaneContact = result.getContact(0);
  const auto* cylinderFirstShapeFrame
      = cylinderFirstPlaneContact.collisionObject1->getShapeFrame();
  if (cylinderFirstShapeFrame == horizontalCylinderFrame.get()) {
    EXPECT_TRUE(cylinderFirstPlaneContact.normal.isApprox(
        Eigen::Vector3d::UnitZ(), 1e-12));
  } else {
    EXPECT_EQ(cylinderFirstShapeFrame, planeFrame.get());
    EXPECT_TRUE(cylinderFirstPlaneContact.normal.isApprox(
        -Eigen::Vector3d::UnitZ(), 1e-12));
  }
  EXPECT_TRUE(cylinderFirstPlaneContact.point.isApprox(
      planeFirstCylinderContact.point, 1e-12));
  EXPECT_NEAR(
      cylinderFirstPlaneContact.penetrationDepth,
      planeFirstCylinderContact.penetrationDepth,
      1e-12);
}

//==============================================================================
TEST_F(Collision, DartCollideAcceptsGenericCollisionObjects)
{
  auto cd = DARTCollisionDetector::create();

  auto frameA = SimpleFrame::createShared(Frame::World());
  auto frameB = SimpleFrame::createShared(Frame::World());

  frameA->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  frameB->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));

  frameA->setTranslation(Eigen::Vector3d::Zero());
  frameB->setTranslation(Eigen::Vector3d(0.5, 0.0, 0.0));

  TestCollisionObject objectA(cd.get(), frameA.get());
  TestCollisionObject objectB(cd.get(), frameB.get());

  CollisionResult result;
  EXPECT_GT(::dart::collision::collide(&objectA, &objectB, result), 0);
  ASSERT_GE(result.getNumContacts(), 1u);
  EXPECT_EQ(result.getContact(0).collisionObject1, &objectA);
  EXPECT_EQ(result.getContact(0).collisionObject2, &objectB);
}

//==============================================================================
TEST_F(Collision, DartParallelFinitePlaneContactsMatchSerial)
{
  constexpr std::size_t kNumBoxes = 140u;

  auto cd = DARTCollisionDetector::create();

  auto planeFrame = SimpleFrame::createShared(Frame::World());
  planeFrame->setShape(
      std::make_shared<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0));
  const Eigen::Vector3d planeTranslation(0.0, 0.0, 0.01);
  planeFrame->setTranslation(planeTranslation);

  std::vector<std::shared_ptr<SimpleFrame>> boxFrames;
  boxFrames.reserve(kNumBoxes);
  for (std::size_t i = 0u; i < kNumBoxes; ++i) {
    auto boxFrame = SimpleFrame::createShared(Frame::World());
    boxFrame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
    boxFrame->setTranslation(Eigen::Vector3d(1.25 * i, 0.0, 0.49));
    boxFrames.push_back(boxFrame);
  }

  auto group = cd->createCollisionGroup();
  group->addShapeFrame(planeFrame.get());
  for (const auto& boxFrame : boxFrames)
    group->addShapeFrame(boxFrame.get());

  CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 1024u;
  option.collisionFilter = std::make_shared<BodyNodeCollisionFilter>();

  cd->setNumCollisionThreads(1u);
  CollisionResult serialResult;
  ASSERT_TRUE(group->collide(option, &serialResult));
  ASSERT_GT(serialResult.getNumContacts(), kNumBoxes);

  // Leave the plane frame dirty before the threaded query. The detector should
  // copy transforms on the main thread instead of warming them in workers.
  planeFrame->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.02));
  planeFrame->setTranslation(planeTranslation);

  cd->setNumCollisionThreads(4u);
  EXPECT_EQ(cd->getNumCollisionThreads(), 4u);
  CollisionResult parallelResult;
  ASSERT_TRUE(group->collide(option, &parallelResult));

  ASSERT_EQ(parallelResult.getNumContacts(), serialResult.getNumContacts());
  for (std::size_t i = 0u; i < serialResult.getNumContacts(); ++i) {
    SCOPED_TRACE(i);
    const auto& serialContact = serialResult.getContact(i);
    const auto& parallelContact = parallelResult.getContact(i);
    EXPECT_EQ(
        parallelContact.collisionObject1->getShapeFrame(),
        serialContact.collisionObject1->getShapeFrame());
    EXPECT_EQ(
        parallelContact.collisionObject2->getShapeFrame(),
        serialContact.collisionObject2->getShapeFrame());
    EXPECT_TRUE(parallelContact.point.isApprox(serialContact.point, 1e-12));
    EXPECT_TRUE(parallelContact.normal.isApprox(serialContact.normal, 1e-12));
    EXPECT_NEAR(
        parallelContact.penetrationDepth,
        serialContact.penetrationDepth,
        1e-12);
  }
}

//==============================================================================
TEST_F(Collision, DartParallelFinitePlaneUsesProjectedContactBounds)
{
  constexpr std::size_t kNumSpheres = 140u;
  constexpr double kRadius = 0.02;

  auto cd = DARTCollisionDetector::create();

  auto planeFrame = SimpleFrame::createShared(Frame::World());
  planeFrame->setShape(
      std::make_shared<PlaneShape>(Eigen::Vector3d::UnitX(), 0.0));

  std::vector<std::shared_ptr<SimpleFrame>> sphereFrames;
  sphereFrames.reserve(kNumSpheres);
  for (std::size_t i = 0u; i < kNumSpheres; ++i) {
    auto sphereFrame = SimpleFrame::createShared(Frame::World());
    sphereFrame->setShape(std::make_shared<SphereShape>(kRadius));
    sphereFrame->setTranslation(Eigen::Vector3d(-0.1 - 0.1 * i, 0.0, 0.0));
    sphereFrames.push_back(sphereFrame);
  }

  auto group = cd->createCollisionGroup();
  group->addShapeFrame(planeFrame.get());
  for (const auto& sphereFrame : sphereFrames)
    group->addShapeFrame(sphereFrame.get());

  CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = kNumSpheres * 2u;

  cd->setNumCollisionThreads(1u);
  CollisionResult serialResult;
  ASSERT_TRUE(group->collide(option, &serialResult));
  ASSERT_EQ(serialResult.getNumContacts(), 1u);

  cd->setNumCollisionThreads(4u);
  CollisionResult parallelResult;
  ASSERT_TRUE(group->collide(option, &parallelResult));
  ASSERT_EQ(parallelResult.getNumContacts(), serialResult.getNumContacts());
  EXPECT_TRUE(parallelResult.getContact(0).point.isApprox(
      Eigen::Vector3d::Zero(), 1e-12));
}

//==============================================================================
TEST_F(Collision, DartParallelFinitePlaneTwoGroupPhasesProbeGlobalIndex)
{
  constexpr std::size_t kNumSpheres = 140u;
  constexpr double kRadius = 0.02;
  constexpr double kProjectedSpacing = 0.25;

  auto cd = DARTCollisionDetector::create();

  auto planeFrame1 = SimpleFrame::createShared(Frame::World());
  auto planeFrame2 = SimpleFrame::createShared(Frame::World());
  planeFrame1->setShape(
      std::make_shared<PlaneShape>(Eigen::Vector3d::UnitX(), 0.0));
  planeFrame2->setShape(
      std::make_shared<PlaneShape>(Eigen::Vector3d::UnitX(), 0.0));

  std::vector<std::shared_ptr<SimpleFrame>> group1Spheres;
  std::vector<std::shared_ptr<SimpleFrame>> group2Spheres;
  group1Spheres.reserve(kNumSpheres);
  group2Spheres.reserve(kNumSpheres);
  for (std::size_t i = 0u; i < kNumSpheres; ++i) {
    const auto projectedY = kProjectedSpacing * i;

    auto sphereFrame1 = SimpleFrame::createShared(Frame::World());
    sphereFrame1->setShape(std::make_shared<SphereShape>(kRadius));
    sphereFrame1->setTranslation(Eigen::Vector3d(-0.1, projectedY, 0.0));
    group1Spheres.push_back(sphereFrame1);

    auto sphereFrame2 = SimpleFrame::createShared(Frame::World());
    sphereFrame2->setShape(std::make_shared<SphereShape>(kRadius));
    sphereFrame2->setTranslation(Eigen::Vector3d(-100.0, projectedY, 0.0));
    group2Spheres.push_back(sphereFrame2);
  }

  auto group1 = cd->createCollisionGroup();
  auto group2 = cd->createCollisionGroup();
  group1->addShapeFrame(planeFrame1.get());
  group2->addShapeFrame(planeFrame2.get());
  for (const auto& sphereFrame : group1Spheres)
    group1->addShapeFrame(sphereFrame.get());
  for (const auto& sphereFrame : group2Spheres)
    group2->addShapeFrame(sphereFrame.get());

  CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = kNumSpheres * 4u;

  cd->setNumCollisionThreads(1u);
  CollisionResult serialResult;
  ASSERT_TRUE(group1->collide(group2.get(), option, &serialResult));
  ASSERT_EQ(serialResult.getNumContacts(), kNumSpheres);

  cd->setNumCollisionThreads(4u);
  CollisionResult parallelResult;
  ASSERT_TRUE(group1->collide(group2.get(), option, &parallelResult));
  ASSERT_EQ(parallelResult.getNumContacts(), serialResult.getNumContacts());

  std::vector<double> contactYs;
  contactYs.reserve(parallelResult.getNumContacts());
  for (const auto& contact : parallelResult.getContacts()) {
    EXPECT_NEAR(contact.point.x(), 0.0, 1e-12);
    EXPECT_NEAR(contact.point.z(), 0.0, 1e-12);
    contactYs.push_back(contact.point.y());
  }

  std::sort(contactYs.begin(), contactYs.end());
  for (std::size_t i = 0u; i < contactYs.size(); ++i)
    EXPECT_NEAR(contactYs[i], kProjectedSpacing * i, 1e-12);
}

//==============================================================================
TEST_F(Collision, DartPerPairContactCapSelectsDeepSpreadContacts)
{
  auto cd = DARTCollisionDetector::create();

  auto planeFrame = SimpleFrame::createShared(Frame::World());
  planeFrame->setShape(
      std::make_shared<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0));

  auto boxFrame = SimpleFrame::createShared(Frame::World());
  boxFrame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));

  Eigen::Isometry3d boxTf = Eigen::Isometry3d::Identity();
  boxTf.linear() = (Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()))
                       .toRotationMatrix();
  boxTf.translation() = Eigen::Vector3d(0.0, 0.0, 0.45);
  boxFrame->setTransform(boxTf);

  auto planeGroup = cd->createCollisionGroup(planeFrame.get());
  auto boxGroup = cd->createCollisionGroup(boxFrame.get());

  CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 10u;

  CollisionResult uncappedResult;
  ASSERT_TRUE(planeGroup->collide(boxGroup.get(), option, &uncappedResult));
  ASSERT_EQ(uncappedResult.getNumContacts(), 3u);

  option.maxNumContactsPerPair = 2u;
  CollisionResult cappedResult;
  ASSERT_TRUE(planeGroup->collide(boxGroup.get(), option, &cappedResult));
  ASSERT_EQ(cappedResult.getNumContacts(), 2u);

  auto containsPoint
      = [](const CollisionResult& result, const Eigen::Vector3d& point) {
          for (const auto& contact : result.getContacts()) {
            if (contact.point.isApprox(point, 1e-12))
              return true;
          }
          return false;
        };
  auto containsPointForFrame = [](const CollisionResult& result,
                                  const ShapeFrame* frame,
                                  const Eigen::Vector3d& point) {
    for (const auto& contact : result.getContacts()) {
      if (contact.collisionObject1->getShapeFrame() != frame
          && contact.collisionObject2->getShapeFrame() != frame) {
        continue;
      }

      if (contact.point.isApprox(point, 1e-12))
        return true;
    }
    return false;
  };

  std::size_t deepestIndex = 0u;
  for (std::size_t i = 1u; i < uncappedResult.getNumContacts(); ++i) {
    if (uncappedResult.getContact(i).penetrationDepth
        > uncappedResult.getContact(deepestIndex).penetrationDepth) {
      deepestIndex = i;
    }
  }

  ASSERT_NE(deepestIndex, 0u);

  std::size_t spreadIndex = deepestIndex == 0u ? 1u : 0u;
  double spreadDistance = -1.0;
  double spreadDepth = -std::numeric_limits<double>::infinity();
  const auto& deepestContact = uncappedResult.getContact(deepestIndex);
  for (std::size_t i = 0u; i < uncappedResult.getNumContacts(); ++i) {
    if (i == deepestIndex)
      continue;

    const auto& candidate = uncappedResult.getContact(i);
    const auto distance
        = (candidate.point - deepestContact.point).squaredNorm();
    if (distance > spreadDistance
        || (distance == spreadDistance
            && candidate.penetrationDepth > spreadDepth)) {
      spreadDistance = distance;
      spreadDepth = candidate.penetrationDepth;
      spreadIndex = i;
    }
  }

  ASSERT_NE(spreadIndex, 0u);

  EXPECT_TRUE(containsPoint(cappedResult, deepestContact.point));
  EXPECT_TRUE(containsPoint(
      cappedResult, uncappedResult.getContact(spreadIndex).point));
  EXPECT_FALSE(containsPoint(cappedResult, uncappedResult.getContact(0).point));

  option.maxNumContacts = 1u;
  CollisionResult globallyCappedResult;
  ASSERT_TRUE(
      planeGroup->collide(boxGroup.get(), option, &globallyCappedResult));
  ASSERT_EQ(globallyCappedResult.getNumContacts(), 1u);
  EXPECT_TRUE(containsPoint(globallyCappedResult, deepestContact.point));

  option.maxNumContacts = 10u;
  auto sphereFrame = SimpleFrame::createShared(Frame::World());
  sphereFrame->setShape(std::make_shared<SphereShape>(0.5));
  sphereFrame->setTranslation(Eigen::Vector3d(
      deepestContact.point.x(), deepestContact.point.y(), 0.45));

  auto mobileGroup
      = cd->createCollisionGroup(sphereFrame.get(), boxFrame.get());

  option.maxNumContacts = 2u;
  CollisionResult priorityBackfilledResult;
  ASSERT_TRUE(planeGroup->collide(
      mobileGroup.get(), option, &priorityBackfilledResult));
  ASSERT_EQ(priorityBackfilledResult.getNumContacts(), 2u);

  std::size_t sphereContacts = 0u;
  std::size_t boxContacts = 0u;
  for (const auto& contact : priorityBackfilledResult.getContacts()) {
    if (contact.collisionObject1->getShapeFrame() == sphereFrame.get()
        || contact.collisionObject2->getShapeFrame() == sphereFrame.get()) {
      ++sphereContacts;
    } else if (
        contact.collisionObject1->getShapeFrame() == boxFrame.get()
        || contact.collisionObject2->getShapeFrame() == boxFrame.get()) {
      ++boxContacts;
    }
  }

  EXPECT_EQ(sphereContacts, 1u);
  EXPECT_EQ(boxContacts, 1u);
  EXPECT_TRUE(containsPoint(
      priorityBackfilledResult, uncappedResult.getContact(spreadIndex).point));
  EXPECT_FALSE(containsPoint(
      priorityBackfilledResult, uncappedResult.getContact(0).point));

  auto separateSphereFrame = SimpleFrame::createShared(Frame::World());
  separateSphereFrame->setShape(std::make_shared<SphereShape>(0.5));
  separateSphereFrame->setTranslation(Eigen::Vector3d(2.0, 0.0, 0.45));

  auto separateMobileGroup
      = cd->createCollisionGroup(separateSphereFrame.get(), boxFrame.get());

  option.maxNumContacts = 3u;
  option.maxNumContactsPerPair = 4u;
  CollisionResult globallyLimitedPairResult;
  ASSERT_TRUE(planeGroup->collide(
      separateMobileGroup.get(), option, &globallyLimitedPairResult));
  ASSERT_EQ(globallyLimitedPairResult.getNumContacts(), 3u);

  sphereContacts = 0u;
  boxContacts = 0u;
  for (const auto& contact : globallyLimitedPairResult.getContacts()) {
    if (contact.collisionObject1->getShapeFrame() == separateSphereFrame.get()
        || contact.collisionObject2->getShapeFrame()
               == separateSphereFrame.get()) {
      ++sphereContacts;
    } else if (
        contact.collisionObject1->getShapeFrame() == boxFrame.get()
        || contact.collisionObject2->getShapeFrame() == boxFrame.get()) {
      ++boxContacts;
    }
  }

  EXPECT_EQ(sphereContacts, 1u);
  EXPECT_EQ(boxContacts, 2u);
  EXPECT_TRUE(containsPointForFrame(
      globallyLimitedPairResult,
      boxFrame.get(),
      uncappedResult.getContact(spreadIndex).point));
  EXPECT_FALSE(containsPointForFrame(
      globallyLimitedPairResult,
      boxFrame.get(),
      uncappedResult.getContact(0).point));

  option.maxNumContacts = 10u;
  option.maxNumContactsPerPair = 2u;
  CollisionResult backfilledResult;
  ASSERT_TRUE(
      planeGroup->collide(mobileGroup.get(), option, &backfilledResult));
  ASSERT_EQ(backfilledResult.getNumContacts(), 3u);

  sphereContacts = 0u;
  boxContacts = 0u;
  for (const auto& contact : backfilledResult.getContacts()) {
    if (contact.collisionObject1->getShapeFrame() == sphereFrame.get()
        || contact.collisionObject2->getShapeFrame() == sphereFrame.get()) {
      ++sphereContacts;
    } else if (
        contact.collisionObject1->getShapeFrame() == boxFrame.get()
        || contact.collisionObject2->getShapeFrame() == boxFrame.get()) {
      ++boxContacts;
    }
  }

  EXPECT_EQ(sphereContacts, 1u);
  EXPECT_EQ(boxContacts, 2u);
  EXPECT_TRUE(containsPoint(backfilledResult, deepestContact.point));
  EXPECT_TRUE(containsPoint(
      backfilledResult, uncappedResult.getContact(spreadIndex).point));
}

//==============================================================================
TEST_F(Collision, DartPerPairContactCapCoalescesNearDuplicatePairContacts)
{
  auto cd = DARTCollisionDetector::create();

  auto planeFrame = SimpleFrame::createShared(Frame::World());
  planeFrame->setShape(
      std::make_shared<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0));

  // Keep the emitted box-plane support points within duplicate tolerance while
  // making a later emitted point the deepest duplicate.
  const auto tinySize = Eigen::Vector3d::Constant(1.0e-13);
  auto boxFrame = SimpleFrame::createShared(Frame::World());
  boxFrame->setShape(std::make_shared<BoxShape>(tinySize));

  Eigen::Isometry3d boxTf = Eigen::Isometry3d::Identity();
  boxTf.linear()
      = Eigen::AngleAxisd(-0.4, Eigen::Vector3d::UnitX()).toRotationMatrix();
  boxFrame->setTransform(boxTf);

  auto planeGroup = cd->createCollisionGroup(planeFrame.get());
  auto boxGroup = cd->createCollisionGroup(boxFrame.get());

  std::vector<double> rawDepths;
  std::vector<Eigen::Vector3d> rawPoints;
  const auto halfExtents = 0.5 * tinySize;
  for (int i = 0; i < 8 && rawDepths.size() < 3u; ++i) {
    const Eigen::Vector3d localCorner(
        (i & 1) ? halfExtents.x() : -halfExtents.x(),
        (i & 2) ? halfExtents.y() : -halfExtents.y(),
        (i & 4) ? halfExtents.z() : -halfExtents.z());

    const auto worldCorner = boxTf * localCorner;
    const auto signedDist = worldCorner.z();
    if (signedDist > 1e-9)
      continue;

    rawDepths.push_back(std::max(0.0, -signedDist));
    rawPoints.push_back(Eigen::Vector3d(worldCorner.x(), worldCorner.y(), 0.0));
  }

  ASSERT_EQ(rawDepths.size(), 3u);
  ASSERT_LT((rawPoints[0] - rawPoints[1]).norm(), 3.0e-12);
  ASSERT_LT((rawPoints[0] - rawPoints[2]).norm(), 3.0e-12);

  const auto deepestDepth
      = *std::max_element(rawDepths.begin(), rawDepths.end());
  ASSERT_GT(deepestDepth, rawDepths[0]);

  CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 10u;

  CollisionResult uncappedResult;
  ASSERT_TRUE(planeGroup->collide(boxGroup.get(), option, &uncappedResult));
  ASSERT_EQ(uncappedResult.getNumContacts(), 1u);
  EXPECT_NEAR(
      uncappedResult.getContact(0).penetrationDepth, rawDepths[0], 1e-18);

  option.maxNumContactsPerPair = 2u;
  CollisionResult cappedResult;
  ASSERT_TRUE(planeGroup->collide(boxGroup.get(), option, &cappedResult));
  ASSERT_EQ(cappedResult.getNumContacts(), 1u);
  EXPECT_NEAR(cappedResult.getContact(0).penetrationDepth, deepestDepth, 1e-18);
  EXPECT_GT(
      cappedResult.getContact(0).penetrationDepth,
      uncappedResult.getContact(0).penetrationDepth);
}

//==============================================================================
TEST_F(Collision, DartContactPointDeduplicationKeepsDistinctPoints)
{
  auto cd = DARTCollisionDetector::create();

  auto planeFrame = SimpleFrame::createShared(Frame::World());
  planeFrame->setShape(
      std::make_shared<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0));

  auto sphere1 = SimpleFrame::createShared(Frame::World());
  auto sphere2 = SimpleFrame::createShared(Frame::World());
  auto sphere3 = SimpleFrame::createShared(Frame::World());
  auto sphere4 = SimpleFrame::createShared(Frame::World());
  auto sphere5 = SimpleFrame::createShared(Frame::World());

  sphere1->setShape(std::make_shared<SphereShape>(0.5));
  sphere2->setShape(std::make_shared<SphereShape>(0.5));
  sphere3->setShape(std::make_shared<SphereShape>(0.5));
  sphere4->setShape(std::make_shared<SphereShape>(0.5));
  sphere5->setShape(std::make_shared<SphereShape>(0.5));

  sphere1->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.45));
  sphere2->setTranslation(Eigen::Vector3d(1.0e-12, 0.0, 0.45));
  sphere3->setTranslation(Eigen::Vector3d(1.0e-10, 0.0, 0.45));
  sphere4->setTranslation(Eigen::Vector3d(3.0e7, 0.0, 0.45));
  sphere5->setTranslation(Eigen::Vector3d(3.0e7, 0.0, 0.45));

  auto planeGroup = cd->createCollisionGroup(planeFrame.get());
  auto sphereGroup = cd->createCollisionGroup(
      sphere1.get(),
      sphere2.get(),
      sphere3.get(),
      sphere4.get(),
      sphere5.get());

  CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 8u;

  CollisionResult result;
  ASSERT_TRUE(planeGroup->collide(sphereGroup.get(), option, &result));
  ASSERT_EQ(result.getNumContacts(), 3u);

  std::vector<double> contactXs;
  for (const auto& contact : result.getContacts()) {
    EXPECT_EQ(contact.collisionObject1->getShapeFrame(), planeFrame.get());
    EXPECT_NEAR(contact.penetrationDepth, 0.05, 1e-12);
    EXPECT_TRUE(contact.normal.isApprox(-Eigen::Vector3d::UnitZ(), 1e-12));
    contactXs.push_back(contact.point.x());
  }

  std::sort(contactXs.begin(), contactXs.end());
  EXPECT_NEAR(contactXs[0], 0.0, 1e-12);
  EXPECT_NEAR(contactXs[1], 1.0e-10, 1e-12);
  EXPECT_NEAR(contactXs[2], 3.0e7, 1e-6);
}

//==============================================================================
TEST_F(Collision, DartSphereCylinderOrderSymmetry)
{
  auto cd = DARTCollisionDetector::create();

  auto sphereFrame = SimpleFrame::createShared(Frame::World());
  auto ellipsoidFrame = SimpleFrame::createShared(Frame::World());
  auto cylinderFrame = SimpleFrame::createShared(Frame::World());

  sphereFrame->setShape(std::make_shared<SphereShape>(0.25));
  ellipsoidFrame->setShape(
      std::make_shared<EllipsoidShape>(Eigen::Vector3d::Constant(0.5)));
  cylinderFrame->setShape(std::make_shared<CylinderShape>(0.5, 1.0));
  sphereFrame->setTranslation(Eigen::Vector3d(0.6, 0.0, 0.0));
  ellipsoidFrame->setTranslation(Eigen::Vector3d(0.6, 0.0, 0.0));

  auto sphereGroup = cd->createCollisionGroup(sphereFrame.get());
  auto cylinderGroup = cd->createCollisionGroup(cylinderFrame.get());

  CollisionOption option;
  option.enableContact = true;

  CollisionResult sphereFirst;
  ASSERT_TRUE(sphereGroup->collide(cylinderGroup.get(), option, &sphereFirst));
  ASSERT_EQ(sphereFirst.getNumContacts(), 1u);
  const auto& sphereContact = sphereFirst.getContact(0);
  EXPECT_EQ(sphereContact.collisionObject1->getShapeFrame(), sphereFrame.get());
  EXPECT_EQ(
      sphereContact.collisionObject2->getShapeFrame(), cylinderFrame.get());
  EXPECT_TRUE(sphereContact.normal.isApprox(Eigen::Vector3d::UnitX(), 1e-12));
  EXPECT_NEAR(sphereContact.penetrationDepth, 0.15, 1e-12);

  CollisionResult cylinderFirst;
  ASSERT_TRUE(
      cylinderGroup->collide(sphereGroup.get(), option, &cylinderFirst));
  ASSERT_EQ(cylinderFirst.getNumContacts(), 1u);
  const auto& cylinderContact = cylinderFirst.getContact(0);
  EXPECT_EQ(
      cylinderContact.collisionObject1->getShapeFrame(), cylinderFrame.get());
  EXPECT_EQ(
      cylinderContact.collisionObject2->getShapeFrame(), sphereFrame.get());
  EXPECT_TRUE(cylinderContact.normal.isApprox(-sphereContact.normal, 1e-12));
  EXPECT_TRUE(cylinderContact.point.isApprox(sphereContact.point, 1e-12));
  EXPECT_NEAR(
      cylinderContact.penetrationDepth, sphereContact.penetrationDepth, 1e-12);

  sphereFrame->setTranslation(Eigen::Vector3d(0.4, 0.0, 0.0));

  CollisionResult sphereInsideFirst;
  ASSERT_TRUE(
      sphereGroup->collide(cylinderGroup.get(), option, &sphereInsideFirst));
  ASSERT_EQ(sphereInsideFirst.getNumContacts(), 1u);
  const auto& sphereInsideContact = sphereInsideFirst.getContact(0);
  EXPECT_TRUE(
      sphereInsideContact.normal.isApprox(Eigen::Vector3d::UnitX(), 1e-12));
  EXPECT_NEAR(sphereInsideContact.penetrationDepth, 0.35, 1e-12);

  CollisionResult cylinderInsideFirst;
  ASSERT_TRUE(
      cylinderGroup->collide(sphereGroup.get(), option, &cylinderInsideFirst));
  ASSERT_EQ(cylinderInsideFirst.getNumContacts(), 1u);
  const auto& cylinderInsideContact = cylinderInsideFirst.getContact(0);
  EXPECT_TRUE(cylinderInsideContact.normal.isApprox(
      -sphereInsideContact.normal, 1e-12));
  EXPECT_TRUE(
      cylinderInsideContact.point.isApprox(sphereInsideContact.point, 1e-12));
  EXPECT_NEAR(
      cylinderInsideContact.penetrationDepth,
      sphereInsideContact.penetrationDepth,
      1e-12);

  auto ellipsoidGroup = cd->createCollisionGroup(ellipsoidFrame.get());

  CollisionResult ellipsoidFirst;
  ASSERT_TRUE(
      ellipsoidGroup->collide(cylinderGroup.get(), option, &ellipsoidFirst));
  ASSERT_EQ(ellipsoidFirst.getNumContacts(), 1u);
  const auto& ellipsoidContact = ellipsoidFirst.getContact(0);
  EXPECT_EQ(
      ellipsoidContact.collisionObject1->getShapeFrame(), ellipsoidFrame.get());
  EXPECT_EQ(
      ellipsoidContact.collisionObject2->getShapeFrame(), cylinderFrame.get());
  EXPECT_TRUE(
      ellipsoidContact.normal.isApprox(Eigen::Vector3d::UnitX(), 1e-12));
  EXPECT_NEAR(ellipsoidContact.penetrationDepth, 0.15, 1e-12);

  CollisionResult cylinderEllipsoid;
  ASSERT_TRUE(
      cylinderGroup->collide(ellipsoidGroup.get(), option, &cylinderEllipsoid));
  ASSERT_EQ(cylinderEllipsoid.getNumContacts(), 1u);
  const auto& cylinderEllipsoidContact = cylinderEllipsoid.getContact(0);
  EXPECT_EQ(
      cylinderEllipsoidContact.collisionObject1->getShapeFrame(),
      cylinderFrame.get());
  EXPECT_EQ(
      cylinderEllipsoidContact.collisionObject2->getShapeFrame(),
      ellipsoidFrame.get());
  EXPECT_TRUE(cylinderEllipsoidContact.normal.isApprox(
      -ellipsoidContact.normal, 1e-12));
  EXPECT_TRUE(
      cylinderEllipsoidContact.point.isApprox(ellipsoidContact.point, 1e-12));
  EXPECT_NEAR(
      cylinderEllipsoidContact.penetrationDepth,
      ellipsoidContact.penetrationDepth,
      1e-12);

  sphereFrame->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.6));

  CollisionResult sphereCapFirst;
  ASSERT_TRUE(
      sphereGroup->collide(cylinderGroup.get(), option, &sphereCapFirst));
  ASSERT_EQ(sphereCapFirst.getNumContacts(), 1u);
  const auto& sphereCapContact = sphereCapFirst.getContact(0);
  EXPECT_TRUE(
      sphereCapContact.normal.isApprox(Eigen::Vector3d::UnitZ(), 1e-12));
  EXPECT_NEAR(sphereCapContact.penetrationDepth, 0.15, 1e-12);

  CollisionResult cylinderCapFirst;
  ASSERT_TRUE(
      cylinderGroup->collide(sphereGroup.get(), option, &cylinderCapFirst));
  ASSERT_EQ(cylinderCapFirst.getNumContacts(), 1u);
  const auto& cylinderCapContact = cylinderCapFirst.getContact(0);
  EXPECT_TRUE(
      cylinderCapContact.normal.isApprox(-sphereCapContact.normal, 1e-12));
  EXPECT_TRUE(cylinderCapContact.point.isApprox(sphereCapContact.point, 1e-12));
  EXPECT_NEAR(cylinderCapContact.penetrationDepth, 0.15, 1e-12);

  sphereFrame->setTranslation(Eigen::Vector3d(0.0, 0.0, -0.6));

  CollisionResult sphereBottomCapFirst;
  ASSERT_TRUE(
      sphereGroup->collide(cylinderGroup.get(), option, &sphereBottomCapFirst));
  ASSERT_EQ(sphereBottomCapFirst.getNumContacts(), 1u);
  const auto& sphereBottomCapContact = sphereBottomCapFirst.getContact(0);
  EXPECT_TRUE(
      sphereBottomCapContact.normal.isApprox(-Eigen::Vector3d::UnitZ(), 1e-12));
  EXPECT_NEAR(sphereBottomCapContact.penetrationDepth, 0.15, 1e-12);

  CollisionResult cylinderBottomCapFirst;
  ASSERT_TRUE(cylinderGroup->collide(
      sphereGroup.get(), option, &cylinderBottomCapFirst));
  ASSERT_EQ(cylinderBottomCapFirst.getNumContacts(), 1u);
  const auto& cylinderBottomCapContact = cylinderBottomCapFirst.getContact(0);
  EXPECT_TRUE(cylinderBottomCapContact.normal.isApprox(
      -sphereBottomCapContact.normal, 1e-12));
  EXPECT_TRUE(cylinderBottomCapContact.point.isApprox(
      sphereBottomCapContact.point, 1e-12));
  EXPECT_NEAR(
      cylinderBottomCapContact.penetrationDepth,
      sphereBottomCapContact.penetrationDepth,
      1e-12);

  sphereFrame->setTranslation(Eigen::Vector3d(0.75, 0.0, 0.0));

  CollisionResult sphereSideTouch;
  ASSERT_TRUE(
      sphereGroup->collide(cylinderGroup.get(), option, &sphereSideTouch));
  ASSERT_EQ(sphereSideTouch.getNumContacts(), 1u);
  EXPECT_TRUE(sphereSideTouch.getContact(0).normal.isApprox(
      Eigen::Vector3d::UnitX(), 1e-12));
  EXPECT_NEAR(sphereSideTouch.getContact(0).penetrationDepth, 0.0, 1e-12);

  CollisionResult cylinderSideTouch;
  ASSERT_TRUE(
      cylinderGroup->collide(sphereGroup.get(), option, &cylinderSideTouch));
  ASSERT_EQ(cylinderSideTouch.getNumContacts(), 1u);
  EXPECT_TRUE(cylinderSideTouch.getContact(0).normal.isApprox(
      -sphereSideTouch.getContact(0).normal, 1e-12));
  EXPECT_TRUE(cylinderSideTouch.getContact(0).point.isApprox(
      sphereSideTouch.getContact(0).point, 1e-12));
  EXPECT_NEAR(cylinderSideTouch.getContact(0).penetrationDepth, 0.0, 1e-12);

  sphereFrame->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.75));

  CollisionResult sphereCapTouch;
  ASSERT_TRUE(
      sphereGroup->collide(cylinderGroup.get(), option, &sphereCapTouch));
  ASSERT_EQ(sphereCapTouch.getNumContacts(), 1u);
  EXPECT_TRUE(sphereCapTouch.getContact(0).normal.isApprox(
      Eigen::Vector3d::UnitZ(), 1e-12));
  EXPECT_NEAR(sphereCapTouch.getContact(0).penetrationDepth, 0.0, 1e-12);

  CollisionResult cylinderCapTouch;
  ASSERT_TRUE(
      cylinderGroup->collide(sphereGroup.get(), option, &cylinderCapTouch));
  ASSERT_EQ(cylinderCapTouch.getNumContacts(), 1u);
  EXPECT_TRUE(cylinderCapTouch.getContact(0).normal.isApprox(
      -sphereCapTouch.getContact(0).normal, 1e-12));
  EXPECT_TRUE(cylinderCapTouch.getContact(0).point.isApprox(
      sphereCapTouch.getContact(0).point, 1e-12));
  EXPECT_NEAR(cylinderCapTouch.getContact(0).penetrationDepth, 0.0, 1e-12);
}

//==============================================================================
TEST_F(Collision, DartCylinderPlaneLegacyHelperLinkage)
{
  Eigen::Isometry3d cylinderTf = Eigen::Isometry3d::Identity();
  cylinderTf.translation() = Eigen::Vector3d(0.0, 0.0, 10.0);

  CollisionResult result;
  EXPECT_EQ(
      collideCylinderPlane(
          nullptr,
          nullptr,
          0.5,
          0.5,
          cylinderTf,
          Eigen::Vector3d::UnitZ(),
          Eigen::Isometry3d::Identity(),
          result),
      0);
  EXPECT_EQ(result.getNumContacts(), 0u);
}

//==============================================================================
TEST_F(Collision, DartCylinderFinitePrimitivePairs)
{
  auto cd = DARTCollisionDetector::create();

  auto cylinderFrame = SimpleFrame::createShared(Frame::World());
  auto boxFrame = SimpleFrame::createShared(Frame::World());
  auto cylinder2Frame = SimpleFrame::createShared(Frame::World());

  cylinderFrame->setShape(std::make_shared<CylinderShape>(0.5, 1.0));
  boxFrame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  cylinder2Frame->setShape(std::make_shared<CylinderShape>(0.5, 1.0));

  auto cylinderGroup = cd->createCollisionGroup(cylinderFrame.get());
  auto boxGroup = cd->createCollisionGroup(boxFrame.get());
  auto cylinder2Group = cd->createCollisionGroup(cylinder2Frame.get());

  CollisionOption option;
  option.enableContact = true;

  boxFrame->setTranslation(Eigen::Vector3d(0.75, 0.0, 0.0));

  CollisionResult cylinderBox;
  ASSERT_TRUE(cylinderGroup->collide(boxGroup.get(), option, &cylinderBox));
  ASSERT_EQ(cylinderBox.getNumContacts(), 1u);
  EXPECT_EQ(
      cylinderBox.getContact(0).collisionObject1->getShapeFrame(),
      cylinderFrame.get());
  EXPECT_EQ(
      cylinderBox.getContact(0).collisionObject2->getShapeFrame(),
      boxFrame.get());
  EXPECT_TRUE(cylinderBox.getContact(0).point.allFinite());
  EXPECT_TRUE(cylinderBox.getContact(0).normal.allFinite());
  EXPECT_GT(cylinderBox.getContact(0).penetrationDepth, 0.0);

  CollisionResult boxCylinder;
  ASSERT_TRUE(boxGroup->collide(cylinderGroup.get(), option, &boxCylinder));
  ASSERT_EQ(boxCylinder.getNumContacts(), 1u);
  EXPECT_TRUE(boxCylinder.getContact(0).normal.isApprox(
      -cylinderBox.getContact(0).normal, 1e-8));
  EXPECT_NEAR(
      boxCylinder.getContact(0).penetrationDepth,
      cylinderBox.getContact(0).penetrationDepth,
      1e-8);

  boxFrame->setTranslation(Eigen::Vector3d(2.0, 0.0, 0.0));
  cylinderBox.clear();
  EXPECT_FALSE(cylinderGroup->collide(boxGroup.get(), option, &cylinderBox));
  EXPECT_EQ(cylinderBox.getNumContacts(), 0u);

  boxFrame->setTranslation(Eigen::Vector3d(1.0, 0.0, 0.0));
  cylinderBox.clear();
  ASSERT_TRUE(cylinderGroup->collide(boxGroup.get(), option, &cylinderBox));
  ASSERT_EQ(cylinderBox.getNumContacts(), 1u);
  EXPECT_NEAR(cylinderBox.getContact(0).penetrationDepth, 0.0, 1e-12);

  boxCylinder.clear();
  ASSERT_TRUE(boxGroup->collide(cylinderGroup.get(), option, &boxCylinder));
  ASSERT_EQ(boxCylinder.getNumContacts(), 1u);
  EXPECT_TRUE(boxCylinder.getContact(0).normal.isApprox(
      -cylinderBox.getContact(0).normal, 1e-8));
  EXPECT_NEAR(boxCylinder.getContact(0).penetrationDepth, 0.0, 1e-12);

  boxFrame->setTranslation(Eigen::Vector3d::Zero());
  cylinderFrame->setTranslation(Eigen::Vector3d(-1.0, 0.25, 0.0));

  boxCylinder.clear();
  ASSERT_TRUE(boxGroup->collide(cylinderGroup.get(), option, &boxCylinder));
  ASSERT_EQ(boxCylinder.getNumContacts(), 1u);
  EXPECT_TRUE(boxCylinder.getContact(0).point.allFinite());
  EXPECT_TRUE(boxCylinder.getContact(0).normal.allFinite());
  EXPECT_NEAR(boxCylinder.getContact(0).normal.norm(), 1.0, 1e-12);
  EXPECT_NEAR(boxCylinder.getContact(0).penetrationDepth, 0.0, 1e-12);

  cylinderBox.clear();
  ASSERT_TRUE(cylinderGroup->collide(boxGroup.get(), option, &cylinderBox));
  ASSERT_EQ(cylinderBox.getNumContacts(), 1u);
  EXPECT_TRUE(cylinderBox.getContact(0).normal.isApprox(
      -boxCylinder.getContact(0).normal, 1e-8));
  EXPECT_NEAR(cylinderBox.getContact(0).penetrationDepth, 0.0, 1e-12);

  cylinderFrame->setTranslation(Eigen::Vector3d::Zero());
  cylinder2Frame->setTranslation(Eigen::Vector3d(0.75, 0.0, 0.0));

  CollisionResult cylinderCylinder;
  ASSERT_TRUE(
      cylinderGroup->collide(cylinder2Group.get(), option, &cylinderCylinder));
  ASSERT_EQ(cylinderCylinder.getNumContacts(), 1u);
  EXPECT_TRUE(cylinderCylinder.getContact(0).point.allFinite());
  EXPECT_TRUE(cylinderCylinder.getContact(0).normal.allFinite());
  EXPECT_GT(cylinderCylinder.getContact(0).penetrationDepth, 0.0);

  cylinder2Frame->setTranslation(Eigen::Vector3d(2.0, 0.0, 0.0));
  cylinderCylinder.clear();
  EXPECT_FALSE(
      cylinderGroup->collide(cylinder2Group.get(), option, &cylinderCylinder));
  EXPECT_EQ(cylinderCylinder.getNumContacts(), 0u);

  cylinder2Frame->setTranslation(Eigen::Vector3d(1.0, 0.0, 0.0));
  cylinderCylinder.clear();
  ASSERT_TRUE(
      cylinderGroup->collide(cylinder2Group.get(), option, &cylinderCylinder));
  ASSERT_EQ(cylinderCylinder.getNumContacts(), 1u);
  EXPECT_NEAR(cylinderCylinder.getContact(0).penetrationDepth, 0.0, 1e-12);
}

//==============================================================================
void testOptions(const std::shared_ptr<CollisionDetector>& cd)
{
  auto simpleFrame1 = SimpleFrame::createShared(Frame::World());
  auto simpleFrame2 = SimpleFrame::createShared(Frame::World());
  auto simpleFrame3 = SimpleFrame::createShared(Frame::World());

  ShapePtr shape1(new BoxShape(Eigen::Vector3d(1.0, 1.0, 1.0)));
  ShapePtr shape2(new BoxShape(Eigen::Vector3d(0.5, 0.5, 0.5)));
  ShapePtr shape3(new BoxShape(Eigen::Vector3d(0.5, 0.5, 0.5)));
  simpleFrame1->setShape(shape1);
  simpleFrame2->setShape(shape2);
  simpleFrame3->setShape(shape3);

  Eigen::Vector3d pos1 = Eigen::Vector3d(0.0, 0.0, -0.5);
  Eigen::Vector3d pos2 = Eigen::Vector3d(0.0, 0.5, 0.25);
  Eigen::Vector3d pos3 = Eigen::Vector3d(0.0, -0.5, 0.25);
  simpleFrame1->setTranslation(pos1);
  simpleFrame2->setTranslation(pos2);
  simpleFrame3->setTranslation(pos3);

  auto group = cd->createCollisionGroup(simpleFrame1.get(), simpleFrame2.get());
  EXPECT_EQ(group->getNumShapeFrames(), 2u);

  collision::CollisionOption option;
  collision::CollisionResult result;

  result.clear();
  option.maxNumContacts = 1000u;
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_EQ(result.getNumContacts(), 4u);

  result.clear();
  option.maxNumContactsPerPair = 2u;
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_GT(result.getNumContacts(), 0u);
  EXPECT_LE(result.getNumContacts(), 2u);

  option.maxNumContactsPerPair = 0u;
  result.clear();
  option.maxNumContacts = 2u;
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_EQ(result.getNumContacts(), 2u);

  group->addShapeFrame(simpleFrame3.get());
  result.clear();
  option.maxNumContacts = 1u;
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_EQ(result.getNumContacts(), 1u);

  // Binary check without passing result
  EXPECT_TRUE(group->collide(option));

  // Binary check without passing option and result
  EXPECT_TRUE(group->collide());

  // Zero maximum number of contacts
  option.maxNumContacts = 0u;
  option.enableContact = true;
  EXPECT_TRUE(group->collide());
  EXPECT_FALSE(group->collide(option));
  EXPECT_FALSE(group->collide(option, nullptr));
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_EQ(result.getNumContacts(), 0u);
  EXPECT_FALSE(result.isCollision());
}

//==============================================================================
void testCylinderCylinder(const std::shared_ptr<CollisionDetector>& cd)
{
  auto simpleFrame1 = SimpleFrame::createShared(Frame::World());
  auto simpleFrame2 = SimpleFrame::createShared(Frame::World());

  auto shape1 = std::make_shared<CylinderShape>(1.0, 1.0);
  auto shape2 = std::make_shared<CylinderShape>(0.5, 1.0);

  simpleFrame1->setShape(shape1);
  simpleFrame2->setShape(shape2);

  auto group = cd->createCollisionGroup(simpleFrame1.get(), simpleFrame2.get());

  EXPECT_EQ(group->getNumShapeFrames(), 2u);

  collision::CollisionOption option;
  option.enableContact = true;

  collision::CollisionResult result;

  result.clear();
  simpleFrame1->setTranslation(Eigen::Vector3d::Zero());
  simpleFrame2->setTranslation(Eigen::Vector3d(2.0, 0.0, 0.0));
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_TRUE(result.getNumContacts() == 0u);

  result.clear();
  simpleFrame1->setTranslation(Eigen::Vector3d::Zero());
  simpleFrame2->setTranslation(Eigen::Vector3d(0.75, 0.0, 0.0));
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_TRUE(result.getNumContacts() >= 1u);
}

//==============================================================================
TEST_F(Collision, testCylinderCylinder)
{
  auto fcl_mesh_dart = FCLCollisionDetector::create();
  fcl_mesh_dart->setPrimitiveShapeType(FCLCollisionDetector::MESH);
  fcl_mesh_dart->setContactPointComputationMethod(FCLCollisionDetector::DART);
  testCylinderCylinder(fcl_mesh_dart);

  // auto fcl_prim_fcl = FCLCollisionDetector::create();
  // fcl_prim_fcl->setPrimitiveShapeType(FCLCollisionDetector::MESH);
  // fcl_prim_fcl->setContactPointComputationMethod(FCLCollisionDetector::FCL);
  // testCylinderCylinder(fcl_prim_fcl);

  // auto fcl_mesh_fcl = FCLCollisionDetector::create();
  // fcl_mesh_fcl->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  // fcl_mesh_fcl->setContactPointComputationMethod(FCLCollisionDetector::DART);
  // testCylinderCylinder(fcl_mesh_fcl);

  // auto fcl_mesh_fcl = FCLCollisionDetector::create();
  // fcl_mesh_fcl->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  // fcl_mesh_fcl->setContactPointComputationMethod(FCLCollisionDetector::FCL);
  // testCylinderCylinder(fcl_mesh_fcl);

#if HAVE_ODE
  auto ode = OdeCollisionDetector::create();
  testCylinderCylinder(ode);
#endif

#if HAVE_BULLET
  auto bullet = BulletCollisionDetector::create();
  testCylinderCylinder(bullet);
#endif

  auto dart = DARTCollisionDetector::create();
  testCylinderCylinder(dart);
}

//==============================================================================
void testConeCone(const std::shared_ptr<CollisionDetector>& cd)
{
  auto simpleFrame1 = SimpleFrame::createShared(Frame::World());
  auto simpleFrame2 = SimpleFrame::createShared(Frame::World());

  auto shape1 = std::make_shared<ConeShape>(1.0, 1.0);
  auto shape2 = std::make_shared<ConeShape>(0.5, 1.0);

  simpleFrame1->setShape(shape1);
  simpleFrame2->setShape(shape2);

  auto group = cd->createCollisionGroup(simpleFrame1.get(), simpleFrame2.get());

  EXPECT_EQ(group->getNumShapeFrames(), 2u);

  collision::CollisionOption option;
  option.enableContact = true;

  collision::CollisionResult result;

  result.clear();
  simpleFrame1->setTranslation(Eigen::Vector3d::Zero());
  simpleFrame2->setTranslation(Eigen::Vector3d(2.0, 0.0, 0.0));
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_TRUE(result.getNumContacts() == 0u);

  result.clear();
  simpleFrame1->setTranslation(Eigen::Vector3d::Zero());
  simpleFrame2->setTranslation(Eigen::Vector3d(0.75, 0.0, 0.0));
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_TRUE(result.getNumContacts() >= 1u);
}

//==============================================================================
TEST_F(Collision, testConeCone)
{
  {
    SCOPED_TRACE("FCLCollisionDetector (MESH, DART)");
    auto fcl_mesh_dart = FCLCollisionDetector::create();
    fcl_mesh_dart->setPrimitiveShapeType(FCLCollisionDetector::MESH);
    fcl_mesh_dart->setContactPointComputationMethod(FCLCollisionDetector::DART);
    testConeCone(fcl_mesh_dart);
  }

  {
    SCOPED_TRACE("FCLCollisionDetector (MESH, FCL)");
    auto fcl_mesh_fcl = FCLCollisionDetector::create();
    fcl_mesh_fcl->setPrimitiveShapeType(FCLCollisionDetector::MESH);
    fcl_mesh_fcl->setContactPointComputationMethod(FCLCollisionDetector::FCL);
    testConeCone(fcl_mesh_fcl);
  }

  {
    SCOPED_TRACE("FCLCollisionDetector (PRIMITIVE, DART)");
    auto fcl_prim_dart = FCLCollisionDetector::create();
    fcl_prim_dart->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
    fcl_prim_dart->setContactPointComputationMethod(FCLCollisionDetector::DART);
    testConeCone(fcl_prim_dart);
  }

  {
    SCOPED_TRACE("FCLCollisionDetector (PRIMITIVE, FCL)");
    auto fcl_prim_fcl = FCLCollisionDetector::create();
    fcl_prim_fcl->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
    fcl_prim_fcl->setContactPointComputationMethod(FCLCollisionDetector::FCL);
    testConeCone(fcl_prim_fcl);
  }

#if HAVE_ODE
  {
      // SCOPED_TRACE("OdeCollisionDetector");
      // auto ode = OdeCollisionDetector::create();
      // testConeCone(ode);
  }
#endif

#if HAVE_BULLET
  {
    SCOPED_TRACE("BulletCollisionDetector");
    auto bullet = BulletCollisionDetector::create();
    testConeCone(bullet);
  }
#endif

  {
    // SCOPED_TRACE("DARTCollisionDetector");
    // auto dart = DARTCollisionDetector::create();
    // testConeCone(dart);
  }
}

//==============================================================================
TEST_F(Collision, FCLDeterministicPairOrdering)
{
  auto detector = FCLCollisionDetector::create();
  detector->setPrimitiveShapeType(FCLCollisionDetector::MESH);
  detector->setContactPointComputationMethod(FCLCollisionDetector::DART);

  auto frameA = SimpleFrame::createShared(Frame::World());
  auto frameB = SimpleFrame::createShared(Frame::World());

  ShapePtr boxShape(new BoxShape(Eigen::Vector3d::Constant(0.1)));
  frameA->setShape(boxShape);
  frameB->setShape(boxShape);

  // Slightly overlap the boxes to guarantee contact.
  frameA->setTranslation(Eigen::Vector3d::Zero());
  frameB->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.05));

  collision::CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 1u;

  collision::CollisionResult abResult;
  auto groupAB1 = detector->createCollisionGroup(frameA.get());
  auto groupAB2 = detector->createCollisionGroup(frameB.get());
  EXPECT_TRUE(groupAB1->collide(groupAB2.get(), option, &abResult));
  ASSERT_EQ(abResult.getNumContacts(), 1u);

  collision::CollisionResult baResult;
  auto groupBA1 = detector->createCollisionGroup(frameB.get());
  auto groupBA2 = detector->createCollisionGroup(frameA.get());
  EXPECT_TRUE(groupBA1->collide(groupBA2.get(), option, &baResult));
  ASSERT_EQ(baResult.getNumContacts(), 1u);

  const auto& contactAB = abResult.getContact(0);
  const auto& contactBA = baResult.getContact(0);

  // The collision pair ordering should be deterministic regardless of the
  // groups passed into collide. Canonicalize the ordering by lexicographic
  // comparison of shape-frame names; flip the normal if we swap.
  const auto name1 = contactAB.getShapeFrame1()->getName();
  const auto name2 = contactAB.getShapeFrame2()->getName();
  auto orderedAB = contactAB;
  if (name2 < name1) {
    std::swap(orderedAB.collisionObject1, orderedAB.collisionObject2);
    orderedAB.normal = -orderedAB.normal;
  }

  const auto baName1 = contactBA.getShapeFrame1()->getName();
  const auto baName2 = contactBA.getShapeFrame2()->getName();
  auto orderedBA = contactBA;
  if (baName2 < baName1) {
    std::swap(orderedBA.collisionObject1, orderedBA.collisionObject2);
    orderedBA.normal = -orderedBA.normal;
  }

  ASSERT_GT(orderedAB.normal.norm(), 0.0);
  ASSERT_GT(orderedBA.normal.norm(), 0.0);
  EXPECT_EQ(
      orderedAB.getShapeFrame1()->getName(),
      orderedBA.getShapeFrame1()->getName());
  EXPECT_EQ(
      orderedAB.getShapeFrame2()->getName(),
      orderedBA.getShapeFrame2()->getName());
}

//==============================================================================
static void checkDeterministicPairOrderingForMethod(
    FCLCollisionDetector::ContactPointComputationMethod method)
{
  auto detector = FCLCollisionDetector::create();
  detector->setPrimitiveShapeType(FCLCollisionDetector::MESH);
  detector->setContactPointComputationMethod(method);

  auto frameA = SimpleFrame::createShared(Frame::World(), "A");
  auto frameB = SimpleFrame::createShared(Frame::World(), "B");

  ShapePtr boxShape(new BoxShape(Eigen::Vector3d::Constant(0.1)));
  frameA->setShape(boxShape);
  frameB->setShape(boxShape);

  // Slightly overlap the boxes to guarantee contact.
  frameA->setTranslation(Eigen::Vector3d::Zero());
  frameB->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.05));

  collision::CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 2u;

  collision::CollisionResult abResult;
  auto groupAB1 = detector->createCollisionGroup(frameA.get());
  auto groupAB2 = detector->createCollisionGroup(frameB.get());
  EXPECT_TRUE(groupAB1->collide(groupAB2.get(), option, &abResult));
  ASSERT_GE(abResult.getNumContacts(), 1u);

  collision::CollisionResult baResult;
  auto groupBA1 = detector->createCollisionGroup(frameB.get());
  auto groupBA2 = detector->createCollisionGroup(frameA.get());
  EXPECT_TRUE(groupBA1->collide(groupBA2.get(), option, &baResult));
  ASSERT_GE(baResult.getNumContacts(), 1u);

  // Both orderings should surface the same canonical pair ordering.
  for (const auto* res : {&abResult, &baResult}) {
    for (std::size_t i = 0; i < res->getNumContacts(); ++i) {
      const auto& contact = res->getContact(i);
      EXPECT_EQ(contact.getShapeFrame1()->getName(), "A");
      EXPECT_EQ(contact.getShapeFrame2()->getName(), "B");
      ASSERT_GT(contact.normal.norm(), 0.0);
    }
  }
}

//==============================================================================
TEST_F(Collision, FCLDeterministicPairOrderingMeshPaths)
{
  SCOPED_TRACE("FCL mesh contact via DART path");
  checkDeterministicPairOrderingForMethod(FCLCollisionDetector::DART);

  SCOPED_TRACE("FCL mesh contact via FCL path");
  checkDeterministicPairOrderingForMethod(FCLCollisionDetector::FCL);
}

//==============================================================================
TEST_F(Collision, FCLPrimitiveSharedShapeNormalsFollowCanonicalOrder)
{
  auto detector = FCLCollisionDetector::create();
  detector->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  detector->setContactPointComputationMethod(FCLCollisionDetector::FCL);

  auto frameA = SimpleFrame::createShared(Frame::World(), "A");
  auto frameB = SimpleFrame::createShared(Frame::World(), "B");

  ShapePtr boxShape(new BoxShape(Eigen::Vector3d::Constant(0.1)));
  frameA->setShape(boxShape);
  frameB->setShape(boxShape);

  frameA->setTranslation(Eigen::Vector3d::Zero());
  frameB->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.05));

  collision::CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 4u;

  auto groupA = detector->createCollisionGroup(frameA.get());
  auto groupB = detector->createCollisionGroup(frameB.get());

  for (const bool abOrder : {true, false}) {
    collision::CollisionResult result;
    auto* g1 = abOrder ? groupA.get() : groupB.get();
    auto* g2 = abOrder ? groupB.get() : groupA.get();

    EXPECT_TRUE(g1->collide(g2, option, &result));
    ASSERT_GE(result.getNumContacts(), 1u);

    for (std::size_t i = 0; i < result.getNumContacts(); ++i) {
      const auto& contact = result.getContact(i);
      EXPECT_EQ(contact.getShapeFrame1()->getName(), "A");
      EXPECT_EQ(contact.getShapeFrame2()->getName(), "B");
      EXPECT_TRUE(contact.normal.isApprox(-Eigen::Vector3d::UnitZ(), 1e-8))
          << contact.normal.transpose();
    }
  }
}

//==============================================================================
TEST_F(Collision, FCLDeterministicEqualKeyOrdering)
{
  // Two identically-named SimpleFrames produce the SAME deterministic key, so
  // the pair ordering cannot be resolved by key comparison alone. The tie is
  // then broken on the underlying FCL object address, which yields a canonical
  // ordering that is stable within a process and independent of the argument
  // order passed to collide(). Verify that the resulting ordering is consistent
  // across repeated detect calls and both argument orders.
  auto detector = FCLCollisionDetector::create();

  // Identical names -> identical collision-object keys.
  auto frameA = SimpleFrame::createShared(Frame::World(), "dup");
  auto frameB = SimpleFrame::createShared(Frame::World(), "dup");

  ShapePtr boxA(new BoxShape(Eigen::Vector3d(1.0, 1.0, 1.0)));
  ShapePtr boxB(new BoxShape(Eigen::Vector3d(1.0, 1.0, 1.0)));
  frameA->setShape(boxA);
  frameB->setShape(boxB);

  // Overlap the boxes to guarantee a contact.
  frameA->setTranslation(Eigen::Vector3d::Zero());
  frameB->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.5));

  collision::CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 1u;

  auto groupA = detector->createCollisionGroup(frameA.get());
  auto groupB = detector->createCollisionGroup(frameB.get());

  // Repeat the detection in both argument orders. With equal keys the pair
  // ordering is resolved by the deterministic address tie-break, so the
  // canonical object ordering must be identical across every call regardless of
  // which group is passed first, and the normal must be stable across repeated
  // calls in the same argument order.
  const CollisionObject* canonicalObj1 = nullptr;
  const CollisionObject* canonicalObj2 = nullptr;
  Eigen::Vector3d abNormal = Eigen::Vector3d::Zero();
  Eigen::Vector3d baNormal = Eigen::Vector3d::Zero();
  bool haveAbNormal = false;
  bool haveBaNormal = false;

  for (int iter = 0; iter < 5; ++iter) {
    for (bool abOrder : {true, false}) {
      collision::CollisionResult result;
      auto* g1 = abOrder ? groupA.get() : groupB.get();
      auto* g2 = abOrder ? groupB.get() : groupA.get();
      EXPECT_TRUE(g1->collide(g2, option, &result));
      ASSERT_EQ(result.getNumContacts(), 1u);

      const auto& contact = result.getContact(0);
      ASSERT_GT(contact.normal.norm(), 0.0);

      // The two collision objects share the same key, so the pair ordering is
      // resolved by the deterministic address tie-break rather than by the key.
      const auto* obj1
          = dynamic_cast<const FCLCollisionObject*>(contact.collisionObject1);
      const auto* obj2
          = dynamic_cast<const FCLCollisionObject*>(contact.collisionObject2);
      ASSERT_NE(obj1, nullptr);
      ASSERT_NE(obj2, nullptr);
      EXPECT_EQ(obj1->getKey(), obj2->getKey());

      // The canonical object ordering must match for every call, regardless of
      // the argument order or repetition (this is what the tie-break fixes).
      if (canonicalObj1 == nullptr) {
        canonicalObj1 = contact.collisionObject1;
        canonicalObj2 = contact.collisionObject2;
      } else {
        EXPECT_EQ(contact.collisionObject1, canonicalObj1);
        EXPECT_EQ(contact.collisionObject2, canonicalObj2);
      }

      // The contact normal must be stable across repeated detect calls in the
      // same argument order. (FCL's box-box contact geometry may differ between
      // the AB and BA argument orders, so the normals are compared per order.)
      if (abOrder) {
        if (!haveAbNormal) {
          abNormal = contact.normal;
          haveAbNormal = true;
        } else {
          EXPECT_TRUE(contact.normal.isApprox(abNormal));
        }
      } else {
        if (!haveBaNormal) {
          baNormal = contact.normal;
          haveBaNormal = true;
        } else {
          EXPECT_TRUE(contact.normal.isApprox(baNormal));
        }
      }
    }
  }
}

//==============================================================================
void testCapsuleCapsule(const std::shared_ptr<CollisionDetector>& cd)
{
  auto simpleFrame1 = SimpleFrame::createShared(Frame::World());
  auto simpleFrame2 = SimpleFrame::createShared(Frame::World());

  auto shape1 = std::make_shared<CapsuleShape>(1.0, 1.0);
  auto shape2 = std::make_shared<CapsuleShape>(0.5, 1.0);

  simpleFrame1->setShape(shape1);
  simpleFrame2->setShape(shape2);

  auto group = cd->createCollisionGroup(simpleFrame1.get(), simpleFrame2.get());

  EXPECT_EQ(group->getNumShapeFrames(), 2u);

  collision::CollisionOption option;
  option.enableContact = true;

  collision::CollisionResult result;

  result.clear();
  simpleFrame1->setTranslation(Eigen::Vector3d::Zero());
  simpleFrame2->setTranslation(Eigen::Vector3d(2.0, 0.0, 0.0));
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_TRUE(result.getNumContacts() == 0u);

  result.clear();
  simpleFrame1->setTranslation(Eigen::Vector3d::Zero());
  simpleFrame2->setTranslation(Eigen::Vector3d(0.74, 0.0, 0.0));
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_TRUE(result.getNumContacts() >= 1u);
}

//==============================================================================
TEST_F(Collision, testCapsuleCapsule)
{
  // auto fcl_mesh_dart = FCLCollisionDetector::create();
  // fcl_mesh_dart->setPrimitiveShapeType(FCLCollisionDetector::MESH);
  // fcl_mesh_dart->setContactPointComputationMethod(FCLCollisionDetector::DART);
  // testCapsuleCapsule(fcl_mesh_dart);

  // auto fcl_prim_fcl = FCLCollisionDetector::create();
  // fcl_prim_fcl->setPrimitiveShapeType(FCLCollisionDetector::MESH);
  // fcl_prim_fcl->setContactPointComputationMethod(FCLCollisionDetector::FCL);
  // testCapsuleCapsule(fcl_prim_fcl);

  // auto fcl_mesh_fcl = FCLCollisionDetector::create();
  // fcl_mesh_fcl->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  // fcl_mesh_fcl->setContactPointComputationMethod(FCLCollisionDetector::DART);
  // testCapsuleCapsule(fcl_mesh_fcl);

  // auto fcl_mesh_fcl = FCLCollisionDetector::create();
  // fcl_mesh_fcl->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  // fcl_mesh_fcl->setContactPointComputationMethod(FCLCollisionDetector::FCL);
  // testCapsuleCapsule(fcl_mesh_fcl);

#if HAVE_ODE
  auto ode = OdeCollisionDetector::create();
  testCapsuleCapsule(ode);
#endif

#if HAVE_BULLET
  auto bullet = BulletCollisionDetector::create();
  testCapsuleCapsule(bullet);
#endif

  auto dart = DARTCollisionDetector::create();
  testCapsuleCapsule(dart);
}

//==============================================================================
TEST_F(Collision, DartCapsulePrimitivePairs)
{
  auto cd = DARTCollisionDetector::create();

  auto capsuleFrame = SimpleFrame::createShared(Frame::World());
  auto sphereFrame = SimpleFrame::createShared(Frame::World());
  auto boxFrame = SimpleFrame::createShared(Frame::World());
  auto cylinderFrame = SimpleFrame::createShared(Frame::World());
  auto capsule2Frame = SimpleFrame::createShared(Frame::World());
  auto planeFrame = SimpleFrame::createShared(Frame::World());

  capsuleFrame->setShape(std::make_shared<CapsuleShape>(0.25, 1.0));
  sphereFrame->setShape(std::make_shared<SphereShape>(0.25));
  boxFrame->setShape(
      std::make_shared<BoxShape>(Eigen::Vector3d::Constant(0.4)));
  cylinderFrame->setShape(std::make_shared<CylinderShape>(0.25, 1.0));
  capsule2Frame->setShape(std::make_shared<CapsuleShape>(0.25, 1.0));
  planeFrame->setShape(
      std::make_shared<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0));

  capsuleFrame->setTranslation(Eigen::Vector3d::Zero());
  sphereFrame->setTranslation(Eigen::Vector3d(0.45, 0.0, 0.0));
  boxFrame->setTranslation(Eigen::Vector3d(0.4, 0.0, 0.0));
  cylinderFrame->setTranslation(Eigen::Vector3d(0.45, 0.0, 0.0));
  capsule2Frame->setTranslation(Eigen::Vector3d(0.45, 0.0, 0.0));

  CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 8u;

  auto expectContactWithPoint = [&](const std::shared_ptr<SimpleFrame>& first,
                                    const std::shared_ptr<SimpleFrame>& second,
                                    const Eigen::Vector3d& expectedNormal,
                                    double expectedDepth,
                                    const Eigen::Vector3d* expectedPoint) {
    auto firstGroup = cd->createCollisionGroup(first.get());
    auto secondGroup = cd->createCollisionGroup(second.get());

    CollisionResult result;
    ASSERT_TRUE(firstGroup->collide(secondGroup.get(), option, &result));
    ASSERT_GE(result.getNumContacts(), 1u);

    const auto& contact = result.getContact(0);
    EXPECT_EQ(contact.collisionObject1->getShapeFrame(), first.get());
    EXPECT_EQ(contact.collisionObject2->getShapeFrame(), second.get());
    EXPECT_TRUE(contact.point.allFinite());
    EXPECT_TRUE(contact.normal.isApprox(expectedNormal, 1e-12))
        << contact.normal.transpose();
    EXPECT_NEAR(contact.penetrationDepth, expectedDepth, 1e-12);
    if (expectedPoint != nullptr) {
      EXPECT_TRUE(contact.point.isApprox(*expectedPoint, 1e-12))
          << contact.point.transpose();
    }
  };

  auto expectContact = [&](const std::shared_ptr<SimpleFrame>& first,
                           const std::shared_ptr<SimpleFrame>& second,
                           const Eigen::Vector3d& expectedNormal,
                           double expectedDepth) {
    expectContactWithPoint(
        first, second, expectedNormal, expectedDepth, nullptr);
  };

  auto expectPositiveContact = [&](const std::shared_ptr<SimpleFrame>& first,
                                   const std::shared_ptr<SimpleFrame>& second) {
    auto firstGroup = cd->createCollisionGroup(first.get());
    auto secondGroup = cd->createCollisionGroup(second.get());

    CollisionResult result;
    ASSERT_TRUE(firstGroup->collide(secondGroup.get(), option, &result));
    ASSERT_GE(result.getNumContacts(), 1u);

    const auto& contact = result.getContact(0);
    EXPECT_EQ(contact.collisionObject1->getShapeFrame(), first.get());
    EXPECT_EQ(contact.collisionObject2->getShapeFrame(), second.get());
    EXPECT_TRUE(contact.point.allFinite());
    EXPECT_TRUE(contact.normal.allFinite());
    EXPECT_GT(contact.penetrationDepth, 0.0);
  };

  expectContact(capsuleFrame, sphereFrame, -Eigen::Vector3d::UnitX(), 0.05);
  expectContact(sphereFrame, capsuleFrame, Eigen::Vector3d::UnitX(), 0.05);
  expectContact(capsuleFrame, boxFrame, -Eigen::Vector3d::UnitX(), 0.05);
  expectContact(boxFrame, capsuleFrame, Eigen::Vector3d::UnitX(), 0.05);
  expectContact(capsuleFrame, cylinderFrame, -Eigen::Vector3d::UnitX(), 0.05);
  expectContact(cylinderFrame, capsuleFrame, Eigen::Vector3d::UnitX(), 0.05);
  expectContact(capsuleFrame, capsule2Frame, -Eigen::Vector3d::UnitX(), 0.05);

  capsuleFrame->setTranslation(Eigen::Vector3d::Zero());
  capsule2Frame->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.2));
  expectContact(capsuleFrame, capsule2Frame, -Eigen::Vector3d::UnitZ(), 0.5);
  expectContact(capsule2Frame, capsuleFrame, Eigen::Vector3d::UnitZ(), 0.5);

  capsuleFrame->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.7));
  expectContact(capsuleFrame, planeFrame, Eigen::Vector3d::UnitZ(), 0.05);
  expectContact(planeFrame, capsuleFrame, -Eigen::Vector3d::UnitZ(), 0.05);

  Eigen::Isometry3d horizontalCapsuleTf = Eigen::Isometry3d::Identity();
  horizontalCapsuleTf.linear()
      = Eigen::AngleAxisd(0.5 * constantsd::pi(), Eigen::Vector3d::UnitY())
            .toRotationMatrix();
  capsuleFrame->setTransform(horizontalCapsuleTf);
  capsuleFrame->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.2));
  const Eigen::Vector3d centeredPlaneContact(0.0, 0.0, 0.025);
  expectContactWithPoint(
      capsuleFrame,
      planeFrame,
      Eigen::Vector3d::UnitZ(),
      0.05,
      &centeredPlaneContact);
  expectContactWithPoint(
      planeFrame,
      capsuleFrame,
      -Eigen::Vector3d::UnitZ(),
      0.05,
      &centeredPlaneContact);

  capsuleFrame->setTransform(horizontalCapsuleTf);
  sphereFrame->setTranslation(Eigen::Vector3d(0.95, 0.0, 0.0));
  expectContact(capsuleFrame, sphereFrame, -Eigen::Vector3d::UnitX(), 0.05);

  capsuleFrame->setShape(std::make_shared<CapsuleShape>(0.1, 0.2));
  cylinderFrame->setShape(std::make_shared<CylinderShape>(0.5, 1.0));
  capsuleFrame->setTransform(Eigen::Isometry3d::Identity());
  cylinderFrame->setTransform(Eigen::Isometry3d::Identity());
  expectPositiveContact(capsuleFrame, cylinderFrame);
  expectPositiveContact(cylinderFrame, capsuleFrame);

  capsuleFrame->setShape(std::make_shared<CapsuleShape>(0.1, 2.0));
  cylinderFrame->setShape(std::make_shared<CylinderShape>(0.25, 1.0));
  capsuleFrame->setTransform(horizontalCapsuleTf);
  expectContact(capsuleFrame, cylinderFrame, Eigen::Vector3d::UnitY(), 0.35);
  expectContact(cylinderFrame, capsuleFrame, -Eigen::Vector3d::UnitY(), 0.35);

  cylinderFrame->setShape(std::make_shared<CylinderShape>(0.5, 1.0));
  Eigen::Isometry3d horizontalCapsuleAboveCapTf = horizontalCapsuleTf;
  horizontalCapsuleAboveCapTf.translation() = Eigen::Vector3d(0.0, 0.0, 0.55);
  capsuleFrame->setTransform(horizontalCapsuleAboveCapTf);
  expectContact(capsuleFrame, cylinderFrame, Eigen::Vector3d::UnitZ(), 0.05);
  expectContact(cylinderFrame, capsuleFrame, -Eigen::Vector3d::UnitZ(), 0.05);
}

//==============================================================================
void testPlane(const std::shared_ptr<CollisionDetector>& cd)
{
  auto planeFrame = SimpleFrame::createShared(Frame::World());
  auto sphereFrame = SimpleFrame::createShared(Frame::World());
  auto boxFrame = SimpleFrame::createShared(Frame::World());

  auto plane = std::make_shared<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0);
  auto sphere = std::make_shared<SphereShape>(0.5);
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));

  planeFrame->setShape(plane);
  sphereFrame->setShape(sphere);
  boxFrame->setShape(box);

  auto group = cd->createCollisionGroup(
      planeFrame.get(), sphereFrame.get(), boxFrame.get());

  EXPECT_EQ(group->getNumShapeFrames(), 3u);

  collision::CollisionOption option;
  option.enableContact = true;

  collision::CollisionResult result;

  result.clear();
  sphereFrame->setTranslation(Eigen::Vector3d(-10.0, 0.0, 1.0));
  boxFrame->setTranslation(Eigen::Vector3d(-8.0, 0.0, 1.0));
  EXPECT_FALSE(group->collide(option, &result));

  result.clear();
  sphereFrame->setTranslation(Eigen::Vector3d(-10.0, 0.0, 0.49));
  boxFrame->setTranslation(Eigen::Vector3d(-8.0, 0.0, 0.49));
  EXPECT_TRUE(group->collide(option, &result));
}

//==============================================================================
TEST_F(Collision, testPlane)
{
#if HAVE_ODE
  auto ode = OdeCollisionDetector::create();
  testPlane(ode);
#endif
}

//==============================================================================
/// \param[in] collidesUnderTerrain Set to true if the collision engine returns
/// collisions when a shape is underneath the terrain, but still above the
/// minimum height. If false, only intersections with the surface mesh will be
/// detected.
/// \param[in] extendsUntilGroundPlane Set to true if the collision engine
/// extends the terrain until the plane z=0
/// \param[in] odeThck: for ODE, use this thickness underneath the heightfield
/// to adjust collision checks.
///
/// \sa dGeomHeightfieldDataBuild*().
template <typename S>
void testHeightmapBox(
    CollisionDetector* cd,
    const bool collidesUnderTerrain = true,
    const bool extendsUntilGroundPlane = false,
    const S odeThck = 0)
{
  using Vector3 = Eigen::Matrix<S, 3, 1>;

  ///////////////////////////////////////
  // Set test parameters.
  // The height field will have a flat, even
  // slope spanned by four corner vertices
  ///////////////////////////////////////

  // size of box
  const S boxSize = S(0.1);
  // terrain scale in x and y direction
  const S terrainScale = S(2.0);
  // z values scale
  const S zScale = S(2.0);

  // minimum hand maximum height of terrain to use
  const S minH = 1.0; // note: ODE doesn't behave well with negative heights
  const S maxH = 3.0;
  // adjusted minimum height: If minH > 0, and extendsUntilGroundPlane true,
  // then the minimum height is actually 0.
  const S adjMinH = (extendsUntilGroundPlane && (minH > S(0))) ? 0.0 : minH;
  const S halfHeight = minH + (maxH - minH) / S(2);
  // ODE thickness is only used if there is not already a layer of this
  // thickness due to a minH > 0 (for ODE, extendsUntilGroundPlane is true)
  const S useOdeThck
      = (odeThck > S(1.0e-06)) ? std::max(odeThck - minH, S(0)) : 0.0;

  ///////////////////////////////////////
  // Create frames and shapes
  ///////////////////////////////////////

  // frames and shapes
  auto terrainFrame = SimpleFrame::createShared(Frame::World());
  auto boxFrame = SimpleFrame::createShared(Frame::World());
  auto terrainShape = std::make_shared<HeightmapShape<S>>();
  auto boxShape = std::make_shared<BoxShape>(
      Eigen::Vector3d::Constant(static_cast<double>(boxSize)));

  // make a terrain with a linearly increasing slope
  std::vector<S> heights = {minH, halfHeight, halfHeight, maxH};
  terrainShape->setHeightField(2u, 2u, heights);
  // set a scale to test this at the same time
  const S terrSize = terrainScale;
  terrainShape->setScale(Vector3(terrainScale, terrainScale, zScale));
  EXPECT_EQ(terrainShape->getHeightField().size(), heights.size());

  terrainFrame->setShape(terrainShape);
  boxFrame->setShape(boxShape);

  ///////////////////////////////////////
  // Test collisions
  ///////////////////////////////////////

  auto group = cd->createCollisionGroup(terrainFrame.get(), boxFrame.get());
  EXPECT_EQ(group->getNumShapeFrames(), 2u);

  collision::CollisionOption option;
  option.enableContact = true;

  collision::CollisionResult result;
  // the terrain is going to remain in the origin. During the tests,
  // we are only moving the box.
  terrainFrame->setTranslation(Eigen::Vector3d::Zero());

  // there should be no collision underneath the height field, which should be
  // on the x/y plane.
  result.clear();
  // Some tolerance (useOdeThck) has to be added for ODE because it adds an
  // extra piece on the bottom to prevent objects from falling through
  // lowest points.
  S transZ = adjMinH * zScale - boxSize * S(0.501) - useOdeThck;
  boxFrame->setTranslation(Vector3(0.0, 0.0, transZ).template cast<double>());
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_EQ(result.getNumContacts(), 0u);

  // expect collision if moved just slightly above the lower terrain bound
  if (collidesUnderTerrain) {
    result.clear();
    transZ = adjMinH * zScale - boxSize * S(0.499) - useOdeThck;
    boxFrame->setTranslation(Vector3(0.0, 0.0, transZ).template cast<double>());
    EXPECT_TRUE(group->collide(option, &result));
    EXPECT_GT(result.getNumContacts(), 0u);
  }

  ///////////////////////////////////////
  // test collisions when box is at extreme corner
  // points (lowest and highest)
  ///////////////////////////////////////

  // some helper vectors
  Vector3 slope(1.0, -1.0, maxH - minH);
  slope.normalize();
  Vector3 crossSection(1.0, 1.0, heights[1] - heights[2]);
  crossSection.normalize();
  const Vector3 normal = slope.cross(crossSection);
  // the two extreme corners:
  const Vector3 highCorner
      = Vector3(terrSize / S(2), -terrSize / S(2), maxH * zScale);
  const Vector3 lowCorner
      = Vector3(-terrSize / S(2), terrSize / S(2), maxH * zScale);

  // ODE doesn't do nicely when boxes are close to the border of the terrain.
  // Shift the boxes along the slope (or normal to slope for some tests)
  // by this length.
  // Technically we should compute this a bit more accurately than this.
  // it basically has to ensure the box is inside or outside the terrain
  // bounds, so the slope plays a role for this factor.
  // But since the box is small, an estimate is used for now.
  const S boxShift = boxSize * S(1.5);

  // expect collision at highest point (at max height)
  Vector3 cornerShift = highCorner - slope * boxShift;
  result.clear();
  boxFrame->setTranslation(cornerShift.template cast<double>());
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_GT(result.getNumContacts(), 0u);

  // .. but not at opposite corner (lowest corner, at overall max height)
  result.clear();
  cornerShift = Vector3(lowCorner + slope * boxShift);
  boxFrame->setTranslation(cornerShift.template cast<double>());
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_EQ(result.getNumContacts(), 0u);

  ///////////////////////////////////////
  // test collisions for box on z axis
  ///////////////////////////////////////

  // box should collide where it intersects the slope
  result.clear();
  Vector3 inMiddle(0.0, 0.0, halfHeight * zScale);
  boxFrame->setTranslation(inMiddle.template cast<double>());
  // TODO(JS): Disabled temporarily
  if (cd->getType() != "bullet") {
    EXPECT_TRUE(group->collide(option, &result));
    EXPECT_GT(result.getNumContacts(), 0u);
  }

  // ... but not if the box is translated away from the slope
  result.clear();
  Vector3 onTopOfSlope = inMiddle + normal * boxShift;
  boxFrame->setTranslation(onTopOfSlope.template cast<double>());
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_EQ(result.getNumContacts(), 0u);

  // ... however it still should collide if translated the
  // other way inside the slope
  if (collidesUnderTerrain) {
    result.clear();
    Vector3 underSlope = inMiddle - normal * boxShift;
    boxFrame->setTranslation(underSlope.template cast<double>());
    EXPECT_TRUE(group->collide(option, &result));
    EXPECT_GT(result.getNumContacts(), 0u);
  }
}

//==============================================================================
TEST_F(Collision, testHeightmapBox)
{
#if HAVE_ODE
  auto ode = OdeCollisionDetector::create();
  // TODO take this message out as soon as testing is done
  dtdbg << "Testing ODE (float)" << std::endl;
  testHeightmapBox<float>(ode.get(), true, true, 0.05f);

  // TODO take this message out as soon as testing is done
  dtdbg << "Testing ODE (double)" << std::endl;
  testHeightmapBox<double>(ode.get(), true, true, 0.05);
#endif

#if HAVE_BULLET
  auto bullet = BulletCollisionDetector::create();

  // TODO take this message out as soon as testing is done
  dtdbg << "Testing Bullet (float)" << std::endl;
  // bullet so far only supports float height fields, so don't test double here.
  testHeightmapBox<float>(bullet.get(), false, false);
#endif
}

//==============================================================================
// A finite-but-astronomical heightmap scale must not abort ODE heightfield
// collision. A scale like 1e308 is finite (so it passes the shape-level
// validation guard), but (width-1)*scale, the vertical scale, and
// maxHeight*scale.z then overflow to +/-Inf inside ODE, which collapses the
// per-cell index range and trips dIASSERT((nMinX < nMaxX) && (nMinZ < nMaxZ))
// in dCollideHeightfield. DART now clamps the extents/scale/bounds handed to
// ODE to a finite value. See https://github.com/gazebosim/gz-physics/issues/847
TEST_F(Collision, testHeightmapHugeScaleNoAbort)
{
#if HAVE_ODE
  auto ode = OdeCollisionDetector::create();
  const double huge = 1e308; // finite but astronomical; passes the shape guard

  auto collideWithBox
      = [&](const std::shared_ptr<HeightmapShape<double>>& terrain) {
          auto terrainFrame = SimpleFrame::createShared(Frame::World());
          auto boxFrame = SimpleFrame::createShared(Frame::World());
          terrainFrame->setShape(terrain);
          boxFrame->setShape(
              std::make_shared<BoxShape>(Eigen::Vector3d::Constant(0.1)));
          boxFrame->setTranslation(Eigen::Vector3d(0.0, 0.0, 2.0));

          auto group
              = ode->createCollisionGroup(terrainFrame.get(), boxFrame.get());
          collision::CollisionOption option;
          option.enableContact = true;
          collision::CollisionResult result;
          // The bug aborted the process here; reaching the next line proves the
          // extents/scale/bounds were clamped to finite values.
          EXPECT_NO_FATAL_FAILURE(group->collide(option, &result));
          for (auto i = 0u; i < result.getNumContacts(); ++i)
            EXPECT_TRUE(result.getContact(i).point.allFinite());
        };

  // Extents-overflow path: a >= 3 sample dimension makes (width-1)*scale
  // overflow ((3-1)*1e308 == Inf).
  {
    auto terrain = std::make_shared<HeightmapShape<double>>();
    std::vector<double> heights = {1.0, 1.0, 1.0, 1.0, 2.0, 1.0, 1.0, 1.0, 1.0};
    terrain->setHeightField(3u, 3u, heights);
    terrain->setScale(Eigen::Vector3d(huge, huge, huge));
    // The finite-but-astronomical scale is accepted by the shape-level guard.
    EXPECT_TRUE(
        terrain->getScale().isApprox(Eigen::Vector3d(huge, huge, huge)));
    collideWithBox(terrain);
  }

  // Bounds/vertical-overflow path: a huge z-scale makes maxHeight*scale.z
  // overflow even for a 2x2 field.
  {
    auto terrain = std::make_shared<HeightmapShape<double>>();
    std::vector<double> heights = {1.0, 2.0, 2.0, 3.0};
    terrain->setHeightField(2u, 2u, heights);
    terrain->setScale(Eigen::Vector3d(2.0, 2.0, huge));
    collideWithBox(terrain);
  }
#endif
}

//==============================================================================
// Tests HeightmapShape::flipY();
TEST_F(Collision, testHeightmapFlipY)
{
  using S = double;

  std::vector<S> heights1 = {-1, -2, 2, 1};
  auto shape = std::make_shared<HeightmapShape<S>>();
  shape->setHeightField(2, 2, heights1);
  shape->flipY();
  EXPECT_EQ(shape->getHeightField().data()[0], heights1[2]);
  EXPECT_EQ(shape->getHeightField().data()[1], heights1[3]);
  EXPECT_EQ(shape->getHeightField().data()[2], heights1[0]);
  EXPECT_EQ(shape->getHeightField().data()[3], heights1[1]);

  // test with odd number of rows
  std::vector<S> heights2 = {-1, -2, 3, 3, 2, 1};
  shape->setHeightField(2, 3, heights2);
  shape->flipY();
  EXPECT_EQ(shape->getHeightField().data()[0], heights2[4]);
  EXPECT_EQ(shape->getHeightField().data()[1], heights2[5]);
  EXPECT_EQ(shape->getHeightField().data()[2], heights2[2]);
  EXPECT_EQ(shape->getHeightField().data()[3], heights2[3]);
  EXPECT_EQ(shape->getHeightField().data()[4], heights2[0]);
  EXPECT_EQ(shape->getHeightField().data()[5], heights2[1]);

  // test higher number of rows
  std::vector<S> heights3 = {1, -1, 2, -2, 3, -3, 4, -4};
  shape->setHeightField(2, 4, heights3);
  shape->flipY();
  EXPECT_EQ(shape->getHeightField().data()[0], heights3[6]);
  EXPECT_EQ(shape->getHeightField().data()[1], heights3[7]);
  EXPECT_EQ(shape->getHeightField().data()[2], heights3[4]);
  EXPECT_EQ(shape->getHeightField().data()[3], heights3[5]);
  EXPECT_EQ(shape->getHeightField().data()[4], heights3[2]);
  EXPECT_EQ(shape->getHeightField().data()[5], heights3[3]);
  EXPECT_EQ(shape->getHeightField().data()[6], heights3[0]);
  EXPECT_EQ(shape->getHeightField().data()[7], heights3[1]);

  // test wider rows
  std::vector<S> heights4 = {1, -1, 1.5, 2, -2, 2.5, 3, -3, 3.5, 4, -4, 4.5};
  shape->setHeightField(3, 4, heights4);
  shape->flipY();
  EXPECT_EQ(shape->getHeightField().data()[0], heights4[9]);
  EXPECT_EQ(shape->getHeightField().data()[1], heights4[10]);
  EXPECT_EQ(shape->getHeightField().data()[2], heights4[11]);
  EXPECT_EQ(shape->getHeightField().data()[3], heights4[6]);
  EXPECT_EQ(shape->getHeightField().data()[4], heights4[7]);
  EXPECT_EQ(shape->getHeightField().data()[5], heights4[8]);
  EXPECT_EQ(shape->getHeightField().data()[6], heights4[3]);
  EXPECT_EQ(shape->getHeightField().data()[7], heights4[4]);
  EXPECT_EQ(shape->getHeightField().data()[8], heights4[5]);
  EXPECT_EQ(shape->getHeightField().data()[9], heights4[0]);
  EXPECT_EQ(shape->getHeightField().data()[10], heights4[1]);
  EXPECT_EQ(shape->getHeightField().data()[11], heights4[2]);

  // test mini (actually meaningless) height field
  std::vector<S> heights5 = {1, 2};
  shape->setHeightField(1, 2, heights5);
  shape->flipY();
  EXPECT_EQ(shape->getHeightField().data()[0], heights5[1]);
  EXPECT_EQ(shape->getHeightField().data()[1], heights5[0]);

  // test height field with only one row (which is actually meaningless)
  std::vector<S> heights6 = {1, 2};
  shape->setHeightField(2, 1, heights6);
  shape->flipY();
  EXPECT_EQ(shape->getHeightField().data()[0], heights6[0]);
  EXPECT_EQ(shape->getHeightField().data()[1], heights6[1]);

  // test height field with only one column (which is actually meaningless)
  std::vector<S> heights7 = {1, 2};
  shape->setHeightField(1, 2, heights7);
  shape->flipY();
  EXPECT_EQ(shape->getHeightField().data()[0], heights7[1]);
  EXPECT_EQ(shape->getHeightField().data()[1], heights7[0]);

  // test height field with only one col and row (which is actually meaningless)
  std::vector<S> heights8 = {1};
  shape->setHeightField(1, 1, heights8);
  shape->flipY();
  EXPECT_EQ(shape->getHeightField().data()[0], heights8[0]);
}

//==============================================================================
TEST_F(Collision, Options)
{
  auto fcl_mesh_dart = FCLCollisionDetector::create();
  fcl_mesh_dart->setPrimitiveShapeType(FCLCollisionDetector::MESH);
  fcl_mesh_dart->setContactPointComputationMethod(FCLCollisionDetector::DART);
  testOptions(fcl_mesh_dart);

  // auto fcl_prim_fcl = FCLCollisionDetector::create();
  // fcl_prim_fcl->setPrimitiveShapeType(FCLCollisionDetector::MESH);
  // fcl_prim_fcl->setContactPointComputationMethod(FCLCollisionDetector::FCL);
  // testOptions(fcl_prim_fcl);

  // auto fcl_mesh_fcl = FCLCollisionDetector::create();
  // fcl_mesh_fcl->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  // fcl_mesh_fcl->setContactPointComputationMethod(FCLCollisionDetector::DART);
  // testOptions(fcl_mesh_fcl);

  // auto fcl_mesh_fcl = FCLCollisionDetector::create();
  // fcl_mesh_fcl->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  // fcl_mesh_fcl->setContactPointComputationMethod(FCLCollisionDetector::FCL);
  // testOptions(fcl_mesh_fcl);

#if HAVE_BULLET
  auto bullet = BulletCollisionDetector::create();
  testOptions(bullet);
#endif

  auto dart = DARTCollisionDetector::create();
  testOptions(dart);
}

//==============================================================================
void testFilter(const std::shared_ptr<CollisionDetector>& cd)
{
  // Create two bodies skeleton. The two bodies are placed at the same position
  // with the same size shape so that they collide by default.
  auto skel = Skeleton::create();
  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d(1, 1, 1));
  auto pair0 = skel->createJointAndBodyNodePair<RevoluteJoint>(nullptr);
  auto* body0 = pair0.second;
  body0->createShapeNodeWith<VisualAspect, CollisionAspect>(shape);
  auto pair1 = body0->createChildJointAndBodyNodePair<RevoluteJoint>();
  auto* body1 = pair1.second;
  body1->createShapeNodeWith<VisualAspect, CollisionAspect>(shape);

  // Create a world and add the created skeleton
  auto world = std::make_shared<simulation::World>();
  auto constraintSolver = world->getConstraintSolver();
  constraintSolver->setCollisionDetector(cd);
  world->addSkeleton(skel);

  // Get the collision group from the constraint solver
  auto group = constraintSolver->getCollisionGroup();
  EXPECT_EQ(group->getNumShapeFrames(), 2u);

  // Default collision filter for Skeleton
  auto& option = constraintSolver->getCollisionOption();
  auto bodyNodeFilter = std::make_shared<BodyNodeCollisionFilter>();
  option.collisionFilter = bodyNodeFilter;

  skel->enableSelfCollisionCheck();
  skel->enableAdjacentBodyCheck();
  EXPECT_TRUE(skel->isEnabledSelfCollisionCheck());
  EXPECT_TRUE(skel->isEnabledAdjacentBodyCheck());
  EXPECT_TRUE(group->collide()); // without filter, always collision
  EXPECT_TRUE(group->collide(option));

  skel->enableSelfCollisionCheck();
  skel->disableAdjacentBodyCheck();
  EXPECT_TRUE(skel->isEnabledSelfCollisionCheck());
  EXPECT_FALSE(skel->isEnabledAdjacentBodyCheck());
  EXPECT_TRUE(group->collide());
  EXPECT_FALSE(group->collide(option));

  skel->disableSelfCollisionCheck();
  skel->enableAdjacentBodyCheck();
  EXPECT_FALSE(skel->isEnabledSelfCollisionCheck());
  EXPECT_TRUE(skel->isEnabledAdjacentBodyCheck());
  EXPECT_TRUE(group->collide());
  EXPECT_FALSE(group->collide(option));

  skel->disableSelfCollisionCheck();
  skel->disableAdjacentBodyCheck();
  EXPECT_FALSE(skel->isEnabledSelfCollisionCheck());
  EXPECT_FALSE(skel->isEnabledAdjacentBodyCheck());
  EXPECT_TRUE(group->collide());
  EXPECT_FALSE(group->collide(option));

  // Test blacklist
  skel->enableSelfCollisionCheck();
  skel->enableAdjacentBodyCheck();
  bodyNodeFilter->addBodyNodePairToBlackList(body0, body1);
  EXPECT_FALSE(group->collide(option));
  bodyNodeFilter->removeBodyNodePairFromBlackList(body0, body1);
  EXPECT_TRUE(group->collide(option));
  bodyNodeFilter->addBodyNodePairToBlackList(body0, body1);
  EXPECT_FALSE(group->collide(option));
  bodyNodeFilter->removeAllBodyNodePairsFromBlackList();
  EXPECT_TRUE(group->collide(option));
}

//==============================================================================
TEST_F(Collision, Filter)
{
  auto fcl_mesh_dart = FCLCollisionDetector::create();
  fcl_mesh_dart->setPrimitiveShapeType(FCLCollisionDetector::MESH);
  fcl_mesh_dart->setContactPointComputationMethod(FCLCollisionDetector::DART);
  testFilter(fcl_mesh_dart);

  // auto fcl_prim_fcl = FCLCollisionDetector::create();
  // fcl_prim_fcl->setPrimitiveShapeType(FCLCollisionDetector::MESH);
  // fcl_prim_fcl->setContactPointComputationMethod(FCLCollisionDetector::FCL);
  // testFilter(fcl_prim_fcl);

  // auto fcl_mesh_fcl = FCLCollisionDetector::create();
  // fcl_mesh_fcl->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  // fcl_mesh_fcl->setContactPointComputationMethod(FCLCollisionDetector::DART);
  // testFilter(fcl_mesh_fcl);

  // auto fcl_mesh_fcl = FCLCollisionDetector::create();
  // fcl_mesh_fcl->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  // fcl_mesh_fcl->setContactPointComputationMethod(FCLCollisionDetector::FCL);
  // testFilter(fcl_mesh_fcl);

#if HAVE_BULLET
  auto bullet = BulletCollisionDetector::create();
  testFilter(bullet);
#endif

  auto dart = DARTCollisionDetector::create();
  testFilter(dart);
}

#if HAVE_BULLET
//==============================================================================
TEST_F(Collision, BulletRefiltersBodyNodeCollisionFilterSubclass)
{
  auto detector = BulletCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  const Eigen::Vector3d size(1.0, 1.0, 1.0);
  auto skeleton1 = createBox(size, Eigen::Vector3d::Zero());
  auto skeleton2 = createBox(size, Eigen::Vector3d(0.25, 0.0, 0.0));
  auto* body1 = skeleton1->getBodyNode(0u);
  auto* body2 = skeleton2->getBodyNode(0u);
  group->addShapeFramesOf(body1);
  group->addShapeFramesOf(body2);

  CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 4u;
  auto filter
      = std::make_shared<ToggleBodyNodeCollisionFilter>(body1, body2, false);
  option.collisionFilter = filter;

  CollisionResult result;
  ASSERT_TRUE(group->collide(option, &result));
  ASSERT_GT(result.getNumContacts(), 0u);

  filter->setIgnorePair(true);
  result.clear();
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_EQ(0u, result.getNumContacts());
}

//==============================================================================
TEST_F(Collision, BulletRefiltersBodyNodeCollisionFilterAfterUntrackedQuery)
{
  auto detector = BulletCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  const Eigen::Vector3d size(1.0, 1.0, 1.0);
  auto skeleton1 = createBox(size, Eigen::Vector3d::Zero());
  auto skeleton2 = createBox(size, Eigen::Vector3d(0.25, 0.0, 0.0));
  auto* body1 = skeleton1->getBodyNode(0u);
  auto* body2 = skeleton2->getBodyNode(0u);
  group->addShapeFramesOf(body1);
  group->addShapeFramesOf(body2);

  CollisionOption bodyNodeFilterOption;
  bodyNodeFilterOption.enableContact = true;
  bodyNodeFilterOption.maxNumContacts = 4u;
  auto bodyNodeFilter = std::make_shared<BodyNodeCollisionFilter>();
  bodyNodeFilter->addBodyNodePairToBlackList(body1, body2);
  bodyNodeFilterOption.collisionFilter = bodyNodeFilter;

  CollisionResult result;
  ASSERT_FALSE(group->collide(bodyNodeFilterOption, &result));
  ASSERT_EQ(0u, result.getNumContacts());

  CollisionOption unfilteredOption;
  unfilteredOption.enableContact = true;
  unfilteredOption.maxNumContacts = 4u;
  result.clear();
  ASSERT_TRUE(group->collide(unfilteredOption, &result));
  ASSERT_GT(result.getNumContacts(), 0u);

  result.clear();
  EXPECT_FALSE(group->collide(bodyNodeFilterOption, &result));
  EXPECT_EQ(0u, result.getNumContacts());

  auto customFilter
      = std::make_shared<ToggleBodyNodeCollisionFilter>(body1, body2, false);
  CollisionOption customFilterOption;
  customFilterOption.enableContact = true;
  customFilterOption.maxNumContacts = 4u;
  customFilterOption.collisionFilter = customFilter;

  result.clear();
  ASSERT_TRUE(group->collide(customFilterOption, &result));
  ASSERT_GT(result.getNumContacts(), 0u);

  result.clear();
  EXPECT_FALSE(group->collide(bodyNodeFilterOption, &result));
  EXPECT_EQ(0u, result.getNumContacts());
}

//==============================================================================
TEST_F(Collision, BulletBinaryCollideFiltersProximityContacts)
{
  auto detector = BulletCollisionDetector::create();

  auto frame1 = SimpleFrame::createShared(Frame::World());
  auto frame2 = SimpleFrame::createShared(Frame::World());
  auto shape1 = std::make_shared<CylinderShape>(0.5, 1.0);
  auto shape2 = std::make_shared<CylinderShape>(0.5, 1.0);
  frame1->setShape(shape1);
  frame2->setShape(shape2);
  frame2->setTranslation(Eigen::Vector3d(1.01, 0.0, 0.0));

  auto group = detector->createCollisionGroup(frame1.get(), frame2.get());
  auto group1 = detector->createCollisionGroup(frame1.get());
  auto group2 = detector->createCollisionGroup(frame2.get());

  CollisionOption option;
  CollisionResult result;
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_EQ(0u, result.getNumContacts());
  EXPECT_FALSE(group->collide(option));
  EXPECT_FALSE(group->collide(option, nullptr));
  EXPECT_FALSE(group1->collide(group2.get(), option));
  EXPECT_FALSE(group1->collide(group2.get(), option, nullptr));

  option.allowNegativePenetrationDepthContacts = true;
  result.clear();
  ASSERT_TRUE(group->collide(option, &result));
  ASSERT_GT(result.getNumContacts(), 0u);
  EXPECT_TRUE(group->collide(option));
  EXPECT_TRUE(group->collide(option, nullptr));
  EXPECT_TRUE(group1->collide(group2.get(), option));
  EXPECT_TRUE(group1->collide(group2.get(), option, nullptr));
}
#endif

//==============================================================================
void testCreateCollisionGroups(const std::shared_ptr<CollisionDetector>& cd)
{
  Eigen::Vector3d size(1.0, 1.0, 1.0);
  Eigen::Vector3d pos1(0.0, 0.0, 0.0);
  Eigen::Vector3d pos2(0.5, 0.0, 0.0);

  auto boxSkeleton1 = createBox(size, pos1);
  auto boxSkeleton2 = createBox(size, pos2);

  auto boxBodyNode1 = boxSkeleton1->getBodyNode(0u);
  auto boxBodyNode2 = boxSkeleton2->getBodyNode(0u);

  auto boxShapeNode1 = boxBodyNode1->getShapeNodeWith<CollisionAspect>(0);
  auto boxShapeNode2 = boxBodyNode2->getShapeNodeWith<CollisionAspect>(0);

  collision::CollisionOption option;
  collision::CollisionResult result;

  auto skeletonGroup1 = cd->createCollisionGroup(boxSkeleton1.get());
  auto skeletonGroup2 = cd->createCollisionGroup(boxSkeleton2.get());

  auto bodyNodeGroup1 = cd->createCollisionGroup(boxBodyNode1);
  auto bodyNodeGroup2 = cd->createCollisionGroup(boxBodyNode2);

  auto shapeNodeGroup1 = cd->createCollisionGroup(boxShapeNode1);
  auto shapeNodeGroup2 = cd->createCollisionGroup(boxShapeNode2);

  EXPECT_TRUE(skeletonGroup1->collide(skeletonGroup2.get(), option, &result));
  EXPECT_TRUE(bodyNodeGroup1->collide(bodyNodeGroup2.get(), option, &result));
  EXPECT_TRUE(shapeNodeGroup1->collide(shapeNodeGroup2.get(), option, &result));

  // Binary check without passing option
  auto oldMaxNumContacts = option.maxNumContacts;
  option.maxNumContacts = 1u;
  EXPECT_TRUE(skeletonGroup1->collide(skeletonGroup2.get(), option));
  EXPECT_TRUE(bodyNodeGroup1->collide(bodyNodeGroup2.get(), option));
  EXPECT_TRUE(shapeNodeGroup1->collide(shapeNodeGroup2.get(), option));
  option.maxNumContacts = oldMaxNumContacts;

  // Binary check without passing option and result
  EXPECT_TRUE(skeletonGroup1->collide(skeletonGroup2.get()));
  EXPECT_TRUE(bodyNodeGroup1->collide(bodyNodeGroup2.get()));
  EXPECT_TRUE(shapeNodeGroup1->collide(shapeNodeGroup2.get()));

  // Regression test for #666
  auto world = std::make_unique<World>();
  world->getConstraintSolver()->setCollisionDetector(cd);
  world->addSkeleton(boxSkeleton1);
  world->addSkeleton(boxSkeleton2);
  DART_SUPPRESS_DEPRECATED_BEGIN
  EXPECT_FALSE(boxBodyNode1->isColliding());
  EXPECT_FALSE(boxBodyNode2->isColliding());
  DART_SUPPRESS_DEPRECATED_END

  const collision::CollisionResult& result1 = world->getLastCollisionResult();
  EXPECT_FALSE(result1.inCollision(boxBodyNode1));
  EXPECT_FALSE(result1.inCollision(boxBodyNode2));

  world->step();
  DART_SUPPRESS_DEPRECATED_BEGIN
  EXPECT_TRUE(boxBodyNode1->isColliding());
  EXPECT_TRUE(boxBodyNode2->isColliding());
  DART_SUPPRESS_DEPRECATED_END

  const collision::CollisionResult& result2 = world->getLastCollisionResult();
  EXPECT_TRUE(result2.inCollision(boxBodyNode1));
  EXPECT_TRUE(result2.inCollision(boxBodyNode2));
}

//==============================================================================
TEST_F(Collision, ContactBodyNodeAccessors)
{
  // For collision objects backed by ShapeNodes, Contact::getShapeNode*/
  // getBodyNodePtr* resolve to the owning shape node and body node. (The
  // SimpleFrame case, where these return null, is covered by the ShapeFrame
  // ordering tests above.)
  auto detector = FCLCollisionDetector::create();

  auto skelA = Skeleton::create("skelA");
  auto skelB = Skeleton::create("skelB");
  auto pairA = skelA->createJointAndBodyNodePair<FreeJoint>();
  auto pairB = skelB->createJointAndBodyNodePair<FreeJoint>();
  BodyNode* bnA = pairA.second;
  BodyNode* bnB = pairB.second;

  auto box = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(1.0));
  auto* snA = bnA->createShapeNodeWith<CollisionAspect>(box);
  auto* snB = bnB->createShapeNodeWith<CollisionAspect>(box);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  pairA.first->setTransform(tf);
  tf.translation()[0] = 0.5; // overlap the unit boxes
  pairB.first->setTransform(tf);

  CollisionOption option;
  option.enableContact = true;
  CollisionResult result;
  auto groupA = detector->createCollisionGroup(snA);
  auto groupB = detector->createCollisionGroup(snB);
  ASSERT_TRUE(groupA->collide(groupB.get(), option, &result));
  ASSERT_GE(result.getNumContacts(), 1u);

  const auto& contact = result.getContact(0);

  ASSERT_NE(contact.getShapeNode1(), nullptr);
  ASSERT_NE(contact.getShapeNode2(), nullptr);
  EXPECT_EQ(contact.getShapeFrame1(), contact.getShapeNode1());
  EXPECT_EQ(contact.getShapeFrame2(), contact.getShapeNode2());

  const ConstBodyNodePtr bn1 = contact.getBodyNodePtr1();
  const ConstBodyNodePtr bn2 = contact.getBodyNodePtr2();
  ASSERT_TRUE(bn1);
  ASSERT_TRUE(bn2);

  // The two collision objects are snA and snB (in some canonical order).
  EXPECT_TRUE(
      (contact.getShapeNode1() == snA && contact.getShapeNode2() == snB)
      || (contact.getShapeNode1() == snB && contact.getShapeNode2() == snA));
  EXPECT_TRUE(
      (bn1.get() == bnA && bn2.get() == bnB)
      || (bn1.get() == bnB && bn2.get() == bnA));
}

//==============================================================================
TEST_F(Collision, CollisionResultCachesCollidingFramesEagerly)
{
  auto detector = FCLCollisionDetector::create();

  auto skelA = Skeleton::create("result_cache_a");
  auto skelB = Skeleton::create("result_cache_b");
  auto skelC = Skeleton::create("result_cache_c");
  auto skelD = Skeleton::create("result_cache_d");

  auto pairA = skelA->createJointAndBodyNodePair<FreeJoint>();
  auto pairB = skelB->createJointAndBodyNodePair<FreeJoint>();
  auto pairC = skelC->createJointAndBodyNodePair<FreeJoint>();
  auto pairD = skelD->createJointAndBodyNodePair<FreeJoint>();

  auto box = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(1.0));
  auto* nodeA = pairA.second->createShapeNodeWith<CollisionAspect>(box);
  auto* nodeB = pairB.second->createShapeNodeWith<CollisionAspect>(box);
  auto* nodeC = pairC.second->createShapeNodeWith<CollisionAspect>(box);
  auto* nodeD = pairD.second->createShapeNodeWith<CollisionAspect>(box);

  CollisionResult result;
  TestCollisionObjectStorage storageA;
  TestCollisionObjectStorage storageB;

  auto* objectA
      = new (storageA.data) TestCollisionObject(detector.get(), nodeA);
  auto* objectB
      = new (storageB.data) TestCollisionObject(detector.get(), nodeB);

  Contact contact;
  contact.collisionObject1 = objectA;
  contact.collisionObject2 = objectB;
  result.addContact(contact);

  objectA->~TestCollisionObject();
  objectB->~TestCollisionObject();

  auto* replacementA
      = new (storageA.data) TestCollisionObject(detector.get(), nodeC);
  auto* replacementB
      = new (storageB.data) TestCollisionObject(detector.get(), nodeD);

  EXPECT_TRUE(result.inCollision(nodeA));
  EXPECT_TRUE(result.inCollision(nodeB));
  EXPECT_TRUE(result.inCollision(pairA.second));
  EXPECT_TRUE(result.inCollision(pairB.second));

  EXPECT_FALSE(result.inCollision(nodeC));
  EXPECT_FALSE(result.inCollision(nodeD));
  EXPECT_FALSE(result.inCollision(pairC.second));
  EXPECT_FALSE(result.inCollision(pairD.second));

  replacementA->~TestCollisionObject();
  replacementB->~TestCollisionObject();
}

//==============================================================================
TEST_F(Collision, CollisionResultKeepsMaterializedCollidingSetsCurrent)
{
  auto detector = FCLCollisionDetector::create();

  auto skelA = Skeleton::create("result_set_a");
  auto skelB = Skeleton::create("result_set_b");
  auto skelC = Skeleton::create("result_set_c");
  auto skelD = Skeleton::create("result_set_d");

  auto pairA = skelA->createJointAndBodyNodePair<FreeJoint>();
  auto pairB = skelB->createJointAndBodyNodePair<FreeJoint>();
  auto pairC = skelC->createJointAndBodyNodePair<FreeJoint>();
  auto pairD = skelD->createJointAndBodyNodePair<FreeJoint>();

  auto box = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(1.0));
  auto* nodeA = pairA.second->createShapeNodeWith<CollisionAspect>(box);
  auto* nodeB = pairB.second->createShapeNodeWith<CollisionAspect>(box);
  auto* nodeC = pairC.second->createShapeNodeWith<CollisionAspect>(box);
  auto* nodeD = pairD.second->createShapeNodeWith<CollisionAspect>(box);

  TestCollisionObject objectA(detector.get(), nodeA);
  TestCollisionObject objectB(detector.get(), nodeB);
  TestCollisionObject objectC(detector.get(), nodeC);
  TestCollisionObject objectD(detector.get(), nodeD);

  CollisionResult result;
  const auto& frames = result.getCollidingShapeFrames();
  const auto& bodies = result.getCollidingBodyNodes();
  EXPECT_TRUE(frames.empty());
  EXPECT_TRUE(bodies.empty());

  Contact contactAB;
  contactAB.collisionObject1 = &objectA;
  contactAB.collisionObject2 = &objectB;
  result.addContact(contactAB);

  EXPECT_TRUE(frames.count(nodeA));
  EXPECT_TRUE(frames.count(nodeB));
  EXPECT_TRUE(bodies.count(pairA.second));
  EXPECT_TRUE(bodies.count(pairB.second));

  result.clear();
  EXPECT_TRUE(frames.empty());
  EXPECT_TRUE(bodies.empty());

  Contact contactCD;
  contactCD.collisionObject1 = &objectC;
  contactCD.collisionObject2 = &objectD;
  result.addContact(contactCD);

  EXPECT_FALSE(frames.count(nodeA));
  EXPECT_FALSE(frames.count(nodeB));
  EXPECT_TRUE(frames.count(nodeC));
  EXPECT_TRUE(frames.count(nodeD));
  EXPECT_FALSE(bodies.count(pairA.second));
  EXPECT_FALSE(bodies.count(pairB.second));
  EXPECT_TRUE(bodies.count(pairC.second));
  EXPECT_TRUE(bodies.count(pairD.second));
}

//==============================================================================
TEST_F(Collision, CollisionResultCanSkipCollidingObjectCachesForScratchUse)
{
  class ScratchCollisionResult final : public CollisionResult
  {
  public:
    ScratchCollisionResult()
    {
      setCollidingObjectCacheEnabled(false);
    }

    void enableCollidingObjectCache()
    {
      setCollidingObjectCacheEnabled(true);
    }
  };

  auto detector = FCLCollisionDetector::create();

  auto skelA = Skeleton::create("scratch_result_a");
  auto skelB = Skeleton::create("scratch_result_b");
  auto pairA = skelA->createJointAndBodyNodePair<FreeJoint>();
  auto pairB = skelB->createJointAndBodyNodePair<FreeJoint>();

  auto box = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(1.0));
  auto* nodeA = pairA.second->createShapeNodeWith<CollisionAspect>(box);
  auto* nodeB = pairB.second->createShapeNodeWith<CollisionAspect>(box);

  TestCollisionObject objectA(detector.get(), nodeA);
  TestCollisionObject objectB(detector.get(), nodeB);

  Contact contact;
  contact.collisionObject1 = &objectA;
  contact.collisionObject2 = &objectB;

  ScratchCollisionResult result;
  result.addContact(contact);

  ASSERT_EQ(1u, result.getNumContacts());
  EXPECT_FALSE(result.inCollision(nodeA));
  EXPECT_FALSE(result.inCollision(nodeB));
  EXPECT_FALSE(result.inCollision(pairA.second));
  EXPECT_FALSE(result.inCollision(pairB.second));

  result.clear();
  result.enableCollidingObjectCache();
  result.addContact(contact);

  EXPECT_TRUE(result.inCollision(nodeA));
  EXPECT_TRUE(result.inCollision(nodeB));
  EXPECT_TRUE(result.inCollision(pairA.second));
  EXPECT_TRUE(result.inCollision(pairB.second));
}

//==============================================================================
TEST_F(Collision, CreateCollisionGroupFromVariousObject)
{
  auto fcl_mesh_dart = FCLCollisionDetector::create();
  fcl_mesh_dart->setPrimitiveShapeType(FCLCollisionDetector::MESH);
  fcl_mesh_dart->setContactPointComputationMethod(FCLCollisionDetector::DART);
  testCreateCollisionGroups(fcl_mesh_dart);

  // auto fcl_prim_fcl = FCLCollisionDetector::create();
  // fcl_prim_fcl->setPrimitiveShapeType(FCLCollisionDetector::MESH);
  // fcl_prim_fcl->setContactPointComputationMethod(FCLCollisionDetector::FCL);
  // testCreateCollisionGroups(fcl_prim_fcl);

  // auto fcl_mesh_fcl = FCLCollisionDetector::create();
  // fcl_mesh_fcl->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  // fcl_mesh_fcl->setContactPointComputationMethod(FCLCollisionDetector::DART);
  // testCreateCollisionGroups(fcl_mesh_fcl);

  // auto fcl_mesh_fcl = FCLCollisionDetector::create();
  // fcl_mesh_fcl->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  // fcl_mesh_fcl->setContactPointComputationMethod(FCLCollisionDetector::FCL);
  // testCreateCollisionGroups(fcl_mesh_fcl);

#if HAVE_BULLET
  auto bullet = BulletCollisionDetector::create();
  testCreateCollisionGroups(bullet);
#endif

  auto dart = DARTCollisionDetector::create();
  testCreateCollisionGroups(dart);
}

//==============================================================================
TEST_F(Collision, CollisionOfPrescribedJoints)
{
  // There are one red plate (static skeleton) and 5 pendulums with different
  // actuator types. This test check if the motion prescribed joints are exactly
  // tracking the prescribed motion eventhough there are collision with other
  // objects.

  const double tol = 1e-9;
  const double timeStep = 1e-3;
  const std::size_t numFrames = 5e+0; // 5 secs

  // Load world and skeleton
  WorldPtr world = SkelParser::readWorld(
      "dart://sample/skel/test/collision_of_prescribed_joints_test.skel");
  world->setTimeStep(timeStep);
  EXPECT_TRUE(world != nullptr);
  EXPECT_NEAR(world->getTimeStep(), timeStep, tol);

  SkeletonPtr skel1 = world->getSkeleton("skeleton 1");
  SkeletonPtr skel2 = world->getSkeleton("skeleton 2");
  SkeletonPtr skel3 = world->getSkeleton("skeleton 3");
  SkeletonPtr skel4 = world->getSkeleton("skeleton 4");
  SkeletonPtr skel5 = world->getSkeleton("skeleton 5");
  SkeletonPtr skel6 = world->getSkeleton("skeleton 6");
  EXPECT_TRUE(skel1 != nullptr);
  EXPECT_TRUE(skel2 != nullptr);
  EXPECT_TRUE(skel3 != nullptr);
  EXPECT_TRUE(skel4 != nullptr);
  EXPECT_TRUE(skel5 != nullptr);
  EXPECT_TRUE(skel6 != nullptr);

  Joint* joint1 = skel1->getJoint(0);
  Joint* joint2 = skel2->getJoint(0);
  Joint* joint3 = skel3->getJoint(0);
  Joint* joint4 = skel4->getJoint(0);
  Joint* joint5 = skel5->getJoint(0);
  Joint* joint6 = skel6->getJoint(0);
  EXPECT_TRUE(joint1 != nullptr);
  EXPECT_TRUE(joint2 != nullptr);
  EXPECT_TRUE(joint3 != nullptr);
  EXPECT_TRUE(joint4 != nullptr);
  EXPECT_TRUE(joint5 != nullptr);
  EXPECT_TRUE(joint6 != nullptr);
  EXPECT_EQ(joint1->getActuatorType(), Joint::FORCE);
  EXPECT_EQ(joint2->getActuatorType(), Joint::PASSIVE);
  EXPECT_EQ(joint3->getActuatorType(), Joint::SERVO);
  EXPECT_EQ(joint4->getActuatorType(), Joint::ACCELERATION);
  EXPECT_EQ(joint5->getActuatorType(), Joint::VELOCITY);
  EXPECT_EQ(joint6->getActuatorType(), Joint::LOCKED);

  for (std::size_t i = 0; i < numFrames; ++i) {
    const double time = world->getTime();

    joint1->setCommand(0, -0.5 * constantsd::pi() * std::cos(time));
    joint2->setCommand(0, -0.5 * constantsd::pi() * std::cos(time));
    joint3->setCommand(0, -0.5 * constantsd::pi() * std::cos(time));
    joint4->setCommand(0, -0.5 * constantsd::pi() * std::cos(time));
    joint5->setCommand(0, -0.5 * constantsd::pi() * std::sin(time));
    joint6->setCommand(0, -0.5 * constantsd::pi() * std::sin(time)); // ignored

    world->step(false);

    EXPECT_TRUE(joint1->isDynamic());
    EXPECT_TRUE(joint2->isDynamic());
    EXPECT_TRUE(joint3->isDynamic());

    // Check if the motion prescribed joints are following the prescribed motion
    // eventhough there is a collision with other objects
    EXPECT_TRUE(joint4->isKinematic());
    EXPECT_NEAR(joint4->getAcceleration(0), joint4->getCommand(0), tol);
    EXPECT_TRUE(joint5->isKinematic());
    EXPECT_NEAR(joint5->getVelocity(0), joint5->getCommand(0), tol);

    // The PASSIVE joint's command should have been cleared (Issue 1899).
    EXPECT_EQ(joint2->getCommand(0), 0.0);

    // The velocity and acceleration of locked joint always must be zero.
    EXPECT_TRUE(joint6->isKinematic());
    EXPECT_NEAR(joint6->getVelocity(0), 0.0, tol);
    EXPECT_NEAR(joint6->getAcceleration(0), 0.0, tol);
  }
}

//==============================================================================
TEST_F(Collision, CollisionOfPrescribedJointsRejectsInvalidTimeStep)
{
  WorldPtr world = SkelParser::readWorld(
      "dart://sample/skel/test/collision_of_prescribed_joints_test.skel");
  ASSERT_TRUE(world != nullptr);

  const double originalTimeStep = world->getTimeStep();
  ASSERT_GT(originalTimeStep, 0.0);

  const double invalidValues[]
      = {std::numeric_limits<double>::quiet_NaN(),
         std::numeric_limits<double>::infinity(),
         -std::numeric_limits<double>::infinity(),
         0.0,
         -1.0};

  for (double invalid : invalidValues) {
    world->setTimeStep(invalid);
    EXPECT_DOUBLE_EQ(world->getTimeStep(), originalTimeStep);
  }

  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    EXPECT_DOUBLE_EQ(world->getSkeleton(i)->getTimeStep(), originalTimeStep);
  }

  EXPECT_DOUBLE_EQ(
      world->getConstraintSolver()->getTimeStep(), originalTimeStep);
  EXPECT_NO_THROW(world->step(false));
}

//==============================================================================
TEST_F(Collision, Factory)
{
  EXPECT_TRUE(collision::CollisionDetector::getFactory()->canCreate("fcl"));
  EXPECT_TRUE(collision::CollisionDetector::getFactory()->canCreate("dart"));

#if HAVE_BULLET
  EXPECT_TRUE(collision::CollisionDetector::getFactory()->canCreate("bullet"));
#else
  EXPECT_TRUE(!collision::CollisionDetector::getFactory()->canCreate("bullet"));
#endif

#if HAVE_ODE
  EXPECT_TRUE(collision::CollisionDetector::getFactory()->canCreate("ode"));
#else
  EXPECT_TRUE(!collision::CollisionDetector::getFactory()->canCreate("ode"));
#endif
}

#if HAVE_ODE
//==============================================================================
TEST(Issue1654, OdeContactHistoryClearsOnObjectRemoval)
{
  auto detector = OdeCollisionDetector::create();
  auto group = detector->createCollisionGroup();
  auto otherGroup = detector->createCollisionGroup();

  CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 4u;

  CollisionResult result;

  // Initial collision using SimpleFrames to populate the cache.
  auto simpleFrame1 = std::make_shared<SimpleFrame>(Frame::World(), "f1");
  auto simpleFrame2 = std::make_shared<SimpleFrame>(Frame::World(), "f2");
  auto sphere = std::make_shared<SphereShape>(1.0);
  simpleFrame1->setShape(sphere);
  simpleFrame2->setShape(sphere);
  simpleFrame2->setTranslation(Eigen::Vector3d::UnitX() * 0.5);

  group->addShapeFrame(simpleFrame1.get());
  group->addShapeFrame(simpleFrame2.get());

  ASSERT_TRUE(group->collide(option, &result));
  ASSERT_FALSE(result.getContacts().empty());

  group->removeAllShapeFrames();
  otherGroup->removeAllShapeFrames();
  result.clear();

  // Recreate collision objects using BodyNodes (mirroring the Python repro).
  auto skeleton1 = Skeleton::create("reuse1");
  auto skeleton2 = Skeleton::create("reuse2");

  auto pair1 = skeleton1->createJointAndBodyNodePair<FreeJoint>();
  auto pair2 = skeleton2->createJointAndBodyNodePair<FreeJoint>();
  auto* joint1 = pair1.first;
  auto* body1 = pair1.second;
  auto* joint2 = pair2.first;
  auto* body2 = pair2.second;

  auto bodySphere = std::make_shared<SphereShape>(1.0);
  body1->createShapeNodeWith<CollisionAspect>(bodySphere);
  body2->createShapeNodeWith<CollisionAspect>(bodySphere);

  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.translate(Eigen::Vector3d::UnitX() * 0.4);
  joint1->setRelativeTransform(pose1);
  joint2->setRelativeTransform(pose2);

  group->addShapeFramesOf(body1);
  group->addShapeFramesOf(body2);

  // Collide within a single group using the new objects.
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_GE(result.getNumContacts(), 1u);

  // Cross-group collision should also remain stable.
  result.clear();
  group->removeAllShapeFrames();
  otherGroup->removeAllShapeFrames();

  group->addShapeFramesOf(body1);
  otherGroup->addShapeFramesOf(body2);

  EXPECT_TRUE(group->collide(otherGroup.get(), option, &result));
  EXPECT_GE(result.getNumContacts(), 1u);

  // Move bodies apart to ensure history drops them cleanly.
  pose2.setIdentity();
  pose2.translate(Eigen::Vector3d::UnitX() * 3.0);
  joint2->setRelativeTransform(pose2);
  result.clear();
  EXPECT_FALSE(group->collide(otherGroup.get(), option, &result));
  EXPECT_TRUE(result.getContacts().empty());

  // Move back into contact and verify we still collide without crashing.
  pose2.setIdentity();
  pose2.translate(Eigen::Vector3d::UnitX() * 0.25);
  joint2->setRelativeTransform(pose2);
  result.clear();
  EXPECT_TRUE(group->collide(otherGroup.get(), option, &result));
  EXPECT_GE(result.getNumContacts(), 1u);
}

//==============================================================================
TEST(Issue1654, OdeContactHistoryClearsAfterKinematicPoseJump)
{
  auto detector = OdeCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 4u;

  auto skeleton1 = Skeleton::create("pose_jump1");
  auto skeleton2 = Skeleton::create("pose_jump2");

  auto pair1 = skeleton1->createJointAndBodyNodePair<FreeJoint>();
  auto pair2 = skeleton2->createJointAndBodyNodePair<FreeJoint>();
  auto* joint1 = pair1.first;
  auto* body1 = pair1.second;
  auto* joint2 = pair2.first;
  auto* body2 = pair2.second;

  auto sphere = std::make_shared<SphereShape>(1.0);
  body1->createShapeNodeWith<CollisionAspect>(sphere);
  body2->createShapeNodeWith<CollisionAspect>(sphere);

  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.translate(Eigen::Vector3d::UnitX() * 0.4);
  joint1->setRelativeTransform(pose1);
  joint2->setRelativeTransform(pose2);

  group->addShapeFramesOf(body1);
  group->addShapeFramesOf(body2);

  CollisionResult result;
  ASSERT_TRUE(group->collide(option, &result));
  ASSERT_EQ(1u, result.getNumContacts());

  pose1.translation() += Eigen::Vector3d::UnitY();
  pose2.translation() += Eigen::Vector3d::UnitY();
  joint1->setRelativeTransform(pose1);
  joint2->setRelativeTransform(pose2);

  result.clear();
  ASSERT_TRUE(group->collide(option, &result));
  EXPECT_EQ(1u, result.getNumContacts());
}

//==============================================================================
TEST(Issue1654, OdeContactHistoryClearsAfterIncrementalKinematicPoseChanges)
{
  auto detector = OdeCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 4u;

  auto skeleton1 = Skeleton::create("incremental_pose1");
  auto skeleton2 = Skeleton::create("incremental_pose2");

  auto pair1 = skeleton1->createJointAndBodyNodePair<FreeJoint>();
  auto pair2 = skeleton2->createJointAndBodyNodePair<FreeJoint>();
  auto* joint1 = pair1.first;
  auto* body1 = pair1.second;
  auto* joint2 = pair2.first;
  auto* body2 = pair2.second;

  auto sphere = std::make_shared<SphereShape>(1.0);
  body1->createShapeNodeWith<CollisionAspect>(sphere);
  body2->createShapeNodeWith<CollisionAspect>(sphere);

  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.translate(Eigen::Vector3d::UnitX() * 0.4);
  joint1->setRelativeTransform(pose1);
  joint2->setRelativeTransform(pose2);

  group->addShapeFramesOf(body1);
  group->addShapeFramesOf(body2);

  CollisionResult result;
  ASSERT_TRUE(group->collide(option, &result));
  ASSERT_EQ(1u, result.getNumContacts());

  for (auto step = 0u; step < 3u; ++step) {
    pose1.translation() += Eigen::Vector3d::UnitY() * 0.04;
    pose2.translation() += Eigen::Vector3d::UnitY() * 0.04;
    joint1->setRelativeTransform(pose1);
    joint2->setRelativeTransform(pose2);

    result.clear();
    ASSERT_TRUE(group->collide(option, &result));
    EXPECT_EQ(1u, result.getNumContacts());
  }
}

//==============================================================================
TEST(Issue1654, OdeContactHistorySkipsDuplicateCurrentContacts)
{
  auto detector = OdeCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 4u;

  auto skeleton1 = Skeleton::create("duplicate_contact1");
  auto skeleton2 = Skeleton::create("duplicate_contact2");

  auto pair1 = skeleton1->createJointAndBodyNodePair<FreeJoint>();
  auto pair2 = skeleton2->createJointAndBodyNodePair<FreeJoint>();
  auto* joint1 = pair1.first;
  auto* body1 = pair1.second;
  auto* joint2 = pair2.first;
  auto* body2 = pair2.second;

  body1->createShapeNodeWith<CollisionAspect>(
      std::make_shared<CapsuleShape>(1.0, 1.0));
  body2->createShapeNodeWith<CollisionAspect>(
      std::make_shared<CapsuleShape>(0.5, 1.0));

  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.translate(Eigen::Vector3d(0.74, 0.0, 0.0));
  joint1->setRelativeTransform(pose1);
  joint2->setRelativeTransform(pose2);

  group->addShapeFramesOf(body1);
  group->addShapeFramesOf(body2);

  CollisionResult result;
  ASSERT_TRUE(group->collide(option, &result));
  ASSERT_EQ(2u, result.getNumContacts());

  result.clear();
  ASSERT_TRUE(group->collide(option, &result));
  EXPECT_EQ(2u, result.getNumContacts());
}

//==============================================================================
TEST(Issue1654, OdeHonorsMaxNumContacts)
{
  auto detector = OdeCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  auto capsuleFrame = std::make_shared<SimpleFrame>(Frame::World(), "capsule");
  auto planeFrame = std::make_shared<SimpleFrame>(Frame::World(), "plane");

  auto capsuleShape = std::make_shared<CapsuleShape>(0.2, 0.6);
  capsuleFrame->setShape(capsuleShape);
  capsuleFrame->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.2));

  auto planeShape = std::make_shared<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0);
  planeFrame->setShape(planeShape);

  group->addShapeFrame(capsuleFrame.get());
  group->addShapeFrame(planeFrame.get());

  CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 1u;
  CollisionResult result;

  ASSERT_TRUE(group->collide(option, &result));
  EXPECT_EQ(1u, result.getNumContacts());

  option.maxNumContacts = 8u;
  option.maxNumContactsPerPair = 1u;
  result.clear();
  ASSERT_TRUE(group->collide(option, &result));
  EXPECT_EQ(1u, result.getNumContacts());

  result.clear();
  ASSERT_TRUE(group->collide(option, &result));
  EXPECT_EQ(1u, result.getNumContacts());
}
#endif // HAVE_ODE

//==============================================================================
#if HAVE_OCTOMAP && FCL_HAVE_OCTOMAP
TEST_F(Collision, VoxelGrid)
{
  auto simpleFrame1 = SimpleFrame::createShared(Frame::World());
  auto simpleFrame2 = SimpleFrame::createShared(Frame::World());

  auto shape1 = std::make_shared<VoxelGridShape>(0.01);
  auto shape2 = std::make_shared<SphereShape>(0.001);

  simpleFrame1->setShape(shape1);
  simpleFrame2->setShape(shape2);

  auto cd = FCLCollisionDetector::create();
  auto group = cd->createCollisionGroup(simpleFrame1.get(), simpleFrame2.get());

  EXPECT_EQ(group->getNumShapeFrames(), 2u);

  collision::CollisionOption option;
  option.enableContact = true;

  collision::CollisionResult result;

  result.clear();
  simpleFrame2->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.0));
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_TRUE(result.getNumContacts() == 0u);

  result.clear();
  shape1->updateOccupancy(Eigen::Vector3d(0.0, 0.0, 0.0), true);
  simpleFrame2->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.0));
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_TRUE(result.getNumContacts() >= 1u);
}
#endif // HAVE_OCTOMAP && FCL_HAVE_OCTOMAP
