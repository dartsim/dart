/*
 * Copyright (c) 2013-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#include <fcl/collision.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/narrowphase/narrowphase.h>

#include "dart/config.h"
#include "dart/common/common.h"
#include "dart/math/math.h"
#include "dart/collision/CollisionGroup.h"
#include "dart/collision/fcl/FCLCollisionDetector.h"
#include "dart/collision/fcl/FCLMeshCollisionDetector.h"
#include "dart/collision/dart/DARTCollisionDetector.h"
#ifdef HAVE_BULLET_COLLISION
  #include "dart/collision/bullet/BulletCollisionDetector.h"
#endif
#include "dart/dynamics/dynamics.h"
#include "dart/simulation/simulation.h"
#include "dart/utils/utils.h"
#include "TestHelpers.h"

using namespace dart;
using namespace common;
using namespace math;
using namespace collision;
using namespace dynamics;
using namespace simulation;
using namespace utils;

class COLLISION : public testing::Test
{
public:
    void unrotatedTest(fcl::CollisionGeometry* _coll1,
                       fcl::CollisionGeometry* _coll2,
                       double expectedContactPoint, int _idxAxis);
    void dropWithRotation(fcl::CollisionGeometry* _object,
                          double EulerZ, double EulerY, double EulerX);
	void printResult(const fcl::CollisionResult& _result);
};

void COLLISION::unrotatedTest(fcl::CollisionGeometry* _coll1,
                              fcl::CollisionGeometry* _coll2,
                              double expectedContactPoint,
                              int _idxAxis)
{
    fcl::CollisionResult result;
    fcl::CollisionRequest request;
    request.enable_contact = true;
    request.num_max_contacts = 100;

    fcl::Vec3f position(0, 0, 0);

    fcl::Transform3f coll1_transform;
    fcl::Transform3f coll2_transform;

    //==========================================================================
    // Approaching test
    //==========================================================================
    result.clear();
    double dpos = -0.001;
    double pos = 10.0;

    coll1_transform.setIdentity();
    coll1_transform.setTranslation(fcl::Vec3f(0, 0, 0));
    coll2_transform.setIdentity();

    // Let's drop box2 until it collide with box1
    do {
        position[_idxAxis] = pos;
        coll2_transform.setTranslation(position);

        fcl::collide(_coll1, coll1_transform,
                     _coll2, coll2_transform,
                     request, result);

        pos += dpos;
    }
    while (result.numContacts() == 0);

    //
    if (_idxAxis == 0)
        std::cout << "The object is collided when its x-axis position is: " << (pos - dpos) << std::endl;
    if (_idxAxis == 1)
        std::cout << "The object is collided when its y-axis position is: " << (pos - dpos) << std::endl;
    if (_idxAxis == 2)
        std::cout << "The object is collided when its z-axis position is: " << (pos - dpos) << std::endl;

    //printResult(result);

    for (size_t i = 0; i < result.numContacts(); ++i)
    {
        EXPECT_GE(result.getContact(i).penetration_depth, 0.0);
//		EXPECT_NEAR(result.getContact(i).normal[_idxAxis], -1.0);
        EXPECT_EQ(result.getContact(i).normal.length(), 1.0);
        EXPECT_NEAR(result.getContact(i).pos[_idxAxis], expectedContactPoint, -dpos*2.0);
    }
}

void COLLISION::dropWithRotation(fcl::CollisionGeometry* _object,
                                 double EulerZ, double EulerY, double EulerX)
{
    // Collision test setting
    fcl::CollisionResult result;
    fcl::CollisionRequest request;
    request.enable_contact = true;
    request.num_max_contacts = 100;

    // Ground like box setting
    fcl::Box groundObject(100, 100, 0.1);
    fcl::Transform3f groundTransf;
    groundTransf.setIdentity();
    fcl::Vec3f ground_position(0, 0, 0);
    groundTransf.setTranslation(ground_position);

    // Dropping object setting
    fcl::Transform3f objectTransf;
    fcl::Matrix3f rot;
    rot.setEulerZYX(EulerZ,
                    EulerY,
                    EulerX);
    objectTransf.setRotation(rot);
    fcl::Vec3f dropping_position(0, 0, 0);
    objectTransf.setTranslation(dropping_position);

    //==========================================================================
    // Dropping test in x, y, z aixs each.
    //==========================================================================
    for (int _idxAxis = 0; _idxAxis < 3; ++_idxAxis)
    {
        result.clear();

        groundObject.side = fcl::Vec3f(100, 100, 100);
        groundObject.side[_idxAxis] = 0.1;
        ground_position.setValue(0, 0, 0);
        ground_position[_idxAxis] = -0.05;
        groundTransf.setTranslation(ground_position);

        // Let's drop the object until it collide with ground
        double posDelta = -0.0001;
        double initPos = 10.0;
        dropping_position.setValue(0, 0, 0);
        do {
            dropping_position[_idxAxis] = initPos;
            objectTransf.setTranslation(dropping_position);

            fcl::collide(_object, objectTransf,
                         &groundObject, groundTransf,
                         request, result);

            initPos += posDelta;
        }
        while (result.numContacts() == 0);

        std::cout << "Current position of the object: "
                  << objectTransf.getTranslation()
                  << std::endl
                  << "Number of contacts: "
                  << result.numContacts()
                  << std::endl;

        fcl::Transform3f objectTransfInv = objectTransf;
        objectTransfInv.inverse();
        for (size_t i = 0; i < result.numContacts(); ++i)
        {
            fcl::Vec3f posWorld = objectTransfInv.transform(result.getContact(i).pos);
            std::cout << "----- CONTACT " << i << " --------" << std::endl;
            std::cout << "contact_points: " << result.getContact(i).pos << std::endl;
            std::cout << "contact_points(w): " << posWorld << std::endl;
            std::cout << "norm: " << result.getContact(i).pos.length() << std::endl;
            std::cout << "penetration_depth: " << result.getContact(i).penetration_depth << std::endl;
            std::cout << "normal: " << result.getContact(i).normal << std::endl;
        }

        std::cout << std::endl;
    }
}

void COLLISION::printResult(const fcl::CollisionResult& _result)
{
	std::cout << "====== [ RESULT ] ======" << std::endl;
	std::cout << "The number of contacts: " << _result.numContacts() << std::endl;

  for (size_t i = 0; i < _result.numContacts(); ++i)
	{
		std::cout << "----- CONTACT " << i << " --------" << std::endl;
		std::cout << "contact_points: " << _result.getContact(i).pos << std::endl;
		std::cout << "penetration_depth: " << _result.getContact(i).penetration_depth << std::endl;
		std::cout << "normal: " << _result.getContact(i).normal << std::endl;
		//std::cout << std::endl;
	}
	std::cout << std::endl;
}

/* ********************************************************************************************* */

//TEST_F(COLLISION, BOX_BOX_X) {
//	fcl::Box box1(2, 2, 2);
//	fcl::Box box2(1, 1, 1);
//	unrotatedTest(&box1, &box2, 1.0, 0); // x-axis
//}

//TEST_F(COLLISION, BOX_BOX_Y) {
//	fcl::Box box1(2, 2, 2);
//	fcl::Box box2(1, 1, 1);
//	unrotatedTest(&box1, &box2, 1.0, 1); // y-axis
//}

//TEST_F(COLLISION, BOX_BOX_Z) {
//	fcl::Box box1(2, 2, 2);
//	fcl::Box box2(1, 1, 1);
//	unrotatedTest(&box1, &box2, 1.0, 2); // z-axis
//}

//TEST_F(COLLISION, BOX_SPHERE_X) {
//	fcl::Box box1(2, 2, 2);
//	fcl::Sphere sphere(0.5);
//	unrotatedTest(&box1, &sphere, 1.0, 0); // x-axis
//}

//TEST_F(COLLISION, BOX_SPHERE_Y) {
//	fcl::Box box1(2, 2, 2);
//	fcl::Sphere sphere(0.5);
//	unrotatedTest(&box1, &sphere, 1.0, 1); // y-axis
//}

//TEST_F(COLLISION, BOX_SPHERE_Z) {
//	fcl::Box box1(2, 2, 2);
//	fcl::Sphere sphere(0.5);
//	unrotatedTest(&box1, &sphere, 1.0, 2); // z-axis
//}

//TEST_F(COLLISION, SPHERE_BOX_X) {
//	fcl::Sphere obj1(0.5);
//	fcl::Box obj2(2, 2, 2);
//	unrotatedTest(&obj1, &obj2, 0.5, 0); // x-axis
//}

//TEST_F(COLLISION, SPHERE_BOX_Y) {
//	fcl::Sphere obj1(0.5);
//	fcl::Box obj2(2, 2, 2);
//	unrotatedTest(&obj1, &obj2, 0.5, 1); // y-axis
//}

//TEST_F(COLLISION, SPHERE_BOX_Z) {
//	fcl::Sphere obj1(0.5);
//	fcl::Box obj2(2, 2, 2);
//	unrotatedTest(&obj1, &obj2, 0.5, 2); // z-axis
//}

//TEST_F(COLLISION, SPHERE_SPHERE_X) {
//	fcl::Sphere sphere1(1);
//	fcl::Sphere sphere2(0.5);
//	unrotatedTest(&sphere1, &sphere2, 1.0, 0); // x-axis
//}

//TEST_F(COLLISION, SPHERE_SPHERE_Y) {
//	fcl::Sphere sphere1(1);
//	fcl::Sphere sphere2(0.5);
//	unrotatedTest(&sphere1, &sphere2, 1.0, 1); // y-axis
//}

//TEST_F(COLLISION, SPHERE_SPHERE_Z) {
//	fcl::Sphere sphere1(1);
//	fcl::Sphere sphere2(0.5);
//	unrotatedTest(&sphere1, &sphere2, 1.0, 2); // z-axis
//}

//TEST_F(COLLISION, PLANE_BOX_X) {
//	fcl::Plane obj1(1, 0, 0, 0);
//	fcl::Box obj2(1, 1, 1);
//	unrotatedTest(&obj1, &obj2, 0.0, 0); // x-axis
//}

//TEST_F(COLLISION, PLANE_BOX_Y) {
//	fcl::Plane obj1(0, 1, 0, 0);
//	fcl::Box obj2(1, 1, 1);
//	unrotatedTest(&obj1, &obj2, 0.0, 1); // x-axis
//}

//TEST_F(COLLISION, PLANE_BOX_Z) {
//	fcl::Plane obj1(0, 0, 1, 0);
//	fcl::Box obj2(1, 1, 1);
//	unrotatedTest(&obj1, &obj2, 0.0, 2); // x-axis
//}

//TEST_F(COLLISION, PLANE_SPHERE_X) {
//	fcl::Plane obj1(1, 0, 0, 0);
//	fcl::Sphere obj2(0.5);
//	unrotatedTest(&obj1, &obj2, 0.0, 0); // x-axis
//}

//TEST_F(COLLISION, PLANE_SPHERE_Y) {
//	fcl::Plane obj1(0, 1, 0, 0);
//	fcl::Sphere obj2(0.5);
//	unrotatedTest(&obj1, &obj2, 0.0, 1); // x-axis
//}

//TEST_F(COLLISION, PLANE_SPHERE_Z) {
//	fcl::Plane obj1(0, 0, 1, 0);
//	fcl::Sphere obj2(0.5);
//	unrotatedTest(&obj1, &obj2, 0.0, 2); // x-axis
//}

//TEST_F(COLLISION, PLANE_CYLINDER_X) {
//	fcl::Plane obj1(1, 0, 0, 0);
//	fcl::Cylinder obj2(0.5, 1);
//	unrotatedTest(&obj1, &obj2, 0.0, 0); // x-axis
//}

//TEST_F(COLLISION, PLANE_CYLINDER_Y) {
//	fcl::Plane obj1(0, 1, 0, 0);
//	fcl::Cylinder obj2(0.5, 1);
//	unrotatedTest(&obj1, &obj2, 0.0, 1); // x-axis
//}

//TEST_F(COLLISION, PLANE_CYLINDER_Z) {
//	fcl::Plane obj1(0, 0, 1, 0);
//	fcl::Cylinder obj2(0.5, 1);
//	unrotatedTest(&obj1, &obj2, 0.0, 2); // x-axis
//}

//TEST_F(COLLISION, PLANE_CAPSULE_X) {
//	fcl::Plane obj1(1, 0, 0, 0);
//	fcl::Capsule obj2(0.5, 2);
//	unrotatedTest(&obj1, &obj2, 0.0, 0); // x-axis
//}

//TEST_F(COLLISION, PLANE_CAPSULE_Y) {
//	fcl::Plane obj1(0, 1, 0, 0);
//	fcl::Capsule obj2(0.5, 2);
//	unrotatedTest(&obj1, &obj2, 0.0, 1); // x-axis
//}

//TEST_F(COLLISION, PLANE_CAPSULE_Z) {
//	fcl::Plane obj1(0, 0, 1, 0);
//	fcl::Capsule obj2(0.5, 2);
//	unrotatedTest(&obj1, &obj2, 0.0, 2); // x-axis
//}

//TEST_F(COLLISION, PLANE_CONE_X) {
//	fcl::Plane obj1(1, 0, 0, 0);
//	fcl::Cone obj2(0.5, 1);
//	unrotatedTest(&obj1, &obj2, 0.0, 0); // x-axis
//}

//TEST_F(COLLISION, PLANE_CONE_Y) {
//	fcl::Plane obj1(0, 1, 0, 0);
//	fcl::Cone obj2(0.5, 1);
//	unrotatedTest(&obj1, &obj2, 0.0, 1); // x-axis
//}

//TEST_F(COLLISION, PLANE_CONE_Z) {
//	fcl::Plane obj1(0, 0, 1, 0);
//	fcl::Cone obj2(0.5, 1);
//	unrotatedTest(&obj1, &obj2, 0.0, 2); // x-axis
//}


TEST_F(COLLISION, DROP)
{
    dtdbg << "Unrotated box\n";
    fcl::Box box1(0.5, 0.5, 0.5);
    dropWithRotation(&box1, 0, 0, 0);

    dtdbg << "Rotated box\n";
    fcl::Box box2(0.5, 0.5, 0.5);
    dropWithRotation(&box2,
                     dart::math::random(-3.14, 3.14),
                     dart::math::random(-3.14, 3.14),
                     dart::math::random(-3.14, 3.14));

    dropWithRotation(&box2,
                     0.0,
                     0.1,
                     0.0);
}

TEST_F(COLLISION, FCL_BOX_BOX)
{
    double EulerZ = 1;
    double EulerY = 2;
    double EulerX = 3;

    // Collision test setting
    fcl::CollisionResult result;
    fcl::CollisionRequest request;
    request.enable_contact = true;
    request.num_max_contacts = 100;

    // Ground like box setting
    fcl::Box groundObject(100, 100, 0.1);
    fcl::Transform3f groundTransf;
    groundTransf.setIdentity();
    fcl::Vec3f ground_position(0.0, 0.0, -0.05);
    groundTransf.setTranslation(ground_position);

    // Dropping box object setting
    fcl::Box box(0.5, 0.5, 0.5);
    fcl::Transform3f objectTransf;
    fcl::Matrix3f rot;
    rot.setEulerZYX(EulerZ, EulerY, EulerX);
    objectTransf.setRotation(rot);
    fcl::Vec3f dropping_position(0.0, 0.0, 5.0);
    objectTransf.setTranslation(dropping_position);

    // Let's drop the object until it collide with ground
    do {
        objectTransf.setTranslation(dropping_position);

        fcl::collide(&box, objectTransf, &groundObject, groundTransf, request, result);

        dropping_position[2] -= 0.00001;
    }
    while (result.numContacts() == 0);

    std::cout << "Current position of the object: "
              << objectTransf.getTranslation()
              << std::endl
              << "Number of contacts: "
              << result.numContacts()
              << std::endl;

    for (size_t i = 0; i < result.numContacts(); ++i)
    {
        std::cout << "----- CONTACT " << i << " --------" << std::endl;
        std::cout << "contact_points: " << result.getContact(i).pos << std::endl;
        std::cout << "penetration_depth: " << result.getContact(i).penetration_depth << std::endl;
        std::cout << "normal: " << result.getContact(i).normal << std::endl;
    }
}

//TEST_F(COLLISION, OWN_COLLISION_CODE)
//{
//    double EulerZ = 1;
//    double EulerY = 2;
//    double EulerX = 3;

//    // Collision test setting
//    CollisionInfoArray result;

//    Vec3 size1(100.0, 100.0, 0.1);
//    SE3 T1(Vec3(0.0, 0.0, -0.05));

//    Vec3 size2(0.5, 0.5, 0.5);
//    Vec3 pos2(0.0, 0.0, 5.0);
//    SE3 T2;
//    T2 = EulerZYX(Vec3(EulerZ, EulerY, EulerX));
//    T2.setPosition(pos2);

//    // Let's drop the object until it collide with ground
//    do {
//        T2.setPosition(pos2);

//        _BoxBox_____________MARK8(size1, T1, size2, T2, result);

//        pos2[2] -= 0.00001;
//    }
//    while (result.size() == 0);

//    std::cout //<< "Current position of the object: "
//              //<< objectTransf.getTranslation()
//              //<< std::endl
//              << "Number of contacts: "
//              << result.size()
//              << std::endl;

//    for (int i = 0; i < result.size(); ++i)
//    {
//        std::cout << "----- CONTACT " << i << " --------" << std::endl;
//        std::cout << "contact_points: " << result.at(i).point;
//        std::cout << "penetration_depth: " << result.at(i).penetration << std::endl;
//        std::cout << "normal: " << result.at(i).normal << std::endl;
//    }


//}

//==============================================================================
template <class CollisionDetectorType>
void testFreeCollisionObjects()
{
  auto cd = CollisionDetectorType::create();

  ShapePtr shape1(new BoxShape(Eigen::Vector3d(1.0, 1.0, 1.0)));
  ShapePtr shape2(new BoxShape(Eigen::Vector3d(1.0, 1.0, 1.0)));
  ShapePtr shape3(new BoxShape(Eigen::Vector3d(1.0, 1.0, 1.0)));

  // Create collision objects in different ways
  auto obj1 = cd->template createCollisionObject<FreeCollisionObject>(shape1);
  auto obj2 = FreeCollisionObject::create(cd, shape2);
  auto obj3 = std::shared_ptr<FreeCollisionObject>(new FreeCollisionObject(cd, shape3));

  collision::CollisionGroupPtr group1(new collision::CollisionGroup(cd));
  group1->addCollisionObject(obj1);
  collision::CollisionGroupPtr group2(new collision::CollisionGroup(cd));
  group2->addCollisionObject(obj2);
  group2->addCollisionObject(obj3);

  collision::Option option;
  collision::Result result;

  obj1->setTranslation(Eigen::Vector3d::Zero());
  obj2->setTranslation(Eigen::Vector3d(1.1, 0.0, 0.0));
  obj3->setTranslation(Eigen::Vector3d(2.2, 0.0, 0.0));
  EXPECT_FALSE(obj1->detect(obj2.get(), option, result));
  EXPECT_FALSE(obj2->detect(obj3.get(), option, result));
  EXPECT_FALSE(obj1->detect(group2.get(), option, result));
  EXPECT_FALSE(group1->detect(obj3.get(), option, result));
  if (cd->getType() != "Bullet")
  {
    EXPECT_FALSE(group1->detect(group2.get(), option, result));
    EXPECT_FALSE(group2->detect(option, result));
  }

  obj1->setTranslation(Eigen::Vector3d::Zero());
  obj2->setTranslation(Eigen::Vector3d(0.5, 0.0, 0.0));
  obj3->setTranslation(Eigen::Vector3d(1.0, 0.0, 0.0));
  EXPECT_TRUE(obj1->detect(obj2.get(), option, result));
  EXPECT_TRUE(obj2->detect(obj3.get(), option, result));
  EXPECT_TRUE(obj1->detect(group2.get(), option, result));
  if (cd->getType() != "Bullet")
    EXPECT_TRUE(group1->detect(group2.get(), option, result));
  EXPECT_TRUE(group2->detect(option, result));
  EXPECT_TRUE(group2->detect(option, result));

  for (auto contact : result.contacts)
  {
    auto freeCollObj1 = dynamic_cast<collision::FreeCollisionObject*>(
          contact.collisionObject1);
    auto freeCollObj2 = dynamic_cast<collision::FreeCollisionObject*>(
          contact.collisionObject2);

    EXPECT_NE(freeCollObj1, nullptr);
    EXPECT_NE(freeCollObj2, nullptr);
  }
}

//==============================================================================
TEST_F(COLLISION, FreeCollisionObjects)
{
  testFreeCollisionObjects<collision::FCLCollisionDetector>();
  testFreeCollisionObjects<collision::FCLMeshCollisionDetector>();
  testFreeCollisionObjects<collision::DARTCollisionDetector>();
  testFreeCollisionObjects<collision::BulletCollisionDetector>();
}

//==============================================================================
template <class CollisionDetectorType>
void testBodyNodes()
{
  auto cd = CollisionDetectorType::create();

  Eigen::Vector3d size(1.0, 1.0, 1.0);
  Eigen::Vector3d pos1(0.0, 0.0, 0.0);
  Eigen::Vector3d pos2(0.5, 0.0, 0.0);

  auto boxSkel1 = createBox(size, pos1);
  auto boxSkel2 = createBox(size, pos2);

  auto boxBody1 = boxSkel1->getBodyNode(0u);
  auto boxBody2 = boxSkel2->getBodyNode(0u);

  auto boxShape1 = boxBody1->getCollisionShape(0u);
  auto boxShape2 = boxBody2->getCollisionShape(0u);

  collision::Option option;
  collision::Result result;

  auto obj1 = cd->template createCollisionObject<ShapeFrameCollisionObject>(
        boxShape1, boxBody1);
  auto obj2 = cd->template createCollisionObject<ShapeFrameCollisionObject>(
        boxShape2, boxBody2);

  EXPECT_TRUE(obj1->detect(obj2.get(), option, result));
  EXPECT_TRUE(cd->detect(obj1.get(), obj2.get(), option, result));

  auto obj3 = cd->template createCollisionObject<ShapeFrameCollisionObject>(
        boxShape2, boxBody2);

  EXPECT_FALSE(obj1->isEqual(obj2.get()));
  EXPECT_FALSE(obj2->isEqual(obj1.get()));

  EXPECT_FALSE(obj1->isEqual(obj3.get()));
  EXPECT_FALSE(obj3->isEqual(obj1.get()));

  EXPECT_TRUE(obj2->isEqual(obj3.get()));
  EXPECT_TRUE(obj3->isEqual(obj2.get()));

  collision::CollisionGroupPtr group1(new collision::CollisionGroup(cd));
  group1->addCollisionObject(obj2);
  collision::CollisionGroupPtr group2(new collision::CollisionGroup(cd));
  group2->addCollisionObject(obj3);

  EXPECT_EQ(group1->getCollisionObjects().size(), 1u);
  EXPECT_EQ(group2->getCollisionObjects().size(), 1u);

  group1->unionGroup(group2);
  EXPECT_EQ(group1->getCollisionObjects().size(), 1u);
}

//==============================================================================
TEST_F(COLLISION, BodyNodeNodes)
{
  testBodyNodes<collision::FCLCollisionDetector>();
  testBodyNodes<collision::FCLMeshCollisionDetector>();
  testBodyNodes<collision::DARTCollisionDetector>();
  testBodyNodes<collision::BulletCollisionDetector>();
}

//==============================================================================
template <class CollisionDetectorType>
void testSkeletons()
{
//  auto engine = CollisionDetectorType::create();

//  Eigen::Vector3d size(1.0, 1.0, 1.0);
//  Eigen::Vector3d pos1(0.0, 0.0, 0.0);
//  Eigen::Vector3d pos2(0.5, 0.0, 0.0);

//  auto boxSkel1 = createBox(size, pos1);
//  auto boxSkel2 = createBox(size, pos2);

//  collision::Option option;
//  collision::Result result;

//  auto hit = dynamics::detect(engine, boxSkel1, boxSkel2, option, result);

//  EXPECT_TRUE(hit);
}

//==============================================================================
TEST_F(COLLISION, Skeletons)
{
  testSkeletons<collision::FCLCollisionDetector>();
  testSkeletons<collision::FCLMeshCollisionDetector>();
  testSkeletons<collision::DARTCollisionDetector>();
  testSkeletons<collision::BulletCollisionDetector>();
}

//==============================================================================
TEST_F(COLLISION, CollisionOfPrescribedJoints)
{
  // There are one red plate (static skeleton) and 5 pendulums with different
  // actuator types. This test check if the motion prescribed joints are exactly
  // tracking the prescribed motion eventhough there are collision with other
  // objects.

  const double tol       = 1e-9;
  const double timeStep  = 1e-3;
  const size_t numFrames = 5e+0;  // 5 secs

  // Load world and skeleton
  WorldPtr world = SkelParser::readWorld(
        DART_DATA_PATH"/skel/test/collision_of_prescribed_joints_test.skel");
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

  for (size_t i = 0; i < numFrames; ++i)
  {
    const double time = world->getTime();

    joint1->setCommand(0, -0.5 * DART_PI * std::cos(time));
    joint2->setCommand(0, -0.5 * DART_PI * std::cos(time));
    joint3->setCommand(0, -0.5 * DART_PI * std::cos(time));
    joint4->setCommand(0, -0.5 * DART_PI * std::cos(time));
    joint5->setCommand(0, -0.5 * DART_PI * std::sin(time));
    joint6->setCommand(0, -0.5 * DART_PI * std::sin(time));  // ignored

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

    // The velocity and acceleration of locked joint always must be zero.
    EXPECT_TRUE(joint6->isKinematic());
    EXPECT_NEAR(joint6->getVelocity(0), 0.0, tol);
    EXPECT_NEAR(joint6->getAcceleration(0), 0.0, tol);
  }
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

