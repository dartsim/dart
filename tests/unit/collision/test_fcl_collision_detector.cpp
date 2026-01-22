// Copyright (c) 2011-2025, The DART development contributors

#include <dart/collision/collision_option.hpp>
#include <dart/collision/collision_result.hpp>
#include <dart/collision/distance_option.hpp>
#include <dart/collision/distance_result.hpp>
#include <dart/collision/fcl/fcl_collision_detector.hpp>

#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/cone_shape.hpp>
#include <dart/dynamics/convex_mesh_shape.hpp>
#include <dart/dynamics/cylinder_shape.hpp>
#include <dart/dynamics/ellipsoid_shape.hpp>
#include <dart/dynamics/mesh_shape.hpp>
#include <dart/dynamics/plane_shape.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/sphere_shape.hpp>

#include <dart/math/constants.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

using namespace dart;
using namespace dart::collision;
using namespace dart::dynamics;

namespace {

class TestCollisionObject final : public collision::CollisionObject
{
public:
  TestCollisionObject(
      collision::CollisionDetector* detector,
      const dynamics::ShapeFrame* shapeFrame)
    : collision::CollisionObject(detector, shapeFrame)
  {
  }

private:
  void updateEngineData() override {}
};

struct CollisionHandle
{
  std::shared_ptr<SimpleFrame> frame;
  std::unique_ptr<TestCollisionObject> object;
};

CollisionHandle makeObject(
    const dynamics::ShapePtr& shape,
    collision::CollisionDetector* detector,
    const Eigen::Isometry3d& tf)
{
  auto frame = std::make_shared<SimpleFrame>(Frame::World(), "frame");
  frame->setShape(shape);
  frame->setRelativeTransform(tf);

  auto object = std::make_unique<TestCollisionObject>(detector, frame.get());
  return {frame, std::move(object)};
}

} // namespace

TEST(FCLCollisionDetector, CreateAndDestroy)
{
  auto detector = FCLCollisionDetector::create();
  EXPECT_TRUE(detector != nullptr);
  EXPECT_EQ(detector->getType(), FCLCollisionDetector::getStaticType());

  // Test destructor
  detector.reset();
}

TEST(FCLCollisionDetector, CloneWithoutCollisionObjects)
{
  auto detector1 = FCLCollisionDetector::create();

  auto sphere = std::make_shared<SphereShape>(0.5);
  auto obj1
      = makeObject(sphere, detector1.get(), Eigen::Isometry3d::Identity());

  detector1->addCollisionObject(obj1.object.get());
  auto detector2 = detector1->cloneWithoutCollisionObjects();

  EXPECT_TRUE(detector2 != nullptr);
  EXPECT_EQ(detector2->getType(), FCLCollisionDetector::getStaticType());

  // Verify collision objects were not cloned
  CollisionResult result;
  detector2->collide(obj1.object.get(), obj1.object.get(), result);
  EXPECT_EQ(result.getNumContacts(), 0u);
}

TEST(FCLCollisionDetector, GetType)
{
  auto detector = FCLCollisionDetector::create();
  EXPECT_EQ(detector->getType(), FCLCollisionDetector::getStaticType());
  EXPECT_EQ(detector->getType(), "fcl");
}

TEST(FCLCollisionDetector, CreateCollisionGroup)
{
  auto detector = FCLCollisionDetector::create();
  auto group = detector->createCollisionGroup();
  EXPECT_TRUE(group != nullptr);
}

TEST(FCLCollisionDetector, CreateCollisionObject)
{
  auto detector = FCLCollisionDetector::create();
  auto sphere = std::make_shared<SphereShape>(0.5);
  auto obj = makeObject(sphere, detector.get(), Eigen::Isometry3d::Identity());

  auto collisionObj = detector->createCollisionObject(obj.object.get());
  EXPECT_TRUE(collisionObj != nullptr);
}

TEST(FCLCollisionDetector, PrimitiveShapeType)
{
  auto detector = FCLCollisionDetector::create();

  // Test default
  EXPECT_EQ(
      detector->getPrimitiveShapeType(),
      FCLCollisionDetector::PrimitiveShape::PRIMITIVE);

  // Test setting to MESH
  detector->setPrimitiveShapeType(FCLCollisionDetector::PrimitiveShape::MESH);
  EXPECT_EQ(
      detector->getPrimitiveShapeType(),
      FCLCollisionDetector::PrimitiveShape::MESH);

  // Test setting to PRIMITIVE again
  detector->setPrimitiveShapeType(
      FCLCollisionDetector::PrimitiveShape::PRIMITIVE);
  EXPECT_EQ(
      detector->getPrimitiveShapeType(),
      FCLCollisionDetector::PrimitiveShape::PRIMITIVE);
}

TEST(FCLCollisionDetector, ContactPointComputationMethod)
{
  auto detector = FCLCollisionDetector::create();

  // Test default
  EXPECT_EQ(
      detector->getContactPointComputationMethod(),
      FCLCollisionDetector::ContactPointComputationMethod::DART);

  // Test setting to FCL
  detector->setContactPointComputationMethod(
      FCLCollisionDetector::ContactPointComputationMethod::FCL);
  EXPECT_EQ(
      detector->getContactPointComputationMethod(),
      FCLCollisionDetector::ContactPointComputationMethod::FCL);

  // Test setting to DART again
  detector->setContactPointComputationMethod(
      FCLCollisionDetector::ContactPointComputationMethod::DART);
  EXPECT_EQ(
      detector->getContactPointComputationMethod(),
      FCLCollisionDetector::ContactPointComputationMethod::DART);
}

TEST(FCLCollisionDetector, BasicCollisionSphereSphere)
{
  auto detector = FCLCollisionDetector::create();
  auto sphere1 = std::make_shared<SphereShape>(0.5);
  auto sphere2 = std::make_shared<SphereShape>(0.3);

  auto obj1
      = makeObject(sphere1, detector.get(), Eigen::Isometry3d::Identity());

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0.7, 0.0, 0.0);
  auto obj2 = makeObject(sphere2, detector.get(), tf2);

  CollisionResult result;
  EXPECT_TRUE(detector->collide(obj1.object.get(), obj2.object.get(), result));
  EXPECT_GT(result.getNumContacts(), 0u);
}

TEST(FCLCollisionDetector, BasicCollisionBoxSphere)
{
  auto detector = FCLCollisionDetector::create();
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));
  auto sphere = std::make_shared<SphereShape>(0.3);

  auto boxObj = makeObject(box, detector.get(), Eigen::Isometry3d::Identity());

  Eigen::Isometry3d sphereTf = Eigen::Isometry3d::Identity();
  sphereTf.translation() = Eigen::Vector3d(0.4, 0.0, 0.0);
  auto sphereObj = makeObject(sphere, detector.get(), sphereTf);

  CollisionResult result;
  EXPECT_TRUE(
      detector->collide(boxObj.object.get(), sphereObj.object.get(), result));
  EXPECT_GT(result.getNumContacts(), 0u);
}

TEST(FCLCollisionDetector, BasicCollisionBoxBox)
{
  auto detector = FCLCollisionDetector::create();
  auto box1 = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));
  auto box2 = std::make_shared<BoxShape>(Eigen::Vector3d(0.5, 0.5, 0.5));

  auto obj1 = makeObject(box1, detector.get(), Eigen::Isometry3d::Identity());

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0.3, 0.0, 0.0);
  auto obj2 = makeObject(box2, detector.get(), tf2);

  CollisionResult result;
  EXPECT_TRUE(detector->collide(obj1.object.get(), obj2.object.get(), result));
  EXPECT_GT(result.getNumContacts(), 0u);
}

TEST(FCLCollisionDetector, CollisionWithOption)
{
  auto detector = FCLCollisionDetector::create();
  auto sphere1 = std::make_shared<SphereShape>(0.5);
  auto sphere2 = std::make_shared<SphereShape>(0.3);

  auto obj1
      = makeObject(sphere1, detector.get(), Eigen::Isometry3d::Identity());
  auto obj2
      = makeObject(sphere2, detector.get(), Eigen::Isometry3d::Identity());

  // Test with collision options
  CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 10;
  option.binaryCheck = false;

  CollisionResult result;
  EXPECT_TRUE(
      detector->collide(obj1.object.get(), obj2.object.get(), option, result));
  EXPECT_GT(result.getNumContacts(), 0u);
}

TEST(FCLCollisionDetector, NoCollision)
{
  auto detector = FCLCollisionDetector::create();
  auto sphere1 = std::make_shared<SphereShape>(0.5);
  auto sphere2 = std::make_shared<SphereShape>(0.3);

  auto obj1
      = makeObject(sphere1, detector.get(), Eigen::Isometry3d::Identity());

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(2.0, 0.0, 0.0); // Far apart
  auto obj2 = makeObject(sphere2, detector.get(), tf2);

  CollisionResult result;
  EXPECT_FALSE(detector->collide(obj1.object.get(), obj2.object.get(), result));
  EXPECT_EQ(result.getNumContacts(), 0u);
}

TEST(FCLCollisionDetector, DistanceSphereSphere)
{
  auto detector = FCLCollisionDetector::create();
  auto sphere1 = std::make_shared<SphereShape>(0.5);
  auto sphere2 = std::make_shared<SphereShape>(0.3);

  auto obj1
      = makeObject(sphere1, detector.get(), Eigen::Isometry3d::Identity());

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0.7, 0.0, 0.0);
  auto obj2 = makeObject(sphere2, detector.get(), tf2);

  DistanceOption option;
  DistanceResult result;
  double distance = detector->distance(
      obj1.object.get(), obj2.object.get(), option, result);
  EXPECT_LT(distance, 0.0); // Should be some positive distance
}

TEST(FCLCollisionDetector, DistanceNoCollision)
{
  auto detector = FCLCollisionDetector::create();
  auto sphere1 = std::make_shared<SphereShape>(0.5);
  auto sphere2 = std::make_shared<SphereShape>(0.3);

  auto obj1
      = makeObject(sphere1, detector.get(), Eigen::Isometry3d::Identity());

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(2.0, 0.0, 0.0); // Far apart
  auto obj2 = makeObject(sphere2, detector.get(), tf2);

  DistanceOption option;
  DistanceResult result;
  double distance = detector->distance(
      obj1.object.get(), obj2.object.get(), option, result);
  EXPECT_GT(distance, 0.0); // Should be larger than overlap distance
}

TEST(FCLCollisionDetector, RefreshCollisionObject)
{
  auto detector = FCLCollisionDetector::create();
  auto sphere = std::make_shared<SphereShape>(0.5);
  auto obj = makeObject(sphere, detector.get(), Eigen::Isometry3d::Identity());

  // Add collision object
  detector->addCollisionObject(obj.object.get());

  // Test refresh
  detector->refreshCollisionObject(obj.object.get());

  // Verify collision still works
  auto sphere2 = std::make_shared<SphereShape>(0.3);
  auto obj2
      = makeObject(sphere2, detector.get(), Eigen::Isometry3d::Identity());

  CollisionResult result;
  EXPECT_TRUE(detector->collide(obj.object.get(), obj2.object.get(), result));
  EXPECT_GT(result.getNumContacts(), 0u);
}

TEST(FCLCollisionDetector, MultipleCollisionObjects)
{
  auto detector = FCLCollisionDetector::create();
  auto sphere1 = std::make_shared<SphereShape>(0.5);
  auto sphere2 = std::make_shared<SphereShape>(0.3);
  auto sphere3 = std::make_shared<SphereShape>(0.4);

  auto obj1
      = makeObject(sphere1, detector.get(), Eigen::Isometry3d::Identity());
  auto obj2
      = makeObject(sphere2, detector.get(), Eigen::Isometry3d::Identity());
  auto obj3
      = makeObject(sphere3, detector.get(), Eigen::Isometry3d::Identity());

  // Add multiple objects
  detector->addCollisionObject(obj1.object.get());
  detector->addCollisionObject(obj2.object.get());
  detector->addCollisionObject(obj3.object.get());

  // Test collision between first two
  CollisionResult result12;
  EXPECT_TRUE(
      detector->collide(obj1.object.get(), obj2.object.get(), result12));
  EXPECT_GT(result12.getNumContacts(), 0u);

  // Test collision with third object
  CollisionResult result23;
  EXPECT_TRUE(
      detector->collide(obj2.object.get(), obj3.object.get(), result23));
  EXPECT_GT(result23.getNumContacts(), 0u);
}

TEST(FCLCollisionDetector, StaticTypeConsistency)
{
  // Test that getStaticType() matches what's returned by create()
  EXPECT_EQ(FCLCollisionDetector::getStaticType(), "fcl");

  auto detector = FCLCollisionDetector::create();
  EXPECT_EQ(detector->getType(), FCLCollisionDetector::getStaticType());
}