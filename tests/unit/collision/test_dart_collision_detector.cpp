// Copyright (c) 2011-2025, The DART development contributors

#include <dart/collision/collision_group.hpp>
#include <dart/collision/collision_option.hpp>
#include <dart/collision/collision_result.hpp>
#include <dart/collision/dart/dart_collision_detector.hpp>
#include <dart/collision/raycast_option.hpp>
#include <dart/collision/raycast_result.hpp>

#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/cylinder_shape.hpp>
#include <dart/dynamics/ellipsoid_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/sphere_shape.hpp>

#include <gtest/gtest.h>

#include <utility>

using namespace dart;
using namespace dart::collision;
using namespace dart::dynamics;
using dart::collision::RaycastOption;
using dart::collision::RaycastResult;

namespace {

struct ShapeSetup
{
  SkeletonPtr skeleton;
  BodyNode* body{nullptr};
  ShapeNode* shapeNode{nullptr};
};

ShapeSetup makeShapeSetup(const std::string& name, const ShapePtr& shape)
{
  ShapeSetup setup;
  setup.skeleton = Skeleton::create(name);
  auto pair = setup.skeleton->createJointAndBodyNodePair<FreeJoint>();
  setup.body = pair.second;
  setup.shapeNode = setup.body->createShapeNodeWith<CollisionAspect>(shape);
  return setup;
}

std::pair<bool, std::size_t> runCollision(
    const ShapePtr& shapeA,
    const ShapePtr& shapeB,
    const Eigen::Vector3d& offset)
{
  auto detector = DARTCollisionDetector::create();
  auto setupA = makeShapeSetup("shape_a", shapeA);
  auto setupB = makeShapeSetup("shape_b", shapeB);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = offset;
  FreeJoint::setTransformOf(setupB.body, tf);

  auto group = detector->createCollisionGroup();
  group->addShapeFrame(setupA.shapeNode);
  group->addShapeFrame(setupB.shapeNode);

  CollisionOption option;
  option.maxNumContacts = 10u;

  CollisionResult result;
  const bool collided = group->collide(option, &result);
  return {collided, result.getNumContacts()};
}

} // namespace

TEST(DARTCollisionDetector, SupportedShapePairs)
{
  const Eigen::Vector3d overlap(0.6, 0.0, 0.0);
  const Eigen::Vector3d separate(2.0, 0.0, 0.0);

  auto sphere = std::make_shared<SphereShape>(0.5);
  auto sphere2 = std::make_shared<SphereShape>(0.5);
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(1.0));
  // EllipsoidShape constructor expects diameters, so 1.0 gives radius 0.5.
  auto ellipsoid
      = std::make_shared<EllipsoidShape>(Eigen::Vector3d::Constant(1.0));

  auto result = runCollision(sphere, sphere2, overlap);
  EXPECT_TRUE(result.first);
  EXPECT_LT(0u, result.second);

  result = runCollision(sphere, box, overlap);
  EXPECT_TRUE(result.first);
  EXPECT_LT(0u, result.second);

  result = runCollision(box, sphere, overlap);
  EXPECT_TRUE(result.first);
  EXPECT_LT(0u, result.second);

  result = runCollision(box, box, overlap);
  EXPECT_TRUE(result.first);
  EXPECT_LT(0u, result.second);

  result = runCollision(sphere, ellipsoid, overlap);
  EXPECT_TRUE(result.first);
  EXPECT_LT(0u, result.second);

  result = runCollision(ellipsoid, sphere, overlap);
  EXPECT_TRUE(result.first);
  EXPECT_LT(0u, result.second);

  result = runCollision(box, ellipsoid, overlap);
  EXPECT_TRUE(result.first);
  EXPECT_LT(0u, result.second);

  result = runCollision(ellipsoid, box, overlap);
  EXPECT_TRUE(result.first);
  EXPECT_LT(0u, result.second);

  result = runCollision(ellipsoid, ellipsoid, overlap);
  EXPECT_TRUE(result.first);
  EXPECT_LT(0u, result.second);

  result = runCollision(sphere, sphere2, separate);
  EXPECT_FALSE(result.first);
  EXPECT_EQ(result.second, 0u);
}

TEST(DARTCollisionDetector, UnsupportedShapePairReturnsFalse)
{
  auto cylinder = std::make_shared<CylinderShape>(0.5, 1.0);
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(1.0));

  const auto result = runCollision(cylinder, box, Eigen::Vector3d::Zero());
  EXPECT_FALSE(result.first);
  EXPECT_EQ(result.second, 0u);
}

TEST(DARTCollisionDetector, RaycastReturnsNotSupported)
{
  // DARTCollisionDetector does not override raycast, so it uses the base
  // implementation which returns false and logs a warning.
  auto detector = DARTCollisionDetector::create();
  auto sphere = std::make_shared<SphereShape>(0.5);
  auto setup = makeShapeSetup("sphere", sphere);

  auto group = detector->createCollisionGroup();
  group->addShapeFrame(setup.shapeNode);

  RaycastOption option;
  RaycastResult result;

  const Eigen::Vector3d from(0.0, 0.0, 2.0);
  const Eigen::Vector3d to(0.0, 0.0, -2.0);

  const bool hit = group->raycast(from, to, option, &result);
  EXPECT_FALSE(hit);
  EXPECT_FALSE(result.hasHit());
}
