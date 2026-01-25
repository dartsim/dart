// Copyright (c) 2011-2025, The DART development contributors

#include <dart/collision/collision_group.hpp>
#include <dart/collision/collision_object.hpp>
#include <dart/collision/collision_option.hpp>
#include <dart/collision/collision_result.hpp>
#include <dart/collision/distance_option.hpp>
#include <dart/collision/distance_result.hpp>
#include <dart/collision/fcl/fcl_collision_detector.hpp>
#include <dart/collision/fcl/fcl_collision_group.hpp>

#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/cone_shape.hpp>
#include <dart/dynamics/cylinder_shape.hpp>
#include <dart/dynamics/ellipsoid_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/mesh_shape.hpp>
#include <dart/dynamics/plane_shape.hpp>
#include <dart/dynamics/pyramid_shape.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/sphere_shape.hpp>

#include <dart/math/tri_mesh.hpp>

#include <dart/common/platform.hpp>

#include <gtest/gtest.h>

#include <memory>

using namespace dart;
using namespace dart::collision;
using namespace dart::dynamics;

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
    CollisionDetectorPtr detector,
    const ShapePtr& shapeA,
    const ShapePtr& shapeB,
    const Eigen::Vector3d& offset)
{
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

std::shared_ptr<math::TriMesh<double>> createSimpleTriMesh()
{
  auto mesh = std::make_shared<math::TriMesh<double>>();
  mesh->reserveVertices(4);
  mesh->addVertex(0.0, 0.0, 0.0);
  mesh->addVertex(1.0, 0.0, 0.0);
  mesh->addVertex(0.5, 1.0, 0.0);
  mesh->addVertex(0.5, 0.5, 1.0);

  mesh->reserveTriangles(4);
  mesh->addTriangle(0, 1, 2);
  mesh->addTriangle(0, 1, 3);
  mesh->addTriangle(1, 2, 3);
  mesh->addTriangle(2, 0, 3);

  return mesh;
}

class ExposedFCLCollisionGroup : public FCLCollisionGroup
{
public:
  using FCLCollisionGroup::FCLCollisionGroup;
  using FCLCollisionGroup::getFCLCollisionManager;
};

} // namespace

TEST(FCLCollisionDetector, CreateAndDestroy)
{
  auto detector = FCLCollisionDetector::create();
  EXPECT_NE(detector, nullptr);
  EXPECT_EQ(detector->getTypeView(), "fcl");
}

TEST(FCLCollisionDetector, CloneWithoutCollisionObjects)
{
  auto detector1 = FCLCollisionDetector::create();
  auto detector2 = detector1->cloneWithoutCollisionObjects();

  EXPECT_NE(detector2, nullptr);
  EXPECT_EQ(detector2->getTypeView(), "fcl");
}

TEST(FCLCollisionDetector, CreateCollisionGroup)
{
  auto detector = FCLCollisionDetector::create();
  auto group = detector->createCollisionGroup();
  EXPECT_NE(group, nullptr);
}

TEST(FCLCollisionDetector, PrimitiveShapeType)
{
  auto detector = FCLCollisionDetector::create();

  EXPECT_EQ(
      detector->getPrimitiveShapeType(),
      FCLCollisionDetector::PrimitiveShape::PRIMITIVE);

  detector->setPrimitiveShapeType(FCLCollisionDetector::PrimitiveShape::MESH);
  EXPECT_EQ(
      detector->getPrimitiveShapeType(),
      FCLCollisionDetector::PrimitiveShape::MESH);

  detector->setPrimitiveShapeType(
      FCLCollisionDetector::PrimitiveShape::PRIMITIVE);
  EXPECT_EQ(
      detector->getPrimitiveShapeType(),
      FCLCollisionDetector::PrimitiveShape::PRIMITIVE);
}

TEST(FCLCollisionDetector, ContactPointComputationMethod)
{
  auto detector = FCLCollisionDetector::create();

  EXPECT_EQ(
      detector->getContactPointComputationMethod(),
      FCLCollisionDetector::ContactPointComputationMethod::DART);

  detector->setContactPointComputationMethod(
      FCLCollisionDetector::ContactPointComputationMethod::FCL);
  EXPECT_EQ(
      detector->getContactPointComputationMethod(),
      FCLCollisionDetector::ContactPointComputationMethod::FCL);

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

  auto result = runCollision(detector, sphere1, sphere2, {0.7, 0.0, 0.0});
  EXPECT_TRUE(result.first);
  EXPECT_GT(result.second, 0u);
}

TEST(FCLCollisionDetector, BasicCollisionBoxSphere)
{
  auto detector = FCLCollisionDetector::create();
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));
  auto sphere = std::make_shared<SphereShape>(0.3);

  auto result = runCollision(detector, box, sphere, {0.4, 0.0, 0.0});
  EXPECT_TRUE(result.first);
  EXPECT_GT(result.second, 0u);
}

TEST(FCLCollisionDetector, BasicCollisionBoxBox)
{
  auto detector = FCLCollisionDetector::create();
  auto box1 = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));
  auto box2 = std::make_shared<BoxShape>(Eigen::Vector3d(0.5, 0.5, 0.5));

  auto result = runCollision(detector, box1, box2, {0.3, 0.0, 0.0});
  EXPECT_TRUE(result.first);
  EXPECT_GT(result.second, 0u);
}

TEST(FCLCollisionDetector, NoCollision)
{
  auto detector = FCLCollisionDetector::create();
  auto sphere1 = std::make_shared<SphereShape>(0.5);
  auto sphere2 = std::make_shared<SphereShape>(0.3);

  auto result = runCollision(detector, sphere1, sphere2, {2.0, 0.0, 0.0});
  EXPECT_FALSE(result.first);
  EXPECT_EQ(result.second, 0u);
}

TEST(FCLCollisionDetector, SharableCollisionObjectCaching)
{
  auto detector = FCLCollisionDetector::create();
  auto sphere = std::make_shared<SphereShape>(0.5);
  auto setup = makeShapeSetup("sphere", sphere);

  auto group1 = detector->createCollisionGroup();
  auto group2 = detector->createCollisionGroup();

  group1->addShapeFrame(setup.shapeNode);
  group2->addShapeFrame(setup.shapeNode);

  EXPECT_EQ(group1->getNumShapeFrames(), 1u);
  EXPECT_EQ(group2->getNumShapeFrames(), 1u);
  EXPECT_TRUE(group1->hasShapeFrame(setup.shapeNode));
  EXPECT_TRUE(group2->hasShapeFrame(setup.shapeNode));

  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.3, 0.0, 0.0);
  auto boxSetup = makeShapeSetup("box", box);
  FreeJoint::setTransformOf(boxSetup.body, tf);

  group1->addShapeFrame(boxSetup.shapeNode);

  CollisionOption option;
  CollisionResult result1;
  EXPECT_TRUE(group1->collide(option, &result1));
  EXPECT_GT(result1.getNumContacts(), 0u);

  CollisionResult result2;
  EXPECT_FALSE(group2->collide(option, &result2));
  EXPECT_EQ(result2.getNumContacts(), 0u);
}

TEST(FCLCollisionDetector, ConeShapeCollision)
{
  auto detector = FCLCollisionDetector::create();
  auto cone = std::make_shared<ConeShape>(0.5, 1.0);
  auto sphere = std::make_shared<SphereShape>(0.3);

  auto result = runCollision(detector, cone, sphere, {0.0, 0.0, 0.3});
  EXPECT_TRUE(result.first);
  EXPECT_GT(result.second, 0u);
}

TEST(FCLCollisionDetector, CylinderShapeCollision)
{
  auto detector = FCLCollisionDetector::create();
  auto cylinder = std::make_shared<CylinderShape>(0.5, 1.0);
  auto sphere = std::make_shared<SphereShape>(0.3);

  auto result = runCollision(detector, cylinder, sphere, {0.4, 0.0, 0.0});
  EXPECT_TRUE(result.first);
  EXPECT_GT(result.second, 0u);
}

TEST(FCLCollisionDetector, PlaneShapeCollision)
{
  auto detector = FCLCollisionDetector::create();
  auto plane = std::make_shared<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0);
  auto sphere = std::make_shared<SphereShape>(0.5);

  auto result = runCollision(detector, plane, sphere, {0.0, 0.0, 0.3});
  EXPECT_TRUE(result.first);
  EXPECT_GT(result.second, 0u);
}

TEST(FCLCollisionDetector, PyramidShapeCollision)
{
  auto detector = FCLCollisionDetector::create();
  auto pyramid = std::make_shared<PyramidShape>(1.0, 1.0, 1.0);
  auto sphere = std::make_shared<SphereShape>(0.3);

  auto result = runCollision(detector, pyramid, sphere, {0.0, 0.0, 0.3});
  EXPECT_TRUE(result.first);
  EXPECT_GT(result.second, 0u);
}

TEST(FCLCollisionDetector, MeshShapeCollision)
{
  auto detector = FCLCollisionDetector::create();
  auto triMesh = createSimpleTriMesh();
  auto mesh = std::make_shared<MeshShape>(Eigen::Vector3d::Ones(), triMesh, "");
  auto sphere = std::make_shared<SphereShape>(0.3);

  auto result = runCollision(detector, mesh, sphere, {0.5, 0.5, 0.3});
  EXPECT_TRUE(result.first);
  EXPECT_GT(result.second, 0u);
}

TEST(FCLCollisionDetector, EllipsoidShapeCollision)
{
  auto detector = FCLCollisionDetector::create();
  auto ellipsoid
      = std::make_shared<EllipsoidShape>(Eigen::Vector3d(0.5, 0.3, 0.4));
  auto sphere = std::make_shared<SphereShape>(0.3);

  auto result = runCollision(detector, ellipsoid, sphere, {0.4, 0.0, 0.0});
  EXPECT_TRUE(result.first);
  EXPECT_GT(result.second, 0u);
}

TEST(FCLCollisionDetector, PrimitiveShapeTypeAsMesh)
{
  auto detector = FCLCollisionDetector::create();
  detector->setPrimitiveShapeType(FCLCollisionDetector::PrimitiveShape::MESH);

  auto sphere = std::make_shared<SphereShape>(0.5);
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));

  auto result = runCollision(detector, sphere, box, {0.4, 0.0, 0.0});
  EXPECT_TRUE(result.first);
  EXPECT_GT(result.second, 0u);
}

TEST(FCLCollisionDetector, ConeShapeAsMesh)
{
  auto detector = FCLCollisionDetector::create();
  detector->setPrimitiveShapeType(FCLCollisionDetector::PrimitiveShape::MESH);

  auto cone = std::make_shared<ConeShape>(0.5, 1.0);
  auto sphere = std::make_shared<SphereShape>(0.3);

  auto result = runCollision(detector, cone, sphere, {0.0, 0.0, 0.3});
  EXPECT_TRUE(result.first);
  EXPECT_GT(result.second, 0u);
}

TEST(FCLCollisionDetector, CylinderShapeAsMesh)
{
  auto detector = FCLCollisionDetector::create();
  detector->setPrimitiveShapeType(FCLCollisionDetector::PrimitiveShape::MESH);

  auto cylinder = std::make_shared<CylinderShape>(0.5, 1.0);
  auto sphere = std::make_shared<SphereShape>(0.3);

  auto result = runCollision(detector, cylinder, sphere, {0.4, 0.0, 0.0});
  EXPECT_TRUE(result.first);
  EXPECT_GT(result.second, 0u);
}

#if !DART_OS_WINDOWS
TEST(FCLCollisionGroup, GetFCLCollisionManager)
{
  auto detector = FCLCollisionDetector::create();
  auto baseGroup = detector->createCollisionGroup();

  auto* exposedGroup = static_cast<ExposedFCLCollisionGroup*>(baseGroup.get());
  ASSERT_NE(exposedGroup, nullptr);

  auto* manager = exposedGroup->getFCLCollisionManager();
  EXPECT_NE(manager, nullptr);
}
#endif // !DART_OS_WINDOWS
