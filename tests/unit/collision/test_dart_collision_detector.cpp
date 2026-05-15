// Copyright (c) 2011, The DART development contributors

#include <dart/config.hpp>

#include <dart/collision/collision_group.hpp>
#include <dart/collision/collision_option.hpp>
#include <dart/collision/collision_result.hpp>
#include <dart/collision/dart/DARTCollisionDetector.hpp>
#include <dart/collision/dart/dart_collision_detector.hpp>
#include <dart/collision/dart/dart_collision_group.hpp>
#include <dart/collision/fcl/FCLCollisionDetector.hpp>
#include <dart/collision/native/persistent_manifold_cache.hpp>
#include <dart/collision/ode/OdeCollisionDetector.hpp>
#include <dart/collision/raycast_option.hpp>
#include <dart/collision/raycast_result.hpp>

#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/convex_mesh_shape.hpp>
#include <dart/dynamics/cylinder_shape.hpp>
#include <dart/dynamics/ellipsoid_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/heightmap_shape.hpp>
#include <dart/dynamics/mesh_shape.hpp>
#include <dart/dynamics/plane_shape.hpp>
#include <dart/dynamics/point_cloud_shape.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/soft_body_node.hpp>
#include <dart/dynamics/soft_mesh_shape.hpp>
#include <dart/dynamics/sphere_shape.hpp>

#include <dart/math/tri_mesh.hpp>

#if DART_HAVE_OCTOMAP
  #include <dart/dynamics/voxel_grid_shape.hpp>
#endif

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
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

struct SoftShapeSetup
{
  SkeletonPtr skeleton;
  SoftBodyNode* body{nullptr};
  ShapeNode* shapeNode{nullptr};
};

class TestDartCollisionDetector : public DartCollisionDetector
{
public:
  static std::shared_ptr<TestDartCollisionDetector> create()
  {
    return std::shared_ptr<TestDartCollisionDetector>(
        new TestDartCollisionDetector());
  }

  using CollisionDetector::claimCollisionObject;

private:
  TestDartCollisionDetector() = default;
};

class TestDartCollisionGroup : public DartCollisionGroup
{
public:
  explicit TestDartCollisionGroup(const CollisionDetectorPtr& collisionDetector)
    : DartCollisionGroup(collisionDetector)
  {
  }

  using CollisionGroup::updateEngineData;
  using DartCollisionGroup::addCollisionObjectsToEngine;
  using DartCollisionGroup::addCollisionObjectToEngine;
  using DartCollisionGroup::initializeEngineData;
  using DartCollisionGroup::removeAllCollisionObjectsFromEngine;
  using DartCollisionGroup::removeCollisionObjectFromEngine;
  using DartCollisionGroup::updateCollisionGroupEngineData;

  std::size_t getNumObjects() const
  {
    return mCollisionObjects.size();
  }
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

SoftShapeSetup makeSoftBoxSetup(const std::string& name)
{
  SoftShapeSetup setup;
  setup.skeleton = Skeleton::create(name);
  auto pair
      = setup.skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>();
  setup.body = pair.second;
  SoftBodyNodeHelper::setBox(
      setup.body,
      Eigen::Vector3d::Constant(1.0),
      Eigen::Isometry3d::Identity(),
      Eigen::Vector3i(3, 3, 3),
      1.0,
      10.0,
      10.0,
      0.01);
  setup.shapeNode = setup.body->getShapeNode(0);
  return setup;
}

std::shared_ptr<math::TriMesh<double>> makeCubeTriMesh()
{
  auto mesh = std::make_shared<math::TriMesh<double>>();
  math::TriMesh<double>::Vertices vertices;
  vertices.emplace_back(-0.5, -0.5, -0.5);
  vertices.emplace_back(0.5, -0.5, -0.5);
  vertices.emplace_back(0.5, 0.5, -0.5);
  vertices.emplace_back(-0.5, 0.5, -0.5);
  vertices.emplace_back(-0.5, -0.5, 0.5);
  vertices.emplace_back(0.5, -0.5, 0.5);
  vertices.emplace_back(0.5, 0.5, 0.5);
  vertices.emplace_back(-0.5, 0.5, 0.5);

  math::TriMesh<double>::Triangles triangles;
  triangles.emplace_back(0, 1, 2);
  triangles.emplace_back(0, 2, 3);
  triangles.emplace_back(4, 6, 5);
  triangles.emplace_back(4, 7, 6);
  triangles.emplace_back(0, 5, 1);
  triangles.emplace_back(0, 4, 5);
  triangles.emplace_back(2, 7, 3);
  triangles.emplace_back(2, 6, 7);
  triangles.emplace_back(0, 3, 7);
  triangles.emplace_back(0, 7, 4);
  triangles.emplace_back(1, 5, 6);
  triangles.emplace_back(1, 6, 2);
  mesh->setTriangles(vertices, triangles);

  return mesh;
}

std::shared_ptr<ConvexMeshShape> makeTetraConvexMeshShape()
{
  ConvexMeshShape::Vertices vertices
      = {Eigen::Vector3d::Zero(),
         Eigen::Vector3d::UnitX(),
         Eigen::Vector3d::UnitY(),
         Eigen::Vector3d::UnitZ()};
  ConvexMeshShape::Triangles triangles
      = {ConvexMeshShape::TriMeshType::Triangle(0, 1, 2),
         ConvexMeshShape::TriMeshType::Triangle(0, 1, 3),
         ConvexMeshShape::TriMeshType::Triangle(0, 2, 3),
         ConvexMeshShape::TriMeshType::Triangle(1, 2, 3)};

  return std::make_shared<ConvexMeshShape>(vertices, triangles);
}

std::pair<bool, std::size_t> runCollision(
    const ShapePtr& shapeA,
    const ShapePtr& shapeB,
    const Eigen::Vector3d& offset)
{
  auto detector = DartCollisionDetector::create();
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

std::pair<bool, CollisionResult> runCollisionWithTransforms(
    const ShapePtr& shapeA,
    const ShapePtr& shapeB,
    const Eigen::Isometry3d& tfA,
    const Eigen::Isometry3d& tfB)
{
  auto detector = DartCollisionDetector::create();
  auto setupA = makeShapeSetup("shape_a", shapeA);
  auto setupB = makeShapeSetup("shape_b", shapeB);

  FreeJoint::setTransformOf(setupA.body, tfA);
  FreeJoint::setTransformOf(setupB.body, tfB);

  auto group = detector->createCollisionGroup();
  group->addShapeFrame(setupA.shapeNode);
  group->addShapeFrame(setupB.shapeNode);

  CollisionOption option;
  option.maxNumContacts = 10u;

  CollisionResult result;
  const bool collided = group->collide(option, &result);
  return {collided, result};
}

} // namespace

TEST(DartCollisionDetector, LegacyFactoryKeysResolveToBuiltInDetector)
{
  auto* factory = CollisionDetector::getFactory();
  ASSERT_NE(factory, nullptr);

  constexpr std::array<const char*, 6> keys{
      "dart", "experimental", "fcl", "fcl_mesh", "bullet", "ode"};

  for (const auto* key : keys) {
    ASSERT_TRUE(factory->canCreate(key)) << key;

    const auto detector = factory->create(key);
    ASSERT_NE(detector, nullptr) << key;
    EXPECT_EQ(detector->getTypeView(), DartCollisionDetector::getStaticType())
        << key;
    EXPECT_NE(dynamic_cast<DartCollisionDetector*>(detector.get()), nullptr)
        << key;

    auto group = detector->createCollisionGroup();
    ASSERT_NE(group, nullptr) << key;
    EXPECT_NE(dynamic_cast<DartCollisionGroup*>(group.get()), nullptr) << key;
  }
}

TEST(DartCollisionDetector, SupportedShapePairs)
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

TEST(DartCollisionGroup, MeshPlaneCollisionUsesBroadphase)
{
  auto detector = DartCollisionDetector::create();
  auto planeSetup = makeShapeSetup(
      "plane", std::make_shared<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0));
  auto meshSetup = makeShapeSetup(
      "mesh",
      std::make_shared<MeshShape>(Eigen::Vector3d::Ones(), makeCubeTriMesh()));

  Eigen::Isometry3d meshTf = Eigen::Isometry3d::Identity();
  meshTf.translation() = Eigen::Vector3d(0.0, 0.0, 0.25);
  FreeJoint::setTransformOf(meshSetup.body, meshTf);

  auto group = detector->createCollisionGroup();
  group->addShapeFrame(planeSetup.shapeNode);
  group->addShapeFrame(meshSetup.shapeNode);

  CollisionOption option;
  option.maxNumContacts = 10u;

  CollisionResult result;
  ASSERT_TRUE(group->collide(option, &result));
  EXPECT_GT(result.getNumContacts(), 0u);
  EXPECT_LT(result.getContact(0).normal.z(), -0.9);
}

TEST(DartCollisionDetector, SphereMeshCollisionDetectsContact)
{
  auto sphere = std::make_shared<SphereShape>(0.25);
  auto mesh
      = std::make_shared<MeshShape>(Eigen::Vector3d::Ones(), makeCubeTriMesh());

  Eigen::Isometry3d sphereTf = Eigen::Isometry3d::Identity();
  sphereTf.translation() = Eigen::Vector3d(0.0, 0.0, 0.65);
  const Eigen::Isometry3d meshTf = Eigen::Isometry3d::Identity();

  auto result = runCollisionWithTransforms(sphere, mesh, sphereTf, meshTf);
  ASSERT_TRUE(result.first);
  ASSERT_GT(result.second.getNumContacts(), 0u);
  EXPECT_GT(result.second.getContact(0).normal.norm(), 0.9);
  EXPECT_GT(result.second.getContact(0).penetrationDepth, 0.0);

  result = runCollisionWithTransforms(mesh, sphere, meshTf, sphereTf);
  ASSERT_TRUE(result.first);
  ASSERT_GT(result.second.getNumContacts(), 0u);
  EXPECT_GT(result.second.getContact(0).normal.norm(), 0.9);
  EXPECT_GT(result.second.getContact(0).penetrationDepth, 0.0);
}

TEST(DartCollisionDetector, ConvexMeshCollisionUsesNativeConvexGeometry)
{
  auto convex = makeTetraConvexMeshShape();
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(0.2));
  ASSERT_TRUE(convex->hasValidMesh());

  Eigen::Isometry3d tfConvex = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();
  tfBox.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);

  auto result = runCollisionWithTransforms(convex, box, tfConvex, tfBox);
  EXPECT_TRUE(result.first);
  EXPECT_GT(result.second.getNumContacts(), 0u);

  tfBox.translation() = Eigen::Vector3d(2.0, 0.0, 0.0);
  result = runCollisionWithTransforms(convex, box, tfConvex, tfBox);
  EXPECT_FALSE(result.first);
  EXPECT_EQ(result.second.getNumContacts(), 0u);
}

TEST(DartCollisionDetector, EmptyConvexMeshIsNotCollidable)
{
  auto convex = std::make_shared<ConvexMeshShape>(
      std::make_shared<ConvexMeshShape::TriMeshType>());
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(0.04));
  ASSERT_FALSE(convex->hasValidMesh());

  Eigen::Isometry3d tfConvex = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfBox = Eigen::Isometry3d::Identity();
  tfBox.translation() = Eigen::Vector3d(0.12, 0.0, 0.0);

  auto result = runCollisionWithTransforms(convex, box, tfConvex, tfBox);
  EXPECT_FALSE(result.first);
  EXPECT_EQ(result.second.getNumContacts(), 0u);

  tfBox.translation() = Eigen::Vector3d(0.16, 0.0, 0.0);
  result = runCollisionWithTransforms(convex, box, tfConvex, tfBox);
  EXPECT_FALSE(result.first);
  EXPECT_EQ(result.second.getNumContacts(), 0u);
}

TEST(DARTCollisionDetector, CloneWithoutCollisionObjects)
{
  auto detector = DARTCollisionDetector::create();
  auto clone = detector->cloneWithoutCollisionObjects();

  ASSERT_NE(clone, nullptr);
  EXPECT_EQ(DARTCollisionDetector::getStaticType(), clone->getTypeView());
}

TEST(DartCollisionDetector, CylinderBoxPairIsSupported)
{
  auto cylinder = std::make_shared<CylinderShape>(0.5, 1.0);
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(1.0));

  const auto result = runCollision(cylinder, box, Eigen::Vector3d::Zero());
  EXPECT_TRUE(result.first);
  EXPECT_LT(0u, result.second);
}

TEST(DartCollisionDetector, BoxBoxEdgeEdgeContact)
{
  auto box1 = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 0.2, 0.2));
  auto box2 = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 0.2, 0.2));

  Eigen::Isometry3d tfA = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfB = Eigen::Isometry3d::Identity();
  tfB.linear() = (Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitZ())
                  * Eigen::AngleAxisd(0.35, Eigen::Vector3d::UnitY()))
                     .toRotationMatrix();
  tfB.translation() = Eigen::Vector3d(0.05, -0.02, 0.03);

  auto result = runCollisionWithTransforms(box1, box2, tfA, tfB);
  ASSERT_TRUE(result.first);
  ASSERT_GT(result.second.getNumContacts(), 0u);

  const auto& contact = result.second.getContact(0);
  const Eigen::Vector3d normal = contact.normal.normalized();
  const Eigen::Matrix3d axesA = tfA.linear();
  const Eigen::Matrix3d axesB = tfB.linear();

  double maxAbsDot = 0.0;
  for (int i = 0; i < 3; ++i) {
    maxAbsDot = std::max(maxAbsDot, std::abs(normal.dot(axesA.col(i))));
    maxAbsDot = std::max(maxAbsDot, std::abs(normal.dot(axesB.col(i))));
  }
  EXPECT_LT(maxAbsDot, 0.95);
}

TEST(DartCollisionDetector, RaycastWorksForSphere)
{
  // DartCollisionDetector supports raycast via the native collision engine.
  auto detector = DartCollisionDetector::create();
  auto sphere = std::make_shared<SphereShape>(0.5);
  auto setup = makeShapeSetup("sphere", sphere);

  auto group = detector->createCollisionGroup();
  group->addShapeFrame(setup.shapeNode);

  RaycastOption option;
  RaycastResult result;

  const Eigen::Vector3d from(0.0, 0.0, 2.0);
  const Eigen::Vector3d to(0.0, 0.0, -2.0);

  const bool hit = group->raycast(from, to, option, &result);
  EXPECT_TRUE(hit);
  EXPECT_TRUE(result.hasHit());
}

TEST(DartCollisionDetector, LegacyUppercaseHeaderKeepsRaycastUnsupported)
{
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
  EXPECT_EQ("dart", detector->getTypeView());
  EXPECT_FALSE(hit);
  EXPECT_FALSE(result.hasHit());
}

TEST(DartCollisionDetector, LegacyReferenceFacadesKeepRaycastUnsupported)
{
  DART_SUPPRESS_DEPRECATED_BEGIN
  const std::array<std::shared_ptr<CollisionDetector>, 2> detectors{
      FCLCollisionDetector::create(), OdeCollisionDetector::create()};
  DART_SUPPRESS_DEPRECATED_END

  for (const auto& detector : detectors) {
    auto sphere = std::make_shared<SphereShape>(0.5);
    auto setup = makeShapeSetup("sphere", sphere);

    auto group = detector->createCollisionGroup();
    group->addShapeFrame(setup.shapeNode);

    RaycastOption option;
    RaycastResult result;

    const Eigen::Vector3d from(0.0, 0.0, 2.0);
    const Eigen::Vector3d to(0.0, 0.0, -2.0);

    const bool hit = group->raycast(from, to, option, &result);
    EXPECT_FALSE(hit) << detector->getTypeView();
    EXPECT_FALSE(result.hasHit()) << detector->getTypeView();
  }
}

TEST(DartCollisionDetector, MaxNumContactsZeroReturnsFalse)
{
  auto detector = DartCollisionDetector::create();
  auto setupA = makeShapeSetup("shape_a", std::make_shared<SphereShape>(0.5));
  auto setupB = makeShapeSetup("shape_b", std::make_shared<SphereShape>(0.5));

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.2, 0.0, 0.0);
  FreeJoint::setTransformOf(setupB.body, tf);

  auto group = detector->createCollisionGroup();
  group->addShapeFrame(setupA.shapeNode);
  group->addShapeFrame(setupB.shapeNode);

  CollisionOption option;
  option.maxNumContacts = 0u;
  CollisionResult result;

  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_EQ(result.getNumContacts(), 0u);
}

TEST(DartCollisionDetector, EmptyGroupReturnsFalse)
{
  auto detector = DartCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  CollisionOption option;
  option.maxNumContacts = 1u;
  CollisionResult result;

  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_EQ(result.getNumContacts(), 0u);
}

TEST(DartCollisionDetector, MismatchedDetectorGroupReturnsFalse)
{
  auto detectorA = DartCollisionDetector::create();
  auto detectorB = DartCollisionDetector::create();

  auto setupA = makeShapeSetup("shape_a", std::make_shared<SphereShape>(0.5));
  auto setupB = makeShapeSetup("shape_b", std::make_shared<SphereShape>(0.5));

  auto groupA = detectorA->createCollisionGroup();
  auto groupB = detectorB->createCollisionGroup();
  groupA->addShapeFrame(setupA.shapeNode);
  groupB->addShapeFrame(setupB.shapeNode);

  CollisionOption option;
  CollisionResult result;

  EXPECT_FALSE(groupA->collide(groupB.get(), option, &result));
}

TEST(DartCollisionDetector, DistanceQueriesReturnZero)
{
  auto detector = DartCollisionDetector::create();
  auto setupA = makeShapeSetup("shape_a", std::make_shared<SphereShape>(0.5));
  auto setupB = makeShapeSetup("shape_b", std::make_shared<SphereShape>(0.5));

  auto groupA = detector->createCollisionGroup();
  auto groupB = detector->createCollisionGroup();
  groupA->addShapeFrame(setupA.shapeNode);
  groupB->addShapeFrame(setupB.shapeNode);

  DistanceOption option;
  DistanceResult result;

  EXPECT_DOUBLE_EQ(groupA->distance(option, &result), 0.0);
  EXPECT_DOUBLE_EQ(groupA->distance(groupB.get(), option, &result), 0.0);
}

TEST(DartCollisionGroup, PersistentSceneTracksMovedObjects)
{
  auto detector = DartCollisionDetector::create();
  auto setupA = makeShapeSetup("shape_a", std::make_shared<SphereShape>(0.5));
  auto setupB = makeShapeSetup("shape_b", std::make_shared<SphereShape>(0.5));

  auto group = detector->createCollisionGroup();
  group->addShapeFrame(setupA.shapeNode);
  group->addShapeFrame(setupB.shapeNode);

  CollisionOption option;
  option.maxNumContacts = 10u;

  CollisionResult result;
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_LT(0u, result.getNumContacts());

  Eigen::Isometry3d separated = Eigen::Isometry3d::Identity();
  separated.translation() = Eigen::Vector3d(4.0, 0.0, 0.0);
  FreeJoint::setTransformOf(setupB.body, separated);

  result.clear();
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_EQ(0u, result.getNumContacts());

  Eigen::Isometry3d overlapping = Eigen::Isometry3d::Identity();
  overlapping.translation() = Eigen::Vector3d(0.2, 0.0, 0.0);
  FreeJoint::setTransformOf(setupB.body, overlapping);

  result.clear();
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_LT(0u, result.getNumContacts());
}

TEST(DartCollisionGroup, PersistentSceneRebuildsAfterShapeChange)
{
  auto detector = DartCollisionDetector::create();
  auto setupA = makeShapeSetup("shape_a", std::make_shared<SphereShape>(0.2));
  auto setupB = makeShapeSetup("shape_b", std::make_shared<SphereShape>(0.2));

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
  FreeJoint::setTransformOf(setupB.body, tf);

  auto group = detector->createCollisionGroup();
  group->addShapeFrame(setupA.shapeNode);
  group->addShapeFrame(setupB.shapeNode);

  CollisionOption option;
  option.maxNumContacts = 10u;

  CollisionResult result;
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_EQ(0u, result.getNumContacts());

  setupA.shapeNode->setShape(std::make_shared<SphereShape>(1.0));

  result.clear();
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_LT(0u, result.getNumContacts());
}

TEST(DartCollisionGroup, PersistentSceneRebuildsAfterShapeMutation)
{
  auto detector = DartCollisionDetector::create();
  auto sphereA = std::make_shared<SphereShape>(0.2);
  auto setupA = makeShapeSetup("shape_a", sphereA);
  auto setupB = makeShapeSetup("shape_b", std::make_shared<SphereShape>(0.2));

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
  FreeJoint::setTransformOf(setupB.body, tf);

  auto group = detector->createCollisionGroup();
  group->addShapeFrame(setupA.shapeNode);
  group->addShapeFrame(setupB.shapeNode);

  CollisionOption option;
  option.maxNumContacts = 10u;

  CollisionResult result;
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_EQ(0u, result.getNumContacts());

  sphereA->setRadius(1.0);

  result.clear();
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_LT(0u, result.getNumContacts());
}

TEST(DartCollisionGroup, PersistentSceneRebuildsAfterMeshScaleMutation)
{
  auto detector = DartCollisionDetector::create();
  auto meshShape = std::make_shared<MeshShape>(
      Eigen::Vector3d::Constant(0.2), makeCubeTriMesh());
  auto setupA = makeShapeSetup("mesh", meshShape);
  auto setupB = makeShapeSetup("sphere", std::make_shared<SphereShape>(0.25));

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.9, 0.0, 0.0);
  FreeJoint::setTransformOf(setupB.body, tf);

  auto group = detector->createCollisionGroup();
  group->addShapeFrame(setupA.shapeNode);
  group->addShapeFrame(setupB.shapeNode);

  CollisionOption option;
  option.maxNumContacts = 10u;

  CollisionResult result;
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_EQ(0u, result.getNumContacts());

  meshShape->setScale(Eigen::Vector3d::Constant(2.0));

  result.clear();
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_LT(0u, result.getNumContacts());
}

TEST(DartCollisionGroup, PersistentSceneRebuildsAfterHeightmapMutation)
{
  auto detector = DartCollisionDetector::create();
  auto heightmap = std::make_shared<HeightmapShaped>();
  HeightmapShaped::HeightField heights(2, 2);
  heights.setZero();
  heightmap->setHeightField(heights);

  auto setupA = makeShapeSetup("heightmap", heightmap);
  auto setupB = makeShapeSetup("sphere", std::make_shared<SphereShape>(0.25));

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.0, 0.0, 1.2);
  FreeJoint::setTransformOf(setupB.body, tf);

  auto group = detector->createCollisionGroup();
  group->addShapeFrame(setupA.shapeNode);
  group->addShapeFrame(setupB.shapeNode);

  CollisionOption option;
  option.maxNumContacts = 10u;

  CollisionResult result;
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_EQ(0u, result.getNumContacts());

  heights.setConstant(1.1);
  heightmap->setHeightField(heights);

  result.clear();
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_LT(0u, result.getNumContacts());
}

#if DART_HAVE_OCTOMAP
TEST(DartCollisionGroup, PersistentSceneRebuildsAfterVoxelGridMutation)
{
  auto detector = DartCollisionDetector::create();
  auto voxelGrid = std::make_shared<VoxelGridShape>(0.5);
  auto setupA = makeShapeSetup("voxel", voxelGrid);
  auto setupB = makeShapeSetup("sphere", std::make_shared<SphereShape>(0.1));

  auto group = detector->createCollisionGroup();
  group->addShapeFrame(setupA.shapeNode);
  group->addShapeFrame(setupB.shapeNode);

  CollisionOption option;
  option.maxNumContacts = 10u;

  CollisionResult result;
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_EQ(0u, result.getNumContacts());

  voxelGrid->updateOccupancy(Eigen::Vector3d::Zero(), true);

  result.clear();
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_LT(0u, result.getNumContacts());
}
#endif

TEST(DartCollisionGroup, PointCloudShapeIsExplicitlyNonCollidable)
{
  auto detector = DartCollisionDetector::create();
  auto pointCloud = std::make_shared<PointCloudShape>(1.0);
  pointCloud->addPoint(Eigen::Vector3d::Zero());
  auto setupA = makeShapeSetup("point_cloud", pointCloud);
  auto setupB = makeShapeSetup("sphere", std::make_shared<SphereShape>(0.5));

  auto group = detector->createCollisionGroup();
  group->addShapeFrame(setupA.shapeNode);
  group->addShapeFrame(setupB.shapeNode);

  CollisionOption option;
  option.maxNumContacts = 10u;

  CollisionResult result;
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_EQ(0u, result.getNumContacts());

  pointCloud->addPoint(Eigen::Vector3d(0.25, 0.0, 0.0));

  result.clear();
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_EQ(0u, result.getNumContacts());
}

TEST(DartCollisionGroup, PersistentSceneRebuildsAfterSoftMeshMutation)
{
  auto detector = DartCollisionDetector::create();
  auto softSetup = makeSoftBoxSetup("soft");
  auto boxSetup = makeShapeSetup(
      "box", std::make_shared<BoxShape>(Eigen::Vector3d::Constant(2.0)));

  ASSERT_NE(nullptr, softSetup.shapeNode);
  ASSERT_NE(nullptr, softSetup.shapeNode->getShape()->as<SoftMeshShape>());

  auto group = detector->createCollisionGroup();
  group->addShapeFrame(softSetup.shapeNode);
  group->addShapeFrame(boxSetup.shapeNode);

  CollisionOption option;
  option.maxNumContacts = 10u;

  CollisionResult result;
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_LT(0u, result.getNumContacts());

  for (std::size_t i = 0; i < softSetup.body->getNumPointMasses(); ++i) {
    softSetup.body->getPointMass(i)->setPositions(
        Eigen::Vector3d(3.0, 0.0, 0.0));
  }

  result.clear();
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_EQ(0u, result.getNumContacts());
}

TEST(DartCollisionGroup, PersistentSceneShapeChangeInvalidatesWarmStartCache)
{
  auto detector = DartCollisionDetector::create();
  auto setupA = makeShapeSetup("shape_a", std::make_shared<SphereShape>(0.5));
  auto setupB = makeShapeSetup("shape_b", std::make_shared<SphereShape>(0.5));

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.75, 0.0, 0.0);
  FreeJoint::setTransformOf(setupB.body, tf);

  auto group = detector->createCollisionGroup();
  group->addShapeFrame(setupA.shapeNode);
  group->addShapeFrame(setupB.shapeNode);

  CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 10u;

  CollisionResult first;
  ASSERT_TRUE(group->collide(option, &first));
  ASSERT_GT(first.getNumContacts(), 0u);

  auto& firstContact = first.getContact(0);
  ASSERT_NE(nullptr, firstContact.userData);
  auto* cached = static_cast<native::CachedContact*>(firstContact.userData);
  cached->cachedNormalImpulse = 3.0;
  cached->cachedFrictionImpulse1 = -1.0;
  cached->cachedFrictionImpulse2 = 0.5;

  setupA.shapeNode->setShape(std::make_shared<SphereShape>(0.75));

  CollisionResult second;
  ASSERT_TRUE(group->collide(option, &second));
  ASSERT_GT(second.getNumContacts(), 0u);

  const auto& secondContact = second.getContact(0);
  EXPECT_DOUBLE_EQ(0.0, secondContact.cachedNormalImpulse);
  EXPECT_DOUBLE_EQ(0.0, secondContact.cachedFrictionImpulse1);
  EXPECT_DOUBLE_EQ(0.0, secondContact.cachedFrictionImpulse2);
}

TEST(DartCollisionGroup, PersistentSceneDropsRemovedObjects)
{
  auto detector = DartCollisionDetector::create();
  auto setupA = makeShapeSetup("shape_a", std::make_shared<SphereShape>(0.5));
  auto setupB = makeShapeSetup("shape_b", std::make_shared<SphereShape>(0.5));

  auto group = detector->createCollisionGroup();
  group->addShapeFrame(setupA.shapeNode);
  group->addShapeFrame(setupB.shapeNode);

  CollisionOption option;
  option.maxNumContacts = 10u;

  CollisionResult result;
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_LT(0u, result.getNumContacts());

  group->removeShapeFrame(setupB.shapeNode);

  result.clear();
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_EQ(0u, result.getNumContacts());
}

TEST(DartCollisionGroup, PersistentSceneRaycastTracksMovedObjects)
{
  auto detector = DartCollisionDetector::create();
  auto setup = makeShapeSetup("shape", std::make_shared<SphereShape>(0.5));

  auto group = detector->createCollisionGroup();
  group->addShapeFrame(setup.shapeNode);

  RaycastOption option;
  RaycastResult result;
  const Eigen::Vector3d from(0.0, 0.0, 2.0);
  const Eigen::Vector3d to(0.0, 0.0, -2.0);

  EXPECT_TRUE(group->raycast(from, to, option, &result));
  EXPECT_TRUE(result.hasHit());

  Eigen::Isometry3d moved = Eigen::Isometry3d::Identity();
  moved.translation() = Eigen::Vector3d(5.0, 0.0, 0.0);
  FreeJoint::setTransformOf(setup.body, moved);

  result.clear();
  EXPECT_FALSE(group->raycast(from, to, option, &result));
  EXPECT_FALSE(result.hasHit());
}

TEST(DartCollisionGroup, EngineDataCallbacks)
{
  auto detector = TestDartCollisionDetector::create();
  TestDartCollisionGroup group(detector);

  auto setupA = makeShapeSetup("shape_a", std::make_shared<SphereShape>(0.5));
  auto setupB = makeShapeSetup("shape_b", std::make_shared<SphereShape>(0.5));

  auto objectA = detector->claimCollisionObject(setupA.shapeNode);
  auto objectB = detector->claimCollisionObject(setupB.shapeNode);

  group.addShapeFrame(setupA.shapeNode);
  ASSERT_EQ(group.getNumObjects(), 1u);
  group.updateEngineData();

  group.removeAllShapeFrames();
  ASSERT_EQ(group.getNumObjects(), 0u);

  group.initializeEngineData();
  group.addCollisionObjectToEngine(objectA.get());
  EXPECT_EQ(group.getNumObjects(), 1u);

  auto objects
      = std::to_array<CollisionObject*>({objectA.get(), objectB.get()});
  group.addCollisionObjectsToEngine(objects);
  EXPECT_EQ(group.getNumObjects(), 2u);

  group.updateEngineData();
  group.updateCollisionGroupEngineData();
  group.removeCollisionObjectFromEngine(objectA.get());
  EXPECT_EQ(group.getNumObjects(), 1u);

  group.removeAllCollisionObjectsFromEngine();
  EXPECT_EQ(group.getNumObjects(), 0u);
}
