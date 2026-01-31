#include <dart/config.hpp>

#include <gtest/gtest.h>

#include <array>

#if DART_HAVE_BULLET

  #include "dart/collision/bullet/bullet_collision_detector.hpp"
  #include "dart/collision/collision_filter.hpp"
  #include "dart/collision/collision_group.hpp"
  #include "dart/collision/collision_object.hpp"
  #include "dart/collision/collision_option.hpp"
  #include "dart/collision/collision_result.hpp"
  #include "dart/collision/distance_option.hpp"
  #include "dart/collision/distance_result.hpp"
  #include "dart/collision/raycast_option.hpp"
  #include "dart/collision/raycast_result.hpp"
  #include "dart/dynamics/box_shape.hpp"
  #include "dart/dynamics/capsule_shape.hpp"
  #include "dart/dynamics/cone_shape.hpp"
  #include "dart/dynamics/cylinder_shape.hpp"
  #include "dart/dynamics/ellipsoid_shape.hpp"
  #include "dart/dynamics/mesh_shape.hpp"
  #include "dart/dynamics/plane_shape.hpp"
  #include "dart/dynamics/simple_frame.hpp"
  #include "dart/dynamics/sphere_shape.hpp"
  #include "dart/math/tri_mesh.hpp"

using dart::collision::BulletCollisionDetector;
using dart::collision::CollisionFilter;
using dart::collision::CollisionGroup;
using dart::collision::CollisionObject;
using dart::collision::CollisionOption;
using dart::collision::CollisionResult;
using dart::collision::DistanceOption;
using dart::collision::DistanceResult;
using dart::collision::RaycastOption;
using dart::collision::RaycastResult;
using dart::dynamics::BoxShape;
using dart::dynamics::CapsuleShape;
using dart::dynamics::ConeShape;
using dart::dynamics::CylinderShape;
using dart::dynamics::EllipsoidShape;
using dart::dynamics::MeshShape;
using dart::dynamics::PlaneShape;
using dart::dynamics::ShapeFrame;
using dart::dynamics::SimpleFrame;
using dart::dynamics::SphereShape;
using dart::math::TriMeshd;

namespace {

class ShapePairFilter final : public CollisionFilter
{
public:
  ShapePairFilter(const ShapeFrame* first, const ShapeFrame* second)
    : mFirst(first), mSecond(second)
  {
  }

  bool ignoresCollision(
      const CollisionObject* object1,
      const CollisionObject* object2) const override
  {
    if (!object1 || !object2) {
      return false;
    }

    const auto* frame1 = object1->getShapeFrame();
    const auto* frame2 = object2->getShapeFrame();

    if ((frame1 == mFirst && frame2 == mSecond)
        || (frame1 == mSecond && frame2 == mFirst)) {
      return true;
    }

    return false;
  }

private:
  const ShapeFrame* mFirst;
  const ShapeFrame* mSecond;
};

std::shared_ptr<TriMeshd> makeSimpleTriMesh()
{
  auto mesh = std::make_shared<TriMeshd>();

  TriMeshd::Vertices vertices;
  vertices.emplace_back(0.0, 0.0, 0.0);
  vertices.emplace_back(1.0, 0.0, 0.0);
  vertices.emplace_back(0.0, 1.0, 0.0);
  vertices.emplace_back(0.0, 0.0, 1.0);

  TriMeshd::Triangles triangles;
  triangles.emplace_back(0, 1, 2);
  triangles.emplace_back(0, 1, 3);

  mesh->setTriangles(vertices, triangles);
  return mesh;
}

} // namespace

TEST(BulletCollisionCoverage, CollideWithShapesAndFilter)
{
  auto detector = BulletCollisionDetector::create();
  ASSERT_TRUE(detector);

  auto planeFrame = SimpleFrame::createShared(dart::dynamics::Frame::World());
  auto sphereFrame = SimpleFrame::createShared(dart::dynamics::Frame::World());
  auto boxFrame = SimpleFrame::createShared(dart::dynamics::Frame::World());
  auto capsuleFrame = SimpleFrame::createShared(dart::dynamics::Frame::World());
  auto cylinderFrame
      = SimpleFrame::createShared(dart::dynamics::Frame::World());
  auto coneFrame = SimpleFrame::createShared(dart::dynamics::Frame::World());
  auto ellipsoidFrame
      = SimpleFrame::createShared(dart::dynamics::Frame::World());
  auto meshFrame = SimpleFrame::createShared(dart::dynamics::Frame::World());

  planeFrame->setShape(
      std::make_shared<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0));
  sphereFrame->setShape(std::make_shared<SphereShape>(0.1));
  boxFrame->setShape(
      std::make_shared<BoxShape>(Eigen::Vector3d(0.2, 0.2, 0.2)));
  capsuleFrame->setShape(std::make_shared<CapsuleShape>(0.05, 0.2));
  cylinderFrame->setShape(std::make_shared<CylinderShape>(0.05, 0.2));
  coneFrame->setShape(std::make_shared<ConeShape>(0.05, 0.2));
  ellipsoidFrame->setShape(
      std::make_shared<EllipsoidShape>(Eigen::Vector3d(0.08, 0.12, 0.1)));
  meshFrame->setShape(std::make_shared<MeshShape>(
      Eigen::Vector3d::Ones(), makeSimpleTriMesh()));

  planeFrame->setTranslation(Eigen::Vector3d::Zero());
  sphereFrame->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.05));
  boxFrame->setTranslation(Eigen::Vector3d(2.0, 0.0, 2.0));
  capsuleFrame->setTranslation(Eigen::Vector3d(-2.0, 0.0, 2.0));
  cylinderFrame->setTranslation(Eigen::Vector3d(0.0, 2.0, 2.0));
  coneFrame->setTranslation(Eigen::Vector3d(0.0, -2.0, 2.0));
  ellipsoidFrame->setTranslation(Eigen::Vector3d(2.0, 2.0, 2.0));
  meshFrame->setTranslation(Eigen::Vector3d(-2.0, -2.0, 2.0));

  auto group = detector->createCollisionGroup();
  group->addShapeFrame(planeFrame.get());
  const std::array<const ShapeFrame*, 2> initialFrames{
      sphereFrame.get(), boxFrame.get()};
  group->addShapeFrames(initialFrames);
  group->addShapeFrame(capsuleFrame.get());
  group->addShapeFrame(cylinderFrame.get());
  group->addShapeFrame(coneFrame.get());
  group->addShapeFrame(ellipsoidFrame.get());
  group->addShapeFrame(meshFrame.get());

  group->removeShapeFrame(cylinderFrame.get());
  group->addShapeFrame(cylinderFrame.get());

  CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 1u;

  CollisionResult result;
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_GE(result.getNumContacts(), 1u);

  auto filter
      = std::make_shared<ShapePairFilter>(planeFrame.get(), sphereFrame.get());
  option.collisionFilter = filter;
  result.clear();
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_EQ(result.getNumContacts(), 0u);

  group->removeAllShapeFrames();
  EXPECT_EQ(group->getNumShapeFrames(), 0u);
}

TEST(BulletCollisionCoverage, DistanceAndRaycastPaths)
{
  auto detector = BulletCollisionDetector::create();
  ASSERT_TRUE(detector);

  auto boxFrame1 = SimpleFrame::createShared(dart::dynamics::Frame::World());
  auto boxFrame2 = SimpleFrame::createShared(dart::dynamics::Frame::World());
  boxFrame1->setShape(
      std::make_shared<BoxShape>(Eigen::Vector3d(0.2, 0.2, 0.2)));
  boxFrame2->setShape(
      std::make_shared<BoxShape>(Eigen::Vector3d(0.2, 0.2, 0.2)));
  boxFrame1->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.0));
  boxFrame2->setTranslation(Eigen::Vector3d(0.0, 0.0, 1.0));

  auto group = detector->createCollisionGroup(boxFrame1.get(), boxFrame2.get());
  ASSERT_EQ(group->getNumShapeFrames(), 2u);

  DistanceOption distanceOption;
  DistanceResult distanceResult;
  EXPECT_DOUBLE_EQ(group->distance(distanceOption, &distanceResult), 0.0);

  auto otherGroup = detector->createCollisionGroup(boxFrame2.get());
  EXPECT_DOUBLE_EQ(
      group->distance(otherGroup.get(), distanceOption, &distanceResult), 0.0);

  RaycastOption rayAllHits;
  rayAllHits.mEnableAllHits = true;
  rayAllHits.mSortByClosest = true;
  RaycastResult rayResult;
  EXPECT_TRUE(group->raycast(
      Eigen::Vector3d(0.0, 0.0, -2.0),
      Eigen::Vector3d(0.0, 0.0, 3.0),
      rayAllHits,
      &rayResult));
  EXPECT_GE(rayResult.mRayHits.size(), 1u);

  const auto* excludeFrame = boxFrame2.get();
  rayAllHits.mFilter = RaycastOption::RaycastFilter(
      [excludeFrame](const CollisionObject* object) -> bool {
        return object && object->getShapeFrame() != excludeFrame;
      });
  rayResult.clear();
  EXPECT_TRUE(group->raycast(
      Eigen::Vector3d(0.0, 0.0, -2.0),
      Eigen::Vector3d(0.0, 0.0, 3.0),
      rayAllHits,
      &rayResult));
  EXPECT_EQ(rayResult.mRayHits.size(), 1u);

  RaycastOption rayClosest;
  rayClosest.mEnableAllHits = false;
  rayClosest.mSortByClosest = true;
  RaycastResult closestResult;
  EXPECT_TRUE(group->raycast(
      Eigen::Vector3d(0.0, 0.0, -2.0),
      Eigen::Vector3d(0.0, 0.0, 3.0),
      rayClosest,
      &closestResult));
  EXPECT_EQ(closestResult.mRayHits.size(), 1u);

  auto otherDetector = BulletCollisionDetector::create();
  auto foreignGroup = otherDetector->createCollisionGroup(boxFrame1.get());
  CollisionOption option;
  EXPECT_FALSE(detector->collide(foreignGroup.get(), option, nullptr));
  EXPECT_FALSE(detector->raycast(
      foreignGroup.get(),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d(0.0, 0.0, 1.0),
      RaycastOption(),
      nullptr));
}

TEST(BulletCollisionCoverage, TwoGroupCollide)
{
  auto detector = BulletCollisionDetector::create();
  ASSERT_TRUE(detector);

  auto frame1 = SimpleFrame::createShared(dart::dynamics::Frame::World());
  auto frame2 = SimpleFrame::createShared(dart::dynamics::Frame::World());
  frame1->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0)));
  frame2->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0)));
  frame1->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.0));
  frame2->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.5));

  auto group1 = detector->createCollisionGroup(frame1.get());
  auto group2 = detector->createCollisionGroup(frame2.get());

  CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 10u;
  CollisionResult result;
  EXPECT_TRUE(detector->collide(group1.get(), group2.get(), option, &result));
  EXPECT_GE(result.getNumContacts(), 1u);

  EXPECT_TRUE(detector->collide(group1.get(), group2.get(), option, nullptr));

  auto combined = detector->createCollisionGroup(frame1.get(), frame2.get());
  EXPECT_TRUE(detector->collide(combined.get(), option, nullptr));
}

TEST(BulletCollisionCoverage, MaxNumContactsZero)
{
  auto detector = BulletCollisionDetector::create();
  ASSERT_TRUE(detector);

  auto frame1 = SimpleFrame::createShared(dart::dynamics::Frame::World());
  frame1->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(0.2, 0.2, 0.2)));

  auto group = detector->createCollisionGroup(frame1.get());

  CollisionOption option;
  option.maxNumContacts = 0u;
  CollisionResult result;
  EXPECT_FALSE(group->collide(option, &result));

  auto group2 = detector->createCollisionGroup(frame1.get());
  EXPECT_FALSE(detector->collide(group.get(), group2.get(), option, &result));
}

TEST(BulletCollisionCoverage, CloneWithoutCollisionObjects)
{
  auto detector = BulletCollisionDetector::create();
  ASSERT_TRUE(detector);

  auto cloned = detector->cloneWithoutCollisionObjects();
  ASSERT_NE(cloned, nullptr);
  EXPECT_EQ(
      BulletCollisionDetector::getStaticType(),
      BulletCollisionDetector::getStaticType());
}

TEST(BulletCollisionCoverage, RaycastNullResultWithFilter)
{
  auto detector = BulletCollisionDetector::create();
  ASSERT_TRUE(detector);

  auto frame1 = SimpleFrame::createShared(dart::dynamics::Frame::World());
  auto frame2 = SimpleFrame::createShared(dart::dynamics::Frame::World());
  frame1->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0)));
  frame2->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0)));
  frame1->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.0));
  frame2->setTranslation(Eigen::Vector3d(0.0, 0.0, 5.0));

  auto group = detector->createCollisionGroup(frame1.get(), frame2.get());

  RaycastOption allHitsOpt;
  allHitsOpt.mEnableAllHits = true;
  allHitsOpt.mFilter = RaycastOption::RaycastFilter(
      [](const CollisionObject*) -> bool { return true; });

  EXPECT_TRUE(group->raycast(
      Eigen::Vector3d(0.0, 0.0, -2.0),
      Eigen::Vector3d(0.0, 0.0, 8.0),
      allHitsOpt,
      nullptr));

  allHitsOpt.mFilter = RaycastOption::RaycastFilter(
      [](const CollisionObject*) -> bool { return false; });
  EXPECT_FALSE(group->raycast(
      Eigen::Vector3d(0.0, 0.0, -2.0),
      Eigen::Vector3d(0.0, 0.0, 8.0),
      allHitsOpt,
      nullptr));

  RaycastOption closestOpt;
  closestOpt.mEnableAllHits = false;
  EXPECT_TRUE(group->raycast(
      Eigen::Vector3d(0.0, 0.0, -2.0),
      Eigen::Vector3d(0.0, 0.0, 8.0),
      closestOpt,
      nullptr));

  RaycastResult missResult;
  EXPECT_FALSE(group->raycast(
      Eigen::Vector3d(100.0, 100.0, -2.0),
      Eigen::Vector3d(100.0, 100.0, 8.0),
      closestOpt,
      &missResult));
  EXPECT_EQ(missResult.mRayHits.size(), 0u);

  EXPECT_FALSE(group->raycast(
      Eigen::Vector3d(100.0, 100.0, -2.0),
      Eigen::Vector3d(100.0, 100.0, 8.0),
      allHitsOpt,
      &missResult));

  allHitsOpt.mFilter = nullptr;
  EXPECT_FALSE(group->raycast(
      Eigen::Vector3d(100.0, 100.0, -2.0),
      Eigen::Vector3d(100.0, 100.0, 8.0),
      allHitsOpt,
      nullptr));
}

TEST(BulletCollisionCoverage, RaycastFilterOnlyClosest)
{
  auto detector = BulletCollisionDetector::create();
  ASSERT_TRUE(detector);

  auto frame1 = SimpleFrame::createShared(dart::dynamics::Frame::World());
  auto frame2 = SimpleFrame::createShared(dart::dynamics::Frame::World());
  frame1->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0)));
  frame2->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0)));
  frame1->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.0));
  frame2->setTranslation(Eigen::Vector3d(0.0, 0.0, 3.0));

  auto group = detector->createCollisionGroup(frame1.get(), frame2.get());

  const auto* exclude = frame2.get();
  RaycastOption opt;
  opt.mEnableAllHits = false;
  opt.mSortByClosest = false;
  opt.mFilter = RaycastOption::RaycastFilter(
      [exclude](const CollisionObject* object) -> bool {
        return object && object->getShapeFrame() != exclude;
      });

  RaycastResult result;
  EXPECT_TRUE(group->raycast(
      Eigen::Vector3d(0.0, 0.0, -2.0),
      Eigen::Vector3d(0.0, 0.0, 8.0),
      opt,
      &result));
  EXPECT_EQ(result.mRayHits.size(), 1u);
}

TEST(BulletCollisionCoverage, TwoGroupCollideWithFilter)
{
  auto detector = BulletCollisionDetector::create();
  ASSERT_TRUE(detector);

  auto frame1 = SimpleFrame::createShared(dart::dynamics::Frame::World());
  auto frame2 = SimpleFrame::createShared(dart::dynamics::Frame::World());
  frame1->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0)));
  frame2->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0)));
  frame1->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.0));
  frame2->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.5));

  auto group1 = detector->createCollisionGroup(frame1.get());
  auto group2 = detector->createCollisionGroup(frame2.get());

  auto filter = std::make_shared<ShapePairFilter>(frame1.get(), frame2.get());
  CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 10u;
  option.collisionFilter = filter;

  CollisionResult result;
  EXPECT_FALSE(detector->collide(group1.get(), group2.get(), option, &result));
  EXPECT_EQ(result.getNumContacts(), 0u);
}

TEST(BulletCollisionCoverage, TwoGroupForeignDetector)
{
  auto detector1 = BulletCollisionDetector::create();
  auto detector2 = BulletCollisionDetector::create();
  ASSERT_TRUE(detector1);
  ASSERT_TRUE(detector2);

  auto frame1 = SimpleFrame::createShared(dart::dynamics::Frame::World());
  frame1->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(0.2, 0.2, 0.2)));

  auto group1 = detector1->createCollisionGroup(frame1.get());
  auto group2 = detector2->createCollisionGroup(frame1.get());

  CollisionOption option;
  EXPECT_FALSE(detector1->collide(group1.get(), group2.get(), option, nullptr));
}

#endif // DART_HAVE_BULLET
