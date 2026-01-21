// Copyright (c) 2011-2025, The DART development contributors

#include <dart/collision/CollisionObject.hpp>
#include <dart/collision/CollisionResult.hpp>
#include <dart/collision/dart/DARTCollide.hpp>
#include <dart/collision/dart/DARTCollisionDetector.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/SphereShape.hpp>
#include <dart/math/Constants.hpp>

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cmath>
#include <memory>

using namespace dart;
using namespace dart::collision;
using namespace dart::dynamics;

namespace dart::collision {
using dVector3 = double[4];
using dMatrix3 = double[12];

void cullPoints(int n, double p[], int m, int i0, int iret[]);
void dLineClosestApproach(
    const dVector3 pa,
    const dVector3 ua,
    const dVector3 pb,
    const dVector3 ub,
    double* alpha,
    double* beta);
int intersectRectQuad(double h[2], double p[8], double ret[16]);
void dClosestLineBoxPoints(
    const dVector3 p1,
    const dVector3 p2,
    const dVector3 c,
    const dMatrix3 R,
    const dVector3 side,
    dVector3 lret,
    dVector3 bret);
} // namespace dart::collision

namespace dart::test {
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
    const ShapePtr& shape,
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

TEST(DARTCollide, SphereSphereCases)
{
  auto detector = DARTCollisionDetector::create();
  auto sphere = std::make_shared<SphereShape>(0.5);

  auto objA = makeObject(
      sphere, detector.get(), Eigen::Isometry3d::Identity());

  Eigen::Isometry3d tfB = Eigen::Isometry3d::Identity();
  tfB.translation() = Eigen::Vector3d(0.4, 0.0, 0.0);
  auto objB = makeObject(sphere, detector.get(), tfB);

  CollisionResult result;
  int contacts = collideSphereSphere(
      objA.object.get(),
      objB.object.get(),
      sphere->getRadius(),
      objA.frame->getWorldTransform(),
      sphere->getRadius(),
      objB.frame->getWorldTransform(),
      result);
  EXPECT_GT(contacts, 0);
  EXPECT_GT(result.getNumContacts(), 0u);

  tfB.translation() = Eigen::Vector3d::Zero();
  objB.frame->setRelativeTransform(tfB);
  CollisionResult overlapResult;
  contacts = collideSphereSphere(
      objA.object.get(),
      objB.object.get(),
      sphere->getRadius(),
      objA.frame->getWorldTransform(),
      sphere->getRadius(),
      objB.frame->getWorldTransform(),
      overlapResult);
  EXPECT_GT(contacts, 0);
  ASSERT_GT(overlapResult.getNumContacts(), 0u);
  EXPECT_NEAR(overlapResult.getContact(0).penetrationDepth, 1.0, 1e-12);

  tfB.translation() = Eigen::Vector3d(2.0, 0.0, 0.0);
  objB.frame->setRelativeTransform(tfB);
  CollisionResult sepResult;
  contacts = collideSphereSphere(
      objA.object.get(),
      objB.object.get(),
      sphere->getRadius(),
      objA.frame->getWorldTransform(),
      sphere->getRadius(),
      objB.frame->getWorldTransform(),
      sepResult);
  EXPECT_EQ(contacts, 0);
  EXPECT_EQ(sepResult.getNumContacts(), 0u);
}

TEST(DARTCollide, BoxSphereContacts)
{
  auto detector = DARTCollisionDetector::create();
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(1.0));
  auto sphere = std::make_shared<SphereShape>(0.4);

  auto boxObj = makeObject(
      box, detector.get(), Eigen::Isometry3d::Identity());

  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(0.6, 0.0, 0.0);
  auto sphereObj = makeObject(sphere, detector.get(), tfSphere);

  CollisionResult result;
  int contacts = collideBoxSphere(
      boxObj.object.get(),
      sphereObj.object.get(),
      box->getSize(),
      boxObj.frame->getWorldTransform(),
      sphere->getRadius(),
      sphereObj.frame->getWorldTransform(),
      result);
  EXPECT_GT(contacts, 0);
  EXPECT_GT(result.getNumContacts(), 0u);

  CollisionResult flipped;
  contacts = collideSphereBox(
      sphereObj.object.get(),
      boxObj.object.get(),
      sphere->getRadius(),
      sphereObj.frame->getWorldTransform(),
      box->getSize(),
      boxObj.frame->getWorldTransform(),
      flipped);
  EXPECT_EQ(contacts, 1);
  EXPECT_GT(flipped.getNumContacts(), 0u);

  tfSphere.translation() = Eigen::Vector3d(3.0, 0.0, 0.0);
  sphereObj.frame->setRelativeTransform(tfSphere);
  CollisionResult separated;
  contacts = collideBoxSphere(
      boxObj.object.get(),
      sphereObj.object.get(),
      box->getSize(),
      boxObj.frame->getWorldTransform(),
      sphere->getRadius(),
      sphereObj.frame->getWorldTransform(),
      separated);
  EXPECT_EQ(contacts, 0);
  EXPECT_EQ(separated.getNumContacts(), 0u);
}

TEST(DARTCollide, BoxSphereInsideAndBoundary)
{
  constexpr double kNearBoundaryEps = 1e-8;
  auto detector = DARTCollisionDetector::create();
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(1.0));
  auto sphere = std::make_shared<SphereShape>(0.2);

  auto boxObj = makeObject(
      box, detector.get(), Eigen::Isometry3d::Identity());
  auto sphereObj = makeObject(
      sphere, detector.get(), Eigen::Isometry3d::Identity());

  CollisionResult inside;
  int contacts = collideBoxSphere(
      boxObj.object.get(),
      sphereObj.object.get(),
      box->getSize(),
      boxObj.frame->getWorldTransform(),
      sphere->getRadius(),
      sphereObj.frame->getWorldTransform(),
      inside);
  EXPECT_EQ(contacts, 1);
  EXPECT_EQ(inside.getNumContacts(), 1u);

  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(0.5 + kNearBoundaryEps, 0.0, 0.0);
  sphereObj.frame->setRelativeTransform(tfSphere);

  CollisionResult nearBoundary;
  contacts = collideBoxSphere(
      boxObj.object.get(),
      sphereObj.object.get(),
      box->getSize(),
      boxObj.frame->getWorldTransform(),
      sphere->getRadius(),
      sphereObj.frame->getWorldTransform(),
      nearBoundary);
  EXPECT_EQ(contacts, 1);
  EXPECT_EQ(nearBoundary.getNumContacts(), 1u);
}

TEST(DARTCollide, SphereBoxInsideAndBoundary)
{
  constexpr double kNearBoundaryEps = 1e-8;
  auto detector = DARTCollisionDetector::create();
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(1.0));
  auto sphere = std::make_shared<SphereShape>(0.2);

  auto boxObj = makeObject(
      box, detector.get(), Eigen::Isometry3d::Identity());
  auto sphereObj = makeObject(
      sphere, detector.get(), Eigen::Isometry3d::Identity());

  CollisionResult inside;
  int contacts = collideSphereBox(
      sphereObj.object.get(),
      boxObj.object.get(),
      sphere->getRadius(),
      sphereObj.frame->getWorldTransform(),
      box->getSize(),
      boxObj.frame->getWorldTransform(),
      inside);
  EXPECT_EQ(contacts, 1);
  EXPECT_EQ(inside.getNumContacts(), 1u);

  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(0.5 + kNearBoundaryEps, 0.0, 0.0);
  sphereObj.frame->setRelativeTransform(tfSphere);

  CollisionResult nearBoundary;
  contacts = collideSphereBox(
      sphereObj.object.get(),
      boxObj.object.get(),
      sphere->getRadius(),
      sphereObj.frame->getWorldTransform(),
      box->getSize(),
      boxObj.frame->getWorldTransform(),
      nearBoundary);
  EXPECT_EQ(contacts, 1);
  EXPECT_EQ(nearBoundary.getNumContacts(), 1u);
}

TEST(DARTCollide, BoxBoxContacts)
{
  auto detector = DARTCollisionDetector::create();
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(1.0));

  Eigen::Isometry3d tfA = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfB = Eigen::Isometry3d::Identity();
  tfB.translation() = Eigen::Vector3d(0.4, 0.0, 0.0);
  tfB.linear() = Eigen::AngleAxisd(0.25 * math::pi, Eigen::Vector3d::UnitZ())
                    .toRotationMatrix();

  auto objA = makeObject(box, detector.get(), tfA);
  auto objB = makeObject(box, detector.get(), tfB);

  CollisionResult result;
  int contacts = collideBoxBox(
      objA.object.get(),
      objB.object.get(),
      box->getSize(),
      objA.frame->getWorldTransform(),
      box->getSize(),
      objB.frame->getWorldTransform(),
      result);
  EXPECT_GT(contacts, 0);
  EXPECT_GT(result.getNumContacts(), 0u);

  tfB.translation() = Eigen::Vector3d(5.0, 0.0, 0.0);
  objB.frame->setRelativeTransform(tfB);
  CollisionResult separated;
  contacts = collideBoxBox(
      objA.object.get(),
      objB.object.get(),
      box->getSize(),
      objA.frame->getWorldTransform(),
      box->getSize(),
      objB.frame->getWorldTransform(),
      separated);
  EXPECT_EQ(contacts, 0);
  EXPECT_EQ(separated.getNumContacts(), 0u);
}

TEST(DARTCollide, CylinderSphereAndPlane)
{
  auto detector = DARTCollisionDetector::create();
  auto shape = std::make_shared<SphereShape>(0.1);

  Eigen::Isometry3d tfCyl = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(0.2, 0.0, 0.9);

  auto cylObj = makeObject(shape, detector.get(), tfCyl);
  auto sphereObj = makeObject(shape, detector.get(), tfSphere);

  CollisionResult result;
  int contacts = collideCylinderSphere(
      cylObj.object.get(),
      sphereObj.object.get(),
      0.5,
      1.0,
      cylObj.frame->getWorldTransform(),
      0.2,
      sphereObj.frame->getWorldTransform(),
      result);
  EXPECT_EQ(contacts, 1);
  EXPECT_GT(result.getNumContacts(), 0u);

  tfSphere.translation() = Eigen::Vector3d(2.0, 0.0, 0.0);
  sphereObj.frame->setRelativeTransform(tfSphere);
  CollisionResult noContact;
  contacts = collideCylinderSphere(
      cylObj.object.get(),
      sphereObj.object.get(),
      0.5,
      1.0,
      cylObj.frame->getWorldTransform(),
      0.2,
      sphereObj.frame->getWorldTransform(),
      noContact);
  EXPECT_EQ(contacts, 0);
  EXPECT_EQ(noContact.getNumContacts(), 0u);

  CollisionResult planeContact;
  contacts = collideCylinderPlane(
      cylObj.object.get(),
      sphereObj.object.get(),
      0.5,
      1.0,
      cylObj.frame->getWorldTransform(),
      Eigen::Vector3d::UnitZ(),
      Eigen::Isometry3d::Identity(),
      planeContact);
  EXPECT_EQ(contacts, 1);
  EXPECT_GT(planeContact.getNumContacts(), 0u);
}

TEST(DARTCollide, CylinderSphereSideAndCap)
{
  auto detector = DARTCollisionDetector::create();
  auto shape = std::make_shared<SphereShape>(0.1);

  auto cylObj = makeObject(shape, detector.get(), Eigen::Isometry3d::Identity());
  auto sphereObj
      = makeObject(shape, detector.get(), Eigen::Isometry3d::Identity());

  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(0.7, 0.0, 0.0);
  sphereObj.frame->setRelativeTransform(tfSphere);

  CollisionResult sideContact;
  int contacts = collideCylinderSphere(
      cylObj.object.get(),
      sphereObj.object.get(),
      0.5,
      1.0,
      cylObj.frame->getWorldTransform(),
      0.3,
      sphereObj.frame->getWorldTransform(),
      sideContact);
  EXPECT_EQ(contacts, 1);
  EXPECT_GT(sideContact.getNumContacts(), 0u);

  tfSphere.translation() = Eigen::Vector3d(0.7, 0.0, 1.1);
  sphereObj.frame->setRelativeTransform(tfSphere);

  CollisionResult capContact;
  contacts = collideCylinderSphere(
      cylObj.object.get(),
      sphereObj.object.get(),
      0.5,
      1.0,
      cylObj.frame->getWorldTransform(),
      0.3,
      sphereObj.frame->getWorldTransform(),
      capContact);
  EXPECT_EQ(contacts, 1);
  EXPECT_GT(capContact.getNumContacts(), 0u);
}

TEST(DARTCollide, CylinderPlaneNonParallelAndSeparated)
{
  auto detector = DARTCollisionDetector::create();
  auto shape = std::make_shared<SphereShape>(0.1);

  auto cylObj = makeObject(shape, detector.get(), Eigen::Isometry3d::Identity());
  auto planeObj
      = makeObject(shape, detector.get(), Eigen::Isometry3d::Identity());

  Eigen::Isometry3d planeTf = Eigen::Isometry3d::Identity();
  planeTf.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);

  CollisionResult contact;
  int contacts = collideCylinderPlane(
      cylObj.object.get(),
      planeObj.object.get(),
      0.5,
      1.0,
      cylObj.frame->getWorldTransform(),
      Eigen::Vector3d::UnitY(),
      planeTf,
      contact);
  EXPECT_EQ(contacts, 1);
  EXPECT_GT(contact.getNumContacts(), 0u);

  planeTf.translation() = Eigen::Vector3d(0.0, -2.0, 0.0);
  CollisionResult separated;
  contacts = collideCylinderPlane(
      cylObj.object.get(),
      planeObj.object.get(),
      0.5,
      1.0,
      cylObj.frame->getWorldTransform(),
      Eigen::Vector3d::UnitY(),
      planeTf,
      separated);
  EXPECT_EQ(contacts, 0);
  EXPECT_EQ(separated.getNumContacts(), 0u);
}

TEST(DARTCollideHelpers, CullPointsAndIntersectRectQuad)
{
  int iret[4] = {-1, -1, -1, -1};

  double single[2] = {1.0, 2.0};
  cullPoints(1, single, 1, 0, iret);
  EXPECT_EQ(iret[0], 0);

  double segment[4] = {0.0, 0.0, 1.0, 0.0};
  cullPoints(2, segment, 2, 0, iret);
  EXPECT_EQ(iret[0], 0);
  EXPECT_EQ(iret[1], 1);

  double quad[8] = {0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0};
  cullPoints(4, quad, 3, 0, iret);
  for (int i = 0; i < 3; ++i) {
    EXPECT_GE(iret[i], 0);
    EXPECT_LT(iret[i], 4);
  }

  double h[2] = {1.0, 1.0};
  double inside[8] = {0.5, 0.5, -0.5, 0.5, -0.5, -0.5, 0.5, -0.5};
  double ret[16] = {};
  const int insideCount = intersectRectQuad(h, inside, ret);
  EXPECT_EQ(insideCount, 4);

  double outside[8] = {3.0, 3.0, 4.0, 3.0, 4.0, 4.0, 3.0, 4.0};
  const int outsideCount = intersectRectQuad(h, outside, ret);
  EXPECT_EQ(outsideCount, 0);
}

TEST(DARTCollideHelpers, LineClosestApproach)
{
  dVector3 pa = {0.0, 0.0, 0.0, 0.0};
  dVector3 pb = {1.0, 1.0, 0.0, 0.0};
  dVector3 ua = {1.0, 0.0, 0.0, 0.0};
  dVector3 ub = {0.0, 1.0, 0.0, 0.0};

  double alpha = 0.0;
  double beta = 0.0;
  dLineClosestApproach(pa, ua, pb, ub, &alpha, &beta);
  EXPECT_NEAR(alpha, 1.0, 1e-12);
  EXPECT_NEAR(beta, -1.0, 1e-12);

  dVector3 ubParallel = {1.0, 0.0, 0.0, 0.0};
  alpha = 1.0;
  beta = 1.0;
  dLineClosestApproach(pa, ua, pb, ubParallel, &alpha, &beta);
  EXPECT_EQ(alpha, 0.0);
  EXPECT_EQ(beta, 0.0);
}

TEST(DARTCollideHelpers, ClosestLineBoxPoints)
{
  dVector3 p1 = {2.0, -2.0, 0.5, 0.0};
  dVector3 p2 = {-2.0, 2.0, -0.5, 0.0};
  dVector3 center = {0.0, 0.0, 0.0, 0.0};
  dMatrix3 rotation = {1.0, 0.0, 0.0, 0.0,
                       0.0, 1.0, 0.0, 0.0,
                       0.0, 0.0, 1.0, 0.0};
  dVector3 side = {1.0, 1.0, 1.0, 0.0};
  dVector3 lret = {0.0, 0.0, 0.0, 0.0};
  dVector3 bret = {0.0, 0.0, 0.0, 0.0};

  dClosestLineBoxPoints(p1, p2, center, rotation, side, lret, bret);

  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(std::isfinite(lret[i]));
    EXPECT_TRUE(std::isfinite(bret[i]));
    EXPECT_LE(std::abs(bret[i]), side[i] + 1e-9);
  }

  dVector3 p3 = {-3.0, -0.5, 0.0, 0.0};
  dVector3 p4 = {3.0, 0.5, 0.0, 0.0};
  dClosestLineBoxPoints(p3, p4, center, rotation, side, lret, bret);
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(std::isfinite(lret[i]));
    EXPECT_TRUE(std::isfinite(bret[i]));
  }
}

} // namespace dart::test
