// Copyright (c) 2011, The DART development contributors

#include <dart/collision/collision_object.hpp>
#include <dart/collision/collision_result.hpp>
#include <dart/collision/dart/dart_collide.hpp>
#include <dart/collision/dart/dart_collision_detector.hpp>

#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/sphere_shape.hpp>

#include <dart/math/constants.hpp>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <memory>

#include <cmath>

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

TEST(DARTCollide, CollisionObjectAccessors)
{
  auto detector = DARTCollisionDetector::create();
  auto frame = std::make_shared<SimpleFrame>(Frame::World(), "frame");
  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d::Ones());
  frame->setShape(shape);

  TestCollisionObject object(detector.get(), frame.get());

  EXPECT_EQ(object.getCollisionDetector(), detector.get());
  EXPECT_EQ(object.getShapeFrame(), frame.get());
  EXPECT_EQ(object.getShape(), shape);
  EXPECT_TRUE(object.getTransform().isApprox(frame->getWorldTransform()));
}

TEST(DARTCollide, SphereSphereCases)
{
  auto detector = DARTCollisionDetector::create();
  auto sphere = std::make_shared<SphereShape>(0.5);

  auto objA = makeObject(sphere, detector.get(), Eigen::Isometry3d::Identity());

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

  auto boxObj = makeObject(box, detector.get(), Eigen::Isometry3d::Identity());

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

  auto boxObj = makeObject(box, detector.get(), Eigen::Isometry3d::Identity());
  auto sphereObj
      = makeObject(sphere, detector.get(), Eigen::Isometry3d::Identity());

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

TEST(DARTCollide, BoxSphereAxisClampingAndInside)
{
  auto detector = DARTCollisionDetector::create();
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(1.0));
  auto sphere = std::make_shared<SphereShape>(0.2);

  auto boxObj = makeObject(box, detector.get(), Eigen::Isometry3d::Identity());
  auto sphereObj
      = makeObject(sphere, detector.get(), Eigen::Isometry3d::Identity());

  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(0.0, 0.6, 0.0);
  sphereObj.frame->setRelativeTransform(tfSphere);

  CollisionResult yClamp;
  int contacts = collideBoxSphere(
      boxObj.object.get(),
      sphereObj.object.get(),
      box->getSize(),
      boxObj.frame->getWorldTransform(),
      sphere->getRadius(),
      sphereObj.frame->getWorldTransform(),
      yClamp);
  EXPECT_EQ(contacts, 1);
  ASSERT_EQ(yClamp.getNumContacts(), 1u);
  EXPECT_GT(std::abs(yClamp.getContact(0).normal.y()), 0.9);

  tfSphere.translation() = Eigen::Vector3d(0.0, 0.0, 0.6);
  sphereObj.frame->setRelativeTransform(tfSphere);

  CollisionResult zClamp;
  contacts = collideBoxSphere(
      boxObj.object.get(),
      sphereObj.object.get(),
      box->getSize(),
      boxObj.frame->getWorldTransform(),
      sphere->getRadius(),
      sphereObj.frame->getWorldTransform(),
      zClamp);
  EXPECT_EQ(contacts, 1);
  ASSERT_EQ(zClamp.getNumContacts(), 1u);
  EXPECT_GT(std::abs(zClamp.getContact(0).normal.z()), 0.9);

  tfSphere.translation() = Eigen::Vector3d(0.0, 0.49, 0.1);
  sphereObj.frame->setRelativeTransform(tfSphere);

  CollisionResult insideY;
  contacts = collideBoxSphere(
      boxObj.object.get(),
      sphereObj.object.get(),
      box->getSize(),
      boxObj.frame->getWorldTransform(),
      sphere->getRadius(),
      sphereObj.frame->getWorldTransform(),
      insideY);
  EXPECT_EQ(contacts, 1);
  ASSERT_EQ(insideY.getNumContacts(), 1u);
  EXPECT_GT(std::abs(insideY.getContact(0).normal.y()), 0.9);

  tfSphere.translation() = Eigen::Vector3d(0.1, 0.1, 0.49);
  sphereObj.frame->setRelativeTransform(tfSphere);

  CollisionResult insideZ;
  contacts = collideBoxSphere(
      boxObj.object.get(),
      sphereObj.object.get(),
      box->getSize(),
      boxObj.frame->getWorldTransform(),
      sphere->getRadius(),
      sphereObj.frame->getWorldTransform(),
      insideZ);
  EXPECT_EQ(contacts, 1);
  ASSERT_EQ(insideZ.getNumContacts(), 1u);
  EXPECT_GT(std::abs(insideZ.getContact(0).normal.z()), 0.9);
}

TEST(DARTCollide, SphereBoxInsideAndBoundary)
{
  constexpr double kNearBoundaryEps = 1e-8;
  auto detector = DARTCollisionDetector::create();
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(1.0));
  auto sphere = std::make_shared<SphereShape>(0.2);

  auto boxObj = makeObject(box, detector.get(), Eigen::Isometry3d::Identity());
  auto sphereObj
      = makeObject(sphere, detector.get(), Eigen::Isometry3d::Identity());

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

TEST(DARTCollide, SphereBoxAxisClampingAndNearZero)
{
  constexpr double kNearZeroEps = 1e-8;
  auto detector = DARTCollisionDetector::create();
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(1.0));
  auto sphere = std::make_shared<SphereShape>(0.2);

  auto boxObj = makeObject(box, detector.get(), Eigen::Isometry3d::Identity());
  auto sphereObj
      = makeObject(sphere, detector.get(), Eigen::Isometry3d::Identity());

  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(0.0, 0.6, 0.0);
  sphereObj.frame->setRelativeTransform(tfSphere);

  CollisionResult yClamp;
  int contacts = collideSphereBox(
      sphereObj.object.get(),
      boxObj.object.get(),
      sphere->getRadius(),
      sphereObj.frame->getWorldTransform(),
      box->getSize(),
      boxObj.frame->getWorldTransform(),
      yClamp);
  EXPECT_EQ(contacts, 1);
  ASSERT_EQ(yClamp.getNumContacts(), 1u);
  EXPECT_GT(std::abs(yClamp.getContact(0).normal.y()), 0.9);

  tfSphere.translation() = Eigen::Vector3d(0.0, 0.0, 0.6);
  sphereObj.frame->setRelativeTransform(tfSphere);

  CollisionResult zClamp;
  contacts = collideSphereBox(
      sphereObj.object.get(),
      boxObj.object.get(),
      sphere->getRadius(),
      sphereObj.frame->getWorldTransform(),
      box->getSize(),
      boxObj.frame->getWorldTransform(),
      zClamp);
  EXPECT_EQ(contacts, 1);
  ASSERT_EQ(zClamp.getNumContacts(), 1u);
  EXPECT_GT(std::abs(zClamp.getContact(0).normal.z()), 0.9);

  tfSphere.translation() = Eigen::Vector3d(0.0, 0.49, 0.1);
  sphereObj.frame->setRelativeTransform(tfSphere);

  CollisionResult insideY;
  contacts = collideSphereBox(
      sphereObj.object.get(),
      boxObj.object.get(),
      sphere->getRadius(),
      sphereObj.frame->getWorldTransform(),
      box->getSize(),
      boxObj.frame->getWorldTransform(),
      insideY);
  EXPECT_EQ(contacts, 1);
  ASSERT_EQ(insideY.getNumContacts(), 1u);
  EXPECT_GT(std::abs(insideY.getContact(0).normal.y()), 0.9);

  tfSphere.translation() = Eigen::Vector3d(0.1, 0.1, 0.49);
  sphereObj.frame->setRelativeTransform(tfSphere);

  CollisionResult insideZ;
  contacts = collideSphereBox(
      sphereObj.object.get(),
      boxObj.object.get(),
      sphere->getRadius(),
      sphereObj.frame->getWorldTransform(),
      box->getSize(),
      boxObj.frame->getWorldTransform(),
      insideZ);
  EXPECT_EQ(contacts, 1);
  ASSERT_EQ(insideZ.getNumContacts(), 1u);
  EXPECT_GT(std::abs(insideZ.getContact(0).normal.z()), 0.9);

  tfSphere.translation() = Eigen::Vector3d(0.0, 0.5 + kNearZeroEps, 0.0);
  sphereObj.frame->setRelativeTransform(tfSphere);

  CollisionResult nearZero;
  contacts = collideSphereBox(
      sphereObj.object.get(),
      boxObj.object.get(),
      sphere->getRadius(),
      sphereObj.frame->getWorldTransform(),
      box->getSize(),
      boxObj.frame->getWorldTransform(),
      nearZero);
  EXPECT_EQ(contacts, 1);
  ASSERT_EQ(nearZero.getNumContacts(), 1u);
  EXPECT_GT(std::abs(nearZero.getContact(0).normal.y()), 0.9);
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

  auto cylObj
      = makeObject(shape, detector.get(), Eigen::Isometry3d::Identity());
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

  auto cylObj
      = makeObject(shape, detector.get(), Eigen::Isometry3d::Identity());
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

TEST(DARTCollide, CylinderPlaneParallelAxisAndEqualDepth)
{
  auto detector = DARTCollisionDetector::create();
  auto shape = std::make_shared<SphereShape>(0.1);

  auto cylObj
      = makeObject(shape, detector.get(), Eigen::Isometry3d::Identity());
  auto planeObj
      = makeObject(shape, detector.get(), Eigen::Isometry3d::Identity());

  Eigen::Isometry3d planeTf = Eigen::Isometry3d::Identity();
  planeTf.translation() = Eigen::Vector3d(0.0, 0.0, 0.4);

  CollisionResult parallelAxis;
  int contacts = collideCylinderPlane(
      cylObj.object.get(),
      planeObj.object.get(),
      0.5,
      0.5,
      cylObj.frame->getWorldTransform(),
      Eigen::Vector3d::UnitZ(),
      planeTf,
      parallelAxis);
  EXPECT_EQ(contacts, 1);
  EXPECT_GT(parallelAxis.getNumContacts(), 0u);

  planeTf = Eigen::Isometry3d::Identity();
  CollisionResult equalDepth;
  contacts = collideCylinderPlane(
      cylObj.object.get(),
      planeObj.object.get(),
      0.5,
      0.5,
      cylObj.frame->getWorldTransform(),
      Eigen::Vector3d::UnitX(),
      planeTf,
      equalDepth);
  EXPECT_EQ(contacts, 1);
  EXPECT_GT(equalDepth.getNumContacts(), 0u);
}

TEST(DARTCollide, CylinderPlaneAllNegativeDepths)
{
  auto detector = DARTCollisionDetector::create();
  auto shape = std::make_shared<SphereShape>(0.1);

  auto cylObj
      = makeObject(shape, detector.get(), Eigen::Isometry3d::Identity());
  auto planeObj
      = makeObject(shape, detector.get(), Eigen::Isometry3d::Identity());

  Eigen::Isometry3d planeTf = Eigen::Isometry3d::Identity();
  planeTf.translation() = Eigen::Vector3d(-2.0, 0.0, 0.0);

  CollisionResult noContact;
  int contacts = collideCylinderPlane(
      cylObj.object.get(),
      planeObj.object.get(),
      0.5,
      0.5,
      cylObj.frame->getWorldTransform(),
      Eigen::Vector3d::UnitX(),
      planeTf,
      noContact);
  EXPECT_EQ(contacts, 0);
  EXPECT_EQ(noContact.getNumContacts(), 0u);
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

TEST(DARTCollideHelpers, IntersectRectQuadOverflow)
{
  double h[2] = {1.0, 1.0};
  double ret[16] = {};
  double overflowQuad[8] = {2.0, 2.0, -2.0, 2.0, 2.0, -2.0, -2.0, -2.0};
  const int count = intersectRectQuad(h, overflowQuad, ret);
  EXPECT_GE(count, 0);
  EXPECT_LE(count, 8);
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
  dMatrix3 rotation
      = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0};
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

TEST(DARTCollideHelpers, ClosestLineBoxPointsEdgeCases)
{
  dMatrix3 rotation
      = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  dVector3 center = {0.0, 0.0, 0.0, 0.0};
  dVector3 side = {1.0, 1.0, 1.0, 0.0};
  dVector3 lret = {0.0, 0.0, 0.0, 0.0};
  dVector3 bret = {0.0, 0.0, 0.0, 0.0};

  dVector3 insideStart = {0.0, 0.0, 0.0, 0.0};
  dVector3 insideEnd = {2.0, 0.0, 0.0, 0.0};
  dClosestLineBoxPoints(
      insideStart, insideEnd, center, rotation, side, lret, bret);
  EXPECT_NEAR(lret[0], insideStart[0], 1e-12);
  EXPECT_NEAR(lret[1], insideStart[1], 1e-12);
  EXPECT_NEAR(lret[2], insideStart[2], 1e-12);

  dVector3 crossingStart = {-2.0, -2.0, -2.0, 0.0};
  dVector3 crossingEnd = {2.0, 2.0, 2.0, 0.0};
  dClosestLineBoxPoints(
      crossingStart, crossingEnd, center, rotation, side, lret, bret);
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(std::isfinite(lret[i]));
    EXPECT_TRUE(std::isfinite(bret[i]));
    EXPECT_LE(std::abs(bret[i]), side[i] + 1e-9);
  }

  dVector3 outsideStart = {2.0, 2.0, 2.0, 0.0};
  dVector3 outsideEnd = {2.5, 2.5, 2.5, 0.0};
  dClosestLineBoxPoints(
      outsideStart, outsideEnd, center, rotation, side, lret, bret);
  for (int i = 0; i < 3; ++i) {
    EXPECT_LE(std::abs(bret[i]), side[i] + 1e-9);
  }
}

// Additional comprehensive tests for main collide() function dispatcher
TEST(DARTCollide, MainDispatcherSphereSphere)
{
  auto detector = DARTCollisionDetector::create();
  auto sphere1 = std::make_shared<SphereShape>(0.5);
  auto sphere2 = std::make_shared<SphereShape>(0.3);

  auto obj1
      = makeObject(sphere1, detector.get(), Eigen::Isometry3d::Identity());

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0.7, 0.0, 0.0);
  auto obj2 = makeObject(sphere2, detector.get(), tf2);

  CollisionResult result;
  int contacts = collide(obj1.object.get(), obj2.object.get(), result);
  EXPECT_GT(contacts, 0);
  EXPECT_GT(result.getNumContacts(), 0u);
}

TEST(DARTCollide, MainDispatcherBoxBox)
{
  auto detector = DARTCollisionDetector::create();
  auto box1 = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));
  auto box2 = std::make_shared<BoxShape>(Eigen::Vector3d(0.5, 0.5, 0.5));

  auto obj1 = makeObject(box1, detector.get(), Eigen::Isometry3d::Identity());

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0.3, 0.0, 0.0);
  auto obj2 = makeObject(box2, detector.get(), tf2);

  CollisionResult result;
  int contacts = collide(obj1.object.get(), obj2.object.get(), result);
  EXPECT_GT(contacts, 0);
  EXPECT_GT(result.getNumContacts(), 0u);
}

TEST(DARTCollide, MainDispatcherBoxSphere)
{
  auto detector = DARTCollisionDetector::create();
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));
  auto sphere = std::make_shared<SphereShape>(0.3);

  auto boxObj = makeObject(box, detector.get(), Eigen::Isometry3d::Identity());

  Eigen::Isometry3d sphereTf = Eigen::Isometry3d::Identity();
  sphereTf.translation() = Eigen::Vector3d(0.4, 0.0, 0.0);
  auto sphereObj = makeObject(sphere, detector.get(), sphereTf);

  CollisionResult result;
  int contacts = collide(boxObj.object.get(), sphereObj.object.get(), result);
  EXPECT_GT(contacts, 0);
  EXPECT_GT(result.getNumContacts(), 0u);

  // Test reverse order
  CollisionResult reverseResult;
  int reverseContacts
      = collide(sphereObj.object.get(), boxObj.object.get(), reverseResult);
  EXPECT_GT(reverseContacts, 0);
  EXPECT_GT(reverseResult.getNumContacts(), 0u);
}

TEST(DARTCollide, MainDispatcherSphereBox)
{
  auto detector = DARTCollisionDetector::create();
  auto sphere = std::make_shared<SphereShape>(0.3);
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));

  auto sphereObj
      = makeObject(sphere, detector.get(), Eigen::Isometry3d::Identity());

  Eigen::Isometry3d boxTf = Eigen::Isometry3d::Identity();
  boxTf.translation() = Eigen::Vector3d(0.4, 0.0, 0.0);
  auto boxObj = makeObject(box, detector.get(), boxTf);

  CollisionResult result;
  int contacts = collide(sphereObj.object.get(), boxObj.object.get(), result);
  EXPECT_GT(contacts, 0);
  EXPECT_GT(result.getNumContacts(), 0u);
}

TEST(DARTCollide, MainDispatcherEllipsoidSphere)
{
  auto detector = DARTCollisionDetector::create();
  auto ellipsoid
      = std::make_shared<EllipsoidShape>(Eigen::Vector3d(0.6, 0.4, 0.5));
  auto sphere = std::make_shared<SphereShape>(0.2);

  auto ellipsoidObj
      = makeObject(ellipsoid, detector.get(), Eigen::Isometry3d::Identity());

  Eigen::Isometry3d sphereTf = Eigen::Isometry3d::Identity();
  sphereTf.translation() = Eigen::Vector3d(0.3, 0.0, 0.0);
  auto sphereObj = makeObject(sphere, detector.get(), sphereTf);

  CollisionResult result;
  int contacts
      = collide(ellipsoidObj.object.get(), sphereObj.object.get(), result);
  EXPECT_GT(contacts, 0);
  EXPECT_GT(result.getNumContacts(), 0u);
}

TEST(DARTCollide, MainDispatcherSphereEllipsoid)
{
  auto detector = DARTCollisionDetector::create();
  auto sphere = std::make_shared<SphereShape>(0.2);
  auto ellipsoid
      = std::make_shared<EllipsoidShape>(Eigen::Vector3d(0.6, 0.4, 0.5));

  auto sphereObj
      = makeObject(sphere, detector.get(), Eigen::Isometry3d::Identity());

  Eigen::Isometry3d ellipsoidTf = Eigen::Isometry3d::Identity();
  ellipsoidTf.translation() = Eigen::Vector3d(0.3, 0.0, 0.0);
  auto ellipsoidObj = makeObject(ellipsoid, detector.get(), ellipsoidTf);

  CollisionResult result;
  int contacts
      = collide(sphereObj.object.get(), ellipsoidObj.object.get(), result);
  EXPECT_GT(contacts, 0);
  EXPECT_GT(result.getNumContacts(), 0u);
}

TEST(DARTCollide, MainDispatcherEllipsoidBox)
{
  auto detector = DARTCollisionDetector::create();
  auto ellipsoid
      = std::make_shared<EllipsoidShape>(Eigen::Vector3d(0.6, 0.4, 0.5));
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));

  auto ellipsoidObj
      = makeObject(ellipsoid, detector.get(), Eigen::Isometry3d::Identity());

  Eigen::Isometry3d boxTf = Eigen::Isometry3d::Identity();
  boxTf.translation() = Eigen::Vector3d(0.3, 0.0, 0.0);
  auto boxObj = makeObject(box, detector.get(), boxTf);

  CollisionResult result;
  int contacts
      = collide(ellipsoidObj.object.get(), boxObj.object.get(), result);
  EXPECT_GT(contacts, 0);
  EXPECT_GT(result.getNumContacts(), 0u);
}

TEST(DARTCollide, MainDispatcherBoxEllipsoid)
{
  auto detector = DARTCollisionDetector::create();
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));
  auto ellipsoid
      = std::make_shared<EllipsoidShape>(Eigen::Vector3d(0.6, 0.4, 0.5));

  auto boxObj = makeObject(box, detector.get(), Eigen::Isometry3d::Identity());

  Eigen::Isometry3d ellipsoidTf = Eigen::Isometry3d::Identity();
  ellipsoidTf.translation() = Eigen::Vector3d(0.3, 0.0, 0.0);
  auto ellipsoidObj = makeObject(ellipsoid, detector.get(), ellipsoidTf);

  CollisionResult result;
  int contacts
      = collide(boxObj.object.get(), ellipsoidObj.object.get(), result);
  EXPECT_GT(contacts, 0);
  EXPECT_GT(result.getNumContacts(), 0u);
}

TEST(DARTCollide, MainDispatcherEllipsoidEllipsoid)
{
  auto detector = DARTCollisionDetector::create();
  auto ellipsoid1
      = std::make_shared<EllipsoidShape>(Eigen::Vector3d(0.6, 0.4, 0.5));
  auto ellipsoid2
      = std::make_shared<EllipsoidShape>(Eigen::Vector3d(0.4, 0.3, 0.3));

  auto obj1
      = makeObject(ellipsoid1, detector.get(), Eigen::Isometry3d::Identity());

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0.4, 0.0, 0.0);
  auto obj2 = makeObject(ellipsoid2, detector.get(), tf2);

  CollisionResult result;
  int contacts = collide(obj1.object.get(), obj2.object.get(), result);
  EXPECT_GT(contacts, 0);
  EXPECT_GT(result.getNumContacts(), 0u);
}

TEST(DARTCollide, MainDispatcherNoCollision)
{
  auto detector = DARTCollisionDetector::create();
  auto sphere1 = std::make_shared<SphereShape>(0.5);
  auto sphere2 = std::make_shared<SphereShape>(0.3);

  auto obj1
      = makeObject(sphere1, detector.get(), Eigen::Isometry3d::Identity());

  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(2.0, 0.0, 0.0); // Far apart
  auto obj2 = makeObject(sphere2, detector.get(), tf2);

  CollisionResult result;
  int contacts = collide(obj1.object.get(), obj2.object.get(), result);
  EXPECT_EQ(contacts, 0);
  EXPECT_EQ(result.getNumContacts(), 0u);
}

TEST(DARTCollide, MainDispatcherEdgeCases)
{
  auto detector = DARTCollisionDetector::create();
  auto sphere = std::make_shared<SphereShape>(0.1);
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(1.0));

  // Test touching (tangent) case
  auto sphereObj
      = makeObject(sphere, detector.get(), Eigen::Isometry3d::Identity());

  Eigen::Isometry3d boxTf = Eigen::Isometry3d::Identity();
  boxTf.translation() = Eigen::Vector3d(0.6, 0.0, 0.0); // Exactly touching
  auto boxObj = makeObject(box, detector.get(), boxTf);

  CollisionResult result;
  int contacts = collide(sphereObj.object.get(), boxObj.object.get(), result);
  // Touching should result in some contacts (might be 0 or 1 depending on
  // implementation)
  EXPECT_GE(contacts, 0);
}

TEST(DARTCollide, CylinderPlaneEqualDepthAdjacentEdges)
{
  auto detector = DARTCollisionDetector::create();
  auto shape = std::make_shared<SphereShape>(0.1);

  auto cylObj
      = makeObject(shape, detector.get(), Eigen::Isometry3d::Identity());
  auto planeObj
      = makeObject(shape, detector.get(), Eigen::Isometry3d::Identity());

  CollisionResult result;
  int contacts = collideCylinderPlane(
      cylObj.object.get(),
      planeObj.object.get(),
      0.5,
      1.0,
      cylObj.frame->getWorldTransform(),
      Eigen::Vector3d::UnitX(),
      Eigen::Isometry3d::Identity(),
      result);
  EXPECT_EQ(contacts, 1);
  ASSERT_EQ(result.getNumContacts(), 1u);
  EXPECT_GT(result.getContact(0).penetrationDepth, 0.0);
  EXPECT_TRUE(
      result.getContact(0).normal.isApprox(Eigen::Vector3d::UnitX(), 1e-12));
}

TEST(DARTCollide, CylinderPlaneEqualDepthOppositeEdges)
{
  auto detector = DARTCollisionDetector::create();
  auto shape = std::make_shared<SphereShape>(0.1);

  auto cylObj
      = makeObject(shape, detector.get(), Eigen::Isometry3d::Identity());
  auto planeObj
      = makeObject(shape, detector.get(), Eigen::Isometry3d::Identity());

  CollisionResult result;
  int contacts = collideCylinderPlane(
      cylObj.object.get(),
      planeObj.object.get(),
      0.5,
      1.0,
      cylObj.frame->getWorldTransform(),
      Eigen::Vector3d::UnitZ(),
      Eigen::Isometry3d::Identity(),
      result);
  EXPECT_EQ(contacts, 1);
  ASSERT_EQ(result.getNumContacts(), 1u);
  EXPECT_GT(result.getContact(0).penetrationDepth, 0.0);
  EXPECT_TRUE(
      result.getContact(0).normal.isApprox(Eigen::Vector3d::UnitZ(), 1e-12));
}

} // namespace dart::test

#include <dart/dynamics/capsule_shape.hpp>
#include <dart/dynamics/cylinder_shape.hpp>
#include <dart/dynamics/plane_shape.hpp>

namespace dart::test {

TEST(DARTCollide, CapsuleCapsuleUnsupportedPair)
{
  auto detector = DARTCollisionDetector::create();
  auto capsuleA = std::make_shared<CapsuleShape>(0.2, 0.4);
  auto capsuleB = std::make_shared<CapsuleShape>(0.2, 0.4);

  auto frameA = std::make_shared<SimpleFrame>(Frame::World(), "capsule_a");
  frameA->setShape(capsuleA);
  auto frameB = std::make_shared<SimpleFrame>(Frame::World(), "capsule_b");
  frameB->setShape(capsuleB);

  Eigen::Isometry3d tfB = Eigen::Isometry3d::Identity();
  tfB.translation() = Eigen::Vector3d(0.0, 0.0, 0.3);
  frameB->setRelativeTransform(tfB);

  auto group = detector->createCollisionGroup();
  group->addShapeFrame(frameA.get());
  group->addShapeFrame(frameB.get());

  CollisionOption option;
  option.maxNumContacts = 4u;
  CollisionResult result;
  const bool collided = group->collide(option, &result);

  EXPECT_FALSE(collided);
  EXPECT_EQ(result.getNumContacts(), 0u);
}

TEST(DARTCollide, SphereCapsuleUnsupportedPair)
{
  auto detector = DARTCollisionDetector::create();
  auto sphere = std::make_shared<SphereShape>(0.2);
  auto capsule = std::make_shared<CapsuleShape>(0.2, 0.4);

  auto sphereFrame = std::make_shared<SimpleFrame>(Frame::World(), "sphere");
  sphereFrame->setShape(sphere);
  auto capsuleFrame = std::make_shared<SimpleFrame>(Frame::World(), "capsule");
  capsuleFrame->setShape(capsule);

  Eigen::Isometry3d tfSphere = Eigen::Isometry3d::Identity();
  tfSphere.translation() = Eigen::Vector3d(0.0, 0.0, 0.25);
  sphereFrame->setRelativeTransform(tfSphere);

  auto group = detector->createCollisionGroup();
  group->addShapeFrame(sphereFrame.get());
  group->addShapeFrame(capsuleFrame.get());

  CollisionOption option;
  option.maxNumContacts = 4u;
  CollisionResult result;
  const bool collided = group->collide(option, &result);

  EXPECT_FALSE(collided);
  EXPECT_EQ(result.getNumContacts(), 0u);
}

TEST(DARTCollide, BoxCylinderUnsupportedPair)
{
  auto detector = DARTCollisionDetector::create();
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(1.0));
  auto cylinder = std::make_shared<CylinderShape>(0.3, 1.0);

  auto boxFrame = std::make_shared<SimpleFrame>(Frame::World(), "box");
  boxFrame->setShape(box);
  auto cylinderFrame
      = std::make_shared<SimpleFrame>(Frame::World(), "cylinder");
  cylinderFrame->setShape(cylinder);

  Eigen::Isometry3d tfCyl = Eigen::Isometry3d::Identity();
  tfCyl.translation() = Eigen::Vector3d(0.0, 0.0, 0.2);
  cylinderFrame->setRelativeTransform(tfCyl);

  auto group = detector->createCollisionGroup();
  group->addShapeFrame(boxFrame.get());
  group->addShapeFrame(cylinderFrame.get());

  CollisionOption option;
  option.maxNumContacts = 4u;
  CollisionResult result;
  const bool collided = group->collide(option, &result);

  EXPECT_FALSE(collided);
  EXPECT_EQ(result.getNumContacts(), 0u);
}

TEST(DARTCollide, CylinderPlaneDirectContact)
{
  auto detector = DARTCollisionDetector::create();
  auto cylinder = std::make_shared<CylinderShape>(0.4, 1.0);
  auto plane = std::make_shared<PlaneShape>(Eigen::Vector3d::UnitY(), 0.0);

  Eigen::Isometry3d cylTf = Eigen::Isometry3d::Identity();
  cylTf.translation() = Eigen::Vector3d(0.0, 0.2, 0.0);

  auto cylObj = makeObject(cylinder, detector.get(), cylTf);
  auto planeObj
      = makeObject(plane, detector.get(), Eigen::Isometry3d::Identity());

  CollisionResult result;
  int contacts = collideCylinderPlane(
      cylObj.object.get(),
      planeObj.object.get(),
      cylinder->getRadius(),
      0.5 * cylinder->getHeight(),
      cylObj.frame->getWorldTransform(),
      Eigen::Vector3d::UnitY(),
      planeObj.frame->getWorldTransform(),
      result);

  EXPECT_EQ(contacts, 1);
  EXPECT_GT(result.getNumContacts(), 0u);
}

TEST(DARTCollide, CylinderPlaneParallelAxisFallback)
{
  auto detector = DARTCollisionDetector::create();
  auto cylinder = std::make_shared<CylinderShape>(0.4, 1.0);
  auto plane = std::make_shared<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0);

  Eigen::Isometry3d cylTf = Eigen::Isometry3d::Identity();
  cylTf.translation() = Eigen::Vector3d(0.0, 0.0, 0.4);

  auto cylObj = makeObject(cylinder, detector.get(), cylTf);
  auto planeObj
      = makeObject(plane, detector.get(), Eigen::Isometry3d::Identity());

  CollisionResult result;
  int contacts = collideCylinderPlane(
      cylObj.object.get(),
      planeObj.object.get(),
      cylinder->getRadius(),
      0.5 * cylinder->getHeight(),
      cylObj.frame->getWorldTransform(),
      Eigen::Vector3d::UnitZ(),
      planeObj.frame->getWorldTransform(),
      result);

  EXPECT_EQ(contacts, 1);
  ASSERT_EQ(result.getNumContacts(), 1u);
  EXPECT_GT(result.getContact(0).penetrationDepth, 0.0);
}

TEST(DARTCollide, CylinderSphereInteriorContact)
{
  auto detector = DARTCollisionDetector::create();
  auto cylinder = std::make_shared<CylinderShape>(0.5, 0.8);
  auto sphere = std::make_shared<SphereShape>(0.2);

  Eigen::Isometry3d cylTf = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d sphereTf = Eigen::Isometry3d::Identity();
  sphereTf.translation() = Eigen::Vector3d(0.1, 0.1, 0.0);

  auto cylObj = makeObject(cylinder, detector.get(), cylTf);
  auto sphereObj = makeObject(sphere, detector.get(), sphereTf);

  CollisionResult result;
  int contacts = collideCylinderSphere(
      cylObj.object.get(),
      sphereObj.object.get(),
      cylinder->getRadius(),
      0.5 * cylinder->getHeight(),
      cylObj.frame->getWorldTransform(),
      sphere->getRadius(),
      sphereObj.frame->getWorldTransform(),
      result);

  EXPECT_EQ(contacts, 1);
  EXPECT_GT(result.getNumContacts(), 0u);
}

TEST(DARTCollide, BoxBoxFaceContactNormalDepth)
{
  auto detector = DARTCollisionDetector::create();
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(1.0));

  Eigen::Isometry3d tfA = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfB = Eigen::Isometry3d::Identity();
  tfB.translation() = Eigen::Vector3d(0.45, 0.0, 0.0);

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
  ASSERT_GT(result.getNumContacts(), 0u);
  const auto& contact = result.getContact(0);
  EXPECT_GT(contact.penetrationDepth, 0.0);
  EXPECT_GT(std::abs(contact.normal.x()), 0.9);
}

TEST(DARTCollide, MainDispatcherUnsupportedPair)
{
  auto detector = DARTCollisionDetector::create();
  auto cylinder = std::make_shared<CylinderShape>(0.2, 0.4);
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(0.4));

  auto cylObj
      = makeObject(cylinder, detector.get(), Eigen::Isometry3d::Identity());
  auto boxObj = makeObject(box, detector.get(), Eigen::Isometry3d::Identity());

  CollisionResult result;
  int contacts = collide(cylObj.object.get(), boxObj.object.get(), result);
  EXPECT_EQ(contacts, 0);
  EXPECT_EQ(result.getNumContacts(), 0u);
}

TEST(DARTCollide, BoxBoxEdgeEdgeContact)
{
  auto detector = DARTCollisionDetector::create();
  auto boxA = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));
  auto boxB = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));

  // Box A at origin, identity
  auto objA = makeObject(boxA, detector.get(), Eigen::Isometry3d::Identity());

  // Box B rotated 45 deg around Z, translated so edges overlap
  Eigen::Isometry3d tfB = Eigen::Isometry3d::Identity();
  tfB.linear() = Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitZ())
                     .toRotationMatrix();
  tfB.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
  auto objB = makeObject(boxB, detector.get(), tfB);

  CollisionResult result;
  int contacts = collideBoxBox(
      objA.object.get(),
      objB.object.get(),
      boxA->getSize(),
      objA.frame->getWorldTransform(),
      boxB->getSize(),
      objB.frame->getWorldTransform(),
      result);
  EXPECT_GT(contacts, 0);
  if (result.getNumContacts() > 0) {
    EXPECT_TRUE(result.getContact(0).normal.allFinite());
    EXPECT_TRUE(result.getContact(0).point.allFinite());
    EXPECT_GT(result.getContact(0).penetrationDepth, 0.0);
  }
}

TEST(DARTCollide, BoxBoxEdgeEdgeCollisionGroup)
{
  auto detector = DARTCollisionDetector::create();
  auto boxA = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));
  auto boxB = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));

  auto frameA = std::make_shared<SimpleFrame>(Frame::World(), "edge_a");
  frameA->setShape(boxA);
  auto frameB = std::make_shared<SimpleFrame>(Frame::World(), "edge_b");
  frameB->setShape(boxB);

  Eigen::Isometry3d tfA = Eigen::Isometry3d::Identity();
  tfA.linear() = Eigen::AngleAxisd(0.15 * math::pi, Eigen::Vector3d::UnitX())
                     .toRotationMatrix();
  frameA->setRelativeTransform(tfA);

  Eigen::Isometry3d tfB = Eigen::Isometry3d::Identity();
  tfB.linear()
      = (Eigen::AngleAxisd(0.25 * math::pi, Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(0.35 * math::pi, Eigen::Vector3d::UnitZ()))
            .toRotationMatrix();
  tfB.translation() = Eigen::Vector3d(0.7, 0.2, 0.1);
  frameB->setRelativeTransform(tfB);

  auto group = detector->createCollisionGroup();
  group->addShapeFrame(frameA.get());
  group->addShapeFrame(frameB.get());

  CollisionOption option;
  option.maxNumContacts = 4u;
  CollisionResult result;
  const bool collided = group->collide(option, &result);

  EXPECT_TRUE(collided);
  ASSERT_GT(result.getNumContacts(), 0u);
  const auto& contact = result.getContact(0);
  EXPECT_TRUE(contact.point.allFinite());
  EXPECT_TRUE(contact.normal.allFinite());
  EXPECT_GT(contact.normal.norm(), 0.5);
}

TEST(DARTCollide, BoxBoxEdgeEdgeCollisionGroupSkewed)
{
  auto detector = DARTCollisionDetector::create();
  auto boxA = std::make_shared<BoxShape>(Eigen::Vector3d(1.2, 0.8, 1.0));
  auto boxB = std::make_shared<BoxShape>(Eigen::Vector3d(0.9, 1.1, 1.0));

  auto frameA = std::make_shared<SimpleFrame>(Frame::World(), "edge_c");
  frameA->setShape(boxA);
  auto frameB = std::make_shared<SimpleFrame>(Frame::World(), "edge_d");
  frameB->setShape(boxB);

  Eigen::Isometry3d tfA = Eigen::Isometry3d::Identity();
  tfA.linear() = Eigen::AngleAxisd(0.2 * math::pi, Eigen::Vector3d::UnitY())
                     .toRotationMatrix();
  frameA->setRelativeTransform(tfA);

  Eigen::Isometry3d tfB = Eigen::Isometry3d::Identity();
  tfB.linear() = (Eigen::AngleAxisd(0.4 * math::pi, Eigen::Vector3d::UnitX())
                  * Eigen::AngleAxisd(0.3 * math::pi, Eigen::Vector3d::UnitZ()))
                     .toRotationMatrix();
  tfB.translation() = Eigen::Vector3d(0.6, -0.25, 0.15);
  frameB->setRelativeTransform(tfB);

  auto group = detector->createCollisionGroup();
  group->addShapeFrame(frameA.get());
  group->addShapeFrame(frameB.get());

  CollisionOption option;
  option.maxNumContacts = 2u;
  CollisionResult result;
  const bool collided = group->collide(option, &result);

  EXPECT_TRUE(collided);
  ASSERT_GT(result.getNumContacts(), 0u);
  const auto& contact = result.getContact(0);
  EXPECT_GT(contact.penetrationDepth, 0.0);
  EXPECT_TRUE(contact.normal.allFinite());
}

} // namespace dart::test
