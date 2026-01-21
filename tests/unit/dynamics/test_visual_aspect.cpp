#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <memory>

using namespace dart::dynamics;

TEST(VisualAspect, TracksExplicitColorState)
{
  auto skeleton = Skeleton::create("skel");
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d::Ones());
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto* shapeNode = pair.second->createShapeNodeWith<VisualAspect>(box);
  auto* visual = shapeNode->getVisualAspect();

  ASSERT_NE(visual, nullptr);
  EXPECT_TRUE(visual->usesDefaultColor());
  EXPECT_TRUE(visual->getRGBA().isApprox(VisualAspect::getDefaultRGBA()));

  visual->setAlpha(0.5);
  EXPECT_TRUE(visual->usesDefaultColor());
  EXPECT_DOUBLE_EQ(visual->getAlpha(), 0.5);

  visual->setRGBA(Eigen::Vector4d::Ones());
  EXPECT_FALSE(visual->usesDefaultColor());
  EXPECT_TRUE(visual->getRGBA().isApprox(Eigen::Vector4d::Ones()));

  visual->resetColor();
  EXPECT_TRUE(visual->usesDefaultColor());
  EXPECT_TRUE(visual->getRGBA().isApprox(VisualAspect::getDefaultRGBA()));

  visual->setRGBA(VisualAspect::getDefaultRGBA());
  EXPECT_FALSE(visual->usesDefaultColor());
}
