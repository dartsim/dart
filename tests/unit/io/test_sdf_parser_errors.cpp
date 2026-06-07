// Copyright (c) 2011, The DART development contributors

#include <dart/config.hpp>

#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/io/All.hpp>

#include <gtest/gtest.h>

#if DART_HAVE_SDFORMAT

using namespace dart;

TEST(SdfParserErrors, NonExistentSkeletonFileReturnsNull)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Sdf;
  const auto skeleton
      = io::readSkeleton("dart://sample/sdf/test/does_not_exist.sdf", options);
  EXPECT_EQ(skeleton, nullptr);
}

TEST(SdfParserErrors, ValidSkeletonLoads)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Sdf;
  const auto skeleton = io::readSkeleton(
      "dart://sample/sdf/test/two_link_revolute_model.sdf", options);
  EXPECT_NE(skeleton, nullptr);
  if (skeleton) {
    EXPECT_EQ(skeleton->getNumBodyNodes(), 2u);
  }
}

TEST(SdfParserErrors, FloatingRootJointByDefault)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Sdf;
  const auto skeleton = io::readSkeleton(
      "dart://sample/sdf/test/two_link_revolute_model.sdf", options);
  ASSERT_NE(skeleton, nullptr);
  ASSERT_GE(skeleton->getNumJoints(), 1u);

  EXPECT_NE(dynamic_cast<dynamics::FreeJoint*>(skeleton->getJoint(0)), nullptr);
}

TEST(SdfParserErrors, FixedRootJointWhenRequested)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Sdf;
  options.sdfDefaultRootJointType = io::RootJointType::Fixed;
  const auto skeleton = io::readSkeleton(
      "dart://sample/sdf/test/two_link_revolute_model.sdf", options);
  ASSERT_NE(skeleton, nullptr);
  ASSERT_GE(skeleton->getNumJoints(), 1u);

  EXPECT_NE(dynamic_cast<dynamics::WeldJoint*>(skeleton->getJoint(0)), nullptr);
}

#endif
