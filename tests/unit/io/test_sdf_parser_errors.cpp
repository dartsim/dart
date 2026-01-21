// Copyright (c) 2011-2025, The DART development contributors

#include <dart/config.hpp>

#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/io/all.hpp>

#include <gtest/gtest.h>

#if DART_HAVE_SDFORMAT

using namespace dart;

TEST(SdfParserErrors, NonExistentWorldFileReturnsNull)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Sdf;
  const auto world
      = io::readWorld("dart://sample/sdf/test/does_not_exist.world", options);
  EXPECT_EQ(world, nullptr);
}

TEST(SdfParserErrors, NonExistentSkeletonFileReturnsNull)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Sdf;
  const auto skeleton
      = io::readSkeleton("dart://sample/sdf/test/does_not_exist.sdf", options);
  EXPECT_EQ(skeleton, nullptr);
}

TEST(SdfParserErrors, ValidWorldLoads)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Sdf;
  const auto world
      = io::readWorld("dart://sample/sdf/double_pendulum.world", options);
  EXPECT_NE(world, nullptr);
}

TEST(SdfParserErrors, HighVersionWorldLoads)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Sdf;
  const auto world
      = io::readWorld("dart://sample/sdf/test/high_version.world", options);
  EXPECT_NE(world, nullptr);
  if (world) {
    EXPECT_EQ(world->getNumSkeletons(), 1u);
  }
}

TEST(SdfParserErrors, SingleBodySkeletonWorldLoads)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Sdf;
  const auto world = io::readWorld(
      "dart://sample/sdf/test/single_bodynode_skeleton.world", options);
  EXPECT_NE(world, nullptr);
  if (world) {
    EXPECT_GE(world->getNumSkeletons(), 1u);
  }
}

TEST(SdfParserErrors, FloatingRootJointByDefault)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Sdf;
  const auto world = io::readWorld(
      "dart://sample/sdf/test/single_bodynode_skeleton.world", options);
  ASSERT_NE(world, nullptr);
  ASSERT_GE(world->getNumSkeletons(), 1u);

  const auto skeleton = world->getSkeleton(0);
  ASSERT_NE(skeleton, nullptr);
  ASSERT_GE(skeleton->getNumJoints(), 1u);

  EXPECT_NE(dynamic_cast<dynamics::FreeJoint*>(skeleton->getJoint(0)), nullptr);
}

TEST(SdfParserErrors, FixedRootJointWhenRequested)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Sdf;
  options.sdfDefaultRootJointType = io::RootJointType::Fixed;
  const auto world = io::readWorld(
      "dart://sample/sdf/test/single_bodynode_skeleton.world", options);
  ASSERT_NE(world, nullptr);
  ASSERT_GE(world->getNumSkeletons(), 1u);

  const auto skeleton = world->getSkeleton(0);
  ASSERT_NE(skeleton, nullptr);
  ASSERT_GE(skeleton->getNumJoints(), 1u);

  EXPECT_NE(dynamic_cast<dynamics::WeldJoint*>(skeleton->getJoint(0)), nullptr);
}

#endif
