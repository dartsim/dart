// Copyright (c) 2011, The DART development contributors

#include <dart/config.hpp>

#include <dart/io/All.hpp>
#include <dart/io/urdf/urdf_parser.hpp>

#include <gtest/gtest.h>

#if DART_IO_HAS_URDF

using namespace dart;

TEST(UrdfParserErrors, NonExistentFileReturnsNull)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Urdf;
  const auto skeleton = io::readSkeleton(
      "dart://sample/urdf/test/does_not_exist.urdf", options);
  EXPECT_EQ(skeleton, nullptr);
}

TEST(UrdfParserErrors, InvalidUrdfReturnsNull)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Urdf;
  const auto skeleton
      = io::readSkeleton("dart://sample/urdf/test/invalid.urdf", options);
  EXPECT_EQ(skeleton, nullptr);
}

TEST(UrdfParserErrors, MissingMeshReturnsNull)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Urdf;
  const auto skeleton
      = io::readSkeleton("dart://sample/urdf/test/missing_mesh.urdf", options);
  EXPECT_EQ(skeleton, nullptr);
}

TEST(UrdfParserErrors, InvalidMeshReturnsNull)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Urdf;
  const auto skeleton
      = io::readSkeleton("dart://sample/urdf/test/invalid_mesh.urdf", options);
  EXPECT_EQ(skeleton, nullptr);
}

TEST(UrdfParserErrors, MissingPackageReturnsNull)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Urdf;
  const auto skeleton = io::readSkeleton(
      "dart://sample/urdf/test/missing_package.urdf", options);
  EXPECT_EQ(skeleton, nullptr);
}

TEST(UrdfParserErrors, EmptyStringReturnsNull)
{
  io::UrdfParser parser;
  const auto skeleton = parser.parseSkeletonString("", common::Uri());
  EXPECT_EQ(skeleton, nullptr);
}

TEST(UrdfParserErrors, GarbageStringReturnsNull)
{
  io::UrdfParser parser;
  const auto skeleton
      = parser.parseSkeletonString("this is not valid XML", common::Uri());
  EXPECT_EQ(skeleton, nullptr);
}

TEST(UrdfParserErrors, IncompleteXmlReturnsNull)
{
  io::UrdfParser parser;
  const auto skeleton
      = parser.parseSkeletonString("<foo><bar/></foo>", common::Uri());
  EXPECT_EQ(skeleton, nullptr);
}

TEST(UrdfParserErrors, ValidPrimitiveGeometryLoads)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Urdf;
  const auto skeleton = io::readSkeleton(
      "dart://sample/urdf/test/primitive_geometry.urdf", options);
  EXPECT_NE(skeleton, nullptr);
}

TEST(UrdfParserErrors, JointPropertiesLoads)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Urdf;
  const auto skeleton = io::readSkeleton(
      "dart://sample/urdf/test/joint_properties.urdf", options);
  EXPECT_NE(skeleton, nullptr);
}

#endif
