// Copyright (c) 2011-2025, The DART development contributors

#include <dart/utils/file_info_dof.hpp>

#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

using namespace dart;
using namespace dart::dynamics;
using namespace dart::utils;

namespace {

std::filesystem::path makeTempPath(const std::string& suffix)
{
  const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  return std::filesystem::temp_directory_path()
         / ("dart_file_info_dof_" + std::to_string(now) + suffix);
}

} // namespace

TEST(FileInfoDof, SaveAndLoadRoundTrip)
{
  auto skeleton = Skeleton::create("dof_skeleton");
  skeleton->createJointAndBodyNodePair<FreeJoint>();

  const auto dofCount = skeleton->getNumDofs();
  Eigen::VectorXd frame = Eigen::VectorXd::LinSpaced(
      static_cast<int>(dofCount), 1.0, static_cast<double>(dofCount));

  FileInfoDof writer(skeleton.get(), 120.0);
  writer.addDof(frame);

  const auto path = makeTempPath(".dof");
  EXPECT_TRUE(writer.saveFile(path.string().c_str(), 0, 0, 0.0));

  FileInfoDof reader(skeleton.get(), 1.0);
  EXPECT_TRUE(reader.loadFile(path.string().c_str()));
  EXPECT_EQ(reader.getNumFrames(), 1);
  EXPECT_DOUBLE_EQ(reader.getDofAt(0, 0), frame[0]);
  EXPECT_DOUBLE_EQ(reader.getFPS(), 120.0);

  std::filesystem::remove(path);
}

TEST(FileInfoDof, SaveRejectsInvalidRange)
{
  auto skeleton = Skeleton::create("dof_skeleton_range");
  skeleton->createJointAndBodyNodePair<FreeJoint>();

  FileInfoDof writer(skeleton.get(), 60.0);
  EXPECT_FALSE(writer.saveFile("/tmp/does_not_matter.dof", 2, 1, 0.0));
}

TEST(FileInfoDof, LoadRejectsMismatchedDofs)
{
  auto skeleton = Skeleton::create("dof_skeleton_mismatch");
  skeleton->createJointAndBodyNodePair<FreeJoint>();

  const auto path = makeTempPath("_bad.dof");
  {
    std::ofstream out(path);
    out << "frames = 1 dofs = 2\n";
    out << "joint0.0 joint0.1\n";
    out << "0 0\n";
    out << "FPS 60\n";
  }

  FileInfoDof reader(skeleton.get(), 30.0);
  EXPECT_FALSE(reader.loadFile(path.string().c_str()));

  std::filesystem::remove(path);
}
