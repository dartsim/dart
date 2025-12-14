/*
 * Copyright (c) 2011-2025, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "helpers/GTestUtils.hpp"

#include "helpers/dynamics_helpers.hpp"

#include "dart/collision/All.hpp"
#include "dart/common/Macros.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/math/Geometry.hpp"
#include "dart/utils/SkelParser.hpp"

#include <gtest/gtest.h>

#include <iostream>
#include <string>
#include <utility>
#if DART_HAVE_BULLET
  #include "dart/collision/bullet/All.hpp"
#endif
#include "dart/collision/dart/DARTCollisionDetector.hpp"
#include "dart/collision/fcl/FCLCollisionDetector.hpp"
#include "dart/constraint/BallJointConstraint.hpp"
#include "dart/constraint/BoxedLcpConstraintSolver.hpp"
#include "dart/constraint/DantzigBoxedLcpSolver.hpp"
#include "dart/constraint/PgsBoxedLcpSolver.hpp"
#include "dart/constraint/RevoluteJointConstraint.hpp"
#include "dart/simulation/World.hpp"

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace simulation;
using namespace dart::test;

namespace {

class ScopedCollisionFactoryDisabler
{
public:
  using Factory = collision::CollisionDetector::Factory;
  using Creator = Factory::Creator;

  ScopedCollisionFactoryDisabler(std::string key, Creator restorer)
    : mFactory(collision::CollisionDetector::getFactory()),
      mKey(std::move(key)),
      mRestorer(std::move(restorer))
  {
    if (!mFactory || !mFactory->canCreate(mKey))
      return;

    mDisabled = true;
    mFactory->unregisterCreator(mKey);
  }

  ScopedCollisionFactoryDisabler(const ScopedCollisionFactoryDisabler&)
      = delete;
  ScopedCollisionFactoryDisabler& operator=(
      const ScopedCollisionFactoryDisabler&)
      = delete;

  ~ScopedCollisionFactoryDisabler()
  {
    if (mFactory && mDisabled && mRestorer)
      mFactory->registerCreator(mKey, mRestorer);
  }

  bool wasDisabled() const
  {
    return mDisabled;
  }

private:
  Factory* mFactory;
  std::string mKey;
  Creator mRestorer;
  bool mDisabled{false};
};

class TrackingSolver final : public WorldSolver
{
public:
  TrackingSolver(
      std::string name,
      std::vector<std::string>& callLog,
      std::optional<RigidSolverType> rigidSolverType = std::nullopt)
    : WorldSolver(std::move(name)),
      mCallLog(callLog),
      mRigidSolverType(std::move(rigidSolverType))
  {
  }

  std::optional<RigidSolverType> getRigidSolverType() const override
  {
    return mRigidSolverType;
  }

  void setTimeStep(double) override {}

  void step(World&, bool) override
  {
    mCallLog.push_back(mName + ".step");
  }

  void sync(World&) override
  {
    mCallLog.push_back(mName + ".sync");
  }

private:
  std::vector<std::string>& mCallLog;
  std::optional<RigidSolverType> mRigidSolverType;
};

class SolverTestWorld final : public World
{
public:
  using World::World;

  using World::addSolver;
  using World::getNumSolvers;
  using World::getSolverIndex;
  using World::moveSolver;
  using World::setSolverEnabled;
};

} // namespace

//==============================================================================
TEST(World, AddingAndRemovingSkeletons)
{
  // World
  WorldPtr world = World::create();

  //-------------------- Test World::removeSkeleton() ------------------------
  SkeletonPtr skeleton1 = createThreeLinkRobot(
      Eigen::Vector3d(1.0, 1.0, 1.0),
      DOF_X,
      Eigen::Vector3d(1.0, 1.0, 1.0),
      DOF_Y,
      Eigen::Vector3d(1.0, 1.0, 1.0),
      DOF_Z,
      false,
      false);

  SkeletonPtr skeleton2 = createThreeLinkRobot(
      Eigen::Vector3d(1.0, 1.0, 1.0),
      DOF_X,
      Eigen::Vector3d(1.0, 1.0, 1.0),
      DOF_Y,
      Eigen::Vector3d(1.0, 1.0, 1.0),
      DOF_Z,
      false,
      false);

  SkeletonPtr skeleton3 = createThreeLinkRobot(
      Eigen::Vector3d(1.0, 1.0, 1.0),
      DOF_X,
      Eigen::Vector3d(1.0, 1.0, 1.0),
      DOF_Y,
      Eigen::Vector3d(1.0, 1.0, 1.0),
      DOF_Z,
      false,
      false);

  SkeletonPtr skeleton4 = createThreeLinkRobot(
      Eigen::Vector3d(1.0, 1.0, 1.0),
      DOF_X,
      Eigen::Vector3d(1.0, 1.0, 1.0),
      DOF_Y,
      Eigen::Vector3d(1.0, 1.0, 1.0),
      DOF_Z,
      false,
      false);

  int nSteps = 20;

  // Empty world
  for (int i = 0; i < nSteps; ++i)
    world->step();

  EXPECT_FALSE(world->hasSkeleton(skeleton1));
  EXPECT_FALSE(world->hasSkeleton(skeleton2));
  EXPECT_FALSE(world->hasSkeleton(skeleton3));
  EXPECT_FALSE(world->hasSkeleton(skeleton4));

  // Add skeleton1, skeleton2
  world->addSkeleton(skeleton1);
  EXPECT_TRUE(world->hasSkeleton(skeleton1));
  world->addSkeleton(skeleton2);
  EXPECT_TRUE(world->hasSkeleton(skeleton2));
  EXPECT_TRUE(world->getNumSkeletons() == 2);
  for (int i = 0; i < nSteps; ++i)
    world->step();

  std::string s1name = skeleton1->getName();
  std::string s2name = skeleton2->getName();
  EXPECT_TRUE(skeleton1 == world->getSkeleton(s1name));
  EXPECT_TRUE(skeleton2 == world->getSkeleton(s2name));

  // Remove skeleton2
  world->removeSkeleton(skeleton2);
  EXPECT_TRUE(world->getNumSkeletons() == 1);
  for (int i = 0; i < nSteps; ++i)
    world->step();

  EXPECT_TRUE(skeleton1 == world->getSkeleton(s1name));
  EXPECT_FALSE(skeleton2 == world->getSkeleton(s2name));
  EXPECT_TRUE(world->getSkeleton(s2name) == nullptr);

  // Add skeleton3, skeleton4
  world->addSkeleton(skeleton3);
  EXPECT_TRUE(world->hasSkeleton(skeleton3));
  world->addSkeleton(skeleton4);
  EXPECT_TRUE(world->hasSkeleton(skeleton4));
  EXPECT_TRUE(world->getNumSkeletons() == 3);
  for (int i = 0; i < nSteps; ++i)
    world->step();

  std::string s3name = skeleton3->getName();
  std::string s4name = skeleton4->getName();

  EXPECT_TRUE(s3name == s2name);
  EXPECT_TRUE(skeleton3 == world->getSkeleton(s3name));
  EXPECT_TRUE(skeleton4 == world->getSkeleton(s4name));

  skeleton4->setName(skeleton1->getName());
  EXPECT_FALSE(skeleton4->getName() == skeleton1->getName());

  // Remove skeleton1
  world->removeSkeleton(skeleton1);
  EXPECT_TRUE(world->getNumSkeletons() == 2);
  for (int i = 0; i < nSteps; ++i)
    world->step();

  EXPECT_FALSE(skeleton1 == world->getSkeleton(s1name));
  EXPECT_TRUE(world->getSkeleton(s1name) == nullptr);

  // Remove all the skeletons
  world->removeAllSkeletons();
  EXPECT_EQ((int)world->getNumSkeletons(), 0);
  for (int i = 0; i < nSteps; ++i)
    world->step();

  EXPECT_FALSE(skeleton3 == world->getSkeleton(s3name));
  EXPECT_TRUE(world->getSkeleton(s3name) == nullptr);

  EXPECT_FALSE(skeleton4 == world->getSkeleton(s4name));
  EXPECT_TRUE(world->getSkeleton(s4name) == nullptr);

  // An error will be thrown here if Skeletons are not being removed correctly
  skeleton1->setName(skeleton4->getName());
}

//==============================================================================
TEST(World, Cloning)
{
  // Create a list of skel files to test with
  std::vector<common::Uri> fileList;
  fileList.push_back("dart://sample/skel/test/chainwhipa.skel");
  fileList.push_back("dart://sample/skel/test/single_pendulum.skel");
  fileList.push_back(
      "dart://sample/skel/test/single_pendulum_euler_joint.skel");
  fileList.push_back("dart://sample/skel/test/single_pendulum_ball_joint.skel");
  fileList.push_back("dart://sample/skel/test/double_pendulum.skel");
  fileList.push_back(
      "dart://sample/skel/test/double_pendulum_euler_joint.skel");
  fileList.push_back("dart://sample/skel/test/double_pendulum_ball_joint.skel");
  fileList.push_back(
      "dart://sample/skel/test/serial_chain_revolute_joint.skel");
  fileList.push_back(
      "dart://sample/skel/test/serial_chain_eulerxyz_joint.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_ball_joint.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_ball_joint_20.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_ball_joint_40.skel");
  fileList.push_back("dart://sample/skel/test/simple_tree_structure.skel");
  fileList.push_back(
      "dart://sample/skel/test/simple_tree_structure_euler_joint.skel");
  fileList.push_back(
      "dart://sample/skel/test/simple_tree_structure_ball_joint.skel");
  fileList.push_back("dart://sample/skel/test/tree_structure.skel");
  fileList.push_back("dart://sample/skel/test/tree_structure_euler_joint.skel");
  fileList.push_back("dart://sample/skel/test/tree_structure_ball_joint.skel");
  fileList.push_back("dart://sample/skel/fullbody1.skel");

  std::vector<dart::simulation::WorldPtr> worlds;
  for (std::size_t i = 0; i < fileList.size(); ++i)
    worlds.push_back(utils::SkelParser::readWorld(fileList[i]));

  for (std::size_t i = 0; i < worlds.size(); ++i) {
    dart::simulation::WorldPtr original = worlds[i];
    std::vector<dart::simulation::WorldPtr> clones;
    clones.push_back(original);
    for (std::size_t j = 1; j < 5; ++j)
      clones.push_back(clones[j - 1]->clone());

#if !defined(NDEBUG)
    std::size_t numIterations = 3;
#else
    std::size_t numIterations = 500;
#endif

    for (std::size_t j = 0; j < numIterations; ++j) {
      for (std::size_t k = 0; k < original->getNumSkeletons(); ++k) {
        dart::dynamics::SkeletonPtr skel = original->getSkeleton(k);

        // Generate a random command vector
        Eigen::VectorXd commands = skel->getCommands();
        for (int q = 0; q < commands.size(); ++q)
          commands[q] = Random::uniform(-0.1, 0.1);

        // Assign the command vector to each clone of the kth skeleton
        for (std::size_t c = 0; c < clones.size(); ++c) {
          dart::dynamics::SkeletonPtr skelClone = clones[c]->getSkeleton(k);
          skelClone->setCommands(commands);
        }
      }

      // Step each clone forward
      for (std::size_t c = 0; c < clones.size(); ++c)
        clones[c]->step(false);
    }

    for (std::size_t c = 0; c < clones.size(); ++c) {
      for (std::size_t k = 0; k < original->getNumSkeletons(); ++k) {
        dart::dynamics::SkeletonPtr skel = original->getSkeleton(k);
        dart::dynamics::SkeletonPtr clone = clones[c]->getSkeleton(k);

        EXPECT_TRUE(equals(skel->getPositions(), clone->getPositions(), 0));
        EXPECT_TRUE(equals(skel->getVelocities(), clone->getVelocities(), 0));
        EXPECT_TRUE(
            equals(skel->getAccelerations(), clone->getAccelerations(), 0));
        EXPECT_TRUE(equals(skel->getForces(), clone->getForces(), 0));
      }
    }
  }
}

//==============================================================================
TEST(World, ValidatingClones)
{
  // Create a list of skel files to test with
  std::vector<common::Uri> fileList;
  fileList.push_back("dart://sample/skel/test/chainwhipa.skel");
  fileList.push_back("dart://sample/skel/test/single_pendulum.skel");
  fileList.push_back(
      "dart://sample/skel/test/single_pendulum_euler_joint.skel");
  fileList.push_back("dart://sample/skel/test/single_pendulum_ball_joint.skel");
  fileList.push_back("dart://sample/skel/test/double_pendulum.skel");
  fileList.push_back(
      "dart://sample/skel/test/double_pendulum_euler_joint.skel");
  fileList.push_back("dart://sample/skel/test/double_pendulum_ball_joint.skel");
  fileList.push_back(
      "dart://sample/skel/test/serial_chain_revolute_joint.skel");
  fileList.push_back(
      "dart://sample/skel/test/serial_chain_eulerxyz_joint.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_ball_joint.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_ball_joint_20.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_ball_joint_40.skel");
  fileList.push_back("dart://sample/skel/test/simple_tree_structure.skel");
  fileList.push_back(
      "dart://sample/skel/test/simple_tree_structure_euler_joint.skel");
  fileList.push_back(
      "dart://sample/skel/test/simple_tree_structure_ball_joint.skel");
  fileList.push_back("dart://sample/skel/test/tree_structure.skel");
  fileList.push_back("dart://sample/skel/test/tree_structure_euler_joint.skel");
  fileList.push_back("dart://sample/skel/test/tree_structure_ball_joint.skel");
  fileList.push_back("dart://sample/skel/fullbody1.skel");

  std::vector<dart::simulation::WorldPtr> worlds;
  for (std::size_t i = 0; i < fileList.size(); ++i) {
    worlds.push_back(utils::SkelParser::readWorld(fileList[i]));

    // Set non default collision detector
#if DART_HAVE_BULLET
    worlds.back()->setCollisionDetector(CollisionDetectorType::Bullet);
#else
    worlds.back()->setCollisionDetector(CollisionDetectorType::Dart);
#endif
  }

  for (std::size_t i = 0; i < worlds.size(); ++i) {
    dart::simulation::WorldPtr original = worlds[i];
    std::vector<dart::simulation::WorldPtr> clones;
    clones.push_back(original);
    for (std::size_t j = 1; j < 5; ++j) {
      clones.push_back(clones[j - 1]->clone());

      auto originalCD = original->getCollisionDetector();
      auto cloneCD = clones.back()->getCollisionDetector();

      std::string originalCDType = originalCD->getType();
      std::string cloneCDType = cloneCD->getType();

      EXPECT_EQ(originalCDType, cloneCDType);
    }
  }
}

//==============================================================================
TEST(World, SetCollisionDetectorByType)
{
  auto factory = collision::CollisionDetector::getFactory();
  ASSERT_NE(factory, nullptr);

  if (!factory->canCreate("dart"))
    GTEST_SKIP() << "dart collision detector is not available in this build";

  auto world = World::create();
  world->setCollisionDetector(CollisionDetectorType::Dart);

  ASSERT_TRUE(world->getCollisionDetector());
  EXPECT_EQ(world->getCollisionDetector()->getType(), "dart");
}

//==============================================================================
TEST(World, ConfiguresCollisionDetectorViaConfig)
{
  auto factory = collision::CollisionDetector::getFactory();
  ASSERT_NE(factory, nullptr);

  if (!factory->canCreate("dart"))
    GTEST_SKIP() << "dart collision detector is not available in this build";

  WorldConfig config;
  config.name = "configured-world";
  config.collisionDetector = CollisionDetectorType::Dart;
  auto world = World::create(config);
  ASSERT_TRUE(world->getCollisionDetector());
  EXPECT_EQ(world->getCollisionDetector()->getType(), "dart");
}

//==============================================================================
TEST(World, DefaultWorldUsesFclMeshPrimitive)
{
  auto factory = collision::CollisionDetector::getFactory();
  ASSERT_NE(factory, nullptr);

  if (!factory->canCreate("fcl"))
    GTEST_SKIP() << "fcl collision detector is not available in this build";

  auto world = World::create();
  auto fclDetector = std::dynamic_pointer_cast<collision::FCLCollisionDetector>(
      world->getCollisionDetector());
  ASSERT_TRUE(fclDetector);
  EXPECT_EQ(
      fclDetector->getPrimitiveShapeType(),
      collision::FCLCollisionDetector::MESH);
}

//==============================================================================
TEST(World, TypedSetterConfiguresFclMeshPrimitive)
{
  auto factory = collision::CollisionDetector::getFactory();
  ASSERT_NE(factory, nullptr);

  if (!factory->canCreate("fcl"))
    GTEST_SKIP() << "fcl collision detector is not available in this build";

  auto world = World::create();
  world->setCollisionDetector(CollisionDetectorType::Dart);
  world->setCollisionDetector(CollisionDetectorType::Fcl);

  auto fclDetector = std::dynamic_pointer_cast<collision::FCLCollisionDetector>(
      world->getCollisionDetector());
  ASSERT_TRUE(fclDetector);
  EXPECT_EQ(
      fclDetector->getPrimitiveShapeType(),
      collision::FCLCollisionDetector::MESH);
}

//==============================================================================
TEST(World, TypedSetterFallsBackWhenDetectorUnavailable)
{
  ScopedCollisionFactoryDisabler disableDart(
      collision::DARTCollisionDetector::getStaticType(),
      []() -> collision::CollisionDetectorPtr {
        return collision::DARTCollisionDetector::create();
      });

  if (!disableDart.wasDisabled())
    GTEST_SKIP() << "dart collision detector is not registered in this build";

  auto world = World::create();
  auto original = world->getCollisionDetector();
  ASSERT_TRUE(original);

  world->setCollisionDetector(CollisionDetectorType::Dart);

  auto current = world->getCollisionDetector();
  ASSERT_TRUE(current);
  EXPECT_EQ(current->getType(), original->getType());
}

//==============================================================================
TEST(World, ConfigFallbacksWhenPreferredDetectorUnavailable)
{
  ScopedCollisionFactoryDisabler disableDart(
      collision::DARTCollisionDetector::getStaticType(),
      []() -> collision::CollisionDetectorPtr {
        return collision::DARTCollisionDetector::create();
      });

  if (!disableDart.wasDisabled())
    GTEST_SKIP() << "dart collision detector is not registered in this build";

  WorldConfig config;
  config.name = "fallback-pref";
  config.collisionDetector = CollisionDetectorType::Dart;

  auto world = World::create(config);
  ASSERT_TRUE(world->getCollisionDetector());
  EXPECT_EQ(
      world->getCollisionDetector()->getType(),
      collision::FCLCollisionDetector::getStaticType());
}

//==============================================================================
TEST(World, ConfigWarnsWhenPreferredAndFallbackUnavailable)
{
  ScopedCollisionFactoryDisabler disableDart(
      collision::DARTCollisionDetector::getStaticType(),
      []() -> collision::CollisionDetectorPtr {
        return collision::DARTCollisionDetector::create();
      });

  if (!disableDart.wasDisabled())
    GTEST_SKIP() << "dart collision detector is not registered in this build";

  ScopedCollisionFactoryDisabler disableFcl(
      collision::FCLCollisionDetector::getStaticType(),
      []() -> collision::CollisionDetectorPtr {
        return collision::FCLCollisionDetector::create();
      });

  if (!disableFcl.wasDisabled())
    GTEST_SKIP() << "fcl collision detector is not registered in this build";

  WorldConfig config;
  config.name = "no-fallback-world";
  config.collisionDetector = CollisionDetectorType::Dart;

  auto world = World::create(config);
  ASSERT_TRUE(world->getCollisionDetector());
  EXPECT_EQ(
      world->getCollisionDetector()->getType(),
      collision::FCLCollisionDetector::getStaticType());
}

//==============================================================================
simulation::WorldPtr createWorld()
{
  // Create and initialize the world
  simulation::WorldPtr world
      = utils::SkelParser::readWorld("dart://sample/skel/chain.skel");
  DART_ASSERT(world != nullptr);

  // Create and initialize the world
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  world->setTimeStep(1.0 / 2000);

  const auto dof = world->getSkeleton(0)->getNumDofs();
  Eigen::VectorXd initPose = Eigen::VectorXd::Zero(static_cast<int>(dof));
  initPose[20] = 3.14159 * 0.4;
  initPose[23] = 3.14159 * 0.4;
  initPose[26] = 3.14159 * 0.4;
  initPose[29] = 3.14159 * 0.4;
  world->getSkeleton(0)->setPositions(initPose);

  // Create a ball joint constraint
  BodyNode* bd1 = world->getSkeleton(0)->getBodyNode("link 6");
  BodyNode* bd2 = world->getSkeleton(0)->getBodyNode("link 10");
  EXPECT_TRUE(bd1 != nullptr);
  EXPECT_TRUE(bd2 != nullptr);
  const Eigen::Vector3d offset(0.0, 0.025, 0.0);
  const Eigen::Vector3d jointPos = bd1->getTransform() * offset;
  auto cl
      = std::make_shared<constraint::BallJointConstraint>(bd1, bd2, jointPos);
  world->getConstraintSolver()->addConstraint(cl);

  return world;
}

//==============================================================================
simulation::WorldPtr createWorldWithRevoluteConstraint()
{
  simulation::WorldPtr world
      = utils::SkelParser::readWorld("dart://sample/skel/chain.skel");
  DART_ASSERT(world != nullptr);

  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  world->setTimeStep(1.0 / 2000);

  const auto dof = world->getSkeleton(0)->getNumDofs();
  Eigen::VectorXd initPose = Eigen::VectorXd::Zero(static_cast<int>(dof));
  initPose[20] = 3.14159 * 0.4;
  initPose[23] = 3.14159 * 0.4;
  initPose[26] = 3.14159 * 0.4;
  initPose[29] = 3.14159 * 0.4;
  world->getSkeleton(0)->setPositions(initPose);

  BodyNode* bd1 = world->getSkeleton(0)->getBodyNode("link 6");
  BodyNode* bd2 = world->getSkeleton(0)->getBodyNode("link 10");
  EXPECT_TRUE(bd1 != nullptr);
  EXPECT_TRUE(bd2 != nullptr);

  const Eigen::Vector3d offset(0.0, 0.025, 0.0);
  const Eigen::Vector3d jointPos = bd1->getTransform() * offset;

  auto hinge = std::make_shared<constraint::RevoluteJointConstraint>(
      bd1, bd2, jointPos, Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitY());
  world->getConstraintSolver()->addConstraint(hinge);

  return world;
}

//==============================================================================
TEST(World, SetNewConstraintSolver)
{
  auto world = createWorld();
  EXPECT_TRUE(world->getConstraintSolver()->getSkeletons().size() == 1);
  EXPECT_TRUE(world->getConstraintSolver()->getNumConstraints() == 1);

  auto solver1 = std::make_unique<constraint::BoxedLcpConstraintSolver>(
      std::make_shared<constraint::DantzigBoxedLcpSolver>());
  EXPECT_TRUE(solver1->getSkeletons().size() == 0);
  EXPECT_TRUE(solver1->getNumConstraints() == 0);

  world->setConstraintSolver(std::move(solver1));
  EXPECT_TRUE(world->getConstraintSolver()->getSkeletons().size() == 1);
  EXPECT_TRUE(world->getConstraintSolver()->getNumConstraints() == 1);

  auto solver2 = std::make_unique<constraint::BoxedLcpConstraintSolver>(
      std::make_shared<constraint::PgsBoxedLcpSolver>());
  EXPECT_TRUE(solver2->getSkeletons().size() == 0);
  EXPECT_TRUE(solver2->getNumConstraints() == 0);

  world->setConstraintSolver(std::move(solver2));
  EXPECT_TRUE(world->getConstraintSolver()->getSkeletons().size() == 1);
  EXPECT_TRUE(world->getConstraintSolver()->getNumConstraints() == 1);
}

//==============================================================================
TEST(World, RevoluteJointConstraintBasics)
{
  auto world = createWorldWithRevoluteConstraint();
  ASSERT_TRUE(world);
  ASSERT_EQ(world->getConstraintSolver()->getNumConstraints(), 1);

  auto* bd1 = world->getSkeleton(0)->getBodyNode("link 6");
  auto* bd2 = world->getSkeleton(0)->getBodyNode("link 10");
  const Eigen::Vector3d offset(0.0, 0.025, 0.0);

  for (int i = 0; i < 200; ++i)
    world->step();

  const Eigen::Vector3d pos1 = bd1->getTransform() * offset;
  const Eigen::Vector3d pos2 = bd2->getTransform() * offset;
  EXPECT_LT((pos1 - pos2).norm(), 5e-2);

  const Eigen::Vector3d axis1
      = bd1->getTransform().linear() * Eigen::Vector3d::UnitY();
  const Eigen::Vector3d axis2
      = bd2->getTransform().linear() * Eigen::Vector3d::UnitY();
  EXPECT_GT(axis1.normalized().dot(axis2.normalized()), 0.2);
}

//==============================================================================
TEST(World, SolverSteppingActiveRigidSolverOnlyOrdersNonRigidAndSync)
{
  auto world = std::make_shared<SolverTestWorld>();
  ASSERT_TRUE(world);

  for (std::size_t i = 0; i < world->getNumSolvers(); ++i)
    EXPECT_TRUE(world->setSolverEnabled(i, false));

  std::vector<std::string> callLog;
  auto* nonRigid
      = world->addSolver(std::make_unique<TrackingSolver>("nonrigid", callLog));
  auto* mirrorRigid = world->addSolver(std::make_unique<TrackingSolver>(
      "mirror", callLog, RigidSolverType::EntityComponent));
  auto* activeRigid = world->addSolver(std::make_unique<TrackingSolver>(
      "active", callLog, RigidSolverType::ClassicSkeleton));

  ASSERT_TRUE(nonRigid);
  ASSERT_TRUE(mirrorRigid);
  ASSERT_TRUE(activeRigid);

  auto moveToIndex = [&](WorldSolver* solver, std::size_t target) {
    const auto index = world->getSolverIndex(solver);
    ASSERT_LT(index, world->getNumSolvers());
    ASSERT_TRUE(world->moveSolver(index, target));
  };

  moveToIndex(nonRigid, 0);
  moveToIndex(mirrorRigid, 1);
  moveToIndex(activeRigid, 2);

  ASSERT_TRUE(world->setActiveRigidSolver(RigidSolverType::ClassicSkeleton));
  world->setSolverSteppingMode(SolverSteppingMode::ActiveRigidSolverOnly);

  callLog.clear();
  world->step(false);

  const std::vector<std::string> expected{
      "active.step",
      "nonrigid.step",
      "mirror.sync",
  };
  EXPECT_EQ(callLog, expected);
}
