/*
 * Copyright (c) 2011-2023, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <dart/simulation/simulation.hpp>

#include <dart/io/io.hpp>

#include <dart/dynamics/dynamics.hpp>

#include <benchmark/benchmark.h>

#include <chrono>
#include <numeric>

using namespace dart;

std::vector<std::string> getSceneFiles()
{
  std::vector<std::string> scenes;
  scenes.push_back("dart://sample/skel/test/chainwhipa.skel");
  scenes.push_back("dart://sample/skel/test/single_pendulum.skel");
  scenes.push_back("dart://sample/skel/test/single_pendulum_euler_joint.skel");
  scenes.push_back("dart://sample/skel/test/single_pendulum_ball_joint.skel");
  scenes.push_back("dart://sample/skel/test/double_pendulum.skel");
  scenes.push_back("dart://sample/skel/test/double_pendulum_euler_joint.skel");
  scenes.push_back("dart://sample/skel/test/double_pendulum_ball_joint.skel");
  scenes.push_back("dart://sample/skel/test/serial_chain_revolute_joint.skel");
  scenes.push_back("dart://sample/skel/test/serial_chain_eulerxyz_joint.skel");
  scenes.push_back("dart://sample/skel/test/serial_chain_ball_joint.skel");
  scenes.push_back("dart://sample/skel/test/serial_chain_ball_joint_20.skel");
  scenes.push_back("dart://sample/skel/test/serial_chain_ball_joint_40.skel");
  scenes.push_back("dart://sample/skel/test/simple_tree_structure.skel");
  scenes.push_back(
      "dart://sample/skel/test/simple_tree_structure_euler_joint.skel");
  scenes.push_back(
      "dart://sample/skel/test/simple_tree_structure_ball_joint.skel");
  scenes.push_back("dart://sample/skel/test/tree_structure.skel");
  scenes.push_back("dart://sample/skel/test/tree_structure_euler_joint.skel");
  scenes.push_back("dart://sample/skel/test/tree_structure_ball_joint.skel");
  scenes.push_back("dart://sample/skel/fullbody1.skel");

  return scenes;
}

std::vector<dart::simulation::WorldPtr> getWorlds()
{
  std::vector<std::string> sceneFiles = getSceneFiles();
  std::vector<dart::simulation::WorldPtr> worlds;
  for (std::size_t i = 0; i < sceneFiles.size(); ++i)
    worlds.push_back(dart::io::SkelParser::readWorld(sceneFiles[i]));

  return worlds;
}

void testForwardKinematicSpeed(
    dart::dynamics::SkeletonPtr skel,
    bool position,
    bool velocity,
    bool acceleration,
    std::size_t numTests)
{
  if (nullptr == skel)
    return;

  dart::dynamics::BodyNode* bn = skel->getBodyNode(0);
  while (bn->getNumChildBodyNodes() > 0)
    bn = bn->getChildBodyNode(0);

  for (std::size_t i = 0; i < numTests; ++i)
  {
    for (std::size_t i = 0; i < skel->getNumDofs(); ++i)
    {
      dart::dynamics::DegreeOfFreedom* dof = skel->getDof(i);
      dof->setPosition(dart::math::Random::uniform(
          std::max(dof->getPositionLowerLimit(), -1.0),
          std::min(dof->getPositionUpperLimit(), 1.0)));
    }

    for (std::size_t i = 0; i < skel->getNumBodyNodes(); ++i)
    {
      if (position)
        skel->getBodyNode(i)->getWorldTransform();
      if (velocity)
      {
        skel->getBodyNode(i)->getSpatialVelocity();
        skel->getBodyNode(i)->getPartialAcceleration();
      }
      if (acceleration)
        skel->getBodyNode(i)->getSpatialAcceleration();
    }
  }
}

void runKinematicsTest(
    const std::vector<dart::simulation::WorldPtr>& worlds,
    bool position,
    bool velocity,
    bool acceleration,
    std::size_t numTests)
{
  // Test for updating the whole skeleton
  for (std::size_t i = 0; i < worlds.size(); ++i)
  {
    dart::simulation::WorldPtr world = worlds[i];
    testForwardKinematicSpeed(
        world->getSkeleton(0), position, velocity, acceleration, numTests);
  }
}

static void BM_Kinematics(benchmark::State& state)
{
  std::vector<dart::simulation::WorldPtr> worlds = getWorlds();

  // Get the input value to be passed to the Kinematics function
  int n = state.range(0);

  // Call the Kinematics function and measure the time it takes
  for (auto _ : state)
  {
    runKinematicsTest(worlds, true, true, true, n);
  }
}

BENCHMARK(BM_Kinematics)->Arg(1)->Arg(10)->Arg(100)->Arg(1000)->Arg(10000);

void testDynamicsSpeed(
    dart::simulation::WorldPtr world, std::size_t numIterations)
{
  if (nullptr == world)
    return;

  world->eachSkeleton([](dart::dynamics::Skeleton* skel) {
    skel->resetPositions();
    skel->resetVelocities();
    skel->resetAccelerations();
  });

  for (std::size_t i = 0; i < numIterations; ++i)
    world->step();
}

void runDynamicsTest(
    const std::vector<dart::simulation::WorldPtr>& worlds,
    std::size_t numIterations)
{
  for (std::size_t i = 0; i < worlds.size(); ++i)
    testDynamicsSpeed(worlds[i], numIterations);
}

static void BM_Dynamics(benchmark::State& state)
{
  std::vector<dart::simulation::WorldPtr> worlds = getWorlds();

  // Get the input value to be passed to the Kinematics function
  int n = state.range(0);

  // Call the Kinematics function and measure the time it takes
  for (auto _ : state)
  {
    runDynamicsTest(worlds, n);
  }
}

BENCHMARK(BM_Dynamics)->Arg(1)->Arg(10)->Arg(100)->Arg(1000)->Arg(10000);
