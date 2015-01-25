
/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#include <chrono>

#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/DegreeOfFreedom.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Joint.h"
#include "dart/simulation/World.h"
#include "dart/utils/SkelParser.h"
#include "dart/math/Helpers.h"
#include "dart/config.h"

double testForwardKinematicSpeed(dart::dynamics::Skeleton* skel,
                                 bool whole_skeleton=true,
                                 bool position=true,
                                 bool velocity=true,
                                 bool acceleration=true,
                                 size_t numTests=100000)
{
  if(NULL==skel)
    return 0;

  dart::dynamics::BodyNode* bn = skel->getBodyNode(0);
  while(bn->getNumChildBodyNodes() > 0)
    bn = bn->getChildBodyNode(0);

  std::chrono::time_point<std::chrono::system_clock> start, end;
  start = std::chrono::system_clock::now();

  for(size_t i=0; i<numTests; ++i)
  {
    for(size_t i=0; i<skel->getNumDofs(); ++i)
    {
      dart::dynamics::DegreeOfFreedom* dof = skel->getDof(i);
      dof->setPosition( dart::math::random(
                          std::max(dof->getPositionLowerLimit(),-1.0),
                          std::min(dof->getPositionUpperLimit(), 1.0)) );
    }

//    skel->computeForwardKinematics();

    if(whole_skeleton)
    {
      for(size_t i=0; i<skel->getNumBodyNodes(); ++i)
      {
        if(position)
          skel->getBodyNode(i)->getWorldTransform();
        if(velocity)
          skel->getBodyNode(i)->getSpatialVelocity();
        if(acceleration)
          skel->getBodyNode(i)->getSpatialAcceleration();
      }
    }
    else
    {
      if(position)
        bn->getWorldTransform();
      if(velocity)
        bn->getSpatialVelocity();
      if(acceleration)
        bn->getSpatialAcceleration();
    }
  }

  end = std::chrono::system_clock::now();

  std::chrono::duration<double> elapsed_seconds = end-start;
  return elapsed_seconds.count();
}

std::vector<std::string> getSceneFiles()
{
  std::vector<std::string> scenes;
  scenes.push_back(DART_DATA_PATH"skel/test/chainwhipa.skel");
  scenes.push_back(DART_DATA_PATH"skel/test/single_pendulum.skel");
  scenes.push_back(DART_DATA_PATH"skel/test/single_pendulum_euler_joint.skel");
  scenes.push_back(DART_DATA_PATH"skel/test/single_pendulum_ball_joint.skel");
  scenes.push_back(DART_DATA_PATH"skel/test/double_pendulum.skel");
  scenes.push_back(DART_DATA_PATH"skel/test/double_pendulum_euler_joint.skel");
  scenes.push_back(DART_DATA_PATH"skel/test/double_pendulum_ball_joint.skel");
  scenes.push_back(DART_DATA_PATH"skel/test/serial_chain_revolute_joint.skel");
  scenes.push_back(DART_DATA_PATH"skel/test/serial_chain_eulerxyz_joint.skel");
  scenes.push_back(DART_DATA_PATH"skel/test/serial_chain_ball_joint.skel");
  scenes.push_back(DART_DATA_PATH"skel/test/serial_chain_ball_joint_20.skel");
  scenes.push_back(DART_DATA_PATH"skel/test/serial_chain_ball_joint_40.skel");
  scenes.push_back(DART_DATA_PATH"skel/test/simple_tree_structure.skel");
  scenes.push_back(DART_DATA_PATH"skel/test/simple_tree_structure_euler_joint.skel");
  scenes.push_back(DART_DATA_PATH"skel/test/simple_tree_structure_ball_joint.skel");
  scenes.push_back(DART_DATA_PATH"skel/test/tree_structure.skel");
  scenes.push_back(DART_DATA_PATH"skel/test/tree_structure_euler_joint.skel");
  scenes.push_back(DART_DATA_PATH"skel/test/tree_structure_ball_joint.skel");
  scenes.push_back(DART_DATA_PATH"skel/fullbody1.skel");

  return scenes;
}

std::vector<dart::simulation::World*> getWorlds()
{
  std::vector<std::string> sceneFiles = getSceneFiles();
  std::vector<dart::simulation::World*> worlds;
  for(size_t i=0; i<sceneFiles.size(); ++i)
  {
    worlds.push_back(dart::utils::SkelParser::readWorld(sceneFiles[i]));
  }

  return worlds;
}

void runTest(std::vector<double>& wholebody_results,
             std::vector<double>& specificBN_results,
             const std::vector<dart::simulation::World*>& worlds,
             bool position, bool velocity, bool acceleration)
{
  double totalTime = 0;
  std::cout << "Testing: ";
  if(position)
    std::cout << "Position ";
  if(velocity)
    std::cout << "Velocity ";
  if(acceleration)
    std::cout << "Acceleration ";
  std::cout << "\n";

  // Test for updating the whole skeleton
  for(size_t i=0; i<worlds.size(); ++i)
  {
    dart::simulation::World* world = worlds[i];
    totalTime += testForwardKinematicSpeed(world->getSkeleton(0), true,
                                        position, velocity, acceleration);
  }
  wholebody_results.push_back(totalTime);
  std::cout << "Whole skeleton: " << totalTime << "s" << std::endl;

  // Test for updating a specific BodyNode
  totalTime = 0;
  for(size_t i=0; i<worlds.size(); ++i)
  {
    dart::simulation::World* world = worlds[i];
    totalTime += testForwardKinematicSpeed(world->getSkeleton(0), false,
                                        position, velocity, acceleration);
  }
  specificBN_results.push_back(totalTime);
  std::cout << "Specific BodyNode: " << totalTime << "s" << std::endl;
}

void print_results(const std::vector<double>& result)
{
  double sum = std::accumulate(result.begin(), result.end(), 0.0);
  double mean = sum/result.size();
  std::cout << "Average: " << mean << "\n";
  std::vector<double> diff(result.size());
  std::transform(result.begin(), result.end(), diff.begin(),
                 std::bind2nd(std::minus<double>(), mean));
  double stddev = std::sqrt(std::inner_product(diff.begin(), diff.end(),
                                        diff.begin(), 0.0)/result.size());
  std::cout << "Std Dev: " << stddev << "\n";
}

int main()
{
  std::vector<dart::simulation::World*> worlds = getWorlds();

  std::vector<double> wb_results_acceleration;
  std::vector<double> bn_results_acceleration;

  std::vector<double> wb_results_velocity;
  std::vector<double> bn_results_velocity;

  std::vector<double> wb_results_position;
  std::vector<double> bn_results_position;

  for(size_t i=0; i<10; ++i)
  {
    std::cout << "Test #" << i << std::endl;
    runTest(wb_results_acceleration, bn_results_acceleration,
            worlds, true, true, true);
    runTest(wb_results_velocity, bn_results_velocity,
            worlds, true, true, false);
    runTest(wb_results_position, bn_results_position,
            worlds, true, false, false);
  }

  std::cout << "\n\n --- Final Results --- \n\n";

  std::cout << "Position, Velocity, Acceleration\n";
  std::cout << "Whole Body\n";
  print_results(wb_results_acceleration);
  std::cout << "Specific BodyNode\n";
  print_results(bn_results_acceleration);

  std::cout << "\nPosition, Velocity\n";
  std::cout << "Whole Body\n";
  print_results(wb_results_velocity);
  std::cout << "Specific BodyNode\n";
  print_results(bn_results_velocity);

  std::cout << "\nPosition\n";
  std::cout << "Whole Body\n";
  print_results(wb_results_position);
  std::cout << "Specific BodyNode\n";
  print_results(bn_results_position);
}
