/*
 * Copyright (c) 2011-2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#include <assimp/cimport.h>

#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/FreeJoint.h"
#include "dart/dynamics/MeshShape.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/simulation/World.h"
#include "dart/utils/Paths.h"
#include "dart/utils/SkelParser.h"
#include "apps/meshCollision/MyWindow.h"

int main(int argc, char* argv[]) {
  using dart::dynamics::BodyNode;
  using dart::dynamics::FreeJoint;
  using dart::dynamics::MeshShape;
  using dart::dynamics::Skeleton;
  using dart::simulation::World;
  using dart::utils::SkelParser;

  // Create and initialize the world
  World* myWorld = dart::utils::SkelParser::readSkelFile(
                     DART_DATA_PATH"/skel/mesh_collision.skel");

  // Create a skeleton
  Skeleton* MeshSkel = new Skeleton("Mesh Skeleton");

  // Always set the root node ( 6DOF for rotation and translation )
  FreeJoint* joint;
  BodyNode* node;

  // Set the initial Rootnode that controls the position and orientation of the
  // whole robot
  node = new BodyNode("rootBodyNode");
  joint = new FreeJoint("rootJoint");

  // Add joint to the body node
  node->setParentJoint(joint);

  // Load a Mesh3DTriangle to save in Shape
  const aiScene* m3d = MeshShape::loadMesh(DART_DATA_PATH"/obj/foot.obj");

  //  Create Shape and assign it to node
  MeshShape* Shape0 = new MeshShape(Eigen::Vector3d(1.0, 1.0, 1.0), m3d);

  node->addVisualizationShape(Shape0);
  node->addCollisionShape(Shape0);
  node->setInertia(0.000416667, 0.000416667, 0.000416667);
  node->setMass(1.0);  // 1 Kg according to cube1.skel

  // Add node to Skel
  MeshSkel->addBodyNode(node);

  // Add MeshSkel to the world
  myWorld->addSkeleton(MeshSkel);

  // Verify that our skeleton has something inside :)
  std::printf("Our skeleton has %d nodes \n", MeshSkel->getNumBodyNodes());
  // std::printf("Our skeleton has %d joints \n", MeshSkel->getNumJoints());
  std::printf("Our skeleton has %d DOFs \n", MeshSkel->getNumGenCoords());

  MyWindow window;
  window.setWorld(myWorld);

  std::cout << "space bar: simulation on/off" << std::endl;
  std::cout << "'s': simulate one step" << std::endl;
  std::cout << "'p': playback/stop" << std::endl;
  std::cout << "'[' and ']': play one frame backward and forward" << std::endl;
  std::cout << "'v': visualization on/off" << std::endl;
  std::cout << "'1' and '2': programmed interaction" << std::endl;

  glutInit(&argc, argv);
  window.initWindow(640, 480, "meshCollision");
  glutMainLoop();

  aiReleaseImport(m3d);

  return 0;
}
