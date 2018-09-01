/*
 * Copyright (c) 2011-2018, The DART development contributors
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

/*
 * Copyright (C) 2018 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cmath>

#include <dart/dart.hpp>
#include <dart/external/imgui/imgui.h>
#include <dart/gui/filament/filament.hpp>
#include <dart/simulation/World.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>
#include <filament/Engine.h>
#include <filament/IndexBuffer.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/RenderableManager.h>
#include <filament/Scene.h>
#include <filament/TransformManager.h>
#include <filament/VertexBuffer.h>
#include <filament/View.h>

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;
using namespace dart::math;
using namespace dart::gui;
using namespace dart::gui::flmt;

using namespace filament;
using ::utils::Entity;

SkeletonPtr createAtlas()
{
  // Parse in the atlas model
  dart::utils::DartLoader urdf;
  SkeletonPtr atlas
      = urdf.parseSkeleton("dart://sample/sdf/atlas/atlas_v3_no_head.urdf");

  // Add a box to the root node to make it easier to click and drag
  double scale = 0.25;
  ShapePtr boxShape
      = std::make_shared<BoxShape>(scale * Eigen::Vector3d(1.0, 1.0, 0.5));

  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0.1 * Eigen::Vector3d(0.0, 0.0, 1.0));

  auto shapeNode
      = atlas->getBodyNode(0)->createShapeNodeWith<VisualAspect>(boxShape);
  shapeNode->getVisualAspect()->setColor(dart::Color::Black());
  shapeNode->setRelativeTransform(tf);

  return atlas;
}

SkeletonPtr createGround(const Eigen::Vector3d& position)
{
  // Create a Skeleton to represent the ground
  SkeletonPtr ground = Skeleton::create("ground");
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  double thickness = 0.01;
  tf.translation() = Eigen::Vector3d(0,0,-thickness/2.0);
  tf.translate(position);
  WeldJoint::Properties joint;
  joint.mT_ParentBodyToJoint = tf;
  ground->createJointAndBodyNodePair<WeldJoint>(nullptr, joint);
  ShapePtr groundShape =
      std::make_shared<BoxShape>(Eigen::Vector3d(10,10,thickness));

  auto shapeNode = ground->getBodyNode(0)->createShapeNodeWith<
      VisualAspect, CollisionAspect, DynamicsAspect>(groundShape);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue(0.2));

  return ground;
}

int main()
{
  Config config;
  config.title = "Sandbox";
  config.backend = Engine::Backend::OPENGL;

  auto world = World::create();
  auto atlas = createAtlas();
  auto ground = createGround(Eigen::Vector3d(0, -2, -2));
  world->addSkeleton(atlas);
  world->addSkeleton(ground);
  auto worldScene = std::make_shared<WorldScene>(world);
  auto viewer
      = std::make_shared<Viewer>(config, "Sandbox", 860, 640, worldScene);

  viewer->run();

  return 0;
}
