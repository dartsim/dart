/*
 * Copyright (c) 2011, The DART development contributors
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

#include <dart/simulation/simulation.hpp>

#include <dart/constraint/constraint.hpp>

#include <dart/collision/bullet/bullet.hpp>

#include <dart/dynamics/dynamics.hpp>

#include <dart/math/math.hpp>

#include <benchmark/benchmark.h>

using namespace dart;

namespace {

[[nodiscard]] dynamics::SkeletonPtr createBox(
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& size = Eigen::Vector3d(1, 1, 1),
    const Eigen::Vector3d& color
    = dart::math::Random::uniform<Eigen::Vector3d>(0.0, 1.0))
{
  static size_t index = 0;
  auto boxSkel = dynamics::Skeleton::create("box" + std::to_string(index++));

  // Give the floor a body
  auto boxBody
      = boxSkel->createJointAndBodyNodePair<dynamics::FreeJoint>(nullptr)
            .second;

  // Give the body a shape
  auto boxShape = std::make_shared<dynamics::BoxShape>(size);
  dynamics::ShapeNode* shapeNode = boxBody->createShapeNodeWith<
      dynamics::VisualAspect,
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(boxShape);
  shapeNode->getVisualAspect()->setColor(color);
  shapeNode->getDynamicsAspect()->setRestitutionCoeff(0.9);

  // Put the body into position
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = position;
  boxBody->getParentJoint()->setTransformFromParentBodyNode(tf);

  return boxSkel;
}

[[nodiscard]] simulation::WorldPtr createWorld(size_t dim)
{
  // Create an empty world
  auto world = simulation::World::create();

  // Set collision detector type
  world->getConstraintSolver()->setCollisionDetector(
      collision::BulletCollisionDetector::create());

  // Create dim x dim x dim boxes
  for (auto i = 0u; i < dim; ++i) {
    for (auto j = 0u; j < dim; ++j) {
      for (auto k = 0u; k < dim; ++k) {
        auto x = i - dim / 2;
        auto y = j - dim / 2;
        auto z = k + 5;
        auto position = Eigen::Vector3d(x, y, z);
        auto size = Eigen::Vector3d(0.9, 0.9, 0.9);
        auto color = Eigen::Vector3d(
            static_cast<double>(i) / dim,
            static_cast<double>(j) / dim,
            static_cast<double>(k) / dim);
        auto box = createBox(position, size, color);
        world->addSkeleton(box);
      }
    }
  }

  // Create ground
  auto ground = dynamics::Skeleton::create("ground");
  auto groundBody
      = ground->createJointAndBodyNodePair<dynamics::WeldJoint>().second;
  auto groundShapeNode = groundBody->createShapeNodeWith<
      dynamics::VisualAspect,
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(10.0, 10.0, 0.1)));
  groundShapeNode->getVisualAspect()->setColor(dart::Color::LightGray());
  groundShapeNode->getDynamicsAspect()->setRestitutionCoeff(0.9);
  world->addSkeleton(ground);

  return world;
}

} // namespace

static void BM_RunBoxes(benchmark::State& state)
{
  const auto steps = 2000u;
  auto world = createWorld(state.range(0));

  for (auto _ : state) {
    for (size_t i = 0; i < steps; ++i) {
      world->step();
    }
  }
}

BENCHMARK(BM_RunBoxes)->Arg(2)->Arg(4)->Arg(8)->Unit(benchmark::kMillisecond);
