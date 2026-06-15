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

#ifndef DART_BENCHMARK_INTEGRATION_BOXES_SCENE_HPP_
#define DART_BENCHMARK_INTEGRATION_BOXES_SCENE_HPP_

#include <dart/simulation/simulation.hpp>

#include <dart/constraint/constraint.hpp>

#include <dart/collision/bullet/bullet.hpp>

#include <dart/dynamics/dynamics.hpp>

#include <dart/math/math.hpp>

#include <cstddef>
#include <string>

namespace dart::test {

/// Builds one free-falling box skeleton. The geometry is fully deterministic;
/// the color is irrelevant to dynamics and is kept only so the scene matches
/// the GUI `boxes` example. Shared by the headless benchmark and the
/// determinism verifier so both exercise byte-for-byte identical setups.
[[nodiscard]] inline dynamics::SkeletonPtr createBox(
    std::size_t index,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& size,
    const Eigen::Vector3d& color)
{
  auto boxSkel = dynamics::Skeleton::create("box" + std::to_string(index));

  auto boxBody
      = boxSkel->createJointAndBodyNodePair<dynamics::FreeJoint>(nullptr)
            .second;

  auto boxShape = std::make_shared<dynamics::BoxShape>(size);
  dynamics::ShapeNode* shapeNode = boxBody->createShapeNodeWith<
      dynamics::VisualAspect,
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(boxShape);
  shapeNode->getVisualAspect()->setColor(color);
  shapeNode->getDynamicsAspect()->setRestitutionCoeff(0.9);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = position;
  boxBody->getParentJoint()->setTransformFromParentBodyNode(tf);

  return boxSkel;
}

/// Builds the `dim x dim x dim` stack-of-boxes world used by the boxes
/// benchmark/example: a Bullet collision detector, dim^3 free boxes dropped
/// above a welded ground plane.
[[nodiscard]] inline simulation::WorldPtr createBoxesWorld(std::size_t dim)
{
  auto world = simulation::World::create();

  world->getConstraintSolver()->setCollisionDetector(
      collision::BulletCollisionDetector::create());

  std::size_t index = 0;
  for (auto i = 0u; i < dim; ++i) {
    for (auto j = 0u; j < dim; ++j) {
      for (auto k = 0u; k < dim; ++k) {
        auto x = i - dim / 2;
        auto y = j - dim / 2;
        auto z = k + 5;
        auto position = Eigen::Vector3d(x, y, z);
        auto size = Eigen::Vector3d(0.9, 0.9, 0.9);
        auto color = Eigen::Vector3d(
            static_cast<double>(i) / static_cast<double>(dim),
            static_cast<double>(j) / static_cast<double>(dim),
            static_cast<double>(k) / static_cast<double>(dim));
        world->addSkeleton(createBox(index++, position, size, color));
      }
    }
  }

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

/// Builds one articulated chain: a free-floating root box link followed by
/// `links - 1` revolute-jointed box links extending along +z. Deterministic.
[[nodiscard]] inline dynamics::SkeletonPtr createChain(
    std::size_t index,
    const Eigen::Vector3d& basePosition,
    std::size_t links)
{
  auto skel = dynamics::Skeleton::create("chain" + std::to_string(index));
  const Eigen::Vector3d linkSize(0.3, 0.3, 0.5);

  dynamics::BodyNode* parent = nullptr;
  for (std::size_t l = 0; l < links; ++l) {
    dynamics::BodyNode* body = nullptr;
    if (l == 0) {
      auto pair
          = skel->createJointAndBodyNodePair<dynamics::FreeJoint>(nullptr);
      body = pair.second;
      Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
      tf.translation() = basePosition;
      pair.first->setTransformFromParentBodyNode(tf);
    } else {
      auto pair = skel->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          parent);
      auto* joint = pair.first;
      body = pair.second;
      // Alternate the hinge axis so the chain folds in a non-planar way.
      joint->setAxis(
          (l % 2 == 0) ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY());
      Eigen::Isometry3d parentToJoint = Eigen::Isometry3d::Identity();
      parentToJoint.translation() = Eigen::Vector3d(0.0, 0.0, linkSize.z());
      joint->setTransformFromParentBodyNode(parentToJoint);
    }

    auto shape = std::make_shared<dynamics::BoxShape>(linkSize);
    auto* shapeNode = body->createShapeNodeWith<
        dynamics::VisualAspect,
        dynamics::CollisionAspect,
        dynamics::DynamicsAspect>(shape);
    shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(
        static_cast<double>(l) / static_cast<double>(links), 0.4, 0.6));
    shapeNode->getDynamicsAspect()->setRestitutionCoeff(0.6);
    parent = body;
  }

  return skel;
}

/// Builds a `dim x dim` grid of articulated chains (each `links` box links long)
/// dropped above a welded ground plane. This exercises contact constraints on
/// articulated bodies: the cost of resolving a contact scales with how far the
/// solver must propagate impulses along each chain, which is exactly the
/// rigid-body case plus articulation depth.
[[nodiscard]] inline simulation::WorldPtr createChainsWorld(
    std::size_t dim, std::size_t links)
{
  auto world = simulation::World::create();

  world->getConstraintSolver()->setCollisionDetector(
      collision::BulletCollisionDetector::create());

  // Pack the chains closely (0.5 m grid for 0.3 m-wide links) and stagger their
  // drop height so they buckle and tangle into a dense pile of articulated
  // bodies in mutual contact, rather than falling as isolated columns.
  std::size_t index = 0;
  for (auto i = 0u; i < dim; ++i) {
    for (auto j = 0u; j < dim; ++j) {
      const double x
          = (static_cast<double>(i) - static_cast<double>(dim) / 2.0) * 0.5;
      const double y
          = (static_cast<double>(j) - static_cast<double>(dim) / 2.0) * 0.5;
      const double z = 3.0 + 0.25 * static_cast<double>((i + j) % 4);
      world->addSkeleton(
          createChain(index++, Eigen::Vector3d(x, y, z), links));
    }
  }

  auto ground = dynamics::Skeleton::create("ground");
  auto groundBody
      = ground->createJointAndBodyNodePair<dynamics::WeldJoint>().second;
  auto groundShapeNode = groundBody->createShapeNodeWith<
      dynamics::VisualAspect,
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(10.0, 10.0, 0.1)));
  groundShapeNode->getVisualAspect()->setColor(dart::Color::LightGray());
  groundShapeNode->getDynamicsAspect()->setRestitutionCoeff(0.6);
  world->addSkeleton(ground);

  return world;
}

} // namespace dart::test

#endif // DART_BENCHMARK_INTEGRATION_BOXES_SCENE_HPP_
