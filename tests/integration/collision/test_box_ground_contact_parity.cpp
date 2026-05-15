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

#include <dart/collision/bullet/reference/bullet_collision_detector.hpp>
#include <dart/collision/collision_detector.hpp>
#include <dart/collision/collision_group.hpp>
#include <dart/collision/collision_option.hpp>
#include <dart/collision/collision_result.hpp>
#include <dart/collision/dart/dart_collision_detector.hpp>
#include <dart/collision/fcl/reference/fcl_collision_detector.hpp>
#include <dart/collision/ode/reference/ode_collision_detector.hpp>

#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/simple_frame.hpp>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <functional>
#include <string>
#include <vector>

#include <cmath>
#include <cstddef>

namespace {

using DetectorFactory = std::function<dart::collision::CollisionDetectorPtr()>;

struct Backend
{
  std::string name;
  DetectorFactory factory;
};

std::size_t countContacts(const Backend& backend)
{
  const Eigen::Vector3d boxSize(0.3, 0.3, 0.3);
  const Eigen::Vector3d boxHalfExtents = 0.5 * boxSize;
  const Eigen::Vector3d groundSize(10.0, 10.0, 0.1);
  const double groundTop = 0.5 * groundSize.z();
  const double penetration = 0.01;

  Eigen::Isometry3d boxTf = Eigen::Isometry3d::Identity();
  boxTf.linear() = (Eigen::AngleAxisd(0.005, Eigen::Vector3d::UnitX())
                    * Eigen::AngleAxisd(-0.004, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()))
                       .toRotationMatrix();
  const double boxRadiusZ
      = boxHalfExtents.x() * std::abs(boxTf.linear().col(0).z())
        + boxHalfExtents.y() * std::abs(boxTf.linear().col(1).z())
        + boxHalfExtents.z() * std::abs(boxTf.linear().col(2).z());
  boxTf.translation()
      = Eigen::Vector3d(0.0, 0.0, groundTop + boxRadiusZ - penetration);

  auto box = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World());
  auto ground = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World());
  box->setShape(std::make_shared<dart::dynamics::BoxShape>(boxSize));
  ground->setShape(std::make_shared<dart::dynamics::BoxShape>(groundSize));
  box->setRelativeTransform(boxTf);
  ground->setRelativeTransform(Eigen::Isometry3d::Identity());

  const auto detector = backend.factory();
  auto group = detector->createCollisionGroup(box.get(), ground.get());

  dart::collision::CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 16u;

  dart::collision::CollisionResult result;
  EXPECT_TRUE(group->collide(option, &result)) << backend.name;
  return result.getNumContacts();
}

} // namespace

TEST(BoxGroundContactParity, RotatedBoxGroundPatchContactCount)
{
  const std::vector<Backend> backends{
      {"native", [] { return dart::collision::DartCollisionDetector::create(); }},
      {"fcl", [] { return dart::collision::FCLCollisionDetector::createReference(); }},
      {"bullet",
       [] { return dart::collision::BulletCollisionDetector::createReference(); }},
      {"ode", [] { return dart::collision::OdeCollisionDetector::createReference(); }},
  };

  std::size_t bulletContacts = 0;
  std::vector<std::pair<std::string, std::size_t>> counts;
  counts.reserve(backends.size());
  for (const auto& backend : backends) {
    const std::size_t contacts = countContacts(backend);
    counts.emplace_back(backend.name, contacts);
    if (backend.name == "bullet") {
      bulletContacts = contacts;
    }
  }

  ASSERT_GT(bulletContacts, 0u);
  for (const auto& [name, contacts] : counts) {
    const auto delta = static_cast<int>(contacts) - static_cast<int>(bulletContacts);
    EXPECT_LE(std::abs(delta), 1) << name << " contacts=" << contacts
                                  << " bullet=" << bulletContacts;
  }
}
