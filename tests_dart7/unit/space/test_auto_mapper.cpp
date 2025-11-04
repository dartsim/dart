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

#include "dart7/common/constants.hpp"
#include "dart7/comps/component_category.hpp"
#include "dart7/space/auto_mapper.hpp"
#include "dart7/space/state_space.hpp"
#include "dart7/space/vector_mapper.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <entt/entt.hpp>
#include <gtest/gtest.h>

using namespace dart7;
using namespace dart7::space;
using namespace dart7::comps;

// Example PropertyComponents for testing

struct Position
{
  static constexpr ComponentCategory category = ComponentCategory::Property;

  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Velocity
{
  static constexpr ComponentCategory category = ComponentCategory::Property;

  Eigen::Vector3d linear{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular{Eigen::Vector3d::Zero()};
};

struct Transform
{
  static constexpr ComponentCategory category = ComponentCategory::Property;

  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
};

struct JointState
{
  static constexpr ComponentCategory category = ComponentCategory::Property;

  double position{0.0};
  double velocity{0.0};
  double acceleration{0.0};
  double effort{0.0};
};

struct NestedData
{
  static constexpr ComponentCategory category = ComponentCategory::Property;

  struct Inner
  {
    double a{0.0};
    double b{0.0};
  };

  Inner inner;
  double outer{0.0};
};

//==============================================================================
// Tests
//==============================================================================

TEST(AutoMapper, SimpleScalarFields)
{
  StateSpace space;
  space.addVariable("positions", 9); // 3 entities × 3 DOF each
  space.finalize();

  VectorMapper mapper(std::move(space));
  mapper.addMapper("positions", makeAutoMapper<Position>());

  // Create entities
  entt::registry registry;
  auto e1 = registry.create();
  auto e2 = registry.create();
  auto e3 = registry.create();

  registry.emplace<Position>(e1, Position{1.0, 2.0, 3.0});
  registry.emplace<Position>(e2, Position{4.0, 5.0, 6.0});
  registry.emplace<Position>(e3, Position{7.0, 8.0, 9.0});

  // Extract
  auto vec = mapper.toVector(registry);

  ASSERT_EQ(vec.size(), 9);
  // Check that all values are present (order not guaranteed)
  std::vector<double> expected = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  for (double val : expected) {
    bool found = false;
    for (double v : vec) {
      if (std::abs(v - val) < 1e-10) {
        found = true;
        break;
      }
    }
    EXPECT_TRUE(found) << "Value " << val << " not found";
  }
}

TEST(AutoMapper, EigenVector3dFields)
{
  StateSpace space;
  space.addVariable("velocities", 6); // 1 entity × 6 DOF (linear + angular)
  space.finalize();

  VectorMapper mapper(std::move(space));
  mapper.addMapper("velocities", makeAutoMapper<Velocity>());

  // Create entity
  entt::registry registry;
  auto entity = registry.create();

  Velocity vel;
  vel.linear << 1.0, 2.0, 3.0;
  vel.angular << 4.0, 5.0, 6.0;
  registry.emplace<Velocity>(entity, vel);

  // Extract
  auto vec = mapper.toVector(registry);

  ASSERT_EQ(vec.size(), 6);
  EXPECT_DOUBLE_EQ(vec[0], 1.0);
  EXPECT_DOUBLE_EQ(vec[1], 2.0);
  EXPECT_DOUBLE_EQ(vec[2], 3.0);
  EXPECT_DOUBLE_EQ(vec[3], 4.0);
  EXPECT_DOUBLE_EQ(vec[4], 5.0);
  EXPECT_DOUBLE_EQ(vec[5], 6.0);
}

TEST(AutoMapper, IsometryField)
{
  StateSpace space;
  space.addVariable(
      "transforms", 7); // 1 entity × 7 DOF (translation + quaternion)
  space.finalize();

  VectorMapper mapper(std::move(space));
  mapper.addMapper("transforms", makeAutoMapper<Transform>());

  // Create entity with transform
  entt::registry registry;
  auto entity = registry.create();

  Transform transform;
  transform.pose.translation() << 1.0, 2.0, 3.0;
  transform.pose.linear()
      = Eigen::AngleAxisd(dart7::pi / 4.0, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
  registry.emplace<Transform>(entity, transform);

  // Extract
  auto vec = mapper.toVector(registry);

  ASSERT_EQ(vec.size(), 7);

  // Check translation
  EXPECT_DOUBLE_EQ(vec[0], 1.0);
  EXPECT_DOUBLE_EQ(vec[1], 2.0);
  EXPECT_DOUBLE_EQ(vec[2], 3.0);

  // Check quaternion (verify it's a unit quaternion)
  double qw = vec[3], qx = vec[4], qy = vec[5], qz = vec[6];
  double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  EXPECT_NEAR(norm, 1.0, 1e-10);
}

TEST(AutoMapper, RoundTripMultipleFields)
{
  StateSpace space;
  space.addVariable("joint_states", 4); // 1 entity × 4 scalars
  space.finalize();

  VectorMapper mapper(std::move(space));
  mapper.addMapper("joint_states", makeAutoMapper<JointState>());

  // Create entity
  entt::registry registry;
  auto entity = registry.create();

  JointState state;
  state.position = 1.5;
  state.velocity = 2.5;
  state.acceleration = 3.5;
  state.effort = 4.5;
  registry.emplace<JointState>(entity, state);

  // Extract
  auto vec1 = mapper.toVector(registry);

  ASSERT_EQ(vec1.size(), 4);
  EXPECT_DOUBLE_EQ(vec1[0], 1.5);
  EXPECT_DOUBLE_EQ(vec1[1], 2.5);
  EXPECT_DOUBLE_EQ(vec1[2], 3.5);
  EXPECT_DOUBLE_EQ(vec1[3], 4.5);

  // Modify
  vec1[0] = 10.0;
  vec1[1] = 20.0;
  vec1[2] = 30.0;
  vec1[3] = 40.0;

  // Inject back
  mapper.fromVector(registry, vec1);

  // Verify
  const auto& modified = registry.get<JointState>(entity);
  EXPECT_DOUBLE_EQ(modified.position, 10.0);
  EXPECT_DOUBLE_EQ(modified.velocity, 20.0);
  EXPECT_DOUBLE_EQ(modified.acceleration, 30.0);
  EXPECT_DOUBLE_EQ(modified.effort, 40.0);
}

TEST(AutoMapper, NestedStructs)
{
  StateSpace space;
  space.addVariable("nested", 3); // inner.a, inner.b, outer
  space.finalize();

  VectorMapper mapper(std::move(space));
  mapper.addMapper("nested", makeAutoMapper<NestedData>());

  // Create entity
  entt::registry registry;
  auto entity = registry.create();

  NestedData data;
  data.inner.a = 1.0;
  data.inner.b = 2.0;
  data.outer = 3.0;
  registry.emplace<NestedData>(entity, data);

  // Extract
  auto vec = mapper.toVector(registry);

  ASSERT_EQ(vec.size(), 3);
  EXPECT_DOUBLE_EQ(vec[0], 1.0);
  EXPECT_DOUBLE_EQ(vec[1], 2.0);
  EXPECT_DOUBLE_EQ(vec[2], 3.0);

  // Modify and inject
  vec[0] = 10.0;
  vec[1] = 20.0;
  vec[2] = 30.0;
  mapper.fromVector(registry, vec);

  // Verify
  const auto& modified = registry.get<NestedData>(entity);
  EXPECT_DOUBLE_EQ(modified.inner.a, 10.0);
  EXPECT_DOUBLE_EQ(modified.inner.b, 20.0);
  EXPECT_DOUBLE_EQ(modified.outer, 30.0);
}

TEST(AutoMapper, MultipleEntitiesSameComponent)
{
  StateSpace space;
  space.addVariable("joints", 12); // 3 entities × 4 DOF each
  space.finalize();

  VectorMapper mapper(std::move(space));
  mapper.addMapper("joints", makeAutoMapper<JointState>());

  // Create entities
  entt::registry registry;
  auto e1 = registry.create();
  auto e2 = registry.create();
  auto e3 = registry.create();

  registry.emplace<JointState>(e1, JointState{1.0, 2.0, 3.0, 4.0});
  registry.emplace<JointState>(e2, JointState{5.0, 6.0, 7.0, 8.0});
  registry.emplace<JointState>(e3, JointState{9.0, 10.0, 11.0, 12.0});

  // Extract
  auto vec = mapper.toVector(registry);

  ASSERT_EQ(vec.size(), 12);

  // Verify all values present
  std::vector<double> expected;
  for (double i = 1.0; i <= 12.0; i += 1.0) {
    expected.push_back(i);
  }

  for (double val : expected) {
    bool found = false;
    for (double v : vec) {
      if (std::abs(v - val) < 1e-10) {
        found = true;
        break;
      }
    }
    EXPECT_TRUE(found) << "Value " << val << " not found";
  }
}

TEST(AutoMapper, MixedAutoAndManualMappers)
{
  StateSpace space;
  space.addVariable("positions", 3);
  space.addVariable("joints", 4);
  space.finalize();

  VectorMapper mapper(std::move(space));

  // Use auto mapper for Position
  mapper.addMapper("positions", makeAutoMapper<Position>());

  // Use auto mapper for JointState
  mapper.addMapper("joints", makeAutoMapper<JointState>());

  // Create entities
  entt::registry registry;
  auto posEntity = registry.create();
  auto jointEntity = registry.create();

  registry.emplace<Position>(posEntity, Position{1.0, 2.0, 3.0});
  registry.emplace<JointState>(jointEntity, JointState{10.0, 20.0, 30.0, 40.0});

  // Extract
  auto vec = mapper.toVector(registry);

  ASSERT_EQ(vec.size(), 7);

  // First 3 are position
  EXPECT_DOUBLE_EQ(vec[0], 1.0);
  EXPECT_DOUBLE_EQ(vec[1], 2.0);
  EXPECT_DOUBLE_EQ(vec[2], 3.0);

  // Next 4 are joint state
  EXPECT_DOUBLE_EQ(vec[3], 10.0);
  EXPECT_DOUBLE_EQ(vec[4], 20.0);
  EXPECT_DOUBLE_EQ(vec[5], 30.0);
  EXPECT_DOUBLE_EQ(vec[6], 40.0);
}

TEST(AutoMapper, EigenConversion)
{
  StateSpace space;
  space.addVariable("velocities", 6);
  space.finalize();

  VectorMapper mapper(std::move(space));
  mapper.addMapper("velocities", makeAutoMapper<Velocity>());

  // Create entity
  entt::registry registry;
  auto entity = registry.create();

  Velocity vel;
  vel.linear << 1.0, 2.0, 3.0;
  vel.angular << 4.0, 5.0, 6.0;
  registry.emplace<Velocity>(entity, vel);

  // Extract to Eigen
  auto eigenVec = mapper.toEigen(registry);

  ASSERT_EQ(eigenVec.size(), 6);
  EXPECT_DOUBLE_EQ(eigenVec[0], 1.0);
  EXPECT_DOUBLE_EQ(eigenVec[1], 2.0);
  EXPECT_DOUBLE_EQ(eigenVec[2], 3.0);
  EXPECT_DOUBLE_EQ(eigenVec[3], 4.0);
  EXPECT_DOUBLE_EQ(eigenVec[4], 5.0);
  EXPECT_DOUBLE_EQ(eigenVec[5], 6.0);

  // Modify and inject
  eigenVec *= 10.0;
  mapper.fromEigen(registry, eigenVec);

  // Verify
  const auto& modified = registry.get<Velocity>(entity);
  EXPECT_DOUBLE_EQ(modified.linear.x(), 10.0);
  EXPECT_DOUBLE_EQ(modified.linear.y(), 20.0);
  EXPECT_DOUBLE_EQ(modified.linear.z(), 30.0);
  EXPECT_DOUBLE_EQ(modified.angular.x(), 40.0);
  EXPECT_DOUBLE_EQ(modified.angular.y(), 50.0);
  EXPECT_DOUBLE_EQ(modified.angular.z(), 60.0);
}
