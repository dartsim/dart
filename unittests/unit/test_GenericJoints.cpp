/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#include <iostream>
#include <gtest/gtest.h>
#include "TestHelpers.hpp"

#include "dart/dart.hpp"

using namespace dart;
using namespace dynamics;

//class ZeroDofJointTest : public GenericJoint<NullSpace>
//{
//public:
//  /// Constructor called by Skeleton class
//  ZeroDofJointTest(const Properties& properties = Properties())
//    : GenericJoint<NullSpace>(properties) {}

//  virtual ~ZeroDofJointTest() {}

//  /// Gets a string representing the joint type
//  const std::string& getType() const override { return getStaticType(); }

//  const std::string& getStaticType() const
//  {
//    static const std::string name = "TestWeldJoint";
//    return name;
//  }

//  // Documentation inherited
//  bool isCyclic(size_t index) const override { return false; }

//  const JacobianMatrix getRelativeJacobianStatic(
//      const Vector& positions) const override
//  { return JacobianMatrix(); }

//protected:
//  // Documentation inherited
//  Joint* clone() const override { return nullptr; }

//  // Documentation inherited
//  void updateDegreeOfFreedomNames() override {}

//  // Documentation inherited
//  void updateRelativeTransform() const override {}

//  // Documentation inherited
//  void updateRelativeJacobian(bool mandatory = true) const override {}

//  // Documentation inherited
//  void updateRelativeJacobianTimeDeriv() const override {}
//};

class SingleDofJointTest : public GenericJoint<R1Space>
{
public:
  /// Constructor called by Skeleton class
  SingleDofJointTest(const Properties& properties = Properties())
    : GenericJoint<R1Space>(properties) {}

  virtual ~SingleDofJointTest() {}

  /// Gets a string representing the joint type
  const std::string& getType() const override { return getStaticType(); }

  const std::string& getStaticType() const
  {
    static const std::string name = "TestSingleDofJoint";
    return name;
  }

  // Documentation inherited
  bool isCyclic(size_t /*index*/) const override { return false; }

  JacobianMatrix getRelativeJacobianStatic(
      const Vector& /*positions*/) const override
  { return JacobianMatrix(); }

protected:
  // Documentation inherited
  Joint* clone() const override { return nullptr; }

  // Documentation inherited
  void updateDegreeOfFreedomNames() override {}

  // Documentation inherited
  void updateRelativeTransform() const override {}

  // Documentation inherited
  void updateRelativeJacobian(bool /*mandatory = true*/) const override {}

  // Documentation inherited
  void updateRelativeJacobianTimeDeriv() const override {}
};

class MultiDofJointTest : public GenericJoint<RealVectorSpace<6>>
{
public:
  /// Constructor called by Skeleton class
  MultiDofJointTest(const Properties& properties = Properties())
    : GenericJoint<RealVectorSpace<6>>(properties) {}

  virtual ~MultiDofJointTest() {}

  /// Gets a string representing the joint type
  const std::string& getType() const override { return getStaticType(); }

  const std::string& getStaticType() const
  {
    static const std::string name = "TestMultiDofJoint";
    return name;
  }

  // Documentation inherited
  bool isCyclic(size_t /*index*/) const override { return false; }

  JacobianMatrix getRelativeJacobianStatic(
      const Vector& /*positions*/) const override
  { return JacobianMatrix(); }

protected:
  // Documentation inherited
  Joint* clone() const override { return nullptr; }

  // Documentation inherited
  void updateDegreeOfFreedomNames() override {}

  // Documentation inherited
  void updateRelativeTransform() const override {}

  // Documentation inherited
  void updateRelativeJacobian(bool /*mandatory = true*/) const override {}

  // Documentation inherited
  void updateRelativeJacobianTimeDeriv() const override {}
};

class SO3JointTest : public GenericJoint<SO3Space>
{
public:
  /// Constructor called by Skeleton class
  SO3JointTest(const Properties& properties = Properties())
    : GenericJoint<SO3Space>(properties) {}

  virtual ~SO3JointTest() {}

  /// Gets a string representing the joint type
  const std::string& getType() const override { return getStaticType(); }

  const std::string& getStaticType() const
  {
    static const std::string name = "TestMultiDofJoint";
    return name;
  }

  // Documentation inherited
  bool isCyclic(size_t /*index*/) const override { return false; }

  JacobianMatrix getRelativeJacobianStatic(
      const Vector& /*positions*/) const override
  { return JacobianMatrix(); }

protected:
  // Documentation inherited
  Joint* clone() const override { return nullptr; }

  // Documentation inherited
  void updateDegreeOfFreedomNames() override {}

  // Documentation inherited
  void updateRelativeTransform() const override {}

  // Documentation inherited
  void updateRelativeJacobian(bool /*mandatory = true*/) const override {}

  // Documentation inherited
  void updateRelativeJacobianTimeDeriv() const override {}
};

//==============================================================================
TEST(GenericJoint, Basic)
{
//  ZeroDofJointTest zeroDofJoint;
  SingleDofJointTest singleDofJoint;
  MultiDofJointTest genericJoint;
  SO3JointTest so3Joint;
}
