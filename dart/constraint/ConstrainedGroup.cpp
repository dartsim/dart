/*
 * Copyright (c) 2014-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2014-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include "dart/constraint/ConstrainedGroup.hpp"

#include <algorithm>
#include <iostream>
#include <vector>

#include "dart/common/Console.hpp"
#include "dart/constraint/ConstraintBase.hpp"
#include "dart/constraint/ConstraintSolver.hpp"

namespace dart {
namespace constraint {

//==============================================================================
ConstrainedGroup::ConstrainedGroup()
{
}

//==============================================================================
ConstrainedGroup::~ConstrainedGroup()
{
}

//==============================================================================
void ConstrainedGroup::addConstraint(const ConstraintBasePtr& _constraint)
{
  assert(_constraint != nullptr && "Attempted to add nullptr.");
  assert(!containConstraint(_constraint)
         && "Attempted to add a duplicate constraint.");
  assert(_constraint->isActive());

  mConstraints.push_back(_constraint);
}

//==============================================================================
std::size_t ConstrainedGroup::getNumConstraints() const
{
  return mConstraints.size();
}

//==============================================================================
ConstraintBasePtr ConstrainedGroup::getConstraint(std::size_t _index) const
{
  assert(_index < mConstraints.size());
  return mConstraints[_index];
}

//==============================================================================
void ConstrainedGroup::removeConstraint(const ConstraintBasePtr& _constraint)
{
  assert(_constraint != nullptr && "Attempted to add nullptr.");
  assert(containConstraint(_constraint)
         && "Attempted to remove not existing constraint.");

  mConstraints.erase(
        remove(mConstraints.begin(), mConstraints.end(), _constraint),
        mConstraints.end());
}

//==============================================================================
void ConstrainedGroup::removeAllConstraints()
{
  mConstraints.clear();
}

//==============================================================================
#ifndef NDEBUG
bool ConstrainedGroup::containConstraint(
    const ConstConstraintBasePtr& _constraint) const
{
  return std::find(mConstraints.begin(), mConstraints.end(), _constraint)
      != mConstraints.end();
}
#endif

//==============================================================================
std::size_t ConstrainedGroup::getTotalDimension() const
{
  std::size_t totalDim = 0;

  for (std::size_t i = 0; i < mConstraints.size(); ++i)
    totalDim += mConstraints[i]->getDimension();

  return totalDim;
}

}  // namespace constraint
}  // namespace dart
