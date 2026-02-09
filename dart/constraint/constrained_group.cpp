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

#include "dart/constraint/constrained_group.hpp"

#include "dart/common/logging.hpp"
#include "dart/common/macros.hpp"
#include "dart/constraint/constraint_base.hpp"
#include "dart/constraint/constraint_solver.hpp"

#include <algorithm>
#include <iostream>
#include <vector>

namespace dart {
namespace constraint {

//==============================================================================
ConstrainedGroup::ConstrainedGroup() {}

//==============================================================================
ConstrainedGroup::~ConstrainedGroup() {}

//==============================================================================
void ConstrainedGroup::addConstraint(const ConstraintBasePtr& _constraint)
{
  DART_ASSERT(_constraint != nullptr && "Attempted to add nullptr.");
  DART_ASSERT(
      !containConstraint(_constraint)
      && "Attempted to add a duplicate constraint.");
  DART_ASSERT(_constraint->isActive());

  mConstraints.push_back(_constraint);
}

//==============================================================================
std::size_t ConstrainedGroup::getNumConstraints() const
{
  return mConstraints.size();
}

//==============================================================================
ConstraintBasePtr ConstrainedGroup::getConstraint(std::size_t _index)
{
  DART_ASSERT(_index < mConstraints.size());
  return mConstraints[_index];
}

//==============================================================================
ConstConstraintBasePtr ConstrainedGroup::getConstraint(std::size_t _index) const
{
  DART_ASSERT(_index < mConstraints.size());
  return mConstraints[_index];
}

//==============================================================================
void ConstrainedGroup::removeConstraint(const ConstraintBasePtr& _constraint)
{
  DART_ASSERT(_constraint != nullptr && "Attempted to add nullptr.");
  DART_ASSERT(
      containConstraint(_constraint)
      && "Attempted to remove not existing constraint.");

  std::erase(mConstraints, _constraint);
}

//==============================================================================
void ConstrainedGroup::removeAllConstraints()
{
  mConstraints.clear();
}

//==============================================================================
bool ConstrainedGroup::containConstraint(
    const ConstConstraintBasePtr& _constraint) const
{
  return std::ranges::find(mConstraints, _constraint) != mConstraints.end();
}

//==============================================================================
std::size_t ConstrainedGroup::getTotalDimension() const
{
  std::size_t totalDim = 0;

  for (const auto& constraint : mConstraints) {
    totalDim += constraint->getDimension();
  }

  return totalDim;
}

} // namespace constraint
} // namespace dart
