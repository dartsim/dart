/*
 * Copyright (c) 2014-2015, Georgia Tech Research Corporation
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

#include "dart/constraint/ConstrainedGroup.h"

#include <iostream>
#include <vector>

#include "dart/common/Console.h"
#include "dart/constraint/ConstraintBase.h"
#include "dart/constraint/ConstraintSolver.h"

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
  assert(_constraint != nullptr && "Null constraint pointer is now allowed.");
  assert(containConstraint(_constraint) == false
         && "Don't try to add same constraint multiple times into Community.");
  assert(_constraint->isActive());

  mConstraints.push_back(_constraint);
}

//==============================================================================
size_t ConstrainedGroup::getNumConstraints() const
{
  return mConstraints.size();
}

//==============================================================================
ConstraintBasePtr ConstrainedGroup::getConstraint(size_t _index) const
{
  assert(_index < mConstraints.size());
  return mConstraints[_index];
}

//==============================================================================
void ConstrainedGroup::removeConstraint(const ConstraintBasePtr& _constraint)
{
  assert(_constraint != nullptr && "Null constraint pointer is now allowed.");
  assert(containConstraint(_constraint) == true
         && "Don't try to remove a constraint not contained in Community.");

  mConstraints.erase(
        remove(mConstraints.begin(), mConstraints.end(), _constraint),
        mConstraints.end());
}

//==============================================================================
void ConstrainedGroup::removeAllConstraints()
{
//  dtwarn << "ConstrainedGroup::removeAllConstraints(): "
//         << "Not implemented." << std::endl;

  // TODO(JS): Temporary implementation
//  for (size_t i = 0; i < mConstraints.size(); ++i)
//  {
//    delete mConstraints[i];
//  }

  mConstraints.clear();
}

//==============================================================================
bool ConstrainedGroup::containConstraint(
    const ConstConstraintBasePtr& _constraint) const
{
//  std::cout << "CommunityTEST::_containConstraint(): Not implemented."
//            << std::endl;

  // TODO(MXG): Is there any reason these functions are not implemented yet?

  return false;
}

//==============================================================================
bool ConstrainedGroup::checkAndAddConstraint(
    const ConstraintBasePtr& _constraint)
{
  std::cout << "CommunityTEST::_checkAndAddConstraint(): Not implemented."
            << std::endl;

  return false;
}

//==============================================================================
size_t ConstrainedGroup::getTotalDimension() const
{
  size_t totalDim = 0;

  for (size_t i = 0; i < mConstraints.size(); ++i)
    totalDim += mConstraints[i]->getDimension();

  return totalDim;
}

}  // namespace constraint
}  // namespace dart
