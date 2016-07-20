/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
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

#include "dart/dynamics/Branch.hpp"
#include "dart/dynamics/BodyNode.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
Branch::Criteria::Criteria(BodyNode* _start)
  : mStart(_start)
{
  // Do nothing
}

//==============================================================================
std::vector<BodyNode*> Branch::Criteria::satisfy() const
{
  return convert().satisfy();
}

//==============================================================================
Linkage::Criteria Branch::Criteria::convert() const
{
  Linkage::Criteria criteria;
  criteria.mStart.mNode = mStart;
  criteria.mStart.mPolicy = Linkage::Criteria::DOWNSTREAM;

  return criteria;
}

//==============================================================================
Branch::Criteria::operator Linkage::Criteria() const
{
  return convert();
}

//==============================================================================
BranchPtr Branch::create(const Branch::Criteria& _criteria,
                         const std::string& _name)
{
  BranchPtr branch(new Branch(_criteria, _name));
  branch->mPtr = branch;
  return branch;
}

//==============================================================================
bool Branch::isStillBranch() const
{
  if(!isAssembled())
    return false;

  for(std::size_t i=0; i<mBodyNodes.size(); ++i)
  {
    BodyNode* bn = mBodyNodes[i];
    if(bn->getNumChildBodyNodes() != mNumChildNodes[i])
      return false;
  }

  return true;
}

//==============================================================================
Branch::Branch(const Branch::Criteria& _criteria, const std::string& _name)
  : Linkage(_criteria, _name)
{
  update();
}

//==============================================================================
void Branch::update()
{
  Linkage::update();

  mNumChildNodes.clear();
  mNumChildNodes.reserve(mBodyNodes.size());
  for(std::size_t i=0; i<mBodyNodes.size(); ++i)
  {
    mNumChildNodes.push_back(mBodyNodes[i]->getNumChildBodyNodes());
  }
}

} // namespace dynamics
} // namespace dart
