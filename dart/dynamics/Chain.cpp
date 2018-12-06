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

#include "dart/dynamics/Chain.hpp"
#include "dart/dynamics/FreeJoint.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
Chain::Criteria::Criteria(BodyNode* _start, BodyNode* _target,
                          bool _includeBoth)
  : mStart(_start),
    mTarget(_target),
    mIncludeBoth(_includeBoth)
{
  // Do nothing
}

//==============================================================================
std::vector<BodyNode*> Chain::Criteria::satisfy() const
{
  return convert().satisfy();
}

//==============================================================================
Linkage::Criteria Chain::Criteria::convert() const
{
  Linkage::Criteria criteria;
  criteria.mStart.mNode = mStart;
  criteria.mStart.mPolicy = Linkage::Criteria::INCLUDE;

  Linkage::Criteria::Target target;
  target.mNode = mTarget;
  target.mChain = true;
  target.mPolicy = Linkage::Criteria::INCLUDE;

  if(!mIncludeBoth)
  {
    if(target.mNode.lock() &&
       target.mNode.lock()->descendsFrom(criteria.mStart.mNode.lock()))
    {
      criteria.mStart.mPolicy = Linkage::Criteria::EXCLUDE;
    }

    if(criteria.mStart.mNode.lock() &&
       criteria.mStart.mNode.lock()->descendsFrom(target.mNode.lock()))
    {
      target.mPolicy = Linkage::Criteria::EXCLUDE;
    }
  }

  criteria.mTargets.push_back(target);

  return criteria;
}

//==============================================================================
Chain::Criteria::operator Linkage::Criteria() const
{
  return convert();
}

//==============================================================================
ChainPtr Chain::create(const Chain::Criteria& _criteria,
                       const std::string& _name)
{
  ChainPtr chain(new Chain(_criteria, _name));
  chain->mPtr = chain;
  return chain;
}

//==============================================================================
ChainPtr Chain::create(BodyNode* _start, BodyNode* _target,
                       const std::string& _name)
{
  ChainPtr chain(new Chain(_start, _target, _name));
  chain->mPtr = chain;
  return chain;
}

//==============================================================================
ChainPtr Chain::create(BodyNode* _start, BodyNode* _target,
                       IncludeBothTag, const std::string& _name)
{
  ChainPtr chain(new Chain(_start, _target, IncludeBoth, _name));
  chain->mPtr = chain;
  return chain;
}

//==============================================================================
MetaSkeletonPtr Chain::cloneMetaSkeleton() const
{
  // Create an empty Group
  ChainPtr newGroup = create(nullptr, nullptr, getName());

  if (getNumBodyNodes() == 0u && (getNumJoints() != 0u || getNumDofs() != 0u))
  {
    dtwarn << "[Chain::cloneMetaSkeletonHelper] Attempting to "
           << "clone a ReferentialSkeleton that doesn't include any BodyNodes "
           << "but including some Joints or DegreeOfFreedoms. This will lead "
           << "to dangling Joints or DegreeOfFreedoms in the cloned "
           << "ReferentialSkeleton because it only holds the stong reference "
           << "to the BodyNodes but not others.\n";
  }

  // Array for Skeleton clones that will be collected durig cloning BodyNodes,
  // Joints, DegreeOfFreedoms.
  //
  // The clones will not be destroyed even after the map is destroyed because
  // the new Linkage will hold the skeleton by holding the strong referecnes of
  // the body nodes.
  std::unordered_map<const Skeleton*, SkeletonPtr> mapToSkeletonClones;
  mapToSkeletonClones.reserve(mSkeletons.size());

  newGroup->cloneCriteria(mCriteria, mapToSkeletonClones);

  // Register BodyNode
  for (const BodyNodePtr& bodyNode : mBodyNodes)
    newGroup->cloneBodyNode(bodyNode.get(), mapToSkeletonClones);

  // Register Joint
  for (const JointPtr& joint : mJoints)
    newGroup->cloneJoint(joint.get(), mapToSkeletonClones);

  // Register DegreeOfFreedom
  for (const DegreeOfFreedomPtr& dof : mDofs)
    newGroup->cloneDegreeOfFreedom(dof.get(), mapToSkeletonClones);

  return newGroup;
}

//==============================================================================
bool Chain::isStillChain() const
{
  if(!isAssembled())
    return false;

  // Make sure there are no Branches and no parent FreeJoints on the BodyNodes
  // on the inside of the chain
  for(std::size_t i=1; i<mBodyNodes.size()-1; ++i)
  {
    if(mBodyNodes[i]->getNumChildBodyNodes() > 1)
      return false;

    if(dynamic_cast<FreeJoint*>(mBodyNodes[i]->getParentJoint()))
      return false;
  }

  // Make sure there is not a FreeJoint at the final BodyNode (which was not
  // tested above)
  if(mBodyNodes.size() > 1)
  {
    if(dynamic_cast<FreeJoint*>(mBodyNodes.back()->getParentJoint()))
      return false;
  }

  return true;
}

//==============================================================================
Chain::Chain(const Chain::Criteria& _criteria, const std::string& _name)
  : Linkage(_criteria, _name)
{
  // Do nothing
}

//==============================================================================
Chain::Chain(BodyNode* _start, BodyNode* _target, const std::string& _name)
  : Linkage(Chain::Criteria(_start, _target), _name)
{
  // Do nothing
}

//==============================================================================
Chain::Chain(BodyNode* _start, BodyNode* _target,
             IncludeBothTag, const std::string& _name)
  : Linkage(Chain::Criteria(_start, _target, true), _name)
{
  // Do nothing
}

} // namespace dynamics
} // namespace dart













