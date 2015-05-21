/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
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

#include <algorithm>

#include "dart/dynamics/Linkage.h"
#include "dart/dynamics/FreeJoint.h"

namespace dart {
namespace dynamics {

//==============================================================================
std::vector<BodyNode*> Linkage::Criteria::satisfy() const
{
  std::vector<BodyNode*> bns;

  if(nullptr == mStart.mTarget)
  {
    dterr << "[Linkage::Criteria::satisfy] Must specify at least a starting "
          << "BodyNode for the criteria!\n";
    assert(false);
    return bns;
  }

  refreshTerminalMap();

  bns.push_back(mStart.mTarget);
  expansionPolicy(mStart.mTarget, mStart.mPolicy, bns);

  for(size_t i=0; i<mTargets.size(); ++i)
  {
    expandToTarget(mStart.mTarget, mTargets[i], bns);
  }

  return bns;
}

//==============================================================================
Linkage::Criteria::Target::Target(BodyNode* _target,
                                  ExpansionPolicy _policy,
                                  bool _chain)
  : mTarget(_target),
    mPolicy(_policy),
    mChain(_chain)
{
  // Do nothing
}

//==============================================================================
Linkage::Criteria::Terminal::Terminal(BodyNode* _terminal, bool _inclusive)
  : mTerminal(_terminal),
    mInclusive(_inclusive)
{
  // Do nothing
}

//==============================================================================
void Linkage::Criteria::refreshTerminalMap() const
{
  mMapOfTerminals.clear();
  for(size_t i=0; i<mTerminals.size(); ++i)
  {
    mMapOfTerminals[mTerminals[i].mTerminal] = mTerminals[i].mInclusive;
  }
}

//==============================================================================
void Linkage::Criteria::expansionPolicy(
    BodyNode* _start,
    Linkage::Criteria::ExpansionPolicy _policy,
    std::vector<BodyNode*>& _bns) const
{
  // If the _start is a terminal, we quit before expanding
  std::unordered_map<BodyNode*, bool>::const_iterator check_start =
      mMapOfTerminals.find(_start);
  if( check_start != mMapOfTerminals.end() )
  {
    bool inclusive = check_start->second;
    if(inclusive)
      _bns.push_back(_start);
    return;
  }

  if(DOWNSTREAM == _policy)
    expandDownstream(_start, _bns);
  else if(UPSTREAM == _policy)
    expandUpstream(_start, _bns);
}

//==============================================================================
struct Recording
{
  Recording(BodyNode* _node = nullptr, int _count = 0)
    : mNode(_node), mCount(_count) { }

  BodyNode* mNode;
  int mCount;
};

//==============================================================================
void Linkage::Criteria::expandDownstream(
    BodyNode* _start, std::vector<BodyNode*>& _bns) const
{
  std::vector<Recording> recorder;
  recorder.reserve(_start->getSkeleton()->getNumBodyNodes());

  _bns.push_back(_start);
  recorder.push_back(Recording(_start, 0));

  while(recorder.size() > 0)
  {
    Recording& r = recorder.back();
    if(r.mCount < static_cast<int>(r.mNode->getNumChildBodyNodes()))
    {
      recorder.push_back(Recording(r.mNode->getChildBodyNode(r.mCount), 0));
      ++r.mCount;
      recorder.push_back(r.mNode);
    }
    else
    {
      recorder.pop_back();
    }
  }
}

//==============================================================================
void Linkage::Criteria::expandUpstream(
    BodyNode* _start, std::vector<BodyNode*>& _bns) const
{
  std::vector<Recording> recorder;
  recorder.reserve(_start->getSkeleton()->getNumBodyNodes());

  _bns.push_back(_start);
  recorder.push_back(Recording(_start, -1));

  while(recorder.size() > 0)
  {
    Recording& r = recorder.back();

    if(r.mCount == -1)
    {
      // -1 means we need to take a step upstream
      if(r.mNode->getParentBodyNode() == nullptr)
      {
        ++r.mCount;
      }
      else if(recorder.size() == 1 ||
              r.mNode->getParentBodyNode() != recorder[recorder.size()-2].mNode)
      {
        // Go toward this node if we did not originally come from this node
        // or if we're at the first iteration
        recorder.push_back(Recording(r.mNode->getParentBodyNode(), -1));
        ++r.mCount;
        _bns.push_back(recorder.back().mNode);
      }
      else
      {
        // If we originally came from this node, then just continue to the next
        ++r.mCount;
      }
    }
    else if(r.mCount < static_cast<int>(r.mNode->getNumChildBodyNodes()))
    {
      if(recorder.size()==1)
      {
        // If we've arrived back at the bottom of the queue, we're finished
        break;
      }
      else if( r.mNode->getChildBodyNode(r.mCount)
               != recorder[recorder.size()-2].mNode)
      {
        // Go toward this node if we did not originally come from this node
        recorder.push_back(Recording(r.mNode->getChildBodyNode(r.mCount), -1));
        ++r.mCount;
        _bns.push_back(recorder.back().mNode);
      }
      else
      {
        // If we originally came from this node, then just continue to the next
        ++r.mCount;
      }
    }
  }
}


//==============================================================================
void Linkage::Criteria::expandToTarget(
    BodyNode* _start,
    const Linkage::Criteria::Target& _target,
    std::vector<BodyNode*>& _bns) const
{
  BodyNode* target_bn = _target.mTarget;
  std::vector<BodyNode*> newBns;
  newBns.reserve(target_bn->getSkeleton()->getNumBodyNodes());

  if(target_bn == nullptr || _start->descendsFrom(target_bn))
    newBns = climbToTarget(_start, target_bn);
  else if(target_bn->descendsFrom(_start))
    newBns = climbToTarget(target_bn, _start);
  else
    newBns = climbToCommonRoot(_start, target_bn);

  trimBodyNodes(newBns, _target.mChain);

  // If we have successfully reached the target, expand from there
  if(newBns.back() == _target.mTarget)
    expansionPolicy(_target.mTarget, _target.mPolicy, newBns);

  _bns.insert(_bns.end(), newBns.begin(), newBns.end());
}

//==============================================================================
std::vector<BodyNode*> Linkage::Criteria::climbToTarget(
    BodyNode* _start, BodyNode* _target) const
{
  std::vector<BodyNode*> newBns;
  newBns.reserve(_start->getSkeleton()->getNumBodyNodes());

  BodyNode* bn = _start;

  BodyNode* finalBn = nullptr==_target? nullptr : _target->getParentBodyNode();

  while( bn != finalBn && bn != nullptr )
  {
    newBns.push_back(bn);
    bn = bn->getParentBodyNode();
  }

  return newBns;
}

//==============================================================================
std::vector<BodyNode*> Linkage::Criteria::climbToCommonRoot(
    BodyNode* _start, BodyNode* _target) const
{
  BodyNode* root = _start->getParentBodyNode();
  while(root != nullptr)
  {
    if(_target->descendsFrom(root))
      break;
  }

  std::vector<BodyNode*> bnStart = climbToTarget(_start, root);
  std::vector<BodyNode*> bnTarget = climbToTarget(_target, root);
  std::reverse(bnTarget.begin(), bnTarget.end());

  std::vector<BodyNode*> bnAll;
  bnAll.reserve(bnStart.size() + bnTarget.size());
  bnAll.insert(bnAll.end(), bnStart.begin(), bnStart.end());
  bnAll.insert(bnAll.end(), bnTarget.begin(), bnTarget.end());

  return bnAll;
}

//==============================================================================
void Linkage::Criteria::trimBodyNodes(
    std::vector<BodyNode *>& _bns, bool _chain) const
{
  std::vector<BodyNode*>::iterator it = _bns.begin();
  while( it != _bns.end() )
  {
    std::unordered_map<BodyNode*, bool>::const_iterator terminal =
        mMapOfTerminals.find(*it);

    if( terminal != mMapOfTerminals.end() )
    {
      bool inclusive = terminal->second;
      if(inclusive)
        ++it;

      break;
    }

    ++it;

    if( it != _bns.end() && _chain)
    {
      if( (*it)->getNumChildBodyNodes() > 1)
        break;

      if( dynamic_cast<FreeJoint*>( (*it)->getParentJoint() ) )
        break;
    }
  }

  _bns.erase(it, _bns.end());
}

} // namespace dynamics
} // namespace dart
