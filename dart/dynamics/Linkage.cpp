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

#include <algorithm>
#include <unordered_set>

#include "dart/dynamics/Linkage.hpp"
#include "dart/dynamics/FreeJoint.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
std::vector<BodyNode*> Linkage::Criteria::satisfy() const
{
  std::vector<BodyNode*> bns;

  if(nullptr == mStart.mNode.lock())
  {
    if(mTargets.size() == 0)
      return bns;

    refreshTerminalMap();

    // If a start node is not given, then we must treat the root node of each
    // target as if it is a start node.

    for(std::size_t i=0; i<mTargets.size(); ++i)
    {
      const Target& target = mTargets[i];
      if(nullptr == target.mNode.lock())
        continue;

      BodyNode* target_bn = target.mNode.lock();

      Target start = mStart;
      start.mPolicy = INCLUDE;

      std::size_t treeIndex = target_bn->getTreeIndex();
      start.mNode = target_bn->getSkeleton()->getRootBodyNode(treeIndex);

      expandToTarget(start, target, bns);
    }
  }
  else
  {
    refreshTerminalMap();

    if(EXCLUDE != mStart.mPolicy)
      bns.push_back(mStart.mNode.lock());
    expansionPolicy(mStart.mNode.lock(), mStart.mPolicy, bns);

    for(std::size_t i=0; i<mTargets.size(); ++i)
    {
      expandToTarget(mStart, mTargets[i], bns);
    }
  }

  // Make sure each BodyNode is only included once
  std::vector<BodyNode*> final_bns;
  final_bns.reserve(bns.size());
  std::unordered_set<BodyNode*> unique_bns;
  unique_bns.reserve(bns.size());
  for(BodyNode* bn : bns)
  {
    if(nullptr == bn)
      continue;

    if( unique_bns.find(bn) != unique_bns.end() )
      continue;

    final_bns.push_back(bn);
    unique_bns.insert(bn);
  }

  return final_bns;
}

//==============================================================================
Linkage::Criteria::Target::Target(BodyNode* _target,
                                  ExpansionPolicy _policy,
                                  bool _chain)
  : mNode(_target),
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
  for(std::size_t i=0; i<mTerminals.size(); ++i)
  {
    mMapOfTerminals[mTerminals[i].mTerminal.lock()] = mTerminals[i].mInclusive;
  }
}

//==============================================================================
void Linkage::Criteria::expansionPolicy(
    BodyNode* _start,
    Linkage::Criteria::ExpansionPolicy _policy,
    std::vector<BodyNode*>& _bns) const
{
  if(EXCLUDE != _policy)
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
  }

  if(DOWNSTREAM == _policy)
    expandDownstream(_start, _bns, EXCLUDE != _policy);
  else if(UPSTREAM == _policy)
    expandUpstream(_start, _bns, EXCLUDE != _policy);
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
void stepToNextChild(std::vector<Recording>& _recorder,
                     std::vector<BodyNode*>& _bns,
                     Recording& _r,
                     const std::unordered_map<BodyNode*, bool>& _terminalMap,
                     int _initValue)
{
  BodyNode* bn = _r.mNode->getChildBodyNode(_r.mCount);
  std::unordered_map<BodyNode*, bool>::const_iterator it =
      _terminalMap.find(bn);

  if(it != _terminalMap.end())
  {
    bool inclusive = it->second;
    if(inclusive)
      _bns.push_back(bn);

    ++_r.mCount;
    return;
  }

  _recorder.push_back(Recording(bn, _initValue));
  _bns.push_back(bn);
}

//==============================================================================
void stepToParent(std::vector<Recording>& _recorder,
                  std::vector<BodyNode*>& _bns,
                  Recording& _r,
                  const std::unordered_map<BodyNode*, bool>& _terminalMap)
{
  BodyNode* bn = _r.mNode->getParentBodyNode();
  std::unordered_map<BodyNode*, bool>::const_iterator it =
      _terminalMap.find(bn);

  if(it != _terminalMap.end())
  {
    bool inclusive = it->second;
    if(inclusive)
      _bns.push_back(bn);

    ++_r.mCount;
    return;
  }

  _recorder.push_back(Recording(bn, -1));
  _bns.push_back(bn);
}

//==============================================================================
void Linkage::Criteria::expandDownstream(
    BodyNode* _start, std::vector<BodyNode*>& _bns, bool _includeStart) const
{
  std::vector<Recording> recorder;
  recorder.reserve(_start->getSkeleton()->getNumBodyNodes());

  if(_includeStart)
    _bns.push_back(_start);
  recorder.push_back(Recording(_start, 0));

  while(recorder.size() > 0)
  {
    Recording& r = recorder.back();
    if(r.mCount < static_cast<int>(r.mNode->getNumChildBodyNodes()))
    {
      stepToNextChild(recorder, _bns, r, mMapOfTerminals, 0);
    }
    else
    {
      recorder.pop_back();
      if(recorder.size() > 0)
        ++recorder.back().mCount;
    }
  }
}

//==============================================================================
void Linkage::Criteria::expandUpstream(
    BodyNode* _start, std::vector<BodyNode*>& _bns, bool _includeStart) const
{
  std::vector<Recording> recorder;
  recorder.reserve(_start->getSkeleton()->getNumBodyNodes());

  if(_includeStart)
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
        // If the parent is a nullptr, we have reached the root
        ++r.mCount;
      }
      else if(recorder.size() == 1 ||
              r.mNode->getParentBodyNode() != recorder[recorder.size()-2].mNode)
      {
        // Go toward this node if we did not originally come from this node
        // or if we're at the first iteration
        stepToParent(recorder, _bns, r, mMapOfTerminals);
      }
      else
      {
        // If we originally came from this node, then just continue to the next
        ++r.mCount;
      }
    }
    else if(r.mCount < static_cast<int>(r.mNode->getNumChildBodyNodes()))
    {
      // Greater than -1 means we need to add the children

      if(recorder.size()==1)
      {
        // If we've arrived back at the bottom of the queue, we're finished,
        // because we don't want to go downstream of the starting BodyNode
        break;
      }
      else if( r.mNode->getChildBodyNode(r.mCount)
               != recorder[recorder.size()-2].mNode)
      {
        // Go toward this node if we did not originally come from this node
        stepToNextChild(recorder, _bns, r, mMapOfTerminals, -1);
      }
      else
      {
        // If we originally came from this node, then just continue to the next
        ++r.mCount;
      }
    }
    else
    {
      // If we've iterated through all the children of this node, pop it
      recorder.pop_back();
      // Move on to the next child
      if(recorder.size() > 0)
        ++recorder.back().mCount;
    }
  }
}

//==============================================================================
void Linkage::Criteria::expandToTarget(
    const Linkage::Criteria::Target& _start,
    const Linkage::Criteria::Target& _target,
    std::vector<BodyNode*>& _bns) const
{
  BodyNode* start_bn = _start.mNode.lock();
  BodyNode* target_bn = _target.mNode.lock();
  std::vector<BodyNode*> newBns;
  newBns.reserve(start_bn->getSkeleton()->getNumBodyNodes());

  if(target_bn == nullptr || start_bn->descendsFrom(target_bn))
  {
    newBns = climbToTarget(start_bn, target_bn);
    trimBodyNodes(newBns, _target.mChain, true);
  }
  else if(target_bn->descendsFrom(start_bn))
  {
    newBns = climbToTarget(target_bn, start_bn);
    std::reverse(newBns.begin(), newBns.end());
    trimBodyNodes(newBns, _target.mChain, false);
  }
  else
  {
    newBns = climbToCommonRoot(_start, _target, _target.mChain);
  }

  // Remove the start BodyNode if it's supposed to be excluded
  if(EXCLUDE == _start.mPolicy &&
     newBns.size() > 0 && newBns.front() == start_bn)
  {
    newBns.erase(newBns.begin());
  }

  // Remove the target BodyNode if it's supposed to be excluded
  if(EXCLUDE == _target.mPolicy &&
     newBns.size() > 0 && newBns.back() == target_bn)
  {
    newBns.pop_back();
  }

  // If we have successfully reached the target, expand from there
  if(!newBns.empty() && newBns.back() == _target.mNode.lock())
    expansionPolicy(_target.mNode.lock(), _target.mPolicy, newBns);

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
    const Target& _start, const Target& _target,
    bool _chain) const
{
  BodyNode* start_bn = _start.mNode.lock();
  BodyNode* target_bn = _target.mNode.lock();
  BodyNode* root = start_bn->getParentBodyNode();
  while(root != nullptr)
  {
    if(target_bn->descendsFrom(root))
      break;

    root = root->getParentBodyNode();
  }

  std::vector<BodyNode*> bnStart = climbToTarget(start_bn, root);
  trimBodyNodes(bnStart, _chain, true);

  if(root != nullptr && bnStart.back() != root)
  {
    // We did not reach the common root, so we should stop here
    return bnStart;
  }

  std::vector<BodyNode*> bnTarget = climbToTarget(target_bn, root);
  std::reverse(bnTarget.begin(), bnTarget.end());
  trimBodyNodes(bnTarget, _chain, false);

  std::vector<BodyNode*> bnAll;
  bnAll.reserve(bnStart.size() + bnTarget.size());
  bnAll.insert(bnAll.end(), bnStart.begin(), bnStart.end());
  bnAll.insert(bnAll.end(), bnTarget.begin(), bnTarget.end());

  return bnAll;
}

//==============================================================================
void Linkage::Criteria::trimBodyNodes(std::vector<BodyNode *>& _bns,
                                      bool _chain, bool _movingUpstream) const
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
      // If this BodyNode has multiple children, cut off any BodyNodes that
      // follow it
      if( (*it)->getNumChildBodyNodes() > 1 )
      {
        if(_movingUpstream)
        {
          break;
        }
        else
        {
          ++it;
          break;
        }
      }

      if( dynamic_cast<FreeJoint*>( (*it)->getParentJoint() ) )
      {
        if(_movingUpstream)
        {
          ++it;
          break;
        }
        else
        {
          break;
        }
      }
    }
  }

  _bns.erase(it, _bns.end());
}

//==============================================================================
LinkagePtr Linkage::create(const Criteria &_criteria, const std::string& _name)
{
  LinkagePtr linkage(new Linkage(_criteria, _name));
  linkage->mPtr = linkage;
  return linkage;
}

//==============================================================================
bool Linkage::isAssembled() const
{
  for(std::size_t i=0; i<mParentBodyNodes.size(); ++i)
  {
    const BodyNode* bn = mBodyNodes[i];
    if(bn->getParentBodyNode() != mParentBodyNodes[i].lock())
      return false;
  }

  return true;
}

//==============================================================================
void Linkage::reassemble()
{
  for(std::size_t i=0; i<mBodyNodes.size(); ++i)
  {
    BodyNode* bn = mBodyNodes[i];
    bn->moveTo(mParentBodyNodes[i].lock());
  }
}

//==============================================================================
void Linkage::satisfyCriteria()
{
  std::vector<BodyNode*> bns = mCriteria.satisfy();
  while(getNumBodyNodes() > 0)
    unregisterComponent(mBodyNodes.back());

  for(BodyNode* bn : bns)
  {
    registerComponent(bn);
  }

  update();
}

//==============================================================================
Linkage::Linkage(const Criteria& _criteria, const std::string& _name)
  : mCriteria(_criteria)
{
  setName(_name);
  satisfyCriteria();
}

//==============================================================================
void Linkage::update()
{
  mParentBodyNodes.clear();
  mParentBodyNodes.reserve(mBodyNodes.size());
  for(std::size_t i=0; i<mBodyNodes.size(); ++i)
  {
    mParentBodyNodes.push_back(mBodyNodes[i]->getParentBodyNode());
  }
}

} // namespace dynamics
} // namespace dart
