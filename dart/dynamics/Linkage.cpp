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

#include "dart/dynamics/Linkage.h"

namespace dart {
namespace dynamics {

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
std::vector<BodyNode*> climbToTarget(BodyNode* _start, BodyNode* _target)
{

}

//==============================================================================
static void expandToTarget(BodyNode* _start,
                           const Linkage::Criteria::Target& _target,
                           std::vector<BodyNode*>& _bns)
{
  BodyNode* target_bn = _target.mTarget;
  if(_start->descendsFrom(target_bn))
  {

  }
  else if(target_bn->descendsFrom(_start))
  {

  }
  else
  {

  }
}

//==============================================================================
static void expansionPolicy(BodyNode* _start,
                            Linkage::Criteria::ExpansionPolicy _policy,
                            std::vector<BodyNode*>& _bns)
{

}

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
    expansionPolicy(mTargets[i].mTarget, mTargets[i].mPolicy, bns);
  }

  return bns;
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

} // namespace dynamics
} // namespace dart
