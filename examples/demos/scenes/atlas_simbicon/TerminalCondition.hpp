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

// Ported wholesale from examples/atlas_simbicon/TerminalCondition.{hpp,cpp}
// into the dart_demos::atlas_simbicon namespace. No behavior changes.

#ifndef DART_EXAMPLES_DEMOS_SCENES_ATLASSIMBICON_TERMINALCONDITION_HPP_
#define DART_EXAMPLES_DEMOS_SCENES_ATLASSIMBICON_TERMINALCONDITION_HPP_

#include <dart/dart.hpp>

namespace dart_demos {
namespace atlas_simbicon {

class State;

//==============================================================================
/// Base terminal condition: a state transitions to its next state once
/// isSatisfied() returns true.
class TerminalCondition
{
public:
  explicit TerminalCondition(State* state);
  virtual ~TerminalCondition();

  virtual bool isSatisfied() = 0;

protected:
  State* mState;
};

//==============================================================================
/// Satisfied once the owning state has been active for longer than
/// `duration` seconds.
class TimerCondition : public TerminalCondition
{
public:
  TimerCondition(State* state, double duration);
  virtual ~TimerCondition();

  bool isSatisfied() override;

protected:
  double mDuration;
};

//==============================================================================
/// Satisfied once `body` is in contact with anything (deprecated
/// BodyNode::isColliding(), same as the original).
class BodyContactCondition : public TerminalCondition
{
public:
  BodyContactCondition(State* state, dart::dynamics::BodyNode* body);
  virtual ~BodyContactCondition();

  bool isSatisfied() override;

protected:
  dart::dynamics::BodyNode* mBodyNode;
};

} // namespace atlas_simbicon
} // namespace dart_demos

#endif // DART_EXAMPLES_DEMOS_SCENES_ATLASSIMBICON_TERMINALCONDITION_HPP_
