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

// Ported wholesale from examples/atlas_simbicon/StateMachine.{hpp,cpp} into
// the dart_demos::atlas_simbicon namespace. No behavior changes.

#ifndef DART_EXAMPLES_DEMOS_SCENES_ATLASSIMBICON_STATEMACHINE_HPP_
#define DART_EXAMPLES_DEMOS_SCENES_ATLASSIMBICON_STATEMACHINE_HPP_

#include <dart/dart.hpp>

#include <string>
#include <vector>

namespace dart_demos {
namespace atlas_simbicon {

class State;

//==============================================================================
/// A named sequence of States (e.g. "walking", "standing"), transitioning
/// from one to the next whenever the current State's terminal condition is
/// satisfied.
class StateMachine
{
public:
  explicit StateMachine(const std::string& name);
  virtual ~StateMachine();

  void setName(const std::string& name);
  const std::string& getName() const;

  void addState(State* state);
  void setInitialState(State* state);

  /// Called when the controller switches to this state machine.
  void begin(double currentTime);

  /// Computes control force for the current state and applies it.
  void computeControlForce(double dt);

  /// Called when the controller switches away from this state machine.
  void end(double currentTime);

  State* getCurrentState();

  void transiteToNextState(double currentTime);
  void transiteTo(State* state, double currentTime);
  void transiteTo(std::string& stateName, double currentTime);
  void transiteTo(std::size_t idx, double currentTime);

  void setVerbosity(bool verbosity);

protected:
  std::string mName;
  std::vector<State*> mStates;
  State* mCurrentState;
  double mBeginTime;
  double mEndTime;
  int mFrame;
  double mElapsedTime;

private:
  bool _containState(const State* state) const;
  bool _containState(const std::string& name) const;
  State* _findState(const std::string& name) const;

  bool mVerbosity;
};

} // namespace atlas_simbicon
} // namespace dart_demos

#endif // DART_EXAMPLES_DEMOS_SCENES_ATLASSIMBICON_STATEMACHINE_HPP_
