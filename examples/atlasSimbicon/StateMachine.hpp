/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#ifndef EXAMPLES_ATLASSIMBICON_STATEMACHINE_HPP_
#define EXAMPLES_ATLASSIMBICON_STATEMACHINE_HPP_

#include <vector>
#include <string>

#include <Eigen/Dense>

#include <dart/dart.hpp>

class State;

/// \brief StateMachine for Atlas robot
class StateMachine
{
public:
  /// \brief Constructor
  explicit StateMachine(const std::string& _name);

  /// \brief Destructor
  virtual ~StateMachine();

  //------------------------------- Setting ------------------------------------
  /// \brief Set name
  void setName(const std::string& _name);

  /// \brief Get name
  const std::string& getName() const;

  /// \brief Add state
  void addState(State* _state);

  /// \brief Set initial state
  void setInitialState(State* _state);

  //------------------------------- Control ------------------------------------
  /// \brief Initiate state. This is called when the contoller change the
  ///        current state machine to this.
  void begin(double _currentTime);

  /// \brief Compute control force and apply it to Atlas robot
  void computeControlForce(double _dt);

  /// \brief Finalize state. This is called when the state machine stransite
  ///        from this state to the next state.
  void end(double _currentTime);

  /// \brief Get current state
  State* getCurrentState();

  /// \brief Transite to the next state manually
  void transiteToNextState(double _currentTime);

  /// \brief Change state to _state
  void transiteTo(State* _state, double _currentTime);

  /// \brief Change state to a state whose names is _stateName
  void transiteTo(std::string& _stateName, double _currentTime);

  /// \brief Change state to a state whose index is _idx
  void transiteTo(std::size_t _idx, double _currentTime);

protected:
  /// \brief Name
  std::string mName;

  /// \brief States
  std::vector<State*> mStates;

  /// \brief Current state
  State* mCurrentState;

  /// \brief Started time
  double mBeginTime;

  /// \brief Stopped time
  double mEndTime;

  /// \brief Frame number
  int mFrame;

  /// \brief Elapsed time which is stopped time minus started time
  double mElapsedTime;

private:
  /// \brief Check if this state machine contains _state
  bool _containState(const State* _state) const;

  /// \brief Check if this state machine contains a state whose name is _name
  bool _containState(const std::string& _name) const;

  /// \brief Find a state whose name is _name
  State* _findState(const std::string& _name) const;
};

#endif  // EXAMPLES_ATLASSIMBICON_STATEMACHINE_HPP_
