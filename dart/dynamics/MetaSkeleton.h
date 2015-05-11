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

#ifndef DART_DYNAMICS_METASKELETON_H_
#define DART_DYNAMICS_METASKELETON_H_

#include <string>

#include "dart/common/Subject.h"

namespace dart {
namespace dynamics {

class BodyNode;
class SoftBodyNode;
class PointMass;
class Joint;
class Marker;
class DegreeOfFreedom;

class MetaSkeleton : public common::Subject
{
public:

  using NameChangedSignal
      = common::Signal<void(const Skeleton* _skeleton,
                            const std::string& _oldName,
                            const std::string& _newName)>;


  MetaSkeleton();

  virtual ~MetaSkeleton() = default;

  //----------------------------------------------------------------------------
  // Properties
  //----------------------------------------------------------------------------

  /// Set the name of this MetaSkeleton
  virtual const std::string& setName(const std::string& _name) = 0;

  /// Get the name of this MetaSkeleton
  virtual const std::string& getName() const = 0;

  //----------------------------------------------------------------------------
  // Structural Properties
  //----------------------------------------------------------------------------

  /// Get number of body nodes
  virtual size_t getNumBodyNodes() const = 0;

  /// Get BodyNode whose index is _idx
  virtual BodyNode* getBodyNode(size_t _idx) = 0;

  /// Get const BodyNode whose index is _idx
  virtual const BodyNode* getBodyNode(size_t _idx) const = 0;

  /// Get number of Joints
  virtual size_t getNumJoints() const = 0;

  /// Get Joint whose index is _idx
  virtual Joint* getJoint(size_t _idx) = 0;

  /// Get const Joint whose index is _idx
  virtual const Joint* getJoint(size_t _idx) = 0;

  /// Return the number of degrees of freedom in this skeleton
  virtual size_t getNumDofs() const = 0;

  /// Get degree of freedom (aka generalized coordinate) whose index is _idx
  virtual DegreeOfFreedom* getDof(size_t _idx) = 0;

  /// Get degree of freedom (aka generalized coordinate) whose index is _idx
  virtual const DegreeOfFreedom* getDof(size_t _idx) const = 0;

  //----------------------------------------------------------------------------
  /// \{ \name Command
  //----------------------------------------------------------------------------

  /// Set a single command
  virtual void setCommand(size_t _index, double _command) = 0;

  /// Get a single command
  virtual double getCommand(size_t _index) const = 0;

  virtual void setCommands(const Eigen::VectorXd& _commands) = 0;

  virtual Eigen::VectorXd getCommands() const = 0;








  virtual double getMass() const = 0;

protected:

  //--------------------------------------------------------------------------
  // Signals
  //--------------------------------------------------------------------------
  NameChangedSignal mNameChangedSignal;

public:

  //--------------------------------------------------------------------------
  // Slot registers
  //--------------------------------------------------------------------------
  common::SlotRegister<NameChangedSignal> onNameChanged;
};

} // namespace dynamics
} // namespace dart


#endif // DART_DYNAMICS_METASKELETON_H_
