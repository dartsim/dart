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

#include "dart/common/Console.hpp"
#include "dart/dynamics/MetaSkeleton.hpp"
#include "dart/dynamics/DegreeOfFreedom.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
static bool checkIndexArrayValidity(const MetaSkeleton* skel,
                                    const std::vector<std::size_t>& _indices,
                                    const std::string& _fname)
{
  std::size_t dofs = skel->getNumDofs();
  for(std::size_t i=0; i<_indices.size(); ++i)
  {
    if(_indices[i] >= dofs)
    {
      if(dofs > 0)
      {
        dterr << "[Skeleton::" << _fname << "] Invalid entry (" << i << ") in "
              << "_indices array: " << _indices[i] << ". Value must be less than "
              << dofs << " for the Skeleton named [" << skel->getName() << "] ("
              << skel << ")\n";
      }
      else
      {
        dterr << "[Skeleton::" << _fname << "] The Skeleton named ["
              << skel->getName() << "] (" << skel << ") is empty, but _indices "
              << "has entries in it. Nothing will be set!\n";
      }

      return false;
    }
  }
  return true;
}

//==============================================================================
static bool checkIndexArrayAgreement(const MetaSkeleton* skel,
                                     const std::vector<std::size_t>& _indices,
                                     const Eigen::VectorXd& _values,
                                     const std::string& _fname,
                                     const std::string& _vname)
{
  if( static_cast<int>(_indices.size()) != _values.size() )
  {
    dterr << "[Skeleton::" << _fname << "] Mismatch between _indices size ("
          << _indices.size() << ") and " << _vname << " size ("
          << _values.size() << ") for Skeleton named [" << skel->getName()
          << "] (" << skel << "). Nothing will be set!\n";
    assert(false);
    return false;
  }

  return checkIndexArrayValidity(skel, _indices, _fname);
}

//==============================================================================
template <void (DegreeOfFreedom::*setValue)(double _value)>
static void setValuesFromVector(MetaSkeleton* skel,
                                const std::vector<std::size_t>& _indices,
                                const Eigen::VectorXd& _values,
                                const std::string& _fname,
                                const std::string& _vname)
{
  if(!checkIndexArrayAgreement(skel, _indices, _values, _fname, _vname))
    return;

  for (std::size_t i=0; i<_indices.size(); ++i)
  {
    DegreeOfFreedom* dof = skel->getDof(_indices[i]);
    if(dof)
    {
      (dof->*setValue)(_values[i]);
    }
    else
    {
      dterr << "[MetaSkeleton::" << _fname << "] DegreeOfFreedom #"
            << _indices[i] << " (entry #" << i << " in " << _vname << ") has "
            << "expired! ReferentialSkeletons should call update() after "
            << "structural changes have been made to the BodyNodes they refer "
            << "to. Nothing will be set for this specific DegreeOfFreedom.\n";
      assert(false);
    }
  }
}

//==============================================================================
template <void (DegreeOfFreedom::*setValue)(double _value)>
static void setAllValuesFromVector(MetaSkeleton* skel,
                                   const Eigen::VectorXd& _values,
                                   const std::string& _fname,
                                   const std::string& _vname)
{
  std::size_t nDofs = skel->getNumDofs();
  if( _values.size() != static_cast<int>(skel->getNumDofs()) )
  {
    dterr << "[MetaSkeleton::" << _fname << "] Invalid number of entries ("
          << _values.size() << ") in " << _vname << " for MetaSkeleton named ["
          << skel->getName() << "] (" << skel << "). Must be equal to ("
          << skel->getNumDofs() << "). Nothing will be set!\n";
    assert(false);
    return;
  }

  for(std::size_t i=0; i < nDofs; ++i)
  {
    DegreeOfFreedom* dof = skel->getDof(i);
    if(dof)
    {
      (dof->*setValue)(_values[i]);
    }
    else
    {
      dterr << "[MetaSkeleton::" << _fname << "] DegreeOfFreedom #" << i
            << " in the MetaSkeleton named [" << skel->getName() << "] ("
            << skel << ") has expired! ReferentialSkeletons should call "
            << "update() after structural changes have been made to the "
            << "BodyNodes they refer to. Nothing will be set for this specific "
            << "DegreeOfFreedom.\n";
      assert(false);
    }
  }
}

//==============================================================================
template <double (DegreeOfFreedom::*getValue)() const>
static Eigen::VectorXd getValuesFromVector(
    const MetaSkeleton* skel, const std::vector<std::size_t>& _indices,
    const std::string& _fname)
{
  Eigen::VectorXd values(_indices.size());

  for(std::size_t i=0; i<_indices.size(); ++i)
  {
    const DegreeOfFreedom* dof = skel->getDof(_indices[i]);
    if(dof)
    {
      values[i] = (dof->*getValue)();
    }
    else
    {
      values[i] = 0.0;
      if(i < skel->getNumDofs())
      {
        dterr << "[MetaSkeleton::" << _fname << "] Requesting value for "
              << "DegreeOfFreedom #" << _indices[i] << " (" << "entry #" << i
              << " in _indices), but this index has expired! "
              << "ReferentialSkeletons should call update() after structural "
              << "changes have been made to the BodyNodes they refer to. The "
              << "return value for this entry will be zero.\n";
      }
      else
      {
        dterr << "[MetaSkeleton::" << _fname << "] Requesting out of bounds "
              << "DegreeOfFreedom #" << _indices[i] << " (entry #" << i
              << " in _indices) for MetaSkeleton named [" << skel->getName()
              << "] (" << skel << "). The max index is (" << skel->getNumDofs()
              << "). The return value for this entry will be zero.\n";
      }
      assert(false);
    }
  }

  return values;
}

//==============================================================================
template <double (DegreeOfFreedom::*getValue)() const>
static Eigen::VectorXd getValuesFromAllDofs(
    const MetaSkeleton* skel, const std::string& _fname)
{
  std::size_t nDofs = skel->getNumDofs();
  Eigen::VectorXd values(nDofs);

  for(std::size_t i=0; i<nDofs; ++i)
  {
    const DegreeOfFreedom* dof = skel->getDof(i);
    if(dof)
    {
      values[i] = (skel->getDof(i)->*getValue)();
    }
    else
    {
      dterr << "[MetaSkeleton::" << _fname << "] DegreeOfFreedom #" << i
            << " has expired! ReferentialSkeletons should call update() after "
            << "structural changes have been made to the BodyNodes they refer "
            << "to. The return value for this entry will be zero.\n";
      values[i] = 0.0;
      assert(false);
    }
  }

  return values;
}

//==============================================================================
template <void (DegreeOfFreedom::*apply)()>
static void applyToAllDofs(MetaSkeleton* skel)
{
  std::size_t nDofs = skel->getNumDofs();
  for(std::size_t i=0; i<nDofs; ++i)
  {
    DegreeOfFreedom* dof = skel->getDof(i);
    if(dof)
      (dof->*apply)();
  }
}

//==============================================================================
template <void (DegreeOfFreedom::*setValue)(double _value)>
static void setValueFromIndex(MetaSkeleton* skel, std::size_t _index, double _value,
                              const std::string& _fname)
{
  if(_index >= skel->getNumDofs())
  {
    if(skel->getNumDofs() > 0)
      dterr << "[MetaSkeleton::" << _fname << "] Out of bounds index ("
            << _index << ") for MetaSkeleton named [" << skel->getName()
            << "] (" << skel << "). Must be less than " << skel->getNumDofs()
            << "!\n";
    else
      dterr << "[MetaSkeleton::" << _fname << "] Index (" << _index
            << ") cannot be used on MetaSkeleton [" << skel->getName() << "] ("
            << skel << ") because it is empty!\n";
    assert(false);
    return;
  }

  DegreeOfFreedom* dof = skel->getDof(_index);
  if(dof)
  {
    (dof->*setValue)(_value);
  }
  else
  {
    dterr << "[MetaSkeleton::" << _fname << "] DegreeOfFreedom #" << _index
          << " in the MetaSkeleton named [" << skel->getName() << "] (" << skel
          << ") has expired! ReferentialSkeletons should call update() after "
          << "structural changes have been made to the BodyNodes they refer "
          << "to. Nothing will be set!\n";
    assert(false);
  }
}

//==============================================================================
template <double (DegreeOfFreedom::*getValue)() const>
static double getValueFromIndex(const MetaSkeleton* skel, std::size_t _index,
                                const std::string& _fname)
{
  if(_index >= skel->getNumDofs())
  {
    if(skel->getNumDofs() > 0)
      dterr << "[MetaSkeleton::" << _fname << "] Out of bounds index ("
            << _index << ") for MetaSkeleton named [" << skel->getName()
            << "] (" << skel << "). Must be less than " << skel->getNumDofs()
            << "! The return value will be zero.\n";
    else
      dterr << "[MetaSkeleton::" << _fname << "] Index (" << _index
            << ") cannot " << "be requested for MetaSkeleton ["
            << skel->getName() << "] (" << skel << ") because it is empty! "
            << "The return value will be zero.\n";
    assert(false);
    return 0.0;
  }

  const DegreeOfFreedom* dof = skel->getDof(_index);
  if(dof)
  {
    return (skel->getDof(_index)->*getValue)();
  }

  dterr << "[MetaSkeleton::" << _fname << "] DegreeOfFreedom #" << _index
        << "in the MetaSkeleton named [" << skel->getName() << "] (" << skel
        << ") has expired! ReferentialSkeletons should call update() after "
        << "structural changes have been made to the BodyNodes they refer to. "
        << "The return value will be zero.\n";
  assert(false);
  return 0.0;
}

//==============================================================================
void MetaSkeleton::setCommand(std::size_t _index, double _command)
{
  setValueFromIndex<&DegreeOfFreedom::setCommand>(
        this, _index, _command, "setCommand");
}

//==============================================================================
double MetaSkeleton::getCommand(std::size_t _index) const
{
  return getValueFromIndex<&DegreeOfFreedom::getCommand>(
        this, _index, "getCommand");
}

//==============================================================================
void MetaSkeleton::setCommands(const Eigen::VectorXd& _commands)
{
  setAllValuesFromVector<&DegreeOfFreedom::setCommand>(
        this, _commands, "setCommands", "_commands");
}

//==============================================================================
void MetaSkeleton::setCommands(const std::vector<std::size_t>& _indices,
                           const Eigen::VectorXd& _commands)
{
  setValuesFromVector<&DegreeOfFreedom::setCommand>(
        this, _indices, _commands, "setCommands", "_commands");
}

//==============================================================================
Eigen::VectorXd MetaSkeleton::getCommands() const
{
  return getValuesFromAllDofs<&DegreeOfFreedom::getCommand>(
        this, "getCommands");
}

//==============================================================================
Eigen::VectorXd MetaSkeleton::getCommands(const std::vector<std::size_t>& _indices) const
{
  return getValuesFromVector<&DegreeOfFreedom::getCommand>(
        this, _indices, "getCommands");
}

//==============================================================================
void MetaSkeleton::resetCommands()
{
  applyToAllDofs<&DegreeOfFreedom::resetCommand>(this);
}

//==============================================================================
void MetaSkeleton::setPosition(std::size_t _index, double _position)
{
  setValueFromIndex<&DegreeOfFreedom::setPosition>(
        this, _index, _position, "setPosition");
}

//==============================================================================
double MetaSkeleton::getPosition(std::size_t _index) const
{
  return getValueFromIndex<&DegreeOfFreedom::getPosition>(
        this, _index, "getPosition");
}

//==============================================================================
void MetaSkeleton::setPositions(const Eigen::VectorXd& _positions)
{
  setAllValuesFromVector<&DegreeOfFreedom::setPosition>(
        this, _positions, "setPositions", "_positions");
}

//==============================================================================
void MetaSkeleton::setPositions(const std::vector<std::size_t>& _indices,
                            const Eigen::VectorXd& _positions)
{
  setValuesFromVector<&DegreeOfFreedom::setPosition>(
        this, _indices, _positions, "setPositions", "_positions");
}

//==============================================================================
Eigen::VectorXd MetaSkeleton::getPositions() const
{
  return getValuesFromAllDofs<&DegreeOfFreedom::getPosition>(
        this, "getPositions");
}

//==============================================================================
Eigen::VectorXd MetaSkeleton::getPositions(const std::vector<std::size_t>& _indices) const
{
  return getValuesFromVector<&DegreeOfFreedom::getPosition>(
        this, _indices, "getPositions");
}

//==============================================================================
void MetaSkeleton::resetPositions()
{
  applyToAllDofs<&DegreeOfFreedom::resetPosition>(this);
}

//==============================================================================
void MetaSkeleton::setPositionLowerLimit(std::size_t _index, double _position)
{
  setValueFromIndex<&DegreeOfFreedom::setPositionLowerLimit>(
        this, _index, _position, "setPositionLowerLimit");
}

//==============================================================================
void MetaSkeleton::setPositionLowerLimits(const Eigen::VectorXd& positions)
{
  setAllValuesFromVector<&DegreeOfFreedom::setPositionLowerLimit>(
        this, positions, "setPositionLowerLimits", "positions");
}

//==============================================================================
void MetaSkeleton::setPositionLowerLimits(
    const std::vector<std::size_t>& indices, const Eigen::VectorXd& positions)
{
  setValuesFromVector<&DegreeOfFreedom::setPositionLowerLimit>(
        this, indices, positions, "setPositionLowerLimits", "positions");
}

//==============================================================================
double MetaSkeleton::getPositionLowerLimit(std::size_t _index) const
{
  return getValueFromIndex<&DegreeOfFreedom::getPositionLowerLimit>(
        this, _index, "getPositionLowerLimit");
}

//==============================================================================
Eigen::VectorXd MetaSkeleton::getPositionLowerLimits() const
{
  return getValuesFromAllDofs<&DegreeOfFreedom::getPositionLowerLimit>(
        this, "getPositionLowerLimits");
}

//==============================================================================
Eigen::VectorXd MetaSkeleton::getPositionLowerLimits(
    const std::vector<std::size_t>& indices) const
{
  return getValuesFromVector<&DegreeOfFreedom::getPositionLowerLimit>(
        this, indices, "getPositionLowerLimits");
}

//==============================================================================
void MetaSkeleton::setPositionUpperLimit(std::size_t _index, double _position)
{
  setValueFromIndex<&DegreeOfFreedom::setPositionUpperLimit>(
        this, _index, _position, "setPositionUpperLimit");
}

//==============================================================================
void MetaSkeleton::setPositionUpperLimits(const Eigen::VectorXd& positions)
{
  setAllValuesFromVector<&DegreeOfFreedom::setPositionUpperLimit>(
        this, positions, "setPositionUpperLimits", "positions");
}

//==============================================================================
void MetaSkeleton::setPositionUpperLimits(
    const std::vector<std::size_t>& indices, const Eigen::VectorXd& positions)
{
  setValuesFromVector<&DegreeOfFreedom::setPositionUpperLimit>(
        this, indices, positions, "setPositionUpperLimits", "positions");
}

//==============================================================================
double MetaSkeleton::getPositionUpperLimit(std::size_t _index) const
{
  return getValueFromIndex<&DegreeOfFreedom::getPositionUpperLimit>(
        this, _index, "getPositionUpperLimit");
}

//==============================================================================
Eigen::VectorXd MetaSkeleton::getPositionUpperLimits() const
{
  return getValuesFromAllDofs<&DegreeOfFreedom::getPositionUpperLimit>(
        this, "getPositionUpperLimits");
}

//==============================================================================
Eigen::VectorXd MetaSkeleton::getPositionUpperLimits(
    const std::vector<std::size_t>& indices) const
{
  return getValuesFromVector<&DegreeOfFreedom::getPositionUpperLimit>(
        this, indices, "getPositionUpperLimits");
}

//==============================================================================
void MetaSkeleton::setVelocity(std::size_t _index, double _velocity)
{
  setValueFromIndex<&DegreeOfFreedom::setVelocity>(
        this, _index, _velocity, "setVelocity");
}

//==============================================================================
double MetaSkeleton::getVelocity(std::size_t _index) const
{
  return getValueFromIndex<&DegreeOfFreedom::getVelocity>(
        this, _index, "getVelocity");
}

//==============================================================================
void MetaSkeleton::setVelocities(const Eigen::VectorXd& _velocities)
{
  setAllValuesFromVector<&DegreeOfFreedom::setVelocity>(
        this, _velocities, "setVelocities", "_velocities");
}

//==============================================================================
void MetaSkeleton::setVelocities(const std::vector<std::size_t>& _indices,
                             const Eigen::VectorXd& _velocities)
{
  setValuesFromVector<&DegreeOfFreedom::setVelocity>(
        this, _indices, _velocities, "setVelocities", "_velocities");
}

//==============================================================================
Eigen::VectorXd MetaSkeleton::getVelocities() const
{
  return getValuesFromAllDofs<&DegreeOfFreedom::getVelocity>(
        this, "getVelocities");
}

//==============================================================================
Eigen::VectorXd MetaSkeleton::getVelocities(const std::vector<std::size_t>& _indices) const
{
  return getValuesFromVector<&DegreeOfFreedom::getVelocity>(
        this, _indices, "getVelocities");
}

//==============================================================================
void MetaSkeleton::resetVelocities()
{
  applyToAllDofs<&DegreeOfFreedom::resetVelocity>(this);
}

//==============================================================================
void MetaSkeleton::setVelocityLowerLimit(std::size_t _index, double _velocity)
{
  setValueFromIndex<&DegreeOfFreedom::setVelocityLowerLimit>(
        this, _index, _velocity, "setVelocityLowerLimit");
}

//==============================================================================
void MetaSkeleton::setVelocityLowerLimits(const Eigen::VectorXd& velocities)
{
  setAllValuesFromVector<&DegreeOfFreedom::setVelocityLowerLimit>(
        this, velocities, "setVelocityLowerLimits", "velocities");
}

//==============================================================================
void MetaSkeleton::setVelocityLowerLimits(
    const std::vector<std::size_t>& indices, const Eigen::VectorXd& velocities)
{
  setValuesFromVector<&DegreeOfFreedom::setVelocityLowerLimit>(
        this, indices, velocities, "setVelocityLowerLimits", "velocities");
}

//==============================================================================
double MetaSkeleton::getVelocityLowerLimit(std::size_t _index)
{
  return getValueFromIndex<&DegreeOfFreedom::getVelocityLowerLimit>(
        this, _index, "getVelocityLowerLimit");
}

//==============================================================================
Eigen::VectorXd MetaSkeleton::getVelocityLowerLimits() const
{
  return getValuesFromAllDofs<&DegreeOfFreedom::getVelocityLowerLimit>(
        this, "getVelocityLowerLimits");
}

//==============================================================================
Eigen::VectorXd MetaSkeleton::getVelocityLowerLimits(
    const std::vector<std::size_t>& indices) const
{
  return getValuesFromVector<&DegreeOfFreedom::getVelocityLowerLimit>(
        this, indices, "getVelocityLowerLimits");
}

//==============================================================================
void MetaSkeleton::setVelocityUpperLimit(std::size_t _index, double _velocity)
{
  setValueFromIndex<&DegreeOfFreedom::setVelocityUpperLimit>(
        this, _index, _velocity, "setVelocityUpperLimit");
}

//==============================================================================
void MetaSkeleton::setVelocityUpperLimits(const Eigen::VectorXd& velocities)
{
  setAllValuesFromVector<&DegreeOfFreedom::setVelocityUpperLimit>(
        this, velocities, "setVelocityUpperLimits", "velocities");
}

//==============================================================================
void MetaSkeleton::setVelocityUpperLimits(
    const std::vector<std::size_t>& indices, const Eigen::VectorXd& velocities)
{
  setValuesFromVector<&DegreeOfFreedom::setVelocityUpperLimit>(
        this, indices, velocities, "setVelocityUpperLimits", "velocities");
}

//==============================================================================
double MetaSkeleton::getVelocityUpperLimit(std::size_t _index)
{
  return getValueFromIndex<&DegreeOfFreedom::getVelocityUpperLimit>(
        this, _index, "getVelocityUpperLimit");
}

//==============================================================================
Eigen::VectorXd MetaSkeleton::getVelocityUpperLimits() const
{
  return getValuesFromAllDofs<&DegreeOfFreedom::getVelocityUpperLimit>(
        this, "getVelocityUpperLimits");
}

//==============================================================================
Eigen::VectorXd MetaSkeleton::getVelocityUpperLimits(
    const std::vector<std::size_t>& indices) const
{
  return getValuesFromVector<&DegreeOfFreedom::getVelocityUpperLimit>(
        this, indices, "getVelocityUpperLimits");
}

//==============================================================================
void MetaSkeleton::setAcceleration(std::size_t _index, double _acceleration)
{
  setValueFromIndex<&DegreeOfFreedom::setAcceleration>(
        this, _index, _acceleration, "setAcceleration");
}

//==============================================================================
double MetaSkeleton::getAcceleration(std::size_t _index) const
{
  return getValueFromIndex<&DegreeOfFreedom::getAcceleration>(
        this, _index, "getAcceleration");
}

//==============================================================================
void MetaSkeleton::setAccelerations(const Eigen::VectorXd& _accelerations)
{
  setAllValuesFromVector<&DegreeOfFreedom::setAcceleration>(
        this, _accelerations, "setAccelerations", "_accelerations");
}

//==============================================================================
void MetaSkeleton::setAccelerations(const std::vector<std::size_t>& _indices,
                                const Eigen::VectorXd& _accelerations)
{
  setValuesFromVector<&DegreeOfFreedom::setAcceleration>(
        this, _indices, _accelerations, "setAccelerations", "_accelerations");
}

//==============================================================================
Eigen::VectorXd MetaSkeleton::getAccelerations() const
{
  return getValuesFromAllDofs<&DegreeOfFreedom::getAcceleration>(
        this, "getAccelerations");
}

//==============================================================================
Eigen::VectorXd MetaSkeleton::getAccelerations(
    const std::vector<std::size_t>& _indices) const
{
  return getValuesFromVector<&DegreeOfFreedom::getAcceleration>(
        this, _indices, "getAccelerations");
}

//==============================================================================
void MetaSkeleton::resetAccelerations()
{
  applyToAllDofs<&DegreeOfFreedom::resetAcceleration>(this);
}

//==============================================================================
void MetaSkeleton::setAccelerationLowerLimit(std::size_t _index, double _acceleration)
{
  setValueFromIndex<&DegreeOfFreedom::setAccelerationLowerLimit>(
        this, _index, _acceleration, "setAccelerationLowerLimit");
}

//==============================================================================
void MetaSkeleton::setAccelerationLowerLimits(
    const Eigen::VectorXd& accelerations)
{
  setAllValuesFromVector<&DegreeOfFreedom::setAccelerationLowerLimit>(
        this, accelerations, "setAccelerationLowerLimits", "accelerations");
}

//==============================================================================
void MetaSkeleton::setAccelerationLowerLimits(
    const std::vector<std::size_t>& indices,
    const Eigen::VectorXd& accelerations)
{
  setValuesFromVector<&DegreeOfFreedom::setAccelerationLowerLimit>(
        this, indices, accelerations, "setAccelerationLowerLimits",
        "accelerations");
}

//==============================================================================
double MetaSkeleton::getAccelerationLowerLimit(std::size_t _index) const
{
  return getValueFromIndex<&DegreeOfFreedom::getAccelerationLowerLimit>(
        this, _index, "getAccelerationLowerLimit");
}

//==============================================================================
Eigen::VectorXd MetaSkeleton::getAccelerationLowerLimits() const
{
  return getValuesFromAllDofs<&DegreeOfFreedom::getAccelerationLowerLimit>(
        this, "getAccelerationLowerLimits");
}

//==============================================================================
Eigen::VectorXd MetaSkeleton::getAccelerationLowerLimits(
    const std::vector<std::size_t>& indices) const
{
  return getValuesFromVector<&DegreeOfFreedom::getAccelerationLowerLimit>(
        this, indices, "getAccelerationLowerLimits");
}

//==============================================================================
void MetaSkeleton::setAccelerationUpperLimit(std::size_t _index, double _acceleration)
{
  setValueFromIndex<&DegreeOfFreedom::setAccelerationUpperLimit>(
        this, _index, _acceleration, "setAccelerationUpperLimit");
}

//==============================================================================
void MetaSkeleton::setAccelerationUpperLimits(
    const Eigen::VectorXd& accelerations)
{
  setAllValuesFromVector<&DegreeOfFreedom::setAccelerationUpperLimit>(
        this, accelerations, "setAccelerationUpperLimits", "accelerations");
}

//==============================================================================
void MetaSkeleton::setAccelerationUpperLimits(
    const std::vector<std::size_t>& indices,
    const Eigen::VectorXd& accelerations)
{
  setValuesFromVector<&DegreeOfFreedom::setAccelerationUpperLimit>(
        this, indices, accelerations, "setAccelerationUpperLimits",
        "accelerations");
}

//==============================================================================
double MetaSkeleton::getAccelerationUpperLimit(std::size_t _index) const
{
  return getValueFromIndex<&DegreeOfFreedom::getAccelerationUpperLimit>(
        this, _index, "getAccelerationUpperLimit");
}

//==============================================================================
Eigen::VectorXd MetaSkeleton::getAccelerationUpperLimits() const
{
  return getValuesFromAllDofs<&DegreeOfFreedom::getAccelerationUpperLimit>(
        this, "getAccelerationUpperLimits");
}

//==============================================================================
Eigen::VectorXd MetaSkeleton::getAccelerationUpperLimits(
    const std::vector<std::size_t>& indices) const
{
  return getValuesFromVector<&DegreeOfFreedom::getAccelerationUpperLimit>(
        this, indices, "getAccelerationUpperLimits");
}

//==============================================================================
void MetaSkeleton::setForce(std::size_t _index, double _force)
{
  setValueFromIndex<&DegreeOfFreedom::setForce>(
        this, _index, _force, "setForce");
}

//==============================================================================
double MetaSkeleton::getForce(std::size_t _index) const
{
  return getValueFromIndex<&DegreeOfFreedom::getForce>(
        this, _index, "getForce");
}

//==============================================================================
void MetaSkeleton::setForces(const Eigen::VectorXd& _forces)
{
  setAllValuesFromVector<&DegreeOfFreedom::setForce>(
        this, _forces, "setForces", "_forces");
}

//==============================================================================
void MetaSkeleton::setForces(const std::vector<std::size_t>& _indices,
                         const Eigen::VectorXd& _forces)
{
  setValuesFromVector<&DegreeOfFreedom::setForce>(
        this, _indices, _forces, "setForces", "_forces");
}

//==============================================================================
Eigen::VectorXd MetaSkeleton::getForces() const
{
  return getValuesFromAllDofs<&DegreeOfFreedom::getForce>(
        this, "getForces");
}

//==============================================================================
Eigen::VectorXd MetaSkeleton::getForces(const std::vector<std::size_t>& _indices) const
{
  return getValuesFromVector<&DegreeOfFreedom::getForce>(
        this, _indices, "getForces");
}

//==============================================================================
void MetaSkeleton::resetGeneralizedForces()
{
  applyToAllDofs<&DegreeOfFreedom::resetForce>(this);
  // Note: This function used to clear the internal forces of SoftBodyNodes as
  // well. Now you should use clearInternalForces for that
}

//==============================================================================
void MetaSkeleton::setForceLowerLimit(std::size_t _index, double _force)
{
  setValueFromIndex<&DegreeOfFreedom::setForceLowerLimit>(
        this, _index, _force, "setForceLowerLimit");
}

//==============================================================================
void MetaSkeleton::setForceLowerLimits(const Eigen::VectorXd& forces)
{
  setAllValuesFromVector<&DegreeOfFreedom::setForceLowerLimit>(
        this, forces, "setForceLowerLimits", "forces");
}

//==============================================================================
void MetaSkeleton::setForceLowerLimits(const std::vector<std::size_t>& indices,
                                       const Eigen::VectorXd& forces)
{
  setValuesFromVector<&DegreeOfFreedom::setForceLowerLimit>(
        this, indices, forces, "setForceLowerLimits", "forces");
}

//==============================================================================
double MetaSkeleton::getForceLowerLimit(std::size_t _index) const
{
  return getValueFromIndex<&DegreeOfFreedom::getForceLowerLimit>(
        this, _index, "getForceLowerLimit");
}

//==============================================================================
Eigen::VectorXd MetaSkeleton::getForceLowerLimits() const
{
  return getValuesFromAllDofs<&DegreeOfFreedom::getForceLowerLimit>(
        this, "getForceLowerLimits");
}

//==============================================================================
Eigen::VectorXd MetaSkeleton::getForceLowerLimits(
    const std::vector<std::size_t>& indices) const
{
  return getValuesFromVector<&DegreeOfFreedom::getForceLowerLimit>(
        this, indices, "getForceLowerLimits");
}

//==============================================================================
void MetaSkeleton::setForceUpperLimit(std::size_t _index, double _force)
{
  setValueFromIndex<&DegreeOfFreedom::setForceUpperLimit>(
        this, _index, _force, "setForceUpperLimit");
}

//==============================================================================
void MetaSkeleton::setForceUpperLimits(const Eigen::VectorXd& forces)
{
  setAllValuesFromVector<&DegreeOfFreedom::setForceUpperLimit>(
        this, forces, "setForceUpperLimits", "forces");
}

//==============================================================================
void MetaSkeleton::setForceUpperLimits(const std::vector<std::size_t>& indices,
                                       const Eigen::VectorXd& forces)
{
  setValuesFromVector<&DegreeOfFreedom::setForceUpperLimit>(
        this, indices, forces, "setForceUpperLimits", "forces");
}

//==============================================================================
double MetaSkeleton::getForceUpperLimit(std::size_t _index) const
{
  return getValueFromIndex<&DegreeOfFreedom::getForceUpperLimit>(
        this, _index, "getForceUpperLimit");
}

//==============================================================================
Eigen::VectorXd MetaSkeleton::getForceUpperLimits() const
{
  return getValuesFromAllDofs<&DegreeOfFreedom::getForceUpperLimit>(
        this, "getForceUpperLimits");
}

//==============================================================================
Eigen::VectorXd MetaSkeleton::getForceUpperLimits(
    const std::vector<std::size_t>& indices) const
{
  return getValuesFromVector<&DegreeOfFreedom::getForceUpperLimit>(
        this, indices, "getForceUpperLimits");
}

//==============================================================================
Eigen::VectorXd MetaSkeleton::getVelocityChanges() const
{
  return getValuesFromAllDofs<&DegreeOfFreedom::getVelocityChange>(
        this, "getVelocityChanges");
}

//==============================================================================
void MetaSkeleton::setJointConstraintImpulses(const Eigen::VectorXd& _impulses)
{
  setAllValuesFromVector<&DegreeOfFreedom::setConstraintImpulse>(
        this, _impulses, "setJointConstraintImpulses", "_impulses");
}

//==============================================================================
Eigen::VectorXd MetaSkeleton::getJointConstraintImpulses() const
{
  return getValuesFromAllDofs<&DegreeOfFreedom::getConstraintImpulse>(
        this, "getJointConstraintImpulses");
}

//==============================================================================
double MetaSkeleton::computeLagrangian() const
{
  return computeKineticEnergy() - computePotentialEnergy();
}

//==============================================================================
double MetaSkeleton::getKineticEnergy() const
{
  return computeKineticEnergy();
}

//==============================================================================
double MetaSkeleton::getPotentialEnergy() const
{
  return computePotentialEnergy();
}

//==============================================================================
MetaSkeleton::MetaSkeleton()
  : onNameChanged(mNameChangedSignal)
{
  // Do nothing
}

} // namespace dynamics
} // namespace dart



