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

#include "dart/dynamics/LineSegmentShape.hpp"
#include "dart/common/Console.hpp"
#include "dart/math/Geometry.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
LineSegmentShape::LineSegmentShape(float _thickness)
  : Shape(LINE_SEGMENT),
    mThickness(_thickness),
    mDummyVertex(Eigen::Vector3d::Zero())
{
  if (_thickness <= 0.0f)
  {
    dtwarn << "[LineSegmentShape::LineSegmentShape] Attempting to set "
           << "non-positive thickness. We set the thickness to 1.0f instead."
           << std::endl;
    mThickness = 1.0f;
  }

  updateVolume();
  mVariance = DYNAMIC_VERTICES;
}

//==============================================================================
LineSegmentShape::LineSegmentShape(const Eigen::Vector3d& _v1,
                                   const Eigen::Vector3d& _v2,
                                   float _thickness)
  : Shape(),
    mThickness(_thickness)
{
  if (_thickness <= 0.0f)
  {
    dtwarn << "[LineSegmentShape::LineSegmentShape] Attempting to set "
           << "non-positive thickness. We set the thickness to 1.0f instead."
           << std::endl;
    mThickness = 1.0f;
  }

  addVertex(_v1);
  addVertex(_v2);
  updateVolume();
  mVariance = DYNAMIC_VERTICES;
}

//==============================================================================
const std::string& LineSegmentShape::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& LineSegmentShape::getStaticType()
{
  static const std::string type("LineSegmentShape");
  return type;
}

//==============================================================================
void LineSegmentShape::setThickness(float _thickness)
{
  if (_thickness <= 0.0f)
  {
    dtwarn << "[LineSegmentShape::setThickness] Attempting to set non-positive "
           << "thickness. We set the thickness to 1.0f instead." << std::endl;
    mThickness = 1.0f;
    return;
  }

  mThickness = _thickness;
}

//==============================================================================
float LineSegmentShape::getThickness() const
{
  return mThickness;
}

//==============================================================================
std::size_t LineSegmentShape::addVertex(const Eigen::Vector3d& _v)
{
  std::size_t parent = mVertices.size();
  if(parent > 0)
    return addVertex(_v, parent-1);

  mVertices.push_back(_v);
  return 0;
}

//==============================================================================
std::size_t LineSegmentShape::addVertex(const Eigen::Vector3d& _v, std::size_t _parent)
{
  std::size_t index = mVertices.size();
  mVertices.push_back(_v);

  if(_parent > mVertices.size())
  {
    if(mVertices.size() == 0)
      dtwarn << "[LineSegmentShape::addVertex(const Eigen::Vector3d&, std::size_t)] "
             << "Attempting to add a vertex to be a child of vertex #"
             << _parent << ", but no vertices exist yet. No connection will be "
             << "created for the new vertex yet.\n";
    else
      dtwarn << "[LineSegmentShape::addVertex(const Eigen::Vector3d&, std::size_t)] "
             << "Attempting to add a vertex to be a child of vertex #"
             << _parent << ", but the vertex indices only go up to "
             << mVertices.size()-1 << ". No connection will be created for the "
             << "new vertex yet.\n";
  }
  else
  {
    mConnections.push_back(Eigen::Vector2i(_parent, index));
  }

  return index;
}

//==============================================================================
void LineSegmentShape::removeVertex(std::size_t _idx)
{
  if(_idx >= mVertices.size())
  {
    if(mVertices.size() == 0)
      dtwarn << "[LineSegmentShape::removeVertex] Attempting to remove vertex #"
             << _idx << ", but "
             << "this LineSegmentShape contains no vertices. "
             << "No vertex will be removed.\n";
    else
      dtwarn << "[LineSegmentShape::removeVertex] Attempting to remove vertex #"
             << _idx << ", but "
             << "vertex indices only go up to #" << mVertices.size()-1 << ". "
             << "No vertex will be removed.\n";

    return;
  }

  mVertices.erase(mVertices.begin()+_idx);
}

//==============================================================================
void LineSegmentShape::setVertex(std::size_t _idx, const Eigen::Vector3d& _v)
{
  if(_idx >= mVertices.size())
  {
    if(mVertices.size() == 0)
      dtwarn << "[LineSegmentShape::setVertex] Attempting to set vertex #" << _idx
             << ", but "
             << "no vertices exist in this LineSegmentShape yet.\n";
    else
      dtwarn << "[LineSegmentShape::setVertex] Attempting to set vertex #" << _idx
             << ", but "
             << "the vertices of this LineSegmentShape only go up to #"
             << mVertices.size()-1 << ".\n";

    return;
  }
  mVertices[_idx] = _v;
}

//==============================================================================
const Eigen::Vector3d& LineSegmentShape::getVertex(std::size_t _idx) const
{
  if(_idx < mVertices.size())
    return mVertices[_idx];

  if(mVertices.size()==0)
    dtwarn << "[LineSegmentShape::getVertex] Requested vertex #" << _idx
           << ", but no vertices currently exist in this LineSegmentShape\n";
  else
    dtwarn << "[LineSegmentShape::getVertex] Requested vertex #" << _idx
           << ", but vertex indices currently only go up to "
           << mVertices.size()-1 << "\n";

  return mDummyVertex;
}

//==============================================================================
const std::vector<Eigen::Vector3d>& LineSegmentShape::getVertices() const
{
  return mVertices;
}

//==============================================================================
void LineSegmentShape::addConnection(std::size_t _idx1, std::size_t _idx2)
{
  if(_idx1 >= mVertices.size() || _idx2 >= mVertices.size())
  {
    if(mVertices.size() == 0)
      dtwarn << "[LineSegmentShape::createConnection] Attempted to create a "
             << "connection between vertex #" << _idx1 << " and vertex #" << _idx2
             << ", but no vertices exist for this LineSegmentShape yet. "
             << "No connection will be made for these non-existent vertices.\n";
    else
      dtwarn << "[LineSegmentShape::createConnection] Attempted to create a "
             << "connection between vertex #" << _idx1 << " and vertex #" << _idx2
             << ", but the vertices only go up to #" << mVertices.size() << ". "
             << "No connection will be made for these non-existent vertices.\n";

    return;
  }

  mConnections.push_back(Eigen::Vector2i(_idx1, _idx2));
}

//==============================================================================
void LineSegmentShape::removeConnection(std::size_t _vertexIdx1, std::size_t _vertexIdx2)
{
  // Search through all connections to remove any that match the request
  Eigen::aligned_vector<Eigen::Vector2i>::iterator it = mConnections.begin();
  while(it != mConnections.end())
  {
    const Eigen::Vector2i c = (*it);
    if(    (c[0] == (int)_vertexIdx1 && c[1] == (int)_vertexIdx2)
        || (c[0] == (int)_vertexIdx2 && c[1] == (int)_vertexIdx1) )
    {
      // Erase this iterator, but not before stepping it forward to the next
      // iterator in the sequence.
      mConnections.erase(it++);
    }
    else
      ++it;
  }
}

//==============================================================================
void LineSegmentShape::removeConnection(std::size_t _connectionIdx)
{
  if(_connectionIdx >= mConnections.size())
  {
    if(mConnections.size() == 0)
      dtwarn << "[LineSegmentShape::removeConnection(std::size_t)] Attempting to "
             << "remove connection #" << _connectionIdx << ", but "
             << "no connections exist yet. "
             << "No connection will be removed.\n";
    else
      dtwarn << "[LineSegmentShape::removeConnection(std::size_t)] Attempting to "
             << "remove connection #" << _connectionIdx << ", but "
             << "connection indices only go up to #" << mConnections.size()-1
             << ". " << "No connection will be removed.\n";

    return;
  }

  mConnections.erase(mConnections.begin()+_connectionIdx);
}

//==============================================================================
const Eigen::aligned_vector<Eigen::Vector2i>&
LineSegmentShape::getConnections() const
{
  return mConnections;
}

//==============================================================================
Eigen::Matrix3d LineSegmentShape::computeInertia(double _mass) const
{
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();

  double totalLength = 0;
  for(const Eigen::Vector2i& c : mConnections)
  {
    const Eigen::Vector3d& v0 = mVertices[c[0]];
    const Eigen::Vector3d& v1 = mVertices[c[1]];

    totalLength += (v1-v0).norm();
  }

  for(const Eigen::Vector2i& c : mConnections)
  {
    const Eigen::Vector3d& v0 = mVertices[c[0]];
    const Eigen::Vector3d& v1 = mVertices[c[1]];

    double radius = 1e-6;
    double height = (v1-v0).norm();

    double partialMass = _mass * height/totalLength;

    Eigen::Matrix3d partialInertia = Eigen::Matrix3d::Zero();
    partialInertia(0, 0) = partialMass * (3.0 * radius*radius + height*height) / 12.0;
    partialInertia(1, 1) = partialInertia(0, 0);
    partialInertia(2, 2) = 0.5 * _mass * radius * radius;

    Eigen::Vector3d v = v1-v0;
    Eigen::Vector3d axis;
    double angle;
    if(v.norm() == 0)
    {
      angle = 0;
      axis = Eigen::Vector3d::UnitX();
    }
    else
    {
      v.normalize();
      Eigen::Vector3d axis = Eigen::Vector3d::UnitZ().cross(v);
      if(axis.norm() == 0)
      {
        angle = 0;
        axis = Eigen::Vector3d::UnitX();
      }
      else
      {
        axis.normalize();
        angle = acos(Eigen::Vector3d::UnitZ().dot(v));
      }
    }

    Eigen::AngleAxisd R(angle, axis);
    Eigen::Vector3d center = (v1+v0)/2.0;
    inertia += R.matrix()
        * math::parallelAxisTheorem(partialInertia, center, partialMass)
        * R.matrix().transpose();
  }

  return inertia;
}

//==============================================================================
void LineSegmentShape::updateVolume()
{
  mVolume = 0.0;
}

} // namespace dynamics
} // namespace dart
