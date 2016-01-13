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

#ifndef KIDO_DYNAMICS_LINESEGMENTSHAPE_H_
#define KIDO_DYNAMICS_LINESEGMENTSHAPE_H_

#include "kido/dynamics/Shape.hpp"

namespace kido {
namespace dynamics {

/// The LineSegmentShape facilitates creating graphs of line segments. The graph
/// can consist of a single line segment or many interconnected line segments.
/// Note: LineSegmentShape may NOT be used as a collision shape for BodyNodes,
/// but it may be used for visualization purposes.
class LineSegmentShape : public Shape
{
public:
  /// Default constructor
  LineSegmentShape(float _thickness = 1.0f);

  /// Constructor for creating a simple line segment that connects two vertices
  LineSegmentShape(const Eigen::Vector3d& _v1, const Eigen::Vector3d& _v2,
                   float _thickness = 1.0f);

  /// Set the line thickness/width for rendering
  void setThickness(float _thickness);

  /// Get the line thickness/width used for rendering
  float getThickness() const;

  /// Add a vertex as a child to the last vertex that was added
  size_t addVertex(const Eigen::Vector3d& _v);

  /// Add a vertex as a child to the specified vertex
  size_t addVertex(const Eigen::Vector3d& _v, size_t _parent);

  /// Remove a vertex from the list of vertices. IMPORTANT: Note that this
  /// alters the indices of all vertices that follow it in the list, which also
  /// clobbers the validity of the list of connections for all those vertices.
  /// A safer and more efficient method might be to recycle vertices by moving
  /// them around with setVertex() and/or altering their connections.
  void removeVertex(size_t _idx);

  /// Change the location of the specified vertex
  void setVertex(size_t _idx, const Eigen::Vector3d& _v);

  /// Get the location of the specified vertex. Returns a zero vector if an
  /// out-of-bounds vertex is requested.
  const Eigen::Vector3d& getVertex(size_t _idx) const;

  /// Get all the vertices
  const std::vector<Eigen::Vector3d>& getVertices() const;

  /// Create a connection between the two specified vertices
  void addConnection(size_t _idx1, size_t _idx2);

  /// Search for a connection between two vertices and break it if it exists.
  /// This is less efficient but more robust than removeConnection(size_t).
  void removeConnection(size_t _vertexIdx1, size_t _vertexIdx2);

  /// Remove the specified connection entry. Note that this will impact the
  /// indices of all connections that come after _connectionIdx. This is more
  /// efficient but less robust than removeConnection(size_t,size_t)
  void removeConnection(size_t _connectionIdx);

  /// Get all the connections
  const Eigen::aligned_vector<Eigen::Vector2i>& getConnections() const;

  // Documentation inherited
  virtual void draw(renderer::RenderInterface* _ri,
                    const Eigen::Vector4d& _color,
                    bool _useDefaultColor) const override;

  /// The returned inertia matrix will be like a very thin cylinder. The _mass
  /// will be evenly distributed across all lines.
  virtual Eigen::Matrix3d computeInertia(double mass) const override;

  // TODO(MXG): Consider supporting colors-per-vertex

protected:

  // Documentation inherited
  void updateVolume() override;

  /// Line thickness for rendering
  float mThickness;

  /// Vector of vertices
  std::vector<Eigen::Vector3d> mVertices;

  /// Vector of connections
  Eigen::aligned_vector<Eigen::Vector2i> mConnections;

  /// A dummy vertex that can be returned when an out-of-bounds vertex is
  /// requested
  const Eigen::Vector3d mDummyVertex;
};

} // namespace dynamics
} // namespace kido

#endif // KIDO_DYNAMICS_LINESEGMENTSHAPE_H_
