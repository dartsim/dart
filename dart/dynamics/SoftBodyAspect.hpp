/*
 * Copyright (c) 2016, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#ifndef DART_DYNAMICS_SOFTBODYASPECT_HPP_
#define DART_DYNAMICS_SOFTBODYASPECT_HPP_

#include <Eigen/Dense>
#include "dart/common/AspectWithVersion.hpp"
#include "dart/dynamics/SmartPointer.hpp"
#include "dart/dynamics/detail/SoftBodyAspect.hpp"
#include "dart/dynamics/PointMass.hpp"

namespace dart {
namespace dynamics {

class BodyNode;
class PointMass;
class PointMassNotifier;

class SoftBodyAspect :
    public common::AspectWithStateAndVersionedProperties<
        SoftBodyAspect,
        detail::SoftBodyAspectState,
        detail::SoftBodyAspectProperties,
        BodyNode>
{
public:

  friend class BodyNode;
  friend class PointMass;
  friend class PointMassNotifier;
  friend class Skeleton;

  using Base = common::AspectWithStateAndVersionedProperties<
      SoftBodyAspect,
      detail::SoftBodyAspectState,
      detail::SoftBodyAspectProperties,
      BodyNode>;

  /// Constructor
  SoftBodyAspect(common::Composite* comp,
                 const StateData& state = StateData(),
                 const PropertiesData& properties = PropertiesData());

  /// Constructor
  SoftBodyAspect(common::Composite* comp,
                 const PropertiesData& properties,
                 const StateData state = StateData());

  /// Copy this Aspect is not safe
  SoftBodyAspect(const SoftBodyAspect&) = delete;


  /// Get the update notifier for the PointMasses of this SoftBodyNode
  PointMassNotifier* getNotifier();

  /// Get the update notifier for the PointMasses of this SoftBodyNode
  const PointMassNotifier* getNotifier() const;

  /// \brief Get mass.
  double getMass() const;

  /// \brief
  void setVertexSpringStiffness(double kv);

  /// \brief
  double getVertexSpringStiffness() const;

  /// \brief
  void setEdgeSpringStiffness(double ke);

  /// \brief
  double getEdgeSpringStiffness() const;

  /// \brief
  void setDampingCoefficient(double damp);

  /// \brief
  double getDampingCoefficient() const;

  /// \brief
  void removeAllPointMasses();

  /// \brief
  PointMass* addPointMass(const PointMass::Properties& properties);

  /// \brief
  std::size_t getNumPointMasses() const;

  /// \brief
  PointMass* getPointMass(std::size_t index);

  /// \brief
  const PointMass* getPointMass(std::size_t index) const;

  /// Return all the point masses in this SoftBodyNode
  const std::vector<PointMass*>& getPointMasses() const;

  /// \brief
  void connectPointMasses(std::size_t idx1, std::size_t idx2);

  /// \brief
  void addFace(const Eigen::Vector3i& face);

  /// \brief
  const Eigen::Vector3i& getFace(std::size_t idx) const;

  /// \brief
  std::size_t getNumFaces() const;

  // Documentation inherited.
  void clearConstraintImpulse();

protected:

  // Documentation inherited
  void setComposite(common::Composite* newComposite) override;

  /// Initialize the vector members with proper sizes.
  void init();

  /// Used by SoftBodyAspect to have this SoftBodyNode reconstruct its
  /// SoftMeshShape
  void configurePointMasses(ShapeNode* softNode);

  //----------------------------------------------------------------------------
  /// \{ \name Recursive dynamics routines
  //----------------------------------------------------------------------------

  /// Update articulated inertia if necessary
  void checkArticulatedInertiaUpdate() const;

  /// \}

  /// \brief List of point masses composing deformable mesh.
  std::vector<PointMass*> mPointMasses;

  /// An Entity which tracks when the point masses need to be updated
  std::unique_ptr<PointMassNotifier> mNotifier;

  /// \brief Soft mesh shape belonging to this node.
  WeakShapeNodePtr mSoftShapeNode;

  /// Generalized inertia with point masses
  math::Inertia mI2;

  ///
  math::Inertia mArtInertia2;

  ///
  math::Inertia mArtInertiaImplicit2;

private:

  void addPiToArtInertia(const Eigen::Vector3d& p, double Pi) const;

  void addPiToArtInertiaImplicit(
      const Eigen::Vector3d& p, double ImplicitPi) const;

};

class SoftBodyNodeHelper
{
public:

  /// Create a Properties struct for a box-shaped BodyNode with 8
  /// PointMasses
  static SoftBodyAspect::PropertiesData makeBoxProperties(
      const Eigen::Vector3d&   _size,
      const Eigen::Isometry3d& _localTransform,
      double                   _totalMass,
      double                   _vertexStiffness = DART_DEFAULT_VERTEX_STIFFNESS,
      double                   _edgeStiffness   = DART_DEFAULT_EDGE_STIFNESS,
      double                   _dampingCoeff    = DART_DEFAULT_DAMPING_COEFF);

  /// \brief
  /// This should be called before BodyNode::init() is called
  static void setBox(BodyNode* bodyNode,
      const Eigen::Vector3d&   _size,
      const Eigen::Isometry3d& _localTransform,
      double                   _totalMass,
      double                   _vertexStiffness = DART_DEFAULT_VERTEX_STIFFNESS,
      double                   _edgeStiffness   = DART_DEFAULT_EDGE_STIFNESS,
      double                   _dampingCoeff    = DART_DEFAULT_DAMPING_COEFF);

  /// Create a Properties struct for a box-shaped BodyNode. Specify the
  /// number of vertices along each axis with _frags. Each component should be
  /// equal to or greater than 3. For example, [3 3 3] is allowed but [2 2 2] is
  /// not.
  static SoftBodyAspect::PropertiesData makeBoxProperties(
      const Eigen::Vector3d&   _size,
      const Eigen::Isometry3d& _localTransform,
      const Eigen::Vector3i&   _frags,
      double                   _totalMass,
      double                   _vertexStiffness = DART_DEFAULT_VERTEX_STIFFNESS,
      double                   _edgeStiffness   = DART_DEFAULT_EDGE_STIFNESS,
      double                   _dampingCoeff    = DART_DEFAULT_DAMPING_COEFF);

  /// \brief
  /// This should be called before BodyNode::init() is called
  /// \param[in] _frags Number of vertices of box mesh. Each component should be
  ///   equal or greater than 3. For example, [3 3 3] is allowed but [2 2 2] is
  ///   not.
  // TODO: The component of _frags should allow 2.
  static void setBox(BodyNode* bodyNode,
      const Eigen::Vector3d&   _size,
      const Eigen::Isometry3d& _localTransform,
      const Eigen::Vector3i&   _frags,
      double                   _totalMass,
      double                   _vertexStiffness = DART_DEFAULT_VERTEX_STIFFNESS,
      double                   _edgeStiffness   = DART_DEFAULT_EDGE_STIFNESS,
      double                   _dampingCoeff    = DART_DEFAULT_DAMPING_COEFF);

  /// Create a Properties struct for a BodyNode with a single PointMass
  static SoftBodyAspect::PropertiesData makeSinglePointMassProperties(
      double _totalMass,
      double _vertexStiffness = DART_DEFAULT_VERTEX_STIFFNESS,
      double _edgeStiffness   = DART_DEFAULT_EDGE_STIFNESS,
      double _dampingCoeff    = DART_DEFAULT_DAMPING_COEFF);

  /// \brief
  /// This should be called before BodyNode::init() is called
  static void setSinglePointMass(
      BodyNode*          bodyNode,
      double                 _totalMass,
      double                 _vertexStiffness = DART_DEFAULT_VERTEX_STIFFNESS,
      double                 _edgeStiffness   = DART_DEFAULT_EDGE_STIFNESS,
      double                 _dampingCoeff    = DART_DEFAULT_DAMPING_COEFF);

  /// Create a Properties struct for an Ellipsoid-shaped BodyNode
  static SoftBodyAspect::PropertiesData makeEllipsoidProperties(
      const Eigen::Vector3d& _size,
      std::size_t            _nSlices,
      std::size_t            _nStacks,
      double                 _totalMass,
      double                 _vertexStiffness = DART_DEFAULT_VERTEX_STIFFNESS,
      double                 _edgeStiffness   = DART_DEFAULT_EDGE_STIFNESS,
      double                 _dampingCoeff    = DART_DEFAULT_DAMPING_COEFF);

  /// \brief
  /// This should be called before BodyNode::init() is called
  static void setEllipsoid(
      BodyNode* bodyNode,
      const Eigen::Vector3d& _size,
      std::size_t _nSlices,
      std::size_t _nStacks,
      double                 _totalMass,
      double                 _vertexStiffness = DART_DEFAULT_VERTEX_STIFFNESS,
      double                 _edgeStiffness   = DART_DEFAULT_EDGE_STIFNESS,
      double                 _dampingCoeff    = DART_DEFAULT_DAMPING_COEFF);

  /// Create a Properties struct for a cylinder-shaped BodyNode
  static SoftBodyAspect::PropertiesData makeCylinderProperties(
      double _radius,
      double _height,
      std::size_t _nSlices,
      std::size_t _nStacks,
      std::size_t _nRings,
      double _totalMass,
      double _vertexStiffness = DART_DEFAULT_VERTEX_STIFFNESS,
      double _edgeStiffness   = DART_DEFAULT_EDGE_STIFNESS,
      double _dampingCoeff    = DART_DEFAULT_DAMPING_COEFF);

  ///
  /// This should be called before BodyNode::init() is called
  static void setCylinder(
      BodyNode* bodyNode,
      double _radius,
      double _height,
      std::size_t _nSlices,
      std::size_t _nStacks,
      std::size_t _nRings,
      double                 _totalMass,
      double                 _vertexStiffness = DART_DEFAULT_VERTEX_STIFFNESS,
      double                 _edgeStiffness   = DART_DEFAULT_EDGE_STIFNESS,
      double                 _dampingCoeff    = DART_DEFAULT_DAMPING_COEFF);
};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_SOFTBODYASPECT_HPP_
