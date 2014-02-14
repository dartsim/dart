/*
 * Copyright (c) 2013-2014, Georgia Tech Research Corporation
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

#ifndef SOFT_DYNAMICS_SOFTBODYNODE_H_
#define SOFT_DYNAMICS_SOFTBODYNODE_H_

#define SOFT_DEFAULT_VERTEX_STIFFNESS (1.0)
#define SOFT_DEFAULT_EDGE_STIFNESS    (1.0)
#define SOFT_DEFAULT_DAMPING_COEFF    (0.01)

#include <string>
#include <vector>

#include <Eigen/Dense>

#include <dart/dynamics/BodyNode.h>

namespace dart {
namespace dynamics {

class PointMass;
class SoftMeshShape;

/// \brief
///
/// We assume that only one soft flesh (mesh) is contained in soft body node
class SoftBodyNode : public BodyNode
{
public:
  friend class SoftSkeleton;

  //--------------------------------------------------------------------------
  // Constructor and Desctructor
  //--------------------------------------------------------------------------
  /// \brief
  explicit SoftBodyNode(const std::string& _name = "Unnamed SoftBodyNode");

  /// \brief
  virtual ~SoftBodyNode();


  /// \brief Get mass.
  double getMass() const;

  /// \brief
  void setVertexSpringStiffness(double _kv);

  /// \brief
  double getVertexSpringStiffness() const;

  /// \brief
  void setEdgeSpringStiffness(double _ke);

  /// \brief
  double getEdgeSpringStiffness() const;

  /// \brief
  void setDampingCoefficient(double _damp);

  /// \brief
  double getDampingCoefficient() const;

  /// \brief
  void removeAllPointMasses();

  /// \brief
  void addPointMass(PointMass* _pointMass);

  /// \brief
  int getNumPointMasses() const;

  /// \brief
  PointMass* getPointMass(int _idx) const;

  /// \brief
  void connectPointMasses(int _idx1, int _idx2);

  /// \brief
  void addFace(const Eigen::Vector3i& _face);

  /// \brief
  const Eigen::Vector3i& getFace(int _idx) const;

  /// \brief
  int getNumFaces();


protected:
  //--------------------------------------------------------------------------
  // Sub-functions for Recursive Kinematics Algorithms
  //--------------------------------------------------------------------------
  // Documentation inherited.
  virtual void init(Skeleton* _skeleton, int _skeletonIndex);

  // Documentation inherited.
  virtual void aggregateGenCoords(std::vector<GenCoord*>* _genCoords);

  // Documentation inherited.
  virtual void aggregatePointMassGenCoords(std::vector<GenCoord*>* _genCoords);

  // Documentation inherited.
  virtual void updateTransform();

  // Documentation inherited.
  virtual void updateTransform_Issue122(double _timeStep);

  // Documentation inherited.
  virtual void updateVelocity();

  // Documentation inherited.
  virtual void updateEta();

  // Documentation inherited.
  virtual void updateEta_Issue122();

  // Documentation inherited.
  virtual void updateAcceleration();

  // Documentation inherited.
  virtual void updateBodyForce(const Eigen::Vector3d& _gravity,
                               bool _withExternalForces = false);

  // Documentation inherited.
  virtual void updateGeneralizedForce(bool _withDampingForces = false);

  // Documentation inherited.
  virtual void updateArticulatedInertia(double _timeStep);

  // Documentation inherited.
  virtual void updateBiasForce(double _timeStep, const Eigen::Vector3d& _gravity);

  // Documentation inherited.
  virtual void update_ddq();

  // Documentation inherited.
  virtual void update_F_fs();

  // Documentation inherited.
  virtual void updateMassMatrix();

  // Documentation inherited.
  virtual void aggregateMassMatrix(Eigen::MatrixXd* _MCol, int _col);

  // Documentation inherited.
  virtual void aggregateAugMassMatrix(Eigen::MatrixXd* _MCol, int _col,
                                      double _timeStep);

  // Documentation inherited.
  virtual void updateInvMassMatrix();

  // Documentation inherited.
  virtual void updateInvAugMassMatrix();

  // Documentation inherited.
  virtual void aggregateInvMassMatrix(Eigen::MatrixXd* _InvMCol, int _col);

  // Documentation inherited.
  virtual void aggregateInvAugMassMatrix(Eigen::MatrixXd* _InvMCol, int _col,
                                         double _timeStep);

  // Documentation inherited.
  // TODO(JS): Not implemented yet.
  virtual void aggregateCoriolisForceVector(Eigen::VectorXd* _C);

  // Documentation inherited.
  virtual void aggregateGravityForceVector(Eigen::VectorXd* _g,
                                           const Eigen::Vector3d& _gravity);

  // Documentation inherited.
  virtual void updateCombinedVector();

  // Documentation inherited.
  virtual void aggregateCombinedVector(Eigen::VectorXd* _Cg,
                                       const Eigen::Vector3d& _gravity);

  // Documentation inherited.
  virtual void aggregateExternalForces(Eigen::VectorXd* _Fext);

  // Documentation inherited.
  virtual void clearExternalForces();

  //--------------------------------------------------------------------------
  // Rendering
  //--------------------------------------------------------------------------
  /// \brief Render the entire subtree rooted at this body node.
  virtual void draw(renderer::RenderInterface* _ri = NULL,
                    const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(),
                    bool _useDefaultColor = true, int _depth = 0) const;

protected:
  /// \brief List of point masses composing deformable mesh.
  std::vector<PointMass*> mPointMasses;

  // TODO(JS): Let's remove this because this is rendering part
  /// \brief Tri-mesh indexes for rendering.
  std::vector<Eigen::Vector3i> mFaces;

  /// \brief Spring stiffness for vertex deformation restoring spring force of
  ///        the point masses.
  double mKv;

  /// \brief Spring stiffness for edge deformation restoring spring force of the
  ///        point masses.
  double mKe;

  /// \brief
  double mDampCoeff;

  /// \brief Soft mesh shape for visualization.
  SoftMeshShape* mSoftVisualShape;

  /// \brief Soft mesh shape for collision.
  SoftMeshShape* mSoftCollShape;

private:
  /// \brief
  void _addPiToArticulatedInertia(const Eigen::Vector3d& _p, double _Pi);

  /// \brief
  void _addImplicitPiToImplicitAI(const Eigen::Vector3d& _p,
                                          double _ImplicitPi);
};

class SoftBodyNodeHelper
{
public:
  /// \brief
  /// This should be called before SoftBodyNode::init() is called
  static void setSingle(
      SoftBodyNode*          _softBodyNode,
      double                 _totalMass,
      double                 _vertexStiffness = SOFT_DEFAULT_VERTEX_STIFFNESS,
      double                 _edgeStiffness   = SOFT_DEFAULT_EDGE_STIFNESS,
      double                 _dampingCoeff    = SOFT_DEFAULT_DAMPING_COEFF);

  /// \brief
  /// This should be called before SoftBodyNode::init() is called
  static void setBox(
      SoftBodyNode*            _softBodyNode,
      const Eigen::Vector3d&   _size,
      const Eigen::Isometry3d& _localTransfom,
      double                   _totalMass,
      double                   _vertexStiffness = SOFT_DEFAULT_VERTEX_STIFFNESS,
      double                   _edgeStiffness   = SOFT_DEFAULT_EDGE_STIFNESS,
      double                   _dampingCoeff    = SOFT_DEFAULT_DAMPING_COEFF);

  /// \brief
  /// This should be called before SoftBodyNode::init() is called
  static void setBox(
      SoftBodyNode*            _softBodyNode,
      const Eigen::Vector3d&   _size,
      const Eigen::Isometry3d& _localTransfom,
      const Eigen::Vector3i&   _frags,
      double                   _totalMass,
      double                   _vertexStiffness = SOFT_DEFAULT_VERTEX_STIFFNESS,
      double                   _edgeStiffness   = SOFT_DEFAULT_EDGE_STIFNESS,
      double                   _dampingCoeff    = SOFT_DEFAULT_DAMPING_COEFF);

  /// \brief
  /// This should be called before SoftBodyNode::init() is called
  static void setSinglePointMass(
      SoftBodyNode*          _softBodyNode,
      double                 _totalMass,
      double                 _vertexStiffness = SOFT_DEFAULT_VERTEX_STIFFNESS,
      double                 _edgeStiffness   = SOFT_DEFAULT_EDGE_STIFNESS,
      double                 _dampingCoeff    = SOFT_DEFAULT_DAMPING_COEFF);

  /// \brief
  /// This should be called before SoftBodyNode::init() is called
  static void setEllipsoid(
      SoftBodyNode*          _softBodyNode,
      const Eigen::Vector3d& _size,
      int                    _nSlices,
      int                    _nStacks,
      double                 _totalMass,
      double                 _vertexStiffness = SOFT_DEFAULT_VERTEX_STIFFNESS,
      double                 _edgeStiffness   = SOFT_DEFAULT_EDGE_STIFNESS,
      double                 _dampingCoeff    = SOFT_DEFAULT_DAMPING_COEFF);
};

}  // namespace dynamics
}  // namespace dart

#endif  // SOFT_DYNAMICS_SOFTBODYNODE_H_
