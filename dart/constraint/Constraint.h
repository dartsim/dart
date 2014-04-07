/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>,
 *            Jeongseok Lee <jslee02@gmail.com>
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
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

#ifndef DART_CONSTRAINT_CONSTRAINT_H_TEST
#define DART_CONSTRAINT_CONSTRAINT_H_TEST

namespace dart {
namespace constraint {

//==============================================================================
// TODO(JS): Restricted to ODE LCP solver. Generalize this class for various
//           types of LCP solvers.
/// \brief LCPTerms class
class ODELcp
{
public:
  /// \brief Constructor
  explicit ODELcp(int _n);

  /// \brief Destructor
  ~ODELcp();

  //-------------------------- TEST CODE ---------------------------------------
  void print();

  void clear();

  bool checkSymmetric();

  bool checkSymmetric2(int _index);

  //----------------------------------------------------------------------------
  /// \brief
  double* A;

  /// \brief
  double* b;

  /// \brief
  double* x;

  /// \brief
  double* w;

  /// \brief Lower bound of x
  double* lb;

  /// \brief Upper bound of x
  double* ub;

  /// \brief Friction index
  int* frictionIndex;

  /// \brief Total dimension of contraints
  int dim;

  /// \brief
  int nSkip;
};

//==============================================================================
/// \brief ConstraintTEST class
class Constraint
{
public:
  enum ConstraintType
  {
    CT_STATIC,
    CT_DYNAMIC
  };

  /// \brief Default contructor
  explicit Constraint(ConstraintType _type);

  /// \brief Default destructor
  virtual ~Constraint();

  //----------------------------------------------------------------------------
  // Pure virtual functions for solving
  //----------------------------------------------------------------------------
  /// \brief
  virtual void preprocess() = 0;

  /// \brief
  virtual void update() = 0;

  /// \brief
  virtual void fillLcpOde(ODELcp* _lcp, int _idx) = 0;

  /// \brief Apply unit impulse to constraint space of _idx
  virtual void applyUnitImpulse(int _idx) = 0;

  /// \brief
  virtual void getVelocityChange(double* _delVel, int _idx) = 0;

  /// \brief
  virtual void excite() {}

  /// \brief
  virtual void unexcite() {}

  /// \brief Apply computed constraint impulse to constrained skeletons
  virtual void applyConstraintImpulse(double* _lambda, int _idx) = 0;

  //----------------------------------------------------------------------------
  /// \brief
  int getDimension() const;

protected:
//  /// \brief
//  void impulseTest();

  /// \brief
  int mDim;

  /// \brief
  ConstraintType mConstraintType;
};

} // namespace constraint
} // namespace dart

#endif  // DART_CONSTRAINT_CONSTRAINT_H_TEST

