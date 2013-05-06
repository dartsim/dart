/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 * Date: 05/06/2013
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

#ifndef DYNAMICS_LIE_GROUPS_H
#define DYNAMICS_LIE_GROUPS_H

#include <Eigen/Dense>
#include "math/UtilsMath.h"

namespace math
{

class Inertia; // Inertia with cog offset (6x6 matrix)
class so3;
class SO3; // Special orthogonal group (3x3 rotation matrix)
class se3;
class SE3; // Special Euclidean group (4x4 transformation matrix)
class dse3;

/// @brief Inertia class
///
/// Generalized inertia, G = | Inertia  0 |
///                          | 0       mI |
/// Generalized inertia with cog offset r, G = | Inertia - m[r][r]   m[r] |
///                                            |             -m[r]     mI |
/// where Inertia is momentum of inertia, m is mass, I is 3x3 identity matrix,
/// and [r] is skew-symmetrix matrix of cog offset.
class Inertia
{
public:
    //
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// @brief
    Inertia();

    /// @brief
    virtual ~Inertia();

    /// @brief
    void setMass(double _mass) { mMass = _mass; }

    /// @brief
    double getMass(void) const { return mMass; }

    /// @brief
    dse3 operator*(const se3& _V);

    /// @brief
    void setMomentsOfInertia(const Eigen::Matrix3d& _moi);

    /// @brief
    Eigen::Matrix3d getMomentsOfInertia() const;

    /// @brief
    void setCenterOfMass(const Eigen::Vector3d& _com);

    /// @brief
    const Eigen::Vector3d& getCenterOfMass(void) { return mCenterOfMass; }

    /// @brief
    ///
    /// \f$ Ad_{T12^{-1}}^{*} G2 Ad_{T12^{-1}} \f$.
    Inertia getdAdInertiaAd(const SE3& _T12);

    /// @brief
    ///
    /// \f$ Ad_{T21}^{*} G2 Ad_{T21} \f$.
    Inertia getInvdAdInertiaAd(const SE3& _T21);

protected:
    /// @brief Mass the object. Default is 1.0.
    double mMass;

    /// @brief Principal moments of inertia. Default is (1.0 1.0 1.0)
    /// These Moments of Inertia are specified in the local Inertial frame.
    Eigen::Vector3d mPrincipals;

    /// @brief Product moments of inertia. Default is (0.0 0.0 0.0)
    /// These MOI off-diagonals are specified in the local Inertial frame.
    /// Where products.x is Ixy, products.y is Ixz and products.z is Iyz.
    Eigen::Vector3d mProducts;

    /// \brief Center of gravity in the Link frame.
    ///        Default is (0.0 0.0 0.0  0.0 0.0 0.0)
    Eigen::Vector3d mCenterOfMass;
private:
};


//==============================================================================
/// @brief
class so3
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// @brief
    so3();

    explicit so3(const Eigen::Vector3d& _v);

    /// @brief
    virtual ~so3();

    /// @brief
    const Eigen::Vector3d& getVector() const { return mAngular; }

protected:
    /// @brief
    Eigen::Vector3d mAngular;

private:
};

//==============================================================================
/// @brief Special orthogonal group in 3d space.
class SO3
{
public:
    //
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// @brief
    SO3();

    /// @brief
    SO3(const Eigen::Matrix3d& _rotation);

    /// @brief
    virtual ~SO3();

public:
    /// @brief
    Eigen::Vector3d operator * (const Eigen::Vector3d& _q) const;

public:
    /// @brief
    void setExp(const so3& _S);

    /// @brief
    void setExp(const so3& _S, double theta);

    /// @brief
    const SO3 getInverse() const { return SO3(mR.transpose()); }

    /// @brief
    const Eigen::Matrix3d& getMatrix() { return mR; }

protected:
    /// @brief
    Eigen::Matrix3d mR;

private:
};

//==============================================================================
/// @brief Special Euclidean group in 3d space.
///
/// SE3 is a class for representing the special Euclidean group.
/// Geometrically, it deals with rigid transformations on \f$ \mathbb{R}^3 \f$.
/// SE(3) is defined as the set of
/// mappings \f$g: \mathbb{R}^3 \rightarrow \mathbb{R}^3\f$ of the form \f$g(x) = Rx + p\f$,
/// where \f$R\in\f$ the special orthogonal group and \f$p\in \mathbb{R}^3\f$.
/// An element of SE(3), written as (R, p), can also be represented in
/// the matrix form	\f$\begin{bmatrix} R & p \\ 0 & 1\end{bmatrix}.\f$
class SE3
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// @brief
    SE3();

    /// @brief
    SE3(const SO3& _rotation, const Eigen::Vector3d& _position);

    /// @brief
    virtual ~SE3();

    /// @brief
    void setRotation(const SO3& _rotation) { mRotation = _rotation; }

    /// @brief
    const SO3& getRotation() const { return mRotation; }

    /// @brief
    void setPosition(const Eigen::Vector3d& _position) { mPosition = _position; }

    /// @brief
    const Eigen::Vector3d& getPosition() const { return mPosition; }

    /// @brief
    Eigen::Matrix4d getMatrix();
protected:
    /// @brief
    SO3 mRotation;

    /// @brief
    Eigen::Vector3d mPosition;
private:
};

//==============================================================================

/// @brief se3 is a class for representing \f$se(3)\f$, the Lie algebra of
/// \f$SE(3)\f$.
///
/// Geometrically se3 deals with generalized velocity which consist of linear
/// velocity and angular velocity.
class se3
{
public: // constructors and destructor
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/// @brief Default constructor.
	se3();

	/// @brief Copy constructor.
	se3(const se3& v);

	/// @brief
	se3(const dart_math::Vector6d& _v) : mLinear(_v.segment(3,3)), mAngular(_v.segment(0,3)) {}

    /// @brief
    se3(const Eigen::Vector3d& _linear, const Eigen::Vector3d& _angular);

    /// @brief Default destructor.
    virtual ~se3();

public: // operators
	/// @brief Substitution operator.
	const se3& operator = (const se3 &);

	/// @brief Casting operator.
	se3* operator&() { return this; }

	/// @brief Const Casting operator.
	const se3* operator&() const { return this; }

public:
    /// @brief
    void setLinear(const Eigen::Vector3d& _linear) { mLinear = _linear; }

    /// @brief
    const Eigen::Vector3d& getLinear() const { return mLinear; }

    /// @brief
    void setAngular(const Eigen::Vector3d& _angular) { mAngular = _angular; }

    /// @brief
    const Eigen::Vector3d& getAngular() const { return mAngular; }

    /// @brief
    void setAd(const SE3& _T12, const se3& _V2);

    /// @brief
    void setInvAd(const SE3& _T21, const se3& _V2);

    /// @brief
    void setad(const se3& _V1, const se3& _V2);

protected:
    /// @brief
    Eigen::Vector3d mLinear;

    /// @brief
    Eigen::Vector3d mAngular;
private:
};

class dse3
{
public:
    /// @brief
    void setLinear(const Eigen::Vector3d& _linear) { mLinear = _linear; }

    /// @brief
    const Eigen::Vector3d& getLinear() const { return mLinear; }

    /// @brief
    void setAngular(const Eigen::Vector3d& _angular) { mAngular = _angular; }

    /// @brief
    const Eigen::Vector3d& getAngular() const { return mAngular; }

protected:
    /// @brief
    Eigen::Vector3d mLinear;

    /// @brief
    Eigen::Vector3d mAngular;

private:

};

class Jacobian
{
public:
    /// @brief
    Jacobian();

    /// @brief
    virtual ~Jacobian();

    /// @brief
    void setSize(int _size) { mJ = Eigen::MatrixXd::Zero(6, _size); }

    /// @brief
    int getSize() const { return mJ.cols(); }

    /// @brief
    void setMatrix(const Eigen::MatrixXd& _J);

    /// @brief
    const Eigen::MatrixXd& getMatrix() const { return mJ; }

    /// @brief
    void setColumn(int _idx, const dart_math::Vector6d& _J) { mJ.col(_idx) = _J; }

    /// @brief
    dart_math::Vector6d getColumn(int _idx) const { return mJ.col(_idx); }

    /// @brief
    void setLinear(int _idx, Eigen::Vector3d& _Jv) { mJ.col(_idx).segment(3,3) = _Jv; }

    /// @brief
    Eigen::Vector3d getLinear(int _idx) const { return mJ.col(_idx).segment(3,3); }

    /// @brief
    void setAngular(int _idx, Eigen::Vector3d& _Jw) { mJ.col(_idx).segment(0,3) = _Jw; }

    /// @brief
    Eigen::Vector3d getAngular(int _idx) const { return mJ.col(_idx).segment(0,3); }


    se3 operator* (const Eigen::VectorXd& _qdot);

    //bool isValidate();

protected:
    /// @brief
    Eigen::MatrixXd mJ;

private:
};

} // namespace dynamics

#endif // DYNAMICS_INERTIA_H
