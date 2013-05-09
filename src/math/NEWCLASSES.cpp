/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 * Date: 05/05/2013
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

#include "math/NEWCLASSES.h"

namespace math
{

#define LIEGROUP_EPS 10e-9
Inertia::Inertia()
    : mMass(1.0),
      mPrincipals(Eigen::Vector3d::Ones()),
      mProducts(Eigen::Vector3d::Zero()),
      mCOM(Eigen::Vector3d::Zero())
{
}

Inertia::Inertia(const Inertia& _I)
    : mMass(_I.mMass),
      mPrincipals(_I.mPrincipals),
      mProducts(_I.mProducts),
      mCOM(_I.mCOM)
{
}

Inertia::Inertia(double _mass, double _Ixx, double _Iyy, double _Izz)
    : mMass(_mass),
      mProducts(Eigen::Vector3d::Zero()),
      mCOM(Eigen::Vector3d::Zero())
{
    mPrincipals << _Ixx, _Iyy, _Izz;
}

Inertia::Inertia(double _mass,
                 double _Ixx, double _Iyy, double _Izz,
                 double _Ixy, double _Ixz, double _Iyz,
                 double _comX, double _comY, double _comZ)
    : mMass(_mass)
{
    mPrincipals << _Ixx, _Iyy, _Izz;
    mProducts << _Ixy, _Ixz, _Iyz;
    mCOM << _comX, _comY, _comZ;
}

Inertia::Inertia(double _mass, const Eigen::Vector3d& _principals,
                 const Eigen::Vector3d& _products, const Eigen::Vector3d& _com)
    : mMass(_mass),
      mPrincipals(_principals),
      mProducts(_products),
      mCOM(_com)
{
}

Inertia::~Inertia()
{
}

const Inertia& Inertia::operator = (const Inertia& _I)
{
	if(this != &_I)
	{
		mMass = _I.mMass;
		mPrincipals = _I.mPrincipals;
		mProducts = _I.mProducts;
		mCOM = _I.mCOM;
	}
	return (*this);
}

dse3 Inertia::operator * (const se3& _V) const
{
	//
	// (angluar) M = I * w + m * (r X v - r X (r X w))
	// (linear)  F = m * (v - r X w)
	//

	dse3 res;

	const Eigen::Vector3d& v = _V.getLinear();
	const Eigen::Vector3d& w = _V.getAngular();

	Eigen::Vector3d Iw;
	Iw(0) = mPrincipals(0) * w(0)
			+ mProducts(0) * w(1)
			+ mProducts(1) * w(2);
	Iw(1) = mProducts(0) * w(0)
			+ mPrincipals(1) * w(1)
			+ mProducts(2) * w(2);
	Iw(2) = mProducts(1) * w(0)
			+ mProducts(2) * w(1)
			+ mPrincipals(2) * w(2);

	res.setLinear(mMass * (v - mCOM.cross(w)));
	res.setAngular(Iw + mCOM.cross(res.getLinear()));

	return res;
}

Inertia Inertia::getTransformed(const SE3& _T12)
{
	// TODO: Not implemented.
}

Inertia Inertia::getTransformedInverse(const SE3& _T21)
{
	// TODO: Not implemented.
}

//==============================================================================
SO3::SO3()
{
}

SO3::SO3(const Eigen::Matrix3d& _rotation)
	: mR(_rotation)
{
}

SO3::~SO3()
{
}

Eigen::Vector3d SO3::operator*(const Eigen::Vector3d& _q) const
{
	return mR * _q;
}

void SO3::setExp(const so3& _S)
{
	const Eigen::Vector3d& s = _S.getVector();

	double s2[] = { s[0] * s[0], s[1] * s[1], s[2] * s[2] };
	double theta = sqrt(s2[0] + s2[1] + s2[2]);
	double st_t = 0.0;
	double ct_t = 0.0;

	if ( theta < LIEGROUP_EPS )
	{
		st_t = 1.0 - theta * theta / (double)6.0;
		ct_t = 0.5 - theta * theta / (double)24.0;
	} else
	{
		st_t = sin(theta) / theta;
		ct_t = (1.0 - cos(theta)) / theta / theta;
	}

	mR(0,0) = 1.0 - ct_t * (s2[1] + s2[2]);
	mR(1,0) = ct_t * s[0] * s[1] + st_t * s[2];
	mR(2,0) = ct_t * s[0] * s[2] - st_t * s[1];
	mR(0,1) = ct_t * s[0] * s[1] - st_t * s[2];
	mR(1,1) = 1.0 - ct_t * (s2[0] + s2[2]);
	mR(2,1) = ct_t * s[1] * s[2] + st_t * s[0];
	mR(0,2) = ct_t * s[0] * s[2] + st_t * s[1];
	mR(1,2) = ct_t * s[1] * s[2] - st_t * s[0];
	mR(2,2) = 1.0 - ct_t * (s2[0] + s2[1]);
}

void SO3::setExp(const so3& _S, double theta)
{
	const Eigen::Vector3d& s = _S.getVector();

	double s2[] = { s[0] * s[0], s[1] * s[1], s[2] * s[2] };

	if ( fabs(s2[0] + s2[1] + s2[2] - 1.0) > LIEGROUP_EPS )
	{
		setExp(so3(theta * s));
	}

	double st = sin(theta),
		   vt = 1.0 - cos(theta),
		   sts[] = { st * s[0], st * s[1], st * s[2] };

	mR(0,0) = 1.0 + vt * (s2[0] - 1.0);
	mR(1,0) = vt * s[0] * s[1] + sts[2];
	mR(2,0) = vt * s[0] * s[2] - sts[1];
	mR(0,1) = vt * s[0] * s[1] - sts[2];
	mR(1,1) = 1.0 + vt * (s2[1] - 1.0);
	mR(2,1) = vt * s[1] * s[2] + sts[0];
	mR(0,2) = vt * s[0] * s[2] + sts[1];
	mR(1,2) = vt * s[1] * s[2] - sts[0];
	mR(2,2) = 1.0 + vt * (s2[2] - 1.0);
}

//==============================================================================
SE3::SE3()
    : mRotation(SO3()),
      mPosition(Eigen::Vector3d::Zero())
{
}

SE3::SE3(const SO3& _rotation, const Eigen::Vector3d& _position)
    : mRotation(_rotation),
      mPosition(_position)
{
}

SE3::~SE3()
{
}

Eigen::Matrix4d SE3::getMatrix()
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    T.topLeftCorner<3,3>() = mRotation.getMatrix();
    T.topRightCorner<3,1>() = mPosition;

    return T;
}

//==============================================================================
se3::se3()
    : mLinear(Eigen::Vector3d::Zero()),
      mAngular(Eigen::Vector3d::Zero())
{
}

se3::se3(const se3& _v)
    : mLinear(_v.mLinear),
      mAngular(_v.mAngular)
{
}

se3::se3(const Eigen::Vector3d& _linear, const Eigen::Vector3d& _angular)
    : mLinear(_linear),
      mAngular(_angular)
{
}

se3::~se3()
{
}

const se3& se3::operator = (const se3& _v)
{
    if(this != &_v)
    {
        mLinear = _v.mLinear;
        mAngular = _v.mAngular;
    }
}

void se3::setAd(const SE3& _T12, const se3& _V2)
{
    //
    // Let,
    //     _T12 = | R12 p12 |,
    //            |   0   1 |
    //      mAngular = w1, mLinear = v1,
    //     _V.mAngular = w2, _V.mLinear = v2.
    // Then,
    //     w1 = R12 * w2
    //     v1 = p12 x (R12 * w2) + R12 * v2
    //

	const Eigen::Vector3d& v2 = _V2.getLinear();
	const Eigen::Vector3d& w2 = _V2.getAngular();
	const SO3& R12 = _T12.getRotation();
	const Eigen::Vector3d& p12 = _T12.getPosition();

	mAngular = R12 * w2;
	mLinear = p12.cross(mAngular) + R12 * v2;
}

void se3::setInvAd(const SE3& _T21, const se3& _V2)
{
	//
	// Let,
	//     _T21 = | R21 p21 |,
	//            |   0   1 |
	//      mAngular = w1, mLinear = v1,
	//     _V.mAngular = w2, _V.mLinear = v2.
	//     T12 = _T21^{-1} = | R21^T   -R21^T * p21 | = | R12 p12 |
	//                       |     0              1 |   |   0   1 |
	//
	// Then,
	//     w1 = R12 * w2
	//        = R21^T * w2
	//     v1 = p12 x (R12 * w2) + R12 * v2
	//        = (-R21^T * p21) x (R21^T * w2) + R21^T * v2
	//        = -R21^T (p21 x w2) + R21^T * v2
	//

	const Eigen::Vector3d& v2 = _V2.getLinear();
	const Eigen::Vector3d& w2 = _V2.getAngular();
	const SO3& R21T = _T21.getRotation().getInverse();
	const Eigen::Vector3d& p21 = _T21.getPosition();

	mAngular = R21T * w2;
	mLinear = -(R21T * p21.cross(w2)) + R21T * v2;
}

void se3::setad(const se3& _V1, const se3& _V2)
{
	const Eigen::Vector3d& v1 = _V1.getLinear();
	const Eigen::Vector3d& w1 = _V1.getAngular();

	const Eigen::Vector3d& v2 = _V2.getLinear();
	const Eigen::Vector3d& w2 = _V2.getAngular();

	mAngular = w1.cross(w2);
	mLinear = v1.cross(w2) + w1.cross(v2);
}

Jacobian::Jacobian()
{
}

Jacobian::~Jacobian()
{
}

void Jacobian::setMatrix(const Eigen::MatrixXd& _J)
{
    assert(_J.rows() == 6);

    mJ = _J;
}

se3 Jacobian::operator*(const Eigen::VectorXd& _qdot)
{
    assert(_qdot.size() == getSize());

    return se3(mJ * _qdot);
}

so3::so3(const Eigen::Vector3d& _v)
    : mAngular(_v)
{

}





} // namespace math

