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

#include "math/Inertia.h"

namespace math
{

#define LIEGROUP_EPS 10e-9

//==============================================================================
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

dart_math::Vector6d Inertia::operator*(const dart_math::Vector6d& _V) const
{
	/*--------------------------------------------------------------------------
	(angluar) M = I * w + m * (r X v - r X (r X w))
	(linear)  F = m * (v - r X w)
	--------------------------------------------------------------------------*/

	dart_math::Vector6d res;

	const Eigen::Vector3d& v = _V.tail<3>();
	const Eigen::Vector3d& w = _V.head<3>();

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

	res.tail<3>() = mMass * (v - mCOM.cross(w));
	res.head<3>() = Iw + mCOM.cross(res.tail<3>());

	return res;
}

Inertia Inertia::getTransformed(const Eigen::Matrix4d& _T12)
{
	// TODO: Not implemented.
}

Inertia Inertia::getTransformedInverse(const Eigen::Matrix4d& _T21)
{
	// TODO: Not implemented.
}

Eigen::Matrix<double,6,6> Inertia::getInertiaTensor() const
{
	Eigen::Matrix<double,6,6> InertiaTensor;

	// TODO: Not implemented.

	return InertiaTensor;
}

} // namespace math

