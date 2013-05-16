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

#ifndef DART_MATH_INERTIA_H
#define DART_MATH_INERTIA_H

#include <Eigen/Dense>
#include "math/UtilsMath.h"

namespace dart_math
{
class Inertia; // Inertia with cog offset (6x6 matrix)

/// @brief Inertia is a class for representing generalized inertia tensor.
///
/// Generalized inertia, G = | Inertia  0 |
///                          | 0       mI |
/// Generalized inertia with cog offset r, G = | Inertia - m[r][r]   m[r] |
///                                            |             -m[r]     mI |
/// where Inertia is momentum of inertia, m is mass, I is 3x3 identity matrix,
/// and [r] is skew-symmetrix matrix of cog offset.
class Inertia
{
    
public: // Constructors and destructor
    // Aligned allocator for Eigen member variable.
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    /// @brief
    Inertia();
    
    /// @brief
    Inertia(const Inertia& _I);
    
    /// @brief
    explicit Inertia(double _mass, double _Ixx, double _Iyy, double _Izz);
    
    /// @brief
    explicit Inertia(double _mass,
                     double _Ixx, double _Iyy, double _Izz,
                     double _Ixy, double _Ixz, double _Iyz,
                     double _comX, double _comY, double _comZ);
    
    /// @brief
    explicit Inertia(double _mass,
                     const Eigen::Vector3d& _principals,
                     const Eigen::Vector3d& _products,
                     const Eigen::Vector3d& _com);
    
    /// @brief
    ~Inertia();
    
public: // Operators
    /// @brief Substitution operator.
    const Inertia& operator = (const Inertia& _I);

    /// @brief Casting operator.
    Inertia* operator&() { return this; }

    /// @brief Const Casting operator.
    const Inertia* operator&() const { return this; }

    /// @brief Multiplication operator.
    /// @note \f$J V = ( Iw + r\times v,~ mv-r\times w)\in se(3)^*\f$,
    /// where \f$J = (I,m,r)\in\f$ Inertia, \f$V = (w,v)\in se(3)\f$.
    dart_math::Vector6d operator*(const dart_math::Vector6d& _V) const;

public: // Others
    /// @brief
    void setMass(double _mass) { mMass = _mass; }
    
    /// @brief
    double getMass() const { return mMass; }
    
    /// @brief
    void setPrincipals(double _Ixx, double _Iyy, double _Izz)
    {
        mPrincipals(0) = _Ixx;
        mPrincipals(1) = _Iyy;
        mPrincipals(2) = _Izz;
    }

    void setProducts(double _Ixy, double _Ixz, double _Iyz)
    {
        mProducts(0) = _Ixy;
        mProducts(1) = _Ixz;
        mProducts(2) = _Iyz;
    }

    /// @brief
    void setPrincipals(const Eigen::Vector3d& _principals)
    { mPrincipals = _principals; }
    
    /// @brief
    const Eigen::Vector3d& getPrincicpals() const { return mPrincipals; }
    
    /// @brief
    void setProducts(const Eigen::Vector3d& _products)
    { mProducts = _products; }
    
    /// @brief
    const Eigen::Vector3d& getProducts() const { return mProducts; }
    
    /// @brief
    void setMomentsOfInertia(const Eigen::Matrix3d& _moi);
    
    /// @brief
    Eigen::Matrix3d getMomentsOfInertia() const;
    
    /// @brief
    void setCenterOfMass(const Eigen::Vector3d& _com);
    
    /// @brief
    const Eigen::Vector3d& getCenterOfMass() const { return mCOM; }
    
    /// @brief
    // TODO: NOT IMPLEMENTED.
    Eigen::Matrix<double,6,6> getInertiaTensor() const;

    // TODO: Not implemented.
    /// @brief Get trnasformed generalized inertia.
    /// @param[in] _T12 Transformation matrix from frame(1) to frame(2)
    /// @return Generalized inertia in frame(1)
    ///
    /// \f$ Ad_{T12^{-1}}^{*} G2 Ad_{T12^{-1}} \f$.
    Inertia getTransformed(const Eigen::Matrix4d& _T12) const;
    
    // TODO: Not implemented.
    /// @brief Get trnasformed generalized inertia.
    /// @param[in] _T21 Transformation matrix from frame(2) to frame(1)
    /// @return Generalized inertia in frame(1)
    ///
    /// \f$ Ad_{T21}^{*} G2 Ad_{T21} \f$.
    Inertia getTransformedInverse(const Eigen::Matrix4d& _T21) const;
    
protected:
    /// @brief Mass the object. Default is 1.0.
    double mMass;
    
    /// @brief Principal moments of inertia. Default is (1.0 1.0 1.0)
    ///
    /// Ixx = mPrincipals(0), Iyy = mPrincipals(1), Izz = mPrincipals(2).
    /// These Moments of Inertia are specified in the local Inertial frame.
    Eigen::Vector3d mPrincipals;
    
    /// @brief Product moments of inertia. Default is (0.0 0.0 0.0)
    /// These MOI off-diagonals are specified in the local Inertial frame.
    /// Where mProducts(0) is Ixy, mProducts(1) is Ixz and mProducts(2) is Iyz.
    Eigen::Vector3d mProducts;
    
    /// @brief Center of mass in the Link frame.
    /// Default is (0.0 0.0 0.0  0.0 0.0 0.0)
    Eigen::Vector3d mCOM;
    
private:
};

} // namespace math

#endif // DART_MATH_LIE_GROUPS_H
