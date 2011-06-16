/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#ifndef MODEL3D_PRIMITIVE_ELLIPSOID_H
#define MODEL3D_PRIMITIVE_ELLIPSOID_H

#include "Primitive.h"

namespace model3d {

    class PrimitiveEllipsoid : public Primitive {
    public:
        PrimitiveEllipsoid(Eigen::Vector3d _dim, double Mass); 
	
        void draw(const Eigen::Vector4d& _col=Eigen::Vector4d::Ones(), bool _useDefaultColor = true) const;
    private:
        void calMassTensor();
        void calVolume();
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

} // namespace model3d

#endif // #ifndef MODEL3D_PRIMITIVE_ELLIPSOID_H


