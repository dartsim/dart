/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#ifndef KINEMATICS_PRIMITIVE_CUBE_H
#define KINEMATICS_PRIMITIVE_CUBE_H

#include "Primitive.h"


namespace kinematics {

    class PrimitiveCube : public Primitive {
    public:
        PrimitiveCube(Eigen::Vector3d _dim, double _mass);

        void draw(renderer::RenderInterface* _ri = NULL, const Eigen::Vector4d& _col=Eigen::Vector4d::Ones(), bool _default = true) const;
    private:
        void computeMassTensor();
        void computeVolume();
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

} // namespace kinematics

#endif // #ifndef KINEMATICS_PRIMITIVE_CUBE_H

