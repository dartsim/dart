/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author     Jie Tan
  Date      07/18/2011
*/

#ifndef RENDERER_LIGHT_H
#define RENDERER_LIGHT_H

#include "Eigen/Core"
using namespace Eigen;

namespace renderer {
    enum LightType {
        LT_PointLight,
        LT_DirectionLight,
    };


    class Light {
    public:
        /* construction function */
        Light();
        Light(LightType type);
        Light(const Vector3d& _pos, const Vector3d& _diffuse, const Vector3d& _specular, LightType _type);

        LightType GetType() const;
        void SetPosition(const Vector3d& _p);
        void GetPosition(float *_pos) const;
        void GetSpecular(float *_specular) const;
        void GetDiffuse(float *_diffuse) const;
        Vector3d GetPosition() const;
        Vector3d GetSpecular() const;
        Vector3d GetDiffuse() const;
    public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	private:
		Vector3d mPosition;
		Vector3d mSpecular;
		Vector3d mDiffuse;
		LightType mType;
    };
} // namespace renderer

#endif // #ifndef RENDERER_LIGHT_H
