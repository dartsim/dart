/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author     Jie Tan
  Date      07/18/2011
*/

#ifndef RENDERER_LIGHT_H
#define RENDERER_LIGHT_H

#include "Eigen/Core"

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
        Light(const Eigen::Vector3d& _pos, const Eigen::Vector3d& _diffuse, const Eigen::Vector3d& _specular, LightType _type);

        LightType GetType() const;
        void SetPosition(const Eigen::Vector3d& _p);
        void GetPosition(float *_pos) const;
        void GetSpecular(float *_specular) const;
        void GetDiffuse(float *_diffuse) const;
        Eigen::Vector3d GetPosition() const;
        Eigen::Vector3d GetSpecular() const;
        Eigen::Vector3d GetDiffuse() const;
    public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	private:
		Eigen::Vector3d mPosition;
		Eigen::Vector3d mSpecular;
		Eigen::Vector3d mDiffuse;
		LightType mType;
    };
} // namespace renderer

#endif // #ifndef RENDERER_LIGHT_H
