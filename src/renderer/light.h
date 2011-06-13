#ifndef LIGHT_H
#define LIGHT_H

#include "Eigen/Core"
using namespace Eigen;

namespace Renderer
{
    enum LightType {
        LT_PointLight,
        LT_DirectionLight,

    };


    class Light {
    private:
        Vector3d mPosition;
        Vector3d mSpecular;
        Vector3d mDiffuse;
        LightType mType;

    public:
        /* construction function */
        Light();
        Light(LightType type);
        Light(const Vector3d& pos, const Vector3d& diffuse, const Vector3d& specular, LightType type);

        LightType GetType() const;
        void SetPosition (const Vector3d& p);
        void GetPosition (float * pos) const;
        void GetSpecular(float* specular) const;
        void GetDiffuse(float* diffuse) const;
        Vector3d GetPosition() const;
        Vector3d GetSpecular() const;
        Vector3d GetDiffuse() const;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}



#endif
