#ifndef _PARTICLE_
#define _PARTICLE_

#include <Eigen/Dense>

class particle {
 public:
    Particle() {
        mMass = 1.0;
        mPosition.setZero();
        mVelocity.setZero();
        mAccumulatedForce.setZero();
    }
    virtual ~Particle() {}

    void draw();
    
    double mMass;
    Eigen::Vector3d mPosition;
    Eigen::Vector3d mVelocity;
    Eigen::Vector3d mAccumulatedForce;

    Eigen::Vector4d mColor;
}

#endif _PARTICLE_
