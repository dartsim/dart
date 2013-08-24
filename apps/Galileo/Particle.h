#ifndef _PARTICLE_
#define _PARTICLE_

#include <Eigen/Dense>

namespace renderer {
    class RenderInterface;
}

class Particle {
 public:
    Particle() {
        mMass = 1.0;
        mPosition.setZero();
        mVelocity.setZero();
        mAccumulatedForce.setZero();
        mColor << 0.9, 0.2, 0.2, 1.0;
    }
    virtual ~Particle() {}

    void draw(renderer::RenderInterface* _ri);
    
    double mMass;
    Eigen::Vector3d mPosition;
    Eigen::Vector3d mVelocity;
    Eigen::Vector3d mAccumulatedForce;

    Eigen::Vector4d mColor;
};

#endif
