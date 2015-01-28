#ifndef _PARTICLE_
#define _PARTICLE_

#include <Eigen/Dense>

namespace dart {
namespace renderer {
    class RenderInterface;
}
}

class Particle {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Particle() {
        // Create a default particle
        mMass = 1.0;
        mPosition.setZero();
        mVelocity.setZero();
        mAccumulatedForce.setZero();
        mColor << 0.9, 0.2, 0.2, 1.0; // Red
    }
    virtual ~Particle() {}

    void draw(dart::renderer::RenderInterface* _ri);
    
    double mMass;
    Eigen::Vector3d mPosition;
    Eigen::Vector3d mVelocity;
    Eigen::Vector3d mAccumulatedForce;

    Eigen::Vector4d mColor;
};

#endif
