#include "MyWorld.h"
#include "Particle.h"
#include "math/UtilsMath.h"

using namespace Eigen;

MyWorld::MyWorld(int _numParticles) {
    // Create particles
    for (int i = 0; i < _numParticles; i++) {
        Particle *p = new Particle();
        mParticles.push_back(p);

        // Init particle positions randomly
        for (int j = 0; j < 3; j++) {
            double position = dart_math::random(-0.49, 0.49);
            mParticles[i]->mPosition[j] = position;
        }
    }

    // Init cube position
    mCubePosition.setZero();
}

MyWorld::~MyWorld() {
    for (int i = 0; i < mParticles.size(); i++)
        delete mParticles[i];
    mParticles.clear();
}

void MyWorld::simulate() {
    // Replace the following code
    for (int i = 0; i < mParticles.size(); i++)
        mParticles[i]->mPosition[1] -= 0.005;
}
