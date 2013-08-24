#include "MyWorld.h"
#include "Particle.h"

using namespace Eigen;

MyWorld::MyWorld(int _numParticles) {
    for (int i = 0; i < _numParticles; i++) {
        Particle *p = new Particle();
        mParticles.push_back(p);
    }

    // init particle positions
    mParticles[0]->mPosition[0] -= 0.3;
    mParticles[0]->mPosition[1] = 20.0;
    mParticles[1]->mPosition[1] = 20.0;
    mParticles[2]->mPosition[0] += 0.3;
    mParticles[2]->mPosition[1] = 20.0;

    // init particle colors
    mParticles[1]->mColor = Vector4d(0.2, 0.2, 0.9, 1.0);
    mParticles[2]->mColor = Vector4d(0.2, 0.2, 0.9, 1.0);
}

MyWorld::~MyWorld() {
    for (int i = 0; i < mParticles.size(); i++)
        delete mParticles[i];
    mParticles.clear();
}

void MyWorld::simulate() {
    // replace the following code
    for (int i = 0; i < mParticles.size(); i++)
        mParticles[i]->mPosition[1] -= 0.005;
}
