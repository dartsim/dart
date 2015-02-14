#include "MyWorld.h"
#include "Particle.h"

using namespace Eigen;

MyWorld::MyWorld(int _numParticles) {
  // Create particles
  for (int i = 0; i < _numParticles; i++) {
    Particle *p1 = new Particle();
    Particle *p2 = new Particle();
    mParticles.push_back(p1);
    mParticles.push_back(p2);
  }
  // Init particle position
  mParticles[0]->mPosition[0] = 0.2;
  mParticles[1]->mPosition[0] = 0.2;
  mParticles[1]->mPosition[1] = -0.1;
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
