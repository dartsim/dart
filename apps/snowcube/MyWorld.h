#ifndef _MYWORLD_
#define _MYWORLD_

#include <vector>
#include <Eigen/Dense>

class Particle;

class MyWorld {
 public:
    MyWorld(int _numParticles);

    virtual ~MyWorld();

    int getNumParticles() {
        return mParticles.size();
    }

    Particle* getParticle(int _index) {
        return mParticles[_index];
    }

    Eigen::Vector3d getCubePosition() {
        return mCubePosition;
    }

    // TODO: your simulation code goes here
    void simulate();
    
 protected:
    std::vector<Particle*> mParticles;
    Eigen::Vector3d mCubePosition;

};

#endif
