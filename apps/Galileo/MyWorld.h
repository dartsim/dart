#ifndef _MYWORLD_
#define _MYWORLD_

#include "yui/Win3D.h"
#include "Particle.h"

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

    void simulate();
    
 protected:
    std::vector<Particle*> mParticles;    
};

#endif
