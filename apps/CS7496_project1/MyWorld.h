#ifndef _MYWORLD_
#define _MYWORLD_

#include "yui/Win3D.h"

class MyWorld {
 public:
    MyWorld() {
    }

    virtual ~MyWorld() {};
    
 protected:
    std::vector<Particle> mParticles;    
};

#endif
