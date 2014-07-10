#ifndef _MYWORLD_
#define _MYWORLD_

#include <vector>

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

  // TODO: your simulation code goes here
  void simulate();
    
 protected:
  std::vector<Particle*> mParticles;    
};

#endif
