#ifndef _MYWORLD_
#define _MYWORLD_

#include "simulation/World.h"
#include "Controller.h"

class MyWorld : public simulation::World
{
 public:
 MyWorld(): World() {
        mAccumulatedImpact = 0.0;
        mController = NULL;
        mWallMaterial = 100000;
        mGroundHeight = -10.0;
    }
    virtual ~MyWorld() {}
    
    void setController(Controller *_controller) {
        mController = _controller;
    }

    Controller* getController() {
        return mController;
    }

    void computeImpact();
    void computeJointStress();

    double getImpact() {
        return mAccumulatedImpact;
    }
    void controlWalls();

    int getWallMaterial() {
        return mWallMaterial;
    }
    void setWallMaterial(int _material) {
        mWallMaterial = _material;
    }

    double getGroundHeight() {
        return mGroundHeight;
    }
 private:    
    Controller *mController;    
    double mAccumulatedImpact;
    int mWallMaterial;
    double mGroundHeight;
};

#endif
