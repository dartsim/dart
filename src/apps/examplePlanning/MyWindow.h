#ifndef _MYWINDOW_
#define _MYWINDOW_

#include "yui/Win3D.h"
#include "integration/EulerIntegrator.h"
#include "integration/RK4Integrator.h"
#include "collision/CollisionShapes.h"
#include "collision/CollisionSkeleton.h"
#include "utils/Parser/dart_parser/DartLoader.h"
#include "robotics/World.h"
#include "utils/Paths.h"
#include "robotics/World.h"
#include "dynamics/SkeletonDynamics.h"
#include "robotics/Object.h"
#include "kinematics/ShapeCube.h"
#include "dynamics/BodyNodeDynamics.h"
#include "kinematics/Joint.h"

using namespace collision_checking;
using namespace std;

namespace dynamics{
    class SkeletonDynamics;
    class ContactDynamics;
}

namespace integration{
    class IntegrableSystem;
}

class MyWindow : public yui::Win3D, public integration::IntegrableSystem {
public:
MyWindow(): Win3D() {
        DartLoader dl;
        mWorld = dl.parseWorld(DART_DATA_PATH"/scenes/DesktopArm.urdf");
        
        // Add ground plane
        robotics::Object* ground = new robotics::Object();
        ground->addDefaultRootNode();
        dynamics::BodyNodeDynamics* node = new dynamics::BodyNodeDynamics();
        node->setShape(new kinematics::ShapeCube(Eigen::Vector3d(10.0, 10.0, 0.0001), 1.0));
        kinematics::Joint* joint = new kinematics::Joint(ground->getRoot(), node);
        ground->addNode(node);
		ground->setPositionZ(-0.1);
        
		ground->initSkel();
		ground->update();
		ground->setImmobileState(true);
        mWorld->addObject(ground);
		mWorld->rebuildCollision();

        mBackground[0] = 1.0;
        mBackground[1] = 1.0;
        mBackground[2] = 1.0;
        mBackground[3] = 1.0;

        mPlayState = PAUSED;
        mSimFrame = 0;
        mPlayFrame = 0;
        mMovieFrame = 0;


        mShowMarker = false;
        mForce = Eigen::Vector3d::Zero();

        mPersp = 45.f;
        mTrans[1] = 300.f;
    
        mGravity = Eigen::Vector3d(0.0, 0.0, -9.8);
        mTimeStep = 1.0/1000.0;
        
        int sumNDofs = 0;
        mIndices.push_back(sumNDofs);
        for (unsigned int i = 0; i < mWorld->getNumSkeletons(); i++) {
            int nDofs = mWorld->getSkeleton(i)->getNumDofs();
            sumNDofs += nDofs;
            mIndices.push_back(sumNDofs);
        }

        initDyn();

        std::cout << 
            "\nKeybindings:\n" <<
            "\n" <<
            "s: start or continue simulating.\n" <<
            "\n" <<
            "p: start or continue playback.\n" <<
            "r, t: move to start or end of playback.\n" <<
            "[, ]: step through playback by one frame.\n" <<
            "\n" <<
            "m: start or continue movie recording.\n" <<
            "\n" <<
            "space: pause/unpause whatever is happening.\n" <<
            "\n" <<
            "q, escape: quit.\n" <<
            std::endl;
    }

    virtual void draw();
    virtual void keyboard(unsigned char key, int x, int y);
    virtual void displayTimer(int _val);

    // Needed for integration
    virtual Eigen::VectorXd getState();
    virtual Eigen::VectorXd evalDeriv();
    virtual void setState(Eigen::VectorXd state);	
protected:
    enum playstate_enum {
        SIMULATE,
        RECORD,
        PLAYBACK,
        PAUSED
    };
    playstate_enum mPlayState;
    playstate_enum mPlayStateLast;
    int mSimFrame;
    int mPlayFrame;
    int mMovieFrame;
    bool mScreenshotScheduled;

    bool mShowMarker;
    std::vector<Eigen::VectorXd> mBakedStates;
    Eigen::Vector3d mForce;

    integration::EulerIntegrator mIntegrator;

    robotics::World* mWorld;
    std::vector<Eigen::VectorXd> mDofVels;
    std::vector<Eigen::VectorXd> mDofs;
    double mTimeStep;
    Eigen::Vector3d mGravity;
    std::vector<int> mIndices;

    void initDyn();
    void bake();
    void retrieveBakedState(int frame);
};

#endif
