#ifndef _MYWINDOW_
#define _MYWINDOW_

#include "yui/Win3D.h"

namespace kinematics {
    class Skeleton;
}

namespace optimizer {
    class ObjectiveBox;
    class Var;
}

class MyWindow : public yui::Win3D {
public:
 MyWindow(kinematics::Skeleton* _skel): Win3D(), mSkel(_skel) {
        mBackground[0] = 1.0;
        mBackground[1] = 1.0;
        mBackground[2] = 1.0;
        mBackground[3] = 1.0;		
        mPersp = 30.f;
        mTrans[2] = -1.f;

        initIK();
    }

    virtual void draw();
    virtual void keyboard(unsigned char key, int x, int y);
    virtual void displayTimer(int _val);

 protected:
    kinematics::Skeleton *mSkel;
    optimizer::ObjectiveBox *mObjBox;
    std::vector<optimizer::Var*> mVariables;

    void initIK();
    void solveIK();
};

#endif
