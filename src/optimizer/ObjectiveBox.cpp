#include "ObjectiveBox.h"
#include "Constraint.h"
using namespace Eigen;

namespace optimizer {

    ObjBox::ObjBox(int numDofs) {
        SetNumDofs(numDofs);
        mG = 0.0;
    }
    
    ObjBox::~ObjBox() {
        Clear();
    }

    void ObjBox::Add(Constraint *newObjective) {
        mObjectives.push_back(newObjective);
    }

    void ObjBox::Clear() {
        mObjectives.clear();
    }

    int ObjBox::TakeOut(Constraint *obj) {
        int index = -1;
        for(int i = 0; i < mObjectives.size(); i++){
            if(mObjectives[i] == obj){
                index = i;
                break;
            }
        }

        if(index == -1) {
            return index;
        }

        //deallocate memory for Constraint
        mObjectives.erase(mObjectives.begin() + index);

        return index;
    }

    int ObjBox::IsInBox(Constraint *testObj)  {
        for(int i = 0; i < mObjectives.size(); i++) {
            if(mObjectives[i] == testObj)
                return i;
        }
        return -1;
    }

    void ObjBox::SetNumDofs(int numDofs) {
        mNumDofs = numDofs;
        mG = 0;
        mdG.resize(numDofs);

        for(int i = 0; i < numDofs; i++) {
            mdG[i] = 0.0;
        }
    }

    void ObjBox::EvaldG() {
        for(int i = 0; i < mObjectives.size(); i++) {
            if(mObjectives[i]->mActive){
                mObjectives[i]->FilldG(mdG);
            }
        }
    }

    void ObjBox::EvalddG() {
    }

    void ObjBox::EvalG() {
        mG = 0;
        for(int i = 0; i < mObjectives.size(); i++)
            if(mObjectives[i]->mActive)
                mG += mObjectives[i]->EvalG();
    }

    void ObjBox::ReallocateMem() {
        for(int i = 0; i < mObjectives.size(); i++)
            mObjectives[i]->AllocateMem();
    }

    
} // namespace optimizer
