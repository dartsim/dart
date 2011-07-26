/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sumit Jain
  Date		07/24/2011
*/
#include "MayaExportMotion.h"
#include "MayaExportSkeleton.h"
#include "utils/RotationConversion.h"

#include "model3d/Skeleton.h"
#include "model3d/FileInfoDof.h"
#include "model3d/BodyNode.h"
#include "model3d/Joint.h"
#include "model3d/Transformation.h"
#include "model3d/Dof.h"

#include <iostream>

using namespace std;
using namespace model3d;
using namespace Eigen;

namespace mayaexports{

    bool MayaExportMotion::exportMayaAnim( const char* _fName, int _start, int _end, const string &_nodesPrefix, int _writeNumNodes ){
        ofstream outFile0(_fName, ios::out);
        if( outFile0.fail()==1 ) return false;

        int numFrames = mDofData->getNumFrames();
        int first = _start<numFrames?_start:numFrames-1;
        int last = _end<numFrames?_end:numFrames-1;

        //outFile0.precision(20);

        outFile0 << "animVersion 1.1;" << endl;
        outFile0 << "mayaVersion 2011;" << endl;
        outFile0 << "timeUnit film;" << endl;
        outFile0 << "linearUnit cm;" << endl;
        outFile0 << "angularUnit rad;" << endl;
        outFile0 << "startTime " << first<<";\n";
        outFile0 << "endTime " << last<<";\n\n";

        //bool success = exportMayaAnimSegment(outFile0, first, last, mSkel->getRoot(), _nodesPrefix, _writeNumNodes, 0);
        bool success = exportMayaAnimSegment2(outFile0, first, last, mSkel->getRoot(), _nodesPrefix, _writeNumNodes, 0);

        outFile0.close();
        return success;
    }

    bool MayaExportMotion::exportMayaAnimSegment(ofstream &outFile0, int _first, int _last, BodyNode *_b, const string &_nodesPrefix, int _writeNumNodes, int level){
        int dofNum=0;
        int rotationCode=0;	// 1 is X, 2 is Y and 4 is Z

        int numChildJoints = _b->getNumChildJoints();
        for(int i=0; i<_b->getNumChildJoints(); i++){
            bool var = false;
            for(int j=0; j<_b->getChildJoint(i)->getNumTransforms(); j++){
                if(_b->getChildJoint(i)->getTransform(j)->getVariable()) {
                    var = true;
                    break;
                }
            }
            //if(!var) numChildJoints--;
            if((!var && _b->getChildJoint(i)->getChildNode()->getNumChildJoints()==0) || _b->getChildJoint(i)->getChildNode()->getSkelIndex()>=_writeNumNodes) numChildJoints--;
        }

        string bodyname = _nodesPrefix;
        bodyname += string(_b->getName());

        vector<int> varIndex;	// indices of variable rot dofs
        for(int i=_b->getParentJoint()->getNumTransforms()-1; i>=0; i--){
            Transformation *tf = _b->getParentJoint()->getTransform(i);
            if(!tf->getVariable()) continue;	// joint order should be determined only by variable dofs
            if(tf->getType()==Transformation::T_ROTATEX) {
                varIndex.push_back(i);
            }
            if(tf->getType()==Transformation::T_ROTATEY) {
                varIndex.push_back(i);
            }
            if(tf->getType()==Transformation::T_ROTATEZ) {
                varIndex.push_back(i);
            }
            if(tf->getType()==Transformation::T_ROTATEEXPMAP) {
                varIndex.push_back(i);
            }
        }
        // -----------------------
        // determine if all variable indices consecutive or not OR whether the last is variable?
        bool rotException = false;
        for(unsigned int i=1; i<varIndex.size(); i++){
            if(varIndex[i-1] - varIndex[i] != 1){
                rotException = true;
                cout<<"rotException (not consecutive) for body: "<<_b->getName()<<endl;
            }
        }
        if(varIndex.size()>0 && varIndex[0]!=_b->getParentJoint()->getNumTransforms()-1) {
            cout<<"rotException (not last) for body: "<<_b->getName()<<endl;
            rotException=true;
        }
        int separateRots = -1;	// index for the first variable dofs
        if(rotException) separateRots = varIndex[varIndex.size()-1];
        // -----------------------

        bool wroteNode = false;

        if(rotException){
            vector<Vector3d> eulerValues;
            for(int fi=_first; fi<=_last; fi++){
                Matrix3d rotdofs = Matrix3d::Identity();
                for(int i=_b->getParentJoint()->getNumTransforms()-1; i>=0; i--){
                    Transformation *tf = _b->getParentJoint()->getTransform(i);
                    if(i<separateRots) break;
                    vector<double> vals(tf->getNumDofs(), 0.0);
                    for(unsigned int di=0; di<vals.size(); di++){
                        if(tf->getVariable()) vals[di] = mDofData->getDofAt(fi, tf->getDof(0)->getSkelIndex());
                        else vals[di] = tf->getDof(0)->getValue();
                    }
                    if(tf->getType()==Transformation::T_ROTATEX) rotdofs = utils::rot_conv::eulerToMatrixX(vals[0])*(rotdofs);
                    if(tf->getType()==Transformation::T_ROTATEY) rotdofs = utils::rot_conv::eulerToMatrixY(vals[0])*(rotdofs);
                    if(tf->getType()==Transformation::T_ROTATEZ) rotdofs = utils::rot_conv::eulerToMatrixZ(vals[0])*(rotdofs);
                    if(tf->getType()==Transformation::T_ROTATEEXPMAP) {
                        Vector3d exmap(vals[0],vals[1],vals[2]);
                        Quaterniond quat = utils::rot_conv::expToQuat(exmap);
                        rotdofs = utils::rot_conv::quatToMatrix(quat)*(rotdofs);
                    }
                }
                // convert to euler angles
                Vector3d erot = utils::rot_conv::matrixToEuler(rotdofs, utils::rot_conv::XYZ);	// leave to radians
                eulerValues.push_back(erot);
            }
            // write out the euler values
            wroteNode = true;

            // X rotation
            outFile0<<"anim rotate.rotateX rotateX "<<bodyname.c_str()<<" "<<level<<" "<<numChildJoints<<" "<<dofNum++<<endl;
            outFile0<<"animData {\n";
            outFile0<<"\tinput time;\n";
            outFile0<<"\toutput angular;\n";
            outFile0<<"\tkeys {\n";
            for(int i=_first; i<= _last; i++){
                outFile0<<"\t\t "<<i<<" "<<eulerValues[i-_first][0]<<" linear linear 1 1 0;\n";
            }
            outFile0<<"\t}\n";	// close keys
            outFile0<<"}\n\n";	// close animData

            // Y rotation
            outFile0<<"anim rotate.rotateY rotateY "<<bodyname.c_str()<<" "<<level<<" "<<numChildJoints<<" "<<dofNum++<<endl;
            outFile0<<"animData {\n";
            outFile0<<"\tinput time;\n";
            outFile0<<"\toutput angular;\n";
            outFile0<<"\tkeys {\n";

            for(int i=_first; i<= _last; i++){
                outFile0<<"\t\t "<<i<<" "<<eulerValues[i-_first][1]<<" linear linear 1 1 0;\n";
            }
            outFile0<<"\t}\n";	// close keys
            outFile0<<"}\n\n";	// close animData

            // Z rotation
            outFile0<<"anim rotate.rotateZ rotateZ "<<bodyname.c_str()<<" "<<level<<" "<<numChildJoints<<" "<<dofNum++<<endl;
            outFile0<<"animData {\n";
            outFile0<<"\tinput time;\n";
            outFile0<<"\toutput angular;\n";
            outFile0<<"\tkeys {\n";

            for(int i=_first; i<= _last; i++){
                outFile0<<"\t\t "<<i<<" "<<eulerValues[i-_first][2]<<" linear linear 1 1 0;\n";
            }
            outFile0<<"\t}\n";	// close keys
            outFile0<<"}\n\n";	// close animData

        }
        else {
            for(int i=_b->getParentJoint()->getNumTransforms()-1; i>=0; i--){
                Transformation *t = _b->getParentJoint()->getTransform(i);
                if(t->getVariable()==false) continue;
                wroteNode = true;
                if(t->getType()==Transformation::T_ROTATEEXPMAP){
                    // ASSUME "xyz" ordering for dof values
                    vector<Vector3d> eulerValues;
                    for(int i=_first; i<=_last; i++){
                        Vector3d exmap(mDofData->getDofAt(i, t->getDof(0)->getSkelIndex()), mDofData->getDofAt(i, t->getDof(1)->getSkelIndex()), mDofData->getDofAt(i, t->getDof(2)->getSkelIndex()));
                        Quaterniond q = utils::rot_conv::expToQuat(exmap);
                        Matrix3d rot = q.matrix();
                        Vector3d angles = utils::rot_conv::matrixToEuler(rot, utils::rot_conv::XYZ);
                        eulerValues.push_back(angles);
                    }

                    // X rotation
                    outFile0<<"anim rotate.rotateX rotateX "<<bodyname.c_str()<<" "<<level<<" "<<numChildJoints<<" "<<dofNum++<<endl;
                    outFile0<<"animData {\n";
                    outFile0<<"\tinput time;\n";
                    outFile0<<"\toutput angular;\n";
                    outFile0<<"\tkeys {\n";
                    for(int i=_first; i<= _last; i++){
                        outFile0<<"\t\t "<<i<<" "<<eulerValues[i-_first][0]<<" linear linear 1 1 0;\n";
                    }
                    outFile0<<"\t}\n";	// close keys
                    outFile0<<"}\n\n";	// close animData

                    // Y rotation
                    outFile0<<"anim rotate.rotateY rotateY "<<bodyname.c_str()<<" "<<level<<" "<<numChildJoints<<" "<<dofNum++<<endl;
                    outFile0<<"animData {\n";
                    outFile0<<"\tinput time;\n";
                    outFile0<<"\toutput angular;\n";
                    outFile0<<"\tkeys {\n";

                    for(int i=_first; i<= _last; i++){
                        outFile0<<"\t\t "<<i<<" "<<eulerValues[i-_first][1]<<" linear linear 1 1 0;\n";
                    }
                    outFile0<<"\t}\n";	// close keys
                    outFile0<<"}\n\n";	// close animData

                    // Z rotation
                    outFile0<<"anim rotate.rotateZ rotateZ "<<bodyname.c_str()<<" "<<level<<" "<<numChildJoints<<" "<<dofNum++<<endl;
                    outFile0<<"animData {\n";
                    outFile0<<"\tinput time;\n";
                    outFile0<<"\toutput angular;\n";
                    outFile0<<"\tkeys {\n";

                    for(int i=_first; i<= _last; i++){
                        outFile0<<"\t\t "<<i<<" "<<eulerValues[i-_first][2]<<" linear linear 1 1 0;\n";
                    }
                    outFile0<<"\t}\n";	// close keys
                    outFile0<<"}\n\n";	// close animData
                }
                else if(t->getType()==Transformation::T_TRANSLATE){
                    // X translation
                    outFile0<<"anim translate.translateX translateX "<<bodyname.c_str()<<" "<<level<<" "<<numChildJoints<<" "<<dofNum++<<endl;
                    outFile0<<"animData {\n";
                    outFile0<<"\tinput time;\n";
                    outFile0<<"\toutput linear;\n";
                    outFile0<<"\tkeys {\n";
                    for(int i=_first; i<= _last; i++){
                        outFile0<<"\t\t "<<i<<" "<<mDofData->getDofAt(i, t->getDof(0)->getSkelIndex())<<" linear linear 1 1 0;\n";
                    }
                    outFile0<<"\t}\n";	// close keys
                    outFile0<<"}\n\n";	// close animData

                    // Y translation
                    outFile0<<"anim translate.translateY translateY "<<bodyname.c_str()<<" "<<level<<" "<<numChildJoints<<" "<<dofNum++<<endl;
                    outFile0<<"animData {\n";
                    outFile0<<"\tinput time;\n";
                    outFile0<<"\toutput linear;\n";
                    outFile0<<"\tkeys {\n";

                    for(int i=_first; i<= _last; i++){
                        outFile0<<"\t\t "<<i<<" "<<mDofData->getDofAt(i, t->getDof(1)->getSkelIndex())<<" linear linear 1 1 0;\n";
                    }
                    outFile0<<"\t}\n";	// close keys
                    outFile0<<"}\n\n";	// close animData

                    // Z translation
                    outFile0<<"anim translate.translateZ translateZ "<<bodyname.c_str()<<" "<<level<<" "<<numChildJoints<<" "<<dofNum++<<endl;
                    outFile0<<"animData {\n";
                    outFile0<<"\tinput time;\n";
                    outFile0<<"\toutput linear;\n";
                    outFile0<<"\tkeys {\n";

                    for(int i=_first; i<= _last; i++){
                        outFile0<<"\t\t "<<i<<" "<<mDofData->getDofAt(i, t->getDof(2)->getSkelIndex())<<" linear linear 1 1 0;\n";
                    }
                    outFile0<<"\t}\n";	// close keys
                    outFile0<<"}\n\n";	// close animData
                }
                else if(t->getType()==Transformation::T_ROTATEX){
                    rotationCode+=1;	// 1 is X, 2 is Y and 4 is Z
                    // X rotation
                    outFile0<<"anim rotate.rotateX rotateX "<<bodyname.c_str()<<" "<<level<<" "<<numChildJoints<<" "<<dofNum++<<endl;
                    outFile0<<"animData {\n";
                    outFile0<<"\tinput time;\n";
                    outFile0<<"\toutput angular;\n";
                    outFile0<<"\tkeys {\n";

                    for(int i=_first; i<=_last; i++){
                        outFile0<<"\t\t "<<i<<" "<<mDofData->getDofAt(i, t->getDof(0)->getSkelIndex())<<" linear linear 1 1 0;\n";
                    }
                    outFile0<<"\t}\n";	// close keys
                    outFile0<<"}\n\n";	// close animData
                }
                else if(t->getType()==Transformation::T_ROTATEY){
                    rotationCode+=2;	// 1 is X, 2 is Y and 4 is Z
                    // Y rotation
                    outFile0<<"anim rotate.rotateY rotateY "<<bodyname.c_str()<<" "<<level<<" "<<numChildJoints<<" "<<dofNum++<<endl;
                    outFile0<<"animData {\n";
                    outFile0<<"\tinput time;\n";
                    outFile0<<"\toutput angular;\n";
                    outFile0<<"\tkeys {\n";

                    for(int i=_first; i<=_last; i++){
                        outFile0<<"\t\t "<<i<<" "<<mDofData->getDofAt(i, t->getDof(0)->getSkelIndex())<<" linear linear 1 1 0;\n";
                    }
                    outFile0<<"\t}\n";	// close keys
                    outFile0<<"}\n\n";	// close animData
                }
                else if(t->getType()==Transformation::T_ROTATEZ){
                    rotationCode+=4;	// 1 is X, 2 is Y and 4 is Z
                    // Z rotation
                    outFile0<<"anim rotate.rotateZ rotateZ "<<bodyname.c_str()<<" "<<level<<" "<<numChildJoints<<" "<<dofNum++<<endl;
                    outFile0<<"animData {\n";
                    outFile0<<"\tinput time;\n";
                    outFile0<<"\toutput angular;\n";
                    outFile0<<"\tkeys {\n";

                    for(int i=_first; i<=_last; i++){
                        outFile0<<"\t\t "<<i<<" "<<mDofData->getDofAt(i, t->getDof(0)->getSkelIndex())<<" linear linear 1 1 0;\n";
                    }
                    outFile0<<"\t}\n";	// close keys
                    outFile0<<"}\n\n";	// close animData
                }
                else if(t->getType()==Transformation::T_TRANSLATEX){
                    printf("MayaExportMotion::exportMayaAnimSegment - type T_TRANSLATEX not implemented\n");
                    return false;
                }
                else if(t->getType()==Transformation::T_TRANSLATEY){
                    printf("MayaExportMotion::exportMayaAnimSegment - type T_TRANSLATEY not implemented\n");
                    return false;
                }
                else if(t->getType()==Transformation::T_TRANSLATEZ){
                    printf("MayaExportMotion::exportMayaAnimSegment - type T_TRANSLATEZ not implemented\n");
                    return false;
                }
            }
        }

        if(!wroteNode){	// write some random attribute to zero
            outFile0<<"anim rotate.rotateX rotateX "<<bodyname.c_str()<<" "<<level<<" "<<numChildJoints<<" "<<dofNum++<<endl;
            outFile0<<"animData {\n";
            outFile0<<"\tinput time;\n";
            outFile0<<"\toutput angular;\n";
            outFile0<<"\tkeys {\n";

            for(int i=_first; i<=_last; i++){
                outFile0<<"\t\t "<<i<<" 0.000000"<<" linear linear 1 1 0;\n";
            }
            outFile0<<"\t}\n";	// close keys
            outFile0<<"}\n\n";	// close animData
        }

        // check the rotation code - need to add more rotation dofs
        // add X rotation
        if(rotationCode==2 || rotationCode==4 || rotationCode==6){
            outFile0<<"anim rotate.rotateX rotateX "<<bodyname.c_str()<<" "<<level<<" "<<numChildJoints<<" "<<dofNum++<<endl;
            outFile0<<"animData {\n";
            outFile0<<"\tinput time;\n";
            outFile0<<"\toutput angular;\n";
            outFile0<<"\tkeys {\n";

            for(int i=_first; i<=_last; i++){
                outFile0<<"\t\t "<<i<<" 0.000000"<<" linear linear 1 1 0;\n";
            }
            outFile0<<"\t}\n";	// close keys
            outFile0<<"}\n\n";	// close animData
        }
        // add Y rotation
        if(rotationCode==1 || rotationCode==4 || rotationCode==5){
            outFile0<<"anim rotate.rotateY rotateY "<<bodyname.c_str()<<" "<<level<<" "<<numChildJoints<<" "<<dofNum++<<endl;
            outFile0<<"animData {\n";
            outFile0<<"\tinput time;\n";
            outFile0<<"\toutput angular;\n";
            outFile0<<"\tkeys {\n";

            for(int i=_first; i<=_last; i++){
                outFile0<<"\t\t "<<i<<" 0.000000"<<" linear linear 1 1 0;\n";
            }
            outFile0<<"\t}\n";	// close keys
            outFile0<<"}\n\n";	// close animData
        }
        // add Z rotation
        if(rotationCode==1 || rotationCode==2 || rotationCode==3){
            outFile0<<"anim rotate.rotateZ rotateZ "<<bodyname.c_str()<<" "<<level<<" "<<numChildJoints<<" "<<dofNum++<<endl;
            outFile0<<"animData {\n";
            outFile0<<"\tinput time;\n";
            outFile0<<"\toutput angular;\n";
            outFile0<<"\tkeys {\n";

            for(int i=_first; i<=_last; i++){
                outFile0<<"\t\t "<<i<<" 0.000000"<<" linear linear 1 1 0;\n";
            }
            outFile0<<"\t}\n";	// close keys
            outFile0<<"}\n\n";	// close animData
        }

        // write segments for children joints as well
        level++;
        for(int i=0; i<_b->getNumChildJoints(); i++){
            bool var = false;
            for(int j=0; j<_b->getChildJoint(i)->getNumTransforms(); j++){
                if(_b->getChildJoint(i)->getTransform(j)->getVariable()) {
                    var = true;
                    break;
                }
            }
            //if(_b->getChildJoint(i)->getChildNode()->getSkelIndex()>=_writeNumNodes) continue;
            if((!var && _b->getChildJoint(i)->getChildNode()->getNumChildJoints()==0) || _b->getChildJoint(i)->getChildNode()->getSkelIndex()>=_writeNumNodes) continue;
            bool success = exportMayaAnimSegment(outFile0, _first, _last, _b->getChildJoint(i)->getChildNode(), _nodesPrefix, _writeNumNodes, level);
            if (!success) return false;
        }
        return true;
    }

    bool MayaExportMotion::exportMayaAnimSegment2(ofstream &outFile0, int _first, int _last, BodyNode *_b, const string &_nodesPrefix, int _writeNumNodes, int level){
        int dofNum=0;

        int numChildJoints = _b->getNumChildJoints();
        for(int i=0; i<_b->getNumChildJoints(); i++){
            if(_b->getChildJoint(i)->getChildNode()->getSkelIndex()>=_writeNumNodes) numChildJoints--;
        }

        string bodyname = _nodesPrefix;
        bodyname += string(_b->getName());

        vector<Vector3d> eulerAngles;
        vector<Vector3d> transVals;
        for(int fi=_first; fi<=_last; fi++){
            _b->getSkel()->setPose(mDofData->getPoseAtFrame(fi));
            Matrix4d Wl = _b->getLocalTransform();
            Matrix3d Rl = Matrix3d::Zero();
            Vector3d tl = Vector3d::Zero();
            for(int ii=0; ii<3; ii++){
                for(int jj=0; jj<3; jj++) Rl(ii, jj) = Wl(ii, jj);
                tl[ii]=Wl(ii, 3);
            }
            Vector3d ei = utils::rot_conv::matrixToEuler(Rl, MayaExportSkeleton::mRotOrder);
            eulerAngles.push_back(ei);
            transVals.push_back(tl);
        }


        assert(MayaExportSkeleton::mRotOrder==utils::rot_conv::XYZ);	// else change the euler angle order
        // X rotation
        outFile0<<"anim rotate.rotateX rotateX "<<bodyname.c_str()<<" "<<level<<" "<<numChildJoints<<" "<<dofNum++<<endl;
        outFile0<<"animData {\n";
        outFile0<<"\tinput time;\n";
        outFile0<<"\toutput angular;\n";
        outFile0<<"\tkeys {\n";
        for(int i=_first; i<= _last; i++){
            outFile0<<"\t\t "<<i<<" "<<eulerAngles[i-_first][0]<<" linear linear 1 1 0;\n";
        }
        outFile0<<"\t}\n";	// close keys
        outFile0<<"}\n\n";	// close animData

        // Y rotation
        outFile0<<"anim rotate.rotateY rotateY "<<bodyname.c_str()<<" "<<level<<" "<<numChildJoints<<" "<<dofNum++<<endl;
        outFile0<<"animData {\n";
        outFile0<<"\tinput time;\n";
        outFile0<<"\toutput angular;\n";
        outFile0<<"\tkeys {\n";

        for(int i=_first; i<= _last; i++){
            outFile0<<"\t\t "<<i<<" "<<eulerAngles[i-_first][1]<<" linear linear 1 1 0;\n";
        }
        outFile0<<"\t}\n";	// close keys
        outFile0<<"}\n\n";	// close animData

        // Z rotation
        outFile0<<"anim rotate.rotateZ rotateZ "<<bodyname.c_str()<<" "<<level<<" "<<numChildJoints<<" "<<dofNum++<<endl;
        outFile0<<"animData {\n";
        outFile0<<"\tinput time;\n";
        outFile0<<"\toutput angular;\n";
        outFile0<<"\tkeys {\n";

        for(int i=_first; i<= _last; i++){
            outFile0<<"\t\t "<<i<<" "<<eulerAngles[i-_first][2]<<" linear linear 1 1 0;\n";
        }
        outFile0<<"\t}\n";	// close keys
        outFile0<<"}\n\n";	// close animData

        // X translation
        outFile0<<"anim translate.translateX translateX "<<bodyname.c_str()<<" "<<level<<" "<<numChildJoints<<" "<<dofNum++<<endl;
        outFile0<<"animData {\n";
        outFile0<<"\tinput time;\n";
        outFile0<<"\toutput linear;\n";
        outFile0<<"\tkeys {\n";
        for(int i=_first; i<= _last; i++){
            outFile0<<"\t\t "<<i<<" "<<transVals[i-_first][0]<<" linear linear 1 1 0;\n";
        }
        outFile0<<"\t}\n";	// close keys
        outFile0<<"}\n\n";	// close animData

        // Y translation
        outFile0<<"anim translate.translateY translateY "<<bodyname.c_str()<<" "<<level<<" "<<numChildJoints<<" "<<dofNum++<<endl;
        outFile0<<"animData {\n";
        outFile0<<"\tinput time;\n";
        outFile0<<"\toutput linear;\n";
        outFile0<<"\tkeys {\n";

        for(int i=_first; i<= _last; i++){
            outFile0<<"\t\t "<<i<<" "<<transVals[i-_first][1]<<" linear linear 1 1 0;\n";
        }
        outFile0<<"\t}\n";	// close keys
        outFile0<<"}\n\n";	// close animData

        // Z translation
        outFile0<<"anim translate.translateZ translateZ "<<bodyname.c_str()<<" "<<level<<" "<<numChildJoints<<" "<<dofNum++<<endl;
        outFile0<<"animData {\n";
        outFile0<<"\tinput time;\n";
        outFile0<<"\toutput linear;\n";
        outFile0<<"\tkeys {\n";

        for(int i=_first; i<= _last; i++){
            outFile0<<"\t\t "<<i<<" "<<transVals[i-_first][2]<<" linear linear 1 1 0;\n";
        }
        outFile0<<"\t}\n";	// close keys
        outFile0<<"}\n\n";	// close animData

        // write segments for children joints as well
        level++;
        for(int i=0; i<_b->getNumChildJoints(); i++){
            if(_b->getChildJoint(i)->getChildNode()->getSkelIndex()>=_writeNumNodes) continue;
            bool success = exportMayaAnimSegment2(outFile0, _first, _last, _b->getChildJoint(i)->getChildNode(), _nodesPrefix, _writeNumNodes, level);
            if (!success) return false;
        }
        return true;
    }

}   // namespace mayaexports   
