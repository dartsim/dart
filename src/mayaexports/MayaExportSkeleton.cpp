/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sumit Jain
  Date		07/24/2011
*/
#include "MayaExportSkeleton.h"
#include "model3d/Dof.h"
#include "model3d/BodyNode.h"
#include "model3d/Joint.h"
#include "model3d/Skeleton.h"
#include "model3d/Primitive.h"
#include "model3d/Transformation.h"
#include "utils/RotationConversion.h"
#include <iostream>

using namespace std;
using namespace model3d;
using namespace Eigen;

namespace mayaexports{

    const string MayaExportSkeleton::mSuffixPrim = "_prim";
    const string MayaExportSkeleton::mSuffixShape = "_shape";
    const string MayaExportSkeleton::mSuffixGeom = "_geom";
    const utils::rot_conv::RotationOrder MayaExportSkeleton::mRotOrder = utils::rot_conv::XYZ;


    bool MayaExportSkeleton::exportMayaAscii( Skeleton* _skel, ofstream &_outFile ){
        // maya .ma file header
        exportMayaAsciiHeader(_outFile);

        // hierarchy tree - export in dfs fashion
        //exportMayaAsciiSegmentSubtree(_skel->getRoot(), _skel->getNumNodes(), _outFile, "", "");
        exportMayaAsciiSegmentSubtree2(_skel->getRoot(), _skel->getNumNodes(), _outFile, "", "");

        // geometries
        for(int i=0; i<_skel->getNumNodes(); i++){
            BodyNode *b = _skel->getNode(i);
            exportMayaAsciiPrimitive(b, _outFile, "", "");
        }

        // footers - connect stuff
        for(int i=0; i<_skel->getNumNodes(); i++){
            exportMayaAsciiSegmentFooter(_skel->getNode(i), _skel->getNumNodes(), _outFile, "", "");
        }

        return true;
    }

    bool MayaExportSkeleton::exportMayaAsciiHeader(ofstream &_outFile ){
        // headers
        _outFile<<"//Maya ASCII 2011 scene"<<endl;
        _outFile<<"requires maya \"2011\";"<<endl;
        _outFile<<"currentUnit -l meter -a radian -t film;"<<endl;
        _outFile<<"fileInfo \"application\" \"maya\";"<<endl;
        return true;
    }

    bool MayaExportSkeleton::exportMayaAsciiSegmentFooter(BodyNode* _b, const int _writeNumNodes, ofstream &_outFile, const string &_prefix, const string &_suffix  ){
        // connections to children
        for(int j=0; j<_b->getNumChildJoints(); j++){
            if(_b->getChildJoint(j)->getChildNode()->getSkelIndex()>=_writeNumNodes) continue;
            _outFile<<"connectAttr \"";
            _outFile<<_prefix<<_b->getName()<<_suffix<<".s\"";
            _outFile<<" \"";
            _outFile<<_prefix<<_b->getChildJoint(j)->getChildNode()->getName()<<_suffix<<".is\";"<<endl;
        }
        // geometry connectors
        string nshape = _prefix+_b->getName()+_suffix; nshape+=mSuffixShape; 
        string ngeom = _prefix+_b->getName()+_suffix; ngeom+=mSuffixGeom; 
        _outFile<<"connectAttr \""<<ngeom.c_str()<<".out\" \""<<nshape.c_str()<<".i\";"<<endl;

        // shading group
        _outFile<<"connectAttr \""<<nshape.c_str()<<".iog\" \":initialShadingGroup.dsm\" -na;"<<endl;

        return true;
    }

    bool MayaExportSkeleton::exportMayaAsciiPrimitive(BodyNode* _b, ofstream &_outFile, const string &_prefix, const string &_suffix  ){
        Primitive *prim = _b->getPrimitive();
        Vector3d dim = prim->getDim();
        string ngeom = _prefix+_b->getName()+_suffix; ngeom+=mSuffixGeom; 

        // transform
        string name1 = _prefix+_b->getName()+_suffix; name1+=mSuffixPrim;
        string name2 = _prefix+_b->getName()+_suffix; name2+=mSuffixShape;
        _outFile<<"createNode transform -n \""<<name1.c_str()<<"\" -p \""<<_prefix+_b->getName()+_suffix<<"\";"<<endl;
        Vector3d off = _b->getLocalCOM(); //prim->getLocalCOM();
        //if(prim->getPrimitiveType()==Primitive::P_CYLINDER) {
        //    GeomCylinder *cyl = static_cast<GeomCylinder*>(prim->getGeom());
        //    Vector3d ax = cyl->getOrientation();
        //    off = off + 0.5*dim[1]*ax;	// to offset opengl and maya coordinate origin
        //}
        _outFile<<"\t setAttr \".t\" -type \"double3\" "<<off[0]<<" "<<off[1]<<" "<<off[2]<<";"<<endl;
        // shape 
        if(prim->getPrimitiveType()==Primitive::P_ELLIPSOID) {
            _outFile<<"\t setAttr \".s\" -type \"double3\" "<<1.0<<" "<<dim[1]/dim[0]<<" "<<dim[2]/dim[0]<<";"<<endl;
        }
        _outFile<<"createNode mesh -n \""<<name2.c_str()<<"\" -p \""<<name1.c_str()<<"\";"<<endl;

        // geom
        switch (prim->getPrimitiveType()){
            case Primitive::P_ELLIPSOID:
                _outFile<<"createNode polySphere -n \""<<ngeom.c_str()<<"\";"<<endl;
                _outFile<<"\t setAttr \".r\" "<<dim[0]/2<<";"<<endl;	// radius
                _outFile<<"\t setAttr \".sa\" 20;"<<endl;	// subdivision axis
                _outFile<<"\t setAttr \".sh\" 20;"<<endl;	// subdivision height
                break;
            case Primitive::P_CUBE:
                _outFile<<"createNode polyCube -n \""<<ngeom.c_str()<<"\";"<<endl;
                _outFile<<"\t setAttr \".w\" "<<dim[0]<<";"<<endl;	// width
                _outFile<<"\t setAttr \".h\" "<<dim[1]<<";"<<endl;	// height
                _outFile<<"\t setAttr \".d\" "<<dim[2]<<";"<<endl;	// depth
                break;
            //case Primitive::P_SPHERE:
            //    _outFile<<"createNode polySphere -n \""<<ngeom.c_str()<<"\";"<<endl;
            //    _outFile<<"\t setAttr \".r\" "<<dim[0]/2<<";"<<endl;	// radius
            //    _outFile<<"\t setAttr \".sa\" 20;"<<endl;	// subdivision axis
            //    _outFile<<"\t setAttr \".sh\" 20;"<<endl;	// subdivision height
            //    break;
            //case Primitive::P_CYLINDER:
            //    {
            //        GeomCylinder *cyl = static_cast<GeomCylinder*>(prim->getGeom());
            //        Vector3d ax = cyl->getOrientation();
            //        _outFile<<"createNode polyCylinder -n \""<<ngeom.c_str()<<"\";"<<endl;
            //        _outFile<<"\t setAttr \".r\" "<<dim[0]/2<<";"<<endl;	// radius
            //        _outFile<<"\t setAttr \".h\" "<<dim[1]<<";"<<endl;	// height
            //        _outFile<<"\t setAttr \".sh\" 20;"<<endl;	// subdivision height
            //        _outFile<<"\t setAttr \".sc\" 20;"<<endl;	// subdivision cap
            //        _outFile<<"\t setAttr \".rcp\" yes;"<<endl;	// rounded caps: change val to geom mDrawCaps
            //        _outFile<<"\t setAttr \".ax\" -type \"double3\" "<<ax[0]<<" "<<ax[1]<<" "<<ax[2]<<";"<<endl;
            //    }
            //    break;
            //case Primitive::P_HEAD:
            //    cout<<"placeholder sphere for head\n";
            //    _outFile<<"createNode polySphere -n \""<<ngeom.c_str()<<"\";"<<endl;
            //    _outFile<<"\t setAttr \".r\" "<<dim[0]/2<<";"<<endl;	// radius
            //    _outFile<<"\t setAttr \".sa\" 20;"<<endl;	// subdivision axis
            //    _outFile<<"\t setAttr \".sh\" 20;"<<endl;	// subdivision height
            //    break;
            case Primitive::P_UNDEFINED:
                cout<<"Primitive type undefined for body "<<_b->getName()<<endl;
                return false;
                break;
        }
        return true;
    }

    bool MayaExportSkeleton::exportMayaAsciiSegmentSubtree(BodyNode *_b, const int _writeNumNodes, ofstream &_outFile, const string &_prefix, const string &_suffix ){
        if(_b->getSkelIndex()>= _writeNumNodes) return true;

        // create the node
        _outFile<<"createNode joint -n \""<<_prefix+_b->getName()+_suffix<<"\"";
        if(_b->getParentJoint()->getParentNode()) _outFile<<" -p \""<<_prefix+_b->getParentJoint()->getParentNode()->getName()+_suffix<<"\"";
        _outFile<<";"<<endl;

        // write the attributes
        _outFile<<"\t addAttr -ci true -sn \"liw\" -ln \"lockInfluenceWeights\" -bt \"lock\" -min 0 -max 1 -at \"bool\";"<<endl;
        Vector3d tdofs = Vector3d::Zero();
        for(int i=0; i<_b->getParentJoint()->getNumTransforms(); i++){
            Transformation *tf = _b->getParentJoint()->getTransform(i);
            // since the joint cannot store translation dofs, this has to be added together for dofs as well
            //if(!tf->getVariable() && tf->getType()==Transformation::T_TRANSLATE){
            if(tf->getType()==Transformation::T_TRANSLATE){
                tdofs[0]+=tf->getDof(0)->getValue();
                tdofs[1]+=tf->getDof(1)->getValue();
                tdofs[2]+=tf->getDof(2)->getValue();
            }
        }
        if(tdofs!=Vector3d::Zero()) _outFile<<"\t setAttr \".t\" -type \"double3\" "<<tdofs[0]<<" "<<tdofs[1]<<" "<<tdofs[2]<<";\n";

        // write out the rotation order (expmap "xyz" order: see anim export also)
        string rot = "";
        int ro = -1;
        vector<int> varIndex;	// indices of variable rot dofs
        for(int i=_b->getParentJoint()->getNumTransforms()-1; i>=0; i--){
            Transformation *tf = _b->getParentJoint()->getTransform(i);
            if(!tf->getVariable()) continue;	// joint order should be determined only by variable dofs
            if(tf->getType()==Transformation::T_ROTATEX) {
                rot += "x";
                varIndex.push_back(i);
            }
            if(tf->getType()==Transformation::T_ROTATEY) {
                rot += "y";
                varIndex.push_back(i);
            }
            if(tf->getType()==Transformation::T_ROTATEZ) {
                rot += "z";
                varIndex.push_back(i);
            }
            if(tf->getType()==Transformation::T_ROTATEEXPMAP) {
                rot = "xyz";
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


        if(rot.compare("xyz")==0 || rot.compare("xy")==0 || rot.compare("yz")==0 || rot.compare("xz")==0 || rot.compare("x")==0 || rot.compare("y")==0 || rot.compare("z")==0) ro=0;
        if(rot.compare("yzx")==0 ) ro=1;
        if(rot.compare("zxy")==0 ) ro=2;
        if(rot.compare("xzy")==0 ) ro=3;
        if(rot.compare("yxz")==0 ) ro=4;
        if(rot.compare("zyx")==0 || rot.compare("zy")==0|| rot.compare("yx")==0|| rot.compare("zx")==0 ) ro=5;

        if(rotException) ro=0;

        if(ro!=-1) _outFile<<"\t setAttr \".ro\" "<<ro<<";"<<endl;

        // collect all constant rotations and write out in default "xyz" order
        // collect all variable rotations and write out in ".ro" order
        Matrix3d rotdofs = Matrix3d::Identity();
        Matrix3d rotcons = Matrix3d::Identity();
        if(rotException){
            for(int i=_b->getParentJoint()->getNumTransforms()-1; i>=0; i--){
                Transformation *tf = _b->getParentJoint()->getTransform(i);
                Matrix3d *rotPtr = NULL;
                if(i>=separateRots) rotPtr = &rotdofs; else rotPtr = &rotcons;
                if(tf->getType()==Transformation::T_ROTATEX) *rotPtr = utils::rot_conv::eulerToMatrixX(tf->getDof(0)->getValue())*(*rotPtr);
                if(tf->getType()==Transformation::T_ROTATEY) *rotPtr = utils::rot_conv::eulerToMatrixY(tf->getDof(0)->getValue())*(*rotPtr);
                if(tf->getType()==Transformation::T_ROTATEZ) *rotPtr = utils::rot_conv::eulerToMatrixZ(tf->getDof(0)->getValue())*(*rotPtr);
                if(tf->getType()==Transformation::T_ROTATEEXPMAP) {
                    Vector3d exmap(tf->getDof(0)->getValue(), tf->getDof(1)->getValue(), tf->getDof(2)->getValue());
                    Quaterniond q = utils::rot_conv::expToQuat(exmap);
                    *rotPtr = utils::rot_conv::quatToMatrix(q)*(*rotPtr);
                }
            }
        }
        else {
            for(int i=_b->getParentJoint()->getNumTransforms()-1; i>=0; i--){
                Transformation *tf = _b->getParentJoint()->getTransform(i);
                //if(tf->getVariable()) continue;	// only constant dofs should contribute to the ".jo" attribute; rest to ".r"
                Matrix3d *rotPtr = NULL;
                if(tf->getVariable()) rotPtr = &rotdofs; else rotPtr = &rotcons;
                if(tf->getType()==Transformation::T_ROTATEX) *rotPtr = utils::rot_conv::eulerToMatrixX(tf->getDof(0)->getValue())*(*rotPtr);
                if(tf->getType()==Transformation::T_ROTATEY) *rotPtr = utils::rot_conv::eulerToMatrixY(tf->getDof(0)->getValue())*(*rotPtr);
                if(tf->getType()==Transformation::T_ROTATEZ) *rotPtr = utils::rot_conv::eulerToMatrixZ(tf->getDof(0)->getValue())*(*rotPtr);
                if(tf->getType()==Transformation::T_ROTATEEXPMAP) {
                    Vector3d exmap(tf->getDof(0)->getValue(), tf->getDof(1)->getValue(), tf->getDof(2)->getValue());
                    Quaterniond q = utils::rot_conv::expToQuat(exmap);
                    *rotPtr = utils::rot_conv::quatToMatrix(q)*(*rotPtr);
                }
            }
        }
        // variable dofs: ".r"
        if(ro!=-1){
            utils::rot_conv::RotationOrder rotorder;	// should match the above
            if(ro==0) rotorder=utils::rot_conv::XYZ;
            else if(ro==1) rotorder=utils::rot_conv::YZX;
            else if(ro==2) rotorder=utils::rot_conv::ZXY;
            else if(ro==3) rotorder=utils::rot_conv::XZY;
            else if(ro==4) rotorder=utils::rot_conv::YXZ;
            else if(ro==5) rotorder=utils::rot_conv::ZYX;
            Vector3d edofs= utils::rot_conv::matrixToEuler(rotdofs, rotorder)*180/M_PI;	// convert to degrees
            if(rotdofs != Matrix3d::Identity()) {
                // write in the correct order
                double ax, ay, az;
                // x
                if(ro==0 || ro==3) ax = edofs[0];
                else if(ro==2 || ro==4) ax = edofs[1];
                else if(ro==1 || ro==5) ax = edofs[2];
                // y
                if(ro==1 || ro==4) ay = edofs[0];
                else if(ro==0 || ro==5) ay = edofs[1];
                else if(ro==2 || ro==3) ay = edofs[2];
                // z
                if(ro==2 || ro==5) az = edofs[0];
                else if(ro==1 || ro==3) az = edofs[1];
                else if(ro==0 || ro==4) az = edofs[2];
                _outFile<<"\t setAttr \".r\" -type \"double3\" "<<ax<<" "<<ay<<" "<<az<<";"<<endl;
            }
        }

        // constant dofs: ".jo"
        Vector3d econs = utils::rot_conv::matrixToEuler(rotcons, utils::rot_conv::XYZ)*180/M_PI;	// convert to degrees
        if(rotcons!=Matrix3d::Identity()) _outFile<<"\t setAttr \".jo\" -type \"double3\" "<<econs[0]<<" "<<econs[1]<<" "<<econs[2]<<";"<<endl;

        // repeat the same for children
        for(int i=0; i<_b->getNumChildJoints(); i++){
            exportMayaAsciiSegmentSubtree(_b->getChildJoint(i)->getChildNode(), _writeNumNodes, _outFile, _prefix, _suffix);
        }
        return true;
    }

    bool MayaExportSkeleton::exportMayaAsciiSegmentSubtree2(BodyNode *_b, const int _writeNumNodes, ofstream &_outFile, const string &_prefix, const string &_suffix ){
        if(_b->getSkelIndex()>= _writeNumNodes) return true;

        // create the node
        _outFile<<"createNode joint -n \""<<_prefix+_b->getName()+_suffix<<"\"";
        if(_b->getParentJoint()->getParentNode()) _outFile<<" -p \""<<_prefix+_b->getParentJoint()->getParentNode()->getName()+_suffix<<"\"";
        _outFile<<";"<<endl;

        // write the attributes
        _outFile<<"\t addAttr -ci true -sn \"liw\" -ln \"lockInfluenceWeights\" -bt \"lock\" -min 0 -max 1 -at \"bool\";"<<endl;

        int ro=-1;
        if(mRotOrder==utils::rot_conv::XYZ) ro=0;
        else if(mRotOrder==utils::rot_conv::YZX) ro=1;
        else if(mRotOrder==utils::rot_conv::ZXY) ro=2;
        else if(mRotOrder==utils::rot_conv::XZY) ro=3;
        else if(mRotOrder==utils::rot_conv::YXZ) ro=4;
        else if(mRotOrder==utils::rot_conv::ZYX) ro=5;

        if(ro!=-1) _outFile<<"\t setAttr \".ro\" "<<ro<<";"<<endl;

        // repeat the same for children
        for(int i=0; i<_b->getNumChildJoints(); i++){
            exportMayaAsciiSegmentSubtree2(_b->getChildJoint(i)->getChildNode(), _writeNumNodes, _outFile, _prefix, _suffix);
        }
        return true;
    }

}   // namespace mayaexports
