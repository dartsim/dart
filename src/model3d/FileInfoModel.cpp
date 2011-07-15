/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#include "FileInfoModel.h"

#include <iomanip>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

#include "Skeleton.h"
#include "BodyNode.h"
#include "ParserVsk.h"
#include "Marker.h"
#include "Dof.h"
#include "Joint.h"
#include "Marker.h"
#include "Transformation.h"
#include "PrimitiveEllipsoid.h"
#include "PrimitiveCube.h"

using namespace std;
using namespace Eigen;

int readSkelFile( const char* const filename, model3d::Skeleton* skel );

namespace model3d{

    FileInfoModel::FileInfoModel() : mModel(NULL){
    }

    FileInfoModel::~FileInfoModel(){
        if(mModel){ 
            delete mModel;
            mModel = NULL;
        }
    }

	void FileInfoModel::draw(renderer::RenderInterface* _ri, const Vector4d& _color, bool _useDefaultColor) const {
        mModel->draw(_ri, _color, _useDefaultColor);
    }

    bool FileInfoModel::loadFile( const char* _fName, FileType _type ) {
        if( mModel )
            delete mModel;
        mModel = new Skeleton;
        bool success = false;
        if(_type == SKEL)
            success = !readSkelFile( _fName, mModel);
        else if(_type == VSK)
            success = (readVSKFile(_fName, mModel) == VSK_OK);
        if(success){	
            string text = _fName;
            int lastSlash = text.find_last_of("/");
            text = text.substr(lastSlash+1);
            strcpy(mFileName, text.c_str());
        }
        return success;
    }

    const string unitlength = "unitlength";

    bool FileInfoModel::saveFile( const char* _fName) const {
        ofstream output(_fName);
        if(output.fail()){
            cout<<"Unable to save file "<<_fName<<endl;
            return false;
        }
        output<<fixed<<setprecision(6);

        int _numLinks = mModel->nNodes;

        output<<"dofs {\n";
        for(int i=0; i<mModel->getNumDofs(); i++){
            Dof *d = mModel->getDof(i);
            if(d->getJoint()->getNodeOut()->getModelIndex() >= _numLinks) break;
            output<<'\t'<<d->getName()<<" { "<<d->getValue()<<", "<<d->getMin()<<", "<<d->getMax()<<" }\n";
        }
        output<<unitlength<<" { 1.0, 1.0, 1.0 }\n";
        output<<"}\n";

        // masses
        output<<"\nmass {\n";
        for(int i=0; i<mModel->nNodes; i++){
            if(i>=_numLinks) break;
            string massname = mModel->getNode(i)->getName();
            massname += "_mass";
            output<<massname<<" { "<<mModel->getNode(i)->getMass()<<" }\n";

        }
        output<<"}\n";

        // nodes
        saveBodyNodeTree(mModel->mRoot, output, _numLinks);

        // markers
        output<<"\nhandles {\n";
        for(int i=0; i<mModel->getNumHandles(); i++){
            if(mModel->getHandle(i)->getNode()->getModelIndex()>=_numLinks) break;
            string hname = mModel->getHandle(i)->getName();
            if(hname.empty()){
                ostringstream ss;
                ss<<"handle"<<i;
                hname = ss.str();
            }
            Vector3d lc = mModel->getHandle(i)->getLocalCoords();
            output<<hname<<" { "<<"<"<<lc[0]<<", "<<lc[1]<<", "<<lc[2]<<">, "<<i<<", "<<mModel->getHandle(i)->getNode()->getName()<<" } "<<endl;

        }
        output<<"}\n";

        cout<<"\n";

        return true;
    }

    void FileInfoModel::saveBodyNodeTree(BodyNode *_b, ofstream &_outfile, int _numLinks) const {
        // save the current one
        _outfile<<"\nnode "<<_b->getName()<<" { "<<_b->getModelIndex()<<"\n";
        // write the trans
        _outfile<<"chain { "<<_b->getJointIn()->getNumTransforms()<<"\n";
        for(int i=0; i<_b->getJointIn()->getNumTransforms(); i++){
            Transformation *tr = _b->getJointIn()->getTransform(i);
            if(!tr->getVariable()){	// constant
                if(tr->getType()==Transformation::T_TRANSLATE){
                    _outfile<<"telescope { <"<<tr->getDof(0)->getValue()<<", "<<tr->getDof(1)->getValue()<<", "<<tr->getDof(2)->getValue()<<">, "<<unitlength<<" }\n";
                }
                else if (tr->getType()==Transformation::T_ROTATEX){
                    _outfile<<"rotate_cons { "<<tr->getDof(0)->getValue()<<", "<<" x }\n";
                }
                else if (tr->getType()==Transformation::T_ROTATEY){
                    _outfile<<"rotate_cons { "<<tr->getDof(0)->getValue()<<", "<<" y }\n";
                }
                else if (tr->getType()==Transformation::T_ROTATEZ){
                    _outfile<<"rotate_cons { "<<tr->getDof(0)->getValue()<<", "<<" z }\n";
                }
                else {
                    _outfile<<"unknown trans\n";
                }

            }
            else {	// variable
                if(tr->getType()==Transformation::T_TRANSLATE){
                    _outfile<<"translate { <"<<tr->getDof(0)->getName()<<", "<<tr->getDof(1)->getName()<<", "<<tr->getDof(2)->getName()<<"> }\n";
                }
                else if (tr->getType()==Transformation::T_ROTATEX){
                    _outfile<<"rotate_euler { "<<tr->getDof(0)->getName()<<", "<<" x }\n";
                }
                else if (tr->getType()==Transformation::T_ROTATEY){
                    _outfile<<"rotate_euler { "<<tr->getDof(0)->getName()<<", "<<" y }\n";
                }
                else if (tr->getType()==Transformation::T_ROTATEZ){
                    _outfile<<"rotate_euler { "<<tr->getDof(0)->getName()<<", "<<" z }\n";
                }
                else if (tr->getType()==Transformation::T_ROTATEEXPMAP){
                    _outfile<<"rotate_expmap { <"<<tr->getDof(0)->getName()<<", "<<tr->getDof(1)->getName()<<", "<<tr->getDof(2)->getName()<<"> }\n";
                }
                else {
                    _outfile<<"unknown trans\n";
                }
            }
        }
        _outfile<<"}\n";	// chain

        // primitive
        Vector3d pdim = _b->getPrimitive()->getDim();
        Vector3d off = _b->getOffset();
        _outfile<<"primitive { <"<<pdim[0]<<", "<<pdim[1]<<", "<<pdim[2]<<">, <"<<off[0]<<", "<<off[1]<<", "<<off[2]<<">, "<<unitlength;
        // different types

        Primitive* prim = _b->getPrimitive();
        PrimitiveEllipsoid* elp = dynamic_cast<PrimitiveEllipsoid*>(prim);
        PrimitiveCube* box = dynamic_cast<PrimitiveCube*>(prim);

        if(elp) _outfile<<", SPHERE";
        else if(box) _outfile<<", CUBE";

        _outfile<<", "<<string(_b->getName())+string("_mass");
        _outfile<<" }\n";

        for(int i=0; i<_b->getNumJoints(); i++){
            if(_b->getNodeOut(i)->getModelIndex()>=_numLinks) continue;
            saveBodyNodeTree(_b->getNodeOut(i), _outfile, _numLinks);
        }

        _outfile<<"}\n";	//node

    }
}
