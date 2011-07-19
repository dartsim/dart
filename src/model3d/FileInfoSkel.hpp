/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/


#ifndef MODEL3D_FILEINFO_SKEL_H
#define MODEL3D_FILEINFO_SKEL_H

#include <fstream>
#include <iomanip>
#include <string>
#include <iostream>
#include <sstream>

#include <Eigen/Dense>

#include "renderer/RenderInterface.h"
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

int readSkelFile( const char* const filename, model3d::Skeleton* skel );

namespace model3d {
    
    class BodyNode;
    class Skeleton;

    enum SkeletonFileType {
        VSK,
        SKEL
    };

    template <class SkeletonType>
    class FileInfoSkel {
    public:
  
    public:
        FileInfoSkel();
        virtual ~FileInfoSkel();
	
        void draw(renderer::RenderInterface* _ri = NULL, const Eigen::Vector4d& _color=Eigen::Vector4d::Ones(), bool _useDefaultColor = true) const; ///< Note: _color is not used when _useDefaultColor=false
        bool loadFile(const char* _filename, SkeletonFileType _type);
        bool saveFile(const char* _filename) const;
        
        SkeletonType* getSkel() const { return mSkel; }

    protected:
        SkeletonType* mSkel;
        char mFileName[256]; // could use string instead

        // used in saveFile to write the subtree of _b
        void saveBodyNodeTree(BodyNode* _b, std::ofstream &_outfile, int _numLinks) const;
    };

    template <class SkeletonType>
    FileInfoSkel<SkeletonType>::FileInfoSkel() : mSkel(NULL){
    }

    template <class SkeletonType>
    FileInfoSkel<SkeletonType>::~FileInfoSkel(){
        if(mSkel){ 
            delete mSkel;
            mSkel = NULL;
        }
    }

    template <class SkeletonType>
    void FileInfoSkel<SkeletonType>::draw(renderer::RenderInterface* _ri, const Vector4d& _color, bool _useDefaultColor) const {
        mSkel->draw(_ri, _color, _useDefaultColor);
    }

    template <class SkeletonType>
    bool FileInfoSkel<SkeletonType>::loadFile( const char* _fName, SkeletonFileType _type ) {
        if( mSkel )
            delete mSkel;
        // mSkel = new Skeleton;
        mSkel = new SkeletonType;

        bool success = false;
        if(_type == SKEL)
            success = !readSkelFile( _fName, mSkel);
        else if(_type == VSK)
            success = (readVSKFile(_fName, mSkel) == VSK_OK);
        if(success){	
            std::string text = _fName;
            int lastSlash = text.find_last_of("/");
            text = text.substr(lastSlash+1);
            strcpy(mFileName, text.c_str());
        }
        return success;
    }

    const static std::string unitlength = "unitlength";

    template <class SkeletonType>
    bool FileInfoSkel<SkeletonType>::saveFile( const char* _fName) const {
        using namespace std;
        ofstream output(_fName);
        if(output.fail()){
            cout<<"Unable to save file "<<_fName<<endl;
            return false;
        }
        output<<fixed<<setprecision(6);

        int _numLinks = mSkel->nNodes;

        output<<"dofs {\n";
        for(int i=0; i<mSkel->getNumDofs(); i++){
            Dof *d = mSkel->getDof(i);
            if(d->getJoint()->getNodeOut()->getSkelIndex() >= _numLinks) break;
            output<<'\t'<<d->getName()<<" { "<<d->getValue()<<", "<<d->getMin()<<", "<<d->getMax()<<" }\n";
        }
        output<<unitlength<<" { 1.0, 1.0, 1.0 }\n";
        output<<"}\n";

        // masses
        output<<"\nmass {\n";
        for(int i=0; i<mSkel->nNodes; i++){
            if(i>=_numLinks) break;
            string massname = mSkel->getNode(i)->getName();
            massname += "_mass";
            output<<massname<<" { "<<mSkel->getNode(i)->getMass()<<" }\n";

        }
        output<<"}\n";

        // nodes
        saveBodyNodeTree(mSkel->mRoot, output, _numLinks);

        // markers
        output<<"\nhandles {\n";
        for(int i=0; i<mSkel->getNumHandles(); i++){
            if(mSkel->getHandle(i)->getNode()->getSkelIndex()>=_numLinks) break;
            string hname = mSkel->getHandle(i)->getName();
            if(hname.empty()){
                ostringstream ss;
                ss<<"handle"<<i;
                hname = ss.str();
            }
            Vector3d lc = mSkel->getHandle(i)->getLocalCoords();
            output<<hname<<" { "<<"<"<<lc[0]<<", "<<lc[1]<<", "<<lc[2]<<">, "<<i<<", "<<mSkel->getHandle(i)->getNode()->getName()<<" } "<<endl;

        }
        output<<"}\n";

        cout<<"\n";

        return true;
    }

    template <class SkeletonType>
    void FileInfoSkel<SkeletonType>::saveBodyNodeTree(BodyNode *_b, std::ofstream &_outfile, int _numLinks) const {
        // save the current one
        _outfile<<"\nnode "<<_b->getName()<<" { "<<_b->getSkelIndex()<<"\n";
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

        _outfile<<", "<<std::string(_b->getName())+std::string("_mass");
        _outfile<<" }\n";

        for(int i=0; i<_b->getNumJoints(); i++){
            if(_b->getNodeOut(i)->getSkelIndex()>=_numLinks) continue;
            saveBodyNodeTree(_b->getNodeOut(i), _outfile, _numLinks);
        }

        _outfile<<"}\n";	//node

    }


} // namespace model3d

#endif // #ifndef MODEL3D_FILEINFO_SKEL_H

