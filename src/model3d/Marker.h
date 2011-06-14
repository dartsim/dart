/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/


#ifndef MODEL3D_MARKER_H
#define MODEL3D_MARKER_H

#include <Eigen/Dense>

namespace Renderer {
    class OpenGLRenderInterface;
} // namespace Renderer

namespace model3d {
#define MAX_MARKER_NAME 256

    class Dof;
    class BodyNode;
    class PrimitiveEllipsoid;

    class Marker {
    public:
        enum ConstraintType {
            NO,
            HARD,
            SOFT
        };

    public:
        Marker(char*, Eigen::Vector3d& , BodyNode*, ConstraintType _type = NO);
        virtual ~Marker();

        void draw(Renderer::OpenGLRenderInterface* RI, bool _offset = true, const Eigen::Vector4d& _color = Eigen::Vector4d::Identity(), bool _default = true);

        /* Eigen::VectorXd getWorldCoords(){return mNode->evalWorldPos(mOffset);} */
        Eigen::Vector3d getWorldCoords();
	
	
        Eigen::Vector3d getLocalCoords(){return mOffset;}
        void setLocalCoords(Eigen::Vector3d& _offset){mOffset = _offset;}
        void setModelIndex(int _idx){mModelIndex=_idx;}
        int getModelIndex(){return mModelIndex;}
        int getID();
        BodyNode* getNode(){return mNode;}
        char* getName(){return mName;}
        // useful for IK
        ConstraintType getConstraintType(){return mType;}
        void setConstraintType(ConstraintType _type){mType = _type;}

    protected:
        BodyNode* mNode;	// body link associated with
        Eigen::Vector3d mOffset;	// local coordinates in the links
        PrimitiveEllipsoid* mSphere;
        char mName[MAX_MARKER_NAME];
        int mModelIndex;	// position in the model class handle vector
        ConstraintType mType;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            };

} // namespace model3d

#endif // #ifndef MODEL3D_MARKER_H

