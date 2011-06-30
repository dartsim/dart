/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/


#ifndef MODEL3D_MARKER_H
#define MODEL3D_MARKER_H

#include <Eigen/Dense>
#include "renderer/RenderInterface.h"

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
        Marker(const char* _name, Eigen::Vector3d& , BodyNode*, ConstraintType _type = NO);
        virtual ~Marker(){}

		void draw(renderer::RenderInterface* RI = NULL, bool _offset = true, const Eigen::Vector4d& _color = Eigen::Vector4d::Identity(), bool _useDefaultColor = true) const;

        Eigen::Vector3d getWorldCoords(); ///< get the world coordinates of mOffset
	
        Eigen::Vector3d getLocalCoords() const {return mOffset;}
        void setLocalCoords(Eigen::Vector3d& _offset){mOffset = _offset;}

        int getModelIndex() const{return mModelIndex;}
        void setModelIndex(int _idx){mModelIndex=_idx;}
        
        int getID() const {return mID;}
        BodyNode* getNode() const {return mNode;}
        const char* getName() const {return mName;}
        
        // useful for IK
        ConstraintType getConstraintType() const {return mType;}
        void setConstraintType(ConstraintType _type){mType = _type;}
    
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    protected:
        BodyNode* mNode;	///< body link associated with
        Eigen::Vector3d mOffset;	///< local coordinates in the links
        char mName[MAX_MARKER_NAME]; ///< name of this marker, max length 256 characters
        int mModelIndex;	///< position in the model class handle vector
        ConstraintType mType; ///< type of constraint
    
    private:
        int mID; ///< a unique ID of this marker globally
        static int msMarkerCount; ///< counts the number of markers globally
    };

} // namespace model3d

#endif // #ifndef MODEL3D_MARKER_H

