#ifndef SRC_MODEL3D_MARKER_H
#define SRC_MODEL3D_MARKER_H

#include <Eigen/Dense>
using namespace Eigen;
#include "primitive_ellipsoid.h"
#include "bodynode.h"

namespace model3d {
#define MAX_MARKER_NAME 256

  class Dof;

  class Marker {
  public:
    enum ConstraintType {
      NO,
      HARD,
      SOFT
    };
  protected:
    BodyNode *mNode;	// body link associated with
    VectorXd mOffset;	// local coordinates in the links
    PrimitiveEllipsoid mSphere;
    char mName[MAX_MARKER_NAME];
    int mModelIndex;	// position in the model class handle vector
    ConstraintType mType;

  public:
    Marker(char*, Vector3d, BodyNode*, ConstraintType _type = NO);
    ~Marker();
	/*
    void draw(bool _offset = true, Vector4d _color = Vector4d::Identity(), bool _default = true);
	*/
	inline VectorXd getWorldCoords(){return mNode->evalWorldPos(mOffset);}
	
	
    inline Vector3d getLocalCoords(){return mOffset;}
    inline Vector3d setLocalCoords(Vector3d _offset){mOffset = _offset;}
    inline void setModelIndex(int _idx){mModelIndex=_idx;}
    inline int getModelIndex(){return mModelIndex;}
    inline int getID(){return mSphere.getID();}
    inline BodyNode* getNode(){return mNode;}
    inline char* getName(){return mName;}
    // useful for IK
    inline ConstraintType getConstraintType(){return mType;}
    inline void setConstraintType(ConstraintType _type){mType = _type;}
  };

} // namespace model3d

#endif // #ifndef SRC_MODEL3D_MARKER_H

