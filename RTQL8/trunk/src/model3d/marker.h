#ifndef SRC_MODEL3D_MARKER_H
#define SRC_MODEL3D_MARKER_H

#include <Eigen/Dense>
using namespace Eigen;
#include "primitive_ellipsoid.h"

namespace model3d {
#define MAX_MARKER_NAME 256

  class Dof;
  class BodyNode;

  class Marker {
  public:
    enum ConstraintType {
      NO,
      HARD,
      SOFT
    };
  protected:
    BodyNode *mNode;	// body link associated with
    Vector3d mOffset;	// local coordinates in the links
    Ellipsoid mSphere;
    char mName[MAX_MARKER_NAME];
    int mModelIndex;	// position in the model class handle vector
    ConstraintType mType;

  public:
    Marker(char*, Vector3d, BodyNode*, ConstraintType _type = NO);
    ~Marker();

    void draw(bool _offset = true, Vector4d _color, bool _default = true);
    inline VectorXd getWorldCoords();
	
    inline Vector3d getLocalCoords();
    inline Vector3d setLocalCoords(Vector3d _offset);
    inline void setModelIndex(int _idx);
    inline int getModelIndex();
    inline int getID();
    inline BodyNode* getNode();
    inline char* getName();
    // useful for IK
    inline ConstraintType getConstraintType();
    inline void setConstraintType(ConstraintType _type);
  };

} // namespace model3d

#endif // #ifndef SRC_MODEL3D_MARKER_H

