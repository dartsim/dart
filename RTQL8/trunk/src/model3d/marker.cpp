#include "marker.h"



namespace model3d {
  Marker::Marker(char* _name, Vector3d& _offset, BodyNode *_node, ConstraintType _type):mSphere(Vector3d(0.01,0.01,0.01), 0)
  { 
    mNode = _node;
    strcpy(mName, _name);
    mOffset = _offset;
    mType = _type;
  }

  Marker::~Marker() {
  }

  VectorXd Marker::getWorldCoords() {
    return mNode->evalWorldPos(mOffset);
  }

  /*
    void Marker::draw(bool _offset, Vector4d _color, bool _default){
    glPushName(mSphere.getID());
    if(mType==NO) mSphere.setColor(Vector3d(0, 0, 1));
    if(mType==HARD) mSphere.setColor(Vector3d(1, 0, 0));
    if(mType==SOFT) mSphere.setColor(Vector3d(0, 1, 0));

    if(_offset){
    glPushMatrix();
    glTranslatef(mOffset[0], mOffset[1], mOffset[2]);
    mSphere.draw(_color, _default);
    glPopMatrix();
    }
    else{
    mSphere.draw(_color, _default);
    }
    glPopName();
    }
  */

} // namespace model3d

