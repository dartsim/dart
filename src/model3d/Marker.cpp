/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#include "Marker.h"
using namespace Eigen;

#include "BodyNode.h"
#include "utils/LoadOpengl.h"


namespace model3d {
    int Marker::msMarkerCount = 0;

    Marker::Marker(const char* _name, Vector3d& _offset, BodyNode *_node, ConstraintType _type)
        : mNode(_node), mOffset(_offset), mType(_type){ 
        mID = Marker::msMarkerCount++;
        strcpy(mName, _name);
    }

    void Marker::draw(bool _offset, const Vector4d& _color, bool _useDefaultColor) const {
        glPushName(getID());
        if(mType==HARD) glColor3f(1, 0, 0);
        else if(mType==SOFT) glColor3f(0, 1, 0);
        else{
            if(_useDefaultColor) glColor3f(0,0,1);
            else glColor4d(_color[0],_color[1],_color[2],_color[3]);
        }

        if(_offset){
            glPushMatrix();
            glTranslatef(mOffset[0], mOffset[1], mOffset[2]);
            glutSolidSphere(0.01,8,8);
            glPopMatrix();
        }
        else{
            glutSolidSphere(0.01,8,8);
        }
        glPopName();
    }

    Vector3d Marker::getWorldCoords() {
        return mNode->evalWorldPos(mOffset);
    }

} // namespace model3d

