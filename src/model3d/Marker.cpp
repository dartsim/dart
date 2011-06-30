/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#include "Marker.h"
using namespace Eigen;

#include "BodyNode.h"

#ifndef _RENDER_TEST
#include "utils/LoadOpengl.h"
#endif

namespace model3d {
    int Marker::msMarkerCount = 0;

    Marker::Marker(const char* _name, Vector3d& _offset, BodyNode *_node, ConstraintType _type)
        : mNode(_node), mOffset(_offset), mType(_type){ 
        mID = Marker::msMarkerCount++;
        strcpy(mName, _name);
    }

    void Marker::draw(renderer::RenderInterface* RI, bool _offset, const Vector4d& _color, bool _useDefaultColor) const {
#ifdef _RENDER_TEST
		if (!RI) return;
		RI->PushName(getID());
		if(mType==HARD) RI->SetPenColor(Vector3d(1, 0, 0));
		else if(mType==SOFT) RI->SetPenColor(Vector3d(0, 1, 0));
		else {
			if(_useDefaultColor) RI->SetPenColor(Vector3d(0,0,1));
			else RI->SetPenColor(Vector4d(_color[0],_color[1],_color[2], _color[3]));
		}
		if(_offset){
			RI->PushMatrix();
			RI->Translate(mOffset);
			RI->DrawEllipsoid(Vector3d(0.01, 0.01, 0.01));
			RI->PopMatrix();
		}
		else{
			RI->DrawEllipsoid(Vector3d(0.01, 0.01, 0.01));
		}

		RI->PopName();
#else
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
#endif
    }

    Vector3d Marker::getWorldCoords() {
        return mNode->evalWorldPos(mOffset);
    }

} // namespace model3d

