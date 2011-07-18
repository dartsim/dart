/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#include "Marker.h"
using namespace Eigen;

#include "BodyNode.h"


namespace model3d {
    int Marker::msMarkerCount = 0;

    Marker::Marker(const char* _name, Vector3d& _offset, BodyNode *_node, ConstraintType _type)
        : mNode(_node), mOffset(_offset), mType(_type){ 
        mID = Marker::msMarkerCount++;
        strcpy(mName, _name);
    }

    void Marker::draw(renderer::RenderInterface* _ri, bool _offset, const Vector4d& _color, bool _useDefaultColor) const {
		if (!_ri) return;
		_ri->pushName(getID());
		if(mType==HARD) _ri->setPenColor(Vector3d(1, 0, 0));
		else if(mType==SOFT) _ri->setPenColor(Vector3d(0, 1, 0));
		else {
			if(_useDefaultColor) _ri->setPenColor(Vector3d(0,0,1));
			else _ri->setPenColor(Vector4d(_color[0],_color[1],_color[2], _color[3]));
		}
		if(_offset){
			_ri->pushMatrix();
			_ri->translate(mOffset);
			_ri->drawEllipsoid(Vector3d(0.01, 0.01, 0.01));
			_ri->popMatrix();
		}
		else{
			_ri->drawEllipsoid(Vector3d(0.01, 0.01, 0.01));
		}

		_ri->popName();

    }

    Vector3d Marker::getWorldCoords() {
        return mNode->evalWorldPos(mOffset);
    }

} // namespace model3d

