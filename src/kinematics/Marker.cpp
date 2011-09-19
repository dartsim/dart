/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
 * Date: 06/12/2011
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "Marker.h"
using namespace Eigen;

#include "BodyNode.h"


namespace kinematics {
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

} // namespace kinematics

