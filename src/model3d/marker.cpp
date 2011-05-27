#include "marker.h"
#ifndef _RENDERER_TEST
#include <gl/glut.h>
#endif



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


    void Marker::draw(Renderer::OpenGLRenderInterface* RI, bool _offset, const Vector4d& _color, bool _default)
	{
#ifdef _RENDERER_TEST
		if (!RI) return;
		RI->PushName(mSphere.getID());
		if(mType==NO) mSphere.setColor(Vector3d(0, 0, 1));
		if(mType==HARD) mSphere.setColor(Vector3d(1, 0, 0));
		if(mType==SOFT) mSphere.setColor(Vector3d(0, 1, 0));

		if(_offset){
			RI->PushMatrix();
			RI->Translate(mOffset);
			mSphere.draw(RI, _color, _default);
			RI->PopMatrix();
		}
		else{
			mSphere.draw(RI, _color, _default);
		}

		RI->PopName();
#else
		glPushName(mSphere.getID());
		if(mType==NO) mSphere.setColor(Vector3d(0, 0, 1));
		if(mType==HARD) mSphere.setColor(Vector3d(1, 0, 0));
		if(mType==SOFT) mSphere.setColor(Vector3d(0, 1, 0));

		if(_offset){
			glPushMatrix();
			glTranslatef(mOffset[0], mOffset[1], mOffset[2]);
			mSphere.draw(RI, _color, _default);
			glPopMatrix();
		}
		else{
			mSphere.draw(RI, _color, _default);
		}
		glPopName();
#endif
    }


} // namespace model3d

