#ifndef OPENGL_CAMERA_H
#define OPENGL_CAMERA_H

namespace renderer
{
	class OpenGLCamera
	{


	public:
		OpenGLCamera() : public Camera {}
		virtual ~OpenGLCamera() {};
		virtual void set (const Vector3d& Eye, const Vector3d& look, const Vector3d& up);
		virtual void slide (double delX, double delY, double delZ, bool bLocal = false);
		virtual void setFrustum (float vAng, float asp, float nearD, float farD);
		virtual void setOrtho(float width, float height, float nearD, float farD);


		virtual void roll (float angle);
		virtual void pitch (float angle);
		virtual void yaw (float angle);
		virtual void localRotate (float angle, AXIS axis);
		virtual void globalRotate (float angle, AXIS axis);

	};
}


#endif
