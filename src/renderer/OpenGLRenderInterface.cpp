#include "OpenGLRenderInterface.h"
#include "utils/LoadOpengl.h"

namespace Renderer
{
	void OpenGLRenderInterface::Initialize()
	{
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glCullFace(GL_FRONT);
		glDisable(GL_LIGHTING);
		glEnable(GL_DEPTH_TEST);
		//glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
		glShadeModel(GL_SMOOTH);
		Clear(Vector3d(1.0, 1.0, 1.0));
	}
	void OpenGLRenderInterface::Destroy()
	{

	}

	void OpenGLRenderInterface::SetViewport(int X,int Y,int Width,int Height)
	{
		glViewport(X, Y, Width, Height);
		mViewportX = X;
		mViewportY = Y;
		mViewportWidth = Width;
		mViewportHeight = Height;

	}
	void OpenGLRenderInterface::GetViewport(int& X, int& Y, int &Width, int& Height) const
	{
		X = mViewportX;
		Y = mViewportY;
		Width = mViewportWidth;
		Height =mViewportHeight;
	}

	void OpenGLRenderInterface::Clear(const Vector3d& color)
	{
		glClearColor((GLfloat)color[0], (GLfloat)color[1], (GLfloat)color[2], 1.0);
		glClear(GL_COLOR_BUFFER_BIT);
	}

	void OpenGLRenderInterface::SetDefaultLight()
	{

	}

	void OpenGLRenderInterface::TurnOffLights()
	{
		glDisable(GL_LIGHTING);
	}
	void OpenGLRenderInterface::TurnOnLights()
	{
//not finished yet
		glEnable(GL_LIGHTING);	
	}
	void OpenGLRenderInterface::SetPenColor(const Vector4d& col)
	{
		glColor4d(col[0], col[1], col[2], col[3]);
	}
	void OpenGLRenderInterface::SetPenColor(const Vector3d& col)
	{
		glColor4d(col[0], col[1], col[2], 1.0);
	}
	void OpenGLRenderInterface::DrawCube(const Vector3d& size)
	{
		glScaled(size(0), size(1), size(2));
		glutSolidCube(1.0);
	}
	void OpenGLRenderInterface::DrawEllipsoid(const Vector3d& size)
	{
		glScaled(size(0), size(1), size(2));
		glutSolidSphere(0.5, 16, 16);
	}
	void OpenGLRenderInterface::PushMatrix()
	{
		glPushMatrix();
	}
	void OpenGLRenderInterface::PopMatrix()
	{
		glPopMatrix();
	}
	void OpenGLRenderInterface::PushName(int id)
	{
		glPushName(id);
	}
	void OpenGLRenderInterface::PopName()
	{
		glPopName();
	}
	void OpenGLRenderInterface::Translate(const Vector3d& offset)
	{
		glTranslated(offset[0], offset[1], offset[2]);
	}
	
	void OpenGLRenderInterface::Scale(const Vector3d& scale)
	{
		glScaled(scale[0], scale[1], scale[2]);
	}
	void OpenGLRenderInterface::Rotate(const Vector3d& axis, float rad)
	{
		glRotated(rad, axis[0], axis[1], axis[2]);
	}

	void OpenGLRenderInterface::SaveToImage(const char * filename, DecoBufferType buffType)
	{

	}
	void OpenGLRenderInterface::ReadFrameBuffer(DecoBufferType buffType, DecoColorChannel ch, void* pixels)
	{

	}
	void OpenGLRenderInterface::SetMaterial(const Vector3d& diffuse, const Vector3d& specular, float cosinePow)
	{

	}
	void OpenGLRenderInterface::GetMaterial(Vector3d& diffuse, Vector3d& specular, float& cosinePow) const
	{

	}
	void OpenGLRenderInterface::SetDefaultMaterial()
	{

	}
}
