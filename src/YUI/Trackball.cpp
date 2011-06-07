#include "Trackball.h"
#include "utils/LoadOpengl.h"

using namespace Eigen;

void Trackball::startBall(double x, double y)
{
	mStartPos = mouseOnSphere(x,y);
}

void Trackball::updateBall(double x, double y)
{
	Vector3d toPos = mouseOnSphere(x,y);
	Quaterniond newQuat(quatFromVectors(mStartPos, toPos));
	mStartPos = toPos;
	mCurrQuat = newQuat*mCurrQuat;
}

void Trackball::applyGLRotation()
{
	Transform<double, 3, Affine> t(mCurrQuat);
	glMultMatrixd(t.data());
}

void Trackball::draw(int winWidth, int winHeight)
{
	glDisable( GL_LIGHTING );
	glDisable( GL_TEXTURE_2D );

	glPushMatrix();
	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glViewport( 0, 0, winWidth, winHeight );
	gluOrtho2D(0.0, (GLdouble)winWidth, 0.0, (GLdouble)winHeight);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	
	glLineWidth(4.0);
	glColor3f( 1.0f, 1.0f, 0.0f );
	glBegin( GL_LINE_LOOP);		
	for(int i = 0; i < 360; i+=4){
		double theta = i / 180.0 * M_PI;
		double x = mRadius * cos(theta);
		double y = mRadius * sin(theta);
		glVertex2d( (GLdouble)((winWidth >> 1) + x), (GLdouble)((winHeight >> 1) + y));
	}	
	glEnd();

	glPopMatrix();
}

Vector3d Trackball::mouseOnSphere(double mouseX, double mouseY) const
{   
	double mag;
	Vector3d pointOnSphere;

	pointOnSphere(0) = (mouseX - mCenter(0)) / mRadius;
	pointOnSphere(1) = (mouseY - mCenter(1)) / mRadius;

	mag = pointOnSphere(0) * pointOnSphere(0) + pointOnSphere(1)*pointOnSphere(1);
	if (mag > 1.0) 
	{	
		register double scale = 1.0/sqrt(mag);
		pointOnSphere(0) *= scale; 
		pointOnSphere(1) *= scale;
		pointOnSphere(2) = 0.0;
	} 
	else 
		pointOnSphere(2) = sqrt(1 - mag);

	return pointOnSphere;
}

Quaterniond Trackball::quatFromVectors(const Vector3d& from, const Vector3d& to) const
{
	Quaterniond quat;
	quat.x() = from(1)*to(2) - from(2)*to(1);
	quat.y() = from(2)*to(0) - from(0)*to(2);
	quat.z() = from(0)*to(1) - from(1)*to(0);
	quat.w() = from(0)*to(0) + from(1)*to(1) + from(2)*to(2);
	return quat;
}
