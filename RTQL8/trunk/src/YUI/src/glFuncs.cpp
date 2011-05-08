#include <gl/glut.h>
#include <string>
#include <Eigen/Eigen>

using namespace Eigen;

void drawStringOnScreen(float x, float y, std::string s)
{ // draws text on the screen
	GLint oldMode;
	glGetIntegerv(GL_MATRIX_MODE, &oldMode);
	glMatrixMode( GL_PROJECTION );

	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D( 0.0, 1.0, 0.0, 1.0 );

	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();
	glLoadIdentity();
	glRasterPos2f(x, y); 
	for (unsigned int c=0; c < s.length(); c++)
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, s.at(c) );
	glPopMatrix();

	glMatrixMode( GL_PROJECTION );
	glPopMatrix();
	glMatrixMode(oldMode);
}

void drawArrow(const Vector3d& pt, const Vector3d& dir, const double length, const double thickness, const double arrowThickness){
	Vector3d normDir = dir;
	normDir.normalize();

	double arrowLength;
	if(arrowThickness == -1)
		arrowLength = 4*thickness;
	else arrowLength = 2*arrowThickness;

	GLUquadricObj *c;
	c = gluNewQuadric();
	gluQuadricDrawStyle(c, GLU_FILL);
	gluQuadricNormals(c, GLU_SMOOTH);
	
	glPushMatrix();
	glTranslatef(pt[0], pt[1], pt[2]);
	glRotated(acos(normDir[2])*180/M_PI, -normDir[1], normDir[0], 0);
	gluCylinder(c, thickness, thickness, length-arrowLength, 16, 16);

	// arrowhed
	glPushMatrix();
	glTranslatef(0, 0, length-arrowLength);
	gluCylinder(c, arrowLength*0.5, 0.0, arrowLength, 10, 10);
	glPopMatrix();

	glPopMatrix();

	gluDeleteQuadric(c);
}
