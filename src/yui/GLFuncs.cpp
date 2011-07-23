/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author    Sumit Jain
  Date      07/21/2011
*/

#include "GLFuncs.h"
#include "utils/LoadOpengl.h"
#include <string>
#include <iostream>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

namespace yui{

    void drawStringOnScreen(float x, float y, const std::string& s)
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
        unsigned int length = s.length();
        for (unsigned int c=0; c < length; c++)
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, s.at(c) );
        glPopMatrix();

        glMatrixMode( GL_PROJECTION );
        glPopMatrix();
        glMatrixMode(oldMode);
    }

    // draw a 3D arrow starting from pt along dir, the arrowhead is on the other end
    void drawArrow3D(const Vector3d& pt, const Vector3d& dir, const double length, const double thickness, const double arrowThickness){
        Vector3d normDir = dir;
        normDir.normalize();

        double arrowLength;
        if(arrowThickness == -1)
            arrowLength = 4*thickness;
        else arrowLength = 2*arrowThickness;

        // draw the arrow body as a cylinder
        GLUquadricObj *c;
        c = gluNewQuadric();
        gluQuadricDrawStyle(c, GLU_FILL);
        gluQuadricNormals(c, GLU_SMOOTH);

        glPushMatrix();
        glTranslatef(pt[0], pt[1], pt[2]);
        glRotated(acos(normDir[2])*180/M_PI, -normDir[1], normDir[0], 0);
        gluCylinder(c, thickness, thickness, length-arrowLength, 16, 16);

        // draw the arrowhed as a cone
        glPushMatrix();
        glTranslatef(0, 0, length-arrowLength);
        gluCylinder(c, arrowLength*0.5, 0.0, arrowLength, 10, 10);
        glPopMatrix();

        glPopMatrix();

        gluDeleteQuadric(c);
    }

    // draw a 2D arrow starting from pt along vec, the arrow head is on the other end
    void drawArrow2D(const Vector2d& pt, const Vector2d& vec, double thickness)
    {
        // draw the arrow body as a thick line
        glLineWidth(thickness);
        glBegin(GL_LINES);
        glVertex2f(pt[0],pt[1]);
        glVertex2f(pt[0]+vec[0],pt[1]+vec[1]);
        glEnd();

        // draw arrowhead as a triangle
        double theta = atan2(vec[1],vec[0]);
        glPushMatrix();
        glTranslatef(pt[0]+vec[0],pt[1]+vec[1],0.0);
        glRotatef(theta*180.0/M_PI,0.0,0.0,1.0);
        glTranslatef(thickness,0.0,0.0);
        glBegin(GL_TRIANGLES);
        glVertex2f(0.0,thickness);
        glVertex2f(2*thickness,0.0);
        glVertex2f(0.0,-thickness);
        glEnd();
        glPopMatrix();
    }

    void drawProgressBar(int currFrame, int totalFrame)
    {
        GLint oldMode;
        glGetIntegerv(GL_MATRIX_MODE, &oldMode);
        glMatrixMode( GL_PROJECTION );

        glPushMatrix();
        glLoadIdentity();
        gluOrtho2D( 0.0, 1.0, 0.0, 1.0 );

        glMatrixMode( GL_MODELVIEW );
        glPushMatrix();
        glLoadIdentity();

        glPolygonMode(GL_FRONT, GL_LINE);
        glColor4d(0.0,0.0,0.0,0.5);
        glBegin(GL_QUADS);
        glVertex2f(0.15f,0.02f);
        glVertex2f(0.85f,0.02f);
        glVertex2f(0.85f,0.08f);
        glVertex2f(0.15f,0.08f);
        glEnd();

        float portion = (float)currFrame/totalFrame;
        float end = 0.15f+portion*0.7f;
        glPolygonMode(GL_FRONT, GL_FILL);
        glColor4d(0.3,0.3,0.3,0.5);
        glBegin(GL_QUADS);
        glVertex2f(0.15f,0.02f);
        glVertex2f(end, 0.02f);
        glVertex2f(end, 0.08f);
        glVertex2f(0.15f,0.08f);
        glEnd();

        glPopMatrix();

        glMatrixMode( GL_PROJECTION );
        glPopMatrix();
        glMatrixMode(oldMode);
    }

    BOOL screenShot(FREE_IMAGE_FORMAT fif, int w, int h, char *fname, bool _antialias){
        return screenShot(fif, 0, 0, w, h, fname, _antialias);
    }

    BOOL screenShot(FREE_IMAGE_FORMAT fif, int x, int y, int w, int h, char *fname, bool _antialias){
        // read the pixels
        int numPixels = w*h;
        unsigned char *pixels = new unsigned char[numPixels*3*sizeof(unsigned char)];
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        if(_antialias) glReadBuffer(GL_ACCUM);
        else glReadBuffer(GL_FRONT);
        glReadPixels(x,y,w,h,GL_BGR_EXT,GL_UNSIGNED_BYTE,pixels);

        FIBITMAP *bmp = FreeImage_ConvertFromRawBits(pixels, w, h, w*3, 24, 0, 0, 0);
        BOOL success = FreeImage_Save(fif, bmp, fname, 0);	// fif is FIF_PNG, FIF_TARGA etc

        FreeImage_Unload(bmp);
        delete []pixels;

        return success;
    }

    bool screenShot(int w, int h, char *fname, bool _antialias) {
        // make sure OpenGL context is current
        //make_current();

        // read the pixels
        int numPixels = w*h;
        unsigned char *pixels = new unsigned char[numPixels*3*sizeof(unsigned char)];
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        if(_antialias) glReadBuffer(GL_ACCUM);
        else glReadBuffer(GL_FRONT);
        glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, pixels);

        // swap red and blue, because TGA format is stupid
        int i;
        for (i=0; i < numPixels; i++) {
            pixels[i * 3 + 0] ^= pixels[i * 3 + 2];
            pixels[i * 3 + 2] ^= pixels[i * 3 + 0];
            pixels[i * 3 + 0] ^= pixels[i * 3 + 2];
        }

        // get file name
        //#if USEFLTK
        //	if (fname == NULL) {
        //		fname = (char *)fltk::file_chooser("L", "*.tga", NULL);
        //	}
        //#endif
        if (fname == NULL)
            return false;

        // open the file
        FILE *fptr;
        fptr = fopen(fname, "wb");
        if (fptr == NULL) {
            //    PrintOnScreen("Failed to open this file");
            return false;
        }

        // create tga header
        putc(0,fptr);
        putc(0,fptr);
        putc(2,fptr);                         // uncompressed RGB
        putc(0,fptr); putc(0,fptr);
        putc(0,fptr); putc(0,fptr);
        putc(0,fptr);
        putc(0,fptr); putc(0,fptr);           // X origin
        putc(0,fptr); putc(0,fptr);           // y origin
        putc((w & 0x00FF),fptr);
        putc((w & 0xFF00) / 256,fptr);
        putc((h & 0x00FF),fptr);
        putc((h & 0xFF00) / 256,fptr);
        putc(24,fptr);                        // 24 bit bitmap
        putc(0,fptr);

        // write the data
        fwrite(pixels, w*h*3*sizeof(char), 1, fptr);
        fclose(fptr);

        delete []pixels;

        cout << fname << " generated" << endl;
        return true;
    }
} // namespace yui
