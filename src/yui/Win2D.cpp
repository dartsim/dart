/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author    Sumit Jain
  Date      07/21/2011
*/
#include <iostream>
#include "Win2D.h"

namespace yui {
    Win2D::Win2D():GlutWindow()
    {
        mTransX = 0;
        mTransY = 0;
        mTranslate = false;
    }

    void Win2D::resize(int w, int h)
    {
        mWinWidth = w;
        mWinHeight = h;

        glViewport( 0, 0, w, h );
        glMatrixMode( GL_PROJECTION );
        glLoadIdentity();
        glOrtho( -w/2, w/2-1, -h/2, h/2-1, -1, 1 );

        glMatrixMode(GL_MODELVIEW); // Select The Modelview Matrix 
        glLoadIdentity(); // Reset The Modelview Matrix 

        glutPostRedisplay();
    }

    void Win2D::keyboard(unsigned char key, int x, int y)
    {
        switch(key){
        case ',': // slow down
            mDisplayTimeout +=2;
            break;
        case '.': // speed up
            mDisplayTimeout -= 2;
            if( mDisplayTimeout <1 )
                mDisplayTimeout = 1;
            break;
        case 'c':
        case 'C': // screen capture
            mCapture = !mCapture;
            break;
        case 27: //ESC
            exit(0);
        }

        glutPostRedisplay();
        //printf("ascii key: %lu\n", key);
    }

    void Win2D::click(int button, int state, int x, int y)
    {
        mMouseDown = !mMouseDown;
        if(mMouseDown){
            mTranslate = true;

            mMouseX = x;
            mMouseY = y;
        }else{
            mTranslate = false;
        }
        glutPostRedisplay();
    }

    void Win2D::drag(int x, int y)
    {
        if(mMouseDown){
            mTransX += (x - mMouseX);
            mTransY += (y - mMouseY);

            mMouseX = x;
            mMouseY = y;
        }
        glutPostRedisplay();
    }

    void Win2D::render()
    {
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho( -mWinWidth/2, mWinWidth/2-1, -mWinHeight/2, mWinHeight/2-1, -1, 1 );

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        initGL();

        // transformation
        glTranslatef(mTransX,-mTransY,0.0);	
        draw();

        // draw axis
        // translate back to the center
        glTranslatef(-mTransX, mTransY, 0.0);
        if(mTranslate){
            glLineWidth(2.0);

            glColor3f( 1.0f, 0.0f, 0.0f );
            glBegin( GL_LINES );
            glVertex2f( -40.f, 0.0f );
            glVertex2f( 40.f, 0.0f );
            glEnd();		

            glColor3f( 0.0f, 1.0f, 0.0f );
            glBegin( GL_LINES );
            glVertex2f( 0.0f, -40.f );
            glVertex2f( 0.0f, 40.f );
            glEnd();
        }

        if(mCapture)
            screenshot();

        glutSwapBuffers();
    }

    void Win2D::initGL()
    {
        glClearColor(mBackground[0],mBackground[1],mBackground[2],mBackground[3]);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_BLEND);
        glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
        glEnable(GL_POINT_SMOOTH);
        glEnable(GL_LINE_SMOOTH);
    }

}   // namespace yui
