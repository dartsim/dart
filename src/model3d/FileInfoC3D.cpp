/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/


#include "FileInfoC3D.h"
#include "C3D.h"

#include <cassert>
using namespace std;

namespace model3d{

    FileInfoC3D::FileInfoC3D()
    {
    }

    FileInfoC3D::~FileInfoC3D()
    {
    }

/*
  void C3DFileInfo::drawConstr(int _frame, bool _default)
  {
  assert( _frame>=0 );
  int idx = _frame;
  if( idx >= mNumFrames )
  idx = mNumFrames-1;
  Ellipsoid e(Vec3d(0.01,0.01,0.01), 0);
  e.setColor( Vec3d(1.0, 0.6, 1.0) );
  for(int i=0; i<mData[idx].size(); i++){
  glPushMatrix();
  glTranslatef(mData[idx][i][0], mData[idx][i][1], mData[idx][i][2]);
  e.draw(mColor, !_default);
  glPopMatrix();
  }
  }
*/

    bool FileInfoC3D::loadFile(const char* _fName)
    {
	if( loadC3DFile( _fName, mData, &mNumFrames, &mNumMarkers, &mFPS) ){
            string text = _fName;
            int lastSlash = text.find_last_of("/");
            text = text.substr(lastSlash+1);
            strcpy(mFileName, text.c_str());
            //createGraphs();
            return true;
	}else{
            return false;
	}
    }

    bool FileInfoC3D::saveFile(const char* _fName, int _start, int _end, double sampleRate)
    {
	// vector< vector<Vector3d> > tmpData = mData;	
	EIGEN_VV_VEC3D tmpData = mData;
        
	int first = _start<mNumFrames?_start:mNumFrames-1;
	int last = _end<mNumFrames?_end:mNumFrames-1;

	tmpData.erase( tmpData.begin()+last+1, tmpData.end()+1);
	tmpData.erase( tmpData.begin(), tmpData.begin()+first); 

	if( saveC3DFile( _fName, tmpData, last-first+1, mData[0].size(), mFPS )){
            string text = _fName;
            int lastSlash = text.find_last_of("/");
            text = text.substr(lastSlash+1);
            strcpy(mFileName, text.c_str());
            return true;
	}else{
            return false;
	}
    }

}
