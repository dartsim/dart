#include "c3d.h"
#include <cstring>
#include <cstdio>
using namespace std;


///////////////////////////////////////////////////////////////////////
//  C3D file reader and writer
///////////////////////////////////////////////////////////////////////

namespace model3d {

  float ConvertDecToFloat(char bytes[4]) 
  {
    char p[4]; 
    p[0] = bytes[2]; 
    p[1] = bytes[3]; 
    p[2] = bytes[0]; 
    p[3] = bytes[1]; 
    if (p[0] || p[1] || p[2] || p[3]) 
      --p[3];          // adjust exponent 
    return *(float*)p; 
  } 


  void ConvertFloatToDec(float f, char* bytes) 
  {
    char* p = (char*)&f; 
    bytes[0] = p[2]; 
    bytes[1] = p[3]; 
    bytes[2] = p[0]; 
    bytes[3] = p[1]; 
    if (bytes[0] || bytes[1] || bytes[2] || bytes[3]) 
      ++bytes[1];      // adjust exponent 
  }


  bool LoadC3DFile( const char *fileName, std::vector< std::vector<Vector3d> > & mPointData, int *nFrame, int *nMarker, double *freq )
  {
    char buf[C3D_REC_SIZE];
    FILE *file;
    int i, j;
    Vector3d v;
    c3d_head hdr;
    c3d_param param;
    c3d_frameSI frameSI;
    c3d_frame frame;
    bool bDecFmt = true;

    if ( (file=fopen( fileName, "rb" )) == NULL )  
      return false; 

    //get the header
    if (!fread(buf, C3D_REC_SIZE, 1, file))
      return false;
    memcpy(&hdr, buf, sizeof(hdr));

    //get number format
    if (hdr.rec_start > 2)
    {
      if (!fread(buf, C3D_REC_SIZE, 1, file))
        return false;
      memcpy(&param, buf, sizeof(param));
      if (param.ftype == 84)
        bDecFmt = false;
    }

    //convert if in dec format
    if (bDecFmt)
    {
      hdr.freq = ConvertDecToFloat((char*)&hdr.freq);
      hdr.scale = ConvertDecToFloat((char*)&hdr.scale);
    }

    int mNumFrames = hdr.end_frame - hdr.start_frame + 1;
    int mNumMarkers = hdr.pnt_cnt;
    double mC3DScale = hdr.scale;

    *freq = hdr.freq;
    *nMarker = mNumMarkers;
    *nFrame = mNumFrames;

    float pntScale = (hdr.scale < 0) ? 1 : hdr.scale;

    //eat parameter records
    for (i = 3; i < hdr.rec_start; i++)
      if (!fread(buf, C3D_REC_SIZE, 1, file))
        return false;

    // start retrieving data
    mPointData.resize(mNumFrames);

    int iRecSize;
    if (mC3DScale < 0)
      iRecSize = sizeof(c3d_frame) + ( hdr.a_channels * hdr.a_frames * sizeof(float));
    else
      iRecSize = sizeof(c3d_frameSI) + ( hdr.a_channels * hdr.a_frames * sizeof(short));

    for( i=0;i<mNumFrames;i++ ){
      mPointData[i].resize(mNumMarkers);
      for( j=0;j<mNumMarkers;j++ )
      {
        if (!fread(buf, iRecSize, 1, file))
          return false;
        if (mC3DScale < 0)
        {
          memcpy(&frame, buf, sizeof(frame));
          v[0] = frame.y / 1000.0;
          v[1] = frame.z / 1000.0;
          v[2] = frame.x / 1000.0;
        }
        else
        {
          memcpy(&frameSI, buf, sizeof(frameSI));
          v[0] = (float)frameSI.y * pntScale / 1000.0;
          v[1] = (float)frameSI.z * pntScale / 1000.0;
          v[2] = (float)frameSI.x * pntScale / 1000.0;
        }
        mPointData[i][j] = v;
      }
    }
    fclose(file);

    const char *pch = strrchr(fileName, '\\');  //clip leading path
    if (pch) fileName = pch+1;

    return true;
  }

  bool SaveC3DFile(const char *fileName, std::vector< std::vector<Vector3d> >& mPointData, int nFrame, int nMarker, double freq)
  {
    FILE* file; 
    int i, j, offset;
    Vector3d v;
    c3d_head hdr;
    c3d_param parm;
    c3d_frame frame;

    if ( (file=fopen( fileName, "wb" )) == NULL )  
      return false; 

    int mrkrCnt = nMarker;

    //write header
    memset(&hdr, 0, sizeof(hdr));
    hdr.prec_start = 1;
    hdr.key = 80;
    hdr.pnt_cnt = mrkrCnt;
    hdr.a_channels = 0;
    hdr.start_frame = 1;
    hdr.end_frame = nFrame;

    //make scale neg to show we are saving floats rather than scaled ints
    hdr.scale = -0.1;
    hdr.rec_start = 3;
    hdr.a_frames = 1;
    hdr.freq = freq;
    fwrite(&hdr, C3D_REC_SIZE, 1, file);

    memset(&parm, 0, sizeof(parm));
    parm.ftype = 84;
    fwrite(&parm, C3D_REC_SIZE, 1, file);

    //write the data
    frame.residual = 0;
    for( i=0;i<nFrame;i++ ) {
      for( j=0;j<nMarker;j++ ) {
        // data is in meters, so put it into millimeters
        v = mPointData[i][j] * 1000.0;
        frame.x = v[2];
        frame.y = v[0];
        frame.z = v[1];
        fwrite(&frame, sizeof(frame), 1, file);
      }
    }

    fclose( file );

    return true;
  }

} // namespace model3d
