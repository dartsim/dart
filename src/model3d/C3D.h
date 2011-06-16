/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#ifndef MODEL3D_C3D_H
#define MODEL3D_C3D_H

#include <vector>
#include <ctime>
#include <Eigen/Dense>
#include "utils/EigenArrayHelper.h"

////////////////////////////////////////////////////////////////////////////////
//  C3D file reader and writer
namespace model3d {

#define C3D_REC_SIZE   512

    typedef struct c3d_head_t {
        unsigned char	prec_start;
        unsigned char	key;
        short	pnt_cnt;
        short	a_channels;
        short	start_frame;
        short	end_frame;
        short	int_gap;
        float	scale;
        short	rec_start;
        short	a_frames;
        float	freq;
        short	stuff[244];	
    } c3d_head;

    typedef struct c3d_param_t {
        unsigned char	reserved[2];
        unsigned char	pblocks;
        unsigned char	ftype;
        char stuff[C3D_REC_SIZE-4];	
    } c3d_param;

    typedef struct c3d_frameSI_t {
        short	x, y, z;
        unsigned char	cam_byte;
        unsigned char	residual;
    } c3d_frameSI;

    typedef struct c3d_frame_t {
        float	x, y, z;
        float	residual;
    } c3d_frame;

    float convertDecToFloat(char _bytes[4]);
    void convertFloatToDec(float _f, char* _bytes);

    bool loadC3DFile( const char *_fileName, EIGEN_VV_VEC3D& _pointData,
                      int *_nFrame, int *_nMarker, double *_freq );
    bool saveC3DFile( const char* _fileName, EIGEN_VV_VEC3D& _pointData,
                      int _nFrame, int _nMarker, double _freq ); 

    double maxElem(std::vector<double>& _arr, int& _index);
    double minElem(std::vector<double>& _arr, int& _index);

} // namespace model3d

#endif // #ifndef MODEL3D_C3D_H


