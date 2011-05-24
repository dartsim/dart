#ifndef SRC_MODEL3D_C3D_H
#define SRC_MODEL3D_C3D_H

#include <vector>
#include <ctime>
using namespace std;
#include <Eigen/Dense>
using namespace Eigen;

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

  float convertDecToFloat(char bytes[4]);
  void convertFloatToDec(float f, char* bytes);

  bool loadC3DFile( const char *fileName, vector< vector<Vector3d> >& mPointData,
                      int *nFrame, int *nMarker, double *freq );
  bool saveC3DFile( const char* fileName, vector< vector<Vector3d> >& mPointData,
                      int nFrame, int nMarker, double freq ); 

  double maxElem(vector<double>& arr, int& index);
  double minElem(vector<double>& arr, int& index);

} // namespace model3d

#endif // #ifndef SRC_MODEL3D_C3D_H


