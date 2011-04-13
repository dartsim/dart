#ifndef SRC_UTILS_UTILS_H
#define SRC_UTILS_UTILS_H

// Standard Libraries
#include <vector>
#include <cmath>
#include <ctime>
#include <climits>
#include <cassert>
using namespace std;
// External Libraries
#include <Eigen/Dense>
using namespace Eigen;
// Local Headers
#include "misc.h"

namespace utils {

  // a cross b = (CR*a) dot b
  /* const Matd CR(2,2,0.0,-1.0,1.0,0.0); */
  const Matrix2d CR( (Matrix2d() << 0.0, -1.0, 1.0, 0.0).finished() );

  inline int delta(int i, int j) {
    if(i == j) return 1;
    return 0;
  }

  inline int sgn(double a){
    if(a<0) return -1;
    else if(a==0) return 0;
    else return 1;
  }

  inline double sqr(double x){
    return x*x;
  }

  inline double asinh(double X){
    return log(X + sqrt(X * X + 1));
  }

  inline double acosh(double X){
    return log(X + sqrt(X * X - 1));
  }

  inline double atanh(double X){
    return log((1 + X)/(1 - X))/ 2;
  }

  inline double asech(double X){
    return log((sqrt(-X * X + 1) + 1) / X);
  }

  inline double acosech(double X){
    return log((sgn(X) * sqrt(X * X + 1) +1) / X);
  }

  inline double acotanh(double X){
    return log((X + 1) / (X - 1)) / 2;
  }

  inline double round(double x){
    return floor(x+0.5);
  }

  inline double round2(double x){
    int gintx = (int)floor(x);
    if(x-gintx<0.5) return (double)gintx;
    else return (double)(gintx+1);
  }

  inline bool isEqual(double x, double y){
    return (fabs(x - y) < EPSILON);
  }
	
  // check if it is an integer
  inline bool isInt(double x){
    if(isEqual(round(x), x)) return true;
    else return false;
  }

  inline double random( double min, double max ){
    return min + ((double)rand()/(RAND_MAX + 1.0)) * (max-min);
  }

  inline unsigned seed_rand(){
    time_t now = time(0);
    unsigned char *p = (unsigned char *)&now;
    unsigned seed = 0;
    size_t i;

    for(i=0; i<sizeof(now); i++)
      seed = seed * (UCHAR_MAX + 2U ) + p[i];

    srand( seed );
    return seed;
  }

  inline VectorXd transform(const MatrixXd& M, const VectorXd& v) {
    int n = v.size();
    assert(M.rows() == n + 1);
    VectorXd augV(n + 1);
    augV.head(n) = v;
    augV(n) = 1.0;

    augV = M * augV;
    VectorXd ret = augV.head(n);
    return ret;
  }

  /* inline VectorXd transform(const VectorXd& v, const MatrixXd& M){ */
  /*   int n = v.size(); */
  /*   assert(M.rows() == n + 1); */
    
  /*   VectorXd augV(n + 1); */
  /*   augV.head(n) = v; */
  /*   augV = augV * M; */
  /*   VectorXd ret = augV.head(n); */
  /*   return ret; */
  /* } */

/*   /\* void drawArrow(Vecd p, Vecd force){ *\/ */
/*   /\*   Vecd begin = p+force*5; *\/ */
		
/*   /\*   glColor3f(1.0,0.0,0.0); *\/ */

/*   /\*   glLineWidth(5.0); *\/ */
/*   /\*   glBegin(GL_LINES); *\/ */
/*   /\*   glVertex2f(begin[0],begin[1]); *\/ */
/*   /\*   glVertex2f(p[0],p[1]); *\/ */
/*   /\*   glEnd(); *\/ */

/*   /\*   // arrow head *\/ */
/*   /\*   double theta = atan2(force[1],force[0]); *\/ */
/*   /\*   glPushMatrix(); *\/ */
/*   /\*   glTranslatef(begin[0],begin[1],0.0); *\/ */
/*   /\*   glRotatef(theta*180.0/M_PI,0.0,0.0,1.0); *\/ */
/*   /\*   glTranslatef(-5.0,0.0,0.0); *\/ */
/*   /\*   glBegin(GL_TRIANGLES); *\/ */
/*   /\*   glVertex2f(0.0,5.0); *\/ */
/*   /\*   glVertex2f(10,0.0); *\/ */
/*   /\*   glVertex2f(0.0,-5.0); *\/ */
/*   /\*   glEnd(); *\/ */
/*   /\*   glPopMatrix(); *\/ */
/*   /\* } *\/ */

/*   inline Matd makeSkewSymmetric(Vecd v){ */
/*     Matd result(3,3,vl_0); */
		
/*     result.Elt(0,1) = -v[2]; */
/*     result.Elt(1,0) = v[2]; */
/*     result.Elt(0,2) = v[1]; */
/*     result.Elt(2,0) = -v[1]; */
/*     result.Elt(1,2) = -v[0]; */
/*     result.Elt(2,1) = v[0]; */

/*     return result; */
/*   } */

/*   inline Vecd cross(Vecd a, Vecd b){ */
/*     Vec3d v1(a[0],a[1],a[2]); */
/*     Vec3d v2(b[0],b[1],b[2]); */
/*     Vec3d v = cross(v1,v2); */
/*     return Vecd(3,v[0],v[1],v[2]); */
/*   } */
	
/*   // *p*artial *S*kew *S*ymmetric matrix / *p*artial *E*lement{1,2,3} */

/*   // frist element */
/*   //  0  0  0 */
/*   //  0  0 -1 */
/*   //  0  1  0 */
/*   const Matd pSSpE1(3,3,0.0,0.0,0.0,0.0,0.0,-1.0,0.0,1.0,0.0); */
	
/*   // second element */
/*   //  0  0  1 */
/*   //  0  0  0 */
/*   // -1  0  0 */
/*   const Matd pSSpE2(3,3,0.0,0.0,1.0,0.0,0.0,0.0,-1.0,0.0,0.0); */

/*   // third element */
/*   //  0 -1  0 */
/*   //  1  0  0 */
/*   //  0  0  0 */
/*   const Matd pSSpE3(3,3,0.0,-1.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0); */

/*   inline Vecd fromSkewSymmetric(const Matd& m) */
/*   { */
/*     if (fabs(m[0][0]) > EPSILON || fabs(m[1][1]) > EPSILON || fabs(m[2][2]) > EPSILON) */
/*     { */
/*       printf("Not skew symmetric matrix"); */
/*       cout<<m<<endl; */
/*       exit(0); */
/*     } */
/*     Vecd ret(3,vl_0); */
/*     Vecd unitY(3,0.0, 1.0, 0.0); */
/*     Vecd xz = m * unitY; */
/*     ret[0] = xz[2]; */
/*     ret[2] = -xz[0]; */
/*     Vecd unitZ(3,0.0, 0.0, 1.0); */
/*     Vecd y = m * unitZ; */
/*     ret[1] = y[0]; */
/*     return ret; */
/*   } */

/*   inline double scale_to_bound(double v, double lower_bound, double upper_bound) { */
/*     return (v - lower_bound) / (upper_bound - lower_bound); */
/*   } */

/*   inline double scale_from_bound(double w, double lower_bound, double upper_bound) { */
/*     return (1.0 - w) * lower_bound + w * upper_bound; */
/*   } */
  
} // namespace utils

#endif // #ifndef SRC_UTILS_UTILS_H


