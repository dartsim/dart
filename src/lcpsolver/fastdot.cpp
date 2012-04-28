/* generated code, do not edit. */

#include "matrix.h"


dReal _dDot (const dReal *a, const dReal *b, int n)
{  
  dReal p0,q0,m0,p1,q1,m1,sum;
  sum = 0;
  n -= 2;
  while (n >= 0) {
    p0 = a[0]; q0 = b[0];
    m0 = p0 * q0;
    p1 = a[1]; q1 = b[1];
    m1 = p1 * q1;
    sum += m0;
    sum += m1;
    a += 2;
    b += 2;
    n -= 2;
  }
  n += 2;
  while (n > 0) {
    sum += (*a) * (*b);
    a++;
    b++;
    n--;
  }
  return sum;
}


#undef dDot

dReal dDot (const dReal *a, const dReal *b, int n)
{
  return _dDot (a, b, n);
}

