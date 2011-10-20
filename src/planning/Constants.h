#ifndef	CONSTANTS_H
#define	CONSTANTS_H

#include <math.h>
#include <float.h>

#define double_MAX		DBL_MAX
#define EPS				(1E-6)
#define ZERO			(0.0)
#define HALF			(0.5)
#define ONE				(1.0)
#define TWO				(2.0)
#define	PI_HALF			(1.57079632679489661923)	//< $\frac{\pi}{2}
#define PI				(3.14159265358979323846)	//< $\pi$
#define TWO_PI			(6.28318530717958647693)	//< $ 2 \times \pi $
#define PI_SQRT2		(2.22144146907918312351)	//< $\frac {pi}{\sqrt{2}}$
#define PI_SQR			(9.86960440108935861883)	//< $\pi^2$
#define ONETHIRD		(0.33333333333333333333)	//< $\frac{1}{3}$
#define ONESIXTH		(0.16666666666666666667)	//< $\frac{1}{6}$
#define FOURTHIRD		(1.33333333333333333333)	//< $\frac{4}{3}$
#define RADIAN			(0.01745329251994329577)	//< pi / 180
#define DEGREE			(57.2957795130823208768)	//< 180 / pi

inline double DEG2RAD(int d)	 { return (d * RADIAN); }
inline double RAD2DEG(int r)	 { return (r * DEGREE); }
inline double DEG2RAD(double d)  { return (d * RADIAN); }
inline double RAD2DEG(double r)	 { return (r * DEGREE); }

#endif
