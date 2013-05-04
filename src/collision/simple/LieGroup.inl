//////////////////////////////////////////////////////////////////////////////////
//
//		title		:	LieGroup.inl
//						
//		version		:	v0.986
//		author		:	Jinwook Kim (jwkim@imrc.kist.re.kr)
//		last update	:	2004.3.9
//
//////////////////////////////////////////////////////////////////////////////////


inline Vec2::Vec2()
{
}

inline Vec2::Vec2(int d)
{
    _v[0] = _v[1] = (double)d;
}

inline Vec2::Vec2(double d)
{
	_v[0] = _v[1] = d;
}

inline Vec2::Vec2(const double v[])
{
	_v[0] = v[0];
	_v[1] = v[1];
}

inline Vec2::Vec2(double v0, double v1)
{
	_v[0] = v0;
	_v[1] = v1;
}

inline const Vec2 &Vec2::operator + (void) const
{
	return *this;
}

inline Vec2 Vec2::operator - (void) const
{
	return Vec2(-_v[0], -_v[1]);
}

inline double &Vec2::operator [] (int i)
{
	return _v[i];
}

inline const double &Vec2::operator [] (int i) const
{
	return _v[i];
}

inline const Vec2 &Vec2::operator = (const Vec2 &v)
{
	_v[0] = v[0];
	_v[1] = v[1];
	return *this;
}

inline const Vec2 &Vec2::operator += (const Vec2 &v)
{
	_v[0] += v[0];
	_v[1] += v[1];
	return *this;
}

inline const Vec2 &Vec2::operator -= (const Vec2 &v)
{
	_v[0] -= v[0];
	_v[1] -= v[1];
	return *this;
}

inline const Vec2 &Vec2::operator *= (double d)
{
	_v[0] *= d;
	_v[1] *= d;
	return *this;
}

inline const Vec2 &Vec2::operator /= (double d)
{
	_v[0] /= d;
	_v[1] /= d;
	return *this;
}

inline Vec2 Vec2::operator * (double d) const
{
	return Vec2(d * _v[0], d * _v[1]);
}

inline Vec2 Vec2::operator / (double d) const
{
	if ( d == 0.0) {
		return Vec2(0.0);
	}
	return Vec2(_v[0]/d, _v[1]/d);
}

inline Vec2 Vec2::operator + (const Vec2 &v) const
{
	return Vec2(_v[0] + v[0], _v[1] + v[1]);
}

inline Vec2 Vec2::operator - (const Vec2 &v) const
{
	return Vec2(_v[0] - v[0], _v[1] - v[1]);
}

inline double Vec2::Normalize(void)
{
    double mag = sqrt(_v[0] * _v[0] + _v[1] * _v[1]);
	if ( mag < SR_EPS )	// make a unit vector in z-direction
	{
		_v[0] = 0.0;
		_v[1] = 0.0;
	} else
	{
		_v[0] /= mag;
		_v[1] /= mag;
	}
	return mag;
}

inline void Vec2::SetValues(double v0, double v1)
{
	_v[0] = v0;
	_v[1] = v1;
}

inline Vec2 operator * (double d, const Vec2 &v)
{
	return Vec2(d * v[0], d * v[1]);
}

inline double Norm(const Vec2 &v)
{
	return sqrt(v[0] * v[0] + v[1] * v[1]);
}

inline Vec2 Normalize(const Vec2 &v)
{
    double mag = sqrt(v[0] * v[0] + v[1] * v[1]);
	if ( mag < SR_EPS )	// make a unit vector in z-direction
		return Vec2(0.0, 0.0);

	mag = 1.0 / mag;
	return Vec2(mag * v[0], mag * v[1]);
}

inline Vec3 Cross(const Vec2 &p, const Vec2 &q)
{
	return Vec3(0.0, 0.0, p[0] * q[1] - p[1] * q[0]);
}

inline double Inner(const Vec2 &p, const Vec2 &q)
{
	return (p[0] * q[0] + p[1] * q[1]);
}

inline double Cosine(const Vec2 &p, const Vec2 &q)
{
	return (Inner(p,q)/(Norm(p)*Norm(q)));
}

inline double Sine(const Vec2 &p, const Vec2 &q)
{
	return sqrt(1-Cosine(p,q)*Cosine(p,q));
}

inline double SquareSum(const Vec2 &p)
{
	return (p[0] * p[0] + p[1] * p[1]);
}






inline Vec3::Vec3()
{
}

inline Vec3::Vec3(int d)
{
    _v[0] = _v[1] = _v[2] = (double)d;
}

inline Vec3::Vec3(double d)
{
	_v[0] = _v[1] = _v[2] = d;
}

inline Vec3::Vec3(const double v[])
{
	_v[0] = v[0];
	_v[1] = v[1];
	_v[2] = v[2];
}

inline Vec3::Vec3(double v0, double v1, double v2)
{
	_v[0] = v0;
	_v[1] = v1;
	_v[2] = v2;
}

inline const Vec3 &Vec3::operator + (void) const
{
	return *this;
}

inline Vec3 Vec3::operator - (void) const
{
	return Vec3(-_v[0], -_v[1], -_v[2]);
}

inline double &Vec3::operator [] (int i)
{
	return _v[i];
}

inline const double &Vec3::operator [] (int i) const
{
	return _v[i];
}

inline const Vec3 &Vec3::operator = (const Vec3 &v)
{
	_v[0] = v[0];
	_v[1] = v[1];
	_v[2] = v[2];
	return *this;
}

inline const Vec3 &Vec3::operator = (const SE3 &T)
{
	_v[0] = T[9];
	_v[1] = T[10];
	_v[2] = T[11];
	return *this;
}

inline const Vec3 &Vec3::operator = (double d)
{
	_v[0] = _v[1] = _v[2] = d;
	return *this;
}

inline const Vec3 &Vec3::operator += (const Vec3 &v)
{
	_v[0] += v[0];
	_v[1] += v[1];
	_v[2] += v[2];
	return *this;
}

inline const Vec3 &Vec3::operator -= (const Vec3 &v)
{
	_v[0] -= v[0];
	_v[1] -= v[1];
	_v[2] -= v[2];
	return *this;
}

inline const Vec3 &Vec3::operator *= (double d)
{
	_v[0] *= d;
	_v[1] *= d;
	_v[2] *= d;
	return *this;
}

inline const Vec3 &Vec3::operator /= (double d)
{
	_v[0] /= d;
	_v[1] /= d;
	_v[2] /= d;
	return *this;
}

inline Vec3 Vec3::operator * (double d) const
{
	return Vec3(d * _v[0], d * _v[1], d * _v[2]);
}

inline Vec3 Vec3::operator / (double d) const
{
	if ( d == 0.0) {
		return Vec3(0.0);
	}
	return Vec3(_v[0]/d, _v[1]/d, _v[2]/d);
}

inline Vec3 Vec3::operator + (const Vec3 &v) const
{
	return Vec3(_v[0] + v[0], _v[1] + v[1], _v[2] + v[2]);
}

inline Vec3 Vec3::operator - (const Vec3 &v) const
{
	return Vec3(_v[0] - v[0], _v[1] - v[1], _v[2] - v[2]);
}

inline double Vec3::Normalize(void)
{
    double mag = sqrt(_v[0] * _v[0] + _v[1] * _v[1] + _v[2] * _v[2]);
	if ( mag < SR_EPS )	// make a unit vector in z-direction
	{
		_v[0] = 0.0;
		_v[1] = 0.0;
		_v[2] = 0.0;
		//_v[2] = 1.0;
	} else
	{
		_v[0] /= mag;
		_v[1] /= mag;
		_v[2] /= mag;
	}
	return mag;
}

inline void Vec3::SetValues(double v0, double v1, double v2)
{
	_v[0] = v0;
	_v[1] = v1;
	_v[2] = v2;
}

inline void Vec3::Clone(const Vec3& rhs)
{
	_v[0] = rhs[0];
	_v[1] = rhs[1];
	_v[2] = rhs[2];
}

inline Vec3 Rotate(const SE3 &T, const Vec3 &v)
{
	return Vec3(T[0] * v[0] + T[3] * v[1] + T[6] * v[2],
		T[1] * v[0] + T[4] * v[1] + T[7] * v[2],
		T[2] * v[0] + T[5] * v[1] + T[8] * v[2]);
}

inline Vec3 InvRotate(const SE3 &T, const Vec3 &v)
{
	return Vec3(T[0] * v[0] + T[1] * v[1] + T[2] * v[2],
		T[3] * v[0] + T[4] * v[1] + T[5] * v[2],
		T[6] * v[0] + T[7] * v[1] + T[8] * v[2]);
}

inline Vec3 operator * (double d, const Vec3 &v)
{
	return Vec3(d * v[0], d * v[1], d * v[2]);
}

inline double Norm(const Vec3 &v)
{
	return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

inline Vec3 Normalize(const Vec3 &v)
{
    double mag = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
	if ( mag < SR_EPS )	// make a unit vector in z-direction
		return Vec3(0.0, 0.0, 1.0);

	mag = 1.0 / mag;
	return Vec3(mag * v[0], mag * v[1], mag * v[2]);
}

inline Vec3 Cross(const Vec3 &p, const Vec3 &q)
{
	return Vec3(p[1] * q[2] - p[2] * q[1],
		p[2] * q[0] - p[0] * q[2],
		p[0] * q[1] - p[1] * q[0]);
}

inline double Inner(const Vec3 &p, const Vec3 &q)
{
	return (p[0] * q[0] + p[1] * q[1] + p[2] * q[2]);
}

inline double Cosine(const Vec3 &p, const Vec3 &q)
{
	return (Inner(p,q)/(Norm(p)*Norm(q)));
}

inline double Sine(const Vec3 &p, const Vec3 &q)
{
	return sqrt(1-Cosine(p,q)*Cosine(p,q));
}

inline double SquareSum(const Vec3 &p)
{
	return (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
}

inline Vec3 MinusLinearAd(const Vec3 &p, const se3 &s)
{
	return Vec3(p[2] * s[1] - p[1] * s[2] + s[3],
		p[0] * s[2] - p[2] * s[0] + s[4],
		p[1] * s[0] - p[0] * s[1] + s[5]);
}

inline Vec3 InvAd(const SE3 &T, const Vec3& v)
{
	return Vec3(T[0] * v[0] + T[1] * v[1] + T[2] * v[2],
		T[3] * v[0] + T[4] * v[1] + T[5] * v[2],
		T[6] * v[0] + T[7] * v[1] + T[8] * v[2]);
}

inline Vec3 iEulerZYX(const SE3 &T)
{
	return Vec3(atan2(T[1], T[0]), atan2(-T[2], sqrt(T[0] * T[0] + T[1] * T[1])), atan2(T[5], T[8]));
}

inline Vec3 iEulerZYZ(const SE3 &T)
{
	return Vec3(atan2(T[7], T[6]), atan2(sqrt(T[2] * T[2] + T[5] * T[5]), T[8]), atan2(T[5], -T[2]));
}

inline Vec3 iEulerZYX(const SO3 &T)
{
	return Vec3(atan2(T[1], T[0]), atan2(-T[2], sqrt(T[0] * T[0] + T[1] * T[1])), atan2(T[5], T[8]));
}

inline Vec3 iEulerZYZ(const SO3 &T)
{
	return Vec3(atan2(T[7], T[6]), atan2(sqrt(T[2] * T[2] + T[5] * T[5]), T[8]), atan2(T[5], -T[2]));
}

inline Vec3 ad(const Vec3 &s1, const se3 &s2)
{
	return Vec3(s2[2] * s1[1] - s2[1] * s1[2],
		s2[0] * s1[2] - s2[2] * s1[0],
		s2[1] * s1[0] - s2[0] * s1[1]);
}

inline se3::se3()
{
}

inline se3::se3(double k)
{
	_w[0] = _w[1] = _w[2] = _w[3] = _w[4] = _w[5] = k;
}

inline se3::se3(int k)
{
    _w[0] = _w[1] = _w[2] = _w[3] = _w[4] = _w[5] = (double)k;
}

inline se3::se3(double w0, double w1, double w2, double w3, double w4, double w5)
{
	_w[0] = w0;
	_w[1] = w1;
	_w[2] = w2;
	_w[3] = w3;
	_w[4] = w4;
	_w[5] = w5;
}

inline se3::se3(const Axis &w, const Vec3 &v)
{
	_w[0] = w[0];
	_w[1] = w[1];
	_w[2] = w[2];
	_w[3] = v[0];
	_w[4] = v[1];
	_w[5] = v[2];
}

inline const se3 &se3::operator + (void) const
{
	return *this;
}

inline se3 se3::operator - (void) const
{
	return se3(-_w[0], -_w[1], -_w[2], -_w[3], -_w[4], -_w[5]);
}

inline const se3 &se3::operator = (const se3 &s)
{
	_w[0] = s[0];
	_w[1] = s[1];
	_w[2] = s[2];
	_w[3] = s[3];
	_w[4] = s[4];
	_w[5] = s[5];
	return *this;
}

inline const se3 &se3::operator = (const Vec3 &s)
{
	_w[0] = _w[1] = _w[2] = 0.0;
	_w[3] = s[0];
	_w[4] = s[1];
	_w[5] = s[2];
	return *this;
}

inline const se3 &se3::operator = (const Axis &s)
{
	_w[0] = s[0];
	_w[1] = s[1];
	_w[2] = s[2];
	_w[3] = _w[4] = _w[5] = 0.0;
	return *this;
}

inline const se3 &se3::operator = (double d)
{
	_w[0] = _w[1] = _w[2] = _w[3] = _w[4] = _w[5] = d;
	return *this;
}

inline const se3 &se3::operator += (const se3 &s)
{
	_w[0] += s[0];
	_w[1] += s[1];
	_w[2] += s[2];
	_w[3] += s[3];
	_w[4] += s[4];
	_w[5] += s[5];
	return *this;
}

inline const se3 &se3::operator += (const Axis &s)
{
	_w[0] += s[0];
	_w[1] += s[1];
	_w[2] += s[2];
	return *this;
}

inline const se3 &se3::operator -= (const se3 &s)
{
	_w[0] -= s[0];
	_w[1] -= s[1];
	_w[2] -= s[2];
	_w[3] -= s[3];
	_w[4] -= s[4];
	_w[5] -= s[5];
	return *this;
}

inline const se3 &se3::operator *= (double d)
{
	_w[0] *= d;
	_w[1] *= d;
	_w[2] *= d;
	_w[3] *= d;
	_w[4] *= d;
	_w[5] *= d;
	return *this;
}

inline se3 se3::operator + (const se3 &s) const
{
	return se3(_w[0] + s[0], _w[1] + s[1], _w[2] + s[2], _w[3] + s[3], _w[4] + s[4], _w[5] + s[5]);
}

inline se3 se3::operator - (const se3 &s) const
{
	return se3(_w[0] - s[0], _w[1] - s[1], _w[2] - s[2], _w[3] - s[3], _w[4] - s[4], _w[5] - s[5]);
}

inline se3 se3::operator * (double d) const
{
	return se3(d * _w[0], d * _w[1], d * _w[2], d * _w[3], d * _w[4], d * _w[5]);
}

inline double &se3::operator [] (int i)
{
	return _w[i];
}

inline const double &se3::operator [] (int i) const
{
	return _w[i];
}

// *this = T * s * Inv(T)
inline void se3::Ad(const SE3 &T, const se3 &s)
{
	_w[0] = T[0] * s[0] + T[3] * s[1] + T[6] * s[2];
	_w[1] = T[1] * s[0] + T[4] * s[1] + T[7] * s[2];
	_w[2] = T[2] * s[0] + T[5] * s[1] + T[8] * s[2];
	_w[3] = T[10] * _w[2] - T[11] * _w[1] + T[0] * s[3] + T[3] * s[4] + T[6] * s[5];
	_w[4] = T[11] * _w[0] - T[9] * _w[2] + T[1] * s[3] + T[4] * s[4] + T[7] * s[5];
	_w[5] = T[9] * _w[1] - T[10] * _w[0] + T[2] * s[3] + T[5] * s[4] + T[8] * s[5];
}

// re = Inv(T) * s * T
inline void se3::InvAd(const SE3 &T, const se3 &s)
{
    double _tmp[3] = {	s[3] + s[1] * T[11] - s[2] * T[10],
		s[4] + s[2] * T[9] - s[0] * T[11], 
		s[5] + s[0] * T[10] - s[1] * T[9] };
	_w[0] = T[0] * s[0] + T[1] * s[1] + T[2] * s[2];
	_w[1] = T[3] * s[0] + T[4] * s[1] + T[5] * s[2];
	_w[2] = T[6] * s[0] + T[7] * s[1] + T[8] * s[2];
	_w[3] = T[0] * _tmp[0] + T[1] * _tmp[1] + T[2] * _tmp[2];
	_w[4] = T[3] * _tmp[0] + T[4] * _tmp[1] + T[5] * _tmp[2];
	_w[5] = T[6] * _tmp[0] + T[7] * _tmp[1] + T[8] * _tmp[2];
}

inline void se3::ad(const se3 &s1, const se3 &s2)
{
	_w[0] =	s1[1] * s2[2] - s1[2] * s2[1];
	_w[1] =	s1[2] * s2[0] - s1[0] * s2[2];
	_w[2] =	s1[0] * s2[1] - s1[1] * s2[0];
	_w[3] =	s1[1] * s2[5] - s1[2] * s2[4] - s2[1] * s1[5] + s2[2] * s1[4];
	_w[4] =	s1[2] * s2[3] - s1[0] * s2[5] - s2[2] * s1[3] + s2[0] * s1[5];
	_w[5] =	s1[0] * s2[4] - s1[1] * s2[3] - s2[0] * s1[4] + s2[1] * s1[3];
}

inline void se3::ad(const se3 &s1, const Axis &s2)
{
	_w[0] =	s1[1] * s2[2] - s1[2] * s2[1];
	_w[1] =	s1[2] * s2[0] - s1[0] * s2[2];
	_w[2] =	s1[0] * s2[1] - s1[1] * s2[0];
	_w[3] =	s2[2] * s1[4] - s2[1] * s1[5];
	_w[4] =	s2[0] * s1[5] - s2[2] * s1[3];
	_w[5] = s2[1] * s1[3] - s2[0] * s1[4];
}

inline se3 operator * (double d, const se3 &s)
{
	return se3(d * s[0], d * s[1], d * s[2], d * s[3], d * s[4], d * s[5]);
}

inline double operator * (const dse3 &t, const se3 &s)
{
	return (t[0] * s[0] + t[1] * s[1] + t[2] * s[2] + t[3] * s[3] + t[4] * s[4] + t[5] * s[5]);
}

inline double operator * (const dse3 &t, const Axis &s)
{
	return (t[0] * s[0] + t[1] * s[1] + t[2] * s[2]);
}

inline double operator * (const se3 &s, const dse3 &t)
{
	return (t[0] * s[0] + t[1] * s[1] + t[2] * s[2] + t[3] * s[3] + t[4] * s[4] + t[5] * s[5]);
}

inline se3 Log(const SE3 &T)
{
    double theta = 0.5 * (T[0] + T[4] + T[8] - 1.0), t_st;

	if ( theta < SR_EPS - 1.0 )
	{
        double w[3];
		if ( T[0] > 1.0 - SR_EPS )
		{
			w[0] = SR_PI;
			w[1] = w[2] = 0.0;
		} else if ( T[4] > 1.0 - SR_EPS )
		{
			w[1] = SR_PI;
			w[0] = w[2] = 0.0;
		} else if ( T[8] > 1.0 - SR_EPS )
		{
			w[2] = SR_PI;
			w[0] = w[1] = 0.0;
		} else
		{
			w[0] = SR_PI_SQRT2 * sqrt((T[1] * T[1] + T[2] * T[2]) / (1.0 - T[0]));
			w[1] = SR_PI_SQRT2 * sqrt((T[3] * T[3] + T[5] * T[5]) / (1.0 - T[4]));
			w[2] = SR_PI_SQRT2 * sqrt((T[6] * T[6] + T[7] * T[7]) / (1.0 - T[8]));
		}

        double w2[] = { w[0] * w[0], w[1] * w[1], w[2] * w[2] }, w3[] = { w2[0] * w[0], w2[1] * w[1], w2[2] * w[2] }, v[] = { w[1] * w[2], w[2] * w[0], w[0] * w[1] };
        double id = (double)0.25 * SR_PI_SQR / (w2[0] * w2[0] + w2[1] * w2[1] + w2[2] * w2[2] + 2.0 * (w2[0] * w2[1] + w2[1] * w2[2] + w2[2] * w2[0]));
        double p[] = { T[9] * id, T[10] * id, T[11] * id };

		return se3(	w[0], w[1], w[2],
			2.0 * (2.0 * w2[0] * p[0] + (w3[2] + v[1] * w[0] + v[0] * w[1] + 2.0 * v[2]) * p[1] + (2.0 * v[1] - w3[1] - w[0] * v[2] - w[2] * v[0]) * p[2]),
			2.0 * ((2.0 * v[2] - w3[2] - v[0] * w[1] - v[1] * w[0]) * p[0] + 2.0 * w2[1] * p[1] + (w3[0] + w[2] * v[1] + v[2] * w[1] + 2.0 * v[0]) * p[2]),
			2.0 * ((w[0] * v[2] + w[2] * v[0] + 2.0 * v[1] + w3[1]) * p[0] + (2.0 * v[0] - w3[0] - v[2] * w[1] - w[2] * v[1]) * p[1] + 2.0 * w2[2] * p[2]));
	} else
	{
		//*bk : acos
		if (theta > 1.0)
		{
			theta = 1.0;
		}
		else if(theta < -1.0)
		{
			theta = -1.0;
		}
		//
		theta = acos(theta);
        if ( theta < SR_EPS )	t_st = (double)3.0 / ((double)6.0 - theta * theta);
		else					t_st = theta / (2.0 * sin(theta));
        double stct, st = sin(theta), w[] = { T[5] - T[7], T[6] - T[2], T[1] - T[3] }, w2[] = { w[0] * w[0], w[1] * w[1], w[2] * w[2] }, w3[] = { w[1] * w[2], w[2] * w[0], w[0] * w[1] };
		w[0] = t_st * w[0];	w[1] = t_st * w[1];	w[2] = t_st * w[2];

		if ( theta < SR_EPS )	stct = theta / 48.0;
		else					stct = (2.0 * st - theta * (1.0 + cos(theta))) / (8.0 * st * st * st);

		return se3(	w[0], w[1], w[2],
			(1.0 - stct * (w2[1] + w2[2])) * T[9] + (0.5 * w[2] + stct * w3[2]) * T[10] + (stct * w3[1] - 0.5 * w[1]) * T[11],
			(stct * w3[2] - 0.5 * w[2]) * T[9] + (1.0 - stct * (w2[0] + w2[2])) * T[10] + (0.5 * w[0] + stct * w3[0]) * T[11],
			(0.5 * w[1] + stct * w3[1]) * T[9] + (stct * w3[0] - 0.5 * w[0]) * T[10] + (1.0 - stct * (w2[1] + w2[0])) * T[11]);
	}
}

inline Axis LogR(const SE3 &T)
{
    double theta = 0.5 * (T[0] + T[4] + T[8] - 1.0), t_st;

	if ( theta < SR_EPS - 1.0 )
	{
		if ( T[0] > 1.0 - SR_EPS ) return Axis(SR_PI, 0.0, 0.0);
		else if ( T[4] > 1.0 - SR_EPS ) return Axis(0.0, SR_PI, 0.0);
		else if ( T[8] > 1.0 - SR_EPS ) return Axis(0.0, 0.0, SR_PI);
		return Axis(SR_PI_SQRT2 * sqrt((T[1] * T[1] + T[2] * T[2]) / (1.0 - T[0])),
			SR_PI_SQRT2 * sqrt((T[3] * T[3] + T[5] * T[5]) / (1.0 - T[4])),
			SR_PI_SQRT2 * sqrt((T[6] * T[6] + T[7] * T[7]) / (1.0 - T[8])));
	} else
	{
		theta = acos(theta);
        if ( theta < SR_EPS )	t_st = (double)3.0 / ((double)6.0 - theta * theta);
		else					t_st = theta / (2.0 * sin(theta));
		return Axis(t_st * (T[5] - T[7]), t_st * (T[6] - T[2]), t_st * (T[1] - T[3]));
	}
}

// re = T * s * Inv(T)
inline se3 Ad(const SE3 &T, const se3 &s)
{
    double tmp[3] = {	T[0] * s[0] + T[3] * s[1] + T[6] * s[2],
		T[1] * s[0] + T[4] * s[1] + T[7] * s[2], 
		T[2] * s[0] + T[5] * s[1] + T[8] * s[2] };
	return se3(	tmp[0], tmp[1], tmp[2],
		T[10] * tmp[2] - T[11] * tmp[1] + T[0] * s[3] + T[3] * s[4] + T[6] * s[5],
		T[11] * tmp[0] - T[9] * tmp[2] + T[1] * s[3] + T[4] * s[4] + T[7] * s[5],
		T[9] * tmp[1] - T[10] * tmp[0] + T[2] * s[3] + T[5] * s[4] + T[8] * s[5]);
}

inline se3 Ad(const SE3 &T, const Axis &s)
{
    double tmp[3] = {	T[0] * s[0] + T[3] * s[1] + T[6] * s[2],
		T[1] * s[0] + T[4] * s[1] + T[7] * s[2], 
		T[2] * s[0] + T[5] * s[1] + T[8] * s[2] };
	return se3(	tmp[0], tmp[1], tmp[2],
		T[10] * tmp[2] - T[11] * tmp[1],
		T[11] * tmp[0] - T[9] * tmp[2],
		T[9] * tmp[1] - T[10] * tmp[0]);
}

// re = Inv(T) * s * T
inline se3 InvAd(const SE3 &T, const se3 &s)
{
    double tmp[3] = {	s[3] + s[1] * T[11] - s[2] * T[10],
		s[4] + s[2] * T[9] - s[0] * T[11], 
		s[5] + s[0] * T[10] - s[1] * T[9] };
	return se3(	T[0] * s[0] + T[1] * s[1] + T[2] * s[2],
		T[3] * s[0] + T[4] * s[1] + T[5] * s[2],
		T[6] * s[0] + T[7] * s[1] + T[8] * s[2],
		T[0] * tmp[0] + T[1] * tmp[1] + T[2] * tmp[2],
		T[3] * tmp[0] + T[4] * tmp[1] + T[5] * tmp[2],
		T[6] * tmp[0] + T[7] * tmp[1] + T[8] * tmp[2]);
}

inline se3 ad(const se3 &s1, const se3 &s2)
{
	return se3(	s1[1] * s2[2] - s1[2] * s2[1],
		s1[2] * s2[0] - s1[0] * s2[2],
		s1[0] * s2[1] - s1[1] * s2[0],
		s1[1] * s2[5] - s1[2] * s2[4] - s2[1] * s1[5] + s2[2] * s1[4],
		s1[2] * s2[3] - s1[0] * s2[5] - s2[2] * s1[3] + s2[0] * s1[5],
		s1[0] * s2[4] - s1[1] * s2[3] - s2[0] * s1[4] + s2[1] * s1[3]);
}

inline double SquareSum(const se3 &s)
{
	return (s[0] * s[0] + s[1] * s[1] + s[2] * s[2] + s[3] * s[3] + s[4] * s[4] + s[5] * s[5]);
}

inline se3 Rotate(const SE3 &T, const se3 &v)
{
	return se3(	T[0] * v[0] + T[3] * v[1] + T[6] * v[2],
		T[1] * v[0] + T[4] * v[1] + T[7] * v[2],
		T[2] * v[0] + T[5] * v[1] + T[8] * v[2],
		T[0] * v[3] + T[3] * v[4] + T[6] * v[5],
		T[1] * v[3] + T[4] * v[4] + T[7] * v[5],
		T[2] * v[3] + T[5] * v[4] + T[8] * v[5]);
}

inline se3 InvRotate(const SE3 &T, const se3 &v)
{
	return se3(	T[0] * v[0] + T[1] * v[1] + T[2] * v[2],
		T[3] * v[0] + T[4] * v[1] + T[5] * v[2],
		T[6] * v[0] + T[7] * v[1] + T[8] * v[2],
		T[0] * v[3] + T[1] * v[4] + T[2] * v[5],
		T[3] * v[3] + T[4] * v[4] + T[5] * v[5],
		T[6] * v[3] + T[7] * v[4] + T[8] * v[5]);
}

inline dse3::dse3()
{
}

inline dse3::dse3(double k)
{
	_m[0] = _m[1] = _m[2] = _m[3] = _m[4] = _m[5] = k;
}

inline dse3::dse3(int k)
{
    _m[0] = _m[1] = _m[2] = _m[3] = _m[4] = _m[5] = (double)k;
}

inline dse3::dse3(double m0, double m1, double m2, double m3, double m4, double m5)
{
	_m[0] = m0;
	_m[1] = m1;
	_m[2] = m2;
	_m[3] = m3;
	_m[4] = m4;
	_m[5] = m5;
}

inline dse3::dse3(const Axis &m, const Vec3 &f)
{
	_m[0] = m[0];
	_m[1] = m[1];
	_m[2] = m[2];
	_m[3] = f[0];
	_m[4] = f[1];
	_m[5] = f[2];
}

inline dse3::dse3(const Vec3 &m, const Vec3 &f)
{
	_m[0] = m[0];
	_m[1] = m[1];
	_m[2] = m[2];
	_m[3] = f[0];
	_m[4] = f[1];
	_m[5] = f[2];
}

inline dse3::dse3(double mass, const se3 &dV)
{
	_m[0] = mass * dV[0];
	_m[1] = mass * dV[1];
	_m[2] = mass * dV[2];
	_m[3] = mass * dV[3];
	_m[4] = mass * dV[4];
	_m[5] = mass * dV[5];
}

inline dse3::dse3(const double v[])
{
	_m[0] = v[0];
	_m[1] = v[1];
	_m[2] = v[2];
	_m[3] = v[3];
	_m[4] = v[4];
	_m[5] = v[5];
}

inline const dse3 &dse3::operator + (void) const
{
	return *this;
}

inline dse3 dse3::operator - (void) const
{
	return dse3(-_m[0], -_m[1], -_m[2], -_m[3], -_m[4], -_m[5]);
}

inline const dse3 &dse3::operator = (const dse3 &t)
{
	_m[0] = t[0];
	_m[1] = t[1];
	_m[2] = t[2];
	_m[3] = t[3];
	_m[4] = t[4];
	_m[5] = t[5];
	return *this;
}

inline const dse3 &dse3::operator = (const se3 &v)
{
	_m[0] = v[0];
	_m[1] = v[1];
	_m[2] = v[2];
	_m[3] = v[3];
	_m[4] = v[4];
	_m[5] = v[5];
	return *this;
}

inline const dse3 &dse3::operator = (const Axis &t)
{
	_m[0] = t[0];
	_m[1] = t[1];
	_m[2] = t[2];
	_m[3] = _m[4] = _m[5] = 0.0;
	return *this;
}

inline const dse3 &dse3::operator = (const Vec3 &t)
{
	_m[0] = _m[1] =  _m[2] = 0.0;
	_m[3] = t[0];
	_m[4] = t[1];
	_m[5] = t[2];
	return *this;
}

inline const dse3 &dse3::operator = (double d)
{
	_m[0] = _m[1] = _m[2] = _m[3] = _m[4] = _m[5] = d;
	return *this;
}

inline const dse3 &dse3::operator += (const dse3 &t)
{
	_m[0] += t[0];
	_m[1] += t[1];
	_m[2] += t[2];
	_m[3] += t[3];
	_m[4] += t[4];
	_m[5] += t[5];
	return *this;
}

inline const dse3 &dse3::operator -= (const dse3 &t)
{
	_m[0] -= t[0];
	_m[1] -= t[1];
	_m[2] -= t[2];
	_m[3] -= t[3];
	_m[4] -= t[4];
	_m[5] -= t[5];
	return *this;
}

inline const dse3 &dse3::operator *= (double d)
{
	_m[0] *= d;
	_m[1] *= d;
	_m[2] *= d;
	_m[3] *= d;
	_m[4] *= d;
	_m[5] *= d;
	return *this;
}

inline dse3 dse3::operator + (const dse3 &t) const
{
	return dse3(_m[0] + t[0], _m[1] + t[1], _m[2] + t[2], _m[3] + t[3], _m[4] + t[4], _m[5] + t[5]);
}	

inline dse3 dse3::operator - (const dse3 &t) const
{
	return dse3(_m[0] - t[0], _m[1] - t[1], _m[2] - t[2], _m[3] - t[3], _m[4] - t[4], _m[5] - t[5]);
}	

inline dse3 dse3::operator * (double d) const
{
	return dse3(d * _m[0], d * _m[1], d * _m[2], d * _m[3], d * _m[4], d * _m[5]);
}

inline double &dse3::operator [] (int i)
{
	return _m[i];
}

inline const double &dse3::operator [] (int i) const
{
	return _m[i];
}

inline dse3 dad(const se3 &s, const dse3 &t)
{
	return dse3(t[1] * s[2] - t[2] * s[1] + t[4] * s[5] - t[5] * s[4],
		t[2] * s[0] - t[0] * s[2] + t[5] * s[3] - t[3] * s[5],
		t[0] * s[1] - t[1] * s[0] + t[3] * s[4] - t[4] * s[3],
		t[4] * s[2] - t[5] * s[1],
		t[5] * s[0] - t[3] * s[2],
		t[3] * s[1] - t[4] * s[0]);
}

inline void dse3::dad(const se3 &s, const dse3 &t)
{
	_m[0] =	t[1] * s[2] - t[2] * s[1] + t[4] * s[5] - t[5] * s[4];
	_m[1] =	t[2] * s[0] - t[0] * s[2] + t[5] * s[3] - t[3] * s[5];
	_m[2] =	t[0] * s[1] - t[1] * s[0] + t[3] * s[4] - t[4] * s[3];
	_m[3] =	t[4] * s[2] - t[5] * s[1];
	_m[4] =	t[5] * s[0] - t[3] * s[2];
	_m[5] =	t[3] * s[1] - t[4] * s[0];
}

inline void dse3::dAd(const SE3 &T, const dse3 &t)
{
    double tmp[3] = {	t[0] - T[10] * t[5] + T[11] * t[4],
		t[1] - T[11] * t[3] + T[9] * t[5], 
		t[2] - T[9] * t[4] + T[10] * t[3] };
	_m[0] = T[0] * tmp[0] + T[1] * tmp[1] + T[2] * tmp[2];
	_m[1] = T[3] * tmp[0] + T[4] * tmp[1] + T[5] * tmp[2];
	_m[2] = T[6] * tmp[0] + T[7] * tmp[1] + T[8] * tmp[2];
	_m[3] = T[0] * t[3] + T[1] * t[4] + T[2] * t[5];
	_m[4] = T[3] * t[3] + T[4] * t[4] + T[5] * t[5];
	_m[5] = T[6] * t[3] + T[7] * t[4] + T[8] * t[5];
}

inline dse3 operator * (double d, const dse3 &t)
{
	return dse3(d * t[0], d * t[1], d * t[2], d * t[3], d * t[4], d * t[5]);
}

inline dse3 dAd(const SE3 &T, const dse3 &t)
{
    double tmp[3] = {	t[0] - T[10] * t[5] + T[11] * t[4],
		t[1] - T[11] * t[3] + T[9] * t[5], 
		t[2] - T[9] * t[4] + T[10] * t[3] };
	return dse3(T[0] * tmp[0] + T[1] * tmp[1] + T[2] * tmp[2],
		T[3] * tmp[0] + T[4] * tmp[1] + T[5] * tmp[2],
		T[6] * tmp[0] + T[7] * tmp[1] + T[8] * tmp[2],
		T[0] * t[3] + T[1] * t[4] + T[2] * t[5],
		T[3] * t[3] + T[4] * t[4] + T[5] * t[5],
		T[6] * t[3] + T[7] * t[4] + T[8] * t[5]);
}

inline dse3 dAd(const SE3 &T, const Vec3 &f)
{
    double tmp[3] = {	- T[10] * f[2] + T[11] * f[1],
		- T[11] * f[0] + T[9] * f[2], 
		- T[9] * f[1] + T[10] * f[0] };
	return dse3(T[0] * tmp[0] + T[1] * tmp[1] + T[2] * tmp[2],
		T[3] * tmp[0] + T[4] * tmp[1] + T[5] * tmp[2],
		T[6] * tmp[0] + T[7] * tmp[1] + T[8] * tmp[2],
		T[0] * f[0] + T[1] * f[1] + T[2] * f[2],
		T[3] * f[0] + T[4] * f[1] + T[5] * f[2],
		T[6] * f[0] + T[7] * f[1] + T[8] * f[2]);
}

inline dse3 InvdAd(const SE3 &T, const dse3 &t)
{
    double tmp[3] = {	T[0] * t[3] + T[3] * t[4] + T[6] * t[5],
		T[1] * t[3] + T[4] * t[4] + T[7] * t[5], 
		T[2] * t[3] + T[5] * t[4] + T[8] * t[5] };
	return dse3(T[10] * tmp[2] - T[11] * tmp[1] + T[0] * t[0] + T[3] * t[1] + T[6] * t[2],
		T[11] * tmp[0] - T[9] * tmp[2] + T[1] * t[0] + T[4] * t[1] + T[7] * t[2],
		T[9] * tmp[1] - T[10] * tmp[0] + T[2] * t[0] + T[5] * t[1] + T[8] * t[2],
		tmp[0], tmp[1], tmp[2]);
}

inline dse3 InvdAd(const Vec3 &p, const Vec3 &f)
{
	return dse3(p[1] * f[2] - p[2] * f[1],
		p[2] * f[0] - p[0] * f[2],
		p[0] * f[1] - p[1] * f[0],
		f[0],
		f[1],
		f[2]);
}

inline double SquareSum(const dse3 &t)
{
	return (t[0] * t[0] + t[1] * t[1] + t[2] * t[2] + t[3] * t[3] + t[4] * t[4] + t[5] * t[5]);
}

inline SE3::SE3()
{
	_T[0] = _T[4] = _T[8] = 1.0;
	_T[1] = _T[2] = _T[3] = _T[5] = _T[6] = _T[7] = _T[9] = _T[10] = _T[11] = 0.0;
}

inline SE3::SE3(const SE3 &T)
{
	_T[0] = T[0];
	_T[1] = T[1];
	_T[2] = T[2];
	_T[3] = T[3];
	_T[4] = T[4];
	_T[5] = T[5];
	_T[6] = T[6];
	_T[7] = T[7];
	_T[8] = T[8];
	_T[9] = T[9];
	_T[10] = T[10];
	_T[11] = T[11];
}

inline SE3::SE3(double T0, double T1, double T2, double T4, double T5, double T6, double T8, double T9, double T10, double T12, double T13, double T14)
{
	_T[0] = T0;
	_T[1] = T1;
	_T[2] = T2;
	_T[3] = T4;
	_T[4] = T5;
	_T[5] = T6;
	_T[6] = T8;
	_T[7] = T9;
	_T[8] = T10;
	_T[9] = T12;
	_T[10] = T13;
	_T[11] = T14;
}

inline SE3::SE3(double T0, double T1, double T2, double T4, double T5, double T6, double T8, double T9, double T10)
{
	_T[0] = T0;
	_T[1] = T1;
	_T[2] = T2;
	_T[3] = T4;
	_T[4] = T5;
	_T[5] = T6;
	_T[6] = T8;
	_T[7] = T9;
	_T[8] = T10;
	_T[9] = _T[10] = _T[11] = 0.0;
}

inline SE3::SE3(const Vec3 &p)
{
	_T[0] = _T[4] = _T[8] = 1.0;
	_T[1] = _T[2] = _T[3] = _T[5] = _T[6] = _T[7] = 0.0;
	_T[9] = p[0];
	_T[10] = p[1];
	_T[11] = p[2];
}

//////////////////////////////////////////////////////////////////////////JS 2006.11.30
inline SE3::SE3(const SO3 &R, const Vec3 &p)
{
	_T[0] = R._R[0];
	_T[1] = R._R[1];
	_T[2] = R._R[2];
	_T[3] = R._R[3];
	_T[4] = R._R[4];
	_T[5] = R._R[5];
	_T[6] = R._R[6];
	_T[7] = R._R[7];
	_T[8] = R._R[8];
	_T[9]	= p._v[0];
	_T[10]	= p._v[1];
	_T[11]	= p._v[2];
}
//////////////////////////////////////////////////////////////////////////

inline SE3::SE3(const Vec3 &Rx, const Vec3 &Ry, const Vec3 &Rz, const Vec3 &p)
{
	_T[0] = Rx[0];
	_T[1] = Rx[1];
	_T[2] = Rx[2];
	_T[3] = Ry[0];
	_T[4] = Ry[1];
	_T[5] = Ry[2];
	_T[6] = Rz[0];
	_T[7] = Rz[1];
	_T[8] = Rz[2];
	_T[9] = p[0];
	_T[10] = p[1];
	_T[11] = p[2];
}

inline SE3::SE3(double p)
{
	_T[0] = _T[4] = _T[8] = 1.0;
	_T[1] = _T[2] = _T[3] = _T[5] = _T[6] = _T[7] = 0.0;
	_T[9] = _T[10] = _T[11] = p;
}

inline SE3::SE3(int p)
{
	_T[0] = _T[4] = _T[8] = 1.0;
	_T[1] = _T[2] = _T[3] = _T[5] = _T[6] = _T[7] = 0.0;
    _T[9] = _T[10] = _T[11] = (double)p;
}

inline SE3::SE3(const double T[])
{
	_T[0] = T[0];
	_T[1] = T[1];
	_T[2] = T[2];
	_T[3] = T[4];
	_T[4] = T[5];
	_T[5] = T[6];
	_T[6] = T[8];
	_T[7] = T[9];
	_T[8] = T[10];
	_T[9] = T[12];
	_T[10] = T[13];
	_T[11] = T[14];
}

inline double SE3::operator () (int i, int j) const
{
	if ( i == 3 ) return j == 3 ? 1.0 : 0.0;
	return _T[i + (3 * j)];
}

inline const double &SE3::operator [] (int i) const
{
	return _T[i];
}

inline double &SE3::operator [] (int i)
{
	return _T[i];
}

inline const SE3 &SE3::operator = (const SE3 &T)
{
	_T[0] = T[0];
	_T[1] = T[1];
	_T[2] = T[2];
	_T[3] = T[3];
	_T[4] = T[4];
	_T[5] = T[5];
	_T[6] = T[6];
	_T[7] = T[7];
	_T[8] = T[8];
	_T[9] = T[9];
	_T[10] = T[10];
	_T[11] = T[11];
	return *this;
}

inline const SE3 &SE3::operator = (const Vec3 &p)
{
	_T[0] = _T[4] = _T[8] = 1.0;
	_T[1] = _T[2] = _T[3] = _T[5] = _T[6] = _T[7] = 0.0;
	_T[9] = p[0];
	_T[10] = p[1];
	_T[11] = p[2];
	return *this;
}

inline SE3 SE3::operator * (const SE3 &T) const
{
	return SE3(	_T[0] * T[0] + _T[3] * T[1] + _T[6] * T[2],
		_T[1] * T[0] + _T[4] * T[1] + _T[7] * T[2],
		_T[2] * T[0] + _T[5] * T[1] + _T[8] * T[2],
		_T[0] * T[3] + _T[3] * T[4] + _T[6] * T[5],
		_T[1] * T[3] + _T[4] * T[4] + _T[7] * T[5],
		_T[2] * T[3] + _T[5] * T[4] + _T[8] * T[5],
		_T[0] * T[6] + _T[3] * T[7] + _T[6] * T[8],
		_T[1] * T[6] + _T[4] * T[7] + _T[7] * T[8],
		_T[2] * T[6] + _T[5] * T[7] + _T[8] * T[8],
		_T[9] + _T[0] * T[9] + _T[3] * T[10] + _T[6] * T[11],
		_T[10] + _T[1] * T[9] + _T[4] * T[10] + _T[7] * T[11],
		_T[11] + _T[2] * T[9] + _T[5] * T[10] + _T[8] * T[11] );
}

inline SE3 SE3::operator / (const SE3 &T) const
{
    double tmp[] = {_T[0] * T[0] + _T[3] * T[3] + _T[6] * T[6],
		_T[1] * T[0] + _T[4] * T[3] + _T[7] * T[6],
		_T[2] * T[0] + _T[5] * T[3] + _T[8] * T[6],
		_T[0] * T[1] + _T[3] * T[4] + _T[6] * T[7],
		_T[1] * T[1] + _T[4] * T[4] + _T[7] * T[7],
		_T[2] * T[1] + _T[5] * T[4] + _T[8] * T[7],
		_T[0] * T[2] + _T[3] * T[5] + _T[6] * T[8],
		_T[1] * T[2] + _T[4] * T[5] + _T[7] * T[8],
		_T[2] * T[2] + _T[5] * T[5] + _T[8] * T[8] };
	return SE3(	tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5], tmp[6], tmp[7], tmp[8], 
		_T[9] - tmp[0] * T[9] - tmp[3] * T[10] - tmp[6] * T[11],
		_T[10] - tmp[1] * T[9] - tmp[4] * T[10] - tmp[7] * T[11],
		_T[11] - tmp[2] * T[9] - tmp[5] * T[10] - tmp[8] * T[11] );
}

inline SE3 SE3::operator % (const SE3 &T) const
{
	return SE3(	_T[0] * T[0] + _T[1] * T[1] + _T[2] * T[2],
		_T[3] * T[0] + _T[4] * T[1] + _T[5] * T[2],
		_T[6] * T[0] + _T[7] * T[1] + _T[8] * T[2],
		_T[0] * T[3] + _T[1] * T[4] + _T[2] * T[5],
		_T[3] * T[3] + _T[4] * T[4] + _T[5] * T[5],
		_T[6] * T[3] + _T[7] * T[4] + _T[8] * T[5],
		_T[0] * T[6] + _T[1] * T[7] + _T[2] * T[8],
		_T[3] * T[6] + _T[4] * T[7] + _T[5] * T[8],
		_T[6] * T[6] + _T[7] * T[7] + _T[8] * T[8],
		_T[0] * (T[9] - _T[9]) + _T[1] * (T[10] - _T[10]) + _T[2] * (T[11] - _T[11]),
		_T[3] * (T[9] - _T[9]) + _T[4] * (T[10] - _T[10]) + _T[5] * (T[11] - _T[11]),
		_T[6] * (T[9] - _T[9]) + _T[7] * (T[10] - _T[10]) + _T[8] * (T[11] - _T[11]) );
}

inline Vec3 SE3::operator % (const Vec3 &p) const
{
	return Vec3(_T[0] * (p[0] - _T[9]) + _T[1] * (p[1] - _T[10]) + _T[2] * (p[2] - _T[11]),
		_T[3] * (p[0] - _T[9]) + _T[4] * (p[1] - _T[10]) + _T[5] * (p[2] - _T[11]),
		_T[6] * (p[0] - _T[9]) + _T[7] * (p[1] - _T[10]) + _T[8] * (p[2] - _T[11]) );
}

inline Vec3 SE3::operator * (const Vec3 &p) const
{
	return Vec3(_T[9] + _T[0] * p[0] + _T[3] * p[1] + _T[6] * p[2],
		_T[10] + _T[1] * p[0] + _T[4] * p[1] + _T[7] * p[2],
		_T[11] + _T[2] * p[0] + _T[5] * p[1] + _T[8] * p[2] );
}

inline const SE3 &SE3::operator *= (const SE3 &T)
{
    double x0, x1, x2;
	_T[9] += _T[0] * T[9] + _T[3] * T[10] + _T[6] * T[11];
	_T[10] += _T[1] * T[9] + _T[4] * T[10] + _T[7] * T[11];
	_T[11] += _T[2] * T[9] + _T[5] * T[10] + _T[8] * T[11];
	x0 = _T[0] * T[0] + _T[3] * T[1] + _T[6] * T[2];
	x1 = _T[0] * T[3] + _T[3] * T[4] + _T[6] * T[5];
	x2 = _T[0] * T[6] + _T[3] * T[7] + _T[6] * T[8];
	_T[0] = x0;	_T[3] = x1;	_T[6] = x2;
	x0 = _T[1] * T[0] + _T[4] * T[1] + _T[7] * T[2];
	x1 = _T[1] * T[3] + _T[4] * T[4] + _T[7] * T[5];
	x2 = _T[1] * T[6] + _T[4] * T[7] + _T[7] * T[8];
	_T[1] = x0;	_T[4] =x1;	_T[7] = x2;
	x0 = _T[2] * T[0] + _T[5] * T[1] + _T[8] * T[2];
	x1 = _T[2] * T[3] + _T[5] * T[4] + _T[8] * T[5];
	x2 = _T[2] * T[6] + _T[5] * T[7] + _T[8] * T[8];
	_T[2] = x0; _T[5] = x1; _T[8] = x2;
	return  *this;
}

inline const SE3 &SE3::operator /= (const SE3 &T)
{
    double tmp[] = {_T[0] * T[0] + _T[3] * T[3] + _T[6] * T[6],
		_T[1] * T[0] + _T[4] * T[3] + _T[7] * T[6],
		_T[2] * T[0] + _T[5] * T[3] + _T[8] * T[6],
		_T[0] * T[1] + _T[3] * T[4] + _T[6] * T[7],
		_T[1] * T[1] + _T[4] * T[4] + _T[7] * T[7],
		_T[2] * T[1] + _T[5] * T[4] + _T[8] * T[7],
		_T[0] * T[2] + _T[3] * T[5] + _T[6] * T[8],
		_T[1] * T[2] + _T[4] * T[5] + _T[7] * T[8],
		_T[2] * T[2] + _T[5] * T[5] + _T[8] * T[8] };
	_T[0] = tmp[0]; _T[1] = tmp[1]; _T[2] = tmp[2];
	_T[3] = tmp[3]; _T[4] = tmp[4]; _T[5] = tmp[5];
	_T[6] = tmp[6]; _T[7] = tmp[7]; _T[8] = tmp[8], 
		_T[9] -= tmp[0] * T[9] + tmp[3] * T[10] + tmp[6] * T[11];
	_T[10] -= tmp[1] * T[9] + tmp[4] * T[10] + tmp[7] * T[11];
	_T[11] -= tmp[2] * T[9] + tmp[5] * T[10] + tmp[8] * T[11];
	return *this;
}

inline const SE3 &SE3::operator %= (const SE3 &T)
{
    double tmp[12] = { _T[0], _T[1], _T[2], _T[3], _T[4], _T[5], _T[6], _T[7], _T[8], T[9] - _T[9], T[10] - _T[10], T[11] - _T[11] };
	_T[0] = tmp[0] * T[0] + tmp[1] * T[1] + tmp[2] * T[2];
	_T[1] = tmp[3] * T[0] + tmp[4] * T[1] + tmp[5] * T[2];
	_T[2] = tmp[6] * T[0] + tmp[7] * T[1] + tmp[8] * T[2];
	_T[3] = tmp[0] * T[3] + tmp[1] * T[4] + tmp[2] * T[5];
	_T[4] = tmp[3] * T[3] + tmp[4] * T[4] + tmp[5] * T[5];
	_T[5] = tmp[6] * T[3] + tmp[7] * T[4] + tmp[8] * T[5];
	_T[6] = tmp[0] * T[6] + tmp[1] * T[7] + tmp[2] * T[8];
	_T[7] = tmp[3] * T[6] + tmp[4] * T[7] + tmp[5] * T[8];
	_T[8] = tmp[6] * T[6] + tmp[7] * T[7] + tmp[8] * T[8];
	_T[9] = tmp[0] * tmp[9] + tmp[1] * tmp[10] + tmp[2] * tmp[11];
	_T[10] = tmp[3] * tmp[9] + tmp[4] * tmp[10] + tmp[5] * tmp[11];
	_T[11] = tmp[6] * tmp[9] + tmp[7] * tmp[10] + tmp[8] * tmp[11];
	return *this;
}

inline void SE3::SetEye(void)
{
	_T[0] = _T[4] = _T[8] = 1.0;
	_T[1] = _T[2] = _T[3] = _T[5] = _T[6] = _T[7] = _T[9] = _T[10] = _T[11] = 0.0;
}

inline void SE3::Set(const SE3 &T, const Vec3 &p)
{
	_T[0] = T[0];
	_T[1] = T[1];
	_T[2] = T[2];
	_T[3] = T[3];
	_T[4] = T[4];
	_T[5] = T[5];
	_T[6] = T[6];
	_T[7] = T[7];
	_T[8] = T[8];
	_T[9] = p[0];
	_T[10] = p[1];
	_T[11] = p[2];
}

inline void SE3::Set(const SE3 &T)
{
	_T[0] = T[0];
	_T[1] = T[1];
	_T[2] = T[2];
	_T[3] = T[3];
	_T[4] = T[4];
	_T[5] = T[5];
	_T[6] = T[6];
	_T[7] = T[7];
	_T[8] = T[8];
	_T[9] = T[9];
	_T[10] = T[10];
	_T[11] = T[11];
}

inline void SE3::Set(const SO3 &R, const Vec3 &P)
{
	_T[0] = R(0, 0);
	_T[1] = R(1, 0);
	_T[2] = R(2, 0);
	_T[3] = R(0, 1);
	_T[4] = R(1, 1);
	_T[5] = R(2, 1);
	_T[6] = R(0, 2);
	_T[7] = R(1, 2);
	_T[8] = R(2, 2);
	_T[9] = P[0];
	_T[10] = P[1];
	_T[11] = P[2];
}

inline void SE3::SetOrientation(const SE3 &T)
{
	_T[0] = T[0];
	_T[1] = T[1];
	_T[2] = T[2];
	_T[3] = T[3];
	_T[4] = T[4];
	_T[5] = T[5];
	_T[6] = T[6];
	_T[7] = T[7];
	_T[8] = T[8];
}

inline void SE3::SetOrientation(const SO3 &R)
{
	_T[0] = R[0];
	_T[1] = R[1];
	_T[2] = R[2];
	_T[3] = R[3];
	_T[4] = R[4];
	_T[5] = R[5];
	_T[6] = R[6];
	_T[7] = R[7];
	_T[8] = R[8];
}

//////////////////////////////////////////////////////////////////////////JS 2006.11.29
inline SO3 SE3::GetOrientation(void) const
{
	return SO3(_T[0], _T[1], _T[2],
		_T[3], _T[4], _T[5],
		_T[6], _T[7], _T[8]);
}
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////JS 2008.1.26
inline Vec3 SE3::GetX(void) const
{
	return Vec3(_T[0], _T[1], _T[2]);
}

inline Vec3 SE3::GetY(void) const
{
	return Vec3(_T[3], _T[4], _T[5]);
}

inline Vec3 SE3::GetZ(void) const
{
	return Vec3(_T[6], _T[7], _T[8]);
}
//////////////////////////////////////////////////////////////////////////

inline void SE3::SetPosition(const Vec3 &Pos)
{
	_T[9] = Pos[0];
	_T[10] = Pos[1];
	_T[11] = Pos[2];
}

inline Vec3 SE3::GetPosition(void) const
{
	return Vec3(_T[9], _T[10], _T[11]);
}

inline void SE3::Exp(const se3 &S)
{
    double s2[] = { S[0] * S[0], S[1] * S[1], S[2] * S[2] }, theta = sqrt(s2[0] + s2[1] + s2[2]), st_t, ct_t, vt_t;

	if ( theta < SR_EPS )
	{
        st_t = 1.0 - theta * theta / (double)6.0;
        ct_t = 0.5 - theta * theta / (double)24.0;
        vt_t = (S[0] * S[3] + S[1] * S[4] + S[2] * S[5]) * (1.0 - theta * theta / (double)20.0) / (double)6.0;
	} else
	{
        double itheta = 1.0 / theta;
		st_t = sin(theta) * itheta;
		itheta *= itheta;
		ct_t = (1.0 - cos(theta)) * itheta;
		vt_t = (S[0] * S[3] + S[1] * S[4] + S[2] * S[5]) * (1.0 - st_t) * itheta;
	}

	_T[0] = 1.0 - ct_t * (s2[1] + s2[2]);
	_T[1] = ct_t * S[0] * S[1] + st_t * S[2];
	_T[2] = ct_t * S[0] * S[2] - st_t * S[1]; 
	_T[3] = ct_t * S[0] * S[1] - st_t * S[2];
	_T[4] = 1.0 - ct_t * (s2[0] + s2[2]);
	_T[5] = ct_t * S[1] * S[2] + st_t * S[0]; 
	_T[6] = ct_t * S[0] * S[2] + st_t * S[1];
	_T[7] = ct_t * S[1] * S[2] - st_t * S[0];
	_T[8] = 1.0 - ct_t * (s2[0] + s2[1]);
	_T[9] = st_t * S[3] + vt_t * S[0] + ct_t * (S[1] * S[5] - S[2] * S[4]);
	_T[10] = st_t * S[4] + vt_t * S[1] + ct_t * (S[2] * S[3] - S[0] * S[5]);
	_T[11] = st_t * S[5] + vt_t * S[2] + ct_t * (S[0] * S[4] - S[1] * S[3]);
}

inline void SE3::Exp(const se3 &S, double theta)
{
    double s2[] = { S[0] * S[0], S[1] * S[1], S[2] * S[2] };
	if ( fabs(s2[0] + s2[1] + s2[2] - 1.0) > SR_EPS ) return Exp(theta * S);
    double st = sin(theta), vt = 1.0 - cos(theta), ut = (theta - st) * (S[0] * S[3] + S[1] * S[4] + S[2] * S[5]), sts[] = { st * S[0], st * S[1], st * S[2] };

	_T[0] = 1.0 + vt * (s2[0] - 1.0);
	_T[1] = vt * S[0] * S[1] + sts[2];
	_T[2] = vt * S[0] * S[2] - sts[1];
	_T[3] = vt * S[0] * S[1] - sts[2];
	_T[4] = 1.0 + vt * (s2[1] - 1.0);
	_T[5] = vt * S[1] * S[2] + sts[0];
	_T[6] = vt * S[0] * S[2] + sts[1];
	_T[7] = vt * S[1] * S[2] - sts[0];
	_T[8] = 1.0 + vt * (s2[2] - 1.0);
	_T[9] = st * S[3] + ut * S[0] + vt * (S[1] * S[5] - S[2] * S[4]);
	_T[10] = st * S[4] + ut * S[1] + vt * (S[2] * S[3] - S[0] * S[5]);
	_T[11] = st * S[5] + ut * S[2] + vt * (S[0] * S[4] - S[1] * S[3]);
}

template <class TYPE>
inline void SE3::ToArray(TYPE M[]) const
{
	M[0] = (TYPE)_T[0];
	M[1] = (TYPE)_T[1];
	M[2] = (TYPE)_T[2];
	M[3] = (TYPE)0.0;
	M[4] = (TYPE)_T[3];
	M[5] = (TYPE)_T[4];
	M[6] = (TYPE)_T[5];
	M[7] = (TYPE)0.0;
	M[8] = (TYPE)_T[6];
	M[9] = (TYPE)_T[7];
	M[10] = (TYPE)_T[8];
	M[11] = (TYPE)0.0;
	M[12] = (TYPE)_T[9];
	M[13] = (TYPE)_T[10];
	M[14] = (TYPE)_T[11];
	M[15] = (TYPE)1.0;
}

inline SE3 RotX(double t)
{
    double c = cos(t), s = sin(t);
	return SE3(1.0, 0.0, 0.0, 0.0, c, s, 0.0, -s, c);
}

inline SE3 RotY(double t)
{
    double c = cos(t), s = sin(t);
	return SE3(c, 0.0, -s, 0.0, 1.0, 0.0, s, 0.0, c);
}

inline SE3 RotZ(double t)
{
    double c = cos(t), s = sin(t);
	return SE3(c, s, 0.0, -s, c, 0.0, 0.0, 0.0, 1.0);
}

inline SE3 EulerZYX(const Vec3 &angle)
{
    double ca = cos(angle[0]), sa = sin(angle[0]), cb = cos(angle[1]), sb = sin(angle[1]), cg = cos(angle[2]), sg = sin(angle[2]);
	return SE3(ca * cb, sa * cb, -sb, ca * sb * sg - sa * cg, sa * sb * sg + ca * cg, cb * sg, ca * sb * cg + sa * sg, sa * sb * cg - ca * sg, cb * cg);
}

inline SE3 EulerZYZ(const Vec3 &angle)
{
    double ca = cos(angle[0]), sa = sin(angle[0]), cb = cos(angle[1]), sb = sin(angle[1]), cg = cos(angle[2]), sg = sin(angle[2]);
	return SE3(ca * cb * cg - sa * sg, sa * cb * cg + ca * sg, -sb * cg,	-ca * cb * sg - sa * cg, ca * cg - sa * cb * sg, sb * sg, ca * sb, sa * sb, cb);
}

inline SE3 EulerXYZ(const Vec3 &angle, const Vec3 &position)
{
	SE3 T = RotX(angle[0]) * RotY(angle[1]) * RotZ(angle[2]);
	T.SetPosition(position);
	return T;
}

inline SE3 EulerZYX(const Vec3 &angle, const Vec3 &pos)
{
    double ca = cos(angle[0]), sa = sin(angle[0]), cb = cos(angle[1]), sb = sin(angle[1]), cg = cos(angle[2]), sg = sin(angle[2]);
	return SE3(ca * cb, sa * cb, -sb, ca * sb * sg - sa * cg, sa * sb * sg + ca * cg, cb * sg, ca * sb * cg + sa * sg, sa * sb * cg - ca * sg, cb * cg, pos[0], pos[1], pos[2]);
}

inline SE3 EulerZYZ(const Vec3 &angle, const Vec3 &pos)
{
    double ca = cos(angle[0]), sa = sin(angle[0]), cb = cos(angle[1]), sb = sin(angle[1]), cg = cos(angle[2]), sg = sin(angle[2]);
	return SE3(ca * cb * cg - sa * sg, sa * cb * cg + ca * sg, -sb * cg,	-ca * cb * sg - sa * cg, ca * cg - sa * cb * sg, sb * sg, ca * sb, sa * sb, cb, pos[0], pos[1], pos[2]);
}

// R = Exp(w)
// p = sin(t) / t * v + (t - sin(t)) / t^3 * <w, v> * w + (1 - cos(t)) / t^2 * (w X v)
// , when S = (w, v), t = |w|
inline SE3 Exp(const se3 &S)
{
    double s2[] = { S[0] * S[0], S[1] * S[1], S[2] * S[2] }, theta = sqrt(s2[0] + s2[1] + s2[2]), st_t, ct_t, vt_t;

	if ( theta < SR_EPS )
	{
        st_t = 1.0 - theta * theta / (double)6.0;
        ct_t = 0.5 - theta * theta / (double)24.0;
        vt_t = (S[0] * S[3] + S[1] * S[4] + S[2] * S[5]) * (1.0 - theta * theta / (double)20.0) / (double)6.0;
	} else
	{
        double itheta = 1.0 / theta;
		st_t = sin(theta) * itheta;
		itheta *= itheta;
		ct_t = (1.0 - cos(theta)) * itheta;
		vt_t = (S[0] * S[3] + S[1] * S[4] + S[2] * S[5]) * (1.0 - st_t) * itheta;
	}

	return SE3( 1.0 - ct_t * (s2[1] + s2[2]), ct_t * S[0] * S[1] + st_t * S[2], ct_t * S[0] * S[2] - st_t * S[1], 
		ct_t * S[0] * S[1] - st_t * S[2], 1.0 - ct_t * (s2[0] + s2[2]), ct_t * S[1] * S[2] + st_t * S[0], 
		ct_t * S[0] * S[2] + st_t * S[1], ct_t * S[1] * S[2] - st_t * S[0], 1.0 - ct_t * (s2[0] + s2[1]),
		st_t * S[3] + vt_t * S[0] + ct_t * (S[1] * S[5] - S[2] * S[4]),
		st_t * S[4] + vt_t * S[1] + ct_t * (S[2] * S[3] - S[0] * S[5]),
		st_t * S[5] + vt_t * S[2] + ct_t * (S[0] * S[4] - S[1] * S[3]));
}

inline SE3 Exp(const se3 &S, double theta)
{
    double s2[] = { S[0] * S[0], S[1] * S[1], S[2] * S[2] };
	if ( fabs(s2[0] + s2[1] + s2[2] - 1.0) > SR_EPS ) return Exp(theta * S);
    double st = sin(theta), vt = 1.0 - cos(theta), ut = (theta - st) * (S[0] * S[3] + S[1] * S[4] + S[2] * S[5]), sts[] = { st * S[0], st * S[1], st * S[2] };
	return SE3( 1.0 + vt * (s2[0] - 1.0), vt * S[0] * S[1] + sts[2], vt * S[0] * S[2] - sts[1],
		vt * S[0] * S[1] - sts[2], 1.0 + vt * (s2[1] - 1.0), vt * S[1] * S[2] + sts[0], 
		vt * S[0] * S[2] + sts[1], vt * S[1] * S[2] - sts[0], 1.0 + vt * (s2[2] - 1.0),
		st * S[3] + ut * S[0] + vt * (S[1] * S[5] - S[2] * S[4]),
		st * S[4] + ut * S[1] + vt * (S[2] * S[3] - S[0] * S[5]),
		st * S[5] + ut * S[2] + vt * (S[0] * S[4] - S[1] * S[3]));
}

// I + sin(t) / t * [S] + (1 - cos(t)) / t^2 * [S]^2, where t = |S|
inline SE3 Exp(const Axis &S)
{
    double s2[] = { S[0] * S[0], S[1] * S[1], S[2] * S[2] }, theta = sqrt(s2[0] + s2[1] + s2[2]), st_t, ct_t;

	if ( theta < SR_EPS )
	{
        st_t = 1.0 - theta * theta / (double)6.0;
        ct_t = 0.5 - theta * theta / (double)24.0;
	} else
	{
		st_t = sin(theta) / theta;
		ct_t = (1.0 - cos(theta)) / theta / theta;
	}

	return SE3( 1.0 - ct_t * (s2[1] + s2[2]), ct_t * S[0] * S[1] + st_t * S[2], ct_t * S[0] * S[2] - st_t * S[1], 
		ct_t * S[0] * S[1] - st_t * S[2], 1.0 - ct_t * (s2[0] + s2[2]), ct_t * S[1] * S[2] + st_t * S[0], 
		ct_t * S[0] * S[2] + st_t * S[1], ct_t * S[1] * S[2] - st_t * S[0], 1.0 - ct_t * (s2[0] + s2[1]));
}

inline SE3 Exp(const Axis &S, double theta)
{
    double s2[] = { S[0] * S[0], S[1] * S[1], S[2] * S[2] };
	if ( fabs(s2[0] + s2[1] + s2[2] - 1.0) > SR_EPS ) return Exp(theta * S);
    double st = sin(theta), vt = 1.0 - cos(theta), sts[] = { st * S[0], st * S[1], st * S[2] };
	return SE3( 1.0 + vt * (s2[0] - 1.0), vt * S[0] * S[1] + sts[2], vt * S[0] * S[2] - sts[1],
		vt * S[0] * S[1] - sts[2], 1.0 + vt * (s2[1] - 1.0), vt * S[1] * S[2] + sts[0], 
		vt * S[0] * S[2] + sts[1], vt * S[1] * S[2] - sts[0], 1.0 + vt * (s2[2] - 1.0));
}

inline SE3 Inv(const SE3 &T)
{
	return SE3(	T[0], T[3], T[6], T[1], T[4], T[7], T[2], T[5], T[8],
		-T[0] * T[9] - T[1] * T[10] - T[2] * T[11],
		-T[3] * T[9] - T[4] * T[10] - T[5] * T[11],
		-T[6] * T[9] - T[7] * T[10] - T[8] * T[11]);
}

inline se3 Linearize(const SE3 &T)	// invskew(T - I)
{
	return se3(0.5 * (T[5] - T[7]), 0.5 * (T[6] - T[2]), 0.5 * (T[1] - T[3]), T[9], T[10], T[11]);
}

inline Inertia::Inertia()
{
	_I[0] = _I[1] = _I[2] = _I[3] = _I[4] = _I[5] = _I[6] = _I[7] = _I[8] = _I[9] = 0.0;
}

inline Inertia::Inertia(double m)
{
	assert(m >= 0 && "A mass of this inertia tensor is not positive definite.");
	_I[9] = _I[0] = _I[1] = _I[2] = m;
	_I[3] = _I[4] = _I[5] = _I[6] = _I[7] = _I[8] = 0.0;
}

inline Inertia::Inertia(double mass, double Ixx, double Iyy, double Izz)
{
	assert(mass >= 0 && Ixx >= 0 && Iyy >= 0 && Izz >= 0 && "A mass of this inertia tensor is not positive definite.");
	_I[9] = mass;
	_I[0] = Ixx; _I[1] = Iyy; _I[2] = Izz;
	_I[3] = _I[4] = _I[5] = _I[6] = _I[7] = _I[8] = 0.0;
}

inline Inertia::Inertia(double Ixx, double Iyy, double Izz, double Ixy, double Iyz, double Izx)
{
	_I[0] = Ixx;
	_I[1] = Iyy;
	_I[2] = Izz;
	_I[3] = Ixy;
	_I[4] = Iyz;
	_I[5] = Izx;

	_I[9] = 0.0;					// mass is zero. MUST be specified.
	_I[6] = _I[7] = _I[8] = 0.0;
}

inline Inertia::Inertia(double I0, double I1, double I2, double I3, double I4, double I5, double r0, double r1, double r2, double m)
{
	_I[0] = I0;
	_I[1] = I1;
	_I[2] = I2;
	_I[3] = I3;
	_I[4] = I4;
	_I[5] = I5;
	_I[6] = r0;
	_I[7] = r1;
	_I[8] = r2;
	_I[9] = m;	
}

inline Inertia Inertia::Transform(const SE3 &T) const
{
    double j0 = _I[0] + _I[9] * T[11] * T[11] + _I[9] * T[10] * T[10] - 2.0 * _I[8] * T[11] - 2.0 * _I[7] * T[10];
    double j1 = _I[1] + _I[9] * T[11] * T[11] + _I[9] * T[9] * T[9] - 2.0 * _I[8] * T[11] - 2.0 * _I[6] * T[9];
    double j2 = _I[2] + _I[9] * T[10] * T[10] + _I[9] * T[9] * T[9] - 2.0 * _I[7] * T[10] - 2.0 * _I[6] * T[9];
    double j3 = _I[3] + T[10] * _I[6] + T[9] * _I[7] - _I[9] * T[10] * T[9];
    double j4 = _I[4] + T[11] * _I[6] + T[9] * _I[8] - _I[9] * T[11] * T[9];
    double j5 = _I[5] + T[11] * _I[7] + T[10] * _I[8] - _I[9] * T[11] * T[10];
    double t0 = T[0] * j0 + T[1] * j3 + T[2] * j4;
    double t1 = T[3] * j0 + T[4] * j3 + T[5] * j4;
    double t2 = T[6] * j0 + T[7] * j3 + T[8] * j4;
    double t3 = T[0] * j3 + T[1] * j1 + T[2] * j5;
    double t4 = T[3] * j3 + T[4] * j1 + T[5] * j5;
    double t5 = T[6] * j3 + T[7] * j1 + T[8] * j5;
    double t6 = T[0] * j4 + T[1] * j5 + T[2] * j2;
    double t7 = T[3] * j4 + T[4] * j5 + T[5] * j2;
    double t8 = T[6] * j4 + T[7] * j5 + T[8] * j2;

	return Inertia(	t0 * T[0] + t3 * T[1] + t6 * T[2],
		t1 * T[3] + t4 * T[4] + t7 * T[5],
		t2 * T[6] + t5 * T[7] + t8 * T[8],
		t1 * T[0] + t4 * T[1] + t7 * T[2],
		t2 * T[0] + t5 * T[1] + t8 * T[2],
		t2 * T[3] + t5 * T[4] + t8 * T[5],
		T[0] * (_I[6] - _I[9] * T[9]) + T[1] * (_I[7] - _I[9] * T[10]) + T[2] * (_I[8] - _I[9] * T[11]),
		T[3] * (_I[6] - _I[9] * T[9]) + T[4] * (_I[7] - _I[9] * T[10]) + T[5] * (_I[8] - _I[9] * T[11]),
		T[6] * (_I[6] - _I[9] * T[9]) + T[7] * (_I[7] - _I[9] * T[10]) + T[8] * (_I[8] - _I[9] * T[11]),
		_I[9]);
}

inline dse3 Inertia::operator * (const se3 &s) const
{
	return dse3(_I[0] * s[0] + _I[3] * s[1] + _I[4] * s[2] + _I[7] * s[5] - _I[8] * s[4],
		_I[3] * s[0] + _I[1] * s[1] + _I[5] * s[2] + _I[8] * s[3] - _I[6] * s[5],
		_I[4] * s[0] + _I[5] * s[1] + _I[2] * s[2] + _I[6] * s[4] - _I[7] * s[3],
		s[1] * _I[8] - s[2] * _I[7] + _I[9] * s[3],
		s[2] * _I[6] - s[0] * _I[8] + _I[9] * s[4],
		s[0] * _I[7] - s[1] * _I[6] + _I[9] * s[5]);
}

inline dse3 Inertia::operator * (const Axis &s) const
{
	return dse3(_I[0] * s[0] + _I[3] * s[1] + _I[4] * s[2],
		_I[3] * s[0] + _I[1] * s[1] + _I[5] * s[2],
		_I[4] * s[0] + _I[5] * s[1] + _I[2] * s[2],
		s[1] * _I[8] - s[2] * _I[7],
		s[2] * _I[6] - s[0] * _I[8],
		s[0] * _I[7] - s[1] * _I[6]);
}

inline dse3 Inertia::operator * (const Vec3 &s) const
{
	return dse3(_I[7] * s[2] - _I[8] * s[1],
		_I[8] * s[0] - _I[6] * s[2],
		_I[6] * s[1] - _I[7] * s[0],
		_I[9] * s[0], _I[9] * s[1], _I[9] * s[2]);
}

inline double &Inertia::operator [] (int i)
{
	return _I[i];
}

inline const double	&Inertia::operator [] (int i) const
{
	return _I[i];
}

template <class TYPE>
inline void Inertia::ToArray(TYPE M[]) const
{
	M[0] =  (TYPE)_I[0];		M[6]  =  (TYPE)_I[3];		M[12] =  (TYPE)_I[4];		M[18] =  (TYPE)0.0;			M[24] = -(TYPE)_I[8];		M[30] =  (TYPE)_I[7];
	M[1] =  (TYPE)_I[3];		M[7]  =  (TYPE)_I[1];		M[13] =  (TYPE)_I[5];		M[19] =  (TYPE)_I[8];		M[25] =  (TYPE)0.0;			M[31] = -(TYPE)_I[6];
	M[2] =  (TYPE)_I[4];		M[8]  =  (TYPE)_I[5];		M[14] =  (TYPE)_I[2];		M[20] = -(TYPE)_I[7];		M[26] =  (TYPE)_I[6];		M[32] =  (TYPE)0.0;
	M[3] =  (TYPE)0.0;			M[9]  =  (TYPE)_I[8];		M[15] = -(TYPE)_I[7];		M[21] =  (TYPE)_I[9];		M[27] =  (TYPE)0.0;			M[33] =  (TYPE)0.0;
	M[4] = -(TYPE)_I[8];		M[10] =  (TYPE)0.0;			M[16] =  (TYPE)_I[6];		M[22] =  (TYPE)0.0;			M[28] =  (TYPE)_I[9];		M[34] =  (TYPE)0.0;
	M[5] =  (TYPE)_I[7];		M[11] = -(TYPE)_I[6];		M[17] =  (TYPE)0.0;			M[23] =  (TYPE)0.0;			M[29] =  (TYPE)0.0;			M[35] =  (TYPE)_I[9];
}

inline const Inertia &Inertia::operator = (const Inertia &I)
{
	_I[0] = I[0];
	_I[1] = I[1];
	_I[2] = I[2];
	_I[3] = I[3];
	_I[4] = I[4];
	_I[5] = I[5];
	_I[6] = I[6];
	_I[7] = I[7];
	_I[8] = I[8];
	_I[9] = I[9];
	return *this;
}

inline Inertia Inertia::operator + (const Inertia &I) const
{
	return Inertia(	_I[0] + I[0], 
		_I[1] + I[1], 
		_I[2] + I[2], 
		_I[3] + I[3], 
		_I[4] + I[4], 
		_I[5] + I[5], 
		_I[6] + I[6], 
		_I[7] + I[7], 
		_I[8] + I[8], 
		_I[9] + I[9]);
}

inline const Inertia &Inertia::operator *= (double x)
{
	_I[0] *= x;
	_I[1] *= x;
	_I[2] *= x;
	_I[3] *= x;
	_I[4] *= x;
	_I[5] *= x;
	_I[6] *= x;
	_I[7] *= x;
	_I[8] *= x;
	_I[9] *= x;
	return *this;
}

inline Inertia operator * (double x, const Inertia &I)
{
	return Inertia(	x * I[0], 
		x * I[1], 
		x * I[2], 
		x * I[3], 
		x * I[4], 
		x * I[5], 
		x * I[6], 
		x * I[7], 
		x * I[8], 
		x * I[9]);
}

inline double Inertia::GetMass(void) const
{
	return _I[9];
}

inline Vec3 Inertia::GetDiag(void) const
{
	return Vec3(_I[0], _I[1], _I[2]);
}

inline Vec3 Inertia::GetOffset(void) const
{
	return Vec3(_I[6], _I[7], _I[8]);
}

inline void Inertia::SetMass(double m)
{
	_I[9] = m;
}

inline void Inertia::SetAngularMoment(double Ixx, double Iyy, double Izz, double Ixy, double Iyz, double Izx)
{
	_I[0] = Ixx;
	_I[1] = Iyy;
	_I[2] = Izz;
	_I[3] = Ixy;
	_I[4] = Iyz;
	_I[5] = Izx;
}

inline void Inertia::SetOffset(Vec3 offset)
{
	_I[6] = offset[0];
	_I[7] = offset[1];
	_I[8] = offset[2];
}

inline AInertia::AInertia()
{
}

inline AInertia::AInertia(double d)
{
	_J[0] = _J[1] = _J[2] = _J[3] = _J[4] = _J[5] = _J[6] = _J[7] = _J[8] = _J[9] = _J[10] = _J[11] = _J[12] = _J[13] = _J[14] = _J[15] = _J[16] = _J[17] = _J[18] = _J[19] = _J[20] = d;
}

inline AInertia::AInertia(const Inertia &I)
{
	_J[0] = I[0];	_J[1] = I[3];	_J[2] = I[4];	_J[3] = 0.0;	_J[4] = -I[8];			_J[5] = I[7];
	_J[6] = I[1];	_J[7] = I[5];	_J[8] = I[8];			_J[9] = 0.0;	_J[10] = -I[6];
	_J[11] = I[2];	_J[12] = -I[7];			_J[13] = I[6];			_J[14] = 0.0;
	_J[15] = I[9];			_J[16] = 0.0;	_J[17] = 0.0;
	_J[18] = I[9];			_J[19] = 0.0;
	_J[20] = I[9];
}

inline AInertia::AInertia(double a0, double a1, double a2, double a3, double a4, double a5, double a6, double a7, double a8, double a9, double a10, double a11, double a12, double a13, double a14, double a15, double a16, double a17, double a18, double a19, double a20)
{
	_J[0] = a0;		_J[1] = a1;		_J[2] = a2;
	_J[3] = a3;		_J[4] = a4;		_J[5] = a5;
	_J[6] = a6;		_J[7] = a7;		_J[8] = a8;
	_J[9] = a9;		_J[10] = a10;	_J[11] = a11;
	_J[12] = a12;	_J[13] = a13;	_J[14] = a14;
	_J[15] = a15;	_J[16] = a16;	_J[17] = a17;
	_J[18] = a18;	_J[19] = a19;	_J[20] = a20;
}

inline const AInertia &AInertia::operator + (void) const
{
	return *this;
}

inline AInertia AInertia::operator - (void) const
{
	return AInertia(-_J[0], -_J[1], -_J[2], -_J[3], -_J[4], -_J[5], -_J[6], -_J[7], -_J[8], -_J[9], -_J[10], -_J[11], -_J[12], -_J[13], -_J[14], -_J[15], -_J[16], -_J[17], -_J[18], -_J[19], -_J[20]);
}

inline dse3 AInertia::operator * (const se3 &a) const
{
	return dse3(_J[0] * a[0] + _J[1] * a[1] + _J[2] * a[2] + _J[3] * a[3] + _J[4] * a[4] + _J[5] * a[5],
		_J[1] * a[0] + _J[6] * a[1] + _J[7] * a[2] + _J[8] * a[3] + _J[9] * a[4] + _J[10] * a[5],
		_J[2] * a[0] + _J[7] * a[1] + _J[11] * a[2] + _J[12] * a[3] + _J[13] * a[4] + _J[14] * a[5],
		_J[3] * a[0] + _J[8] * a[1] + _J[12] * a[2] + _J[15] * a[3] + _J[16] * a[4] + _J[17] * a[5],
		_J[4] * a[0] + _J[9] * a[1] + _J[13] * a[2] + _J[16] * a[3] + _J[18] * a[4] + _J[19] * a[5],
		_J[5] * a[0] + _J[10] * a[1] + _J[14] * a[2] + _J[17] * a[3] + _J[19] * a[4] + _J[20] * a[5]);
}

inline dse3 AInertia::operator * (const Axis &a) const
{
	return dse3(_J[0] * a[0] + _J[1] * a[1] + _J[2] * a[2],
		_J[1] * a[0] + _J[6] * a[1] + _J[7] * a[2],
		_J[2] * a[0] + _J[7] * a[1] + _J[11] * a[2],
		_J[3] * a[0] + _J[8] * a[1] + _J[12] * a[2],
		_J[4] * a[0] + _J[9] * a[1] + _J[13] * a[2],
		_J[5] * a[0] + _J[10] * a[1] + _J[14] * a[2]);
}

inline double &AInertia::operator [] (int i)
{
	return _J[i];
}

inline const double	&AInertia::operator [] (int i) const
{
	return _J[i];
}

inline AInertia AInertia::operator + (const AInertia &J) const
{
	return AInertia(_J[0] + J[0], _J[1] + J[1], _J[2] + J[2], _J[3] + J[3], _J[4] + J[4], _J[5] + J[5], _J[6] + J[6], _J[7] + J[7], _J[8] + J[8], _J[9] + J[9], _J[10] + J[10], _J[11] + J[11], _J[12] + J[12], _J[13] + J[13], _J[14] + J[14], _J[15] + J[15], _J[16] + J[16], _J[17] + J[17], _J[18] + J[18], _J[19] + J[19], _J[20] + J[20]);
}

inline AInertia AInertia::operator + (const Inertia &J) const
{
	return AInertia(_J[0] + J[0], _J[1] + J[3], _J[2] + J[4], _J[3], _J[4] - J[8], _J[5] + J[7], _J[6] + J[1], _J[7] + J[5], _J[8] + J[8], _J[9], _J[10] - J[6], _J[11] + J[2], _J[12] - J[7], _J[13] + J[6], _J[14], _J[15] + J[9], _J[16], _J[17], _J[18] + J[9], _J[19], _J[20] + J[9]);
}

inline AInertia AInertia::operator - (const AInertia &J) const
{
	return AInertia(_J[0] - J[0], _J[1] - J[1], _J[2] - J[2], _J[3] - J[3], _J[4] - J[4], _J[5] - J[5], _J[6] - J[6], _J[7] - J[7], _J[8] - J[8], _J[9] - J[9], _J[10] - J[10], _J[11] - J[11], _J[12] - J[12], _J[13] - J[13], _J[14] - J[14], _J[15] - J[15], _J[16] - J[16], _J[17] - J[17], _J[18] - J[18], _J[19] - J[19], _J[20] - J[20]);
}

inline AInertia AInertia::operator - (const Inertia &J) const
{
	return AInertia(_J[0] - J[0], _J[1] - J[3], _J[2] - J[4], _J[3], _J[4] + J[8], _J[5] - J[7], _J[6] - J[1], _J[7] - J[5], _J[8] - J[8], _J[9], _J[10] + J[6], _J[11] - J[2], _J[12] + J[7], _J[13] - J[6], _J[14], _J[15] - J[9], _J[16], _J[17], _J[18] - J[9], _J[19], _J[20] - J[9]);
}

inline const AInertia &AInertia::operator += (const AInertia &J)
{
	_J[0] += J[0]; _J[1] += J[1]; _J[2] += J[2]; _J[3] += J[3]; _J[4] += J[4]; _J[5] += J[5]; _J[6] += J[6]; _J[7] += J[7]; _J[8] += J[8]; _J[9] += J[9]; _J[10] += J[10]; _J[11] += J[11]; _J[12] += J[12]; _J[13] += J[13]; _J[14] += J[14]; _J[15] += J[15]; _J[16] += J[16]; _J[17] += J[17]; _J[18] += J[18]; _J[19] += J[19]; _J[20] += J[20];
	return *this;
}

inline const AInertia &AInertia::operator -= (const AInertia &J)
{
	_J[0] -= J[0]; _J[1] -= J[1]; _J[2] -= J[2]; _J[3] -= J[3]; _J[4] -= J[4]; _J[5] -= J[5]; _J[6] -= J[6]; _J[7] -= J[7]; _J[8] -= J[8]; _J[9] -= J[9]; _J[10] -= J[10]; _J[11] -= J[11]; _J[12] -= J[12]; _J[13] -= J[13]; _J[14] -= J[14]; _J[15] -= J[15]; _J[16] -= J[16]; _J[17] -= J[17]; _J[18] -= J[18]; _J[19] -= J[19]; _J[20] -= J[20];
	return *this;
}

inline const AInertia &AInertia::operator += (const Inertia &J)
{
	_J[0] += J[0]; _J[1] += J[3]; _J[2] += J[4]; _J[4] -= J[8]; _J[5] += J[7]; _J[6] += J[1]; _J[7] += J[5]; _J[8] += J[8]; _J[10] -= J[6]; _J[11] += J[2]; _J[12] -= J[7]; _J[13] += J[6]; _J[15] += J[9]; _J[18] += J[9]; _J[20] += J[9];
	return *this;
}

inline const AInertia &AInertia::operator -= (const Inertia &J)
{
	_J[0] -= J[0]; _J[1] -= J[3]; _J[2] -= J[4]; _J[4] -= J[8]; _J[5] -= J[7]; _J[6] -= J[1]; _J[7] -= J[5]; _J[8] -= J[8]; _J[10] -= J[6]; _J[11] -= J[2]; _J[12] -= J[7]; _J[13] -= J[6]; _J[15] -= J[9]; _J[18] -= J[9]; _J[20] -= J[9];
	return *this;
}

template <class TYPE>
inline void AInertia::ToArray(TYPE M[]) const
{
	M[0] = (TYPE)_J[0]; M[6] = (TYPE)_J[1];  M[12] = (TYPE)_J[2];  M[18] = (TYPE)_J[3];  M[24] = (TYPE)_J[4];  M[30] = (TYPE)_J[5];
	M[1] = (TYPE)_J[1]; M[7] = (TYPE)_J[6];  M[13] = (TYPE)_J[7];  M[19] = (TYPE)_J[8];  M[25] = (TYPE)_J[9];  M[31] = (TYPE)_J[10];
	M[2] = (TYPE)_J[2]; M[8] = (TYPE)_J[7];  M[14] = (TYPE)_J[11]; M[20] = (TYPE)_J[12]; M[26] = (TYPE)_J[13]; M[32] = (TYPE)_J[14];
	M[3] = (TYPE)_J[3]; M[9] = (TYPE)_J[8];  M[15] = (TYPE)_J[12]; M[21] = (TYPE)_J[15]; M[27] = (TYPE)_J[16]; M[33] = (TYPE)_J[17];
	M[4] = (TYPE)_J[4]; M[10] = (TYPE)_J[9];  M[16] = (TYPE)_J[13]; M[22] = (TYPE)_J[16]; M[28] = (TYPE)_J[18]; M[34] = (TYPE)_J[19];
	M[5] = (TYPE)_J[5]; M[11] = (TYPE)_J[10]; M[17] = (TYPE)_J[14]; M[23] = (TYPE)_J[17]; M[29] = (TYPE)_J[19]; M[35] = (TYPE)_J[20];
}

inline AInertia AInertia::Transform(const SE3 &T) const
{
    double d0 = _J[3] + T[11] * _J[16] - T[10] * _J[17];
    double d1 = _J[8] - T[11] * _J[15] + T[9] * _J[17];
    double d2 = _J[12] + T[10] * _J[15] - T[9] * _J[16];
    double d3 = _J[4] + T[11] * _J[18] - T[10] * _J[19];
    double d4 = _J[9] - T[11] * _J[16] + T[9] * _J[19];
    double d5 = _J[13] + T[10] * _J[16] - T[9] * _J[18];
    double d6 = _J[5] + T[11] * _J[19] - T[10] * _J[20];
    double d7 = _J[10] - T[11] * _J[17] + T[9] * _J[20];
    double d8 = _J[14] + T[10] * _J[17] - T[9] * _J[19];
    double e0 = _J[0] + T[11] * _J[4] - T[10] * _J[5] + d3 * T[11] - d6 * T[10];
    double e3 = _J[1] + T[11] * _J[9] - T[10] * _J[10] - d0 * T[11] + d6 * T[9];
    double e4 = _J[6] - T[11] * _J[8] + T[9] * _J[10] - d1 * T[11] + d7 * T[9];
    double e6 = _J[2] + T[11] * _J[13] - T[10] * _J[14] + d0 * T[10] - d3 * T[9];
    double e7 = _J[7] - T[11] * _J[12] + T[9] * _J[14] + d1 * T[10] - d4 * T[9];
    double e8 = _J[11] + T[10] * _J[12] - T[9] * _J[13] + d2 * T[10] - d5 * T[9];
    double f0 = T[0] * e0 + T[1] * e3 + T[2] * e6;
    double f1 = T[0] * e3 + T[1] * e4 + T[2] * e7;
    double f2 = T[0] * e6 + T[1] * e7 + T[2] * e8;
    double f3 = T[0] * d0 + T[1] * d1 + T[2] * d2;
    double f4 = T[0] * d3 + T[1] * d4 + T[2] * d5;
    double f5 = T[0] * d6 + T[1] * d7 + T[2] * d8;
    double f6 = T[3] * e0 + T[4] * e3 + T[5] * e6;
    double f7 = T[3] * e3 + T[4] * e4 + T[5] * e7;
    double f8 = T[3] * e6 + T[4] * e7 + T[5] * e8;
    double g0 = T[3] * d0 + T[4] * d1 + T[5] * d2;
    double g1 = T[3] * d3 + T[4] * d4 + T[5] * d5;
    double g2 = T[3] * d6 + T[4] * d7 + T[5] * d8;
    double g3 = T[6] * d0 + T[7] * d1 + T[8] * d2;
    double g4 = T[6] * d3 + T[7] * d4 + T[8] * d5;
    double g5 = T[6] * d6 + T[7] * d7 + T[8] * d8;
    double h0 = T[0] * _J[15] + T[1] * _J[16] + T[2] * _J[17];
    double h1 = T[0] * _J[16] + T[1] * _J[18] + T[2] * _J[19];
    double h2 = T[0] * _J[17] + T[1] * _J[19] + T[2] * _J[20];
    double h3 = T[3] * _J[15] + T[4] * _J[16] + T[5] * _J[17];
    double h4 = T[3] * _J[16] + T[4] * _J[18] + T[5] * _J[19];
    double h5 = T[3] * _J[17] + T[4] * _J[19] + T[5] * _J[20];

	return AInertia(f0 * T[0] + f1 * T[1] + f2 * T[2],
		f0 * T[3] + f1 * T[4] + f2 * T[5],
		f0 * T[6] + f1 * T[7] + f2 * T[8],
		f3 * T[0] + f4 * T[1] + f5 * T[2],
		f3 * T[3] + f4 * T[4] + f5 * T[5],
		f3 * T[6] + f4 * T[7] + f5 * T[8],
		f6 * T[3] + f7 * T[4] + f8 * T[5],
		f6 * T[6] + f7 * T[7] + f8 * T[8],
		g0 * T[0] + g1 * T[1] + g2 * T[2],
		g0 * T[3] + g1 * T[4] + g2 * T[5],
		g0 * T[6] + g1 * T[7] + g2 * T[8],
		(T[6] * e0 + T[7] * e3 + T[8] * e6) * T[6] + (T[6] * e3 + T[7] * e4 + T[8] * e7) * T[7] + (T[6] * e6 + T[7] * e7 + T[8] * e8) * T[8],
		g3 * T[0] + g4 * T[1] + g5 * T[2],
		g3 * T[3] + g4 * T[4] + g5 * T[5],
		g3 * T[6] + g4 * T[7] + g5 * T[8],
		h0 * T[0] + h1 * T[1] + h2 * T[2],
		h0 * T[3] + h1 * T[4] + h2 * T[5],
		h0 * T[6] + h1 * T[7] + h2 * T[8],
		h3 * T[3] + h4 * T[4] + h5 * T[5],
		h3 * T[6] + h4 * T[7] + h5 * T[8],
		(T[6] * _J[15] + T[7] * _J[16] + T[8] * _J[17]) * T[6] + (T[6] * _J[16] + T[7] * _J[18] + T[8] * _J[19]) * T[7] + (T[6] * _J[17] + T[7] * _J[19] + T[8] * _J[20]) * T[8]);
}

inline void AInertia::AddTransform(const AInertia &J, const SE3 &T)
{	
    double d0 = J[3] + T[11] * J[16] - T[10] * J[17];
    double d1 = J[8] - T[11] * J[15] + T[9] * J[17];
    double d2 = J[12] + T[10] * J[15] - T[9] * J[16];
    double d3 = J[4] + T[11] * J[18] - T[10] * J[19];
    double d4 = J[9] - T[11] * J[16] + T[9] * J[19];
    double d5 = J[13] + T[10] * J[16] - T[9] * J[18];
    double d6 = J[5] + T[11] * J[19] - T[10] * J[20];
    double d7 = J[10] - T[11] * J[17] + T[9] * J[20];
    double d8 = J[14] + T[10] * J[17] - T[9] * J[19];
    double e0 = J[0] + T[11] * J[4] - T[10] * J[5] + d3 * T[11] - d6 * T[10];
    double e3 = J[1] + T[11] * J[9] - T[10] * J[10] - d0 * T[11] + d6 * T[9];
    double e4 = J[6] - T[11] * J[8] + T[9] * J[10] - d1 * T[11] + d7 * T[9];
    double e6 = J[2] + T[11] * J[13] - T[10] * J[14] + d0 * T[10] - d3 * T[9];
    double e7 = J[7] - T[11] * J[12] + T[9] * J[14] + d1 * T[10] - d4 * T[9];
    double e8 = J[11] + T[10] * J[12] - T[9] * J[13] + d2 * T[10] - d5 * T[9];
    double f0 = T[0] * e0 + T[1] * e3 + T[2] * e6;
    double f1 = T[0] * e3 + T[1] * e4 + T[2] * e7;
    double f2 = T[0] * e6 + T[1] * e7 + T[2] * e8;
    double f3 = T[0] * d0 + T[1] * d1 + T[2] * d2;
    double f4 = T[0] * d3 + T[1] * d4 + T[2] * d5;
    double f5 = T[0] * d6 + T[1] * d7 + T[2] * d8;
    double f6 = T[3] * e0 + T[4] * e3 + T[5] * e6;
    double f7 = T[3] * e3 + T[4] * e4 + T[5] * e7;
    double f8 = T[3] * e6 + T[4] * e7 + T[5] * e8;
    double g0 = T[3] * d0 + T[4] * d1 + T[5] * d2;
    double g1 = T[3] * d3 + T[4] * d4 + T[5] * d5;
    double g2 = T[3] * d6 + T[4] * d7 + T[5] * d8;
    double g3 = T[6] * d0 + T[7] * d1 + T[8] * d2;
    double g4 = T[6] * d3 + T[7] * d4 + T[8] * d5;
    double g5 = T[6] * d6 + T[7] * d7 + T[8] * d8;
    double h0 = T[0] * J[15] + T[1] * J[16] + T[2] * J[17];
    //	double h1 = T[0] * J[16] + T[1] * J[18] + T[2] * J[19];
    //	double h2 = T[0] * J[17] + T[1] * J[19] + T[2] * J[20];
    double h3 = T[3] * J[15] + T[4] * J[16] + T[5] * J[17];
    //	double h4 = T[3] * J[16] + T[4] * J[18] + T[5] * J[19];
    //	double h5 = T[3] * J[17] + T[4] * J[19] + T[5] * J[20];

	_J[0] += f0 * T[0] + f1 * T[1] + f2 * T[2];
	_J[1] += f0 * T[3] + f1 * T[4] + f2 * T[5];
	_J[2] += f0 * T[6] + f1 * T[7] + f2 * T[8];
	_J[3] += f3 * T[0] + f4 * T[1] + f5 * T[2];
	_J[4] += f3 * T[3] + f4 * T[4] + f5 * T[5];
	_J[5] += f3 * T[6] + f4 * T[7] + f5 * T[8];
	_J[6] += f6 * T[3] + f7 * T[4] + f8 * T[5];
	_J[7] += f6 * T[6] + f7 * T[7] + f8 * T[8];
	_J[8] += g0 * T[0] + g1 * T[1] + g2 * T[2];
	_J[9] += g0 * T[3] + g1 * T[4] + g2 * T[5];
	_J[10] += g0 * T[6] + g1 * T[7] + g2 * T[8];
	_J[11] += (T[6] * e0 + T[7] * e3 + T[8] * e6) * T[6] + (T[6] * e3 + T[7] * e4 + T[8] * e7) * T[7] + (T[6] * e6 + T[7] * e7 + T[8] * e8) * T[8];
	_J[12] += g3 * T[0] + g4 * T[1] + g5 * T[2];
	_J[13] += g3 * T[3] + g4 * T[4] + g5 * T[5];
	_J[14] += g3 * T[6] + g4 * T[7] + g5 * T[8];
	_J[15] += h0 * T[0] + (T[0] * J[16] + T[1] * J[18] + T[2] * J[19]) * T[1] + (T[0] * J[17] + T[1] * J[19] + T[2] * J[20]) * T[2];
	_J[16] += h0 * T[3] + (T[0] * J[16] + T[1] * J[18] + T[2] * J[19]) * T[4] + (T[0] * J[17] + T[1] * J[19] + T[2] * J[20]) * T[5];
	_J[17] += h0 * T[6] + (T[0] * J[16] + T[1] * J[18] + T[2] * J[19]) * T[7] + (T[0] * J[17] + T[1] * J[19] + T[2] * J[20]) * T[8];
	_J[18] += h3 * T[3] + (T[3] * J[16] + T[4] * J[18] + T[5] * J[19]) * T[4] + (T[3] * J[17] + T[4] * J[19] + T[5] * J[20]) * T[5];
	_J[19] += h3 * T[6] + (T[3] * J[16] + T[4] * J[18] + T[5] * J[19]) * T[7] + (T[3] * J[17] + T[4] * J[19] + T[5] * J[20]) * T[8];
	_J[20] += (T[6] * J[15] + T[7] * J[16] + T[8] * J[17]) * T[6] + (T[6] * J[16] + T[7] * J[18] + T[8] * J[19]) * T[7] + (T[6] * J[17] + T[7] * J[19] + T[8] * J[20]) * T[8];
}

inline se3 AInertia::operator * (const dse3 &f) const
{
	return se3(	_J[0] * f[0] + _J[1] * f[1] + _J[2] * f[2] + _J[3] * f[3] + _J[4] * f[4] + _J[5] * f[5],
		_J[1] * f[0] + _J[6] * f[1] + _J[7] * f[2] + _J[8] * f[3] + _J[9] * f[4] + _J[10] * f[5],
		_J[2] * f[0] + _J[7] * f[1] + _J[11] * f[2] + _J[12] * f[3] + _J[13] * f[4] + _J[14] * f[5],
		_J[3] * f[0] + _J[8] * f[1] + _J[12] * f[2] + _J[15] * f[3] + _J[16] * f[4] + _J[17] * f[5],
		_J[4] * f[0] + _J[9] * f[1] + _J[13] * f[2] + _J[16] * f[3] + _J[18] * f[4] + _J[19] * f[5],
		_J[5] * f[0] + _J[10] * f[1] + _J[14] * f[2] + _J[17] * f[3] + _J[19] * f[4] + _J[20] * f[5]);
}

// 0.5 * ( x * ~y + y * ~x )
inline AInertia KroneckerProduct(const dse3 &x, const dse3 &y)
{
    double y_m0 = 0.5 * y[0];
    double y_m1 = 0.5 * y[1];
    double y_m2 = 0.5 * y[2];
    double y_m3 = 0.5 * y[3];
    double y_m4 = 0.5 * y[4];
    double y_m5 = 0.5 * y[5];

	return AInertia(x[0] * y[0], 
		x[0] * y_m1 + x[1] * y_m0, 
		x[0] * y_m2 + x[2] * y_m0,
		x[0] * y_m3 + x[3] * y_m0,
		x[0] * y_m4 + x[4] * y_m0,
		x[0] * y_m5 + x[5] * y_m0,
		x[1] * y[1], 
		x[1] * y_m2 + x[2] * y_m1,
		x[1] * y_m3 + x[3] * y_m1,
		x[1] * y_m4 + x[4] * y_m1,
		x[1] * y_m5 + x[5] * y_m1,
		x[2] * y[2],
		x[2] * y_m3 + x[3] * y_m2,
		x[2] * y_m4 + x[4] * y_m2,
		x[2] * y_m5 + x[5] * y_m2,
		x[3] * y[3],
		x[3] * y_m4 + x[4] * y_m3,
		x[3] * y_m5 + x[5] * y_m3,
		x[4] * y[4],
		x[4] * y_m5 + x[5] * y_m4,
		x[5] * y[5]);
}

// *this -= KroneckerProduct(x, y)
inline void AInertia::SubtractKroneckerProduct(const dse3 &x, const dse3 &y)
{
    double y_m0 = 0.5 * y[0];
    double y_m1 = 0.5 * y[1];
    double y_m2 = 0.5 * y[2];
    double y_m3 = 0.5 * y[3];
    double y_m4 = 0.5 * y[4];
    double y_m5 = 0.5 * y[5];

	_J[0]  -= x[0] * y[0]; 
	_J[1]  -= x[0] * y_m1 + x[1] * y_m0; 
	_J[2]  -= x[0] * y_m2 + x[2] * y_m0;
	_J[3]  -= x[0] * y_m3 + x[3] * y_m0;
	_J[4]  -= x[0] * y_m4 + x[4] * y_m0;
	_J[5]  -= x[0] * y_m5 + x[5] * y_m0;
	_J[6]  -= x[1] * y[1]; 
	_J[7]  -= x[1] * y_m2 + x[2] * y_m1;
	_J[8]  -= x[1] * y_m3 + x[3] * y_m1;
	_J[9]  -= x[1] * y_m4 + x[4] * y_m1;
	_J[10] -= x[1] * y_m5 + x[5] * y_m1;
	_J[11] -= x[2] * y[2];
	_J[12] -= x[2] * y_m3 + x[3] * y_m2;
	_J[13] -= x[2] * y_m4 + x[4] * y_m2;
	_J[14] -= x[2] * y_m5 + x[5] * y_m2;
	_J[15] -= x[3] * y[3];
	_J[16] -= x[3] * y_m4 + x[4] * y_m3;
	_J[17] -= x[3] * y_m5 + x[5] * y_m3;
	_J[18] -= x[4] * y[4];
	_J[19] -= x[4] * y_m5 + x[5] * y_m4;
	_J[20] -= x[5] * y[5];
}

inline se3 AInertia::operator % (const dse3 &b) const
{
    double a00 = _J[6] * _J[11] - _J[7] * _J[7];
    double a01 = _J[2] * _J[7] - _J[1] * _J[11];
    double a02 = _J[1] * _J[7] - _J[2] * _J[6];
    double idet = 1.0 / (_J[0] * a00 + _J[1] * a01 + _J[2] * a02);
    double a11 = idet * (_J[0] * _J[11] - _J[2] * _J[2]);
    double a12 = idet * (_J[2] * _J[1] - _J[0] * _J[7]);
    double a22 = idet * (_J[0] * _J[6] - _J[1] * _J[1]);
	a00 *= idet;
	a01 *= idet;
	a02 *= idet;	
    double t00 = a00 * _J[3] + a01 * _J[8] + a02 * _J[12];
    double t01 = a00 * _J[4] + a01 * _J[9] + a02 * _J[13];
    double t02 = a00 * _J[5] + a01 * _J[10] + a02 * _J[14];
    double t10 = a01 * _J[3] + a11 * _J[8] + a12 * _J[12];
    double t11 = a01 * _J[4] + a11 * _J[9] + a12 * _J[13];
    double t12 = a01 * _J[5] + a11 * _J[10] + a12 * _J[14];
    double t20 = a02 * _J[3] + a12 * _J[8] + a22 * _J[12];
    double t21 = a02 * _J[4] + a12 * _J[9] + a22 * _J[13];
    double t22 = a02 * _J[5] + a12 * _J[10] + a22 * _J[14];
    double r0 = a00 * b[0] + a01 * b[1] + a02 * b[2];
    double r1 = a01 * b[0] + a11 * b[1] + a12 * b[2];
    double r2 = a02 * b[0] + a12 * b[1] + a22 * b[2];
	a00 = r0;
	a01 = r1;
	a02 = r2;	
    double x0 = b[3] - _J[3] * r0 - _J[8] * r1 - _J[12] * r2;
    double x1 = b[4] - _J[4] * r0 - _J[9] * r1 - _J[13] * r2;
    double x2 = b[5] - _J[5] * r0 - _J[10] * r1 - _J[14] * r2;
    double c00 = _J[15] - _J[3] * t00 - _J[8] * t10 - _J[12] * t20;
    double c01 = _J[16] - _J[3] * t01 - _J[8] * t11 - _J[12] * t21;
    double c11 = _J[18] - _J[4] * t01 - _J[9] * t11 - _J[13] * t21;
    double c02 = _J[17] - _J[3] * t02 - _J[8] * t12 - _J[12] * t22;
    double c12 = _J[19] - _J[4] * t02 - _J[9] * t12 - _J[13] * t22;
    double c22 = _J[20] - _J[5] * t02 - _J[10] * t12 - _J[14] * t22;
    double r3 = c02 * c01 - c00 * c12;
	r0 = c11 * c22 - c12 * c12;
	r1 = c02 * c12 - c01 * c22;
	r2 = c01 * c12 - c02 * c11;
	idet = 1.0 / (c00 * r0 + c01 * r1 + c02 * r2);
	a22 = idet * (r0 * x0 + r1 * x1 + r2 * x2);
	a11 = idet * (r1 * x0 + (c00 * c22 - c02 * c02) * x1 + r3 * x2);
	a12 = idet * (r2 * x0 + r3 * x1 + (c00 * c11 - c01 * c01) * x2);	
	a00 -= t00 * a22 + t01 * a11 + t02 * a12;
	a01 -= t10 * a22 + t11 * a11 + t12 * a12;
	a02 -= t20 * a22 + t21 * a11 + t22 * a12;

	return se3(a00, a01 ,a02, a22, a11, a12);
}


inline AInertia Inv(const Inertia &I)
{
    double im = 1.0 / I[9];
    double ims = sqrt(im);
    double r0 = ims * I[6];
    double r1 = ims * I[7];
    double r2 = ims * I[8];
    double a00 = I[0] - (r1 * r1 + r2 * r2);
    double a11 = I[1] - (r0 * r0 + r2 * r2);
    double a22 = I[2] - (r0 * r0 + r1 * r1);
    double a01 = I[3] + r0 * r1;
    double a02 = I[4] + r0 * r2;
    double a12 = I[5] + r1 * r2;
    double j0 = a11 * a22 - a12 * a12;
    double j6 = a02 * a12 - a01 * a22;
    double j12 = a01 * a12 - a02 * a11;
    double idet = 1.0 / (a00 * j0 + a01 * j6 + a02 * j12);
	j0 *= idet;
	j6 *= idet;
	j12 *= idet;
    double j7 = idet * (a00 * a22 - a02 * a02);
    double j13 = idet * (a02 * a01 - a00 * a12);
    double j14 = idet * (a00 * a11 - a01 * a01);
	r0 *= ims;
	r1 *= ims;
	r2 *= ims;
    double j18 = j12 * r1 - j6 * r2;
    double j19 = j13 * r1 - j7 * r2;
    double j20 = j14 * r1 - j13 * r2;
    double j24 = j0 * r2 - j12 * r0;
    double j25 = j6 * r2 - j13 * r0;
    double j26 = j12 * r2 - j14 * r0;
    double j30 = j6 * r0 - j0 * r1;
    double j31 = j7 * r0 - j6 * r1;
    double j32 = j13 * r0 - j12 * r1;

	return AInertia(j0, j6, j12, j18, j24, j30, j7, j13, j19, j25, j31, j14, j20, j26, j32, r1 * j20 - r2 * j19 + im, r1 * j26 - r2 * j25, r1 * j32 - r2 * j31, r2 * j24 - r0 * j26 + im, r2 * j30 - r0 * j32, r0 * j31 - r1 * j30 + im);
}

inline const AInertia &AInertia::operator = (const AInertia &J)
{
	_J[0] = J[0];	_J[1] = J[1];	_J[2] = J[2];
	_J[3] = J[3];	_J[4] = J[4];	_J[5] = J[5];
	_J[6] = J[6];	_J[7] = J[7];	_J[8] = J[8];
	_J[9] = J[9];	_J[10] = J[10];	_J[11] = J[11];
	_J[12] = J[12];	_J[13] = J[13];	_J[14] = J[14];
	_J[15] = J[15];	_J[16] = J[16];	_J[17] = J[17];
	_J[18] = J[18];	_J[19] = J[19];	_J[20] = J[20];
	return *this;
}

inline const AInertia &AInertia::operator = (const Inertia &I)
{
	_J[0] = I[0];	_J[1] = I[3];	_J[2] = I[4];	_J[3] = 0.0;	_J[4] = -I[8];			_J[5] = I[7];
	_J[6] = I[1];	_J[7] = I[5];	_J[8] = I[8];			_J[9] = 0.0;	_J[10] = -I[6];
	_J[11] = I[2];	_J[12] = -I[7];			_J[13] = I[6];			_J[14] = 0.0;
	_J[15] = I[9];			_J[16] = 0.0;	_J[17] = 0.0;
	_J[18] = I[9];			_J[19] = 0.0;
	_J[20] = I[9];
	return *this;
}

inline Axis::Axis()
{
}

inline Axis::Axis(int d)
{
    _v[0] = _v[1] = _v[2] = (double)d;
}

inline Axis::Axis(double d)
{
	_v[0] = _v[1] = _v[2] = d;
}

inline Axis::Axis(const double v[])
{
	_v[0] = v[0];
	_v[1] = v[1];
	_v[2] = v[2];
}

inline Axis::Axis(double v0, double v1, double v2)
{
	_v[0] = v0;
	_v[1] = v1;
	_v[2] = v2;
}

inline const Axis &Axis::operator + (void) const
{
	return *this;
}

inline Axis Axis::operator - (void) const
{
	return Axis(-_v[0], -_v[1], -_v[2]);
}

inline double &Axis::operator [] (int i)
{
	return _v[i];
}

inline const double &Axis::operator [] (int i) const
{
	return _v[i];
}

inline const Axis &Axis::operator = (const Axis &v)
{
	_v[0] = v[0];
	_v[1] = v[1];
	_v[2] = v[2];
	return *this;
}

inline const Axis &Axis::operator = (const se3 &v)
{
	_v[0] = v[0];
	_v[1] = v[1];
	_v[2] = v[2];
	return *this;
}

inline const Axis &Axis::operator = (double d)
{
	_v[0] = _v[1] = _v[2] = d;
	return *this;
}

inline const Axis &Axis::operator *= (double d)
{
	_v[0] *= d;
	_v[1] *= d;
	_v[2] *= d;
	return *this;
}

inline Axis Axis::operator * (double d) const
{
	return Axis(d * _v[0], d * _v[1], d * _v[2]);
}

inline double Axis::Normalize(void)
{
    double mag = sqrt(_v[0] * _v[0] + _v[1] * _v[1] + _v[2] * _v[2]);
	if ( mag < SR_EPS )	// make a unit vector in z-direction
	{
		_v[0] = _v[1] = 0.0;
		_v[2] = 1.0;
	} else
	{
		_v[0] /= mag;
		_v[1] /= mag;
		_v[2] /= mag;
	}
	return mag;
}

inline Axis Rotate(const SE3 &T, const Axis &v)
{
	return Axis(T[0] * v[0] + T[3] * v[1] + T[6] * v[2],
		T[1] * v[0] + T[4] * v[1] + T[7] * v[2],
		T[2] * v[0] + T[5] * v[1] + T[8] * v[2]);
}

inline Axis InvRotate(const SE3 &T, const Axis &v)
{
	return Axis(T[0] * v[0] + T[1] * v[1] + T[2] * v[2],
		T[3] * v[0] + T[4] * v[1] + T[5] * v[2],
		T[6] * v[0] + T[7] * v[1] + T[8] * v[2]);
}

inline Axis operator * (double d, const Axis &v)
{
	return Axis(d * v[0], d * v[1], d * v[2]);
}

inline double Norm(const Axis &v)
{
	return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

inline Axis Normalize(const Axis &v)
{
    double mag = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
	if ( mag < SR_EPS )	// make a unit vector in z-direction
		return Axis(0.0, 0.0, 1.0);

	mag = 1.0 / mag;
	return Axis(mag * v[0], mag * v[1], mag * v[2]);
}

inline Axis Cross(const Axis &p, const Axis &q)
{
	return Axis(p[1] * q[2] - p[2] * q[1],
		p[2] * q[0] - p[0] * q[2],
		p[0] * q[1] - p[1] * q[0]);
}

inline double Inner(const Axis &p, const Axis &q)
{
	return (p[0] * q[0] + p[1] * q[1] + p[2] * q[2]);
}

inline double Inner(const Vec3 &p, const Axis &q)
{
	return (p[0] * q[0] + p[1] * q[1] + p[2] * q[2]);
}

inline double Inner(const Axis &p, const Vec3 &q)
{
	return (p[0] * q[0] + p[1] * q[1] + p[2] * q[2]);
}

inline double SquareSum(const Axis &p)
{
	return (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
}

inline Axis Square(const Axis &p)
{
	return Axis(p[0] * p[0], p[1] * p[1], p[2] * p[2]);
}

inline Axis InvAd(const SE3 &T, const Axis& v)
{
	return Axis(T[0] * v[0] + T[1] * v[1] + T[2] * v[2],
		T[3] * v[0] + T[4] * v[1] + T[5] * v[2],
		T[6] * v[0] + T[7] * v[1] + T[8] * v[2]);
}

inline Axis ad(const Axis &s1, const se3 &s2)
{
	return Axis(s2[2] * s1[1] - s2[1] * s1[2],
		s2[0] * s1[2] - s2[2] * s1[0],
		s2[1] * s1[0] - s2[0] * s1[1]);
}

inline Axis ad(const Axis &s1, const Axis &s2)
{
	return Axis(s2[2] * s1[1] - s2[1] * s1[2],
		s2[0] * s1[2] - s2[2] * s1[0],
		s2[1] * s1[0] - s2[0] * s1[1]);
}

inline Axis Axis::operator + (const Axis &v) const
{
	return Axis(_v[0] + v[0], _v[1] + v[1], _v[2] + v[2]);
}

inline Axis Axis::operator - (const Axis &v) const
{
	return Axis(_v[0] - v[0], _v[1] - v[1], _v[2] - v[2]);
}

inline const Axis &Axis::operator += (const Axis &v)
{
	_v[0] += v[0];
	_v[1] += v[1];
	_v[2] += v[2];
	return *this;
}

inline const Axis &Axis::operator -= (const Axis &v)
{
	_v[0] -= v[0];
	_v[1] -= v[1];
	_v[2] -= v[2];
	return *this;
}


///////////////////////////////////

#define _SMALL	1E-9
#define _TINY	1E-12

inline SO3::SO3()
{
	_R[0] = _R[4] = _R[8] = 1.0; _R[1] = _R[2] = _R[3] = _R[5] = _R[6] = _R[7] = 0.0; 
}

inline SO3::SO3(const SO3 &R)
{ 
	_R[0] = R._R[0];	_R[3] = R._R[3];	_R[6] = R._R[6];
	_R[1] = R._R[1];	_R[4] = R._R[4];	_R[7] = R._R[7];
	_R[2] = R._R[2];	_R[5] = R._R[5];	_R[8] = R._R[8];
}

inline SO3::SO3(const double R[])
{ 
	_R[0] = R[0]; _R[3] = R[3]; _R[6] = R[6];
	_R[1] = R[1]; _R[4] = R[4]; _R[7] = R[7];
	_R[2] = R[2]; _R[5] = R[5]; _R[8] = R[8];
}

inline SO3::SO3(double R0, double R1, double R2, double R3, double R4, double R5, double R6, double R7, double R8)
{
	_R[0] = R0; _R[1] = R1; _R[2] = R2; 
	_R[3] = R3; _R[4] = R4; _R[5] = R5; 
	_R[6] = R6; _R[7] = R7; _R[8] = R8;
}

inline const double &SO3::operator [] (int i) const
{
	return _R[i];
}

inline double &SO3::operator [] (int i)
{
	return _R[i];
}

inline const double &SO3::operator () (int i, int j) const
{ 
	return _R[i+3*j];
}

inline double &SO3::operator () (int i, int j)
{ 
	return _R[i+3*j]; 
}

inline SO3 &SO3::operator = (const SO3 &R)
{ 
	_R[0] = R._R[0]; _R[3] = R._R[3]; _R[6] = R._R[6]; 
	_R[1] = R._R[1]; _R[4] = R._R[4]; _R[7] = R._R[7]; 
	_R[2] = R._R[2]; _R[5] = R._R[5]; _R[8] = R._R[8]; 
	return *this; 
}

inline bool SO3::operator == (const SO3 &R) const	
{ 
	return (_R[0] == R._R[0]) && (_R[1] == R._R[1]) && (_R[2] == R._R[2]) && (_R[3] == R._R[3]) && (_R[4] == R._R[4]) && (_R[5] == R._R[5]) && (_R[6] == R._R[6]) && (_R[7] == R._R[7]) && (_R[8] == R._R[8]);	
}

inline Vec3 SO3::operator * (const Vec3 &p) const	
{ 
	return Vec3(_R[0] * p._v[0] + _R[3] * p._v[1] + _R[6] * p._v[2], _R[1] * p._v[0] + _R[4] * p._v[1] + _R[7] * p._v[2], _R[2] * p._v[0] + _R[5] * p._v[1] + _R[8] * p._v[2]); 
}

inline SO3 SO3::operator ~ (void) const
{ 
	return SO3(_R[0], _R[3], _R[6], _R[1], _R[4], _R[7], _R[2], _R[5], _R[8]); 
}

inline SO3 SO3::operator * (const SO3 &R) const
{
	return SO3(	_R[0] * R._R[0] + _R[3] * R._R[1] + _R[6] * R._R[2], _R[1] * R._R[0] + _R[4] * R._R[1] + _R[7] * R._R[2], _R[2] * R._R[0] + _R[5] * R._R[1] + _R[8] * R._R[2],
		_R[0] * R._R[3] + _R[3] * R._R[4] + _R[6] * R._R[5], _R[1] * R._R[3] + _R[4] * R._R[4] + _R[7] * R._R[5], _R[2] * R._R[3] + _R[5] * R._R[4] + _R[8] * R._R[5],
		_R[0] * R._R[6] + _R[3] * R._R[7] + _R[6] * R._R[8], _R[1] * R._R[6] + _R[4] * R._R[7] + _R[7] * R._R[8], _R[2] * R._R[6] + _R[5] * R._R[7] + _R[8] * R._R[8]);
}

inline const SO3 &SO3::operator *= (const SO3 &R)
{
	*this = *this * R;
	return *this;
}
//
//inline RMatrix SO3::ToRMatrix(void)
//{ 
//	return RMatrix(3, 3, _R); 
//}

inline SO3 Inv(const SO3 &R)
{
	return SO3(
		R[0], R[3], R[6],
		R[1], R[4], R[7],
		R[2], R[5], R[8]
	);
}

inline void SO3::RotX(const double t)
{
    //double c = cos(t), s = sin(t);

	//_R[0] = 1.0; _R[3] = 0.0; _R[6] = 0.0;
	//_R[1] = 0.0; _R[4] = c;   _R[7] = -s;
	//_R[2] = 0.0; _R[5] = s;  _R[8] = c;

	_R[0] = 1.0;
	_R[6] = _R[3] = _R[2] = _R[1] = 0.0;
	_R[8] = _R[4] = cos(t);
	_R[5] = sin(t);
	_R[7] = -_R[5];
}

inline void SO3::RotY(double t)
{
    //double c = cos(t), s = sin(t);

	//_R[0] = cos; _R[3] = 0.0; _R[6] = sin;
	//_R[1] = 0.0; _R[4] = 1.0; _R[7] = 0.0;
	//_R[2] =-sin; _R[5] = 0.0; _R[8] = cos;

	_R[4] = 1.0;
	_R[1] = _R[3] = _R[5] = _R[7] = 0.0;
	_R[8] = _R[0] = cos(t);
	_R[6] = sin(t);
	_R[2] = -_R[6];
}

inline void SO3::RotZ(double t)
{
    //double c = cos(t), s = sin(t);

	//_R[0] = cos; _R[3] =-sin; _R[6] = 0.0;
	//_R[1] = sin; _R[4] = cos; _R[7] = 0.0;
	//_R[2] = 0.0; _R[5] = 0.0; _R[8] = 1.0;

	_R[8] = 1.0;
	_R[2] = _R[5] = _R[6] = _R[7] = 0.0;
	_R[4] = _R[0] = cos(t);
	_R[1] = sin(t);
	_R[3] = -_R[1];
}

inline Vec3 SO3::GetX(void) const
{
	return Vec3(_R[0], _R[1], _R[2]);
}

inline Vec3 SO3::GetY(void) const
{
	return Vec3(_R[3], _R[4], _R[5]);
}

inline Vec3 SO3::GetZ(void) const
{
	return Vec3(_R[6], _R[7], _R[8]);
}

inline SO3 Exp(double w0, double w1, double w2)
{
    double theta = (double)sqrt(w0 * w0 + w1 * w1 + w2 * w2);
	if ( fabs(theta) < _TINY ) return SO3(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);

	w0 /= theta; w1 /= theta; w2 /= theta;
    double st = sin(theta), ct = cos(theta), vt = 1.0 - ct, t0 = w2 * st, t1 = w1 * st, t2 = w0 * st;

	return SO3( w0 * w0 * vt + ct, w0 * w1 * vt + t0, w0 * w2 * vt - t1,
		w0 * w1 * vt - t0, w1 * w1 * vt + ct, w1 * w2 * vt + t2,
		w0 * w2 * vt + t1, w1 * w2 * vt - t2, w2 * w2 * vt + ct );
}

inline SO3 Exp(const Vec3 &w)
{
	return Exp(w[0], w[1], w[2]);
}


inline SO3 Exp(const Vec3 &w, double theta)
{
	return Exp(w[0] * theta, w[1] * theta, w[2] * theta);
}

inline bool Vec3::operator == (const Vec3 &v) const
{ 
	return (_v[0] == v._v[0]) && (_v[1] == v._v[1]) && (_v[2] == v._v[2]); 
}

inline bool se3::operator == (const se3 &s) const
{ 
	return (_w[0] == s._w[0]) && (_w[1] == s._w[1]) && (_w[2] == s._w[2]) && (_w[3] == s._w[3]) && (_w[4] == s._w[4]) && (_w[5] == s._w[5]); 
}

inline bool dse3::operator == (const dse3 &t) const
{ 
	return (_m[0] == t._m[0]) && (_m[1] == t._m[1]) && (_m[2] == t._m[2]) && (_m[3] == t._m[3]) && (_m[4] == t._m[4]) && (_m[5] == t._m[5]);
}

inline bool SE3::operator == (const SE3 &T) const
{
	////////////////////////////////////////////////////////////////////////// JS 2006.11.30
	// 
	return (_T[0] == T._T[0]) &&
		(_T[1] == T._T[1]) && 
		(_T[2] == T._T[2]) && 
		(_T[3] == T._T[3]) &&
		(_T[4] == T._T[4]) && 
		(_T[5] == T._T[5]) && 
		(_T[6] == T._T[6]) && 
		(_T[7] == T._T[7]) &&
		(_T[8] == T._T[8]) && 
		(_T[9] == T._T[9]) && 
		(_T[10] == T._T[10]) && 
		(_T[11] == T._T[11]);// &&
	//(_T[12] == T._T[12]) && (_T[13] == T._T[13]) && (_T[14] == T._T[14]) && (_T[15] == T._T[15]);
}

inline bool Axis::operator == (const Axis &A) const
{
	return (_v[0] == A._v[0]) && (_v[1] == A._v[1]) && (_v[2] == A._v[2]);
}

//////////////////////////////////////////////////////////////////////////JS 2006.11.30
inline Vec3 Log(const SO3 &R)
{
    double w1, w2, w3;
    double trR = R._R[0] + R._R[4] + R._R[8];

	// [w] = Log(R) when tr(R) = -1
	//if(trR + 1 < _EPS)
	if(trR + 1 < 2.2204E-16)
	{
		w1 = sqrt( (R._R[0] + 1.0)/2.0 );
		w2 = sqrt( (R._R[4] + 1.0)/2.0 );
		w3 = sqrt( (R._R[8] + 1.0)/2.0 );
		if( w1 != 0.0 )
		{
			w2 = R._R[3]/(2.0*w1);
			w3 = R._R[6]/(2.0*w1);
		}
		else if ( w2 != 0.0 )
		{
			w1 = R._R[3]/(2.0*w2);
			w3 = R._R[7]/(2.0*w2);
		}
		else if ( w3 != 0.0 )
		{
			w1 = R._R[6]/(2.0*w3);
			w2 = R._R[7]/(2.0*w3);
		}
		else
		{
			//Error!
		}
		return Vec3( w1, w2, w3);
	}


    double theta = acos(0.5 * (R._R[0] + R._R[4] + R._R[8] - 1.0));
	if ( fabs(theta) < _SMALL ) return Vec3(0.0, 0.0, 0.0);

    double cof = theta / (2.0 * sin(theta));

	return Vec3(cof * (R._R[5] - R._R[7]), cof * (R._R[6] - R._R[2]), cof * (R._R[1] - R._R[3]));
}
//////////////////////////////////////////////////////////////////////////

