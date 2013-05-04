//////////////////////////////////////////////////////////////////////////////////
//
//		title		:	LieGroup.h 
//						
//		version		:	v0.989
//		author		:	Jinwook Kim (jwkim@imrc.kist.re.kr)
//		last update	:	2006.6.1
//
//		Note		:	v0.989 bug fixed in Log
//						v0.988 added Axis class to represent angular velocity
//						v0.987 Performance of AInertia::operator % and Inv(Inertia) is significantly improved.
//						v0.986 SO3 obsoleted. bugs fixed in Rotate, Log. inlining added
//						v0.985 lots of friend functions are ported to member functions
//						v0.98 InvSkew -> Linearize
//						v0.97 supports articulated inertia class
//						v0.95 library title changed : robotics.* -> liegroup.*
//						v0.95 does not require RMatrix
//						v0.95 supports SO3 class
//						v0.95 Inertia class uses smaller number of member variables
//						v0.95 supports friend functions InvAd, InvdAd
//						v0.95 supports /, % operators in SE3 class
//
//////////////////////////////////////////////////////////////////////////////////

#ifndef SRLIB_LIE_ALGEBRA
#define SRLIB_LIE_ALGEBRA

#include <math.h>
#include <assert.h>
#include <iostream>
#include <float.h>

using namespace std;
//#include "common/types.h"

#define SRLIB_DOUBLE
#ifdef SRLIB_DOUBLE
	// ⺻ double  ,
//	typedef double			real;
	#define SR_EPS			(1E-6)
	#define SR_ZERO			(0.0)
	#define SR_HALF			(0.5)
	#define SR_ONE			(1.0)
	#define SR_TWO			(2.0)
	#define	SR_PI_HALF		(1.57079632679489661923)	//< $\frac{\pi}{2}
	#define SR_PI			(3.14159265358979323846)	//< $\pi$
	#define SR_TWO_PI		(6.28318530717958647693)	//< $ 2 \times \pi $
	#define SR_PI_SQRT2		(2.22144146907918312351)	//< $\frac {pi}{\sqrt{2}}$
	#define SR_PI_SQR		(9.86960440108935861883)	//< $\pi^2$
	#define SR_ONETHIRD		(0.33333333333333333333)	//< $\frac{1}{3}$
	#define SR_ONESIXTH		(0.16666666666666666667)	//< $\frac{1}{6}$
	#define SR_FOURTHIRD	(1.33333333333333333333)	//< $\frac{4}{3}$
	#define SR_RADIAN		(0.01745329251994329577)	//< pi / 180
	#define SR_DEGREE		(57.2957795130823208768)	//< 180 / pi

	#define SR_ISZERO(x)	(fabs(x) < SR_EPS)			// zero test for floating point numbers
	#define SR_ISEQUAL(x,y)	(fabs((x) - (y)) < SR_EPS) // test for equality of float numbers
	#define SR_ROUND(x)		(floor((x) + 0.5))			// floating point number rounding
	#define SR_RAND(l,u)	((double)rand() / RAND_MAX * ((u) - (l)) + (l))	// float random number from interval < l ; u >
#else
	// ⺻ float  ,
//	typedef float			real;
	#define SR_EPS			(0.000001f)
	#define SR_ZERO			(0.0f)
	#define SR_HALF			(0.5f)
	#define SR_ONE			(1.0f)
	#define SR_TWO			(2.0f)
	#define	SR_PI_HALF		(1.57079632679489661923f)	//< $\frac{\pi}{2}
	#define SR_PI			(3.14159265358979323846f)	//< $\pi$
	#define SR_TWO_PI		(6.28318530717958647693f)	//< $ 2 \times \pi $
	#define SR_PI_SQRT2		(2.22144146907918312351f)	//< $\frac {pi}{\sqrt{2}}$
	#define SR_PI_SQR		(9.86960440108935861883f)	//< $\pi^2$
	#define SR_ONETHIRD		(0.33333333333333333333f)	//< $\frac{1}{3}$
	#define SR_ONESIXTH		(0.16666666666666666667f)	//< $\frac{1}{6}$
	#define SR_FOURTHIRD	(1.33333333333333333333f)	//< $\frac{4}{3}$
	#define SR_RADIAN		(0.01745329251994329577f)	//< pi / 180
	#define SR_DEGREE		(57.2957795130823208768f)	//< 180 / pi

	#define SR_ISZERO(x)	(abs(x) < EPSILON)     // zero test for floating point numbers
	#define SR_ISEQUAL(x,y)	(abs((x) - (y)) < EPSILON) // test for equality of float numbers
	#define SR_ROUND(x)		(floor((x) + 0.5))			// floating point number rounding
	#define SR_RAND(l,u)	((float)rand() / RAND_MAX * ((u) - (l)) + (l))	// float random number from interval < l ; u >
#endif

inline double DEG2RAD(int d)	 { return (d * SR_RADIAN); }
inline double RAD2DEG(int r)	 { return (r * SR_DEGREE); }
inline double DEG2RAD(float d) { return (d * SR_RADIAN); }
inline double RAD2DEG(float r) { return (r * SR_DEGREE); }
inline double DEG2RAD(double d){ return (d * SR_RADIAN); }
inline double RAD2DEG(double r){ return (r * SR_DEGREE); }

typedef	short int			Int16;		//< 2 bytes integer -32768 ~ 32767
typedef unsigned short int	UInt16;		//< 2 bytes integer 0 ~ 65535
typedef int					Int32;		//< 4 bytes integer -2147483648 ~ 2147483647
typedef unsigned int		UInt32;		//< 4 bytes integer 0 ~ 4294967255


class	Vec2;
class	Vec3;
class	so3;
class	SO3;
class	se3;
class	dse3;
class	SE3;
class	Inertia;
class	AInertia;

Inertia BoxInertia(double density, const Vec3 &size);
Inertia SphereInertia(double density, double rad);
Inertia CylinderInertia(double density, double rad, double height);
Inertia	CapsuleInertia(double density, double rad, double height);

class Vec2
{
public:
	Vec2();

	/*!
	constructor : (c0, c1, c2)
	*/
	explicit		 Vec2(double c0, double c1);



	/*!
	constructor : (c, c, c)
	*/
	explicit		 Vec2(int c);

	/*!
	constructor : (c, c, c)
	*/
	explicit		 Vec2(double c);

	/*!
	constructor : (v[0], v[1], v[2])
	*/
	explicit		 Vec2(const double v[]);

	/** @name Operators
	*/
	//@{
	/*!
	unary plus operator
	*/
	const Vec2		&operator + (void) const;

	/*!
	unary minus operator
	*/
	Vec2			 operator - (void) const;

	/*!
	access to the idx th element.
	*/
	double			&operator [] (int idx);
	const double	&operator [] (int) const;

	/*!
	substitution operator
	*/
	const Vec2		&operator = (const Vec2 &);

	/*!
	addition and substitution operator
	*/
	const Vec2		&operator += (const Vec2 &);

	/*!
	-= operator
	*/
	const Vec2		&operator -= (const Vec2 &);

	/*!
	*= operator
	*/
	const Vec2		&operator *= (double);

	/*!
	/= operator with double
	*/
	const Vec2		&operator /= (double c);

	/*!
	multiplication operator
	*/
	Vec2			 operator * (double) const;

	/*!
	division operator
	*/
	Vec2			 operator / (double) const;

	/*!
	addition operator
	*/
	Vec2			 operator + (const Vec2 &) const;

	/*!
	subtraction operator
	*/
	Vec2			 operator - (const Vec2 &) const;
	//@}

	/*!
	normalize the vector.
	\return length of the vector.
	*/
	double			 Normalize(void);

	void			 SetValues(double v0, double v1);

	/*!
	standard output operator
	*/
	friend ostream	&operator << (ostream &, const Vec2 &);

	/*!
	double multiplication
	*/
	friend Vec2		 operator * (double c, const Vec2 &p);

	/*!
	get a magnitude of p.
	*/
	friend double		 Norm(const Vec2 &p);

	/*!
	get a normalized vector from p.
	*/
	friend Vec2		 Normalize(const Vec2 &p);

	/*!
	get a cross product of p and q.
	*/
	friend Vec3		 Cross(const Vec2 &p, const Vec2 &a);

	/*!
	get an inner product of p and q.
	*/
	friend double	 Inner(const Vec2 &p, const Vec2 &a);

	/*!

	get a value of cosine theta between two vectors
	*/
	friend double	 Cosine(const Vec2 &p, const Vec2 &q);

	/*!
	get a value of cosine theta between two vectors
	*/
	friend double	 Sine(const Vec2 &p, const Vec2 &q);

	/*!
	get a squared sum of all the elements in p.
	*/
	friend double	 SquareSum(const Vec2 &);

	bool operator == (const Vec2 &v) const;




private:
	double			_v[2];
};

/*!
	\class Vec3
	\brief 3 dimensional vector
	
	Vec3 is a class for representing 3 dimensional vector.
*/
class Vec3
{
public:
					 Vec3();

	/*!
		constructor : (c0, c1, c2)
	*/
	explicit		 Vec3(double c0, double c1, double c2);

	

 	/*!
		constructor : (c, c, c)
	*/
	explicit		 Vec3(int c);

 	/*!
		constructor : (c, c, c)
	*/
	explicit		 Vec3(double c);

 	/*!
		constructor : (v[0], v[1], v[2])
	*/
	explicit		 Vec3(const double v[]);

	/** @name Operators
	*/
	//@{
 	/*!
		unary plus operator
	*/
	const Vec3		&operator + (void) const;

 	/*!
		unary minus operator
	*/
	Vec3			 operator - (void) const;

 	/*!
		access to the idx th element.
	*/
	double			&operator [] (int idx);
	const double	&operator [] (int) const;

 	/*!
		substitution operator
	*/
	const Vec3		&operator = (const Vec3 &);

 	/*!
		substitute operator
		set all the elements to be c.
	*/
	const Vec3		&operator = (double c);

 	/*!
		substitute operator
		set elements from the position part of T.
	*/
	const Vec3		&operator = (const SE3 &T);

 	/*!
		addition and substitution operator
	*/
	const Vec3		&operator += (const Vec3 &);

 	/*!
		-= operator
	*/
	const Vec3		&operator -= (const Vec3 &);

 	/*!
		*= operator
	*/
	const Vec3		&operator *= (double);

	/*!
		/= operator with double
	*/
	const Vec3		&operator /= (double c);

 	/*!
		multiplication operator
	*/
	Vec3			 operator * (double) const;

	/*!
		division operator
	*/
	Vec3			 operator / (double) const;

 	/*!
		addition operator
	*/
	Vec3			 operator + (const Vec3 &) const;

 	/*!
		subtraction operator
	*/
	Vec3			 operator - (const Vec3 &) const;
	//@}

 	/*!
		normalize the vector.
		\return length of the vector.
	*/
	double			 Normalize(void);

	void			 SetValues(double v0, double v1, double v2);

	////////////////////////////////////////////////////////////////////////// added by Jeongseok 2008-03-23
	void			 Clone(const Vec3& rhs);
	//////////////////////////////////////////////////////////////////////////

  	/*!
		standard output operator
	*/
	friend ostream	&operator << (ostream &, const Vec3 &);

 	/*!
        double multiplication
	*/
	friend Vec3		 operator * (double c, const Vec3 &p);

	/*!
		get a magnitude of p.
	*/
	friend double	 Norm(const Vec3 &p);

	/*!
		get a normalized vector from p.
	*/
	friend Vec3		 Normalize(const Vec3 &p);

	/*!
		get a cross product of p and q.
	*/
	friend Vec3		 Cross(const Vec3 &p, const Vec3 &a);

 	/*!
		get an inner product of p and q.
	*/
	friend double	 Inner(const Vec3 &p, const Vec3 &a);

	/*!

		get a value of cosine theta between two vectors
	*/
	friend double	 Cosine(const Vec3 &p, const Vec3 &q);

	/*!
		get a value of cosine theta between two vectors
	*/
	friend double	 Sine(const Vec3 &p, const Vec3 &q);

 	/*!
		get a squared sum of all the elements in p.
	*/
	friend double	 SquareSum(const Vec3 &);

 	/*!
		fast version of Ad(Inv(T), dse3(Vec3(0), F))
	*/
	friend dse3		 dAd(const SE3 &T, const Vec3 &F);

	/*!
		fast version of dAd(Inv(SE3(p)), dse3(Vec3(0), F))
	*/
	friend dse3		 InvdAd(const Vec3 &p, const Vec3 &F);

 	/*!
		fast version of Ad(Inv(T), se3(Vec3(0), v))
	*/
	friend Vec3		 InvAd(const SE3 &T, const Vec3 &v);

 	/*!
		get a transformation matrix given by the Euler ZYX angle, 
		where the positional part is set to be zero.
		\sa SE3::iEulerZYX
	*/
	friend SE3		 EulerZYX(const Vec3 &);

 	/*!
		get a transformation matrix given by the Euler ZYX angle and position, 
		where the positional part is set to be zero.
	*/
	friend SE3		 EulerZYX(const Vec3 &angle, const Vec3 &position);
	friend SE3		 EulerXYZ(const Vec3 &angle, const Vec3 &position);

	/*!
		get a transformation matrix given by the Euler ZYZ angle, 
		where the positional part is set to be zero.
		\sa SE3::iEulerZYZ
	*/
	friend SE3		 EulerZYZ(const Vec3 &);

 	/*!
		get a transformation matrix given by the Euler ZYZ angle and position, 
		where the positional part is set to be zero.
	*/
	friend SE3		 EulerZYZ(const Vec3 &angle, const Vec3 &position);

	/*!
		rotate q by T.
		\return \f$R q\f$, where \f$T=(R,p)\f$.
	*/
	friend Vec3		 Rotate(const SE3 &T, const Vec3 &q);

 	/*!
		rotate q by Inv(T).
	*/
	friend Vec3		 InvRotate(const SE3 &T, const Vec3 &q);

 	/*!
		fast version of ad(se3(Vec3(0), v), S)
	*/
	friend Vec3		 ad(const Vec3 &v, const se3 &S);

/////////////////////////
	friend class SO3;
//////////////////////////////////////////////////////////////////////////JS 2006.11.30
	friend class SE3;
//////////////////////////////////////////////////////////////////////////

	bool operator == (const Vec3 &v) const;

/////////////////////////



private:
	double			_v[3];
};

/*!
	\class Axis
	\brief 3 dimensional vector but used for angular part of se(3) or dse(3)
*/
class Axis
{
public:
					 Axis();

	/*!
		constructor : (c0, c1, c2)
	*/
	explicit		 Axis(double c0, double c1, double c2);

 	/*!
		constructor : (c, c, c)
	*/
	explicit		 Axis(int c);

 	/*!
		constructor : (c, c, c)
	*/
	explicit		 Axis(double c);

 	/*!
		constructor : (v[0], v[1], v[2])
	*/
	explicit		 Axis(const double v[]);

	/** @name Operators
	*/
	//@{
 	/*!
		unary plus operator
	*/
	const Axis		&operator + (void) const;

 	/*!
		unary minus operator
	*/
	Axis			 operator - (void) const;

 	/*!
		access to the idx th element.
	*/
	double			&operator [] (int idx);
	const double	&operator [] (int) const;

 	/*!
		substitution operator
	*/
	const Axis		&operator = (const Axis &);

	/*!
		fast version of = Axis(s[0], s[1], s[2])
	*/
	const Axis		&operator = (const se3 &);

 	/*!
		substitute operator
		set all the elements to be c.
	*/
	const Axis		&operator = (double c);

 	/*!
		*= operator
	*/
	const Axis		&operator *= (double);

 	/*!
		multiplication operator
	*/
	Axis			 operator * (double) const;

	/*!
		addition operator
	*/
	Axis			 operator + (const Axis &) const;

 	/*!
		subtraction operator
	*/
	Axis			 operator - (const Axis &) const;

	/*!
		addition and substitution operator
	*/
	const Axis		&operator += (const Axis &);

 	/*!
		-= operator
	*/
	const Axis		&operator -= (const Axis &);
	//@}

 	/*!
		normalize the vector.
		\return length of the vector.
	*/
	double			 Normalize(void);

 	/*!
		standard output operator
	*/
	friend ostream	&operator << (ostream &, const Axis &);

 	/*!
        double multiplication
	*/
	friend Axis		 operator * (double c, const Axis &p);

	/*!
		get a magnitude of p.
	*/
	friend double	 Norm(const Axis &p);

	/*!
		get a normalized vector from p.
	*/
	friend Axis		 Normalize(const Axis &p);

	/*!
		get a cross product of p and q.
	*/
	friend Axis		 Cross(const Axis &p, const Axis &a);

 	/*!
		get an inner product of p and q.
	*/
	friend double	 Inner(const Axis &p, const Axis &a);
	friend double	 Inner(const Vec3 &p, const Axis &a);
	friend double	 Inner(const Axis &p, const Vec3 &a);

 	/*!
		get a squared sum of all the elements in p.
	*/
	friend double	 SquareSum(const Axis &);

	/*!
		rotate p by T.
		\return \f$R q\f$, where \f$T=(R,p)\f$.
	*/
	friend Axis		 Rotate(const SE3 &T, const Axis &q);

 	/*!
		rotate q by Inv(T).
	*/
	friend Axis		 InvRotate(const SE3 &T, const Axis &q);

 	/*!
		fast version of ad(se3(w, 0), se3(v, 0))	-> check
	*/
	friend Axis		 ad(const Axis &w, const Axis &v);

 	/*!
		rotate q by Inv(T).							-> check
	*/
	friend se3		 Ad(const SE3 &T, const Axis &q);

	/*!
		fast version of Ad(Inv(T), se3(w, Vec3(0)))
	*/
	friend Axis		 InvAd(const SE3 &T, const Axis &w);

//////////////////////////////////////
	bool operator == (const Axis&) const;
//////////////////////////////////////

private:
	double			_v[3];
};

/*!
	\class se3
	\brief Lie algebra of SE(3)
	
	se3 is a class for representing \f$se(3)\f$, the Lie algebra of \f$SE(3)\f$.
	Geometrically it deals with generalized velocity.
	The first three elements correspond to angular velocity
	and the last three elements correspond to linear velocity.
*/
class se3
{
public:
					 se3();

	/*!
		constructor : (c, c, c, c, c, c)
	*/
	explicit		 se3(int c);

	/*!
		constructor : (c, c, c, c, c, c)
	*/
	explicit		 se3(double c);

	/*!
		constructor : (c0, c1, c2, c3, c4, c5)
	*/
	explicit		 se3(double c0, double c1, double c2, double c3, double c4, double c5);

	/*!
		constructor : (w[0], w[1], w[2], v[0], v[1], v[2])
	*/
	explicit		 se3(const Axis &w, const Vec3 &v);
	
 	/*!
		unary plus operator
	*/
	const se3		&operator + (void) const;

 	/*!
		unary minus operator
	*/
	se3				 operator - (void) const;

 	/*!
		substitution operator
	*/
	const se3		&operator = (const se3 &);

	/*!
		fast version of = se3(w, 0)
	*/
	const se3		&operator = (const Axis &w);

	/*!
		fast version of = se3(0, v)
	*/
	const se3		&operator = (const Vec3 &v);

 	/*!
		substitution operator, fast version of = se3(c)
	*/
	const se3		&operator = (double c);

 	/*!
		+= operator
	*/
	const se3		&operator += (const se3 &);

	/*!
		fast version of += se3(w, 0)
	*/
	const se3		&operator += (const Axis &w);

 	/*!
		-= operator
	*/
	const se3		&operator -= (const se3 &);

 	/*!
        *= operator with double
	*/
	const se3		&operator *= (double c);

	/*!
		addition operator
	*/
	se3				 operator + (const se3 &) const;

	/*!
		subtraction operator
	*/
	se3				 operator - (const se3 &) const;

	/*!
		double multiplication operator
	*/
	se3				 operator * (double) const;

 	/*!
		access to the idx th element.
	*/
	double			&operator [] (int idx);

	const double	&operator [] (int) const;
	
 	/*!
		set itself to be Ad(T, V).
	*/
	void			 Ad(const SE3 &T, const se3 &V);

 	/*!
		set itself to be Ad(Inv(T), V).
	*/
	void			 InvAd(const SE3 &T, const se3 &V);

 	/*!
		set itself to be ad(V, W).
	*/
	void			 ad(const se3 &V, const se3 &W);
	
	/*!
		fast version ad(V, se3(W, 0))
	*/
	void			 ad(const se3 &V, const Axis &W);
	
 	/*!
		standard output operator
	*/
	friend ostream	&operator << (ostream &, const se3 &);

 	/*!
        double multiplicaiton operator
	*/
	friend se3		 operator * (double, const se3 &);

 	/*!
		inner product
		\note \f$ \langle F, V\rangle = \langle V, F\rangle = \langle m, w\rangle + \langle f, v\rangle \f$
		,where \f$F=(m,f)\in se(3)^*,\quad V=(w,v)\in se(3)\f$.
	*/
	friend double	 operator * (const dse3 &F, const se3 &V);

 	/*!
		inner product
	*/
	friend double	 operator * (const se3 &V, const dse3 &F);

 	/*!
		Exponential mapping
	*/
	friend SE3		 Exp(const se3 &);

 	/*!
		fast version of Exp(se3(s, 0))
	*/
	friend SE3		 Exp(const Axis &s);

	/*!
		fast version of Exp(t * s)
		\note If the angular part of s is unit, Exp(s, t) will be fater than Exp(t * s)
	*/
	friend SE3		 Exp(const se3 &s, double t);

	/*!
		fast version of Exp(t * s), when |s| = 1
	*/
	friend SE3		 Exp(const Axis &s, double t);

 	/*!
		Log mapping
	*/
	friend se3		 Log(const SE3 &);

	/*!
		adjoint mapping
		\note \f$Ad_TV = ( Rw\,, ~p \times Rw + Rv)\f$
		, where \f$T=(R,p)\in SE(3), \quad V=(w,v)\in se(3) \f$.
	*/
	friend se3		 Ad(const SE3 &T, const se3 &V);

	/*!
		fast version of Ad(Inv(T), V)
	*/
	friend se3		 InvAd(const SE3 &T, const se3 &V);

	/*!
		get a linear part of Ad(SE3(-p), V).
	*/
	friend Vec3		 MinusLinearAd(const Vec3 &p, const se3 &V);

 	/*!
		adjoint mapping
		\note \f$ad_X Y = ( w_X \times w_Y\,,~w_X \times v_Y - w_Y \times v_X),\f$
		, where \f$X=(w_X,v_X)\in se(3), \quad Y=(w_Y,v_Y)\in se(3) \f$.
	*/
	friend se3		 ad(const se3 &X, const se3 &Y);
	
 	/*!
		dual adjoint mapping
		\note \f$ad^{\,*}_V F = (m \times w + f \times v\,,~ f \times w),\f$
		, where \f$F=(m,f)\in se^{\,*}(3), \quad V=(w,v)\in se(3) \f$.
	*/
	friend dse3		 dad(const se3 &V, const dse3 &F);

	/*!
		get squared sum of all the elements
	*/
	friend double	 SquareSum(const se3 &);

	/*!
		fast version of se3(Rotate(T, Vec3(S[0], S[1], S[2])), Rotate(T, Vec3(S[3], S[4], S[5])))
	*/
	friend se3		 Rotate(const SE3 &T, const se3 &S);

	/*!
		fast version of se3(Rotate(Inv(T), Vec3(S[0], S[1], S[2])), Rotate(Inv(T), Vec3(S[3], S[4], S[5])))
	*/
	friend se3		 InvRotate(const SE3 &T, const se3 &S);

//////////////////////////////
	bool operator == (const se3 &s) const;
//////////////////////////////

private:
	double			_w[6];
};

/*!
	\class dse3
	\brief Dual space of se(3)
	
	dse3 is a class for representing \f$se(3)^*\f$, a dual of the Lie algebra \f$se(3)\f$.
	Geometrically it deals with generalized force.
	The first three elements correspond to moment(or torque)
	and the last three elements correspond to force.
*/
class dse3
{
public:
					 dse3();
	/*!
		constructor : (c, c, c, c, c, c)
	*/
	explicit		 dse3(int c);

	/*!
		constructor : (c, c, c, c, c, c)
	*/
	explicit		 dse3(double c);

	/*!
		constructor : (c0, c1, c2, c3, c4, c5)
	*/
	explicit		 dse3(double c0, double c1, double c2, double c3, double c4, double c5);

	/*!
		constructor : (m[0], m[1], m[2], f[0], f[1], f[2])
	*/
	explicit		 dse3(const Axis &m, const Vec3 &f);

	/*!
	 constructor : (m[0], m[1], m[2], f[0], f[1], f[2])
	 */
	explicit		 dse3(const Vec3 &m, const Vec3 &f);

	/*!
		constructor : Inertia(mass) * dV
	*/
	explicit		 dse3(double mass, const se3 &dV);

	/*!
		constructor :  (v[0], v[1], v[2], v[3], v[4], v[5])
	*/
	explicit		 dse3(const double v[]);

	/*!
		unary plus operator
	*/
	const dse3		&operator + (void) const;

	/*!
		unary minus operator
	*/
	dse3			 operator - (void) const;

	/*!
		substitution operator
	*/
	const dse3		&operator = (const dse3 &);



	const dse3		&operator = (const se3 &);

	/*!
		fast version of = dse3(0, f)
	*/
	const dse3		&operator = (const Vec3 &f);

	/*!
		fast version of = dse3(m, 0)
	*/
	const dse3		&operator = (const Axis &m);

	/*!
		substitution operator, fast version of = dse3(c)
	*/
	const dse3		&operator = (double c);

	/*!
		+= operator
	*/
	const dse3		&operator += (const dse3 &);

	/*!
		-= operator
	*/
	const dse3		&operator -= (const dse3 &);

	/*!
		*= operator
	*/
	const dse3		&operator *= (double);

	/*!
		addition operator
	*/
	dse3			 operator + (const dse3 &) const;

	/*!
		subtraction operator
	*/
	dse3			 operator - (const dse3 &) const;

	/*!
		double multiplication operator
	*/
	dse3			 operator * (double) const;

	/*!
		access to the idx th element.
	*/
	double			&operator [] (int idx);

	const double	&operator [] (int) const;
	
 	/*!
		set itself to be dad(V, F).
	*/
	void			 dad(const se3 &V, const dse3 &F);

 	/*!
		set itself to be dAd(T, F).
	*/
	void			 dAd(const SE3 &T, const dse3 &F);

 	/*!
		standard output operator
	*/
	friend ostream	&operator << (ostream &, const dse3 &);

 	/*!
        double multiplication operator
	*/
	friend dse3		 operator * (double, const dse3 &);

	/*!
		fast version of F * se3(V, 0)
	*/
	friend double	 operator * (const dse3 &F, const Axis &V);

	/*!
		dual adjoint mapping
		\note \f$Ad^{\,*}_TF = ( R^T (m - p\times f)\,,~ R^T f)\f$, where \f$T=(R,p)\in SE(3), F=(m,f)\in se(3)^*\f$.
	*/
	friend dse3		 dAd(const SE3 &T, const dse3 &F);

 	/*!
		fast version of dAd(Inv(T), F)
	*/
	friend dse3		 InvdAd(const SE3 &T, const dse3 &F);

 	/*!
		get a squared sum of all the elements in p.
	*/
	friend double	 SquareSum(const dse3 &);
	
////////////////////////////////
		bool operator == (const dse3 &t) const;
////////////////////////////////
private:
	double			_m[6];
};

/*!
	\class SE3
	\brief Special Euclidean group
	
	SE3 is a class for representing the special Euclidean group.
	Geometrically, it deals with  rigid transformations on \f$ \mathbb{R}^3 \f$.
	SE(3) is defined as the set of
	mappings \f$g: \mathbb{R}^3 \rightarrow \mathbb{R}^3\f$ of the form \f$g(x) = Rx + p\f$,
	where \f$R\in\f$ the special orthogonal group and \f$p\in \mathbb{R}^3\f$.
	An element of SE(3), written as (R, p), can also be represented in
	the matrix form	\f$\begin{bmatrix} R & p \\ 0 & 1\end{bmatrix}.\f$
*/
class SE3
{
public:
					 SE3();

 	/*!
		copy constructor
	*/
					 SE3(const SE3 &);

	/*!
		constructor
		rotation part
	*/
	explicit		 SE3(double, double, double, double, double, double, double, double, double);

	/*!
		constructor
		rotation and position part
	*/
	explicit		 SE3(double, double, double, double, double, double, double, double, double, double, double, double);

 	/*!
		constructor
		rotation part = an identity, position part = p
	*/
					 SE3(const Vec3 &p);
					 
//////////////////////////////////////////////////////////////////////////JS 2006.11.29
	explicit		SE3(const SO3 &R, const Vec3 &p);
//////////////////////////////////////////////////////////////////////////

 	/*!
		constructor
		Rx, Ry, Rz = x, y, z axes of rotation matrix, p = position
	*/
	explicit		 SE3(const Vec3 &Rx, const Vec3 &Ry, const Vec3 &Rz, const Vec3 &p);

 	/*!
		constructor
		fast version of SE3(Vec3(c))
	*/
	explicit		 SE3(double c);
	explicit		 SE3(int c);

	/*!
		constructor
		array T is assumed to be column based 4X4 matrix
	*/
	explicit		 SE3(const double T[]);

 	/*!
		get the i-th row and the j-th column element.
	*/
	double			 operator () (int i, int j) const;

 	/*!
		access to the i-th element, where it is assumed as an array.
		in a matrix form, it looks like
		| T[0]	T[3]	T[6]	T[ 9] |
		| T[1]	T[4]	T[7]	T[10] |
		| T[2]	T[5]	T[8]	T[11] |
		,where the left 3X3 matrix is the rotation matrix and the right 3 vector is the position.
	*/
	const double	&operator [] (int i) const;

	double			&operator [] (int);

	/*!
		substitution operator
	*/
	const SE3		&operator = (const SE3 &);

	/*!
		substitution operator, fast version of = SE3(p)
	*/
	const SE3		&operator = (const Vec3 &p);

	/*!
		multiplication operator
	*/
	SE3				 operator * (const SE3 &T) const;

	/*!
		multiplication operator, Ta / Tb = Ta * Inv(Tb)
	*/
	SE3				 operator / (const SE3 &T) const;

	/*!
		multiplication operator, Ta \% Tb = Inv(Ta) * Tb
	*/
	SE3				 operator % (const SE3 &T) const;

	/*!
		multiplication operator
		\note \f$ T q = R q + p\f$, where \f$T=(R,p)\in SE(3), \quad q\in\mathbb{R}^3\f$.
	*/
	Vec3			 operator * (const Vec3 &) const;

	/*!
		multiplication operator, T \% p = Inv(T) * p
	*/
	Vec3			 operator % (const Vec3 &p) const;

	/*!
		multiplication operator\n
		Ta *= Tb is a fast version of Ta = Ta * Tb
	*/
	const SE3		&operator *= (const SE3 &);

	/*!
		multiplication operator\n
		Ta /= Tb is a fast version of Ta = Ta * Inv(Tb)
	*/
	const SE3		&operator /= (const SE3 &);

	/*!
		multiplication operator\n
		Ta \%= Tb is a fast version of Ta = Inv(Ta) * Tb
	*/
	const SE3		&operator %= (const SE3 &);
	
	/*!
		set itself to be identity.
	*/
	void			 SetEye(void);

	/*!
		set rotation part from T and position part from p.
	*/
	void			 Set(const SE3 &T, const Vec3 &p);

	//////////////////////////////////////////////////////////////////////////JS 2008.03.23
	/*!
		set rotation part and position part from T
	*/
	void			 Set(const SE3 &T);

	/*!
		set rotation part from R and position part from P.
	*/
	void			 Set(const SO3 &R, const Vec3 &P);
	//////////////////////////////////////////////////////////////////////////

	/*!
		set rotation part only from T
	*/
	void			 SetOrientation(const SE3 &T);
	void			 SetOrientation(const SO3 &R);

	//////////////////////////////////////////////////////////////////////////JS 2006.11.29
	/*!
		get orientation part.
	*/
	SO3			GetOrientation(void) const;
	//////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////JS 2008.1.26
	/*!
		get x-axis from orientation part.
	*/
	Vec3			GetX(void) const;

	/*!
		get y-axis from orientation part.
	*/
	Vec3			GetY(void) const;

	/*!
		get z-axis from orientation part.
	*/
	Vec3			GetZ(void) const;
	//////////////////////////////////////////////////////////////////////////

	/*!
		set position part from p.
	*/
	void			 SetPosition(const Vec3 &p);

	/*!
		get position part.
	*/
	Vec3			 GetPosition(void) const;

	/*!
		Exponential mapping
	*/
	void			 Exp(const se3 &);

	/*!
		Exponential mapping for unit length axis
	*/
	void			 Exp(const se3 &, double);

	/*!
		Fill in the array M
		M[0] = T[0]		M[4] = T[3]		M[ 8] = T[6]		M[12] = T[ 9]
		M[1] = T[1]		M[5] = T[4]		M[ 9] = T[7]		M[13] = T[10]
		M[2] = T[2]		M[6] = T[5]		M[10] = T[8]		M[14] = T[11]
		M[3] = 0		M[7] = 0		M[11] = 0			M[15] = 1
	*/
	template <class TYPE>
	void			 ToArray(TYPE M[]) const;

 	/*!
		standard output operator
	*/
	friend ostream	&operator << (ostream &, const SE3 &);

	/*!
		get inversion of T
		\note \f$T^{-1} = (R^T, -R^T p), where T=(R,p)\in SE(3)\f$.
	*/
	friend SE3		 Inv(const SE3 &T);

	/*!
		Exponential mapping
	*/
	friend SE3		 Exp(const se3 &);

	/*!
		Exponential mapping for unit length axis
	*/
	friend SE3		 Exp(const se3 &, double);

	/*!
		get rotation matrix rotated along x-axis by theta angle.
		\note theta is represented in radian.
	*/
	friend SE3		 RotX(double);

	/*!
		get rotation matrix rotated along y-axis by theta angle.
	*/
	friend SE3		 RotY(double);

	/*!
		get rotation matrix rotated along z-axis by theta angle.
	*/
	friend SE3		 RotZ(double);

	/*!
		Log mapping
	*/
	friend se3		 Log(const SE3 &);

	/*!
		Log mapping of rotation part only
		\note A linear part of the return value is the position of T.
			If \f$|LogR(T)| = \pi\f$, Exp(LogR(T) = Exp(-LogR(T)).
			The implementation returns only the positive one.
	*/
	friend Axis		 LogR(const SE3 &T);

	/*!
		get the first order approximation of T.
		\note If T is near to an identity, Linearize(T) ~= Log(T).
			  Hence it is cheaper than Log, it is recommended to use Linearize 
			  rather than Log near identity.
	*/
	friend se3		 Linearize(const SE3 &T);

 	/*!
		get the Euler ZYX angle from T
		\sa Vec3::EulerZYX
	*/
	friend Vec3		 iEulerZYX(const SE3 &T);

 	/*!
		get the Euler ZYZ angle from T
		\sa Vec3::EulerZYZ
	*/
	friend Vec3		 iEulerZYZ(const SE3 &);

///////////////////////////////
	bool operator == (const SE3 &T) const;
///////////////////////////////

private:
	double			_T[12];
};

typedef	Vec3	InvVec3;
/*!
	\class Inertia
	\brief Generalized inertia tensor
	
	Inertia is a class for representing generalized inertia tensor.
	Generalized inertia tensor can be expressed as the triple (I, m, r),
	where \f$m\in\mathbb{R},\,r\in\mathbb{R}^3\f$ and \f$I\in\mathbb{R}^{3\times 3}\f$ is positive
definite.
*/
class Inertia
{
public:
					 Inertia();
	/*!
		constructor : mass = Ixx = Iyy = Izz = m
	*/
	explicit		 Inertia(double m);

	/*!
		constructor
	*/
	explicit		 Inertia(double mass, double Ixx, double Iyy, double Izz);

	explicit		 Inertia(double, double, double, double, double, double, double, double, double, double);

	/*!
		constructor
		I_xx, I_yy, I_zz, I_xy, I_yz, I_zx.
		mass is not specified.
		so you MUST specify mass value after this constructor.
	*/
	explicit		 Inertia(double, double, double, double, double, double);

	/*!
		multiplication operator
		\note \f$J V = ( Iw + r\times v,~ mv-r\times w)\in se(3)^*\f$, where \f$J=(I,m,r)\in\f$ Inertia, \f$V=(w,v)\in se(3)\f$.
	*/
	dse3			 operator * (const se3 &V) const;

	/*!
		fast version of * se3(Vec3(0), v))
	*/
	dse3			 operator * (const Vec3 &v) const;
	/*!
		fast version of * se3(v, Vec3(0)))
	*/
	dse3			 operator * (const Axis &v) const;

	/*!
		addition operator
	*/
	Inertia			 operator + (const Inertia &) const;

	/*!
		substitution operator
	*/
	const Inertia	&operator = (const Inertia &);

	/*!
		double multiplication operator
	*/
	const Inertia	&operator *= (double);
	
	/*!
		access to ith element
		Inertia[i]	= inertia tensor, i \in [0, 5]
					= offset		, i \in [6, 8]
					= mass			, i = 9
	*/
	double			&operator [] (int);
	const double	&operator [] (int) const;

	/*!
		get coordinate transformed inertia
	*/
	Inertia			 Transform(const SE3 &) const;

	/*!
		Fill in the array M
		M[0]  =  I[0];		M[6]  =  I[3];		M[12] =  I[4];		M[18] =  0;			M[24] = -I[8];		M[30] =  I[7];
		M[1]  =  I[3];		M[7]  =  I[1];		M[13] =  I[5];		M[19] =  I[8];		M[25] =  0;			M[31] = -I[6];
		M[2]  =  I[4];		M[8]  =  I[5];		M[14] =  I[2];		M[20] = -I[7];		M[26] =  I[6];		M[32] =  0;
		M[3]  =  0;			M[9]  =  I[8];		M[15] = -I[7];		M[21] =  I[9];		M[27] =  0;			M[33] =  0;
		M[4]  = -I[8];		M[10] =  0;			M[16] =  I[6];		M[22] =  0;			M[28] =  I[9];		M[34] =  0;
		M[5]  =  I[7];		M[11] = -I[6];		M[17] =  0;			M[23] =  0;			M[29] =  0;			M[35] =  I[9];
	*/
	template <class TYPE>
	void			 ToArray(TYPE M[]) const;

	/*!
		get a mass.
	*/
	double			 GetMass(void) const;

	/*!
		get the diagonals, that is, Ixx, Iyy and Izz.
	*/
	Vec3			 GetDiag(void) const;

	/*!
		get the offset r.
	*/
	Vec3			 GetOffset(void) const;

	/*!
		set a mass.
	*/
	void			 SetMass(double);

	/*!
		Set angular moment of inertia, i.e. Ixx, Iyy, Izz, Ixy, Iyz, and Izx.
	*/
	void			 SetAngularMoment(double, double, double, double, double, double);

	/*!
		set the offset r.
	*/
	void			 SetOffset(Vec3);

	/*!
		double multiplication operator
	*/
	friend Inertia	 operator * (double, const Inertia &);

	/*!
		get inverse of J.
	*/
	friend AInertia	 Inv(const Inertia &J);

	/*!
		get an inertia of box shaped geometry.
		\param d density of the geometry
		\param sz size of the box
	*/
	friend Inertia	 BoxInertia(double d, const Vec3 &sz);

	/*!
	get an inertia of box shaped geometry.
	\param d density of the geometry
	\param sz half size of the box
	*/
	friend Inertia	 BoxInertia_Half(double d, const Vec3 &sz);

	/*!
		get an inertia of sphere shaped geometry.
		\param d density of the geometry
		\param r radius of the sphere
	*/
	friend Inertia	 SphereInertia(double d, double r);

	/*!
		get an inertia of cylindrical geometry.
		\param d density of the geometry
		\param r radius of the cylinder
		\param h height of the cylinder
	*/
	friend Inertia	 CylinderInertia(double d, double r, double h);

	/*!
	get an inertia of capsular geometry.
	\param d density of the geometry
	\param r radius of the capsule
	\param h height of the capsule
	*/
	friend Inertia	 CapsuleInertia(double d, double r, double h);

private:
	double			_I[10];
};

/*!
	\class AInertia
	\brief Articulated inertia tensor
	
	AInertia is a class for representing articulated inertia tensor.
*/
class AInertia
{
public:
					 AInertia();
	explicit		 AInertia(double);
					 AInertia(const Inertia &);
	explicit		 AInertia(double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double);

	double			&operator [] (int);
	const double	&operator [] (int) const;
	const AInertia	&operator + (void) const;
	AInertia		 operator - (void) const;
	dse3			 operator * (const se3 &) const;
	dse3			 operator * (const Axis &) const;
	se3				 operator * (const dse3 &) const;			// assumed to be an inverse of AInertia
	AInertia		 operator + (const AInertia &) const;
	AInertia		 operator + (const Inertia &) const;
	AInertia		 operator - (const AInertia &) const;
	AInertia		 operator - (const Inertia &) const;
	const AInertia	&operator += (const AInertia &);
	const AInertia	&operator += (const Inertia &);
	const AInertia	&operator -= (const AInertia &);
	const AInertia	&operator -= (const Inertia &);
	se3				 operator % (const dse3 &) const;
	const AInertia	&operator = (const AInertia &);
	const AInertia	&operator = (const Inertia &);

	void			 SubtractKroneckerProduct(const dse3 &, const dse3 &);
	void			 AddTransform(const AInertia &, const SE3 &);
	AInertia		 Transform(const SE3 &) const;
	template <class TYPE>
	void			 ToArray(TYPE []) const;
	
 	/*!
		The Kronecker product
	*/
	friend AInertia	 KroneckerProduct(const dse3 &, const dse3 &);
	
private:
	double			_J[21];
};


///////////////////////////////////

class SO3
{
public:
	// constructors
	SO3();
	SO3(const SO3 &R);
	SO3(const double R[]);
	SO3(double R0, double R1, double R2, double R3, double R4, double R5, double R6, double R7, double R8);

	// Add by Jeongseok, 2009-03-29
	void RotX(const double t);
	void RotY(const double t);
	void RotZ(const double t);

	// How about this?
	//void MakeEulerZXY(const double z, const double x, const double y);

	////////////////////////////////////////////////////////////////////////// Jeongseok Lee 2008-03-24
	// operators
	/*!
	access to the i-th element, where it is assumed as an array.
	in a matrix form, it looks like
	| R[0]	R[3]	R[6] |
	| R[1]	R[4]	R[7] |
	| R[2]	R[5]	R[8] |
	,where the left 3X3 matrix is the rotation matrix.
	*/
	const double	&operator [] (int i) const;
	double			&operator [] (int);
	//////////////////////////////////////////////////////////////////////////
	const double &operator () (int i, int j) const;
	double &operator () (int i, int j);
	SO3 &operator = (const SO3 &R);
	bool operator == (const SO3 &R) const;
	const SO3 &operator *= (const SO3 &R);
	SO3 operator * (const SO3 &R) const;
	Vec3 operator * (const Vec3 &p) const;
	SO3 operator ~ (void) const;
//	RMatrix ToRMatrix(void);

	// friend functions
	friend std::ostream &operator << (std::ostream &os, const SO3 &R);	// ostream standard output
	////////////////////////////////////////////////////////////////////////// Jeongseok Lee 2008-03-24
	friend SO3 Inv(const SO3 &R);
	Vec3			GetX(void) const;
	Vec3			GetY(void) const;
	Vec3			GetZ(void) const;
	//////////////////////////////////////////////////////////////////////////
	friend SO3 Exp(double w0, double w1, double w2);
	friend SO3 Exp(const Vec3 &w);
	friend SO3 Exp(const Vec3 &w, double theta);
	friend Vec3 Log(const SO3 &R);
	friend Vec3 iEulerZYX(const SO3 &R);
	friend Vec3 iEulerZYZ(const SO3 &R);
	
	// friend class
	friend class SE3;

private:
	double _R[9];
};


#include "LieGroup.inl"

#endif

