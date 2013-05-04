//////////////////////////////////////////////////////////////////////////////////
//
//		title		:	LieGroup.cpp
//						
//		version		:	v0.987
//		author		:	Jinwook Kim (jwkim@imrc.kist.re.kr)
//		last update	:	2004.9.13
//
//////////////////////////////////////////////////////////////////////////////////

#include "LieGroup.h"
#include <iomanip>



ostream &operator << (ostream &os, const Vec3 &v)
{
	ios_base::fmtflags flags = os.setf(ios::left | ios::fixed);
	streamsize sz = os.precision(3);
	os << "[ ";
	for ( int i = 0; i < 3; i++ )
	{ 
		if ( v[i] >= 0.0 ) os << " " << setw(6) << v[i] << " ";
		else os << setw(7) << v[i] << " ";
	}
	os << "];" << endl;
	os.setf(flags);
	os.precision(sz);
	return os;
}

ostream &operator << (ostream &os, const Axis &v)
{
	ios_base::fmtflags flags = os.setf(ios::left | ios::fixed);
	streamsize sz = os.precision(3);
	os << "[ ";
	for ( int i = 0; i < 3; i++ )
	{ 
		if ( v[i] >= 0.0 ) os << " " << setw(6) << v[i] << " ";
		else os << setw(7) << v[i] << " ";
	}
	os << "];" << endl;
	os.setf(flags);
	os.precision(sz);
	return os;
}

ostream &operator << (ostream &os, const se3 &s)
{
	ios_base::fmtflags flags = os.setf(ios::left | ios::fixed);
	streamsize sz = os.precision(3);
	os << "[ ";
	for ( int i = 0; i < 6; i++ )
	{ 
		if ( s[i] >= 0.0 ) os << " " << setw(6) << s[i] << " ";
		else os << setw(7) << s[i] << " ";
	}
	os << "];" << endl;
	os.setf(flags);
	os.precision(sz);
	return os;
}

ostream &operator << (ostream &os, const dse3 &t)
{
	ios_base::fmtflags flags = os.setf(ios::left | ios::fixed);
	streamsize sz = os.precision(3);
	os << "[ ";
	for ( int i = 0; i < 6; i++ )
	{ 
		if ( t[i] >= 0.0 ) os << " " << setw(6) << t[i] << " ";
		else os << setw(7) << t[i] << " ";
	}
	os << "];" << endl;
	os.setf(flags);
	os.precision(sz);
	return os;
}

ostream &operator << (ostream &os, const SE3 &T)
{
	ios_base::fmtflags flags = os.setf(ios::left | ios::fixed);
	streamsize sz = os.precision(3);
	os << "[" << endl;
	for ( int i = 0; i < 4; i++ )
	{
		for ( int j = 0; j < 4; j++ )
		{
			if ( T(i,j) >= 0.0 ) os << " " << setw(6) << T(i,j) << " ";
			else os << setw(7) << T(i,j) << " ";
		}
		os << ";" << endl;
	}
	os << "];" << endl;
	os.setf(flags);
	os.precision(sz);
	return os;
}

// Vec3 size: half of each dimension i.e. size[0] = width/2, size[1] = depth/2, size[2] = height/2
Inertia BoxInertia(double density, const Vec3 &size)
{
	double mass = (double)8.0 * density * size[0] * size[1] * size[2];

	return Inertia(
		mass,	// mass
		mass * (size[1] * size[1] + size[2] * size[2]) * SR_ONETHIRD,	// ix
		mass * (size[0] * size[0] + size[2] * size[2]) * SR_ONETHIRD,	// iy
		mass * (size[0] * size[0] + size[1] * size[1]) * SR_ONETHIRD	// iz
		);
}

Inertia SphereInertia(double density, double rad)
{
	double mass = density * SR_FOURTHIRD * SR_PI * rad * rad * rad;
	double i = (double)0.4 * mass * rad * rad;

	return Inertia(mass, i, i, i);
}

Inertia CylinderInertia(double density, double rad, double height)
{
	rad *= rad;
	double mass = density * SR_PI * rad * height;
	double ix = mass * height * height / (double)12.0 + (double)0.25 * mass * rad;

	return Inertia(
		mass,						// mass
		ix,							// ix
		ix,							// iy
		0.5 * mass * rad	// iz
		);
}


Inertia	 CapsuleInertia(double density, double rad, double height)
{
	double cyl_mass = density * SR_PI * rad * rad * height;
	double sph_mass	= density * SR_FOURTHIRD * SR_PI * rad * rad * rad;

	double ix = cyl_mass * height * height / (double)12.0 + (double)0.25 * cyl_mass * rad			// cylinder
				+ (double)0.4 * sph_mass * rad * rad + (double)0.25 * sph_mass * height * height;	// sphere

	return Inertia(
		cyl_mass + sph_mass,													// mass
		ix,																		// ix
		ix,																		// iy
		0.5 * cyl_mass * rad * rad + (double)0.4 * sph_mass * rad * rad	// iz
		);
}




