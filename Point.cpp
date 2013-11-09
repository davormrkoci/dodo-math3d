#include "dodoMath/PCHeaders.h"

#include "dodoBase/Assert.h"
#include "dodoBase/Const.h"
#include "dodoMath/Vector.h"
#include "dodoMath/Matrix4x4.h"
#include "dodoMath/Point.h"

namespace dodoEng
{
	
	//--------------------------------------------------------------------
	// Default constructor
	// Creates a point (0, 0, 0)
	Point::Point()
		: BaseTriple()
	{
	}
	
	//--------------------------------------------------------------------
	// Constructor
	// Creates a point (x, y, z)
	Point::Point(float x, float y, float z)
		: BaseTriple(x, y, z)
	{
	}
	
	//--------------------------------------------------------------------
	// Destructor
	Point::~Point()
	{
	}
	
	//--------------------------------------------------------------------
	// Subtract a point from another point.  Result is a vector from the
	// first point to the second.
	const Vector Point::operator-(const Point& rhs) const
	{
		return Vector(
			m_data[X] - rhs.m_data[X],
			m_data[Y] - rhs.m_data[Y],
			m_data[Z] - rhs.m_data[Z]);
	}
	
	//--------------------------------------------------------------------
	// Compare two points for equality.  If all corresponding coordinates
	// are the same, the function returns true.
	bool Point::operator==(const Point& rhs) const
	{
		return ( (fabs(m_data[X] - rhs.m_data[X]) < EPSILON)
			&& (fabs(m_data[Y] - rhs.m_data[Y]) < EPSILON)
			&& (fabs(m_data[Z] - rhs.m_data[Z]) < EPSILON) );
	}
	
	//--------------------------------------------------------------------
	// Compare two points for equality.  If any of the corresponding 
	// coordinates are are different, it return true.
	bool Point::operator!=(const Point& rhs) const
	{
		return !(*this == rhs);
	}
	
	//--------------------------------------------------------------------
	// Calculate the average location of nPts number of points.  Array
	// has to have nPts number of Point objects.
	//
	// Throws an exception if number of points is <= 0
	const Point Average(Point pt[], int nPts)
	{
		DE_ASSERT(nPts > 0);
		
		float x = 0.0f;
		float y = 0.0f;
		float z = 0.0f;
		
		for (int i=0; i<nPts; ++i)
		{
			x += pt[i][X];
			y += pt[i][Y];
			z += pt[i][Z];
		}
		
		const float scale = 1.0f/nPts;
		
		return Point(
			x * scale,
			y * scale,
			z * scale );
	}
	
	//--------------------------------------------------------------------
	// Returns a vector going from the origin (0,0,0) to the point.
	const Vector Point::ToVector() const
	{
		return Vector(m_data[X], m_data[Y], m_data[Z]);
	}
	
	//--------------------------------------------------------------------
	// Moves the point by a vector.
	void Point::operator+=(const Vector& v)
	{
		m_data[X] += v[X];
		m_data[Y] += v[Y];
		m_data[Z] += v[Z];
	}
	
	//--------------------------------------------------------------------
	// Moves the point by the inverse of a vector.
	void Point::operator-=(const Vector& v)
	{
		m_data[X] -= v[X];
		m_data[Y] -= v[Y];
		m_data[Z] -= v[Z];
	}
}

const dodoEng::Point operator+(const dodoEng::Point& pt, const dodoEng::Vector& v)
{
	using dodoEng::X;
	using dodoEng::Y;
	using dodoEng::Z;

	return dodoEng::Point(pt[0] + v[0], pt[1] + v[1], pt[2] + v[2]);
}

const dodoEng::Point operator+(const dodoEng::Vector& v, const dodoEng::Point& pt)
{
	return pt + v;
}

const dodoEng::Point operator-(const dodoEng::Point& pt, const dodoEng::Vector& v)
{
	using dodoEng::X;
	using dodoEng::Y;
	using dodoEng::Z;

	return dodoEng::Point(pt[0] - v[0], pt[1] - v[1], pt[2] - v[2]);
}

const float Distance(const dodoEng::Point& p0, const dodoEng::Point& p1)
{
	return(sqrtf(p0[0]-p1[0])*((p0[0]-p1[0]))+(p0[1]-p1[1])*(p0[1]-p1[1])+(p0[2]-p1[2])*(p0[2]-p1[2]));

}

//--------------------------------------------------------------------
// Apply transform m to the point.  The result is a transformed point.
// Since point is a single column matrix, it can only be post-
// multiplied with a matrix.
//
// For example, M*Pt is legal, while Pt*M is not.
// This relationship is enforced.
//
// Throws an exception if resulting point can not be homoginized, i.e.
// w = 0
const dodoEng::Point operator*(const dodoEng::Matrix4x4& m, const dodoEng::Point& pt)
{
	using dodoEng::X;
	using dodoEng::Y;
	using dodoEng::Z;

	const float newH = pt[X]*m(3,0) + pt[Y]*m(3,1) +
		pt[Z]*m(3,2) + m(3,3);
	DE_ASSERT(fabs(newH) > dodoEng::EPSILON);
	
	const float scale = 1.0f/newH;
	
	return dodoEng::Point(
		(pt[X]*m(0,0) + pt[Y]*m(0,1) +
		pt[Z]*m(0,2) + m(0,3)) * scale,
		
		(pt[X]*m(1,0) + pt[Y]*m(1,1) +
		pt[Z]*m(1,2) + m(1,3)) * scale,
		
		(pt[X]*m(2,0) + pt[Y]*m(2,1) +
		pt[Z]*m(2,2) + m(2,3)) * scale );
}