// Point.h: interface for the Point class.
// Author: Davor Mrkoci
//
// This class represents a three dimensional point in space.  It
// derives from BaseTriple.
//
// The class provides functionality for comparing points, adding and
// subtracting vectors, and applying transforms.
//
// The class interacts with Point, Vector, and Matrix4x4 classes.

#if !defined(DODOENG_POINT_H_)
#define DODOENG_POINT_H_

#include "dodoBase/BaseTriple.h"

namespace dodoEng
{
	class Vector;
	class Matrix4x4;
}

namespace dodoEng
{
	class Point : public BaseTriple  
	{
		//--------------------------------------------------------------------
		// Interface
		//--------------------------------------------------------------------
	public:
		// Default constructor
		// Creates a point (0, 0, 0)
		Point();
		
		// Constructor
		// Creates a point (x, y, z)
		Point(float x, float y, float z);
		
		// Destructor
		virtual ~Point();
		
		// Point & point math operations
		// note: addition does not make sense because the homogenous
		//		 component is assumed to be 1.
		const Vector operator-(const Point& rhs) const;
		bool operator==(const Point& rhs) const;
		bool operator!=(const Point& rhs) const;
		
		// Calculate the average location of nPts number of points.  Array
		// has to have nPts number of Point objects.
		friend	const Point Average(Point pt[], int nPts);
		
		// Point & Vector math operations
		const Vector ToVector() const;
		void operator+=(const Vector& v);
		void operator-=(const Vector& v);
	};
}

const dodoEng::Point operator+(const dodoEng::Point& pt, const dodoEng::Vector& v);
const dodoEng::Point operator+(const dodoEng::Vector& v, const dodoEng::Point& pt);
const dodoEng::Point operator-(const dodoEng::Point& pt, const dodoEng::Vector& v);

const float Distance(const dodoEng::Point& p0, const dodoEng::Point& p1);

// Apply transform m to the point.  The result is a transformed point.
// Since point is a single column matrix, it can only be post-
// multiplied with a matrix.
//
// For example, M*Pt is legal, while Pt*M is not.
// This relationship is enforced.
const dodoEng::Point operator*(const dodoEng::Matrix4x4& m, const dodoEng::Point& p);

#endif
