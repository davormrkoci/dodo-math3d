// Vector.h: interface for the Vector class.
// Author: Davor Mrkoci
//
// This class represents a three dimensional vector.  It derives from
// BaseTriple.
//
// Support for many mathematical functions is provided including
// dot product, cross product, addition, subtraction, comparison, etc.
// 
// This class interacts with Point, Vector, and Matrix4x4 classes.

#if !defined(DODOENG_VECTOR_H_)
#define DODOENG_VECTOR_H_

#include "BaseTriple.h"

namespace dodoEng
{
	class Point;
	class Matrix4x4;
}

namespace dodoEng
{
	class Vector : public BaseTriple
	{
		//--------------------------------------------------------------------
		// Interface
		//--------------------------------------------------------------------
	public:
		// Default constructor
		// Creates the vector (0,0,1)
		Vector();
		
		// Constructor
		// Creates the vector (x, y, z)
		Vector(float x, float y, float z);
		
		// Destructor
		virtual ~Vector();
		
		// Returns the length of the vector
		float Length() const;
		
		// Normalize the vector.  Make it a unit vector (length = 1).
		// Returns (0.0f, 0.0f, 0.0f) if length = 0
		void Normalize();

		// Returns the unit vector in the same direction as *this.
		const Vector Unit() const;
		
		// Vector & Vector math operations
		const Vector operator+(const Vector& rhs) const;
		const Vector operator-(const Vector& rhs) const;
		//dot product
		float operator*(const Vector& rhs) const;
		//cross product
		const Vector operator^(const Vector& rhs) const;
		void operator+=(const Vector& rhs);
		void operator-=(const Vector& rhs);
		void operator^=(const Vector& rhs);
		bool operator==(const Vector& rhs) const;
		bool operator!=(const Vector& rhs) const;
		
		// Vector & scalar math operations
		const Vector operator/(float d) const;
		const Vector operator*(float d) const;
		void operator*=(float d);
		void operator/=(float d);
	};
	
	//--------------------------------------------------------------------
	// Global functions related to this class
	//--------------------------------------------------------------------
	
	// Allows writing Lenth(v)
	float Length(const Vector& v);
	
	// Allows writing Unit(v)
	// Returns (0.0f, 0.0f, 0.0f) if length = 0
	const Vector Unit(const Vector& v);
	
	// Allows writing DotProduct(u, v)
	float DotProduct(const Vector& v1, const Vector& v2);
	
	// Allows writing CrossProduct(u,v)
	const Vector CrossProduct(const Vector& v1, const Vector& v2);
}

const dodoEng::Vector operator*(float d, const dodoEng::Vector& v);		
		
// Apply transform m to the vector v.  The result is a transformed
// vector.
// Since a vector is a single column matrix, it can only be post-
// multiplied with a matrix.
//
// For example, M*v is legal, while v*M is not.
// This relationship is enforced.
const dodoEng::Vector operator*(const dodoEng::Matrix4x4& m, const dodoEng::Vector& v);

#endif
