#include "Assert.h"
#include "String.h"
#include "Const.h"
#include "Point.h"
#include "Matrix4x4.h"
#include "Vector.h"

namespace dodoEng
{
	
	//--------------------------------------------------------------------
	// Default constructor
	// Creates the vector (0,0,0)
	Vector::Vector()
		: BaseTriple(0.0f, 0.0f, 0.0f)
	{
	}
	
	//--------------------------------------------------------------------
	// Constructor
	// Creates the vector (x, y, z)
	Vector::Vector(float x, float y, float z)
		: BaseTriple(x, y, z)
	{
	}
	
	//--------------------------------------------------------------------
	// Destructor
	Vector::~Vector()
	{
	}
	
	//--------------------------------------------------------------------
	// Returns the length of the vector
	float Vector::Length() const
	{
		return sqrtf((*this) * (*this));
	}
	
	//--------------------------------------------------------------------
	// Normalize the vector.  Make it a unit vector (length = 1).
	void Vector::Normalize()
	{
		const float l = Length();
		
		DE_ASSERT(l > EPSILON);
		
		if (l <= EPSILON)
		{
			m_data[X] = 0.0f;
			m_data[Y] = 0.0f;
			m_data[Z] = 0.0f;
		}
		else
		{
			const float scale = 1.0f/l;
			m_data[X] *= scale;
			m_data[Y] *= scale;
			m_data[Z] *= scale;
		}
	}
	
	//--------------------------------------------------------------------
	// Returns the unit vector in the same direction as the original.
	const Vector Vector::Unit() const
	{
		Vector v = *this;
		v.Normalize();
		return v;
	}
	
	//--------------------------------------------------------------------
	// Add two vectors.  Result is a vector.
	const Vector Vector::operator+(const Vector& rhs) const
	{
		return Vector(
			m_data[X] + rhs.m_data[X],
			m_data[Y] + rhs.m_data[Y],
			m_data[Z] + rhs.m_data[Z] );
	}
	
	//--------------------------------------------------------------------
	// Subtract two vectors.  Result is a Vector.
	const Vector Vector::operator-(const Vector& rhs) const
	{
		return Vector(
			m_data[X] - rhs.m_data[X],
			m_data[Y] - rhs.m_data[Y],
			m_data[Z] - rhs.m_data[Z] );
	}
	
	//--------------------------------------------------------------------
	// Calculate the dot product of two vectors.
	// v1*v2 = v1x*v2x + v1y*v2y + v1z*v2z
	float Vector::operator*(const Vector& rhs) const
	{
		return (m_data[X]*rhs.m_data[X]
			+ m_data[Y]*rhs.m_data[Y]
			+ m_data[Z]*rhs.m_data[Z] );
	}
	
	//--------------------------------------------------------------------
	// Calculate the cross product of two vectors.  The result is a vector
	// perpendicular to two original vectors.
	const Vector Vector::operator^(const Vector& rhs) const
	{
		return Vector(
			m_data[Y]*rhs.m_data[Z] - m_data[Z]*rhs.m_data[Y],
			m_data[Z]*rhs.m_data[X] - m_data[X]*rhs.m_data[Z],
			m_data[X]*rhs.m_data[Y] - m_data[Y]*rhs.m_data[X] );
	}
	
	//--------------------------------------------------------------------
	// Add a vector to the this vector.
	void Vector::operator+=(const Vector& rhs)
	{
		m_data[X] += rhs.m_data[X];
		m_data[Y] += rhs.m_data[Y];
		m_data[Z] += rhs.m_data[Z];
	}
	
	//--------------------------------------------------------------------
	// Subtract a vector from the this vector
	void Vector::operator-=(const Vector& rhs)
	{
		m_data[X] -= rhs.m_data[X];
		m_data[Y] -= rhs.m_data[Y];
		m_data[Z] -= rhs.m_data[Z];
	}
	
	//--------------------------------------------------------------------
	// Calculate the cross product of two vectors, and put the result in
	// the this vector.
	void Vector::operator^=(const Vector& rhs)
	{
		*this = *this ^ rhs;;
	}
	
	//--------------------------------------------------------------------
	// Compare two vectors for equality.  Vectors are equal if their
	// corresponding dimensions are equal.
	bool Vector::operator==(const Vector& rhs) const
	{
		return ( (fabs(m_data[X] - rhs.m_data[X]) < EPSILON)
			&& (fabs(m_data[Y] - rhs.m_data[Y]) < EPSILON)
			&& (fabs(m_data[Z] - rhs.m_data[Z]) < EPSILON) );
	}
	
	//--------------------------------------------------------------------
	// Compare two vectors for equality.  Vectors are equal if their
	// corresponding dimensions are equal.
	bool Vector::operator!=(const Vector& rhs) const
	{
		return !(*this == rhs);
	}
	
	//--------------------------------------------------------------------
	// Divide a vector by a scalar.  The result is a Vector.
	// Throws an exception if divide by zero occurs, i.e.
	// parameter d is 0.
	const Vector Vector::operator/(float d) const
	{
		DE_ASSERT(d > EPSILON);
		
		const float scale = 1.0f/d;
		return Vector(
			m_data[X] * scale,
			m_data[Y] * scale,
			m_data[Z] * scale );
	}
	
	//--------------------------------------------------------------------
	// Multiply a vector by a scalar.  the result is a Vector.
	const Vector Vector::operator*(float d) const
	{
		return Vector(
			m_data[X] * d,
			m_data[Y] * d,
			m_data[Z] * d );
	}
	
	//--------------------------------------------------------------------
	// Scale this vector by a scalar.
	void Vector::operator*=(float d)
	{
		m_data[X] *= d;
		m_data[Y] *= d;
		m_data[Z] *= d;
	}
	
	//--------------------------------------------------------------------
	// Scale this vector by the 1/scalar.
	void Vector::operator/=(float d)
	{
		DE_ASSERT(d > EPSILON);
		
		const float scale = 1.0f/d;
		m_data[X] *= scale;
		m_data[Y] *= scale;
		m_data[Z] *= scale;
	}
	
	//--------------------------------------------------------------------
	// Get the length of a vector v.
	// Friend function that allows writing Length(v).
	float Length(const Vector& v)
	{
		return v.Length();
	}
	
	//--------------------------------------------------------------------
	// Get the unit vector pointing in the same direction as vector v.
	// Friend function that allows writing Unit(v)
	// Throws exception if the length of the vector is 0.
	const Vector Unit(const Vector& v)
	{
		return v.Unit();
	}
	
	//--------------------------------------------------------------------
	// Calculate the dot product of two vectors.
	// Friend function that allows writing DotProduct(v1, v2)
	float DotProduct(const Vector& v1, const Vector& v2)
	{
		return (v1*v2);
	}
	
	//--------------------------------------------------------------------
	// Calculate the cross product of two vectors.
	// Friend function that allows writing CrossProduct(v1, v2)
	const Vector CrossProduct(const Vector& v1, const Vector& v2)
	{
		return (v1^v2);
	}
		
	//--------------------------------------------------------------------
	// Move a point by the vector v. (pt + v)
	const Point operator+(const Point& pt, const Vector& v)
	{
		return Point(
			pt[X] + v[X],
			pt[Y] + v[Y],
			pt[Z] + v[Z] );
	}
	
	//--------------------------------------------------------------------
	// Move a point by the vector v. (v + pt)
	const Point operator+(const Vector& v, const Point& pt)
	{
		return (pt + v);
	}
	
	//--------------------------------------------------------------------
	// Move a point by the opposite of vector v. (pt - v)
	const Point operator-(const Point& pt, const Vector& v)
	{
		return Point(
			pt[X] - v[X],
			pt[Y] - v[Y],
			pt[Z] - v[Z] );
	}
}

//--------------------------------------------------------------------
// Multiply the vector by a scalar. (d * v)
const dodoEng::Vector operator*(float d, const dodoEng::Vector& v)
{
	return v.operator*(d);
}

//--------------------------------------------------------------------
// Apply transform m to the vector v.  The result is a transformed
// vector.
//
// Since a vector is a single column matrix, it can only be post-
// multiplied with a matrix.
//
// For example, M*v is legal, while v*M is not.
// This relationship is enforced.
//
// Throws an exception if matrix is not a vector transform matrix,
// i.e. have the bottom row different from [0 0 0 ?].
const dodoEng::Vector operator*(const dodoEng::Matrix4x4& m, const dodoEng::Vector& v)
{
	using dodoEng::X;
	using dodoEng::Y;
	using dodoEng::Z;

	//check if bottom row of matrix is 0 0 0 ?
	//if not, throw exception
	DE_ASSERT(fabs(m(3,0)) < dodoEng::EPSILON && fabs(m(3,1)) < dodoEng::EPSILON && fabs(m(3,2)) < dodoEng::EPSILON);
	return dodoEng::Vector(
		m(0,0)*v[X] + m(0,1)*v[Y] + m(0,2)*v[Z],
		m(1,0)*v[X] + m(1,1)*v[Y] + m(1,2)*v[Z],
		m(2,0)*v[X] + m(2,1)*v[Y] + m(2,2)*v[Z]);
}
