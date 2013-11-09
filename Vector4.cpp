#include "dodoMath/PCHeaders.h"

#include "dodoBase/Assert.h"
#include "dodoBase/String.h"
#include "dodoBase/Const.h"
#include "dodoMath/Point.h"
#include "dodoMath/Vector.h"
#include "dodoMath/Matrix4x4.h"

#include "dodoMath/Vector4.h"

namespace dodoEng
{
	
	//--------------------------------------------------------------------
	Vector4::Vector4()
		: BaseQuad(0.0f, 0.0f, 0.0f, 0.0f)
	{
	}
	
	//--------------------------------------------------------------------
	Vector4::Vector4(float x, float y, float z, float h)
		: BaseQuad(x, y, z, h)
	{
	}
	
	//--------------------------------------------------------------------
	Vector4::Vector4(const Vector& v)
		: BaseQuad(v[X], v[Y], v[Z], 0.0f)
	{
	}
	
	//--------------------------------------------------------------------
	Vector4::Vector4(const Point& pt)
		: BaseQuad(pt[X], pt[Y], pt[Z], 1.0f)
	{
	}
	
	//--------------------------------------------------------------------
	// Destructor
	Vector4::~Vector4()
	{
	}
	
	//--------------------------------------------------------------------
	void Vector4::Homogenize()
	{
		DE_ASSERT(fabs(m_data[H]) > EPSILON);
		const float scale = 1.0f/m_data[H];
		
		m_data[X] *= scale;
		m_data[Y] *= scale;
		m_data[Z] *= scale;
		m_data[H] = 1.0f;
	}
	
	//--------------------------------------------------------------------
	const Vector4 Vector4::Homogenized() const
	{
		Vector4 res = *this;
		res.Homogenize();
		return res;
	}
	
	//--------------------------------------------------------------------
	const Vector4 Homogenized(const Vector4& v)
	{
		return v.Homogenized();
	}
	
	//--------------------------------------------------------------------
	// Add two vectors.  Result is a vector.
	const Vector4 Vector4::operator+(const Vector4& rhs) const
	{
		return Vector4(
			m_data[X] + rhs.m_data[X],
			m_data[Y] + rhs.m_data[Y],
			m_data[Z] + rhs.m_data[Z],
			m_data[H] + rhs.m_data[H] );
	}
	
	//--------------------------------------------------------------------
	// Subtract two vectors.  Result is a Vector.
	const Vector4 Vector4::operator-(const Vector4& rhs) const
	{
		return Vector4(
			m_data[X] - rhs.m_data[X],
			m_data[Y] - rhs.m_data[Y],
			m_data[Z] - rhs.m_data[Z],
			m_data[H] - rhs.m_data[H] );
	}
	
	//--------------------------------------------------------------------
	// Add a vector to the this vector.
	void Vector4::operator+=(const Vector4& rhs)
	{
		m_data[X] += rhs.m_data[X];
		m_data[Y] += rhs.m_data[Y];
		m_data[Z] += rhs.m_data[Z];
		m_data[H] += rhs.m_data[H];
	}
	
	//--------------------------------------------------------------------
	// Subtract a vector from the this vector
	void Vector4::operator-=(const Vector4& rhs)
	{
		m_data[X] -= rhs.m_data[X];
		m_data[Y] -= rhs.m_data[Y];
		m_data[Z] -= rhs.m_data[Z];
		m_data[H] -= rhs.m_data[H];
	}
	
	//--------------------------------------------------------------------
	// Compare two vectors for equality.  Vectors are equal if their
	// corresponding dimensions are equal.
	bool Vector4::operator==(const Vector4& rhs) const
	{
		return ( (fabs(m_data[X] - rhs.m_data[X]) < EPSILON)
			&& (fabs(m_data[Y] - rhs.m_data[Y]) < EPSILON)
			&& (fabs(m_data[Z] - rhs.m_data[Z]) < EPSILON)
			&& (fabs(m_data[H] - rhs.m_data[H]) < EPSILON) );
	}
	
	//--------------------------------------------------------------------
	// Compare two vectors for equality.  Vectors are equal if their
	// corresponding dimensions are equal.
	bool Vector4::operator!=(const Vector4& rhs) const
	{
		return !(*this == rhs);
	}
	
	//--------------------------------------------------------------------
	const Vector Vector4::ToVector() const
	{
		DE_ASSERT(fabs(m_data[H]) <= EPSILON);
		
		return Vector(m_data[X], m_data[Y], m_data[Z]);
	}
	
	//--------------------------------------------------------------------
	const Point Vector4::ToPoint() const
	{
		DE_ASSERT(fabs(m_data[H]) > EPSILON);
		const float scale = 1.0f/m_data[H];
		
		return Point(m_data[X]*scale, m_data[Y]*scale, m_data[Z]*scale);
	}
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
const dodoEng::Vector4 operator*(const dodoEng::Matrix4x4& m, const dodoEng::Vector4& v)
{
	using dodoEng::X;
	using dodoEng::Y;
	using dodoEng::Z;
	using dodoEng::H;

	return dodoEng::Vector4(
		v[X]*m(0,0) + v[Y]*m(0,1) + v[Z]*m(0,2) + v[H]*m(0,3),
		v[X]*m(1,0) + v[Y]*m(1,1) + v[Z]*m(1,2) + v[H]*m(1,3),
		v[X]*m(2,0) + v[Y]*m(2,1) + v[Z]*m(2,2) + v[H]*m(2,3),
		v[X]*m(3,0) + v[Y]*m(3,1) + v[Z]*m(3,2) + v[H]*m(3,3));
}