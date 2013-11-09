#include "dodoMath/PCHeaders.h"

#include "dodoBase/Assert.h"
#include "dodoBase/String.h"
#include "dodoBase/Const.h"
#include "dodoMath/Vector.h"
#include "dodoMath/Matrix4x4.h"
#include "dodoMath/Quaternion.h"

namespace dodoEng
{
	
	//--------------------------------------------------------------------
	// Default constructor
	// Creates an identity quaternion (1, v(0,0,0))
	Quaternion::Quaternion()
		: m_w(1.0f), m_v(0.0f, 0.0f, 0.0f)
	{
	}
	
	//--------------------------------------------------------------------
	// Constructor
	// Creates a quaternion that represents a rotation of [angle] radians
	// around the [rotAxis] axis.
	Quaternion::Quaternion(float angle, const Vector& rotAxis)
		: m_w( cosf(angle) ), m_v( sinf(angle)*Unit(rotAxis) )
	{
	}
	
	//--------------------------------------------------------------------
	// Constructor
	// This private constructor assumes that the parameters specify a unit
	// quaternion.
	Quaternion::Quaternion(float w, float x, float y, float z)
		: m_w(w), m_v(x, y, z)
	{
	}
	
	//--------------------------------------------------------------------
	// Destructor
	Quaternion::~Quaternion()
	{
	}
	
	//--------------------------------------------------------------------
	// Equality
	bool Quaternion::operator==(const Quaternion& rhs) const
	{
		return (fabs(m_w - rhs.m_w) < EPSILON) && (m_v == rhs.m_v);
	}

	//--------------------------------------------------------------------
	// Inequality
	bool Quaternion::operator!=(const Quaternion& rhs) const
	{
		return !(this->operator==(rhs));
	}

	//--------------------------------------------------------------------
	// Convert a rotation matrix into a quaternion.
	// Algorithm is described in "Real-Time Rendering" by Moller and
	// Haines, pg. 48
	//
	// Throws an exception if the matrix parameter  is not a rotation
	// matrix.
	void Quaternion::CreateFromMatrix(const Matrix4x4& m)
	{
		//trace
		const float tr = m(0,0) + m(1,1) + m(2,2);
		
		float s, w, x, y, z;
		if (tr  > 0.0f)
		{
			//w is greatest
			s = sqrtf(tr + 1.0f);
			w = s * 0.5f;
			s = 0.5f/s;
			
			x = (m(2,1) - m(1,2))*s;
			y = (m(2,0) - m(0,2))*s;
			z = (m(1,0) - m(0,1))*s;
		}
		else
		{
			if ( (m(2,2)>m(1,1)) && (m(2,2)>m(0,0)) )
			{
				//z is greatest
				s = sqrtf(m(2,2) - m(1,1) - m(0,0) + 1.0f);
				z = s*0.5f;
				s = 0.5f/s;
				w = (m(1,0) - m(0,1))*s;
				x = (m(2,0) + m(0,2))*s;
				y = (m(2,1) + m(1,2))*s;
			}
			else if ( m(1,1)>m(0,0) )
			{
				//y is gratest
				s = sqrtf(m(1,1) - m(2,2) - m(0,0) + 1.0f);
				y = s*0.5f;
				s = 0.5f/s;
				w = (m(2,0) - m(0,2))*s;
				x = (m(1,0) + m(0,1))*s;
				z = (m(2,1) + m(1,2))*s;
			}
			else
			{
				//x is greatest
				s = sqrtf(m(0,0) - m(1,1) - m(2,2) + 1.0f);
				x = s*0.5f;
				s = 0.5f/s;
				w = (m(2,1) - m(1,2))*s;
				y = (m(0,1) + m(1,0))*s;
				z = (m(2,0) + m(0,2))*s;
			}
		}
		
		m_w = w;
		m_v.Set(x, y, z);
	}
	
	//--------------------------------------------------------------------
	// Returns the vector (imaginary) component of the quaternion
	const Vector& Quaternion::GetV() const
	{
		return m_v;
	}
	
	Vector& Quaternion::GetV()
	{
		return m_v;
	}
	
	//--------------------------------------------------------------------
	// Returns the w (real) component of the quaternion
	const float& Quaternion::GetW() const
	{
		return m_w;
	}
	
	float& Quaternion::GetW()
	{
		return m_w;
	}
	
	//--------------------------------------------------------------------
	// Quaternion multiplication
	const Quaternion Quaternion::operator*(const Quaternion& rhs) const
	{
		Vector temp = (m_w*rhs.m_v) + (rhs.m_w*m_v) + (m_v ^ rhs.m_v);
		
		return Quaternion(
			m_w*rhs.m_w - m_v*rhs.m_v,
			temp[0],
			temp[1],
			temp[2] );
	}
	
	//--------------------------------------------------------------------
	// Quaternion multiplication
	void Quaternion::operator*=(const Quaternion& rhs)
	{
		*this = *this * rhs;
	}
	
	//--------------------------------------------------------------------
	// Calculate the conjugate of the Quaternion.
	// Conjugate of (w, v) = (w, -v)
	const Quaternion Quaternion::Conjugate() const
	{
		return Quaternion( m_w, -1.0f * m_v );
	}
	
	//--------------------------------------------------------------------
	// Calculate the conjugate of the Quaternion q.
	// Conjugate of (w, v) = (w, -v)
	const Quaternion Conjugate(const Quaternion& q)
	{
		return q.Conjugate();
	}
	
	//--------------------------------------------------------------------
	// Calculate the norm of the Quaternion.
	// Norm of (w, v) = w^2 + Length(v)^2
	float Quaternion::Norm() const
	{
		const float l = m_v.Length();
		return (m_w*m_w + l*l);
	}
	
	//--------------------------------------------------------------------
	// Calculate the norm of the Quaternion.
	// Norm of (w, v) = w^2 + Length(v)^2
	float Norm(const Quaternion& q)
	{
		return q.Norm();
	}
	
	//--------------------------------------------------------------------
	// Calculate the inverse of the Quaternion.  Inverse of the quaternion
	// represents the opposite rotation of the quternion.
	// Inverse of (w, v) = 1/Norm(q) * Conjugate(q)
	//
	// Throws an exception if the Norm of a quaternion is 0.
	const Quaternion Quaternion::Inverse() const
	{
		const float norm = dodoEng::Norm(*this);
		
		DE_ASSERT(fabs(norm) > EPSILON);
		
		Quaternion temp(1.0f/norm, 0.0f, 0.0f, 0.0f);
		
		return Quaternion(  temp * dodoEng::Conjugate(*this) );
	}
	
	//--------------------------------------------------------------------
	// Calculate the inverse of the Quaternion.  Inverse of the quaternion
	// represents the opposite rotation of the quternion.
	// Inverse of (w, v) = 1/Norm(q) * Conjugate(q)
	//
	// Throws an exception if the Norm of a quaternion is 0.
	const Quaternion Inverse(const Quaternion& q)
	{
		return q.Inverse();
	}
	
	//--------------------------------------------------------------------
	// Performs Spherical Linear Interpolation (SLERP) of two quaternions.
	// SLERP is an operation that, given two unit quaternions and a
	// parameter t ( 0<=t<=1), computes an interpolated quaternion.
	//		"Real-Time Rendering" by Moller and haines, pg. 48
	//
	// Algorith is from "Advanced Animation and Rendering Techniques" by
	// Watt & Watt, pg. 364
	const Quaternion Slerp(const Quaternion& q, const Quaternion& r, float t)
	{
		const float QUATERNION_EPSILON = EPSILON * 100.0f;
		
		const Vector& qv = q.GetV();
		const Vector& rv = r.GetV();
		
		const float& qx = qv[0]; const float& qy = qv[1]; const float& qz = qv[2];
		const float& rx = rv[0]; const float& ry = rv[1]; const float& rz = rv[2];
		
		const float cosom = qx*rx + qy*ry + qz*rz + q.GetW()*r.GetW();
		
		if ( (1.0f + cosom) > QUATERNION_EPSILON)
		{
			float sclq, sclr;
			
			if ( (1.0f-cosom) > QUATERNION_EPSILON )
			{
				const float omega = acosf(cosom);
				const float sinom = sinf(omega);
				sclq = sinf( (1.0f-t)*omega)/sinom;
				sclr = sinf( t*omega )/sinom;
			}
			else
			{
				sclq = 1.0f - t;
				sclr = t;
			}
			
			return Quaternion( sclq*q.GetW() + sclr*r.GetW(),
				Vector(sclq*qx + sclr*rx,
				sclq*qy + sclr*ry,
				sclq*qz + sclr*rz) );
		}
		else
		{
			const float sclq = sinf(1.0f-t)*HALFPI;
			const float sclr = sinf(t*HALFPI);
			
			return Quaternion( qz,
				Vector(sclq*qx + sclr*(-qy),
				sclq*qy + sclr*(qx),
				sclq*qz + sclr*(-q.GetW())) );
		}
	}

	//--------------------------------------------------------------------
	const std::string ToString(const Quaternion& q)
	{
		return "[" + dodoEng::ToString(q.GetW()) + ", " + dodoEng::ToString(q.GetV()) + "]";
	}
}