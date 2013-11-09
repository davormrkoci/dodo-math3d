// Quaternion.h: interface for the Quaternion class.
// Author: Davor Mrkoci
//
// This class represents a unit quaternion (Norm(q) = 1).  This type
// of quaternion is very usefull in representing rotations and
// orientations.
//
// Functions are provided for changing quaternions into rotation
// martrices, and vice-versa.  Quaternion interpolation and
// multiplication functions are also provided.

#if !defined(DODOENG_QUATERNION_H_)
#define DODOENG_QUATERNION_H_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "Vector.h"

namespace dodoEng
{
	class Vector;
	class Matrix4x4;
}

namespace dodoEng
{
	class Quaternion  
	{
		//--------------------------------------------------------------------
		// Interface
		//--------------------------------------------------------------------
	public:
		// Default constructor
		// Creates an identity quaternion (1, v(0,0,0))
		Quaternion();
		
		// Constructor
		// Creates a quaternion that represents a rotation of [angle]
		// radians around the [rotAxis] axis.
		Quaternion(float angle, const Vector& rotAxis);
		
		// Destructor
		// Notice that the destructor is not virtual.  This means that this
		// class is not meant to be inherited from.
		~Quaternion();
		
		// Convert a rotation matrix into a quaternion.
		// Algorithm is described in "Real-Time Rendering" by Moller and
		// Haines, pg. 48
		// Throws an exception if the matrix passed in is not a rotation
		// matrix.
		void CreateFromMatrix(const Matrix4x4& m);
		
		// Returns the vector (imaginary) component of the quaternion
		const Vector& GetV() const;
		Vector& GetV();
		
		// Returns the vector (imaginary) component of the quaternion
		const float& GetW() const;
		float& GetW();
		
		// Quaternion multiplication
		const Quaternion operator*(const Quaternion& rhs) const;
		void operator*=(const Quaternion& rhs);
		
		// Calculate the conjugate of the Quaternion.
		// Conjugate of (w, v) = (w, -v)
		const Quaternion Conjugate() const;
		
		// Calculate the norm of the Quaternion.
		// Norm of (w, v) = w^2 + Length(v)^2
		float Norm() const;
		
		// Calculate the inverse of the Quaternion.  Inverse of the
		// quaternion represents the opposite rotation of the quternion.
		// Inverse of (w, v) = 1/Norm(q) * Conjugate(q)
		const Quaternion Inverse() const;
		
		bool operator==(const Quaternion& rhs) const;
		bool operator!=(const Quaternion& rhs) const;

		// Performs Spherical Linear Interpolation (SLERP) on two
		// quaternions. SLERP is an operation that, given two unit
		// quaternions and a parameter t ( 0<=t<=1), computes an
		// interpolated quaternion.
		friend const Quaternion Slerp(
			const Quaternion& q, const Quaternion& r, float t);
		
		//--------------------------------------------------------------------
		// Implementation
		//--------------------------------------------------------------------
	private:
		Quaternion(float w, float x, float y, float z);
		
		//--------------------------------------------------------------------
		// Data Members
		//--------------------------------------------------------------------
	private:
		//imaginary (vector) part of the quaternion
		Vector m_v;
		//real part of the quaternion
		float m_w;
	};
	
	//--------------------------------------------------------------------
	// Global functions related to this class
	//--------------------------------------------------------------------
	
	// Allows writing Inverse(q)
	// Throws an exception if the Norm of a quaternion is 0.
	const Quaternion Inverse(const Quaternion& q);
	
	// Allows writing Conjugate(q)
	const Quaternion Conjugate(const Quaternion& q);
	
	// Allows writing Norm(q)
	float Norm(const Quaternion& q);

	const std::string ToString(const Quaternion& q);
}

#endif
