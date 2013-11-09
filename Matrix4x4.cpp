#include "dodoMath/PCHeaders.h"

#include "dodoBase/UtilFnc.h"
#include "dodoBase/Assert.h"
#include "dodoBase/Const.h"
#include "dodoBase/String.h"
#include "dodoMath/Vector.h"
#include "dodoMath/Quaternion.h"
#include "dodoMath/Matrix4x4.h"

namespace dodoEng
{
	//--------------------------------------------------------------------
	// Default constructor
	// Creates an Identity Matrix
	Matrix4x4::Matrix4x4()
	{
	}
	
	//--------------------------------------------------------------------
	// Constructor
	// Creates a uniform scaling matrix
	// s 0 0 0
	// 0 s 0 0
	// 0 0 s 0
	// 0 0 0 1
	Matrix4x4::Matrix4x4(float s)
	{
		LoadScale(s, s, s);
	}
	
	//--------------------------------------------------------------------
	// Constructor
	// Creates a matrix with specified parameters.
	Matrix4x4::Matrix4x4(float a0, float a1, float a2, float a3,
		float a4, float a5, float a6, float a7,
		float a8, float a9, float a10, float a11,
		float a12, float a13, float a14, float a15)
	{
		m_data[0] = a0; m_data[1] = a1; m_data[2] = a2; m_data[3] = a3;
		m_data[4] = a4; m_data[5] = a5; m_data[6] = a6; m_data[7] = a7;
		m_data[8] = a8; m_data[9] = a9; m_data[10] =a10;m_data[11] =a11;
		m_data[12] =a12;m_data[13] =a13;m_data[14] =a14;m_data[15] =a15;
	}
	
	//--------------------------------------------------------------------
	// Destructor
	Matrix4x4::~Matrix4x4()
	{
	}
	
	//--------------------------------------------------------------------
	// Check whether the transform is reflective.  Transform is reflective
	// if the determinant of the upper 3x3 matrix is negative.
	//
	// If a reflective transform is used to transform polygon vertices, 
	// vertex ordering will change.  It will change from clockwise to
	// counter-clockwise, or vice versa.
	bool Matrix4x4::IsReflective() const
	{
		float m33[9];
		Get3x3MatrixAroundPivot(3, 3, m33);
		
		if (Det3x3(m33) < 0.0f)
			return true;
		else
			return false;
	}
	
	//--------------------------------------------------------------------
	// Returns the element in specific column and row.
	//
	// Throws an exception of type DMGraphException if parameters passed
	// in are not valid.
	const float& Matrix4x4::operator() (int row, int column) const
	{
		DE_ASSERT(IsWithinRange(row, 0, 3) && IsWithinRange(column, 0, 3));
		return m_data[row*4 + column];
	}
	
	float& Matrix4x4::operator() (int row, int column)
	{
		DE_ASSERT(IsWithinRange(row, 0, 3) && IsWithinRange(column, 0, 3));
		return m_data[row*4 + column];
	}
	
	//--------------------------------------------------------------------
	// Creates an Identity Matrix
	// 1 0 0 0
	// 0 1 0 0
	// 0 0 1 0
	// 0 0 0 1
	void Matrix4x4::LoadIdentity()
	{
		m_data[0] = 1.0f; m_data[1] = 0.0f; m_data[2] = 0.0f;  m_data[3] = 0.0f;
		m_data[4] = 0.0f; m_data[5] = 1.0f; m_data[6] = 0.0f;  m_data[7] = 0.0f;
		m_data[8] = 0.0f; m_data[9] = 0.0f; m_data[10] = 1.0f; m_data[11] = 0.0f;
		m_data[12] =0.0f; m_data[13] =0.0f; m_data[14] = 0.0f; m_data[15] = 1.0f;
	}
	
	//--------------------------------------------------------------------
	// Scales along all axis according to sx, sy, and sz.
	// sx 0  0  0
	// 0  sy 0  0
	// 0  0  sz 0
	// 0  0  0  1
	void Matrix4x4::LoadScale(float sx, float sy, float sz)
	{
		LoadIdentity();
		m_data[0] = sx;
		m_data[5] = sy;
		m_data[10] = sz;
	}
	
	//--------------------------------------------------------------------
	// Rotates rx radians around the x-axis.
	// 1  0       0        0
	// 0  cos(rx) -sin(rx) 0
	// 0  sin(rx) cos(rx)  0
	// 0  0       0        1
	void Matrix4x4::LoadRotateX(float rx)
	{
		LoadIdentity();
		m_data[5] = cosf(rx);
		m_data[10] = m_data[5];
		m_data[9] = sinf(rx);
		m_data[6] = -m_data[9];
	}
	
	//--------------------------------------------------------------------
	// Rotates ry radians around the y-axis.
	// cos(ry)  0  sin(ry) 0
	// 0        1  0       0
	// -sin(ry) 0  cos(ry) 0
	// 0        0  0       1
	void Matrix4x4::LoadRotateY(float ry)
	{
		LoadIdentity();
		m_data[0] = cosf(ry);
		m_data[10] = m_data[0];
		m_data[2] = sinf(ry);
		m_data[8] = -m_data[2];
	}
	
	//--------------------------------------------------------------------
	// Rotates rz radians around the z-axis.
	// cos(rz) -sin(rz) 0  0
	// sin(rz) cos(rz)  0  0
	// 0       0        1  0
	// 0       0        0  1
	void Matrix4x4::LoadRotateZ(float rz)
	{
		LoadIdentity();
		m_data[0] = cosf(rz);
		m_data[5] = m_data[0];
		m_data[4] = sinf(rz);
		m_data[1] = -m_data[4];
	}
	
	//--------------------------------------------------------------------
	// Orientation matrix given by the Euler angles head, pitch, and roll.
	// Algorithm from "Real-Time Rendering" by Moller and haines pg.38
	void Matrix4x4::LoadEuler(float head, float pitch, float roll)
	{
		const float ch = cosf(head);
		const float cp = cosf(pitch);
		const float cr = cosf(roll);
		const float sh = sinf(head);
		const float sp = sinf(pitch);
		const float sr = sinf(roll);
		
		LoadIdentity();
		m_data[0] = cr*ch - sr*sp*sh;
		m_data[1] = -sr*cp;
		m_data[2] = cr*sh + sr*sp*ch;
		m_data[4] = sr*ch + cr*sp*sh;
		m_data[5] = cr*cp;
		m_data[6] = sr*sh - cr*sp*ch;
		m_data[8] = -cp*sh;
		m_data[9] = sp;
		m_data[10] = cp*ch;
	}
	
	//--------------------------------------------------------------------
	// Create a rotation matrix from a quaternion.
	// Useful for specifying rotations around specified axis.
	// Algorithm from "Real-Time Rendering" by Moller and haines pg.46
	void Matrix4x4::LoadQuaternion(const Quaternion& q)
	{
		const Vector& v = q.GetV();
		const float& x = v[0];	const float& y = v[1];
		const float& z = v[2];	const float& w = q.GetW();
		const float xx = x*x;	const float yy = y*y;		const float zz = z*z;
		const float xy = x*y;	const float xz = x*z;
		const float wz = w*z;	const float wy = w*y;
		const float yz = y*z;	const float wx = w*x;
		
		LoadIdentity();
		m_data[0] = 1.0f - 2.0f*(yy+zz);
		m_data[1] = 2.0f*(xy - wz);
		m_data[2] = 2.0f*(xz + wy);
		m_data[4] = 2.0f*(xy + wz);
		m_data[5] = 1.0f - 2.0f*(xx + zz);
		m_data[6] = 2.0f*(yz - wx);
		m_data[8] = 2.0f*(xz - wy);
		m_data[9] = 2.0f*(yz + wx);
		m_data[10] = 1.0f - 2.0f*(xx + yy);
	}
	
	//--------------------------------------------------------------------
	// Creates the transform that rotates vector source into vector dest.
	// Algorithm from "Real-Time Rendering" by Moller and haines pg.52
	void Matrix4x4::LoadRotateVIntoV(const Vector& source, const Vector& dest)
	{
		const float PARALLEL_EPSILON = EPSILON * 10.0f;
		
		const Vector s = Unit(source);
		const Vector t = Unit(dest);
		
		const float e = s * t;
		
		// if s and t are near parallel, return either the identity matrix
		// or rotation of PI around any axis
		if ( (fabs(e) - 1.0f) <= PARALLEL_EPSILON )
		{
			if (e < 0.0f)
				LoadRotateX(PI);
			else
				LoadIdentity();
		}
		else
		{
			Vector v = s ^ t;
			const float& vx = v[0];
			const float& vy = v[1];
			const float& vz = v[2];
			const float h = (1.0f - e)/(v*v);
			
			m_data[0] = e + h*vx*vx;
			m_data[1] = h*vx*vy - vz;
			m_data[2] = h*vx*vz + vy;
			m_data[4] = h*vx*vy + vz;
			m_data[5] = e + h*vy*vy;
			m_data[6] = h*vy*vz - vx;
			m_data[8] = h*vx*vz - vy;
			m_data[9] = h*vy*vz + vx;
			m_data[10] = e + h*vz*vz;
		}
	}
	
	//--------------------------------------------------------------------
	// Shears component i by a factor s, with respect to component j.
	// i, j can be X_AXIS, Y_AXIS, or Z_AXIS.
	//
	//  For example:
	//	Shear (AXIS_X, AXIS_Z, s)
	//
	//	^ z						^ z
	//	|						|
	//	|------+				| s +--------+
	//	|	   |		==>		|  /		/
	//	|      |				| /		   /
	//	|	   |  x				|/		  /  x
	//	+--------->				+------------>
	void Matrix4x4::LoadShear(Axis i, Axis j, float s)
	{
		LoadIdentity();
		m_data[i*4 + j] = s;
	}
	
	//--------------------------------------------------------------------
	// Creates a transform that moves a point by the vector (tx, ty, tz)
	// 1 0 0 tx
	// 0 1 0 ty
	// 0 0 1 tz
	// 0 0 0 1
	void Matrix4x4::LoadTranslate(float tx, float ty, float tz)
	{
		LoadIdentity();
		m_data[3] = tx;
		m_data[7] = ty;
		m_data[11] = tz;
	}
	
	//--------------------------------------------------------------------
	// Creates a transform that moves a point by the vector v
	// 1 0 0 vx
	// 0 1 0 vy
	// 0 0 1 vz
	// 0 0 0 1
	void Matrix4x4::LoadTranslate(const Vector& v)
	{
		LoadTranslate(v[0], v[1], v[2]);
	}
	
	//--------------------------------------------------------------------
	// Create a transform that changes coordinate system from left-handed
	// to right-handed and vice-versa.
	//
	// 1 0  0 0
	// 0 1  0 0
	// 0 0 -1 0
	// 0 0  0 1
	//
	void Matrix4x4::LoadChangeCoordinateSystemHandednes()
	{
		LoadIdentity();
		
		m_data[10] = -1.0f;
	}
	
	//--------------------------------------------------------------------
	// Create Perspective projection matrix.
	// Algorithm from "Real-Time Rendering" by Moller and haines pg.63
	//
	// n - near plane		             |
	// f - far plane		     L   	 |
	// l - left plane		     |		 |
	// r - right plane		+   N|	    F|
	// b - bottom plane		     |		 |
	// t - top plane		     R  	 |
	//						             |
	//
	// Throws an exception if parameters are not sensible, i.e.
	// n>=f, l>=r, or b>=t			
	void Matrix4x4::LoadPerspectiveProjection(float n, float f,
		float l, float r,
		float b, float t,
		bool rightHanded)
	{
		DE_ASSERT( (n<f) || (l<r) || (b<t) );
		
		LoadIdentity();
		m_data[0] = 2.0f*n/(r-l);
		m_data[2] = -(r+l)/(r-l);
		m_data[5] = 2.0f*n/(t-b);
		m_data[6] = -(t+b)/(t-b);
		m_data[10] = (f+n)/(f-n);
		m_data[11] = -2.0f*f*n/(f-n);
		m_data[14] = 1.0f;
		m_data[15] = 0.0f;
		
		if (!rightHanded)
		{
			Matrix4x4 sHand;
			sHand.LoadScale(1.0f, 1.0f, -1.0f);
			*this = *this * sHand;
		}
	}
	
	//--------------------------------------------------------------------
	// Create Orthographic (parallel) projection matrix.
	// Algorithm from "Real-Time Rendering" by Moller and haines pg.59
	//
	// n - near plane		 L
	// f - far plane		 +-------+
	// l - left plane		 |		 |
	// r - right plane		N|		F|
	// b - bottom plane		 |		 |
	// t - top plane		 +-------+
	//						 R
	//
	// Throws an exception if parameters are not sensible, i.e.
	// n>=f, l>=r, or b>=t			
	void Matrix4x4::LoadOrthographicProjection(float n, float f,
		float l, float r,
		float b, float t,
		bool rightHanded)
	{
		DE_ASSERT( (n<f) || (l<r) || (b<t) );
		
		Matrix4x4 S, T;
		S.LoadScale( 2.0f/(r-l), 2.0f/(t-b), 2.0f/(f-n) );
		T.LoadTranslate( -0.5f*(l+r), -0.5f*(t+b), -0.5f*(f+n) );
		
		if (rightHanded)
		{
			*this = S * T;
		}
		else
		{
			Matrix4x4 sHand;
			sHand.LoadScale(1.0f, 1.0f, -1.0f);
			*this = S * (T * sHand);
		}
	}
	
	//--------------------------------------------------------------------
	// Set the martix elements to new values.
	void Matrix4x4::Load(float a0, float a1, float a2, float a3,
		float a4, float a5, float a6, float a7,
		float a8, float a9, float a10, float a11,
		float a12, float a13, float a14, float a15)
	{
		m_data[0] = a0; m_data[1] = a1; m_data[2] = a2; m_data[3] = a3;
		m_data[4] = a4; m_data[5] = a5; m_data[6] = a6; m_data[7] = a7;
		m_data[8] = a8; m_data[9] = a9; m_data[10] =a10;m_data[11] =a11;
		m_data[12] =a12;m_data[13] =a13;m_data[14] =a14;m_data[15] =a15;
	}
	
	//--------------------------------------------------------------------
	// Set the matrix to the Bezier Basis
	//
	// -1  3 -3  1
	//  3 -6  3  0
	// -3  3  0  0
	//  1  0  0  0
	void Matrix4x4::LoadBezierBasis()
	{
		m_data[0] = -1.0f; m_data[1] = 3.0f;  m_data[2] = -3.0f; m_data[3] = 1.0f;
		m_data[4] = 3.0f;  m_data[5] = -6.0f; m_data[6] = 3.0f;  m_data[7] = 0.0f;
		m_data[8] = -3.0f; m_data[9] = 3.0f;  m_data[10] = 0.0f; m_data[11] = 0.0f;
		m_data[12] =1.0f;  m_data[13] =0.0f;  m_data[14] =0.0f;  m_data[15] =0.0f;
	}
	
	//--------------------------------------------------------------------
	// Set the matrix to the B-Spline basis
	//
	//        -1  3 -3  1
	//  1/6*   3 -6  3  0
	//        -3  0  3  0
	//         1  4  1  0
	void Matrix4x4::LoadBSplineBasis()
	{
		const float s = 1.0f/6.0f;
		m_data[0] = -1.0f*s; m_data[1] = 3.0f*s;  m_data[2] = -3.0f*s; m_data[3] = 1.0f*s;
		m_data[4] = 3.0f*s;  m_data[5] = -6.0f*s; m_data[6] = 3.0f*s;  m_data[7] = 0.0f;
		m_data[8] = -3.0f*s; m_data[9] = 0.0f;	  m_data[10] = 3.0f*s; m_data[11] = 0.0f;
		m_data[12] =1.0f*s;  m_data[13] =4.0f*s;  m_data[14] =1.0f*s;  m_data[15] =0.0f;
	}
	//--------------------------------------------------------------------
	// Set the matrix to the Catmull-Rom basis
	//
	//      -1  3 -3  1
	// .5*   2 -5  4 -1
	//      -1  0  1  0
	//       0  2  0  0
	void Matrix4x4::LoadCatmullRomBasis()
	{
		const float s = 0.5f;
		m_data[0] = -1.0f*s; m_data[1] = 3.0f*s;  m_data[2] = -3.0f*s; m_data[3] = 1.0f*s;
		m_data[4] = 2.0f*s;  m_data[5] = -5.0f*s; m_data[6] = 4.0f*s;  m_data[7] = -1.0f*s;
		m_data[8] = -1.0f*s; m_data[9] = 0.0f;    m_data[10] = 1.0f*s; m_data[11] = 0.0f;
		m_data[12] =0.0f;    m_data[13] =2.0f*s;  m_data[14] =0.0f;    m_data[15] =0.0f;
	}
	
	//--------------------------------------------------------------------
	// Calculate the determinant of the matrix.
	float Matrix4x4::Determinant() const
	{
		float m00[9];
		Get3x3MatrixAroundPivot(0, 0, m00);
		
		float m01[9];
		Get3x3MatrixAroundPivot(0, 1, m01);
		
		float m02[9];
		Get3x3MatrixAroundPivot(0, 2, m02);
		
		float m03[9];
		Get3x3MatrixAroundPivot(0, 3, m03);
		
		return ( (*this)(0,0) * Det3x3(m00) -
			(*this)(0,1) * Det3x3(m01) +
			(*this)(0,2) * Det3x3(m02) -
			(*this)(0,3) * Det3x3(m03) );
	}
	
	//--------------------------------------------------------------------
	// Set the matrix to its transpose.
	void Matrix4x4::Transpose()
	{
		std::swap(m_data[1], m_data[4]);
		std::swap(m_data[2], m_data[8]);
		std::swap(m_data[3], m_data[12]);
		std::swap(m_data[6], m_data[9]);
		std::swap(m_data[7], m_data[13]);
		std::swap(m_data[11], m_data[14]);
	}
	
	//--------------------------------------------------------------------
	// Set the matrix to its inverse.  Optimizations are implemented for
	// several different transform types.  The following types are
	// optimized:
	//		TRANSFORMTYPE_IDENTITY
	//		TRANSFORMTYPE_ORTHOGONAL
	//		TRANSFORMTYPE_RIGID_BODY
	//		TRANSFORMTYPE_ANGLE_PRESERVING
	//
	// All other transforms are inverted using Gaussian Elimination.
	//
	// Throws an exception if the inverse does not exist.
	void Matrix4x4::InvertOrthogonal()
	{
		this->Transpose();
	}

	void Matrix4x4::InvertRigidBody()
	{
		// to find the rigid body transform's inverse we use optimized
		// algorithm for angle-preserving inverses.
		*this = this->AnglePreservingInverse();
	}
	
	void Matrix4x4::InvertAnglePreserving()
	{
		//Check if the transform is invertible
		DE_ASSERT(fabs(dodoEng::Determinant(*this)) > EPSILON);
		*this = this->AnglePreservingInverse();
	}

	void Matrix4x4::Invert()
	{
		// For all other transforms use Gaussian Elimination

		//Check if the transform is invertible
		DE_ASSERT(fabs(dodoEng::Determinant(*this)) > EPSILON);
		
		// use Gaussian Elimination to find the inverse
		*this = this->GaussianEliminationInverse();
	}
	
	//--------------------------------------------------------------------
	// Calculate the trace of the matrix.
	// Trace: Sum of all diagonal elements
	float Matrix4x4::Trace() const
	{
		return (m_data[0] + m_data[5] + m_data[10] + m_data[15]);
	}
	
	//--------------------------------------------------------------------
	// Calculate and return the transform to be used for transforming
	// normal vectors.  This transform is equal to Transpose(Inverse(M)),
	// but some optimizations can be used in special cases.
	//
	// It is improtaint to check the type of the original transform when
	// applying normal transforms.  Transform type
	// TRANSFORMTYPE_ANGLE_PRESERVING needs special handling.  Normals
	// transformed by such transform need to be renormalized.
	//
	//
	// For example:
	// m - Matrix4x4 of type TRANSFORMTYPE_ANGLE_PRESERVING
	// n - normal Vector
	//
	const Matrix4x4 Matrix4x4::GetNormalTransformOrthogonal() const
	{
		return *this;
	}

	const Matrix4x4 Matrix4x4::GetNormalTransformRigidBody() const
	{
		//TODO
		DE_ASSERT(false);
		return *this;
	}
			
	const Matrix4x4 Matrix4x4::GetNormalTransformAnglePreserving() const
	{
		//TODO
		DE_ASSERT(false);
		return *this;
	}
			
	const Matrix4x4 Matrix4x4::GetNormalTransform() const
	{
		return dodoEng::Transpose(dodoEng::Inverse(*this));
	}
	
	//--------------------------------------------------------------------
	// Calculate the determinant of the matrix.
	// Friend function that allows writing Determinant(m).
	float Determinant(const Matrix4x4& m)
	{
		return m.Determinant();
	}
	
	//--------------------------------------------------------------------
	// Calculate the transpose of the matrix.
	// Friend function that allows writing Transpose(m).
	const Matrix4x4 Transpose(const Matrix4x4& m)
	{
		Matrix4x4 r(m);
		r.Transpose();
		return r;
	}
	
	//--------------------------------------------------------------------
	// Calculate the inverse of the matrix
	// Friend function that allows writing Inverse(m).
	const Matrix4x4 Inverse(const Matrix4x4& m)
	{
		Matrix4x4 res = m;
		res.Invert();
		return res;
	}
	
	//--------------------------------------------------------------------
	// Calculate the trace of the matrix
	// Trace: Sum of all diagonal elements
	// Friend function that allows writing Trace(m).
	float Trace(const Matrix4x4& m)
	{
		return m.Trace();
	}
	
	//--------------------------------------------------------------------
	// Calculate and return the transform to be used for transforming
	// normal vectors.  This transform is equal to Transpose(Inverse(M)),
	// but some optimizations can be used in special cases.
	// Friend function that allows writing NormalTransform(m).
	const Matrix4x4 NormalTransform(const Matrix4x4& m)
	{
		return m.GetNormalTransform();
	}
	
	//--------------------------------------------------------------------
	// Addition of two matrices.  Result is returned.
	const Matrix4x4 Matrix4x4::operator+(const Matrix4x4& rhs) const
	{
		return Matrix4x4(
			m_data[0] + rhs.m_data[0], m_data[1] + rhs.m_data[1],
			m_data[2] + rhs.m_data[2], m_data[3] + rhs.m_data[3],
			m_data[4] + rhs.m_data[4], m_data[5] + rhs.m_data[5],
			m_data[6] + rhs.m_data[6], m_data[7] + rhs.m_data[7],
			m_data[8] + rhs.m_data[8], m_data[9] + rhs.m_data[9],
			m_data[10] + rhs.m_data[10], m_data[11] + rhs.m_data[11],
			m_data[12] + rhs.m_data[12], m_data[13] + rhs.m_data[13],
			m_data[14] + rhs.m_data[14], m_data[15] + rhs.m_data[15]
			);
	}
	
	//--------------------------------------------------------------------
	// Addition of two matrices.  Result is stored in left handed matrix.
	// More efficient then the operator+.
	void Matrix4x4::operator+=(const Matrix4x4& rhs)
	{
		m_data[0] += rhs.m_data[0]; m_data[1] += rhs.m_data[1];
		m_data[2] += rhs.m_data[2]; m_data[3] += rhs.m_data[3];
		m_data[4] += rhs.m_data[4]; m_data[5] += rhs.m_data[5];
		m_data[6] += rhs.m_data[6]; m_data[7] += rhs.m_data[7];
		m_data[8] += rhs.m_data[8]; m_data[9] += rhs.m_data[9];
		m_data[10] += rhs.m_data[10]; m_data[11] += rhs.m_data[11];
		m_data[12] += rhs.m_data[12]; m_data[13] += rhs.m_data[13];
		m_data[14] += rhs.m_data[14]; m_data[15] += rhs.m_data[15];
	}
	
	//--------------------------------------------------------------------
	// Subtracts a matrix from another one.  Result is returned
	const Matrix4x4 Matrix4x4::operator-(const Matrix4x4& rhs) const
	{
		return Matrix4x4(
			m_data[0] - rhs.m_data[0], m_data[1] - rhs.m_data[1],
			m_data[2] - rhs.m_data[2], m_data[3] - rhs.m_data[3],
			m_data[4] - rhs.m_data[4], m_data[5] - rhs.m_data[5],
			m_data[6] - rhs.m_data[6], m_data[7] - rhs.m_data[7],
			m_data[8] - rhs.m_data[8], m_data[9] - rhs.m_data[9],
			m_data[10] - rhs.m_data[10], m_data[11] - rhs.m_data[11],
			m_data[12] - rhs.m_data[12], m_data[13] - rhs.m_data[13],
			m_data[14] - rhs.m_data[14], m_data[15] - rhs.m_data[15]
			);
	}
	
	//--------------------------------------------------------------------
	// Subtracts a matrix on the right from the left one.  Result is
	// stored in the left matrix.
	// More efficient then the operator-.
	void Matrix4x4::operator-=(const Matrix4x4& rhs)
	{
		m_data[0] -= rhs.m_data[0]; m_data[1] -= rhs.m_data[1];
		m_data[2] -= rhs.m_data[2]; m_data[3] -= rhs.m_data[3];
		m_data[4] -= rhs.m_data[4]; m_data[5] -= rhs.m_data[5];
		m_data[6] -= rhs.m_data[6]; m_data[7] -= rhs.m_data[7];
		m_data[8] -= rhs.m_data[8]; m_data[9] -= rhs.m_data[9];
		m_data[10] -= rhs.m_data[10]; m_data[11] -= rhs.m_data[11];
		m_data[12] -= rhs.m_data[12]; m_data[13] -= rhs.m_data[13];
		m_data[14] -= rhs.m_data[14]; m_data[15] -= rhs.m_data[15];
	}
	
	//--------------------------------------------------------------------
	// Multiply two matrices.  Returns the resulting matrix.
	const Matrix4x4 Matrix4x4::operator*(const Matrix4x4& rhs) const
	{
		Matrix4x4 out(
			m_data[0]*rhs.m_data[0] + m_data[1]*rhs.m_data[4] +
			m_data[2]*rhs.m_data[8] + m_data[3]*rhs.m_data[12],
			
			m_data[0]*rhs.m_data[1] + m_data[1]*rhs.m_data[5] +
			m_data[2]*rhs.m_data[9] + m_data[3]*rhs.m_data[13],
			
			m_data[0]*rhs.m_data[2] + m_data[1]*rhs.m_data[6] +
			m_data[2]*rhs.m_data[10] + m_data[3]*rhs.m_data[14],
			
			m_data[0]*rhs.m_data[3] + m_data[1]*rhs.m_data[7] +
			m_data[2]*rhs.m_data[11] + m_data[3]*rhs.m_data[15],
			
			m_data[4]*rhs.m_data[0] + m_data[5]*rhs.m_data[4] +
			m_data[6]*rhs.m_data[8] + m_data[7]*rhs.m_data[12],
			
			m_data[4]*rhs.m_data[1] + m_data[5]*rhs.m_data[5] +
			m_data[6]*rhs.m_data[9] + m_data[7]*rhs.m_data[13],
			
			m_data[4]*rhs.m_data[2] + m_data[5]*rhs.m_data[6] +
			m_data[6]*rhs.m_data[10] + m_data[7]*rhs.m_data[14],
			
			m_data[4]*rhs.m_data[3] + m_data[5]*rhs.m_data[7] +
			m_data[6]*rhs.m_data[11] + m_data[7]*rhs.m_data[15],
			
			m_data[8]*rhs.m_data[0] + m_data[9]*rhs.m_data[4] +
			m_data[10]*rhs.m_data[8] + m_data[11]*rhs.m_data[12],
			
			m_data[8]*rhs.m_data[1] + m_data[9]*rhs.m_data[5] +
			m_data[10]*rhs.m_data[9] + m_data[11]*rhs.m_data[13],
			
			m_data[8]*rhs.m_data[2] + m_data[9]*rhs.m_data[6] +
			m_data[10]*rhs.m_data[10] + m_data[11]*rhs.m_data[14],
			
			m_data[8]*rhs.m_data[3] + m_data[9]*rhs.m_data[7] +
			m_data[10]*rhs.m_data[11] + m_data[11]*rhs.m_data[15],
			
			m_data[12]*rhs.m_data[0] + m_data[13]*rhs.m_data[4] +
			m_data[14]*rhs.m_data[8] + m_data[15]*rhs.m_data[12],
			
			m_data[12]*rhs.m_data[1] + m_data[13]*rhs.m_data[5] +
			m_data[14]*rhs.m_data[9] + m_data[15]*rhs.m_data[13],
			
			m_data[12]*rhs.m_data[2] + m_data[13]*rhs.m_data[6] +
			m_data[14]*rhs.m_data[10] + m_data[15]*rhs.m_data[14],
			
			m_data[12]*rhs.m_data[3] + m_data[13]*rhs.m_data[7] +
			m_data[14]*rhs.m_data[11] + m_data[15]*rhs.m_data[15]
			);
		
		return out;
	}
	
	//--------------------------------------------------------------------
	// Tests whether all elements of two matrices are the same.  Returns
	// true if they are, and false otherwise.
	bool Matrix4x4::operator==(const Matrix4x4& rhs)
	{
		for (int i=0; i<16; ++i)
		{
			if (fabs(m_data[i] - rhs.m_data[i]) > EPSILON)
				return false;
		}
		return true;
	}
	
	//--------------------------------------------------------------------
	// Tests whether all elements of two matrices are the same.  Returns
	// false if the are, adn true otherwise.
	bool Matrix4x4::operator!=(const Matrix4x4& rhs)
	{
		return !(*this == rhs);
	}
	
	//--------------------------------------------------------------------
	// Calculate the determinant of the 3x3 matrix.  This functions is
	// used by Matrix4x4::Determinant() to find the determinant of the 4x4
	// matrix.
	//
	//	  [a b c]
	// det[d e f] = aei - afh - bdi + cdh + bfg - ceg
	//    [g h i]
	//
	float Matrix4x4::Det3x3(float m[]) const
	{
		return ( m[0]*m[4]*m[8] -
			m[0]*m[5]*m[7] -
			m[1]*m[3]*m[8] +
			m[2]*m[3]*m[7] +
			m[1]*m[5]*m[6] -
			m[2]*m[4]*m[6] );
	}
	
	//--------------------------------------------------------------------
	// Get the 3x3 matrix from a 4x4 matrix based on the pivot position.
	// This matrix is obtained by discarding pivot column and row.  This
	// function is used by Matrix4x4::Determinant() to find the
	// determinant of the 4x4 matrix.
	void Matrix4x4::Get3x3MatrixAroundPivot(int row, int column, float r[]) const
	{
		int tc = 0;
		int tr = 0;
		
		for (int i=0; i<4; ++i)
		{
			if (i != row)
			{
				for (int j=0; j<4; ++j)
				{
					if (j != column)
					{
						r[tr*3+tc] = (*this)(i, j);
						++tc;
					}
				}
				tc = 0;
				++tr;
			}
		}
	}
	
	//--------------------------------------------------------------------
	// Computes the inverse of a 3-D angle-preserving matrix.
	//
	// This procedure treats the 4 by 4 angle-preserving matrix as a block
	// matrix and calculates the inverse of one submatrix for a significant
	// performance improvement over a general procedure that can invert any
	// nonsingular matrix.
	//
	// Algorithm was adapted from "Graphics Gems IV" (Paul Heckbert, Ed.),
	// section 3.3
	Matrix4x4 Matrix4x4::AnglePreservingInverse() const
	{
		Matrix4x4 out;
		
		// Calculate the square of the isotropic scale factor
		float  scale = (*this)(0,0) * (*this)(0,0) +
			(*this)(0,1) * (*this)(0,1) +
			(*this)(0,2) * (*this)(0,2);
		
		// Calculate the inverse of the square of the isotropic scale factor
		scale = 1.0f / scale;
		
		// Transpose and scale the 3 by 3 upper-left submatrix
		out.m_data[0] = scale * (*this)(0,0);
		out.m_data[4] = scale * (*this)(0,1);
		out.m_data[8] = scale * (*this)(0,2);
		out.m_data[1] = scale * (*this)(1,0);
		out.m_data[5] = scale * (*this)(1,1);
		out.m_data[9] = scale * (*this)(1,2);
		out.m_data[2] = scale * (*this)(2,0);
		out.m_data[6] = scale * (*this)(2,1);
		out.m_data[10] = scale * (*this)(2,2);
		
		// Calculate -(transpose(A) / s*s) C
		out.m_data[3] = - ( out(0,0) * (*this)(0,3) +
			out(0,1) * (*this)(1,3) +
			out(0,2) * (*this)(2,3) );
		out.m_data[7] = - ( out(1,0) * (*this)(0,3) +
			out(1,1) * (*this)(1,3) +
			out(1,2) * (*this)(2,3) );
		out.m_data[11] = - (out(2,0) * (*this)(0,3) +
			out(2,1) * (*this)(1,3) +
			out(2,2) * (*this)(2,3) );
		
		// Fill in last row
		out.m_data[12] = out.m_data[13] = out.m_data[14] = 0.0f;
		out.m_data[15] = 1.0f;
		
		return out;
	}
	
	//--------------------------------------------------------------------
	// Computes the inverse of a general transform using the Gaussian
	// Elimination.
	Matrix4x4 Matrix4x4::GaussianEliminationInverse() const
	{
		//create an identity matrix
		Matrix4x4 res;
		Matrix4x4 in = *this;
		
		// first column
		int pivot = in.FindPivotRow(0);
		in.SwapRows(0, pivot, res);
		in.EliminateColumn(0, res);
		
		
		// second column
		pivot = in.FindPivotRow(1);
		in.SwapRows(1, pivot, res);
		in.EliminateColumn(1, res);
		
		
		//third column
		pivot = in.FindPivotRow(2);
		in.SwapRows(2, pivot, res);
		in.EliminateColumn(2, res);
		
		
		//fourth column
		pivot = in.FindPivotRow(3);
		in.EliminateColumn(3, res);
		
		//divide rows by its pivot elements
		in.NormalizePivots(res);
		
		return res;
	}
	
	
	//--------------------------------------------------------------------
	// Returns the row index of the row that has the largest value in the
	// specified column.
	// Used by Gaussian-Elimination inverse algorithm.
	int Matrix4x4::FindPivotRow(int col) const
	{
		DE_ASSERT((col>=0) && (col<=3));
		
		const float r0 = fabsf(m_data[col]);
		const float r1 = fabsf(m_data[col+4]);
		const float r2 = fabsf(m_data[col+8]);
		const float r3 = fabsf(m_data[col+12]);
		
		if ( (r0 >= r1) && (r0 >= r2) && (r0 >= r3) && (col <= 0) )
			return 0;
		else if ( (r1 >= r2) && (r1 >= r3) && (col <= 1) )
			return 1;
		else if ( (r2 >= r3) && (col <= 2) )
			return 2;
		else
			return 3;
	}
	
	//--------------------------------------------------------------------
	// Swap rows row1, and row2 in both the *this matrix, and the res
	// matrix.
	// Used by Gaussian-Elimination inverse algorithm.
	void Matrix4x4::SwapRows(int row1, int row2, Matrix4x4& res)
	{
		DE_ASSERT((row1>=0) && (row1<=3) && (row2>=0) && (row2<=3));
		
		int r1i = row1*4;
		int r2i = row2*4;
		
		std::swap(m_data[r1i], m_data[r2i]);
		std::swap(m_data[r1i+1], m_data[r2i+1]);
		std::swap(m_data[r1i+2], m_data[r2i+2]);
		std::swap(m_data[r1i+3], m_data[r2i+3]);
		
		std::swap(res.m_data[r1i], res.m_data[r2i]);
		std::swap(res.m_data[r1i+1], res.m_data[r2i+1]);
		std::swap(res.m_data[r1i+2], res.m_data[r2i+2]);
		std::swap(res.m_data[r1i+3], res.m_data[r2i+3]);
	}
	
	//--------------------------------------------------------------------
	// Subtract a multiple of the pivot element from all other elements in
	// a column, untill only the pivot is non-zero.  All the operations
	// are also perfomed on the res matrix.
	// Used by Gaussian-Elimination inverse algorithm.
	void Matrix4x4::EliminateColumn(int col, Matrix4x4& res)
	{
		DE_ASSERT((col>=0) && (col<=3));
		
		for (int i=0; i<4; ++i)
		{
			if ( (i != col) && ( fabs(m_data[i*4 + col]) >= EPSILON) )
			{
				this->MultiplyRow( i,
					m_data[col*4 + col]/m_data[i*4 + col],
					res );
				this->SubtractRow(i, col, res);
			}
		}
	}
	
	//--------------------------------------------------------------------
	// Multiply all elements in the specified rows of both the *this and 
	// the res matrices by value m.
	// Used by Gaussian-Elimination inverse algorithm.
	void Matrix4x4::MultiplyRow(int row, float m, Matrix4x4& res)
	{
		DE_ASSERT((row>=0) && (row<=3));
		
		int ri = row*4;
		
		m_data[ri] *= m;
		m_data[ri+1] *= m;
		m_data[ri+2] *= m;
		m_data[ri+3] *= m;
		
		res.m_data[ri] *= m;
		res.m_data[ri+1] *= m;
		res.m_data[ri+2] *= m;
		res.m_data[ri+3] *= m;
	}
	
	//--------------------------------------------------------------------
	// Subtract row rb from row ra in both the *this and the res matrices.
	// Used by Gaussian-Elimination inverse algorithm.
	void Matrix4x4::SubtractRow(int ra, int rb, Matrix4x4& res)
	{
		DE_ASSERT((ra>=0) && (ra<=3));
		DE_ASSERT((rb>=0) && (rb<=3));
		
		int rai = ra*4;
		int rbi = rb*4;
		
		m_data[rai] -= m_data[rbi];
		m_data[rai+1] -= m_data[rbi+1];
		m_data[rai+2] -= m_data[rbi+2];
		m_data[rai+3] -= m_data[rbi+3];
		
		res.m_data[rai] -= res.m_data[rbi];
		res.m_data[rai+1] -= res.m_data[rbi+1];
		res.m_data[rai+2] -= res.m_data[rbi+2];
		res.m_data[rai+3] -= res.m_data[rbi+3];
	}
	
	//--------------------------------------------------------------------
	// Divide all rows of the res matrix by the diagonal element in the
	// row.  This is the final step in the Gaussian-Elimination inverse
	// algorithm.
	void Matrix4x4::NormalizePivots(Matrix4x4& res)
	{
		for (int i=0; i<4; ++i)
		{
			if (m_data[i*4 + i] != 1.0f)
				res.DivideRow(i, m_data[i*4 + i]);
		}
	}
	
	//--------------------------------------------------------------------
	// Divide all elements in the specified row by the specified divisor.
	// Used by Gaussian-Elimination inverse algorithm.
	void Matrix4x4::DivideRow(int row, float divisor)
	{
		DE_ASSERT((row>=0) && (row<=3));
		DE_ASSERT(divisor != 0.0f);
		
		const float m = 1.0f/divisor;
		const int ri = row*4;
		
		m_data[ri] *= m;
		m_data[ri+1] *= m;
		m_data[ri+2] *= m;
		m_data[ri+3] *= m;
	}

	//--------------------------------------------------------------------
	//Text representation of the object
	const std::string ToString(const Matrix4x4& m)
	{
		return
			std::string("[") + dodoEng::ToString(m(0,0)) + std::string(", ") + dodoEng::ToString(m(0, 1)) + std::string(", ") + dodoEng::ToString(m(0, 2)) + ", " + dodoEng::ToString(m(0, 3)) + std::string("]\n") + 
			std::string("[") + dodoEng::ToString(m(1,0)) + std::string(", ") + dodoEng::ToString(m(1, 1)) + std::string(", ") + dodoEng::ToString(m(1, 2)) + ", " + dodoEng::ToString(m(1, 3)) + std::string("]\n") + 
			std::string("[") + dodoEng::ToString(m(2,0)) + std::string(", ") + dodoEng::ToString(m(2, 1)) + std::string(", ") + dodoEng::ToString(m(2, 2)) + ", " + dodoEng::ToString(m(2, 3)) + std::string("]\n") +
			std::string("[") + dodoEng::ToString(m(3,0)) + std::string(", ") + dodoEng::ToString(m(3, 1)) + std::string(", ") + dodoEng::ToString(m(3, 2)) + ", " + dodoEng::ToString(m(3, 3)) + std::string("]\n");
	}

	//--------------------------------------------------------------------
}