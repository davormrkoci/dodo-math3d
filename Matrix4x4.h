// Matrix4x4.h: interface for the Matrix4x4 class.
// Author: Davor Mrkoci
//
// This class represents a 4x4 matrix composed of floats.  It
// provides operators that make it act as a built in type.  The
// multiplication operator is used for  matrix multiplication,
// as well as applying a transform to a Point or a Vector.
// Note that Matrix4x4 also provides the functionality for
// creating different types of geometric transforms.

#if !defined(DODOENG_MATRIX4X4_H_)
#define DODOENG_MATRIX4X4_H_

#include "dodoMath/Vector.h"

namespace dodoEng
{
	class Vector;
	class Quaternion;
}

namespace dodoEng
{
	// Axis aliases (used for specifying shearing axis)
	enum Axis
	{
		X_AXIS = 0,
		Y_AXIS,
		Z_AXIS,
	};

	class Matrix4x4
	{
		//--------------------------------------------------------------------
		// Interface
		//--------------------------------------------------------------------
	public:
		// Default constructor
		// Creates an Identity Matrix
		Matrix4x4();
		
		// Constructor
		// Creates a uniform scaling matrix with scaling factor of s.
		explicit Matrix4x4(float s);
		
		// Constructor
		// Creates a matrix with specified parameters.
		Matrix4x4(float a0, float a1, float a2, float a3,
			float a4, float a5, float a6, float a7,
			float a8, float a9, float a10, float a11,
			float a12, float a13, float a14, float a15);
		
		// Destructor
		~Matrix4x4();
		
		// Check whether the transform is reflective.  Transform is
		// reflective if the determinant of the upper 3x3 matrix is
		// negative.
		bool IsReflective() const;
		
		// Returns the element in specific column and row.
		// Throws an exception of type DMGraphException if parameters
		// passed in are not valid.
		const float& operator() (int row, int column) const;
		float& operator() (int row, int column);
		
		// Creates an Identity Matrix
		void LoadIdentity();
		
		// Scales along all axis according to sx, sy, and sz parameters.
		void LoadScale(float sx, float sy, float sz);
		
		// Rotates rx radians around the x-axis.
		void LoadRotateX(float rx);
		
		// Rotates ry radians around the y-axis.
		void LoadRotateY(float ry);
		
		// Rotates rz radians around the z-axis.
		void LoadRotateZ(float rz);
		
		// Orientation matrix given by the Euler angles head, pitch,
		// and roll.
		void LoadEuler(float head, float pitch, float roll);
		
		// Create a rotation matrix from a quaternion.
		void LoadQuaternion(const Quaternion& q);
		
		// Creates the transform that rotates vector source into vector
		// dest.
		void LoadRotateVIntoV(const Vector& source, const Vector& dest);
		
		// Shears component i by a factor s, with respect to component j.
		// i, j can be X_AXIS, Y_AXIS, or Z_AXIS.
		void LoadShear(Axis i, Axis j, float s);
		
		// Creates a transform that moves a point by the vector (tx, ty, tz)
		void LoadTranslate(float tx, float ty, float tz);
		
		// Creates a transform that moves a point by the vector v
		void LoadTranslate(const Vector& v);
		
		// Create a transform that changes coordinate system from left-
		// handed to right-handed and vice-versa.
		void LoadChangeCoordinateSystemHandednes();
		
		// Create Perspective projection matrix.
		void LoadPerspectiveProjection(float n, float f,
			float l, float r,
			float b, float t,
			bool rightHanded = true);
		
		// Create Orthographic (parallel) projection matrix.
		void LoadOrthographicProjection(float n, float f,
			float l, float r,
			float b, float t,
			bool rightHanded = true);
		
		// Set the martix elements to new values.
		void Load(float a0, float a1, float a2, float a3,
			float a4, float a5, float a6, float a7,
			float a8, float a9, float a10, float a11,
			float a12, float a13, float a14, float a15);
		
		// Set the matrix to the Bezier Basis
		void LoadBezierBasis();
		
		// Set the matrix to the B-Spline basis
		void LoadBSplineBasis();
		
		// Set the matrix to the Catmull-Rom basis
		void LoadCatmullRomBasis();
		
		
		// MATH SUPPORT
		// Calculate the determinant of the matrix.
		float Determinant() const;
		
		// Set the matrix to its transpose.
		void Transpose();
		
		// Set the matrix to its inverse.
		void Invert();
		void InvertOrthogonal();
		void InvertRigidBody();
		void InvertAnglePreserving();
		
		// Returns the matrix inverse.
		// Throws an exception if the inverse does not exist.
		friend const Matrix4x4 Inverse(const Matrix4x4& m);
		
		// Calculate the trace of the matrix.
		// Trace: Sum of all diagonal elements
		float Trace() const;
		
		// Calculate and return the transform to be used for transforming
		// normal vectors.
		//
		// It is improtaint to check the type of the original transform
		// when applying normal transforms.  This can be done using the
		// GetTransformType() function.  Transform type
		// TRANSFORMTYPE_ANGLE_PRESERVING needs special handling.  Normals
		// transformed by such transform need to be renormalized.
		const Matrix4x4 GetNormalTransform() const;
		const Matrix4x4 GetNormalTransformOrthogonal() const;
		const Matrix4x4 GetNormalTransformRigidBody() const;
		const Matrix4x4 GetNormalTransformAnglePreserving() const;
		
		// Matrix & Matrix math operations
		const Matrix4x4 operator+(const Matrix4x4& rhs) const;
		void operator+=(const Matrix4x4& rhs);
		const Matrix4x4 operator-(const Matrix4x4& rhs) const;
		void operator-=(const Matrix4x4& rhs);
		const Matrix4x4 operator*(const Matrix4x4& rhs) const;
		bool operator==(const Matrix4x4& rhs);
		bool operator!=(const Matrix4x4& rhs);

		const float* GetPtr() const { return &m_data[0]; }
		float* GetPtr() { return &m_data[0]; }

		Vector ExtractTranslation() const { return Vector(m_data[3], m_data[7], m_data[11]); }
		
		//--------------------------------------------------------------------
		// Implementation
		//--------------------------------------------------------------------
	private:
		// helper functions for finding the determinant
		float Det3x3(float m[]) const;
		void Get3x3MatrixAroundPivot(int row, int column, float r[]) const;
	
		// helper functions for finding the inverse
		Matrix4x4 AnglePreservingInverse() const;
		Matrix4x4 GaussianEliminationInverse() const;
		int FindPivotRow(int col) const;
		void SwapRows(int row1, int row2, Matrix4x4& res);
		void EliminateColumn(int col, Matrix4x4& res);
		void MultiplyRow(int row, float m, Matrix4x4& res);
		void SubtractRow(int ra, int rb, Matrix4x4& res);
		void NormalizePivots(Matrix4x4& res);
		void DivideRow(int row, float divisor);
		
		//--------------------------------------------------------------------
		// Data Members
		//--------------------------------------------------------------------
	private:
		// 4x4 matrix
		float m_data[16];
	};

	//--------------------------------------------------------------------
	// Global functions related to this class
	//--------------------------------------------------------------------

	// Calculate the determinant of the matrix.
	// Friend function that allows writing Determinant(m).
	float Determinant(const Matrix4x4& m);

	// Calculate the transpose of the matrix.
	// Friend function that allows writing Transpose(m).
	const Matrix4x4 Transpose(const Matrix4x4& m);

	// Calculate the trace of the matrix.
	// Friend function that allows writing Trace(m).
	float Trace(const Matrix4x4& m);

	// Calculate the transform for transforming normals.
	// Friend function that allows writing NormalTransform(m).
	const Matrix4x4 NormalTransform(const Matrix4x4& m);

	//Text representation of the object
	const std::string ToString(const Matrix4x4& m);
}

#endif
