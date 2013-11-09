#if !defined(DODOENG_VECTOR4_H_)
#define DODOENG_VECTOR4_H_

#include "BaseQuad.h"

namespace dodoEng
{
	class Matrix4x4;
}

namespace dodoEng
{
	class Vector4 : public BaseQuad
	{
		//--------------------------------------------------------------------
		// Interface
		//--------------------------------------------------------------------
	public:
		Vector4();
		Vector4(float x, float y, float z, float h);
		explicit Vector4(const Vector& v);
		explicit Vector4(const Point& pt);
		virtual ~Vector4();
		
		void Homogenize();
		const Vector4 Homogenized() const;
		
		const Vector4 operator+(const Vector4& rhs) const;
		const Vector4 operator-(const Vector4& rhs) const;
		void operator+=(const Vector4& rhs);
		void operator-=(const Vector4& rhs);
		bool operator==(const Vector4& rhs) const;
		bool operator!=(const Vector4& rhs) const;

		const Vector ToVector() const;
		const Point ToPoint() const;
	};

	//--------------------------------------------------------------------
	// Global functions related to this class
	//--------------------------------------------------------------------
	
	const Vector4 Homogenized(const Vector4& v);
}

// Apply transform m to the vector v.  The result is a transformed
// vector.
// Since a vector is a single column matrix, it can only be post-
// multiplied with a matrix.
//
// For example, M*v is legal, while v*M is not.
// This relationship is enforced.
const dodoEng::Vector4 operator*(const dodoEng::Matrix4x4& m, const dodoEng::Vector4& v);

#endif
