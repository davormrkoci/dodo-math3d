#include "dodoBase/PCHeaders.h"

#include "dodoBase/Assert.h"
#include "dodoBase/UtilFnc.h"
#include "dodoBase/String.h"

#include "dodoBase/BaseQuad.h"

namespace dodoEng
{
	
	//--------------------------------------------------------------------
	// Default constructor
	BaseQuad::BaseQuad()
	{
		m_data[0] = 0.0f;
		m_data[1] = 0.0f;
		m_data[2] = 0.0f;
		m_data[3] = 0.0f;
	}
	
	//--------------------------------------------------------------------
	// Constructor
	BaseQuad::BaseQuad(float x, float y, float z, float h)
	{
		m_data[0] = x;
		m_data[1] = y;
		m_data[2] = z;
		m_data[3] = h;
	}
	
	//--------------------------------------------------------------------
	// Destructor
	BaseQuad::~BaseQuad()
	{
	}
	
	//--------------------------------------------------------------------
	// Get the i'th element of the quad.  Index starts from 0.
	const float& BaseQuad::operator[](int index) const
	{
		DE_ASSERT(IsWithinRange(index, 0, 3));
		return m_data[index];
	}
	
	float& BaseQuad::operator[](int index)
	{
		DE_ASSERT(IsWithinRange(index, 0, 3));
		return m_data[index];
	}
	
	//--------------------------------------------------------------------
	// Set the quad to (x, y, z, h)
	void BaseQuad::Set(float x, float y, float z, float h)
	{
		m_data[0] = x;
		m_data[1] = y;
		m_data[2] = z;
		m_data[3] = h;
	}
	
	//--------------------------------------------------------------------
	// Make sure that all elements are less then or equal to corresponding
	// limit's elements.
	// x<=limit.x, y<=limit.y, z<=limit.z, h<=limit.h
	void BaseQuad::ClampUpper(const BaseQuad& limit)
	{
		m_data[0] = Min(m_data[0], limit.m_data[0]);
		m_data[1] = Min(m_data[1], limit.m_data[1]);
		m_data[2] = Min(m_data[2], limit.m_data[2]);
		m_data[3] = Min(m_data[3], limit.m_data[3]);
	}
	
	//--------------------------------------------------------------------
	// Make sure that all elements are greater then or equal to
	// corresponding limit's elements.
	// x<=limit.x, y<=limit.y, z<=limit.z
	void BaseQuad::ClampLower(const BaseQuad& limit)
	{
		m_data[0] = Max(m_data[0], limit.m_data[0]);
		m_data[1] = Max(m_data[1], limit.m_data[1]);
		m_data[2] = Max(m_data[2], limit.m_data[2]);
		m_data[3] = Max(m_data[3], limit.m_data[3]);
	}
	
	//--------------------------------------------------------------------
	void BaseQuad::Clamp(const BaseQuad& loLimit, const BaseQuad& hiLimit)
	{
		ClampLower(loLimit);
		ClampUpper(hiLimit);
	}
	
	//--------------------------------------------------------------------
	// Write the text representation of the object to the output stream.
	// Format: (x, y, z)
	std::string ToString(const BaseQuad& bq)
	{
		return "(" + dodoEng::ToString(bq[0])
				+ ", " + dodoEng::ToString(bq[1])
				+ ", " + dodoEng::ToString(bq[2])
				+ ", " + dodoEng::ToString(bq[3]) + ")";
	}
}