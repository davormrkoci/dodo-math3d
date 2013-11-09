#include "Assert.h"
#include "UtilFnc.h"
#include "String.h"

#include "BaseTriple.h"

namespace dodoEng
{
	
	//--------------------------------------------------------------------
	// Default constructor
	BaseTriple::BaseTriple()
	{
		m_data[0] = 0.0f;
		m_data[1] = 0.0f;
		m_data[2] = 0.0f;
	}
	
	//--------------------------------------------------------------------
	// Constructor
	BaseTriple::BaseTriple(float x, float y, float z)
	{
		m_data[0] = x;
		m_data[1] = y;
		m_data[2] = z;
	}
	
	//--------------------------------------------------------------------
	// Destructor
	BaseTriple::~BaseTriple()
	{
	}
	
	//--------------------------------------------------------------------
	// Get the i'th element of the triple.  Index starts from 0.
	const float& BaseTriple::operator[](int index) const
	{
		DE_ASSERT(IsWithinRange(index, 0, 2));
		return m_data[index];
	}
	
	//--------------------------------------------------------------------
	// Get the i'th element of the triple.  Index starts from 0.
	float& BaseTriple::operator[](int index)
	{
		DE_ASSERT(IsWithinRange(index, 0, 2));
		return m_data[index];
	}
	
	//--------------------------------------------------------------------
	// Set the triple to (x, y, z)
	void BaseTriple::Set(float x, float y, float z)
	{
		m_data[0] = x;
		m_data[1] = y;
		m_data[2] = z;
	}
	
	//--------------------------------------------------------------------
	// Make sure that all elements are less then or equal to corresponding
	// limit's elements.
	// x<=limit.x, y<=limit.y, z<=limit.z
	void BaseTriple::ClampUpper(const BaseTriple& limit)
	{
		m_data[0] = Min(m_data[0], limit.m_data[0]);
		m_data[1] = Min(m_data[1], limit.m_data[1]);
		m_data[2] = Min(m_data[2], limit.m_data[2]);
	}
	
	//--------------------------------------------------------------------
	// Make sure that all elements are greater then or equal to
	// corresponding limit's elements.
	// x<=limit.x, y<=limit.y, z<=limit.z
	void BaseTriple::ClampLower(const BaseTriple& limit)
	{
		m_data[0] = Max(m_data[0], limit.m_data[0]);
		m_data[1] = Max(m_data[1], limit.m_data[1]);
		m_data[2] = Max(m_data[2], limit.m_data[2]);
	}
	
	//--------------------------------------------------------------------
	void BaseTriple::Clamp(const BaseTriple& loLimit, const BaseTriple& hiLimit)
	{
		ClampLower(loLimit);
		ClampUpper(hiLimit);
	}
	
	//--------------------------------------------------------------------
	// Write the text representation of the object to the output stream.
	// Format: (x, y, z)
	std::string ToString(const BaseTriple& bt)
	{
		return "(" + dodoEng::ToString(bt[0])
				+ ", " + dodoEng::ToString(bt[1])
				+ ", " + dodoEng::ToString(bt[2]) + ")";
	}
}
