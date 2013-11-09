// BaseTriple.h: interface for the BaseTriple class.
// Author: Davor Mrkoci
//
// BaseTriple provides basic functionality for three dimensional graphics
// objects such as points and vectors.

#if !defined(DODOENG_BASETRIPLE_H_)
#define DODOENG_BASETRIPLE_H_

namespace dodoEng
{
	class BaseTriple  
	{
		//--------------------------------------------------------------------
		// Interface
		//--------------------------------------------------------------------
	public:
		// Default constructor
		BaseTriple();
		
		// Constructor
		BaseTriple(float x, float y, float z);
		
		// Destructor
		virtual ~BaseTriple();
		
		// Get the i'th element of the triple.  Index starts from 0.
		const float& operator[](int index) const;
		float& operator[](int index);
		
		// Set the triple to (x, y, z)
		void Set(float x, float y, float z);
		
		// Clamp the values of the base triple elements.
		void ClampUpper(const BaseTriple& limit);
		void ClampLower(const BaseTriple& limit);
		void Clamp(const BaseTriple& loLimit, const BaseTriple& hiLimit);
				
		//--------------------------------------------------------------------
		// Data Members
		//--------------------------------------------------------------------
	protected:
		float m_data[3];
	};

	//return string representation of the object
	std::string ToString(const BaseTriple& bt);
}

#endif
