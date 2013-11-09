#if !defined(DODOENG_BASEQUAD_H_)
#define DODOENG_BASEQUAD_H_

namespace dodoEng
{
	class BaseQuad  
	{
		//--------------------------------------------------------------------
		// Interface
		//--------------------------------------------------------------------
	public:
		// Default constructor
		BaseQuad();
		
		// Constructor
		BaseQuad(float x, float y, float z, float h);
		
		// Destructor
		virtual ~BaseQuad();
		
		// Get the i'th element of the triple.  Index starts from 0.
		const float& operator[](int index) const;
		float& operator[](int index);
		
		// Set the quad to (x, y, z, h)
		void Set(float x, float y, float z, float h);
		
		// Clamp the values of the base quad elements.
		void ClampUpper(const BaseQuad& limit);
		void ClampLower(const BaseQuad& limit);
		void Clamp(const BaseQuad& loLimit, const BaseQuad& hiLimit);
				
		//--------------------------------------------------------------------
		// Data Members
		//--------------------------------------------------------------------
	protected:
		float m_data[4];
	};

	//return string representation of the object
	std::string ToString(const BaseQuad& bq);
}

#endif
