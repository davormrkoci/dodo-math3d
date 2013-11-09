#if !defined(DODOENG_UTILFNC_H_)
#define DODOENG_UTILFNC_H_

namespace dodoEng
{
	//util functions
	template<class T>
		const T& Max(const T& a, const T& b)
	{
		return (a<b) ? b : a;
	}
	
	template<class T>
		const T& Min(const T& a, const T& b)
	{
		return (a<b) ? a : b;
	}

	template <class T>
		const T Clamp(const T& value, const T& loLimit, const T& hiLimit)
	{
		return Min(Max(value, loLimit), hiLimit);
	}

	template<class T>
		bool IsWithinRange(const T& value, const T& loLimit, const T& hiLimit)
	{
		return (value >= loLimit) && (value <= hiLimit);
	}
}

#endif