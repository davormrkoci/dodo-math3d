#if !defined(DODOENG_CONST_H_)
#define DODOENG_CONST_H_

namespace dodoEng
{
	//index pt/vector
	const int X = 0;
	const int Y = 1;
	const int Z = 2;
	const int H = 3;

	//index color
	const int R = 0;
	const int G = 1;
	const int B = 2;
	const int A = 3;

	//constants
	const float EPSILON = std::numeric_limits<float>::epsilon();
	const float PI = 3.1415926535897932384626433832795028841971693993751058209749445923078164062862f;
	const float HALFPI = PI * 0.5f;
	const float QUATERPI = PI * 0.25f;
	const float TWOPI = PI + PI;
}

#endif
