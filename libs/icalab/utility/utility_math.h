#pragma once
//#define _USE_MATH_DEFINES // for C++
#include <cmath>
#ifndef M_E
#define M_E        2.71828182845904523536   // e
#endif

#ifndef M_LOG2E
#define M_LOG2E    1.44269504088896340736   // log2(e)
#endif

#ifndef M_LOG10E
#define M_LOG10E   0.434294481903251827651  // log10(e)
#endif

#ifndef M_LN2
#define M_LN2      0.693147180559945309417  // ln(2)
#endif

#ifndef M_LN10
#define M_LN10     2.30258509299404568402   // ln(10)
#endif

#ifndef M_PI
#define M_PI       3.14159265358979323846   // pi
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923   // pi/2
#endif

#ifndef M_PI_6
#define M_PI_6     0.523598775598299        // pi/6
#endif

#ifndef M_PI_3
#define M_PI_3     1.047197551196598        // pi/3
#endif

#ifndef M_PI_4
#define M_PI_4     0.785398163397448309616  // pi/4
#endif

#ifndef M_1_PI
#define M_1_PI     0.318309886183790671538  // 1/pi
#endif

#ifndef M_2_PI
#define M_2_PI     0.636619772367581343076  // 2/pi
#endif

#ifndef M_2_SQRTPI
#define M_2_SQRTPI 1.12837916709551257390   // 2/sqrt(pi)
#endif

#ifndef M_SQRT2
#define M_SQRT2    1.41421356237309504880   // sqrt(2)
#endif

#ifndef M_SQRT1_2
#define M_SQRT1_2  0.707106781186547524401  // 1/sqrt(2)
#endif
// float:
const float F_PI = static_cast<float>(M_PI);
const float rad60_F = F_PI / 3.0f;
const float rad45_F = F_PI / 4.0f;
const float rad30_F = F_PI / 6.0f;
inline float rad2degf(float rad) { return rad * 180.0f / F_PI; }
inline float deg2radf(float deg) { return deg * F_PI / 180.0f; }
// double:
const double rad60_D = M_PI / 3.0;
const double rad45_D = M_PI / 4.0;
const double rad30_D = M_PI / 6.0;
inline double rad2deg(double rad) { return rad * 180.0 / M_PI; }
inline void rad2deg(double* rad, double* deg, int len) {
	for (int i = 0; i < len; ++i)
		deg[i] = rad[i] * 180.0 / M_PI;
}
inline void rad2deg(double* deg, int len) {
	for (int i = 0; i < len; ++i)
		deg[i] = deg[i] * 180.0 / M_PI;
}
inline double deg2rad(double deg) { return deg * M_PI / 180.0; }
inline void deg2rad(double* deg, double* rad, int len) {
	for (int i = 0; i < len; ++i)
		rad[i] = deg[i] * M_PI / 180.0;
}
inline void deg2rad(double* rad, int len) {
	for (int i = 0; i < len; ++i)
		rad[i] = rad[i] * M_PI / 180.0;
}

template<typename T>
inline void swap(T* a, T* b, unsigned int len) {
	T* tmp = static_cast<T*>(malloc(sizeof(T) * len));
	memcpy(tmp, a, sizeof(T) * len);
	memmove(a, b, sizeof(T) * len);
	memmove(b, tmp, sizeof(T) * len);
	free(tmp);
}
template<typename T, unsigned int len>
inline void swap(T* a, T* b) {
	T tmp[len];
	memcpy(tmp, a, sizeof(T) * len);
	memmove(a, b, sizeof(T) * len);
	memmove(b, tmp, sizeof(T) * len);
}