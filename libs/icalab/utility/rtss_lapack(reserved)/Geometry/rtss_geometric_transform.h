/*
	- Create: 2022/08/25, b.r.tseng
	- Edit: 2022/08/25, b.r.tseng
*/
#ifndef __RTSS_GEOMETRIC_TRANSFORM_H__
#define __RTSS_GEOMETRIC_TRANSFORM_H__
#include<math.h>

#include<array>

// 將力量感測器 FTS 之量測值轉換至 工具端點 TCP 所感受的等效作用力：
void StaticForceTransform_FtsToTcp(double* transform_fts_tcp, double* FTS, double* Ftcp);
void StaticForceTransform_FtsToTcp(std::array<double, 6>& transform_fts_tcp, std::array<double, 6>& FTS, std::array<double, 6>& Ftcp);
void StaticForceTransform_FtsToTcp(std::array<double, 6>& transform_fts_tcp, double* FTS, double* Ftcp);

#endif
