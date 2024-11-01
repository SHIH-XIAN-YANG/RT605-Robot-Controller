#pragma once

#include <Windows.h>
#include <stdio.h>
#include <tchar.h>
#include <rtapi.h>    // RTX64 APIs that can be used in real-time and Windows applications.
#include <math.h>

#ifdef UNDER_RTSS
#include <rtssapi.h>  // RTX64 APIs that can only be used in real-time applications.
#endif
#include "../libs/shared.hpp"
//#include "ksm64shared.h"

constexpr auto LENGTH = 4.5; // Active area length;
constexpr auto VMAX = 10.0;
constexpr auto VMIN = -10.0;
constexpr auto DAQ_RANGE = 65535.0;

extern LARGE_INTEGER glb_cpuFreq;
extern LARGE_INTEGER glb_tick_begin;
extern LARGE_INTEGER glb_tick_now;

extern FILE* fs;

inline int RTFCNDCL RtMeasurementRoutine(void* nContext) {
	// rtx64 real-time system measurement routine define below
	PsdData* psd = static_cast<PsdData*>(nContext);
	int ret = 0;
	// update the measurement:
	QueryPerformanceCounter(&glb_tick_now);
	static double time = 0.0;
	time = (double)(glb_tick_now .QuadPart - glb_tick_begin.QuadPart) / (double)(glb_cpuFreq.QuadPart); //s
	WORD raw_data[8] = {};
	KsError code;
	for (int i = 0; i < 8; i++) {
		
		if (i < 4)
			code = ReadInputWord(7, i*2, &(raw_data[i])); // since the input is 16bit == 2 byte each offset
		else
			code = ReadInputWord(8, (i - 4)*2, &(raw_data[i]));
		if (code != errNoError) {
			RtPrintf("Read Input Word Error!! index = %d Error:0x%x\n",i, code);
		}
		//RtPrintf("%i ", raw_data[i]);
	}
	//RtPrintf("\n");

	// Convert from 16-bit uint to double
	psd->psd1_vx1 = static_cast<double>(raw_data[0]) / DAQ_RANGE * (VMAX - VMIN) + VMIN;
	psd->psd1_vx2 = static_cast<double>(raw_data[1]) / DAQ_RANGE * (VMAX - VMIN) + VMIN;
	psd->psd1_vy1 = static_cast<double>(raw_data[2]) / DAQ_RANGE * (VMAX - VMIN) + VMIN;
	psd->psd1_vy2 = static_cast<double>(raw_data[3]) / DAQ_RANGE * (VMAX - VMIN) + VMIN;
	psd->psd2_vx1 = static_cast<double>(raw_data[4]) / DAQ_RANGE * (VMAX - VMIN) + VMIN;
	psd->psd2_vx2 = static_cast<double>(raw_data[5]) / DAQ_RANGE * (VMAX - VMIN) + VMIN;
	psd->psd2_vy1 = static_cast<double>(raw_data[6]) / DAQ_RANGE * (VMAX - VMIN) + VMIN;
	psd->psd2_vy2 = static_cast<double>(raw_data[7]) / DAQ_RANGE * (VMAX - VMIN) + VMIN;

	static double dx1, dy1, dx2, dy2;
	dx1 = (psd->psd1_vx2 + psd->psd1_vy1) - (psd->psd1_vx1 + psd->psd1_vy2);
	dy1 = (psd->psd1_vx2 + psd->psd1_vy2) - (psd->psd1_vx1 + psd->psd1_vy1);
	psd->ui->sigma1 = (psd->psd1_vx1 + psd->psd1_vx2 + psd->psd1_vy1 + psd->psd1_vy2);

	psd->ui->x1 = (dx1 / psd->ui->sigma1) * (LENGTH / 2.0);
	psd->ui->y1 = (dy1 / psd->ui->sigma1) * (LENGTH / 2.0);

	psd->ui->x1 = isinf(psd->ui->x1) ? 0.0 : psd->ui->x1;
	psd->ui->y1 = isinf(psd->ui->y1) ? 0.0 : psd->ui->y1;

	dx2 = (psd->psd2_vx2 + psd->psd2_vy1) - (psd->psd2_vx1 + psd->psd2_vy2);
	dy2 = (psd->psd2_vx2 + psd->psd2_vy2) - (psd->psd2_vx1 + psd->psd2_vy1);
	psd->ui->sigma2 = (psd->psd2_vx1 + psd->psd2_vx2 + psd->psd2_vy1 + psd->psd2_vy2);

	psd->ui->x2 = dx2 / psd->ui->sigma2 * LENGTH / 2;
	psd->ui->y2 = dy2 / psd->ui->sigma2 * LENGTH / 2;

	psd->ui->x2 = isinf(psd->ui->x2) ? 0.0 : psd->ui->x2;
	psd->ui->y2 = isinf(psd->ui->y2) ? 0.0 : psd->ui->y2;
	if (psd->ui->log && fs != nullptr) {
		fprintf_s(fs, "%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g\n", time, psd->psd1_vx1, psd->psd1_vx2, psd->psd1_vy1, psd->psd1_vy2, psd->ui->x1, psd->ui->y1, psd->ui->sigma1, dx1, dy1, \
			psd->psd2_vx1, psd->psd2_vx2, psd->psd2_vy1, psd->psd2_vy2, psd->ui->x2, psd->ui->y2, psd->ui->sigma2, dx2, dy2, \
			psd->ui->robot_pos[0], psd->ui->robot_pos[1], psd->ui->robot_pos[2]);
	}
	//RtPrintf("%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g\n", time, psd->psd1_vx1, psd->psd1_vx2, psd->psd1_vy1, psd->psd1_vy2, psd->ui->x1, psd->ui->y1, psd->ui->sigma1, dx1, dy1, \
	//	psd->psd2_vx1, psd->psd2_vx2, psd->psd2_vy1, psd->psd2_vy2, psd->ui->x2, psd->ui->y2, psd->ui->sigma2, dx2, dy2, \
	//	psd->ui->robot_pos[0], psd->ui->robot_pos[1], psd->ui->robot_pos[2]);
	return ret;
}