/*
	- Create: 2022/08/25, b.r.tseng
	- Edit: 2022/08/25, b.r.tseng
*/
#ifndef __PERIPHERAL_H__
#define __PERIPHERAL_H__

#include<windows.h>
#include<stdio.h>
#include<string.h>

#include<array>

#include <rtapi.h>
//#include"ksm64shared.h"
#include "ksapi.h"
#include "ksmotion.h"

#include"eigen-3.4.0/Eigen/Dense"
#include"eigen-3.4.0/Eigen/QR"

namespace sensor {

	struct FtData {
		std::array<double, 6> raw;   // Sensor 原始量測資料，轉換成所需的物理單位 (e.g. N, N-m)
		std::array<double, 6> cal;   // 裝置工具後，經過重力補償所計算的值
		std::array<double, 6> tcp;   // 將經過重力補償所計算的值進行靜態轉換，計算出當前 tcp 座標系下的受力
	};
	//
	enum class SamplingRate : DWORD {
		fs_500Hz = 0x00000000,
		fs_1000Hz = 0x00001000,
		fs_2000Hz = 0x00002000,
		fs_4000Hz = 0x00003000
	};
	enum class LPF : DWORD {
		/*------------------------------------------
		*       -3dB Cuttoff Frequency (in Hz)		|
		* ------------------------------------------|
		|Selected|                                  |
		|Filter  | 0.5kHz | 1.0kHz | 2.0kHz | 4.0kHz|
		--------------------------------------------|
		|0		 | 200    | 350	   | 500    | 1000  |
		|1		 | 58	  |115	   | 235    | 460   |
		|2		 | 22     | 45	   | 90	    | 180	|
		|3		 | 10     | 21	   | 43	    | 84	|
		|4		 | 5      | 10	   | 20	    | 40	|
		|5		 | 2.5    | 5	   | 10	    | 20	|
		|6		 | 1.3    | 3	   | 5		| 10	|
		|7		 | 0.6    | 1.2	   | 2.4    | 4.7	|
		|8		 | 0.3    | 0.7	   | 1.4    | 2.7	|
		--------------------------------------------
		*/
		//  sel0: fs = 0.5kHz, 1.0kHz, 2.0kHz, 4.0kHz => fc = 200Hz, 350Hz, 500Hz, 1000Hz
		sel0 = 0x00000000,
		//  sel1: fs = 0.5kHz, 1.0kHz, 2.0kHz, 4.0kHz => fc = 58Hz, 115Hz, 235Hz, 460Hz
		sel1 = 0x00000010,
		//  sel2: fs = 0.5kHz, 1.0kHz, 2.0kHz, 4.0kHz => fc = 22Hz, 45Hz, 90Hz, 180Hz
		sel2 = 0x00000020,
		//  sel3: fs = 0.5kHz, 1.0kHz, 2.0kHz, 4.0kHz => fc = 10Hz, 21Hz, 43Hz, 84Hz
		sel3 = 0x00000030,
		//  sel4: fs = 0.5kHz, 1.0kHz, 2.0kHz, 4.0kHz => fc = 5Hz, 10Hz, 20Hz, 40Hz
		sel4 = 0x00000040,
		//  sel5: fs = 0.5kHz, 1.0kHz, 2.0kHz, 4.0kHz => fc = 2.5Hz, 5Hz, 10Hz, 20Hz
		sel5 = 0x00000050,
		//  sel6: fs = 0.5kHz, 1.0kHz, 2.0kHz, 4.0kHz => fc = 1.3Hz, 3Hz, 5Hz, 10Hz
		sel6 = 0x00000060,
		//  sel7: fs = 0.5kHz, 1.0kHz, 2.0kHz, 4.0kHz => fc = 0.6Hz, 1.2Hz, 2.4Hz, 4.7Hz
		sel7 = 0x00000070,
		//  sel8: fs = 0.5kHz, 1.0kHz, 2.0kHz, 4.0kHz => fc = 0.3Hz, 0.7Hz, 1.4Hz, 2.7Hz
		sel8 = 0x00000080,

	};
	// Tool:
	struct ToolDynamics {
		double mass;
		double fx0;
		double fy0;
		double fz0;
		double rcx;
		double rcy;
		double rcz;
		double ixx;
		double iyy;
		double mx0;
		double my0;
		double mz0;
		double izz;
	};
	//
	class Axia80 {
	private:
		DWORD m_raw_measurementI32[6];     // Sensor 原始量測資料，EtherCAT 抓取的原始 INT 32 資料
		std::array<double, 6> m_bias_Fg;  
		std::array<double, 6> m_bias_q;
		ToolDynamics m_tool;
		int m_ecat_slave_id;
		int m_ecat_io_id;
		// RobotInfo:
		double m_transform_sensor_to_tcp[6];
		double* m_q; // robot joint (unit: rad)
		// 重力補償 與 座標轉換：
		void m_GravityCompensation(FtData& data_buf);
		void m_StaticForceTransform_FtsToTcp(FtData& data_buf);
	public:
		Axia80(int ecat_slave_id, int ecat_io_id);
		Axia80(void);
		~Axia80(void);

		void setEtherCATid(int ecat_slave_id, int ecat_io_id);
		void setupDAQ(SamplingRate fs, LPF fc);
		void setTransformSensorToTCP(double* transform_sensor_tcp);
		void setRobotJointPointer(double* q);
		void setSensorZero(void);
		void setBias(void);
		void computeGravity(double* q, double* ret_fg);
		void setToolDynamics(ToolDynamics& tool);
		void setToolDynamics(double* tool);
		void ReadData(FtData& data_buf);
		void GetRawMeasurement(DWORD* raw_data);
		void IdentifyToolDynamics(char* id_data_file);
		void getToolDynamics(sensor::ToolDynamics* tool_val);

	};
}
//
namespace tool {
	class BLDC_Spindle {
	private:
		int m_dac_ecat_slave_id;
		int m_dac_ecat_io_id;

		int m_dir_ecat_slave_id;
		int m_dir_ecat_io_id;

		WORD m_vel_cmd;
		WORD m_vel;
	public:
		BLDC_Spindle(void);
		~BLDC_Spindle(void);

		void setEtherCATid(int* m_dac_ecat_id, int* m_dir_ecat_id);
		void TurnOff(void);
		WORD GetSpindleVel_rpm(void);
		void operator()(float vel_cmd);
	};
}
#endif
