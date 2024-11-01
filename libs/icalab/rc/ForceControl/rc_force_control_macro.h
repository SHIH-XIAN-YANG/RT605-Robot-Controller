/*
	- Create: 2022/08/25, b.r.tseng
	- Edit: 2022/08/25, b.r.tseng
*/
#ifndef __RC_FORCE_CONTROL_MACRO_H__
#define __RC_FORCE_CONTROL_MACRO_H__
// -----------------------------------------------------------------------------------------------------
// 其他不同的預設值：
// -----------------------------------------------------------------------------------------------------
#define CALIBRATE_FROM_TOOL_TO_SENSOR false
#define CALIBRATE_FROM_SENSOR_TO_END  true
// -----------------------------------------------------------------------------------------------------
// Force controller 控制暫存器-0 icalab::ForceController::m_setting[0]<16-bit> 之各個 位元 所設定的參數:
// -----------------------------------------------------------------------------------------------------
// bit 7:0
#define FC_SET0_BYTE0   0b00ff
#define FC_ON           0x0001  // bit 7:0 |= Bit0
#define CONTROL_SPACE   0x0002  // bit 7:0 |= Bit1 => True:False = Joint-space:Cartesian-space
#define CARTESIAN_SPACE false
#define JOINT_SPACE	true

#define FC_G_COMPENSATE 0x0004  // bit 7:0 |= Bit2
#define RESERVED1_3     0x0008  // bit 7:0 |= Bit3
#define RESERVED1_4     0x0010  // bit 7:0 |= Bit4
#define RESERVED1_5     0x0020  // bit 7:0 |= Bit5
#define RESERVED1_6     0x0040  // bit 7:0 |= Bit6
#define RESERVED1_7     0x0080  // bit 7:0 |= Bit7
// bit 15:8
#define FC_SET0_BYTE1   0bff00
#define RESERVED1_8     0x0100  // bit 15:8 |= Bit0
#define RESERVED1_9     0x0200  // bit 15:8 |= Bit1
#define RESERVED1_10    0x0400  // bit 15:8 |= Bit2
#define RESERVED1_11    0x0800  // bit 15:8 |= Bit3
#define RESERVED1_12    0x1000  // bit 15:8 |= Bit4
#define RESERVED1_13    0x2000  // bit 15:8 |= Bit5
#define RESERVED1_14    0x4000  // bit 15:8 |= Bit6
#define RESERVED1_15    0x8000  // bit 15:8 |= Bit7
// -----------------------------------------------------------------------------------------------------
// Force controller 控制暫存器-1 icalab::ForceController::m_setting[1]<16-bit> 之各個 位元 所設定的參數:
// -----------------------------------------------------------------------------------------------------
// bit 5:0  關節空間的力量控制方向
#define FC_SET1_JOINT_DIR \
               0b0000000000111111
#define FC_q1  0b0000000000000001  // bit 5:0 |= Bit0
#define FC_q2  0b0000000000000010  // bit 5:0 |= Bit1
#define FC_q3  0b0000000000000100  // bit 5:0 |= Bit2
#define FC_q4  0b0000000000001000  // bit 5:0 |= Bit3
#define FC_q5  0b0000000000010000  // bit 5:0 |= Bit4
#define FC_q6  0b0000000000100000  // bit 5:0 |= Bit5
// bit 9:6 卡氏座標的力量控制方向：
#define FC_SET1_CARTESIAN_DIR \
               0b0000111111000000
#define FC_X   0b0000000001000000  // bit 9:6 = Decimal(1)
#define FC_Y   0b0000000010000000  // bit 9:6 = Decimal(2)
#define FC_Z   0b0000000011000000  // bit 9:6 = Decimal(3)
#define FC_Tx  0b0000000100000000  // bit 9:6 = Decimal(4)
#define FC_Ty  0b0000000101000000  // bit 9:6 = Decimal(5)
#define FC_Tz  0b0000000110000000  // bit 9:6 = Decimal(6)
// bit 11:10 力量控制器輸出命令的單位設定：
#define FC_SET1_OP_LIM    0b0000110000000000
#define FC_LIM_CLOSE      0b0000000000000000 // bit 11:10 = Decimal(0)
#define FC_LIM_UNIT_mm    0b0000010000000000 // bit 11:10 = Decimal(1)  如果 m_setting[0] 之 Bit1 設為 Joint-space 則此單位代表 m-radian 反之則代表 mm
/*     列舉目前試驗的 "力量控制演算法"，其中皆是 Position-based 的力量控制，
   亦即是:
		Xd(k+1) = X(k)    + FC_function( Fd - Fext )
   或
		Xd(k+1) = Xr(k+1) + FC_function( Fd - Fext )
*/
enum class FcAlgorithm : unsigned short {
	Impedance1, // 本實驗室 ICALAB 之研究
	Impedance2, // 基於文獻 [1, 2] 之方法
	PID1,
	PID2, 
	Admittance // 順應性
};

#endif