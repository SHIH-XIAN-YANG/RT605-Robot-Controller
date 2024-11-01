#ifndef __RT605_710_PARAMETERS_H__
#define __RT605_710_PARAMETERS_H__


#define ServoResolution 131072

const double ServoPitchNum[] = { 262144, 327680, 262144, 147456, 262144, 163840 };
const double ServoPitchDen[] = { 9, 9, 9, 5, 9, 9 };
const bool ServoReverse[] = { 0, 0, 1, 1, 1, 1 };
const double GR[6] = { 80, 100, 80, 81, 80, 50 };
const double ServoPositionOffset[] = { // Negative offset in Joint space, e.g. q_actual = q_feedback - ServoPositionOffset.
									-206.0321045,      //-75014/   1/  (ServoResolution/360) ,      // J1_bias
									242.4160767,        // 88261/   1/  (ServoResolution / 360) ,	// J2_bias
									-134.6044922 * -1, //-49008/   -1/  (ServoResolution / 360),	// J3_bias
									-27.90802002 * -1,  //-10161/   -1/  (ServoResolution / 360),	// J4_bias
									- 31.33575439 * -1, //-11409/   -1/  (ServoResolution / 360),	// J5_bias
									- 265.7922363 * -1 //-96772 /  -1/  (ServoResolution / 360)};	// J6_bias
};
const double ServoRatedTorque[6] = { 1.27, 1.27, 0.636, 0.318, 0.159, 0.159 };
//const double RT605_HomePosition[6] = { 90.0, 90.0, 0.0, 0.0, -90.0, 0.0 };
//const double RT605_VerticalConfiguration[6] = { 0.0, 0.0, 90.0, 0.0, 0.0, 0.0 };
enum class RobotParameterErrorCode : unsigned char {
	PPR        = 0, // pitch per revolute
	Resolution = 1, 
	Zeros      = 2, // motor zero pulse
	Bias       = 3, // kinematics joint bias of D-H table
	GR         = 4, // joint gear ratio
	Home       = 5, // joint home position
	RateTor    = 6, // motor rated torque
	JHlim      = 7, // joint hardware limit
	JSlim      = 8, // joint software limit
};
typedef struct RobotParameter{
	double Motor_pitch_per_revolute; // PPR
	unsigned int Servo_resolution;
	double Motor_zero_pulse[6];
	double Kinematics_joint_bias[6];
	double Joint_gear_ratio[6];
	double Joint_home_position[6];
	double Joint_custom_position1[6];
	double Joint_custom_position2[6];
	double Motor_rated_torque[6];
	double Joint_hardware_limit[6][2];
	double Joint_Software_limit[6][2];
	double Servo_electronic_gear_ratio[6][2];
	//
	inline RobotParameterErrorCode ImportRobotParameter(void){
		
	}
	//
	inline bool SetServoElectronicGearRatio(void){
		
	}
}*pRobotParameter, RobotParameter;

#endif
/*
 - Robot Parameters Database Example: (csv form)
   =================================================
   (PPR)     360.0\n
   (Res)     131072\
   (Zeros)   -75061.0,67294.0,-30938.0,-10178.0,-5821.0,-110312.0\n
   (Bias)    90,90,0.0,0.0,0.0,0.0\n
   (GR)	     80,100,-80,-81,-80,-50\n
   (Home)    90,90,0.0,0.0,-90,0.0\n
   (Custom1) 90,90,0.0,0.0,-90,0.0\n
   (Custom2) 90,90,0.0,0.0,-90,0.0\n
   (RateTor) 1.27,1.27,0.636,0.318,0.159,0.159\n
   (J1_H_lim)-75.0,255.0\n
   (J2_H_lim)-35.0,175.0\n
   (J3_H_lim)-55.0,185.0\n
   (J4_H_lim)-190.0,190.0\n
   (J5_H_lim)-115.0,115.0\n
   (J6_H_lim)-360.0,360.0\n
   (J1_S_lim)-75.0,255.0\n
   (J2_S_lim)-35.0,175.0\n
   (J3_S_lim)-55.0,185.0\n
   (J4_S_lim)-190.0,190.0\n
   (J5_S_lim)-115.0,115.0\n
   (J6_S_lim)-360.0,360.0\n
   =================================================
*/