/*
* -	Create: B. R. Tseng, 2023/03/08
* - Edit: B. R. Tseng, 2023/03/18
*/
#ifndef _SYSTEM_ETHERCAT_SETTING_H__
#define _SYSTEM_ETHERCAT_SETTING_H__

// threads' cyclic routine period:   
#define MOTION_CYCLE_ms				1.0 // 1.0 ms 
#define MOTION_FEEDBACK_CYCLE_ms	0.5 // 0.5 ms 
#define SAFETY_DETECT_CYCLE_ms		0.5 // 0.5 ms
#define ETHERCAT_CYCLE_us			500 // 0.5 ms

// EtherCAT slave index:
extern int SERVO_DRIVER_ECAT_SLAVE_ID[6];
extern int ATI_FT_SENSOR_ECAT_SLAVE_ID;
extern int ATI_FT_SENSOR_ECAT_IO_ID;
extern int  BLDC_SPINDLE_DAC_ECAT_SLAVE_ID;
extern int BLDC_SPINDLE_DAC_IO_ID;

#endif
