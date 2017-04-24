/*******************************************************************************
* config.h
*
* Contains the settings struct for configuration of balance.c
* Settings are contained here for neatness.
* Macros are used to take CONFIG_TABLE and both declare a struct
* and then fill it with the default values.
*******************************************************************************/

#ifndef BALANCE_CONFIG
#define BALANCE_CONFIG

#define SAMPLE_RATE_HZ 200	// main filter and control loop speed
#define DT 0.005      		// 1/sample_rate


// Structural properties of BBB

#define 	CAPE_MOUNT_ANGLE_X		M_PI/2//  0.35 for blUE
#define 	CAPE_MOUNT_ANGLE_Y		M_PI/2//  0.35 for blUE
#define 	CAPE_MOUNT_ANGLE_Z		0//  0.35 for blUE
//#define 	GEARBOX 				35.577
#define 	ENCODER_RES				60
//#define 	WHEEL_RADIUS_M			0.034
//#define 	TRACK_WIDTH_M			0.035
#define 	V_NOMINAL				7.4


// inner loop controller 200hz
#define 	D1_GAIN					-3.5
#define 	D1_ORDER				2
#define 	D1_NUM					{1,0.14,0.0049}
#define 	D1_DEN					{1, 0.707, 0.0049}
#define 	D1_SATURATION_TIMEOUT	0.5

// outer loop controller original 20hz
#define		D2_GAIN					0.3
#define		D2_ORDER				1
#define		D2_NUM					{ 1, -1 }
#define		D2_DEN					{ 1.0000, -0.9 }
#define 	THETA_REF_MAX	0.37

// electrical hookups
#define 	MOTOR_CHANNEL_L 		4
#define 	MOTOR_CHANNEL_R 		1
#define 	MOTOR_POLARITY_L 		1
#define 	MOTOR_POLARITY_R 		-1
#define 	ENCODER_CHANNEL_L 		3
#define 	ENCODER_CHANNEL_R 		2
#define 	ENCODER_POLARITY_L 		1
#define 	ENCODER_POLARITY_R 		-1

// Thread Loop Rates
#define		BATTERY_CHECK_HZ		5
#define 	REFERENCE_VALUE_MANAGER_HZ	100
#define		PRINTF_HZ			50
#define		GET_INPUT_HZ		10
// other
#define 	TIP_ANGLE 			0.75
#define 	START_ANGLE 			0.3	
#define 	START_DELAY 			0.5	
#define 	PICKUP_DETECTION_TIME 	0.65
#define 	ENABLE_POSITION_HOLD	1
#define 	SOFT_START_SEC			0.7

#endif //BALANCE_CONFIG
