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

#define SAMPLE_RATE_HZ 					200	// main filter and control loop speed
#define DT 								0.005      		// 1/sample_rate


// Structural properties of BBB

#define 	CAPE_MOUNT_ANGLE_X			M_PI/2
#define 	CAPE_MOUNT_ANGLE_Y			M_PI/2
#define 	CAPE_MOUNT_ANGLE_Z			0
#define 	V_NOMINAL					7.4

// electrical hookups
#define 	WS_MOTOR_CHANNEL 			3
#define 	BL_MOTOR_CHANNEL_L 			4
#define 	BL_MOTOR_CHANNEL_R 			1
#define 	BL_MOTOR_POLARITY_L 		1
#define 	BL_MOTOR_POLARITY_R 		-1

// DSM (radio) hookups
#define		WS_RADIO_CHANNEL			1
#define		BL_RADIO_CHANNEL_L			2
#define		BL_RADIO_CHANNEL_R			3

// Thread Loop Rates
#define		BATTERY_CHECK_HZ			5
#define 	REFERENCE_VALUE_MANAGER_HZ	100
#define		PRINTF_HZ					50
#define		READ_INPUT_HZ				10
#define		CHECK_RADIO_SIGNAL_HZ 		10
// other


#endif //BALANCE_CONFIG
