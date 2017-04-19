#include <usefulincludes.h>
#include <roboticscape.h>
#include "balance_config.h"

/*******************************************************************************
* armstate_t
*
* ARMED or DISARMED to indicate if the controller is running
*******************************************************************************/
typedef enum armstate_t{
	ARMED,
	DISARMED
} armstate_t;

/*******************************************************************************
* reference_value_t
*	
* Controller reference_value written to by reference_value_manager and read by the controller.
*******************************************************************************/
typedef struct reference_value_t{ //setpoints
	armstate_t armstate;	// see armstate_t declaration
	float theta;			// body lean angle (rad)
	float phi;				// wheel position (rad)
} reference_value_t;

/*******************************************************************************
* core_state_t
*
* This is the system state written to by the controller.	
*******************************************************************************/
typedef struct core_state_t{
	float right_pulley_angle;	// pulley rotation
	float left_pulley_angle;
	float angle_about_x_axis; 		// body angle radians
	float angle_about_y_axis; 		// body angle radians
	float angle_about_z_axis; 		// body angle radians
	float weightshift_dist;			//distance of weight from neutral position 
	float battery_voltage; 		// battery voltage 
	float u1;			// output of controller to weight shift
	float u2_left;			// output of controller to pulley motors
	float u2_right;
} core_state_t;

/*******************************************************************************
* Local Function declarations	
*******************************************************************************/
// IMU interrupt routine
int control_tilt();
// threads
void* reference_value_manager(void* ptr);
void* battery_checker(void* ptr);
void* printf_loop(void* ptr);
void* outer_loop(void* ptr);
// regular functions
int zero_out_controller();
int disarm_controller();
int arm_controller();
int on_pause_pressed();
int on_pause_released();

/*******************************************************************************
* Global Variables				
*******************************************************************************/
core_state_t sys_state;
reference_value_t reference_value;
imu_data_t data;

//create filters
d_filter_t lowpass;
d_filter_t highpass;

//global variable for angle so it can be used as a reference for each call from imu interrupt
static double xangle = 0;
static double x_orientation_from_accel;
static double yangle = 0;
static double y_orientation_from_accel;
static double zangle = 0;
static double z_orientation_from_accel;

/*******************************************************************************
* main()
*
* Initialize the filters, IMU, threads, & wait until shut down
*******************************************************************************/
int main(){
	set_cpu_frequency(FREQ_1000MHZ);

	if(initialize_cape()<0){
		printf("ERROR: failed to initialize cape!\n");
		return -1;
	}
	set_state(UNINITIALIZED);


	// make sure reference_value starts at normal values
	reference_value.armstate = DISARMED;
		
	//timestep, dt
	double const dt = 1.0/(float)SAMPLE_RATE_HZ;
	//rise time, tr **********************************************find new rise time if needed************************
	double const tr = 1;
	//time constant from rise time
	float const tau = tr/2.2;

	//set up filters for finding theta from sensors
	lowpass  = create_first_order_lowpass(dt, tau);
	highpass  = create_first_order_highpass(dt, tau);

	// set up button handlers
	set_pause_pressed_func(&on_pause_pressed);
	set_mode_released_func(&on_pause_released);
	
	// start a thread to slowly sample battery 
	pthread_t battery_thread;
	pthread_create(&battery_thread, NULL, battery_checker, (void*) NULL);
	// wait for the battery thread to make the first read
	while(sys_state.battery_voltage==0 && get_state()!=EXITING) usleep(1000);
	
	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	if(isatty(fileno(stdout))){
		pthread_t  printf_thread;
		pthread_create(&printf_thread, NULL, printf_loop, (void*) NULL);
	}
	
	// set up IMU configuration
	imu_config_t conf = get_default_imu_config();
	conf.dmp_sample_rate = SAMPLE_RATE_HZ;
	


	//*****************************need orientation***********************************
	conf.orientation = ORIENTATION_Y_UP;
	//


	// start imu
	if(initialize_imu_dmp(&data, conf)){
		printf("ERROR: IMU initialization failed!\n");
		return -1;
	}
	
	// start stack to control reference_values
	pthread_t reference_value_thread;
	pthread_create(&reference_value_thread, NULL, reference_value_manager, (void*) NULL);

	//start thread for outer loop controller
	pthread_t outer_loop_thread;
	pthread_create(&outer_loop_thread, NULL, outer_loop, (void*) NULL);

	//set imu interrupt function
	set_imu_interrupt_func(&control_tilt);
	
	
	set_state(RUNNING);
	set_led(RED,0);
	set_led(GREEN,1);
	
	//wait while stuff is going on
	while(get_state()!=EXITING){
		usleep(10000);
	}
	
	//cleanup
	destroy_filter(&lowpass);
	destroy_filter(&highpass);
	power_off_imu();
	cleanup_cape();
	set_cpu_frequency(FREQ_ONDEMAND);
	return 0;
}


/*******************************************************************************
* void* reference_value_manager(void* ptr)
*
* Detects start conditions to control arming the controller.
*******************************************************************************/
void* reference_value_manager(void* ptr){
	// wait for IMU to settle
	disarm_controller();
	usleep(2500000);
		
	while(get_state()!=EXITING){
		// sleep at beginning of loop so we can use the 'continue' statement
		usleep(1000000/REFERENCE_VALUE_MANAGER_HZ); 
		
		// nothing to do if paused, go back to beginning of loop
		if(get_state() != RUNNING) continue;

		// if we got here the state is RUNNING, but controller is not
		// necessarily armed. If DISARMED, wait for the user to pick MIP up
		// which will we detected by wait_for_starting_condition()
		if(reference_value.armstate == DISARMED){
			if(wait_for_starting_condition()==0){
				zero_out_controller();
				arm_controller();
			} 
			else continue;
		}
	
		
	}

	// if state becomes EXITING the above loop exists and we disarm here
	disarm_controller();
	return NULL;
}

/*******************************************************************************
* int control_tilt()
*
* Finds orientation from sensors and sends signal to motors to meet setpoint
*******************************************************************************/
int control_tilt(){

	// motor duty cycles
	float left_pulley_duty_cycle, right_pulley_duty_cycle;

	

	// angle theta is positive in the direction of forward tip around X axis********************redefine******************change data.accel[], data.gyro[] etc because numbers might be wrong
	// find angle from accelerometer
	x_orientation_from_accel = atan2(-1*data.accel[2],data.accel[1]);

	// integrates the gyroscope angle rate using Euler's method to get the angle
	xangle = xangle + 0.01*data.gyro[0]*DEG_TO_RAD;
	
	// filter angle data
	double lp_x = march_filter(&lowpass, x_orientation_from_accel);
	double hp_x = march_filter(&highpass, xangle);
	
	// get most recent filtered value
	double lp_filtered_output_x = newest_filter_output(&lowpass);
	double hp_filtered_output_x = newest_filter_output(&highpass);
	
	//complementary filter to get theta
	sys_state.angle_about_x_axis = lp_filtered_output+hp_filtered_output + CAPE_MOUNT_ANGLE_X;
	
	

	// angle theta is positive in the direction of forward tip around X axis********************redefine***************
	// find angle from accelerometer
	y_orientation_from_accel = atan2(-1*data.accel[2],data.accel[1]);

	// integrates the gyroscope angle rate using Euler's method to get the angle
	yangle = yangle + 0.01*data.gyro[0]*DEG_TO_RAD;
	
	// filter angle data
	double lp_y = march_filter(&lowpass, y_orientation_from_accel);
	double hp_y = march_filter(&highpass, yangle);
	
	// get most recent filtered value
	double lp_filtered_output_y = newest_filter_output(&lowpass);
	double hp_filtered_output_y = newest_filter_output(&highpass);
	
	//complementary filter to get theta
	sys_state.angle_about_y_axis = lp_filtered_output_y+hp_filtered_output_y + CAPE_MOUNT_ANGLE_Y;
	
	

	// angle theta is positive in the direction of forward tip around X axis*********************redefine**********
	// find angle from accelerometer
	z_orientation_from_accel = atan2(-1*data.accel[2],data.accel[1]);

	// integrates the gyroscope angle rate using Euler's method to get the angle
	zangle = zangle + 0.01*data.gyro[0]*DEG_TO_RAD;
	
	// filter angle data
	double lp_z = march_filter(&lowpass, z_orientation_from_accel);
	double hp_z = march_filter(&highpass, zangle);
	
	// get most recent filtered value
	double lp_filtered_output_z = newest_filter_output(&lowpass);
	double hp_filtered_output_z = newest_filter_output(&highpass);
	
	//complementary filter to get theta
	sys_state.angle_about_z_axis = lp_filtered_output_z+hp_filtered_output_z + CAPE_MOUNT_ANGLE_Z;
	
	
	//check for various exit conditions AFTER state estimate
	
	if(get_state() == EXITING){
		disable_motors();
		return 0;
	}
	// if controller is still ARMED while state is PAUSED, disarm it
	if(get_state()!=RUNNING && reference_value.armstate==ARMED){
		disarm_controller();
		return 0;
	}
	// exit if the controller is disarmed
	if(reference_value.armstate==DISARMED){
		return 0;
	}
	
	/* //example of how to implement duty cycle
	//gain of difference equation, adjusted by battery voltage
	double K = -7 * V_NOMINAL/sys_state.battery_voltage;
	double u1n;

	//difference function for u1[n], D1(s) output is motor duty value
	u1n=1.997*u1_buf1-0.9965*u1_buf2+1*thetaR_buf0*K-2*0.99965*thetaR_buf1*K+0.99965*0.99965*thetaR_buf2*K;
	
	//normalize duty cycle - this part might be wrong, maybe if > vmax {v=vmax} and if < vmin {v=vmin}******
	u1n = u1n/V_NOMINAL;
	
	//buffer u1 values for use in difference functions
	u1_buf2 = u1_buf1;
	u1_buf1 = u1_buf0;
	u1_buf0 = u1n;

	//set u state to u1[n]
	sys_state.u1 = u1n;
		
	//Send signal to motors
	left_duty_cycle = sys_state.u1;
	right_duty_cycle = sys_state.u1;	
	set_motor(MOTOR_CHANNEL_L, MOTOR_POLARITY_L * left_duty_cycle); 
	set_motor(MOTOR_CHANNEL_R, MOTOR_POLARITY_R * right_duty_cycle); 
	*/
	return 0;
}

/*******************************************************************************
* int zero_out_controller()
*
* Sets all controller values to zero.
*******************************************************************************/
int zero_out_controller(){
	//set all difference equation buffers to zero
	u1_buf0 =0;
	u1_buf1=0;
	u1_buf2=0;

	thetaR_buf0=0;
	thetaR_buf1=0;
	thetaR_buf2=0;

	u2_buf0=0;
	u2_buf1=0;

	phiR_buf0=0;
	phiR_buf1=0;
	
	reference_value.theta = 0.0;
	reference_value.phi   = 0.0;
	set_motor_all(0);
	return 0;
}

//disable motors in case something happens
int disarm_controller(){
	disable_motors();
	reference_value.armstate = DISARMED;
	return 0;
}

//turn the encoders and motors on and initialize their values
int arm_controller(){
	zero_out_controller();
	set_encoder_pos(ENCODER_CHANNEL_L,0);
	set_encoder_pos(ENCODER_CHANNEL_R,0);
	reference_value.armstate = ARMED;
	enable_motors();
	return 0;
}



//thread to check battery voltage
void* battery_checker(void* ptr){
		float new_v;
		while(get_state()!=EXITING){
			new_v = get_battery_voltage();
			// if the value doesn't make sense, use nominal voltage
			if (new_v>9.0 || new_v<5.0) new_v = V_NOMINAL;
			sys_state.battery_voltage = new_v;
			usleep(1000000 / BATTERY_CHECK_HZ);
		}
		return NULL;
	}

/*******************************************************************************
* void* printf_loop(void* ptr)
*
* Thread to print information for debugging.
*******************************************************************************/
void* printf_loop(void* ptr){
	state_t last_state, new_state; // keep track of last state 
	while(get_state()!=EXITING){
		new_state = get_state();
		// check if this is the first time since being paused
		if(new_state==RUNNING && last_state!=RUNNING){
			printf("\nRUNNING\n"); //**************************change variables below and these **************************************
			printf("  Roll     |");
			printf("  Roll Reference     |");
			printf("  Pitch    |");
			printf("  Pitch Reference     |");
			printf("  Yaw      |");
			printf("  Yaw Reference     |");
			printf("  battery_voltage  |");
			printf("armstate|");
			printf("\n");
		}
		else if(new_state==PAUSED && last_state!=PAUSED){
			printf("\nPAUSED: press pause again to start.\n");
		}
		last_state = new_state;
		
		// decide what to print or exit
		if(new_state == RUNNING){	
			printf("\r");
			printf("%7.2f  |", sys_state.angle_about_x_axis);
			printf("%7.2f  |", reference_value.angle_about_x_axis_ref);

			printf("%7.2f  |", sys_state.angle_about_y_axis);
			printf("%7.2f  |", reference_value.angle_about_y_axis_ref);

			printf("%7.2f  |", sys_state.angle_about_y_axis);
			printf("%7.2f  |", reference_value.angle_about_y_axis_ref);

			
			printf("%7.2f  |", sys_state.battery_voltage);
			
			if(reference_value.armstate == ARMED) printf("  ARMED  |");
			else printf("DISARMED |");
			fflush(stdout);
		}
		usleep(1000000 / PRINTF_HZ);
	}
	return NULL;
}
/*******************************************************************************
* int wait_for_starting_condition()
*
* Check if in the right position before starting.
*******************************************************************************/
int wait_for_starting_condition(){
	int checks = 0;
	const int check_hz = 20;	// check 20 times per second
	int checks_needed = round(START_DELAY*check_hz);
	int wait_us = 1000000/check_hz; 

	// exit if state becomes paused or exiting
	while(get_state()==RUNNING){
		// if within range, start counting
		if(fabs(sys_state.theta) < START_ANGLE){
			checks++;
			// waited long enough, return
			if(checks >= checks_needed) return 0;
		}
		// fell out of range, restart counter
		else checks = 0;
		usleep(wait_us);
	}
	return -1;
}


/*******************************************************************************
* int on_pause_pressed()
*
* Quits if pause button is held.
*******************************************************************************/
int on_pause_pressed(){
	int i=0;
	const int samples = 100;	// check for release 100 times in this period
	const int us_wait = 1000000; // 1 second
	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		usleep(us_wait/samples);
		if(get_pause_button() == RELEASED) return 0;
	}
	printf("Shutting down...\n");
	set_state(EXITING);
	return 0;
}
int on_pause_released(){
	// toggle betewen paused and running modes
	if(get_state()==RUNNING)   		set_state(PAUSED);
	else if(get_state()==PAUSED)	set_state(RUNNING);
	return 0;
}


