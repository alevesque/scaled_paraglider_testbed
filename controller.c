#include <usefulincludes.h>
#include <roboticscape.h>
#include "config.h"

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
	float right_pulley_angle_ref;	// pulley rotation
	float left_pulley_angle_ref;
	float angle_about_x_axis_ref; //body angle about x axis
	float angle_about_y_axis_ref; //body angle about y axis
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
	float weightshift_dist;			//distance of weight from neutral position 
	float battery_voltage; 		// battery voltage 
	float u1;			// output of controller to weight shift
	float u2_left;			// output of controller to pulley motors
	float u2_right;
	int start_cond;
} core_state_t;

/*******************************************************************************
* orientation_t
*
* This is the system orientation as seen by accelerometer, gyroscope
*******************************************************************************/
typedef struct orientation_t{
	float x_gyro;
	float x_accel;
	float y_gyro;
	float y_accel;
} orientation_t;


/*******************************************************************************
* Local Function declarations	
*******************************************************************************/
// IMU interrupt routine
int control_tilt();

// threads
void* reference_value_manager(void* ptr);
void* battery_checker(void* ptr);
void* printf_loop(void* ptr);
void* read_input(void* ptr);

// regular functions
int is_flying();
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
rc_imu_data_t data;
orientation_t orientation;

//create filters
rc_filter_t lowpass_x;
rc_filter_t highpass_x;
rc_filter_t lowpass_y;
rc_filter_t highpass_y;

/*******************************************************************************
* main()
*
* Initialize the filters, IMU, threads, & wait until shut down
*******************************************************************************/
int main(){
	rc_set_cpu_freq(FREQ_1000MHZ);

	if(rc_initialize()<0){
		printf("ERROR: failed to initialize cape!\n");
		return -1;
	}
	rc_set_state(UNINITIALIZED);


	// make sure reference_value starts at normal values
	reference_value.armstate = DISARMED;
		
	//timestep, dt
	double const dt = 1.0/(float)SAMPLE_RATE_HZ;
	//rise time, tr **********************************************find new rise time if needed, prob need for high pass cuz noisy************************
	double const tr = 1;
	//time constant from rise time
	float const tau = tr/2.2;

	//set up filters for finding theta from sensors
	lowpass_x  = rc_first_order_lowpass(dt, tau);
	highpass_x  = rc_first_order_highpass(dt, tau);
	lowpass_y  = rc_first_order_lowpass(dt, tau);
	highpass_y  = rc_first_order_highpass(dt, tau);

	// set up button handlers
	rc_set_pause_pressed_func(&on_pause_pressed);
	rc_set_mode_released_func(&on_pause_released);
	
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
	
	//get orientation of beaglebone
	conf.orientation = ORIENTATION_Z_UP;
	
	// start imu
	if(initialize_imu_dmp(&data, conf)){
		printf("ERROR: IMU initialization failed!\n");
		return -1;
	}
	
	// start stack to control reference_values
	pthread_t reference_value_thread;
	pthread_create(&reference_value_thread, NULL, reference_value_manager, (void*) NULL);

	//start thread for reading user input
	pthread_t read_input_thread;
	pthread_create(&read_input_thread, NULL, read_input, (void*) NULL);
	
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
	destroy_filter(&lowpass_x);
	destroy_filter(&highpass_x);
	destroy_filter(&lowpass_y);
	destroy_filter(&highpass_y);
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
		// necessarily armed. If DISARMED, wait for the user to send start signal
		// which will we detected by is_flying()
		if(reference_value.armstate == DISARMED){
			if(is_flying()==0){
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

	//brake line motor duty cycles
	float left_pulley_duty_cycle, right_pulley_duty_cycle;
	//weight shift motor duty cycle
	float ws_duty;
	
	// find angle from accelerometer
	orientation.x_accel = atan2(-1*data.accel[1],sqrt(pow(data.accel[2],2) + pow(data.accel[0],2)))*RAD_TO_DEG;

	// integrates the gyroscope angle rate using Euler's method to get the angle
	orientation.x_gyro = sys_state.angle_about_x_axis + 0.01*data.gyro[0];
	
	// filter angle data
	double lp_x = march_filter(&lowpass_x, orientation.x_accel);
	double hp_x = march_filter(&highpass_x, orientation.x_gyro);
	
	// get most recent filtered value
	double lp_filtered_output_x = newest_filter_output(&lowpass_x);
	double hp_filtered_output_x = newest_filter_output(&highpass_x);
	
	//complementary filter to get theta
	sys_state.angle_about_x_axis = (lp_filtered_output_x+hp_filtered_output_x + CAPE_MOUNT_ANGLE_X); //(0.98*orientation.x_gyro+0.02*orientation.x_accel);
	
	

	
	// find angle from accelerometer
	orientation.y_accel = atan2(-1*data.accel[0],sqrt(pow(data.accel[2],2) + pow(data.accel[1],2)))*RAD_TO_DEG;

	// integrates the gyroscope angle rate using Euler's method to get the angle
	orientation.y_gyro = sys_state.angle_about_y_axis + 0.01*data.gyro[1];
	
	// filter angle data
	double lp_y = march_filter(&lowpass_y, orientation.y_accel);
	double hp_y = march_filter(&highpass_y, orientation.y_gyro);
	
	// get most recent filtered value
	double lp_filtered_output_y = newest_filter_output(&lowpass_y);
	double hp_filtered_output_y = newest_filter_output(&highpass_y);
	
	//complementary filter to get theta
	sys_state.angle_about_y_axis = (lp_filtered_output_y+hp_filtered_output_y + CAPE_MOUNT_ANGLE_Y);
	


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
	/*
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
	*/
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
			printf("\nRUNNING\n");
			printf("  Pitch  |");
			printf("  Pitch Reference  |");
			printf("  Roll  |");
			printf("  Roll Reference  |");
			printf("  Battery Voltage  |");
			printf("  Armstate  |");
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
			printf("%17.2f  |", reference_value.angle_about_x_axis_ref);

			printf("%6.2f  |", sys_state.angle_about_y_axis);
			printf("%16.2f  |", reference_value.angle_about_y_axis_ref);
			
			printf("%17.2f  |", sys_state.battery_voltage);
			
			printf("%10.2d  |", reference_value.armstate);
			fflush(stdout);
		}
		usleep(1000000 / PRINTF_HZ);
	}
	return NULL;
}

void* read_input(void* ptr){
	while(get_state()!=EXITING){
		sys_state.start_cond=1; //temporary until implement user input

		usleep(1000000 / GET_INPUT_HZ);
	}
	return NULL;
}

/*******************************************************************************
* int is_flying()
*
* Checks if it should start controlling the motors
*******************************************************************************/
int is_flying(){
	const int check_hz = 20;
	int wait_us = 1000000/check_hz;
	if(sys_state.start_cond == 1){
		return 0;
	}
	else{
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
