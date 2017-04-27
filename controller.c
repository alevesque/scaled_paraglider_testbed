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
* controller_arming_t
*	
* Controller arming state written to by controller_arming_manager and read by the controller.
*******************************************************************************/
typedef struct controller_arming_t{ //setpoints
	armstate_t armstate;	// see armstate_t declaration
} controller_arming_t;

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
	float weightshift_dist;			//distance of weight from neutral position -- need to know how to read, wait on physical implementation
	float battery_voltage; 		// battery voltage 
	float WS_duty_signal;			// output of controller to weight shift
	float BL_duty_signal_left;			// output of controller to pulley motors
	float BL_duty_signal_right;
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
int collect_data();

// threads
void* controller_arming_manager(void* ptr);
void* battery_checker(void* ptr);
void* printf_loop(void* ptr);
void* read_input(void* ptr);
void* check_radio_signal(void* ptr);

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
controller_arming_t controller_arming;
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
* Initialize the filters, IMU, DSM, threads, & wait until shut down
*******************************************************************************/
int main(){
	rc_set_cpu_freq(FREQ_1000MHZ);

	if(rc_initialize()<0){
		printf("ERROR: failed to initialize cape!\n");
		return -1;
	}
	rc_set_state(UNINITIALIZED);

	//initialize radio receiver
	rc_initialize_dsm();

	// make sure controller_arming starts at normal values
	controller_arming.armstate = DISARMED;
		

	/******************************************
	* Set up filters for data gathering.
	******************************************/	
	//timestep, dt
	double const dt = 1.0/(float)SAMPLE_RATE_HZ;
	//rise time, tr 
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
	
	
	//can use gps header for gpio if needed, where WS_MOTOR_CHANNEL is the gpio pin #
	//rc_gpio_set_dir(WS_MOTOR_CHANNEL, OUTPUT_PIN);
	//rc_set_pinmux_mode(WS_MOTOR_CHANNEL, PINMUX_PWM);

	


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
	
	// start thread to control arming/disarming controller.
	pthread_t controller_arming_thread;
	pthread_create(&controller_arming_thread, NULL, controller_arming_manager, (void*) NULL);

	//start thread for reading user input
	pthread_t read_input_thread;
	pthread_create(&read_input_thread, NULL, read_input, (void*) NULL);
	
	//thread for ensuring transmitter signal is being received
	pthread_t check_radio_signal_thread
	pthread_create(&check_radio_signal_thread, NULL, check_radio_signal, (void*) NULL);

	//set imu interrupt function
	rc_set_imu_interrupt_func(&collect_data);
	rc_set_new_dsm_data_func(&read_input);
		
	rc_set_state(RUNNING);
	rc_set_led(RED,0);
	rc_set_led(GREEN,1);
	
	//wait while stuff is going on
	while(rc_get_state()!=EXITING){
		rc_usleep(10000);
	}
	
	//cleanup memory
	rc_free_filter(&lowpass_x);
	rc_free_filter(&highpass_x);
	rc_free_filter(&lowpass_y);
	rc_free_filter(&highpass_y);
	rc_stop_dsm_service();
	rc_power_off_imu();
	rc_set_cpu_freq(FREQ_ONDEMAND);
	rc_cleanup();
	
	return 0;
}

/*******************************************************************************
void* check_radio_signal(void* ptr)
*
* Detects if radio signal is being received by Beaglebone and if not, do something.
*******************************************************************************/
void* check_radio_signal(void* ptr){
		
	while(rc_get_state()!=EXITING){
		if(!rc_is_dsm_active()){ //if not active
			//set motors to neutral or to circle or to RTB until signal is received
		}	
		rc_usleep(1000000/CHECK_RADIO_SIGNAL_HZ); 
	}
	return NULL;
}

/*******************************************************************************
* void* controller_arming_manager(void* ptr)
*
* Detects start conditions to control arming the controller.
*******************************************************************************/
void* controller_arming_manager(void* ptr){
	// wait for IMU to settle
	disarm_controller();
	rc_usleep(2500000);
		
	while(rc_get_state()!=EXITING){
		// sleep at beginning of loop so we can use the 'continue' statement
		rc_usleep(1000000/CONTROLLER_ARMING_MANAGER_HZ); 
		
		// nothing to do if paused, go back to beginning of loop
		if(rc_get_state() != RUNNING) continue;

		// if we got here the state is RUNNING, but controller is not
		// necessarily armed. If DISARMED, wait for the user to send start signal
		// which will we detected by is_flying()
		if(controller_arming.armstate == DISARMED){
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
* int collect_data()
*
* Finds orientation from sensors.
*******************************************************************************/
int collect_data(){

	// find angle from accelerometer
	orientation.x_accel = atan2(-1*data.accel[1],sqrt(pow(data.accel[2],2) + pow(data.accel[0],2)))*RAD_TO_DEG;

	// integrates the gyroscope angle rate using Euler's method to get the angle
	orientation.x_gyro = sys_state.angle_about_x_axis + 0.01*data.gyro[0];
	
	// filter angle data
	double lp_x = rc_march_filter(&lowpass_x, orientation.x_accel);
	double hp_x = rc_march_filter(&highpass_x, orientation.x_gyro);
	
	// get most recent filtered value
	double lp_filtered_output_x = rc_newest_filter_output(&lowpass_x);
	double hp_filtered_output_x = rc_newest_filter_output(&highpass_x);
	
	//complementary filter to get theta
	sys_state.angle_about_x_axis = (lp_filtered_output_x+hp_filtered_output_x + CAPE_MOUNT_ANGLE_X); //(0.98*orientation.x_gyro+0.02*orientation.x_accel);
	
	

	
	// find angle from accelerometer
	orientation.y_accel = atan2(-1*data.accel[0],sqrt(pow(data.accel[2],2) + pow(data.accel[1],2)))*RAD_TO_DEG;

	// integrates the gyroscope angle rate using Euler's method to get the angle
	orientation.y_gyro = sys_state.angle_about_y_axis + 0.01*data.gyro[1];
	
	// filter angle data
	double lp_y = rc_march_filter(&lowpass_y, orientation.y_accel);
	double hp_y = rc_march_filter(&highpass_y, orientation.y_gyro);
	
	// get most recent filtered value
	double lp_filtered_output_y = rc_newest_filter_output(&lowpass_y);
	double hp_filtered_output_y = rc_newest_filter_output(&highpass_y);
	
	//complementary filter to get theta
	sys_state.angle_about_y_axis = (lp_filtered_output_y+hp_filtered_output_y + CAPE_MOUNT_ANGLE_Y);
	


	//check for various exit conditions AFTER state estimate
	
	if(rc_get_state() == EXITING){
		//rc_disable_servo_power_rail()
		rc_disable_motors();
		return 0;
	}
	// if controller is still ARMED while state is PAUSED, disarm it
	if(rc_get_state()!=RUNNING && controller_arming.armstate==ARMED){
		disarm_controller();
		return 0;
	}
	// exit if the controller is disarmed
	if(controller_arming.armstate==DISARMED){
		return 0;
	}
	
	return 0;
}

/*******************************************************************************
* int zero_out_controller()
*
* Sets all controller values to zero.
*******************************************************************************/
int zero_out_controller(){
	rc_set_motor_all(0);
	return 0;
}

/*******************************************************************************
* int arm_controller()
*
* Turns the motors on and initialize their values.
*******************************************************************************/
int arm_controller(){
	zero_out_controller();
	controller_arming.armstate = ARMED;
	rc_enable_motors();

	//can also use servo rails if needed
	// rc_enable_servo_power_rail();
	return 0;
}

/*******************************************************************************
* int disarm_controller()
*
* Disable motors in case something happens.
*******************************************************************************/
int disarm_controller(){
	rc_disable_motors();
	controller_arming.armstate = DISARMED;
	return 0;
}

/*******************************************************************************
* void* battery_checker(void* ptr)
*
* Checks battery voltage to ensure it's normal.
*******************************************************************************/
void* battery_checker(void* ptr){
		float new_v;
		while(rc_get_state()!=EXITING){
			new_v = rc_battery_voltage();
			// if the value doesn't make sense, use nominal voltage
			if (new_v>9.0 || new_v<5.0) new_v = V_NOMINAL;
			sys_state.battery_voltage = new_v;
			rc_usleep(1000000 / BATTERY_CHECK_HZ);
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
	while(rc_get_state()!=EXITING){
		new_state = rc_get_state();
		// check if this is the first time since being paused
		if(new_state==RUNNING && last_state!=RUNNING){
			printf("\nRUNNING\n");
			printf("  Pitch  |");
			printf("  Roll  |");
			printf("  WS Duty  |");
			printf("  BL Duty L  |");
			printf("  BL Duty R  |");
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
			printf("%6.2f  |", sys_state.angle_about_y_axis);
			printf("%8.2f  |", sys_state.WS_duty_signal);
			printf("%11.2f  |", sys_state.BL_duty_signal_left);
			printf("%11.2f  |", sys_state.BL_duty_signal_right);
			printf("%17.2f  |", sys_state.battery_voltage);
			printf("%10.2d  |", controller_arming.armstate);
			fflush(stdout);
		}
		rc_usleep(1000000 / PRINTF_HZ);
	}
	return NULL;
}

/*******************************************************************************
* void* read_input(void* ptr)
*
* Reads user input from radio transmitter. 
*******************************************************************************/
void* read_input(void* ptr){
	while(rc_get_state()!=EXITING){

		//For the normalized data to be accurate, you must run the rc_calibrate_dsm2 example program!!!
		//normalized reading between -1,1 depending on how far switch was moved
		sys_state.WS_duty_signal = rc_get_dsm_ch_normalized(WS_RADIO_CHANNEL);
		sys_state.BL_duty_signal_left = rc_get_dsm_ch_normalized(BL_RADIO_CHANNEL_L);
		sys_state.BL_duty_signal_right = rc_get_dsm_ch_normalized(BL_RADIO_CHANNEL_R);
		
		//signal PWM duty to motor driver
		rc_set_motor(WS_MOTOR_CHANNEL, sys_state.WS_duty_signal); 
		rc_set_motor(BL_MOTOR_CHANNEL_L, BL_MOTOR_POLARITY_L * sys_state.BL_duty_signal_left); 
		rc_set_motor(BL_MOTOR_CHANNEL_R, BL_MOTOR_POLARITY_R * sys_state.BL_duty_signal_right); 

		//can use servo rails maybe
		//need to map duty cycle to us pulse width (NOT MODULATED BY FREQ, different from PWM)
		//rc_send_servo_pulse_us(WS_MOTOR_CHANNEL, int us)

		//or gps headers as gpio
		//create thread with freq of pwm? where toggles high/low
		//rc_gpio_set_value(WS_MOTOR_CHANNEL, HIGH);

		rc_usleep(1000000 / READ_INPUT_HZ);
	}
	return NULL;
}

/*******************************************************************************
* int is_flying()
*
* Checks if it should start controlling the motors yet.
*******************************************************************************/
int is_flying(){
	const int check_hz = 20;
	int wait_us = 1000000/check_hz;
	if(rc_is_dsm_active()){
		return 0;
	}
	else{
		rc_usleep(wait_us);
	}
	return -1;
}

/*******************************************************************************
* int on_pause_pressed()
*
* Quits if pause button is held. Mainly for debugging.
*******************************************************************************/
int on_pause_pressed(){
	int i=0;
	const int samples = 100;	// check for release 100 times in this period
	const int us_wait = 1000000; // 1 second
	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		rc_usleep(us_wait/samples);
		if(rc_get_pause_button() == RELEASED) return 0;
	}
	printf("Shutting down...\n");
	rc_set_state(EXITING);
	return 0;
}

/*******************************************************************************
* int on_pause_released()
*
* Pauses if pause button is pressed. Mainly for debugging.
*******************************************************************************/
int on_pause_released(){
	// toggle betewen paused and running modes
	if(rc_get_state()==RUNNING)   		rc_set_state(PAUSED);
	else if(rc_get_state()==PAUSED)	rc_set_state(RUNNING);
	return 0;
}
