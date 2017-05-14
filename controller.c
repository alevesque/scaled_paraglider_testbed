#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include <libconfig.h>

/*******************************************************************************
* cfg_settings_t
*
* Holds values of settings read from cfg file.
*******************************************************************************/
typedef struct cfg_settings_t{
	
	int SAMPLE_RATE_HZ;

	// Structural properties of BBB
	int STEPS_PER_WS_ANGLE_DEGREE;
	float CAPE_MOUNT_ANGLE_X;
	float CAPE_MOUNT_ANGLE_Y;
	float CAPE_MOUNT_ANGLE_Z;
	int V_NOMINAL;

	// electrical hookups
	int WS_MOTOR_CHANNEL;
	int WS_MOTOR_DIR_PIN;
	/*
	int BL_MOTOR_CHANNEL_L;
	int BL_MOTOR_DIR_PIN_L;
	int BL_MOTOR_CHANNEL_R;
	int BL_MOTOR_DIR_PIN_R;
	int BL_MOTOR_POLARITY_L;
	int BL_MOTOR_POLARITY_R;
	*/

	// Thread Loop Rates
	int	BATTERY_CHECK_HZ;
	int CONTROLLER_ARMING_MANAGER_HZ;
	int	PRINTF_HZ;
	int	READ_INPUT_HZ;

	// PID Parameters
	float K_P;
	float K_I;
	float K_D;

} cfg_settings_t;

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
	int motor_on;
} controller_arming_t;


/*******************************************************************************
* controller_state_t
*
* This is the system state written to by the controller.	
*******************************************************************************/
typedef struct controller_state_t{
		//setup controller values
		int steps;
		int error;
		int last_error;
		int derivative;
		int integral;
} controller_state_t;

/*******************************************************************************
* core_state_t
*
* This is the physical system state.
*******************************************************************************/
typedef struct core_state_t{
	//float right_pulley_angle;	// pulley rotation
	//float left_pulley_angle;
	float angle_about_x_axis; 		// body angle radians
	float angle_about_y_axis; 		// body angle radians
	//float weightshift_dist;			//distance of weight from neutral position -- need to know how to read, wait on physical implementation
	float battery_voltage; 		// battery voltage 
	float WS_angle_setpoint;			// output of controller to weight shift
	//float BL_duty_signal_left;			// output of controller to pulley motors
	//float BL_duty_signal_right;
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
void collect_data();

// threads
void* controller_arming_manager(void* ptr);
void* battery_checker(void* ptr);
void* printf_loop(void* ptr);
void* read_input(void* ptr);


// regular functions
int print_usage();
int motor_output();
int zero_out_controller();
int disarm_controller();
int arm_controller();
int cleanup_everything();
int get_config_settings();
void on_pause_pressed();
void on_pause_released();


/*******************************************************************************
* Global Variables				
*******************************************************************************/
core_state_t sys_state;
controller_arming_t controller_arming;
rc_imu_data_t data;
orientation_t orientation;
controller_state_t controller_state;
config_t cfg;
cfg_settings_t cfg_setting;
/*
rc_filter_t lowpass_x_filt;
rc_filter_t lowpass_y_filt;
rc_filter_t highpass_x_filt;
rc_filter_t highpass_y_filt;
*/
pthread_t battery_thread;
pthread_t controller_arming_thread;
pthread_t read_input_thread;
pthread_t  printf_thread;

/*******************************************************************************
* main()
*
* Initialize the filters, IMU, DSM, threads, & wait until shut down
*******************************************************************************/
int main(){

	/*****************************************
	* Begin initializing
	*****************************************/
	//rc_set_cpu_freq(FREQ_1000MHZ);

	if(rc_initialize()<0){
		printf("ERROR: failed to initialize cape!\n");
		return -1;
	}
	rc_set_state(UNINITIALIZED);

	rc_imu_config_t conf = rc_default_imu_config();

	// make sure controller_arming starts at normal values
	controller_arming.armstate = DISARMED;


	//get configuration settings
	get_config_settings();
	/****************************************/



	/******************************************
	* Set up filters for data gathering.
	******************************************/	
	//create empty filters
	/*
	lowpass_x_filt  = rc_empty_filter();
	highpass_x_filt  = rc_empty_filter();
	lowpass_y_filt  = rc_empty_filter();
	highpass_y_filt  = rc_empty_filter();
	
	//timestep, dt
	double const dt = 1.0/(float)cfg_setting.SAMPLE_RATE_HZ;
	//rise time, tr 
	double const tr = 1;
	//time constant from rise time
	float const tau = tr/2.2;

	//set up filters for finding theta from sensors
	rc_first_order_lowpass(&lowpass_x_filt, dt, tau);
	rc_first_order_highpass(&highpass_x_filt, dt, tau);
	rc_first_order_lowpass(&lowpass_y_filt, dt, tau);
	rc_first_order_highpass(&highpass_y_filt, dt, tau);
	*/
	/***********************/



	// set up button handlers
	rc_set_pause_pressed_func(&on_pause_pressed);
	rc_set_mode_released_func(&on_pause_released);
	
	
	//can use gps header for gpio if needed, where WS_MOTOR_CHANNEL is the gpio pin #
	//rc_gpio_set_dir(WS_MOTOR_CHANNEL, OUTPUT_PIN);
	//rc_set_pinmux_mode(WS_MOTOR_CHANNEL, PINMUX_PWM);

	


	// start a thread to slowly sample battery 
	pthread_create(&battery_thread, NULL, battery_checker, (void*) NULL);
	// wait for the battery thread to make the first read
	while(sys_state.battery_voltage==0 && rc_get_state()!=EXITING) usleep(1000);
	
	
	
	// set up IMU configuration
	conf.dmp_sample_rate = cfg_setting.SAMPLE_RATE_HZ;
	
	//set orientation of beaglebone
	conf.orientation = ORIENTATION_Z_UP;
	
	// start imu
	if(rc_initialize_imu_dmp(&data, conf)){
		printf("ERROR: IMU initialization failed!\n");
		return -1;
	}


	// start thread to control arming/disarming controller.
	pthread_create(&controller_arming_thread, NULL, controller_arming_manager, (void*) NULL);


	//start thread for reading user input
	pthread_create(&read_input_thread, NULL, read_input, (void*) NULL);
	
	//set imu interrupt function
	rc_set_imu_interrupt_func(&collect_data);
		
	rc_set_state(RUNNING);
	rc_set_led(RED,0);
	rc_set_led(GREEN,1);
	
	printf("@-----------------------------@\n");
	printf("| Paraglider Control Software |\n");
	printf("@-----------------------------@\n\n");
	//wait while stuff is going on
	while(rc_get_state()!=EXITING){
		rc_usleep(10000);
	}
	


	//now exiting
	cleanup_everything();
	pthread_cancel(read_input_thread);
	return 0;
}


/*******************************************************************************
* int get_settings()
*
* Inputs config settings from config text file.
*******************************************************************************/
int get_config_settings(){
	int cfg_value_int;
	double cfg_value_float;
	config_init(&cfg);
	/* Read the file. If there is an error, report it and exit. */
 	if(!config_read_file(&cfg, "paraglider_config.cfg"))
  	{
	    fprintf(stderr, "%s:%d - %s\n", config_error_file(&cfg),
	    config_error_line(&cfg), config_error_text(&cfg));
	    config_destroy(&cfg);
	    return(EXIT_FAILURE);
	}

	if(config_lookup_int(&cfg, "SAMPLE_RATE_HZ", &cfg_value_int)){
    	cfg_setting.SAMPLE_RATE_HZ = cfg_value_int;
	}
  	else{
		fprintf(stderr, "No 'SAMPLE_RATE_HZ' setting in configuration file.\n");
  	}
  	if(config_lookup_int(&cfg, "STEPS_PER_WS_ANGLE_DEGREE", &cfg_value_int)){
    	cfg_setting.STEPS_PER_WS_ANGLE_DEGREE = cfg_value_int;
	}
  	else{
		fprintf(stderr, "No 'STEPS_PER_WS_ANGLE_DEGREE' setting in configuration file.\n");
  	}
  	if(config_lookup_float(&cfg, "CAPE_MOUNT_ANGLE_X", &cfg_value_float)){
    	cfg_setting.CAPE_MOUNT_ANGLE_X = cfg_value_float;
	}
  	else{
		fprintf(stderr, "No 'CAPE_MOUNT_ANGLE_X' setting in configuration file.\n");
  	}
  	if(config_lookup_float(&cfg, "CAPE_MOUNT_ANGLE_Y", &cfg_value_float)){
    	cfg_setting.CAPE_MOUNT_ANGLE_Y = cfg_value_float;
	}
  	else{
		fprintf(stderr, "No 'CAPE_MOUNT_ANGLE_Y' setting in configuration file.\n");
  	}
  	if(config_lookup_float(&cfg, "CAPE_MOUNT_ANGLE_Z", &cfg_value_float)){
    	cfg_setting.CAPE_MOUNT_ANGLE_Z = cfg_value_float;
	}
  	else{
		fprintf(stderr, "No 'CAPE_MOUNT_ANGLE_Z' setting in configuration file.\n");
  	}
  	if(config_lookup_float(&cfg, "V_NOMINAL", &cfg_value_float)){
    	cfg_setting.V_NOMINAL = cfg_value_float;
	}
  	else{
		fprintf(stderr, "No 'V_NOMINAL' setting in configuration file.\n");
  	}
  	if(config_lookup_int(&cfg, "WS_MOTOR_CHANNEL", &cfg_value_int)){
    	cfg_setting.WS_MOTOR_CHANNEL = cfg_value_int;
	}
  	else{
		fprintf(stderr, "No 'WS_MOTOR_CHANNEL' setting in configuration file.\n");
  	}
  	if(config_lookup_int(&cfg, "WS_MOTOR_DIR_PIN", &cfg_value_int)){
    	cfg_setting.WS_MOTOR_DIR_PIN = cfg_value_int;
	}
  	else{
		fprintf(stderr, "No 'WS_MOTOR_DIR_PIN' setting in configuration file.\n");
  	}
  	/*
  	if(config_lookup_int(&cfg, "BL_MOTOR_CHANNEL_L", &cfg_value_int)){
    	cfg_setting.BL_MOTOR_CHANNEL_L = cfg_value_int;
	}
	else{
		fprintf(stderr, "No 'BL_MOTOR_CHANNEL_L' setting in configuration file.\n");
  	}
  	if(config_lookup_int(&cfg, "BL_MOTOR_DIR_PIN_L", &cfg_value_int)){
    	cfg_setting.BL_MOTOR_DIR_PIN_L = cfg_value_int;
	}
  	else{
		fprintf(stderr, "No 'BL_MOTOR_DIR_PIN_L' setting in configuration file.\n");
  	}
  	if(config_lookup_int(&cfg, "BL_MOTOR_CHANNEL_R", &cfg_value_int)){
    	cfg_setting.BL_MOTOR_CHANNEL_R = cfg_value_int;
	}
  	else{
		fprintf(stderr, "No 'BL_MOTOR_CHANNEL_R' setting in configuration file.\n");
  	}
  	if(config_lookup_int(&cfg, "BL_MOTOR_DIR_PIN_R", &cfg_value_int)){
    	cfg_setting.BL_MOTOR_DIR_PIN_R = cfg_value_int;
	}
  	else{
		fprintf(stderr, "No 'BL_MOTOR_DIR_PIN_R' setting in configuration file.\n");
  	}
  	if(config_lookup_int(&cfg, "BL_MOTOR_POLARITY_L", &cfg_value_int)){
    	cfg_setting.BL_MOTOR_POLARITY_L = cfg_value_int;
	}
  	else{
		fprintf(stderr, "No 'BL_MOTOR_POLARITY_L' setting in configuration file.\n");
  	}
  	if(config_lookup_int(&cfg, "BL_MOTOR_POLARITY_R", &cfg_value_int)){
    	cfg_setting.BL_MOTOR_POLARITY_R = cfg_value_int;
	}
  	else{
		fprintf(stderr, "No 'BL_MOTOR_POLARITY_R' setting in configuration file.\n");
  	}
  	*/
  	if(config_lookup_int(&cfg, "BATTERY_CHECK_HZ", &cfg_value_int)){
    	cfg_setting.BATTERY_CHECK_HZ = cfg_value_int;
	}
  	else{
		fprintf(stderr, "No 'BATTERY_CHECK_HZ' setting in configuration file.\n");
  	}
  	if(config_lookup_int(&cfg, "CONTROLLER_ARMING_MANAGER_HZ", &cfg_value_int)){
    	cfg_setting.CONTROLLER_ARMING_MANAGER_HZ = cfg_value_int;
	}
  	else{
		fprintf(stderr, "No 'CONTROLLER_ARMING_MANAGER_HZ' setting in configuration file.\n");
  	}
  	if(config_lookup_int(&cfg, "PRINTF_HZ", &cfg_value_int)){
    	cfg_setting.PRINTF_HZ = cfg_value_int;
	}
  	else{
		fprintf(stderr, "No 'PRINTF_HZ' setting in configuration file.\n");
  	}
  	if(config_lookup_int(&cfg, "READ_INPUT_HZ", &cfg_value_int)){
    	cfg_setting.READ_INPUT_HZ = cfg_value_int;
	}
  	else{
		fprintf(stderr, "No 'READ_INPUT_HZ' setting in configuration file.\n");
  	}
  	if(config_lookup_float(&cfg, "K_P", &cfg_value_float)){
    	cfg_setting.K_P = cfg_value_float;
	}
  	else{
		fprintf(stderr, "No 'K_P' setting in configuration file.\n");
  	}
  	if(config_lookup_float(&cfg, "K_I", &cfg_value_float)){
    	cfg_setting.K_I = cfg_value_float;
	}
  	else{
		fprintf(stderr, "No 'K_I' setting in configuration file.\n");
  	}
  	if(config_lookup_float(&cfg, "K_D", &cfg_value_float)){
    	cfg_setting.K_D = cfg_value_float;
	}
  	else{
		fprintf(stderr, "No 'K_D' setting in configuration file.\n");
  	}

  	
	return 0;
}


/*******************************************************************************
* void* controller_arming_manager(void* ptr)
*
* Detects start conditions to control arming the controller.
*******************************************************************************/
int cleanup_everything(){
	//cleanup memory
	/*
	rc_free_filter(&lowpass_x_filt);
	rc_free_filter(&highpass_x_filt);
	rc_free_filter(&lowpass_y_filt);
	rc_free_filter(&highpass_y_filt);
	*/
	rc_power_off_imu();
	config_destroy(&cfg);
	//rc_set_cpu_freq(FREQ_ONDEMAND);
	pthread_cancel(controller_arming_thread);
	pthread_cancel(battery_thread);
	pthread_cancel(printf_thread);

	rc_cleanup();
	return 0;
}

/*******************************************************************************
* void* controller_arming_manager(void* ptr)
*
* Detects start conditions to control arming the controller.
*******************************************************************************/
void* controller_arming_manager(void* ptr){
	
	disarm_controller();
	rc_usleep(2500000);
		
	while(rc_get_state()!=EXITING){
		// sleep at beginning of loop so we can use the 'continue' statement
		rc_usleep(1000000/cfg_setting.CONTROLLER_ARMING_MANAGER_HZ); 
		
		// nothing to do if paused, go back to beginning of loop
		if(rc_get_state() != RUNNING) continue;

		// if we got here the state is RUNNING, but controller is not
		// necessarily armed. If DISARMED, wait for the user to send start signal
		// which will we detected by is_flying()
		if(controller_arming.armstate == DISARMED){
				zero_out_controller();
				arm_controller(); 
			}
			else continue;		
	}

	// if state becomes EXITING the above loop exists and we disarm here
	disarm_controller();
	return NULL;
}

/*******************************************************************************
* void collect_data()
*
* Finds orientation from sensors.
*******************************************************************************/
void collect_data(){



	/*****************************************
	* Find Pitch Data
	*****************************************/
	// find angle from accelerometer
	orientation.x_accel = atan2(-1*data.accel[1],sqrt(pow(data.accel[2],2) + pow(data.accel[0],2)))*RAD_TO_DEG;

	// integrates the gyroscope angle rate using Euler's method to get the angle
	orientation.x_gyro = sys_state.angle_about_x_axis + 0.01*data.gyro[0];
	
	/*
	// filter angle data
	double lp_filtered_output_x = rc_march_filter(&lowpass_x_filt, orientation.x_accel);
	double hp_filtered_output_x = rc_march_filter(&highpass_x_filt, orientation.x_gyro);
	
	//complementary filter to get pitch angle
	//sys_state.angle_about_x_axis = (lp_filtered_output_x+hp_filtered_output_x + cfg_setting.CAPE_MOUNT_ANGLE_X); 
	*/
	sys_state.angle_about_x_axis = (0.9*orientation.x_gyro+0.1*orientation.x_accel);
	/*****************************************/
	


	/*****************************************
	* Find Roll Data
	*****************************************/
	// find angle from accelerometer
	orientation.y_accel = atan2(-1*data.accel[0],sqrt(pow(data.accel[2],2) + pow(data.accel[1],2)))*RAD_TO_DEG;

	// integrates the gyroscope angle rate using Euler's method to get the angle
	orientation.y_gyro = sys_state.angle_about_y_axis + 0.01*data.gyro[1];
	
	/*
	// filter angle data
	double lp_filtered_output_y= rc_march_filter(&lowpass_y_filt, orientation.y_accel);
	double hp_filtered_output_y = rc_march_filter(&highpass_y_filt, orientation.y_gyro);
	
	// get most recent filtered value
	//double lp_filtered_output_y = rc_newest_filter_output(&lowpass_y_filt);
	//double hp_filtered_output_y = rc_newest_filter_output(&highpass_y_filt);
	
	//complementary filter to get pitch angle
	sys_state.angle_about_y_axis = (lp_filtered_output_y+hp_filtered_output_y + cfg_setting.CAPE_MOUNT_ANGLE_Y);
	*/
	sys_state.angle_about_y_axis = (0.9*orientation.y_gyro+0.1*orientation.y_accel);
	/****************************************/



	//check for various exit conditions AFTER state estimate
	
	if(rc_get_state() == EXITING){
		//rc_disable_servo_power_rail()
		rc_disable_motors();
		return;
	}
	// if controller is still ARMED while state is PAUSED, disarm it
	if(rc_get_state()!=RUNNING && controller_arming.armstate==ARMED){
		disarm_controller();
		return;
	}
	// exit if the controller is disarmed
	if(controller_arming.armstate==DISARMED){
		return;
	}
	
	return;
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
			if (new_v>9.0 || new_v<5.0) new_v = cfg_setting.V_NOMINAL;
			sys_state.battery_voltage = new_v;
			rc_usleep(1000000 / cfg_setting.BATTERY_CHECK_HZ);
		}
		return NULL;
	}

/*******************************************************************************
* int motor_output()
*
* Outputs duty cycle to motors based on number of PID control of # of steps.
*******************************************************************************/
int motor_output(){
	if (controller_arming.motor_on == 1){
	
		int i;
		//find error between current orientation and setpoint
		controller_state.error = sys_state.WS_angle_setpoint - sys_state.angle_about_y_axis;
		
		//PID control while error too big
		while(abs(controller_state.error) > 0.1){
			//set motor direction based on sign of error signal
			if(controller_state.error<0){
				rc_gpio_set_value_mmap(cfg_setting.WS_MOTOR_DIR_PIN,LOW);
			}
			else{
				rc_gpio_set_value_mmap(cfg_setting.WS_MOTOR_DIR_PIN,HIGH);
			}

			//pulses to motor based on number of steps
			for(i=0;i<controller_state.steps;i++){ //maybe need <=
				rc_gpio_set_value_mmap(cfg_setting.WS_MOTOR_CHANNEL,HIGH); //pulse on
				rc_usleep(150); //pulse width
				rc_gpio_set_value_mmap(cfg_setting.WS_MOTOR_CHANNEL,LOW); //pulse off
				rc_usleep(400); //wait between pulses
			}
			//PID
			controller_state.last_error = controller_state.error;
			controller_state.error = sys_state.WS_angle_setpoint - sys_state.angle_about_y_axis;
			controller_state.derivative = controller_state.error - controller_state.last_error;
			controller_state.integral = controller_state.integral + controller_state.error;
			controller_state.steps = (cfg_setting.K_P*controller_state.error) + (cfg_setting.K_I*controller_state.integral) + (cfg_setting.K_D*controller_state.derivative);
		}
		
	}
	else{
		return -1;
		printf("Error! Attempt to control unarmed motors\n");
	}
	controller_arming.motor_on = 0;
	return 0;
}
/*******************************************************************************
* void* read_input(void* ptr)
*
* Reads user input from command line.
*******************************************************************************/
void* read_input(void* ptr){

	
	printf("Enter command ('help' for command list): \n");

	while(rc_get_state()!=EXITING){		

		char c[21]; //user input array
		char c_trimmed[21]; //cleaned up user input array

		//read input from keyboard
		if (fgets(c,20,stdin) != NULL){ //if there is input:
			strcpy(c_trimmed,trimwhitespace(c)); //delete trailing and leading whitespace
			if (c_trimmed[0] != '\n' && c_trimmed[0] != '\0'){ //if what is left is not a newline or NULL
				
				char *command,*command_opt;
								
				command = strtok(c_trimmed," "); //break input into format 'COMMMAND OPTION'
				command_opt = strtok(NULL," ");

				
				

				if(!strcmp(command,"display")){ //if COMMAND is 'display':
					if(command_opt != NULL){ //and if there is an OPTION specified
						if(!strcmp(command_opt,"exit")){ //and that option is to exit
							
							pthread_cancel(printf_thread); //exit display thread
						}
						else{
							print_usage(); //otherwise show correct command syntax
						}
					}
					
					//if there isn't OPTION specified:
					//start printf_thread if running from a terminal
					//if it was started as a background process then don't bother
					else if(isatty(fileno(stdout))){  
						pthread_create(&printf_thread, NULL, printf_loop, (void*) NULL); //thread for printing data to screen
						pthread_detach(printf_thread);  //detach thread so it can be closed easier. 
														//fine to do since it doesn't need to do anything except display stuff on screen
					}
				}
				else if(!strcmp(command,"drive")){ //if COMMAND is 'drive'
					snprintf(command_opt,strlen(command_opt)+1,"%li",strtol(command_opt,NULL,10)); //filter out non-numeric arguments to OPTION
					if(command_opt != NULL){ //if there is an arguement passed
						controller_arming.motor_on = 1; //arm motors
						sys_state.WS_angle_setpoint = cfg_setting.STEPS_PER_WS_ANGLE_DEGREE*atoi(command_opt); //send # of steps to motor_output()
						motor_output(); //drive motors
					}
					else{
						printf("Invalid command.\n");
						print_usage(); //otherwise show correct command syntax
					}
				}
				else if (!strcmp(command,"exit")){
					cleanup_everything();
					rc_set_state(EXITING);
					return NULL;
				}
				else if (!strcmp(command,"help")){
					print_usage();
				}
				else{
					printf("Invalid command.\n");
					print_usage();
				}
		
			}
		}

		//signal PWM duty to motor driver
		//rc_set_motor(WS_MOTOR_CHANNEL, sys_state.WS_angle_setpoint); 
		//rc_set_motor(BL_MOTOR_CHANNEL_L, BL_MOTOR_POLARITY_L * sys_state.BL_duty_signal_left); 
		//rc_set_motor(BL_MOTOR_CHANNEL_R, BL_MOTOR_POLARITY_R * sys_state.BL_duty_signal_right); 

		//can use servo rails maybe
		//need to map duty cycle to us pulse width (NOT MODULATED BY FREQ, different from PWM)
		//rc_send_servo_pulse_us(WS_MOTOR_CHANNEL, int us)

		//or gps headers as gpio
		//create thread with freq of pwm? where toggles high/low
		//rc_gpio_set_value(WS_MOTOR_CHANNEL, HIGH);

		rc_usleep(1000000 / cfg_setting.READ_INPUT_HZ);
	}
	return NULL;
}


/*******************************************************************************
* void* printf_loop(void* ptr)
*
* Thread to print information for debugging.
*******************************************************************************/
void* printf_loop(void* ptr){
	rc_state_t last_state, new_state; // keep track of last state 
	while(rc_get_state()!=EXITING){
		new_state = rc_get_state();
		// check if this is the first time since being paused
		if(new_state==RUNNING && last_state!=RUNNING){
			printf("\nRUNNING\n");
			printf("  Pitch  |");
			printf("  Roll  |");
			printf("  WS Steps  |");
			printf("  Error  |");
			//printf("  BL Duty L  |");
			//printf("  BL Duty R  |");
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
			printf("%10.2f  |", sys_state.WS_angle_setpoint);
			printf("%7.2f  |", controller_state.error);
			//printf("%11.2f  |", sys_state.BL_duty_signal_left);
			//printf("%11.2f  |", sys_state.BL_duty_signal_right);
			printf("%17.2f  |", sys_state.battery_voltage);
			printf("%10.2d  |", controller_arming.armstate);
			fflush(stdout);
		}
		rc_usleep(1000000 / cfg_setting.PRINTF_HZ);
	}
	return NULL;
}

/*******************************************************************************
* int print_usage()
*
* Prints default arguments to controller
*******************************************************************************/
int print_usage(){
	printf("\nCommand List: \n");
	printf("1) display - Displays orientation data. 'display exit' stops display output.\n");
	printf("2) drive ## - Sets desired angle of ## and sends to weight shift motor. \n");
	printf("3) exit - Quit control software.\n");
	printf("4) help - Displays list of commands.\n");
	return 0;
}

/*******************************************************************************
* void on_pause_pressed()
*
* Quits if pause button is held. Mainly for debugging.
*******************************************************************************/
void on_pause_pressed(){
	int i=0;
	const int samples = 100;	// check for release 100 times in this period
	const int us_wait = 1000000; // 1 second
	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		rc_usleep(us_wait/samples);
		if(rc_get_pause_button() == RELEASED) return;
	}
	printf("Shutting down...\n");
	rc_set_state(EXITING);
	return;
}

/*******************************************************************************
* void on_pause_released()
*
* Pauses if pause button is pressed. Mainly for debugging.
*******************************************************************************/
void on_pause_released(){
	// toggle betewen paused and running modes
	if(rc_get_state()==RUNNING)   		rc_set_state(PAUSED);
	else if(rc_get_state()==PAUSED)	rc_set_state(RUNNING);
	return;
}

char *trimwhitespace(char *str)
{
  char *end;

  // Trim leading space
  while(isspace((unsigned char)*str)) str++;

  if(*str == 0)  // All spaces?
    return str;

  // Trim trailing space
  end = str + strlen(str) - 1;
  while(end > str && isspace((unsigned char)*end)) end--;

  // Write new null terminator
  *(end+1) = 0;

  return str;
}
