#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include <libconfig.h>

/*******************************************************************************
* cfg_settings_t
*
* Holds values of settings read from cfg file.
*******************************************************************************/
typedef struct cfg_settings_t{

	/*IMU sample rate*/
	int SAMPLE_RATE_HZ;

	/*Structural properties of Beaglebone*/
	int STEPS_PER_WS_ANGLE_DEGREE;
	float CAPE_MOUNT_ANGLE_X;
	float CAPE_MOUNT_ANGLE_Y;
	float CAPE_MOUNT_ANGLE_Z;
	int V_NOMINAL;
	int PULSE_DELAY;

	/*electrical hookups*/
	int WS_MOTOR_CHANNEL;
	int WS_MOTOR_DIR_PIN;
	int BL_MOTOR_CHANNEL_L;
	int BL_MOTOR_DIR_PIN_L;
	int BL_MOTOR_CHANNEL_R;
	int BL_MOTOR_DIR_PIN_R;
	int BL_MOTOR_POLARITY_L;
	int BL_MOTOR_POLARITY_R;
	int LIMIT_SWITCH_1_PIN;
	int LIMIT_SWITCH_2_PIN;

	/*Number of steps per cm in brake line travel*/
	float BL_STEP_TO_DIST;

	/*Thread Loop Rates */
	int	BATTERY_CHECK_HZ;
	int	PRINTF_HZ;
	int	READ_INPUT_HZ;

	/* PID Parameters*/
	float K_P;
	float K_I;
	float K_D;

	/*wingover arrays*/
	int WINGOVER_ANGLES[30];
	int WINGOVER_TIMING[30];

} cfg_settings_t;

/*******************************************************************************
* controller_state_t
*
* This is the system state written to by the controller.
*******************************************************************************/
typedef struct controller_state_t{
		/*set up controller values*/
		float error;
		float last_error;
		float derivative;
		int wingover_flag;
		int i;
		float WS_control_signal;
		float WS_angle_setpoint;
		struct timespec seconds;
		FILE *data_file;
} controller_state_t;

/*******************************************************************************
* core_state_t
*
* This is the physical system state.
*******************************************************************************/
typedef struct core_state_t{
	float angle_about_x_axis; 		// body angle radians
	float angle_about_y_axis; 		// body angle radians
	float battery_voltage; 		// battery voltage
} core_state_t;

/*******************************************************************************
* orientation_t
*
* This is the system orientation as seen by accelerometer, gyroscope.
*******************************************************************************/
typedef struct orientation_t{
	float x_gyro;
	float x_accel;
	float y_gyro;
	float y_accel;
} orientation_t;


/*******************************************************************************
* Function Declarations
*******************************************************************************/
// IMU interrupt routine
void collect_data();

// thread declarations
void* battery_checker(void* ptr);
void* printf_loop(void* ptr);
void* read_input(void* ptr);
void* motor_output(void* ptr);

// regular functions
int print_usage();
int cleanup_everything();
int get_config_settings();
int setup_gpio_pins();
int brakeline_control();
char *trimwhitespace( char *str);

/*******************************************************************************
* Global Variables
*******************************************************************************/
core_state_t sys_state;
rc_imu_data_t data;
orientation_t orientation;
controller_state_t controller_state;
config_t cfg;
cfg_settings_t cfg_setting;

/*******************************************************************************
* Threads
*******************************************************************************/
pthread_t battery_thread;
pthread_t read_input_thread;
pthread_t printf_thread;
pthread_t motor_thread;

/*******************************************************************************
* main()
*
* Initialize the GPIO pins, IMU, threads, & wait until shut down
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

	/*set default config settings for IMU*/
	rc_imu_config_t conf = rc_default_imu_config();

	/*get configuration settings*/
	get_config_settings();

	/*set up pins for General Purpose Input/Output mode (GPIO)*/
	setup_gpio_pins();

	/*thread to sample beaglebone battery*/
	pthread_create(&battery_thread, NULL, battery_checker, (void*) NULL);

	/* wait for the battery thread to make the first read*/
	while(sys_state.battery_voltage==0 && rc_get_state()!=EXITING) rc_usleep(1000);

	/* set up IMU configuration*/
	conf.dmp_sample_rate = cfg_setting.SAMPLE_RATE_HZ;

	/*set orientation of beaglebone */
	conf.orientation = ORIENTATION_Z_UP;

	/* start imu */
	if(rc_initialize_imu_dmp(&data, conf)){
		printf("ERROR: IMU initialization failed!\n");
		return -1;
	}

	/*open data file for writing CSV position information for plotting*/
	controller_state.data_file = fopen("data_file.txt","w");

	//start thread for reading user input
	pthread_create(&read_input_thread, NULL, read_input, (void*) NULL);

	/*start thread for motor output*/
	pthread_create(&motor_thread, NULL, motor_output, (void*) NULL);

	/*set imu interrupt function - when IMU detects change, run this funciton*/
	rc_set_imu_interrupt_func(&collect_data);

	/*start actual stuff*/
	rc_set_state(RUNNING);
	rc_set_led(RED,0);
	rc_set_led(GREEN,1);


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
* int cleanup_everything()
*
* Frees up memory and powers off imu to prepare for shutdown.
*******************************************************************************/
int cleanup_everything(){
	rc_power_off_imu();
	//rc_set_cpu_freq(FREQ_ONDEMAND);
	pthread_cancel(battery_thread);
	pthread_cancel(printf_thread);
	fclose(controller_state.data_file);
	rc_gpio_unexport(cfg_setting.WS_MOTOR_CHANNEL);
	rc_gpio_unexport(cfg_setting.WS_MOTOR_DIR_PIN);
	rc_gpio_unexport(cfg_setting.BL_MOTOR_CHANNEL_L);
	rc_gpio_unexport(cfg_setting.BL_MOTOR_CHANNEL_R);
	rc_gpio_unexport(cfg_setting.BL_MOTOR_DIR_PIN_L);
	rc_gpio_unexport(cfg_setting.BL_MOTOR_DIR_PIN_R);
	rc_gpio_unexport(cfg_setting.LIMIT_SWITCH_1_PIN);
	rc_gpio_unexport(cfg_setting.LIMIT_SWITCH_2_PIN);

	config_destroy(&cfg);
	rc_cleanup();
	return 0;
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

	//complementary filter to get pitch angle
	sys_state.angle_about_x_axis = (0.9*orientation.x_gyro+0.1*orientation.x_accel);
	/*****************************************/

	/*****************************************
	* Find Roll Data
	*****************************************/
	// find angle from accelerometer
	orientation.y_accel = atan2(-1*data.accel[0],sqrt(pow(data.accel[2],2) + pow(data.accel[1],2)))*RAD_TO_DEG;

	// integrates the gyroscope angle rate using Euler's method to get the angle
	orientation.y_gyro = sys_state.angle_about_y_axis + 0.01*data.gyro[1];

	//complementary filter to get roll angle
	sys_state.angle_about_y_axis = (0.9*orientation.y_gyro+0.1*orientation.y_accel);
	/****************************************/

	return;
}

/*******************************************************************************
* void* motor_output(void* ptr)
*
* Outputs PDM (Pulse Delay Modulation) to motors based on PID control of pulse delay.
*******************************************************************************/
void* motor_output(void* ptr){

		/*PID control*/
		while(rc_get_state()!=EXITING){

			/*if doing wingover, and length of wingover vector (number of setpoints in cfg_setting.WINGOVER_ANGLES) minus the amount of points gone to (controller_state.i) isn't zero*/
			if(controller_state.wingover_flag == 1 && ((((sizeof(cfg_setting.WINGOVER_ANGLES))/((sizeof(cfg_setting.WINGOVER_ANGLES[0]))-2)) - controller_state.i) != 0)){
				/*setpoint goes to next wingover point*/
				controller_state.WS_angle_setpoint = cfg_setting.WINGOVER_ANGLES[controller_state.i];
				controller_state.error = controller_state.WS_angle_setpoint - sys_state.angle_about_x_axis;
			}

			/*set motor direction based on sign of error signal*/
			if(controller_state.error>0){
				rc_gpio_set_value_mmap(cfg_setting.WS_MOTOR_DIR_PIN,HIGH);
			}
			else{
				rc_gpio_set_value_mmap(cfg_setting.WS_MOTOR_DIR_PIN,LOW);
			}


			/*if either limit switch is triggered, switch direction and step off the trigger so that the motor won't be prevented from moving due to triggered limit switch*/
				if(rc_gpio_get_value_mmap(cfg_setting.LIMIT_SWITCH_2_PIN)==HIGH || rc_gpio_get_value_mmap(cfg_setting.LIMIT_SWITCH_1_PIN)==HIGH){

					if(controller_state.WS_angle_setpoint<0){
							controller_state.WS_angle_setpoint = abs(sys_state.angle_about_x_axis) + 50;

						}
					else if(controller_state.WS_angle_setpoint>0){
								controller_state.WS_angle_setpoint = abs(sys_state.angle_about_x_axis) - 50;

							}

				int count;
				for(count=0;count<150;count++){
					/*Number of steps (count) determined by: distance needed to untrigger switch divided by timing belt pulley radius to find angle of arc in rad
					then convert rad into degrees, and divide by number of degrees per step (given in motor spec) to find steps. in this case switch protrudes 0.9" and pulley is 0.875" radius so
					arc is 1.028 rad =  58.9 deg. With 1.8deg per step, 1/2 microstepping makes it 0.9deg per step, so get 65.48 steps needed to release switch.*/

					rc_gpio_set_value_mmap(cfg_setting.WS_MOTOR_CHANNEL,HIGH);
					rc_usleep(100); /* want it done smoothly for demonstration: 300|500 delay is very smooth*/
					rc_gpio_set_value_mmap(cfg_setting.WS_MOTOR_CHANNEL,LOW);
					rc_usleep(200);
				}

			}
			/*checks if errors were neglibible so that it doesn't send constant high if there was zero delay*/
			/*also checks if limit switches were hit so it doesn't rip itself apart trying to get to the setpoint*/
				if(rc_gpio_get_value_mmap(cfg_setting.LIMIT_SWITCH_1_PIN) != HIGH && rc_gpio_get_value_mmap(cfg_setting.LIMIT_SWITCH_2_PIN) != HIGH  && abs(controller_state.last_error) > 2.0  && abs(controller_state.error) > 2.0){
					rc_gpio_set_value_mmap(cfg_setting.WS_MOTOR_CHANNEL,HIGH); //pulse on
				}
				/*otherwise, if error is low and you're doing wingover*/
				else if (controller_state.wingover_flag == 1 && ((abs(controller_state.last_error) <= 2.0  && abs(controller_state.error) <= 2.0) || rc_gpio_get_value_mmap(cfg_setting.LIMIT_SWITCH_1_PIN) == HIGH || rc_gpio_get_value_mmap(cfg_setting.LIMIT_SWITCH_2_PIN) == HIGH)){
					/* pause at the corresponding post-setpoint delay*/
					rc_usleep(cfg_setting.WINGOVER_TIMING[controller_state.i]);

					/*and if you aren't at the end of the wingover setpoint array, increase the iteration*/
					if(controller_state.i < (sizeof(cfg_setting.WINGOVER_ANGLES))/((sizeof(cfg_setting.WINGOVER_ANGLES[0]))-2)){
						controller_state.i++;
					}
					/*otherwise you are at the end so reset to neutral position and quit*/
					else{
						printf("ending\n" );
						controller_state.i = 0;
						controller_state.wingover_flag = 0;
						controller_state.WS_angle_setpoint = 0.0;
					}
				}
				rc_usleep(100); /*stepper motors use fixed pulse width and modulate delay*/
				rc_gpio_set_value_mmap(cfg_setting.WS_MOTOR_CHANNEL,LOW); /*pulse off*/

			/*PD control */
			controller_state.last_error = controller_state.error;
			controller_state.error = controller_state.WS_angle_setpoint - sys_state.angle_about_x_axis;
			controller_state.derivative = controller_state.error - controller_state.last_error;
			controller_state.WS_control_signal = (cfg_setting.K_P*controller_state.error) + (cfg_setting.K_D*controller_state.derivative);

			/*pulse delay modulation*/
			if(abs(controller_state.WS_control_signal) >= 0.1){
				/*assuming inverse relationship btwn error and delay: large error means moves faster to compensate and vice versa for small error*/
				cfg_setting.PULSE_DELAY = round(1000*(1/abs(controller_state.WS_control_signal)));
				rc_usleep(cfg_setting.PULSE_DELAY); /*wait between pulses*/
			}

		}
	return NULL;
}
/*******************************************************************************
* int brakeline_control(float dist, int pul_pin[], int dir_pin[])
*
* Pulls/releases brake lines based on user inputted line travel distance.
*******************************************************************************/
int brakeline_control(float dist[], int pul_pin[], int dir_pin[]){
	int i;
	float bl_dist;

	/*set direction for each motor depending on its polarity (based on orientation)*/
	if(dist[0] >= 0){
		rc_gpio_set_value_mmap(dir_pin[0],HIGH);
	}
	else if(dist[0] < 0){
		rc_gpio_set_value_mmap(dir_pin[0],LOW);
	}

	if(dist[1] >= 0){
		rc_gpio_set_value_mmap(dir_pin[1],HIGH);
	}
	else if(dist[1] < 0){
		rc_gpio_set_value_mmap(dir_pin[1],LOW);
	}

	/*make motors move distance*/
	if(dist[0]!='\0'){
		bl_dist = abs(dist[0]);
	}
	else if (dist[1]!='\0'){
		bl_dist = abs(dist[1]);
	}

	/*move each for fixed number of steps. no need for closed loop control for this basic task.*/
	for(i=0;i*cfg_setting.BL_STEP_TO_DIST<bl_dist;i++){

		rc_gpio_set_value_mmap(pul_pin[0],HIGH); //pulse on left motor
		rc_gpio_set_value_mmap(pul_pin[1],HIGH); //pulse on right motor
		rc_usleep(300); //pulse width
		rc_gpio_set_value_mmap(pul_pin[0],LOW); //pulse off left motor
		rc_gpio_set_value_mmap(pul_pin[1],LOW); //pulse off right motor
		rc_usleep(500);
	}
	return 0;
}
/*******************************************************************************
* void* read_input(void* ptr)
*
* Reads user input from command line.
*******************************************************************************/
void* read_input(void* ptr){

	printf("@-----------------------------@\n");
	printf("| Paraglider Control Software |\n");
	printf("@-----------------------------@\n\n");
	printf("Enter command ('help' for command list): \n");

	while(rc_get_state()!=EXITING){

		char c[21]; //user input array
		char c_trimmed[21]; //cleaned up user input array

		//read input from keyboard
		if (fgets(c,20,stdin) != NULL){ //if there is input:
			strcpy(c_trimmed,trimwhitespace(c)); //delete trailing and leading whitespace
			if (c_trimmed[0] != '\n' && c_trimmed[0] != '\0'){ //if what is left is not a newline or NULL

				/* break input into format 'COMMMAND OPTION' */
				char *command,*command_opt;
				command = strtok(c_trimmed," ");
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
					if(command_opt != NULL){ //if there is an argument passed

						/*if either limit switch is triggered, switch direction and step off the trigger so that the motor won't be prevented from moving due to triggered limit switch*/
							if(rc_gpio_get_value_mmap(cfg_setting.LIMIT_SWITCH_2_PIN)==HIGH || rc_gpio_get_value_mmap(cfg_setting.LIMIT_SWITCH_1_PIN)==HIGH){

								if(controller_state.WS_angle_setpoint<0){
										controller_state.WS_angle_setpoint = abs(sys_state.angle_about_x_axis) + 50;

									}
									else if(controller_state.WS_angle_setpoint>0){
											controller_state.WS_angle_setpoint = abs(sys_state.angle_about_x_axis) - 50;

										}

							int count;
							for(count=0;count<150;count++){
								/*Number of steps (count) determined by: distance needed to untrigger switch divided by timing belt pulley radius to find angle of arc in rad
								then convert rad into degrees, and divide by number of degrees per step (given in motor spec) to find steps. in this case switch protrudes 0.9" and pulley is 0.875" radius so
								arc is 1.028 rad =  58.9 deg. With 1.8deg per step, 1/2 microstepping makes it 0.9deg per step, so get 65.48 steps needed to release switch.*/

								rc_gpio_set_value_mmap(cfg_setting.WS_MOTOR_CHANNEL,HIGH);
								rc_usleep(300); /* want it done smoothly for demonstration: 300|500 delay is very smooth*/
								rc_gpio_set_value_mmap(cfg_setting.WS_MOTOR_CHANNEL,LOW);
								rc_usleep(500);
							}

						}
						controller_state.WS_angle_setpoint = atoi(command_opt); //send argument as angle to motor_output()
					}
					else{
						printf("Invalid command.\n");
						print_usage(); //otherwise show correct command syntax
					}
				}
				/*if brake left should move*/
				else if (!strcmp(command,"brakel")){
					snprintf(command_opt,strlen(command_opt)+1,"%li",strtol(command_opt,NULL,10)); //filter out non-numeric arguments to OPTION
					if(command_opt != NULL){ /*if there is an argument passed*/
						float bl_arg_dist[2] = {atof(command_opt)*cfg_setting.BL_MOTOR_POLARITY_L, '\0'}; /*set only left to be driven, right to NULL*/
						int bl_arg_pul[2] = {cfg_setting.BL_MOTOR_CHANNEL_L, '\0'}; //set up pulse pin argument
						int bl_arg_dir[2] = {cfg_setting.BL_MOTOR_DIR_PIN_L, '\0'}; //set up dir pin argument
						brakeline_control(bl_arg_dist,bl_arg_pul,bl_arg_dir); //control bl motors
					}
					else{
						printf("Invalid command.\n");
						print_usage(); //otherwise show correct command syntax
					}
				}
				/*same as previous except for right motor*/
				else if (!strcmp(command,"braker")){
					snprintf(command_opt,strlen(command_opt)+1,"%li",strtol(command_opt,NULL,10)); //filter out non-numeric arguments to OPTION
					if(command_opt != NULL){ //if there is an argument passed
						float bl_arg_dist[2] = {'\0', atof(command_opt)*cfg_setting.BL_MOTOR_POLARITY_R}; /*left motor goes to NULL and right gets told to drive*/
						int bl_arg_pul[2] = {'\0', cfg_setting.BL_MOTOR_CHANNEL_R}; //set up pulse pin argument
						int bl_arg_dir[2] = {'\0', cfg_setting.BL_MOTOR_DIR_PIN_R}; //set up dir pin argument
						brakeline_control(bl_arg_dist,bl_arg_pul,bl_arg_dir); //control bl motors
					}
					else{
						printf("Invalid command.\n");
						print_usage(); //otherwise show correct command syntax
					}
				}
				/*drive both BL motors*/
				else if (!strcmp(command,"brakes")){
					snprintf(command_opt,strlen(command_opt)+1,"%li",strtol(command_opt,NULL,10)); //filter out non-numeric arguments to OPTION
					if(command_opt != NULL){ //if there is an argument passed
						float bl_arg_dist[2] = {atof(command_opt)*cfg_setting.BL_MOTOR_POLARITY_L, atof(command_opt)*cfg_setting.BL_MOTOR_POLARITY_R};
						int bl_arg_pul[2] = {cfg_setting.BL_MOTOR_CHANNEL_L, cfg_setting.BL_MOTOR_CHANNEL_R}; //set up pulse pin argument
						int bl_arg_dir[2] = {cfg_setting.BL_MOTOR_DIR_PIN_L, cfg_setting.BL_MOTOR_DIR_PIN_R}; //set up dir pin argument
						brakeline_control(bl_arg_dist,bl_arg_pul,bl_arg_dir); //control bl motors
					}
					else{
						printf("Invalid command.\n");
						print_usage(); /*otherwise show correct command syntax*/
					}
				}
				/*activate wingover code*/
				else if (!strcmp(command,"wingover")){
					controller_state.wingover_flag = 1;
					controller_state.i = 0;
				}
				else if (!strcmp(command,"exit")){
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

		rc_usleep(1000000 / cfg_setting.READ_INPUT_HZ);
	}
	return NULL;
}


/*******************************************************************************
* void* printf_loop(void* ptr)
*
* Thread to print information to screen.
*******************************************************************************/
void* printf_loop(void* ptr){
	rc_state_t last_state, new_state; // keep track of last state
	while(rc_get_state()!=EXITING){
		new_state = rc_get_state();
		// check if this is the first time since being stopped
		if(new_state==RUNNING && last_state!=RUNNING){
			printf("\nRUNNING\n");
			printf("  Pitch  |");
			printf("  Roll  |");
			printf("  WS Delay  |");
			printf("  Setpoint  |");
			printf("  Error  |");
			printf("  LS L  |");
			printf("  LS R  |");
			printf("  Battery Voltage  |");
			printf("  WS Dir  |");
			printf("  WS Ctl  |");
			printf("\n");
		}
		else if(new_state==PAUSED && last_state!=PAUSED){
			printf("\nPAUSED: press pause again to start.\n");
		}
		last_state = new_state;

		// decide what to print or exit
		if(new_state == RUNNING){
			printf("\r");
			printf("  %5.2f  |", sys_state.angle_about_y_axis);
			printf("%6.2f  |", sys_state.angle_about_x_axis);
			printf("%10.2d  |", cfg_setting.PULSE_DELAY);
			printf("%10.2f  |", controller_state.WS_angle_setpoint);
			printf("%7.2f  |", controller_state.error);
			printf("%6.2d  |", rc_gpio_get_value_mmap(cfg_setting.LIMIT_SWITCH_1_PIN));
			printf("%6.2d  |", rc_gpio_get_value_mmap(cfg_setting.LIMIT_SWITCH_2_PIN));
			printf("%17.2f  |", sys_state.battery_voltage);
			printf("%8.2d  |", rc_gpio_get_value_mmap(cfg_setting.WS_MOTOR_DIR_PIN));
			printf("%8.2f  |", controller_state.WS_control_signal);

			/*get time in seconds and roll angle. Write into file as CSV for plotting in matlab. Optional, was only for getting pictures for poster*/
			clock_gettime(CLOCK_MONOTONIC,&controller_state.seconds);
			char data[20];
			sprintf(data,"%f,%lf\n",sys_state.angle_about_x_axis,controller_state.seconds.tv_sec + 1e-9 * controller_state.seconds.tv_nsec);
			fputs(data,controller_state.data_file);
			/*************************/

			fflush(stdout);
		}
		rc_usleep(1000000 / cfg_setting.PRINTF_HZ);
	}
	return NULL;
}

/*******************************************************************************
* int print_usage()
*
* Prints default arguments to controller.
*******************************************************************************/
int print_usage(){
	printf("\nCommand List: \n");
	printf("1) display - Displays orientation data. 'display exit' stops display output.\n");
	printf("2) drive # - Sets desired angle of # (deg) and sends to weight shift motor. \n");
	printf("3) brakel # - Pulls left brake line down # centimeters.\n");
	printf("4) braker # - Pulls right brake line down # centimeters.\n");
	printf("5) brakes # - Pulls both brake lines down # centimeters.\n");
	printf("6) wingover - Performs the wingover maneuver.\n");
	printf("7) exit - Quit control software.\n");
	printf("8) help - Displays list of commands.\n");
	return 0;
}

/*******************************************************************************
* int setup_gpio_pins()
*
* Initializes gpio pins for reading/writing.
*******************************************************************************/
int setup_gpio_pins(){

	/*Weight-shift motor pulse pin*/
	rc_set_pinmux_mode(cfg_setting.WS_MOTOR_CHANNEL, PINMUX_GPIO);
	rc_gpio_export(cfg_setting.WS_MOTOR_CHANNEL);
	rc_gpio_set_dir(cfg_setting.WS_MOTOR_CHANNEL, OUTPUT_PIN);
	rc_gpio_set_value_mmap(cfg_setting.WS_MOTOR_CHANNEL,LOW);

	/*Weight-shift motor direction pin*/
	rc_set_pinmux_mode(cfg_setting.WS_MOTOR_DIR_PIN, PINMUX_GPIO_PU);
	rc_gpio_export(cfg_setting.WS_MOTOR_DIR_PIN);
	rc_gpio_set_dir(cfg_setting.WS_MOTOR_DIR_PIN, OUTPUT_PIN);
	rc_gpio_set_value_mmap(cfg_setting.WS_MOTOR_DIR_PIN,LOW);

	/*Left Brake Line motor pulse pin*/
	rc_set_pinmux_mode(cfg_setting.BL_MOTOR_CHANNEL_L, PINMUX_GPIO);
	rc_gpio_export(cfg_setting.BL_MOTOR_CHANNEL_L);
	rc_gpio_set_dir(cfg_setting.BL_MOTOR_CHANNEL_L, OUTPUT_PIN);
	rc_gpio_set_value_mmap(cfg_setting.BL_MOTOR_CHANNEL_L,LOW);

	/*Left Brake Line motor direction pin*/
	rc_set_pinmux_mode(cfg_setting.BL_MOTOR_DIR_PIN_L, PINMUX_GPIO_PU);
	rc_gpio_export(cfg_setting.BL_MOTOR_DIR_PIN_L);
	rc_gpio_set_dir(cfg_setting.BL_MOTOR_DIR_PIN_L, OUTPUT_PIN);
	rc_gpio_set_value_mmap(cfg_setting.BL_MOTOR_DIR_PIN_L,LOW);

	/*Right Brake Line motor pulse pin*/
	rc_set_pinmux_mode(cfg_setting.BL_MOTOR_CHANNEL_R, PINMUX_GPIO);
	rc_gpio_export(cfg_setting.BL_MOTOR_CHANNEL_R);
	rc_gpio_set_dir(cfg_setting.BL_MOTOR_CHANNEL_R, OUTPUT_PIN);
	rc_gpio_set_value_mmap(cfg_setting.BL_MOTOR_CHANNEL_R,LOW);

	/*Right Brake Line motor direction pin*/
	rc_set_pinmux_mode(cfg_setting.BL_MOTOR_DIR_PIN_R, PINMUX_GPIO_PU);
	rc_gpio_export(cfg_setting.BL_MOTOR_DIR_PIN_R);
	rc_gpio_set_dir(cfg_setting.BL_MOTOR_DIR_PIN_R, OUTPUT_PIN);
	rc_gpio_set_value_mmap(cfg_setting.BL_MOTOR_DIR_PIN_R,LOW);

	/*Limit switch 1 pin*/
	rc_set_pinmux_mode(cfg_setting.LIMIT_SWITCH_1_PIN, PINMUX_GPIO_PU);
	rc_gpio_export(cfg_setting.LIMIT_SWITCH_1_PIN);
	rc_gpio_set_dir(cfg_setting.LIMIT_SWITCH_1_PIN, INPUT_PIN);
	rc_gpio_set_value_mmap(cfg_setting.LIMIT_SWITCH_1_PIN,LOW);

	/*Limit switch 2 pin*/
	rc_set_pinmux_mode(cfg_setting.LIMIT_SWITCH_2_PIN, PINMUX_GPIO_PU);
	rc_gpio_export(cfg_setting.LIMIT_SWITCH_2_PIN);
	rc_gpio_set_dir(cfg_setting.LIMIT_SWITCH_2_PIN, INPUT_PIN);
	rc_gpio_set_value_mmap(cfg_setting.LIMIT_SWITCH_2_PIN,LOW);

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
 	if(!config_read_file(&cfg, "paraglider_config.cfg")){
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

	if(config_lookup_int(&cfg, "PULSE_DELAY", &cfg_value_int)){
		cfg_setting.PULSE_DELAY = cfg_value_int;
	}
	else{
	fprintf(stderr, "No 'PULSE_DELAY' setting in configuration file.\n");
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

	if(config_lookup_int(&cfg, "LIMIT_SWITCH_1_PIN", &cfg_value_int)){
		cfg_setting.LIMIT_SWITCH_1_PIN = cfg_value_int;
	}
	else{
	fprintf(stderr, "No 'LIMIT_SWITCH_1_PIN' setting in configuration file.\n");
	}

	if(config_lookup_int(&cfg, "LIMIT_SWITCH_2_PIN", &cfg_value_int)){
		cfg_setting.LIMIT_SWITCH_2_PIN = cfg_value_int;
	}
	else{
	fprintf(stderr, "No 'LIMIT_SWITCH_2_PIN' setting in configuration file.\n");
	}

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

	if(config_lookup_float(&cfg, "BL_STEP_TO_DIST", &cfg_value_float)){
  	cfg_setting.BL_STEP_TO_DIST = cfg_value_float;
	}
	else{
	fprintf(stderr, "No 'BL_STEP_TO_DIST' setting in configuration file.\n");
	}

	if(config_lookup_int(&cfg, "BATTERY_CHECK_HZ", &cfg_value_int)){
  	cfg_setting.BATTERY_CHECK_HZ = cfg_value_int;
	}
	else{
	fprintf(stderr, "No 'BATTERY_CHECK_HZ' setting in configuration file.\n");
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

	int j;
	config_setting_t *setting1, *setting2;
	setting1 = config_lookup(&cfg,"WINGOVER_ANGLES");
	if(setting1 != NULL){
		for(j=0;j<config_setting_length(setting1);j++){
			cfg_setting.WINGOVER_ANGLES[j] = config_setting_get_int_elem(setting1,j);
			printf("%d ",cfg_setting.WINGOVER_ANGLES[j] );
		}
	}
	else{
		fprintf(stderr, "No 'WINGOVER_ANGLES' setting in configuration file.\n");
	}
	printf("\n");

	setting2 = config_lookup(&cfg,"WINGOVER_TIMING");
	if(setting2 != NULL){
		for(j=0;j<config_setting_length(setting2);j++){
			cfg_setting.WINGOVER_TIMING[j] = config_setting_get_int_elem(setting2,j);
		}
	}
	else{
		fprintf(stderr, "No 'WINGOVER_TIMING' setting in configuration file.\n");
	}
	return 0;
}

/******************************************************************************
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
* char *trimwhitespace(char *str)
*
* Removes leading and trailing whitespace from a string.
*******************************************************************************/
char *trimwhitespace(char *str)
{
  char *end;
	/*remove leading whitespace*/
  while(isspace((unsigned char)*str)){
		str++;
	 }
	 /* check if it's all spaces*/
	if(*str == 0){
    return str;
	}
  /*remove trailing whitespace */
  end = str + strlen(str) - 1;
  while(end > str && isspace((unsigned char)*end)){
		end--;
	 }
  /* add new null terminator */
  *(end+1) = 0;

  return str;
}
