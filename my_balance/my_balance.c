/*******************************************************************************
* complementary_filters.c
*
* This is meant to be a demonstration of reading IMU data from the Robotics 
* Cape and fusing the gyroscope and accelerometer values using complementary
* filters.
*******************************************************************************/

#include <roboticscape-usefulincludes.h>
#include <roboticscape.h>

#define IMU_SAMPLE_RATE 200

// function declarations
int on_pause_pressed();
int on_pause_released();
int on_imu_data();
void arm_controller();
void disarm_controller();
void zero_filters();


imu_data_t imu_data;


float march_theta_estimator(int reset_filter);
float march_inner_loop(float input_curr, int reset_filter);
float march_outer_loop(float input_curr, int reset_filter);
void* outer_loop_runner(void* ptr);

#define ENCODER_CHANNEL_L		3
#define ENCODER_CHANNEL_R		2
#define ENCODER_POLARITY_L		1
#define ENCODER_POLARITY_R		-1
#define GEARBOX 				35.577
#define ENCODER_RES				60

int controller_armed = 0;
float phi = 0;
float d2_u = 0;



/*******************************************************************************
* int main() 
*	
* This template main function contains these critical components
* - call to initialize_cape
* - main while loop that checks for EXITING condition
* - cleanup_cape() at the end
*******************************************************************************/
int main(){
	// always initialize cape library first
	initialize_cape();

	// do your own initialization here
	set_pause_pressed_func(&on_pause_pressed);
	set_pause_released_func(&on_pause_released);

	// Set up IMU
	imu_config_t imu_config = get_default_imu_config();
	imu_config.dmp_sample_rate = IMU_SAMPLE_RATE;

	if(initialize_imu_dmp(&imu_data, imu_config)<0) {
		printf("Error initializing IMU");
		return -1;
	}
	printf("hp,lp,sum,accel,gyro\r\n");
	fflush(stdout);
	set_imu_interrupt_func(&on_imu_data);

	// Start the outer loop thread
	pthread_t  outer_loop_thread;
	pthread_create(&outer_loop_thread, NULL, outer_loop_runner, (void*) NULL);

	// done initializing so set state to RUNNING
	set_state(RUNNING);

	// Keep looping until state changes to EXITING
	while(get_state()!=EXITING){
		// handle other states
		if(get_state()==RUNNING){
			// do things
			set_led(GREEN, ON);
			set_led(RED, OFF);
		}
		else if(get_state()==PAUSED){
			// do other things
			set_led(GREEN, OFF);
			set_led(RED, ON);
		}
		// always sleep at some point
		usleep(1000);
	}
	
	// exit cleanly
	disable_motors();
	power_off_imu();
	cleanup_cape(); 
	return 0;
}


/*******************************************************************************
* int on_pause_released() 
*	
* Make the Pause button toggle between paused and running states.
*******************************************************************************/
int on_pause_released(){
	// toggle between paused and running modes
	if(get_state()==RUNNING)   		set_state(PAUSED);
	else if(get_state()==PAUSED)	set_state(RUNNING);
	return 0;
}

/*******************************************************************************
* int on_pause_pressed() 
*	
* If the user holds the pause button for 2 seconds, set state to exiting which 
* triggers the rest of the program to exit cleanly.
*******************************************************************************/
int on_pause_pressed(){
	int i=0;
	const int samples = 100;	// check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds
	
	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		usleep(us_wait/samples);
		if(get_pause_button() == RELEASED) return 0;

	}
	printf("long press detected, shutting down\n");
	set_state(EXITING);
	return 0;
}

int on_imu_data() {

	float theta = march_theta_estimator(0);
	theta += 0.503;
	if ((theta < -0.37 || theta > 0.37) && controller_armed) {
		disarm_controller();
		printf("Tip Detected...\n");
	} else if (!controller_armed && theta > -0.2 && theta < 0.2) {
		arm_controller();
		printf("Recovered from tip!\n");
	}

	// collect encoder positions, right wheel is reversed 
	float wheelAngleR = (get_encoder_pos(ENCODER_CHANNEL_R) * 2*M_PI) \
								/(ENCODER_POLARITY_R * GEARBOX * ENCODER_RES);
	float wheelAngleL = (get_encoder_pos(ENCODER_CHANNEL_L) * 2*M_PI) \
								/(ENCODER_POLARITY_L * GEARBOX * ENCODER_RES);
	
	// Phi is average wheel rotation also add theta body angle to get absolute 
	// wheel position in global frame since encoders are attachde to the body
	phi = ((wheelAngleL+wheelAngleR)/2) + theta;


	

	float d1_u = march_inner_loop(d2_u - (theta),0);
	

	float dutyL = -d1_u;
	float dutyR = -d1_u;	
	set_motor(1, -1 * dutyL); 
	set_motor(2, 1 * dutyR); 
	printf("%6f %6f %6f\n",(theta),d2_u, d1_u);
	fflush(stdout);

	/*
	printf("%f,%f,%f,%f,%f\r\n", prev_output_hp, prev_output_lp, 
		prev_output_lp + prev_output_hp, angle, gyro_angle);
	fflush(stdout);*/
	return 0;
}

void* outer_loop_runner(void* ptr) {
	while (get_state() != EXITING) {
		d2_u = march_outer_loop(-phi,0);
		usleep(1000000 / 20);
	}
	return NULL;
}

float march_theta_estimator(int reset_filter) {
	static float gyro_angle;
	static float tau = 0.5;
	static float dt = .01;
	float angle = -atan2(imu_data.accel[2],imu_data.accel[1]);
	static float input_prev_hp;
	static float input_prev_lp;
	static float output_prev_lp;
	static float output_prev_hp;
	if (reset_filter != 0) {
		gyro_angle = angle;
		// Prefill inputs/outputs
		output_prev_lp = angle;
		input_prev_hp = angle;
		input_prev_lp = 0;
		output_prev_hp = 0;
		return angle;

	} else {
		gyro_angle += (imu_data.gyro[0] * M_PI/(float)180)/IMU_SAMPLE_RATE;
	}

	// Run the new readings through the filters
	output_prev_hp = (1 - dt/tau) * gyro_angle + (dt/tau - 1) * input_prev_hp - 
					 (dt/tau - 1) * output_prev_hp;
	output_prev_lp = dt/tau * angle - (dt/tau - 1) * output_prev_lp;
	input_prev_hp = gyro_angle;
	return output_prev_hp + output_prev_lp;
}

// Assume 2nd order filter
float march_inner_loop(float input_curr, int reset_filter) {

	/* // James values
	static float num[] = {-6.289, 11.910, -5.634 };
	static float den[] = { 1.000, -1.702,  0.702 };
	*/
	// My values
	static float num[] = {-25.04, 45.16, -20.23};
	static float den[] = {1, -1.434, 0.434};
	static float input_prev;
	static float input_prev2;
	static float output_prev;
	static float output_prev2;
	static float gain = 0.25;
	if (reset_filter != 0) {
		input_prev = 0;
		input_prev2 = 0;
		output_prev = 0;
		output_prev2 = 0;
		input_curr = 0;
	}

	float output_curr = 1/den[0] * (gain*num[0]*input_curr + gain*num[1]*input_prev + gain*num[2]*input_prev2 - den[1]*output_prev - den[2]*output_prev2);
	// saturate the output
	output_curr = output_curr > 1 ? 1 : output_curr;
	output_curr = output_curr < -1 ? -1 : output_curr;
	input_prev2 = input_prev;
	input_prev = input_curr;
	output_prev2 = output_prev;
	output_prev = output_curr;

	return output_curr;
}

// Assume 1st order filter
float march_outer_loop(float input_curr, int reset_filter) {

	 // James values
	//static float num[] = { 0.3858, -0.3853 };
	//static float den[] = { 1.0000, -0.9277 };
	
	// My values
	static float num[] = {-0.04783, 0.04783};
	static float den[] = {1, -0.4545};
	static float input_prev;
	static float output_prev;
	static float gain = -7.5*0.5;
	if (reset_filter != 0) {
		input_prev = 0;
		output_prev = 0;
		input_curr = 0;
	}

	float output_curr = 1/den[0] * (gain*num[0]*input_curr + gain*num[1]*input_prev - den[1]*output_prev );
	// saturate the output
	output_curr = output_curr > .37 ? .37 : output_curr;
	output_curr = output_curr < -.37 ? -.37 : output_curr;
	input_prev = input_curr;
	output_prev = output_curr;

	return output_curr;
}

void arm_controller() {
	zero_filters();
	enable_motors();
	controller_armed = 1;
}

void disarm_controller() {
	disable_motors();
	controller_armed = 0;
}

void zero_filters() {
	// Reset filters
	march_inner_loop(0,1);
	march_outer_loop(0,1);
	march_theta_estimator(1);
	// Reset encoder positions
	set_encoder_pos(ENCODER_CHANNEL_L,0);
	set_encoder_pos(ENCODER_CHANNEL_R,0);
}