/*******************************************************************************
* my_read_sensors.c
*
* This is meant to be a domonstration of reading IMU data from the Robotics 
* Cape.
*******************************************************************************/

#include <roboticscape-usefulincludes.h>
#include <roboticscape.h>

#define IMU_SAMPLE_RATE 20


// function declarations
int on_pause_pressed();
int on_pause_released();
int on_imu_data();

imu_data_t imu_data;


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
	printf("\nHello BeagleBone\n");
	set_pause_pressed_func(&on_pause_pressed);
	set_pause_released_func(&on_pause_released);

	// Set up IMU
	imu_config_t imu_config = get_default_imu_config();
	imu_config.dmp_sample_rate = IMU_SAMPLE_RATE;

	if(initialize_imu_dmp(&imu_data, imu_config)<0) {
		printf("Error initializing IMU");
	}

	set_imu_interrupt_func(&on_imu_data);

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
	printf("\r");
	printf(" ");
	float angle = -atan2(imu_data.accel[2],imu_data.accel[1]);
	printf(" %6.2f %6.2f %6.2f %6.2f|",	imu_data.accel[0],\
					imu_data.accel[1],\
					imu_data.accel[2],\
					angle);
	fflush(stdout);
	return 0;
}
