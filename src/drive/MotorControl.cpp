/***************************************************************************
 * MotorControl.cpp
 * Charles Sedgwick
 * The MotorControl class is reponsible for controlling the direction and speed
 * of the 4 motors. 
 *
 ***************************************************************************/

// usefulincludes is a collection of common system includes for the lazy
// This is not necessary for roboticscape projects but here for convenience
#include <rc_usefulincludes.h> 
// main roboticscape API header
#include <roboticscape.h>
#include "MotorControl.hpp"
#include <iostream>

using namespace std;

namespace rover {

/***************************************************************************
 * CONSTRUCTOR
 * @brief  Instantiates a new motor control class
 *************************************************************************/
MotorControl::MotorControl(){
	// always initialize cape library first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to initialize rc_initialize(), are you root?\n");
		return -1;
	}

	// do your own initialization here
	rc_set_pause_pressed_func(&on_pause_pressed);
	rc_set_pause_released_func(&on_pause_released);

    // done initializing so set state to RUNNING
	set_state(rc_state); 
    rc_enable_motors();
    set_duty(DEFAULT_DUTY_CYCLE);
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

int MotorControl::turn_right(){
	cout<<"Turning right..."<<endl;
    rc_set_motor(1, duty);
    rc_set_motor(2, duty);
    rc_set_motor(3, -duty);
    rc_set_motor(4, -duty);
	return 1;
}

int MotorControl::turn_left(){
	stop();
	cout<<"Turning left..."<<endl;
    rc_set_motor(1, -duty);
    rc_set_motor(2, -duty);
    rc_set_motor(3, duty);
    rc_set_motor(4, duty);
	return 1;
}

int MotorControl::stop(){
	cout<<"Stopping all motors..."<<endl;
    rc_set_motor_brake_all();
	return 1;
}

int MotorControl::forward(){
	cout<<"Moving forward..."<<endl;
    rc_set_motor_all(duty);
	return 1;
}

int MotorControl::backward(){
	cout<<"Moving backward..."<<endl;
    rc_set_motor_all(-duty);
	return 1;
}

void MotorControl::set_speed(double pDuty){
    duty = pDuty;
}

double MotorControl::get_speed(){
	return duty;
}

void set_state(rc_state_t new_state){
    rc_set_state(new_state);
}

rc_state_t get_state(){
    return rc_get_state();
}

int MotorControl::clean_up(){
    rc_cleanup();
	return 1;
}

MotorControl::~MotorControl(){
    clean_up();
}

/*******************************************************************************
* void on_pause_released() 
*	
* Make the Pause button toggle between paused and running states.
*******************************************************************************/
void MotorControl::on_pause_released(){
	// toggle betewen paused and running modes
	if(get_state()==RUNNING)		set_state(PAUSED);
	else if(get_state()==PAUSED)	set_state(RUNNING);
	return;
}

/*******************************************************************************
* void on_pause_pressed() 
*
* If the user holds the pause button for 2 seconds, set state to exiting which 
* triggers the rest of the program to exit cleanly.
*******************************************************************************/
void MotorControl::on_pause_pressed(){
	int i=0;
	const int samples = 100;	// check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds
	
	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		rc_usleep(us_wait/samples);
		if(rc_get_pause_button() == RELEASED) return;
	}
	printf("long press detected, shutting down\n");
	set_state(EXITING);
	return;
}

}; // rover namespace

