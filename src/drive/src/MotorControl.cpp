/***************************************************************************
 * MotorControl.cpp
 * Charles Sedgwick
 * The MotorControl class is reponsible for controlling the direction and speed
 * of the 4 motors. 
 *
 ***************************************************************************/

// main roboticscape API header
#include "MotorControl/MotorControl.hpp"
extern "C"
{  
#include "roboticscape.h"
}
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
        return;
	}

	// do your own initialization here
	//rc_set_pause_pressed_func(&rover::MotorControl::on_pause_pressed);
	//rc_set_pause_released_func(&rover::MotorControl::on_pause_released);

    // done initializing so set state to RUNNING
	set_state(RUNNING); 
    rc_enable_motors();
    set_speed(DEFAULT_DUTY_CYCLE);
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

int MotorControl::turn_right(){
    rc_set_motor(1, duty);
    rc_set_motor(2, duty);
    rc_set_motor(3, -duty);
    rc_set_motor(4, -duty);
	return 1;
}

int MotorControl::turn_left(){
    rc_set_motor(1, -duty);
    rc_set_motor(2, -duty);
    rc_set_motor(3, duty);
    rc_set_motor(4, duty);
	return 1;
}

int MotorControl::stop(){
    rc_set_motor_brake_all();
	return 1;
}

int MotorControl::forward(){
    rc_set_motor_all(duty);
	return 1;
}

int MotorControl::backward(){
    rc_set_motor_all(-duty);
	return 1;
}

void MotorControl::set_speed(double pDuty){
    duty = pDuty;
}

double MotorControl::get_speed(){
	return duty;
}

void MotorControl::set_diff(int pDiff){
    diff = pDiff;
}

int MotorControl::get_diff(){
	return diff;
}


void MotorControl::set_state(rc_state_t new_state){
    rc_set_state(new_state);
}

rc_state_t MotorControl::get_state(){
    return rc_get_state();
}

int MotorControl::clean_up(){
    rc_set_state(EXITING);
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

void MotorControl::speedCallback(const std_msgs::Float32::ConstPtr& msg){
    /* if they differ from current values then set speed and diff to new */
    if( msg->data != duty ){
        duty = msg->data;
    }
    /* publish new speed to /drive_status */
}

void MotorControl::diffCallback(const std_msgs::Int32::ConstPtr& msg){
    /* if they differ from current values then set speed and diff to new */
    if( msg->data != diff ){
        diff = msg->data;
    }
    if( diff < 0 ){
        turn_left();
    }
    if( diff == 0 ){
        forward();
    }
    if( diff == 1 ){
        turn_right();
    }
    if( diff == 2 ){
        backward();
    }
    if( diff == 3 ){
        stop();
    }
    if( diff == 9 ){
        clean_up();
    }
    /* publish new diff to /drive_status */
}

}; // rover namespace

