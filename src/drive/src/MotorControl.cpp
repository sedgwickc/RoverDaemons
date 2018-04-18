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
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <actionlib_msgs/GoalStatus.h>

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
    //set_speed(DEFAULT_SPEED_X);
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

int MotorControl::turn_right(){
    rc_set_motor(1, linear.x);
    rc_set_motor(2, linear.x);
    rc_set_motor(3, -linear.x);
    rc_set_motor(4, -linear.x);
	return 1;
}

int MotorControl::turn_left(){
    rc_set_motor(1, -linear.x);
    rc_set_motor(2, -linear.x);
    rc_set_motor(3, linear.x);
    rc_set_motor(4, linear.x);
	return 1;
}

int MotorControl::stop(){
    rc_set_motor_brake_all();
	return 1;
}

int MotorControl::forward(){
    rc_set_motor_all(linear.x);
	return 1;
}

int MotorControl::backward(){
    rc_set_motor_all(-linear.x);
	return 1;
}

void MotorControl::set_speed(geometry_msgs::Vector3 pLinear){
    linear = pLinear;
}

geometry_msgs::Vector3 MotorControl::get_speed(){
	return linear;
}

void MotorControl::set_angular(geometry_msgs::Vector3 pAngular){
    angular = pAngular;
}

geometry_msgs::Vector3 MotorControl::get_angular(){
	return angular;
}

void MotorControl::set_state(rc_state_t new_state){
    rc_set_state(new_state);
}

rc_state_t MotorControl::get_state(){
    return rc_get_state();
}

void MotorControl::publish_state(){

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

void MotorControl::status_callback(const actionlib_msgs::GoalStatus::ConstPtr& msg){
    if( msg->status == msg->ABORTED ){
        clean_up();
    }
}

/**
 * This call back will convert the linear components of the message read from
 * the topic in to movement of the rover. The following diagram illustrates how
 * moving a joystick will move the rover:
 * 
 *            XFFFFFX    F: forward
 *            LXFFFXR    R: right
 *            LLXFXRR    L: left
 *            LLLXRRR    B: backward
 *            LLXBXRR    X: no movement
 *            LXBBBXR
 *            XBBBBBX
 */
void MotorControl::callback(const geometry_msgs::Twist::ConstPtr& msg){
    /* forward: +linear.x; backward: -linear.x; left: angular.z; right:
     * -anguler.z */
    linear = msg->linear;
    angular = msg->angular;

    /* if the linear component is greater then it will be used to set the
     * direction */
    if(abs(linear.x) > abs(angular.z)){
        rc_set_motor_all(linear.x);
    } else if(abs(linear.x) < abs( angular.z)){
        /* if the angular component is greater then it will be used to determine
         * the direction */
        rc_set_motor(1, -angular.z);
        rc_set_motor(2, -angular.z);
        rc_set_motor(3, angular.z);
        rc_set_motor(4, angular.z);
    } else {
        /* do not move */
    }
}

}; // rover namespace

