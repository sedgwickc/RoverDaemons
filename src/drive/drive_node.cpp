/* drive_node.cpp
 * Charles Sedgwick
 * licence: GPLv3
 */

#include <iostream>
#include "MotorControl.hpp"
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <rc_usefulincludes.h> 
#include <roboticscape.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace std;
using namespace rover;

void speedCallback(const std_msgs::Float32::ConstPtr& msg, 
        MotorControl motor_control){
    /* if they differ from current values then set speed and diff to new */
    if( msg->data != motor_control.get_speed() ){
        motor_control.set_speed(msg->data);
    }

    /* publish new speed to /drive_status */

}

void diffCallback(const std_msgs::Float32::ConstPtr& msg,
        MotorControl motor_control){
    /* if they differ from current values then set speed and diff to new */
    if( msg->data != motor_control.get_diff() ){
        motor_control.set_diff(msg->data);
    }

    /* publish new diff to /drive_status */

}

int main( int argc, char** argv ) {

    MotorControl motor_control = new MotorControl();

    ros::init(argc, argv, "drive");
	
    ros::NodeHandle n;

    /* subscribe to /drive_speed and /drive_diff */
    ros::Subscriber speedSub = n.subscribe("drive_speed", 1000, 
        boost::bind(speedCallback, _1, motor_control));
    ros::Subscriber diffSub = n.subscribe("diff_speed", 1000, 
        boost::bind(diffCallback, _1, motor_control));

    ros::spin();

	// exit cleanly
    delete(motor_control);

	return 0;
}
