/* drive_node.cpp
 * Charles Sedgwick
 * licence: GPLv3
 */

#include "MotorControl/MotorControl.hpp"
extern "C"
{  
#include "roboticscape.h"
}
#include <ros/ros.h>

using namespace std;
using namespace rover;


int main( int argc, char** argv ) {

    MotorControl *motor_control = new MotorControl();

    ros::init(argc, argv, "drive");
	
    ros::NodeHandle n;

    /* subscribe to /drive_speed and /drive_diff */
    ros::Subscriber speedSub = n.subscribe("cmd_vel", 5, 
        &MotorControl::callback, motor_control);

    ros::spin();

	// exit cleanly
    free(motor_control);

	return 0;
}
