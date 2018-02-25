/* drive_node.cpp
 * Charles Sedgwick
 * licence: GPLv3
 */

#include "MotorControl/MotorControl.hpp"
extern "C"
{  
#include "roboticscape.h"
}
#include <std_msgs/Float32.h>
#include <ros/ros.h>

using namespace std;
using namespace rover;


int main( int argc, char** argv ) {

    MotorControl *motor_control = new MotorControl();

    ros::init(argc, argv, "drive");
	
    ros::NodeHandle n;

    /* subscribe to /drive_speed and /drive_diff */
    ros::Subscriber speedSub = n.subscribe("drive_speed", 1000, 
        &MotorControl::speedCallback, motor_control);
    ros::Subscriber diffSub = n.subscribe("drive_direction", 1000, 
        &MotorControl::diffCallback, motor_control);

    ros::spin();

	// exit cleanly
    free(motor_control);

	return 0;
}
