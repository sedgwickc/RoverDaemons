/*
 * Control package
 * 
 * This package manages the basic behaviour of the rover.
 *
 */

extern "C"
{  
#include "roboticscape.h"
}
// main roboticscape API header
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <actionlib_msgs/GoalStatus.h>

#define CLOCK 1
#define ANTICLOCK -1
#define SERVO_MAX 1.5
#define FREQ_DEFAULT 90

using namespace std;

void range_callback(const sensor_msgs::LaserScan::ConstPtr& msg);

/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(int argc, char** argv){

    uint16_t distance = 0;

    ros::init(argc, argv, "control");
	
    ros::NodeHandle n;  

    /* subscribe to rover status topic */

    /* subscribe to laser_range topic */
    ros::Subscriber sub = n.subscribe("range_laser", 180, range_callback);

    ros::spin();
	
	// exit cleanly
	return 0;
}

/* range finder callback
 * 
 * Analyzes the range and angle to determine if it is safe to continue current
 * action
 */
void range_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
    fprintf(stdout,"Angle: %f; Distance: %f\n)", msg->angle_min, msg->range_min);
}

void status_callback(const actionlib_msgs::GoalStatus::ConstPtr& msg){
    if( msg->status == msg->ABORTED ){
        rc_i2c_release_bus(1);
        rc_disable_servo_power_rail();
        rc_cleanup(); 
    }
}

