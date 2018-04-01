/*
 * Range_Finder package
 * 
 * This package retrieves and publishes distance data from a lidar lite v2 
 * mounted on a servo which sweeps from FREQ_DEFAULT to -FREQ_DEFAULT.  
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
#define SERVO_MAX 1.35
#define FREQ_DEFAULT 90

using namespace std;

/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(int argc, char** argv){

    /* the distance value read from the sensor */
    uint16_t distance = 0;

    /* position of the servo */
    float position = 0.0;

    /* angle of the servo at a current position */
    float angle = 0.0;

    /* the direction the servo is currently sweeping */ 
    int8_t swp_drctn = CLOCK;

	// always initialize cape library first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to initialize rc_initialize(), are you root?\n");
		return -1;
	}

	if(rc_i2c_get_in_use_state(1)){
		fprintf(stderr,"ERROR: i2c bus claimed by another process\n");
        return -1;
    }

	if(rc_i2c_init(1,0x62)<0){
		fprintf(stderr,"ERROR: failed to initialize i2c bus\n");
		return -1;
	}
    rc_i2c_claim_bus(1);

    if( rc_enable_servo_power_rail()<0){
		fprintf(stderr, "ERROR: failed to initialize servo power rail\n");
		return -1;
    }
    
    ros::init(argc, argv, "range_finder");
	
    ros::NodeHandle n;

    /* advertise the topic "range_laser" and buffer up to 10 messages */
    ros::Publisher range_pub = n.advertise<sensor_msgs::LaserScan>("range_laser", 10);

    /* loop at FREQ_DEFAULT Hz */
    ros::Rate loop_rate(FREQ_DEFAULT);

    if( rc_send_servo_pulse_normalized(7, position) < 0){
        fprintf(stderr,"ERROR: failed to move to servo to position %f\n", position);
    }
    rc_set_state(RUNNING);
    while( rc_get_state() == RUNNING && ros::ok()){
        position += swp_drctn * SERVO_MAX / FREQ_DEFAULT;
        if(swp_drctn == CLOCK ){
            angle++;
        }else{
            angle--;
        }

        if( position > SERVO_MAX || angle == FREQ_DEFAULT){
            position = SERVO_MAX;
            swp_drctn = ANTICLOCK;
        }
        if (position < -SERVO_MAX || angle == -FREQ_DEFAULT){
            position = -SERVO_MAX;
            swp_drctn = CLOCK;
        }
            
        if( rc_send_servo_pulse_normalized(7, position) < 0){
            fprintf(stderr,"ERROR: failed to move to servo to position %f\n", position);
            break;
        }

        distance = 0;
        if(rc_i2c_write_byte(1, 0x00, 0x04)<0){
            fprintf(stderr,"ERROR: failed to write to range finder\n");
            rc_set_state(EXITING);
        } else {
            rc_i2c_read_word(1, 0x8f, &distance);
            /* publish servo position and distance */
            sensor_msgs::LaserScan msg;
            msg.header.stamp = ros::Time::now();
            msg.angle_min = angle;
            msg.range_min = distance;
            range_pub.publish(msg);
        }
        ros::spinOnce();

        loop_rate.sleep();
	}
	
	// exit cleanly
    rc_i2c_release_bus(1);
    rc_disable_servo_power_rail();
	rc_cleanup(); 
	return 0;
}

void status_callback(const actionlib_msgs::GoalStatus::ConstPtr& msg){
    if( msg->status == msg->ABORTED ){
        rc_i2c_release_bus(1);
        rc_disable_servo_power_rail();
        rc_cleanup(); 
    }
}

