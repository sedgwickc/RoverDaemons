/***************************************************************************
 * MotorControl.hpp
 * Charles Segdwick
 * Defines the class members got MotorControl which is used to control a rover
 * with up to 4 motors (teo on each side)
 ***************************************************************************/

#include <stdint.h>
extern "C"
{  
#include "roboticscape.h"
}
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

namespace rover {

/* Defines */
#define DEFAULT_DUTY_CYCLE 0.5

class MotorControl
{
public:
	MotorControl();

	int turn_left();
	int turn_right();
	int stop();
	int forward();
	int backward();
	void set_speed(double pDuty);
	double get_speed();
	void set_diff(int pDiff);
	int get_diff();
    void set_state(rc_state_t new_state);
    rc_state_t get_state();

	int clean_up();
    void on_pause_released();
    void on_pause_pressed();
    void speedCallback(const std_msgs::Float32::ConstPtr& msg);
    void diffCallback(const std_msgs::Int32::ConstPtr& msg);
    virtual ~MotorControl();
private:
	double duty;
    int diff;
};

} // rover namespace
