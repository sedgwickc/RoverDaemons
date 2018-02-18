/***************************************************************************
 * MotorControl.hpp
 * Charles Segdwick
 * Defines the class members got MotorControl which is used to control a rover
 * with up to 4 motors (teo on each side)
 ***************************************************************************/

#include <stdint.h>

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

	int set_speed(double pDuty);

	double get_speed();
    void set_state(rc_state_t new_state);
    rc_state_t get_state();

	int clean_up();
    void MotorControl::on_pause_released();
    void MotorControl::on_pause_pressed();

    virtual ~MotorControl();
private:
	double duty;
    int diff;
};

} // rover namespace
#endif
