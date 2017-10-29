/***************************************************************************
  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#ifndef __L3GD20_H__
#define __L3GD20_H__

#include <stdint.h>
#include "mraa.hpp"
using namespace mraa;

namespace rover {

/* Defines */
#define PIN_COUNT 5
#define PWM_COUNT 2
#define DUTY_CYCLE 0.5
#define HIGH 1
#define LOW 0
/* define/declare default pin */
#define PWMA_LEFT_PIN 36
#define AIN1_LEFT_PIN 38
#define AIN2_LEFT_PIN 37
#define BIN1_LEFT_PIN 40
#define STDBY_LEFT_PIN 35
#define BIN2_LEFT_PIN 33
#define PWMB_LEFT_PIN 34
#define STDBY_LEFT_INDX 4
#define AIN1_LEFT_INDX 2
#define AIN2_LEFT_INDX 3
#define BIN1_LEFT_INDX 1
#define BIN2_LEFT_INDX 0
/* PWM indeces */
#define PWMA_LEFT_INDX 0
#define PWMB_LEFT_INDX 1

/* Right side pins */
#define PWMA_RIGHT_PIN 45
#define AIN1_RIGHT_PIN 44
#define AIN2_RIGHT_PIN 39
#define BIN1_RIGHT_PIN 42
#define STDBY_RIGHT_PIN 43
#define BIN2_RIGHT_PIN 41
#define PWMB_RIGHT_PIN 46
#define STDBY_RIGHT_INDX 4
#define AIN1_RIGHT_INDX 2
#define AIN2_RIGHT_INDX 3
#define BIN1_RIGHT_INDX 1
#define BIN2_RIGHT_INDX 0
/* right PWM indeces */
#define PWMA_RIGHT_INDX 0
#define PWMB_RIGHT_INDX 1

class MotorControl
{
public:

	MotorControl(unsigned int *p_front, unsigned int *p_rear);

	int turn_left();

	int turn_right();

	int stop();

	int forward();

	int backward();

	int set_speed();

	int get_speed();

	int set_default_pins();

	int clean_up();

    virtual ~MotorControl();
private:
	int curr_speed;
	mraa::Gpio* pins_right[PIN_COUNT];
	mraa::Gpio* pins_left[PIN_COUNT];
//	mraa::Pwm*  pwm_left[PWM_COUNT];
//	mraa::Pwm*  pwm_right[PWM_COUNT];
	mraa::Result res_right[PIN_COUNT];
	mraa::Result res_left[PIN_COUNT];
};

} // rover namespace
#endif
