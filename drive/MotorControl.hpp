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
#define PIN_COUNT 4
#define HIGH 1
#define LOW 0
/* define/declare default pin */
#define AIN1_LEFT_PIN 38
#define AIN2_LEFT_PIN 37
#define BIN1_LEFT_PIN 36
#define BIN2_LEFT_PIN 34
#define AIN1_LEFT_INDX 2
#define AIN2_LEFT_INDX 3
#define BIN1_LEFT_INDX 1
#define BIN2_LEFT_INDX 0


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
	mraa::Result res_right[PIN_COUNT];
	mraa::Result res_left[PIN_COUNT];
};

} // rover namespace
#endif
