/***************************************************************************
 * Charles Sedgwick
 * MotorControl.cpp
 * The MotorControl class is reponsible for controlling the direction and speed
 * of the 4 motors. 
 ***************************************************************************/

#include "MotorControl.hpp"
#include <iostream>

using namespace std;

namespace rover {

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/
 
/**************************************************************************/
/*!
    @brief  Instantiates a new motor control class
*/
/**************************************************************************/
MotorControl::MotorControl(unsigned int *p_left, unsigned int *p_right)
{
	if( p_left == NULL || p_right == NULL){
		cout<<"No pins passed in... \nUsing defaults..."<<endl;
		set_default_pins();
		return;
	}

	/* Set up GPIO pins if not 
	for( int i = 0; i < 4; i++)
	{
		pins_left[i] = new mraa::Gpio(p_left[i]);
		res_left[i] = pins_left[i]->dir(mraa::DIR_OUT);

		pins_right[i] = new mraa::Gpio(p_right[i]);
		res_right[i] = pins_right[i]->dir(mraa::DIR_OUT);
	}*/
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/

int MotorControl::turn_right()
{
	return 1;
}

int MotorControl::turn_left()
{
	return 1;
}

int MotorControl::stop()
{
	cout<<"Stopping all motors..."<<endl;
	for( int i = 0; i < PIN_COUNT; i++ )
	{
		res_left[i] = pins_left[i]->write(LOW);
		res_right[i] = pins_right[i]->write(LOW);
	}
	return 1;
}

int MotorControl::forward()
{
	stop();
	cout<<"Setting motor direction forward..."<<endl;
	// check state
	//set pins
	res_left[0] = pins_left[0]->write(HIGH);
	res_left[2] = pins_left[2]->write(HIGH);
	res_right[0] = pins_right[0]->write(HIGH);
	res_right[2] = pins_right[2]->write(HIGH);
	return 1;
}

int MotorControl::backward()
{
	stop();
	res_left[1] = pins_left[1]->write(HIGH);
	res_left[3] = pins_left[3]->write(HIGH);
	res_right[1] = pins_right[1]->write(HIGH);
	res_right[3] = pins_right[3]->write(HIGH);
	return 1;
}

int MotorControl::set_speed()
{
	return 1;
}

int MotorControl::get_speed()
{
	return 1;
}

int MotorControl::set_default_pins()
{
	cout<<"Setting default pins..."<<endl;
	//create left side pins
	pins_left[0] = new mraa::Gpio(40);
	if (pins_left[0] == NULL) {
	    return mraa::ERROR_UNSPECIFIED;
	}
	pins_left[1] = new mraa::Gpio(42);
	if (pins_left[1] == NULL) {
	    return mraa::ERROR_UNSPECIFIED;
	}
	pins_left[2] = new mraa::Gpio(44);
	if (pins_left[2] == NULL) {
	    return mraa::ERROR_UNSPECIFIED;
	}
	pins_left[3] = new mraa::Gpio(46);
	if (pins_left[3] == NULL) {
	    return mraa::ERROR_UNSPECIFIED;
	}
	
	// create right side pins
	pins_right[0] = new mraa::Gpio(45);
	if (pins_right[0] == NULL) {
	    return mraa::ERROR_UNSPECIFIED;
	}
	pins_right[1] = new mraa::Gpio(43);
	if (pins_right[1] == NULL) {
	    return mraa::ERROR_UNSPECIFIED;
	}
	pins_right[2] = new mraa::Gpio(41);
	if (pins_right[2] == NULL) {
	    return mraa::ERROR_UNSPECIFIED;
	}
	pins_right[3] = new mraa::Gpio(39);
	if (pins_right[3] == NULL) {
	    return mraa::ERROR_UNSPECIFIED;
	}

	//set pin direction
	for( int i = 0; i < PIN_COUNT; i++ ){
		res_left[i] = pins_left[i]->dir(mraa::DIR_OUT);
		if(res_left[i] != mraa::SUCCESS){
			mraa::printError(res_left[i]);
			return 1;
		}
		// add right pins
		res_right[i] = pins_right[i]->dir(mraa::DIR_OUT);
		if(res_right[i] != mraa::SUCCESS){
			mraa::printError(res_right[i]);
			return 1;
		}
	}
	return 1;
}

int MotorControl::clean_up()
{
	for(int i = 0; i < PIN_COUNT; i++){
		delete pins_left[i];
		delete pins_right[i];
	}
	return 1;
}

MotorControl::~MotorControl(){}

}; // rover namespace

