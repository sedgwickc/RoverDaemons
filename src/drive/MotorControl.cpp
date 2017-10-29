/***************************************************************************
 * Charles Sedgwick
 * MotorControl.cpp
 * The MotorControl class is reponsible for controlling the direction and speed
 * of the 4 motors. 
 *
 * Changlog
 * 
 * Ver    Date       User      Issue #  Change
 * -----------------------------------------------------------------------------
 * 100    25sep2015  sedgwickc          Initial creation. 
 * 101    08nov2016  sedgwickc          Implement control of all motors. 
 *                                      Remove handling of PWM pins as mraa
 *                                      does not support this atm.
 ***************************************************************************/

#include "MotorControl.hpp"
#include <iostream>

using namespace std;

namespace rover {

/***************************************************************************
 * CONSTRUCTOR
 * @brief  Instantiates a new motor control class
 *************************************************************************/
MotorControl::MotorControl(unsigned int *p_left, unsigned int *p_right)
{
	if( p_left == NULL || p_right == NULL){
		cout<<"No pins passed in... \nUsing defaults..."<<endl;
		set_default_pins();
    	/* Set PWM and STDBY pins to high otherwise motors sent brake signal */
		res_left[STDBY_LEFT_INDX] = pins_left[STDBY_LEFT_INDX]->write(HIGH);
		res_right[STDBY_RIGHT_INDX] = pins_right[STDBY_RIGHT_INDX]->write(HIGH);
		return;
	}
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

int MotorControl::turn_right()
{
	stop();
	cout<<"Turning right..."<<endl;
	res_left[BIN1_LEFT_INDX] = pins_left[BIN1_LEFT_INDX]->write(HIGH);
	res_left[AIN2_LEFT_INDX] = pins_left[AIN2_LEFT_INDX]->write(HIGH);
	res_right[BIN1_RIGHT_INDX] = pins_right[BIN1_RIGHT_INDX]->write(HIGH);
	res_right[AIN2_RIGHT_INDX] = pins_right[AIN2_RIGHT_INDX]->write(HIGH);
	return 1;
}

int MotorControl::turn_left()
{
	stop();
	cout<<"Turning left..."<<endl;
	res_left[BIN2_LEFT_INDX] = pins_left[BIN2_LEFT_INDX]->write(HIGH);
	res_left[AIN1_LEFT_INDX] = pins_left[AIN1_LEFT_INDX]->write(HIGH);
	res_right[BIN2_RIGHT_INDX] = pins_right[BIN2_RIGHT_INDX]->write(HIGH);
	res_right[AIN1_RIGHT_INDX] = pins_right[AIN1_RIGHT_INDX]->write(HIGH);
	return 1;
}

int MotorControl::stop()
{
	cout<<"Stopping all motors..."<<endl;
	/* minus 2 so that stdby pins are not touched*/
	for( int i = 0; i < PIN_COUNT; i++ )
	{
	    /* Do no set standby pins to low. 
	     * Standby pins share same index into pin arrays.
	     */
	    if( i != STDBY_RIGHT_INDX ){
			res_left[i] = pins_left[i]->write(LOW);
			res_right[i] = pins_right[i]->write(LOW);
		}
	}
	return 1;
}

int MotorControl::forward()
{
	stop();
	cout<<"Setting motor direction forward..."<<endl;
	/* check state */
	/* set pins */
	res_left[BIN2_LEFT_INDX] = pins_left[BIN2_LEFT_INDX]->write(HIGH);
	res_left[AIN2_LEFT_INDX] = pins_left[AIN2_LEFT_INDX]->write(HIGH);
	res_right[BIN2_RIGHT_INDX] = pins_right[BIN2_RIGHT_INDX]->write(HIGH);
	res_right[AIN2_RIGHT_INDX] = pins_right[AIN2_RIGHT_INDX]->write(HIGH);
	return 1;
}

int MotorControl::backward()
{
	stop();
	cout<<"Moving backward..."<<endl;
	res_left[BIN1_LEFT_INDX] = pins_left[BIN1_LEFT_INDX]->write(HIGH);
	res_left[AIN1_LEFT_INDX] = pins_left[AIN1_LEFT_INDX]->write(HIGH);
	res_right[BIN1_RIGHT_INDX] = pins_right[BIN1_RIGHT_INDX]->write(HIGH);
	res_right[AIN1_RIGHT_INDX] = pins_right[AIN1_RIGHT_INDX]->write(HIGH);
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
	/* bin2 */
	cout<<"Setting up BIN1_LEFT..."<<endl;
	pins_left[BIN2_LEFT_INDX] = new mraa::Gpio(BIN2_LEFT_PIN);
	if (pins_left[BIN2_LEFT_INDX] == NULL) {
	    return mraa::ERROR_UNSPECIFIED;
	}

	/* bin1 */
	cout<<"Setting up BIN1_LEFT..."<<endl;
	pins_left[BIN1_LEFT_INDX] = new mraa::Gpio(BIN1_LEFT_PIN);
	if (pins_left[BIN1_LEFT_INDX] == NULL) {
	    return mraa::ERROR_UNSPECIFIED;
	}

	/* AIN1 */
	cout<<"Setting up AIN1_LEFT..."<<endl;
	pins_left[AIN1_LEFT_INDX] = new mraa::Gpio(AIN1_LEFT_PIN);
	if (pins_left[AIN1_LEFT_INDX] == NULL) {
	    return mraa::ERROR_UNSPECIFIED;
	}

	/* AIN2 */
	cout<<"Setting up AIN2_LEFT..."<<endl;
	pins_left[AIN2_LEFT_INDX] = new mraa::Gpio(AIN2_LEFT_PIN);
	if (pins_left[AIN2_LEFT_INDX] == NULL) {
	    return mraa::ERROR_UNSPECIFIED;
	}
	
	/* STDBY */
	cout<<"Setting up STDBY_LEFT..."<<endl;
	pins_left[STDBY_LEFT_INDX] = new mraa::Gpio(STDBY_LEFT_PIN);
	if (pins_left[STDBY_LEFT_INDX] == NULL) {
	    return mraa::ERROR_UNSPECIFIED;
	}

	/* PWMB */
//	cout<<"Setting up PWMB_LEFT..."<<endl;
//    pwm_left[PWMB_LEFT_INDX] = new mraa::Pwm(PWMB_LEFT_PIN);
//	if (pwm_left[PWMB_LEFT_INDX] == NULL) {
//	    return mraa::ERROR_UNSPECIFIED;
//	}
//    pwm_left[PWMB_LEFT_INDX]->enable(true);

	/* PWMA */
//	cout<<"Setting up PWMA_LEFT..."<<endl;
//	pwm_left[PWMA_LEFT_INDX] = new mraa::Pwm(PWMA_LEFT_PIN);
//	if (pwm_left[PWMA_LEFT_INDX] == NULL) {
//	    return mraa::ERROR_UNSPECIFIED;
//	}
//    pwm_left[PWMA_LEFT_INDX]->enable(true);

	/*create right side pins */
	/* bin2 */
	cout<<"Setting up BIN1_RIGHT..."<<endl;
	pins_right[BIN2_RIGHT_INDX] = new mraa::Gpio(BIN2_RIGHT_PIN);
	if (pins_right[BIN2_RIGHT_INDX] == NULL) {
	    return mraa::ERROR_UNSPECIFIED;
	}

	/* bin1 */
	cout<<"Setting up BIN1_RIGHT..."<<endl;
	pins_right[BIN1_RIGHT_INDX] = new mraa::Gpio(BIN1_RIGHT_PIN);
	if (pins_right[BIN1_RIGHT_INDX] == NULL) {
	    return mraa::ERROR_UNSPECIFIED;
	}

	/* AIN1 */
	cout<<"Setting up AIN1_RIGHT..."<<endl;
	pins_right[AIN1_RIGHT_INDX] = new mraa::Gpio(AIN1_RIGHT_PIN);
	if (pins_right[AIN1_RIGHT_INDX] == NULL) {
	    return mraa::ERROR_UNSPECIFIED;
	}

	/* AIN2 */
	cout<<"Setting up AIN2_RIGHT..."<<endl;
	pins_right[AIN2_RIGHT_INDX] = new mraa::Gpio(AIN2_RIGHT_PIN);
	if (pins_right[AIN2_RIGHT_INDX] == NULL) {
	    return mraa::ERROR_UNSPECIFIED;
	}
	
	/* STDBY */
	cout<<"Setting up STDBY_RIGHT..."<<endl;
	pins_right[STDBY_RIGHT_INDX] = new mraa::Gpio(STDBY_RIGHT_PIN);
	if (pins_right[STDBY_RIGHT_INDX] == NULL) {
	    return mraa::ERROR_UNSPECIFIED;
	}

	/* PWMB */
//	cout<<"Setting up PWMB_RIGHT..."<<endl;
//	pwm_right[PWMB_RIGHT_INDX] = new mraa::Pwm(PWMB_RIGHT_PIN);
//	if (pwm_right[PWMB_RIGHT_INDX] == NULL) {
//	    return mraa::ERROR_UNSPECIFIED;
//	}
//    pwm_right[PWMB_RIGHT_INDX]->enable(true);

	/* PWMA */
//	cout<<"Setting up PWMA_RIGHT..."<<endl;
//	pwm_right[PWMA_RIGHT_INDX] = new mraa::Pwm(PWMA_RIGHT_PIN);
//	if (pwm_right[PWMA_RIGHT_INDX] == NULL) {
//	    return mraa::ERROR_UNSPECIFIED;
//	}
//    pwm_right[PWMA_RIGHT_INDX]->enable(true);

	//set pin direction to output
	for( int i = 0; i < PIN_COUNT; i++ ){
	    /* set left pins direction */	
		res_left[i] = pins_left[i]->dir(mraa::DIR_OUT);
		if(res_left[i] != mraa::SUCCESS){
			mraa::printError(res_left[i]);
			return 1;
		}
		/* set right pins direction */
		res_right[i] = pins_right[i]->dir(mraa::DIR_OUT);
		if(res_right[i] != mraa::SUCCESS){
			mraa::printError(res_right[i]);
			return 1;
		}
	}

	/* set up pwm duty cycle */
//	pwm_right[PWMA_RIGHT_INDX]->write(DUTY_CYCLE);
//	pwm_right[PWMB_RIGHT_INDX]->write(DUTY_CYCLE);
//	pwm_left[PWMA_LEFT_INDX]->write(DUTY_CYCLE);
//	pwm_left[PWMB_LEFT_INDX]->write(DUTY_CYCLE);

	return 1;
}

int MotorControl::clean_up()
{
	for(int i = 0; i < PIN_COUNT; i++){
		delete pins_left[i];
		delete pins_right[i];
	}
	/*
    delete pwm_right[PWMA_RIGHT_INDX];
    delete pwm_right[PWMB_RIGHT_INDX];
    delete pwm_left[PWMA_LEFT_INDX];
    delete pwm_left[PWMB_LEFT_INDX];
    */

	return 1;
}

MotorControl::~MotorControl(){}

}; // rover namespace

