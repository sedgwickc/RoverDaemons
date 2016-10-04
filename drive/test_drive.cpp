/* MotorControl_test.cpp
 * Charles Sedgwick
 * licence: GPLv3
 */

#include <iostream>
#include "MotorControl.hpp"
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std;
using namespace rover;

int main( int argc, char** argv ) {

    /* create MotorControl object and pass null to have it set up default pins
     */
	MotorControl* mc = new MotorControl(NULL,NULL);
	//testing
	/* Investigate why test_drive dies and ssh connection lost when turn_right
	 * and turn_left called in sequence with only sleep(5) call between them 
	 */
	mc->forward();
	sleep(5);
	mc->turn_right();
	sleep(5);
	mc->backward();
	sleep(5);
	mc->turn_left();
	sleep(5);
	mc->stop();
	sleep(5);
	mc->clean_up();
    return 0;
}
