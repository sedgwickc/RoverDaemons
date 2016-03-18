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
#include "../Rover_ACH.hpp"

using namespace std;
using namespace rover;

int main( int argc, char** argv ) {

	// fork()
	int res = fork();

	// check PID
	if ( res < 0 ){
		exit(EXIT_FAILURE);
	} if ( res > 0 ) {
		return 0;
	} else {
		MotorControl* mc = new MotorControl(NULL,NULL);
		// open ach channel 

		Rover_ACH *drive_chan = new Rover_ACH();

        drive_chan->opt_sub = 1;
        drive_chan->opt_chan_name = "drive_chan";
        // test sending a MAV message rather than getting messages from stdin
    	drive_chan->fin = stdin;
    	drive_chan->fout = stdout;

    	}
    	if(  drive_chan->opt_pub ==  drive_chan->opt_sub ) {
        	fprintf(stderr, "Error: cannot publish and subscribe\n");
        	exit(EXIT_FAILURE);
    	}

    	ach::CerrChannel channel;
    	channel.open( drive_chan->opt_chan_name );
    	//how does subscribe work?
		subscribe(&channel);
    	assert( 0 );
    	return 0;

		//testing
		/*
		mc->forward();
		sleep(5);
		mc->stop();
		sleep(5);
		mc->backward();
		sleep(5);
		mc->stop();
		mc->clean_up();
		*/
	} 
}
