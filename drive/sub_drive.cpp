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
	//int res = fork();
	int res = 0; // spoof successful fork for testing.

	// check PID
	if ( res < 0 ){
		exit(EXIT_FAILURE);
	} if ( res > 0 ) {
		return 0;
	}

	MotorControl* mc = new MotorControl(NULL,NULL);
	// open drive_data channel 

	Rover_ACH *drive_data = new Rover_ACH();

    drive_data->opt_sub = 1;
    drive_data->opt_chan_name = "drive_data";
    // test sending a MAV message rather than getting messages from stdin
    //drive_data->fin = stdin;
    //drive_data->fout = stdout;

    if(  drive_data->opt_pub ==  drive_data->opt_sub ) {
        fprintf(stderr, "Error: cannot publish and subscribe\n");
        exit(EXIT_FAILURE);
    }

    drive_chan->chnl.open( drive_data->opt_chan_name );

    //get status of motors, speed, power usage?

	// open data_cmd channel
	Rover_ACH *drive_cmd = new Rover_ACH();

    drive_cmd->opt_sub = 1;
    drive_cmd->opt_chan_name = "drive_data";
    // create buffer to store cmd from control daemon in

    // while no new cmd has been published by control daemon, publish status
    // call RoverACH function to get latest cmd
	while(1){
		publish(&channel);
	}
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
