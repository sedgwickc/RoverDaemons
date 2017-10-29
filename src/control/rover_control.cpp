/*
 *
 * Github: https://github.com/sedgwickc/RoverDaemons/
 * 
 * Changlog
 * 
 * Ver    Date       User      Issue #  Change
 * -----------------------------------------------------------------------------
 * 100    25sep2015  sedgwickc           Initial creation. 
 * 101    08nov2016  sedgwickc           Implement printing of 100 frames from 
 *                                   sub_nav daemon
 *!!!!!!!!!!!!!!!!!!!!!!!!!!!UPDATE VERSION VARIABLE!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <inttypes.h>
#include "../../RoverACH/RoverACH.hpp"
#include "rover_control.hpp"

using namespace rover;
using namespace std;

int main( int argc, char **argv ) {
    /* TODO: Allow user to specify via command line whether data recieved over channels
     * should be logged
     */

    /* TODO: start other daemons */
    start_navigation();
    start_drive();

	/* TODO: create a comms object */

	/* make sure connected to qgroundcontrol open */
	
	/* TODO: for each subscribe_channel, create a new RoverACH object, open the
	 * channel and subscribe to it
	 */
	RoverACH *nav_data = new RoverACH();

    /* setup nav_data channel to receive data to be stored in curr_nav_data */
	nav_data->opt_pub = 0;
	nav_data->opt_sub = 1;
	nav_data->opt_buffer_type = TYPE_NAV_DATA;
	strncpy(nav_data->opt_chan_name, "nav_data", NAME_SIZE);
	nav_data->fin = stdin;
	nav_data->fout = stdout;    	

	nav_data->chnl->open( nav_data->opt_chan_name );

    /* print 100 frames of data sent by the navigation sub system */
	for( int i = 0; i < 100; i++ ){
	    nav_data->getFrame();
	    printf("Nav Data frame #%d recieved:\n", i);
        printf("    gyro_x: %f\n", nav_data->curr_nav_data.gyro_x);
        printf("    gyro_y: %f\n", nav_data->curr_nav_data.gyro_y );
        printf("    gyro_z: %f\n", nav_data->curr_nav_data.gyro_z );
	    printf("    mag_x: %f\n", nav_data->curr_nav_data.mag_x );
	    printf("    mag_y: %f\n", nav_data->curr_nav_data.mag_y );
	    printf("    mag_z: %f\n", nav_data->curr_nav_data.mag_z );
	    printf("    acc_x: %f\n", nav_data->curr_nav_data.acc_x );
	    printf("    acc_y: %f\n", nav_data->curr_nav_data.acc_y );
	    printf("    acc_z: %f\n", nav_data->curr_nav_data.acc_z );
	    printf("    pressure: %f\n",nav_data->curr_nav_data.pressure );
	    printf("    temperature: %f\n",nav_data->curr_nav_data.temperature );

	    printf("%s\n", nav_data->pbuffer);
	    usleep(100000);
	}


	/* TODO: send GPS location to qgroundcontrol */
	
	/* for each publish channel, create a new RoverACH object, open the channel
	 * and set it to publish
	 */

	nav_data->chnl->close();
	return 0;

}

void start_navigation(){

}

void start_drive(){

}

/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/* Local Variables:                          */
/* mode: c++                                 */
/* indent-tabs-mode:  nil                    */
/* c-basic-offset: 4                         */
/* c-file-offsets: ((innamespace . 0))       */
/* End:                                      */
