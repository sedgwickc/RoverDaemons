/*
 *
 * Github: https://github.com/sedgwickc/RoverDaemons/
 * 
 * Changlog
 * 
 * Ver    Date       User   Issue #  Change
 * -----------------------------------------------------------------------------
 * 100    25sep2015  cwick           Initial creation. 
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

using namespace rover;
using namespace std;

int main( int argc, char **argv ) {
    /* Allow user to specify via command line that data recieved over channels
     * should be output to stdout
     */

	// change to vector
	//string[] subscribe_channel = { "nav_data" };

	/* create a comms object */

	/* make sure connected to qgroundcontrol open */
	
	/* for each subscribe_channel, create a new RoverACH object, open the
	 * channel and subscribe to it
	 */
	RoverACH *nav_data = new RoverACH();

	nav_data->opt_pub = 0;
	nav_data->opt_sub = 1;
	strncpy(nav_data->opt_chan_name, "nav_data", NAME_SIZE);
	nav_data->fin = stdin;
	nav_data->fout = stdout;    	

	nav_data->chnl->open( nav_data->opt_chan_name );

	nav_data->subscribe();

	/* send GPS location to qgroundcontrol */
	
	/* for each publish channel, create a new RoverACH object, open the channel
	 * and set it to publish
	 */

	return 0;

}


/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/* Local Variables:                          */
/* mode: c++                                 */
/* indent-tabs-mode:  nil                    */
/* c-basic-offset: 4                         */
/* c-file-offsets: ((innamespace . 0))       */
/* End:                                      */
