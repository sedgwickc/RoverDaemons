/* MotorControl_test.cpp
 * Charles Sedgwick
 * licence: GPLv3
 */

#include <iostream>
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h> //umask()
#include "../RoverACH/RoverACH.hpp"
#include "BBB_BMP180/Adafruit_BMP180.hpp"

#define BUFF_SIZE 4096
#define NUM_SIZE 128

using namespace std;
using namespace rover;

int main( int argc, char** argv ) {
	// fork()
	int res = fork();

	// check PID
	if ( res < 0 ){
		exit(EXIT_FAILURE);
	}
	if ( res > 0 ) {
		cout<<"Fork successful. Child created with PID "<<res<<endl;
		exit(0);
	} 

	FILE* fp_log = fopen("/home/cwick/rover/logs/nav_log.txt", "w+");
	if( fp_log == NULL ){
		cerr<<"Could not open log file"<<endl;
		return -1;
	}

	fprintf(fp_log, "Starting navigation daemon...\n");
	umask(0);
	int sid = setsid();
	if( sid < 0 ){
		fprintf(fp_log, "Failed to set SID. Exiting\n");
		exit(1);
	}

	close(STDIN_FILENO);
	close(STDOUT_FILENO);
	close(STDERR_FILENO);

	fflush(fp_log);

	// open nav data channel
	fprintf(fp_log, "Opening nav_data channel.\n");
	fflush(fp_log);
	RoverACH* nav_data = new RoverACH();

    nav_data->opt_sub = 0;
    nav_data->opt_pub = 1;
    strncpy(nav_data->opt_chan_name, "nav_data", NAME_SIZE);
    nav_data->fin = stdin;
    nav_data->fout = stdout;

    if(  nav_data->opt_sub == 1 && nav_data->opt_pub ==  nav_data->opt_sub ) {
        fprintf(fp_log, "Error: cannot publish and subscribe\n");
        exit(EXIT_FAILURE);
    }
    
    nav_data->chnl.open( nav_data->opt_chan_name );

	fprintf(fp_log, "Setting up sensor BMP180...");
	fflush(fp_log);
    Adafruit_BMP180 BMP180(1,0x77);
    if( BMP180.begin() == false){
            fprintf(fp_log,"Could not find a valid BMP180!\n");
			fflush(fp_log);
            return 0;
    }

    float* temp = (float*)calloc(1,sizeof(float));
    float* pressure = (float*)calloc(1, sizeof(float));
    char num_buff[NUM_SIZE];
  
	// TODO: add condition to check that RoverControl process hasn't sent a stop signal 
	// Continually publish sensor readings to channel
	fprintf(fp_log, "Begin publishing sensor data...");
	while(1){ 
		BMP180.getTemperature(temp);
		memset(num_buff, 0, NUM_SIZE);
		strncpy(nav_data->pbuffer, "T: ", BUFF_SIZE);
		snprintf(num_buff, NUM_SIZE, "%f", *temp);
		strncat(nav_data->pbuffer, num_buff, BUFF_SIZE);
		nav_data->publish();
        
        BMP180.getPressure(pressure);
		memset(num_buff, 0, NUM_SIZE);
		strncpy(nav_data->pbuffer, "P: ", BUFF_SIZE);
		snprintf(num_buff, NUM_SIZE, "%f\n", *pressure);
		strncat(nav_data->pbuffer, num_buff, BUFF_SIZE);

		nav_data->publish();
	}
	// free alloced memory before terminating
    return 0;
}
