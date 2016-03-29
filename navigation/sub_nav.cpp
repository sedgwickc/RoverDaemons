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
#include "BBB_L3GD20/Adafruit_L3GD20.hpp"
#include "BBB_LSM303/Adafruit_LSM303.h"

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
  
	fprintf(fp_log, "Setting up sensor L3GD20...");
	Adafruit_L3GD20 L3GD20(1);
	if( L3GD20.begin() == false)
	{
		fprintf(fp_log, "Could not find a valid L3GD20!\n");
		return 0;
	}
	float gyro_x = 0.0;
	float gyro_y = 0.0;
	float gyro_z = 0.0;
    int  millisec = 500;
    struct timespec rec = {0};
    rec.tv_sec = 0;
    rec.tv_nsec = millisec * 1000000L;

	fprintf(fp_log, "Setting up sensor LSM303...");
	Adafruit_LSM303 LSM303(1);
	if( LSM303.begin() == false)
	{
		fprintf(fp_log, "Error setting up LSM303!\n");
		return 0;
	}
	float mag_x = 0.0;
	float mag_y = 0.0;
	float mag_z = 0.0;
	float acc_x = 0.0;
	float acc_y = 0.0;
	float acc_z = 0.0;

	LSM303.getOrientation( &mag_x, &mag_y, &mag_z );

	// TODO: add while condition to check that RoverControl process hasn't sent a stop signal 
	// Continually publish sensor readings to channel
	fprintf(fp_log, "Begin publishing sensor data...");
	fflush(fp_log);
	while(1){ 
		BMP180.getTemperature(temp);
		memset(num_buff, 0, NUM_SIZE);
		strncpy(nav_data->pbuffer, ">T: ", BUFF_SIZE);
		snprintf(num_buff, NUM_SIZE, "%f", *temp);
		strncat(nav_data->pbuffer, num_buff, BUFF_SIZE);
		nav_data->publish();
        
        BMP180.getPressure(pressure);
		memset(num_buff, 0, NUM_SIZE);
		strncpy(nav_data->pbuffer, ">P: ", BUFF_SIZE);
		snprintf(num_buff, NUM_SIZE, "%f", *pressure);
		strncat(nav_data->pbuffer, num_buff, BUFF_SIZE);
		nav_data->publish();

		L3GD20.readGyro(&gyro_x, &gyro_y, &gyro_z);
		memset(num_buff, 0, NUM_SIZE);
		strncpy(nav_data->pbuffer, ">gyroX: ", BUFF_SIZE);
		snprintf(num_buff, NUM_SIZE, "%f", gyro_x);
		strncat(nav_data->pbuffer, num_buff, BUFF_SIZE);
		nav_data->publish();

		memset(num_buff, 0, NUM_SIZE);
		strncpy(nav_data->pbuffer, ">gyroY: ", BUFF_SIZE);
		snprintf(num_buff, NUM_SIZE, "%f", gyro_y);
		strncat(nav_data->pbuffer, num_buff, BUFF_SIZE);
		nav_data->publish();
		
		memset(num_buff, 0, NUM_SIZE);
		strncpy(nav_data->pbuffer, ">gyroZ: ", BUFF_SIZE);
		snprintf(num_buff, NUM_SIZE, "%f", gyro_z);
		strncat(nav_data->pbuffer, num_buff, BUFF_SIZE);
		nav_data->publish();

		LSM303.getAcceleration(&acc_x, &acc_y, &acc_z);
		memset(num_buff, 0, NUM_SIZE);
		strncpy(nav_data->pbuffer, ">accX: ", BUFF_SIZE);
		snprintf(num_buff, NUM_SIZE, "%f", acc_x);
		strncat(nav_data->pbuffer, num_buff, BUFF_SIZE);
		nav_data->publish();

		memset(num_buff, 0, NUM_SIZE);
		strncpy(nav_data->pbuffer, ">accY: ", BUFF_SIZE);
		snprintf(num_buff, NUM_SIZE, "%f", acc_y);
		strncat(nav_data->pbuffer, num_buff, BUFF_SIZE);
		nav_data->publish();

		memset(num_buff, 0, NUM_SIZE);
		strncpy(nav_data->pbuffer, ">accZ: ", BUFF_SIZE);
		snprintf(num_buff, NUM_SIZE, "%f", acc_z);
		strncat(nav_data->pbuffer, num_buff, BUFF_SIZE);
		nav_data->publish();

		LSM303.getOrientation( &mag_x, &mag_y, &mag_z );
		memset(num_buff, 0, NUM_SIZE);
		strncpy(nav_data->pbuffer, ">magX: ", BUFF_SIZE);
		snprintf(num_buff, NUM_SIZE, "%f", mag_x);
		strncat(nav_data->pbuffer, num_buff, BUFF_SIZE);
		nav_data->publish();

		memset(num_buff, 0, NUM_SIZE);
		strncpy(nav_data->pbuffer, ">magY: ", BUFF_SIZE);
		snprintf(num_buff, NUM_SIZE, "%f", mag_y);
		strncat(nav_data->pbuffer, num_buff, BUFF_SIZE);
		nav_data->publish();

		memset(num_buff, 0, NUM_SIZE);
		strncpy(nav_data->pbuffer, ">magZ: ", BUFF_SIZE);
		snprintf(num_buff, NUM_SIZE, "%f", mag_z);
		strncat(nav_data->pbuffer, num_buff, BUFF_SIZE);
		nav_data->publish();

		strncat(nav_data->pbuffer, "\n", BUFF_SIZE);
    	nanosleep(&rec, (struct timespec *) NULL);
		nav_data->publish();
	}
	// free alloced memory before terminating
	L3GD20.cleanup();
//	LSM303.cleanup();
    return 0;
}
