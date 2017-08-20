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
#include <termios.h>
#include <unistd.h>

using namespace std;
using namespace rover;

/** this method disables buffered input in terminal so users do not have to hit
 * enter when entering cmds. the cmd is returned as soon as user enters it
 */
char getch() {
    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0)
        perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME]= 0;
    if(tcsetattr(0, TCSANOW, &old)<0)
        perror("tcsetattrICANON");
    if(read(0,&buf,1)<0)
        perror ("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if(tcsetattr(0, TCSADRAIN,&old) < 0)
        perror("tcsetattr~ICANON");
    return (buf);
}

int main( int argc, char** argv ) {

    /* create MotorControl object and pass null to have it set up default pins
     */
	MotorControl* mc = new MotorControl(NULL,NULL);
	//testing
	/* Investigate why test_drive dies and ssh connection lost when turn_right
	 * and turn_left called in sequence with only sleep(5) call between them 
	 */
    int cmd;
	while(1){

        if( cmd = getchar() ) 
        {
            switch( cmd ){
                case 'f':
	                mc->forward();
                    break;
                case 'b':
	                mc->backward();
                    break;
                case 'r':
	                mc->turn_right();
                    break;
                case 'l':
	                mc->turn_left();
                    break;
                case 's':
	                mc->stop();
                    break;
                case 'x':
	                mc->stop();
	                mc->clean_up();
                    return 0;
                    break;
                default:
                    break;

            }
	    }
	}

    return 0;
}
