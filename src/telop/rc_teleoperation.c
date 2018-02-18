/*******************************************************************************
* rc_project_template.c
*
* This is meant to be a skeleton program for robotics cape projects. 
* Change this description and file name 
*******************************************************************************/

// usefulincludes is a collection of common system includes for the lazy
// This is not necessary for roboticscape projects but here for convenience
#include <rc_usefulincludes.h> 
// main roboticscape API header
#include <roboticscape.h>


// possible modes, user selected with command line arguments
typedef enum m_mode_t{
    DISABLED,
    NORMAL,
    BRAKE,
    FREE,
    SWEEP
} m_mode_t;


// function declarations
void on_pause_pressed();
void on_pause_released();


/*******************************************************************************
* int main() 
*
* This template main function contains these critical components
* - call to rc_initialize() at the beginning
* - main while loop that checks for EXITING condition
* - rc_cleanup() at the end
*******************************************************************************/
int main(){
	double duty = 0.0;
	int ch = 1;
	int c, in;
	int all = 1;	// set to 0 if a motor (-m) argument is given 
	m_mode_t m_mode = DISABLED;
    char direction = 'w';

	// always initialize cape library first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to initialize rc_initialize(), are you root?\n");
		return -1;
	}

	// do your own initialization here
	printf("\nStarting Teleoperation of BeagleBone Blue\n");
	rc_set_pause_pressed_func(&on_pause_pressed);
	rc_set_pause_released_func(&on_pause_released);

	// done initializing so set state to RUNNING
	rc_set_state(RUNNING); 
    rc_enable_motors();

	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){
		// handle other states
		if(rc_get_state()==RUNNING){
			// do things
			rc_set_led(GREEN, ON);
			rc_set_led(RED, OFF);
            m_mode = NORMAL;
            duty = 0.5;

            direction = getc(stdin);

            switch(direction){
                case 'w':
                    printf("Moving forward\n");
                    rc_set_motor_all(duty);
                    break;
                case 'a':
                    printf("Turning left\n");
                    rc_set_motor(1, -duty);
                    rc_set_motor(2, -duty);
                    rc_set_motor(3, duty);
                    rc_set_motor(4, duty);
                    break;
                case 'd':
                    printf("Turning right\n");
                    rc_set_motor(1, duty);
                    rc_set_motor(2, duty);
                    rc_set_motor(3, -duty);
                    rc_set_motor(4, -duty);
                    break;
                case 's':
                    printf("Reversing\n");
                    rc_set_motor_all(-duty);
                    break;
                case 'e':
                    printf("Braking all motors\n");
                    rc_set_motor_brake_all();
                    break;
                case '+':
                    printf("increasing speed\n");
                    duty += 0.1;
                    break;
                case '-':
                    printf("decreasing speed\n");
                    duty -= 0.1;
                    break;
                default: 
                    /* conitnue last action */
                    break;
            }
        }else if(rc_get_state()==PAUSED){
            // do other things
            rc_set_led(GREEN, OFF);
            rc_set_led(RED, ON);
        }
		// always sleep at some point
		usleep(100000);
	}
	
	// exit cleanly
	rc_cleanup(); 
	return 0;
}


/*******************************************************************************
* void on_pause_released() 
*	
* Make the Pause button toggle between paused and running states.
*******************************************************************************/
void on_pause_released(){
	// toggle betewen paused and running modes
	if(rc_get_state()==RUNNING)		rc_set_state(PAUSED);
	else if(rc_get_state()==PAUSED)	rc_set_state(RUNNING);
	return;
}

/*******************************************************************************
* void on_pause_pressed() 
*
* If the user holds the pause button for 2 seconds, set state to exiting which 
* triggers the rest of the program to exit cleanly.
*******************************************************************************/
void on_pause_pressed(){
	int i=0;
	const int samples = 100;	// check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds
	
	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		rc_usleep(us_wait/samples);
		if(rc_get_pause_button() == RELEASED) return;
	}
	printf("long press detected, shutting down\n");
	rc_set_state(EXITING);
	return;
}
