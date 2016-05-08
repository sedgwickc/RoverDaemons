#! /bin/bash

# Charles Sedgwick
# This script creates the channels required for the Goeff rover project daemons

BUFF_SIZE=4096

while [ "$1" != "" ]; do
	case $1 in 
		-c | --create )    
			# navigation related sensor data
			ach mk nav_data -m 20 -n $BUFF_SIZE

			# navigation commands
			ach mk nav_cmd -m 20 -n $BUFF_SIZE

			# drive encoder data
			ach mk drive_data -m 20 -n $BUFF_SIZE

			#drive commands
			ach mk drive_cmd -m 20 -n $BUFF_SIZE

			# power sensor data
			ach mk power_data -m 20 -n $BUFF_SIZE
			;;
		-d | --delete )
			# navigation related sensor data
			ach rm nav_data -m 20 -n $BUFF_SIZE

			# navigation commands
			ach rm nav_cmd -m 20 -n $BUFF_SIZE

			# drive encoder data
			ach rm drive_data -m 20 -n $BUFF_SIZE

			#drive commands
			ach rm drive_cmd -m 20 -n $BUFF_SIZE

			# power sensor data
			ach rm power_data -m 20 -n $BUFF_SIZE
			;;
		* )
			echo "Usage: create_channels -d | --delete | -c | --create";
			exit 1;
	esac
	shift
done
