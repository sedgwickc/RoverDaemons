#! /bin/bash

export MAVLINK=$HOME"rover/mavlink/c_library_v2/minimal/" 
export ROVER_HOME=$HOME"rover/"
export CPATH=$CPATH:$MAVLINK
export PATH=$PATH:$MAVLINK
