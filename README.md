Rover Control Project
=====================
Charles Sedgwick
[charlessedgwick.com]

RoverDaemons exits as a test bed for learning how to implement control
software for autonomous land based vehicles. The core computing platforms
supported are the the Beaglebone Black and Green. The Beaglebone blue should be
compatible but has not been tested. 

Dependancies
------------
All branches:
 - mraa: https://github.com/intel-iot-devkit/mraa
 - bb.org-overlays: https://github.com/RobertCNelson/bb.org-overlays
 - RoverACH
 - BMP180 Beaglebone Black C++ driver
 - LSM303 Beaglebone Black C++ driver
 - L3DG20 Beaglebone black C++ driver

@master:
 - ROS kinetic

@feature-ach:
 - ACH: https://github.com/golems/ach

Compilation
-----------

@master:

@feature-ach:
1. clone directory
    ```
    https://github.com/sedgwickc/RoverDaemons.git
    ```
2. navigate to newly created RoverDaemons directory
3. run make
    ```
    make all
    ```

Startup
-------

@master:


@feature-ach:
0. Ensure pwm pins are setup:
    ``` 
    sudo setupPWM.sh
    ```

1. create channels 
    ```
    sudo ./create_channels.sh -c
    ```
2. start daemons
    ```
    sudo ./build/sub_nav
    ```
3. start rover controller
    ```
    sudo ./build/control
    ```

Shutdown
--------

@master:

@feature-ach:
1. Kill daemons
    ```
    sudo kill $DAEMON_PID
    ```

Troubleshooting
---------------

@master:

@feature-ach:
1. delete channels
    ```
    sudo ./create_channels.sh -d
    ```
2. logging
    - logs are stored in the following directory: 
        ```
        /var/log/rover_logs/nav_log.txt
        ```

