Rover 
=====
Charles Sedgwick
charlessedgwick.com

Dependancies
------------
 - mraa
 - ACH
 - RoverACH
 - BMP180 Beaglebone Black C++ driver
 - LSM303 Beaglebone Black C++ driver
 - L3DG20 Beaglebone black C++ driver

Compilation
-----------
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
1. Kill daemons
    ```
    sudo kill $DAEMON_PID
    ```

Troubleshooting
---------------
1. delete channels
    ```
    sudo ./create_channels.sh -d
    ```
2. logging
    - logs are stored in the following directory: 
        ```
        /var/log/rover_logs/nav_log.txt
        ```

