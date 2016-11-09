TOP=..
GPP=g++
CFLAGS= 
DEBUG=-Wall -g -DDEBUG
LINKER= -lmraa 
BUILD=build/


all: sub_nav control test_drive

test_drive: MotorControl.o
	$(GPP) $(CFLAGS) $(DEBUG) $(BUILD)$^ drive/test_drive.cpp -o $(BUILD)$@ $(LINKER) 

RoverACH.o: $(TOP)/RoverACH/RoverACH.hpp $(TOP)/RoverACH/RoverACH.cpp
	$(GPP) $(CFLAGS) -c $(TOP)/RoverACH/RoverACH.cpp -o $(BUILD)$@ -lach

bmp180.o: navigation/BBB_BMP180/Adafruit_BMP180.hpp navigation/BBB_BMP180/Adafruit_BMP180.cpp
	$(GPP) $(CFLAGS) -c navigation/BBB_BMP180/Adafruit_BMP180.cpp -o $(BUILD)$@

L3GD20.o: navigation/BBB_L3GD20/Adafruit_L3GD20.hpp navigation/BBB_L3GD20/Adafruit_L3GD20.cpp
	$(GPP) $(CFLAGS) -c navigation/BBB_L3GD20/Adafruit_L3GD20.cpp -o $(BUILD)$@

LSM303.o: navigation/BBB_LSM303/Adafruit_LSM303.h navigation/BBB_LSM303/Adafruit_LSM303.cpp
	$(GPP) $(CFLAGS) -c navigation/BBB_LSM303/Adafruit_LSM303.cpp -o $(BUILD)$@

control: RoverACH.o
	$(GPP) $(CFLAGS) $(DEBUG) RoverControl/RoverControl.cpp $(BUILD)$^ -o \
		$(BUILD)$@ $(LINKER) -lach

sub_nav: RoverACH.o bmp180.o L3GD20.o LSM303.o
	$(GPP) $(CFLAGS) $(DEBUG) navigation/sub_nav.cpp $(BUILD)RoverACH.o $(BUILD)bmp180.o \
	$(BUILD)L3GD20.o $(BUILD)LSM303.o -o $(BUILD)$@ $(LINKER) -lach

MotorControl.o: drive/MotorControl.hpp drive/MotorControl.cpp
	$(GPP) $(CFLAGS) -c drive/MotorControl.cpp -o $(BUILD)$@

clean:
	rm *.o sub_nav control test_drive *.log
