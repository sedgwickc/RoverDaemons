TOP=../..
GPP=g++
CFLAGS= 
DEBUG=-Wall -g -DDEBUG
LINKER= -lmraa 
BUILD=build/


all: sub_nav control

MotorControl.o: MotorControl.hpp MotorControl.cpp
	$(GPP) $(CFLAGS) -c MotorControl.cpp -o $(BUILD)$@

test_drive: MotorControl.o
	$(GPP) $(CFLAGS) $(DEBUG) $(BUILD)$^ test_drive.cpp -o $(BUILD)$@ $(LINKER) 

clean:
	rm *.o test_drive *.log
