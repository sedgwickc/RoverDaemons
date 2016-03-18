GPP=g++
CFLAGS= 
DEBUG=-Wall -g -DDEBUG
LINKER= -lmraa 
BUILD=build/


all: sub_nav control

RoverACH.o: RoverACH/RoverACH.hpp RoverACH/RoverACH.cpp
	$(GPP) $(CFLAGS) -c RoverACH/RoverACH.cpp -o $(BUILD)$@ -lach

bmp180.o: navigation/BBB_BMP180/Adafruit_BMP180.hpp navigation/BBB_BMP180/Adafruit_BMP180.cpp
	$(GPP) $(CFLAGS) -c navigation/BBB_BMP180/Adafruit_BMP180.cpp -o $(BUILD)$@

control: RoverACH.o
	$(GPP) $(CFLAGS) $(DEBUG) RoverControl/RoverControl.cpp $(BUILD)$^ -o \
		$(BUILD)$@ $(LINKER) -lach

sub_nav: RoverACH.o bmp180.o
	$(GPP) $(CFLAGS) $(DEBUG) navigation/sub_nav.cpp $(BUILD)RoverACH.o $(BUILD)bmp180.o \
	-o $(BUILD)$@ $(LINKER) -lach

clean:
	rm *.o sub_nav control *.log
