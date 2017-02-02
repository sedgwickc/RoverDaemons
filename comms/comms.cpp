/*
 * Author: Michael Ring <mail@michael-ring.org>
 * Author: Thomas Ingleby <thomas.c.ingleby@intel.com>
 * Copyright (c) 2014 Intel Corporation.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

 /* 
  * comms.cpp
  * Charles Sedgwick
  * transciever wired to SPIDEV0.0
  * BB-SPIDEV0 overlay must be loaded if /dev/SPIDEV0.0 not present
  */

#include <sstream>
#include <stdio.h>
#include <iostream>
#include "RFM69/rfm69.h"

using namespace exploringBB;
using namespace std;
using namespace rover;

int main(int argc, char** argv) {
    //! [Interesting]
    cout << "Starting SPI COMMS Example" << endl;

    /* initialize comms */
    RFM69HCW *comms = new RFM69HCW(0,0);
 
    cout << "Exiting SPI COMMS Example" << endl;
    delete comms;

    //! [Interesting]
}
