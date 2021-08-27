#ifndef _GPS_
#define _GPS_

#include "Adafruit_PMTK.h"

#define PMTK_SET_BAUD_38400 "$PMTK251,38400*27"
#define PMTK_SET_BAUD_115200 "$PMTK251,115200*1F"

#define END "\r\n"

void gps_setup();
String gps_read();

#endif