#ifndef _LSM_
#define _LSM_

#include <Adafruit_LSM9DS1.h>
#include <Adafruit_LSM9DS0.h>

extern Adafruit_LSM9DS1 lsm;
extern Adafruit_LSM9DS0 lsm_0;
void setupLsmSensor();
void setupLsm_0_Sensor();

#endif