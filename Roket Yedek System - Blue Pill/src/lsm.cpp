#include "lsm.h"
#include <cmath>


Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
Adafruit_LSM9DS0 lsm_0 = Adafruit_LSM9DS0();


void setupLsmSensor()
{
  // 1.) Set the accelerometer range
  // lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}



void setupLsm_0_Sensor()
{
  // 1.) Set the accelerometer range
  // lsm_0.setupAccel(lsm_0.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  lsm_0.setupAccel(lsm_0.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm_0.setupMag(lsm_0.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm_0.setupGyro(lsm_0.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

#define DECLINATION 5.75
// #define M_PI 3.14159265358979323846


float getPitch(float ax, float ay, float az)
{
  float pitch;

  pitch = atan2(ax, sqrt(ay * ay) + (az * az));
  pitch *= 180.0 / M_PI;

  return pitch;
}

float getRoll(float ax, float ay, float az)
{
  float roll;
  roll = atan2(ay, sqrt(ax * ax) + (az * az));
  roll *= 180.0 / M_PI;
  return roll;
}


float getHeading(float mx, float my){
    mx = -mx;
    my = -my;

    float heading;
      if (my == 0)
        heading = (mx < 0) ? M_PI : 0;
      else
        heading = atan2(mx, my);

      heading -= DECLINATION * M_PI / 180;

      if (heading > M_PI) heading -= (2 * M_PI);
      else if (heading < -M_PI) heading += (2 * M_PI);

      // Convert everything from radians to degrees:
      heading *= 180.0 / M_PI;

    return heading;
}