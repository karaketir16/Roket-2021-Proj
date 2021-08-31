#ifndef _BME_
#define _BME_

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#include "Adafruit_BME280.h"

#define SEALEVELPRESSURE_HPA (1013.25)

extern Adafruit_BMP3XX bmp;
extern Adafruit_BME280 bme;

void BMPsetup();
void BMPloop();

void BMEsetup();
void BMEloop();

#endif