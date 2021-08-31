#include "bme.h"


// #define BMP_SCK 13
// #define BMP_MISO 12
// #define BMP_MOSI 11
#define BMP_CS PB15



void BMPsetup() {
  // Serial.begin(115200);
  // while (!Serial);
  // Serial.println("Adafruit BMP388 / BMP390 test");

  // if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
  if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
  //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

void BMEsetup() {

  if (! bme.begin()) {  // hardware SPI mode  
  //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2,  // temperature
                    Adafruit_BME280::SAMPLING_X8, // pressure
                    Adafruit_BME280::SAMPLING_X1,  // humidity
                    Adafruit_BME280::FILTER_X8,
                    Adafruit_BME280::STANDBY_MS_0_5 );
}



void BMPloop() {
  if (! bmp.performReading()) {
    Serial.print("BME: Failed to perform reading :(\n");
    return;
  }
}

void BMEloop() {
  bme.readTemperature();
}