#include "bme.h"


// #define BMP_SCK 13
// #define BMP_MISO 12
// #define BMP_MOSI 11
#define BMP_CS PB15



extern Adafruit_BMP3XX bmp;


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
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}



void BMPloop() {


  if (! bmp.performReading()) {
    Serial.print("BME: Failed to perform reading :(\n");
    return;
  }

  // Serial.print("Temperature = ");
  // Serial.println(bmp.temperature);


  // Serial.print("3\n");

  // Serial.print("Pressure = ");
  // Serial.print(bmp.pressure / 100.0);
  // Serial.println(" hPa\n");

  // Serial.print("Approx. Altitude = ");
  // Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  // Serial.println(" m\n");
}