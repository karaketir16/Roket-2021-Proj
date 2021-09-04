#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_GPS.h>

#define PMTK_SET_BAUD_38400 "$PMTK251,38400*27"
#define PMTK_SET_BAUD_115200 "$PMTK251,115200*1F"

#define BMP_SCK 24
#define BMP_MISO 22
#define BMP_MOSI 23
#define BMP_CS 6

#define SEALEVELPRESSURE_HPA (1013.25)
#define GPSSerial Serial1
#define GPSECHO false
// #define GPSECHO true

#include "shared.h"

#include <RadioLib.h>

uint32_t timer = millis();
int buzz = 12;

packet_payload p1;
packet_payload p_send;

Adafruit_BMP3XX bmp;
Adafruit_GPS GPS(&GPSSerial);
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

//#define LSM9DS1_SCK A5
//#define LSM9DS1_MISO 12
//#define LSM9DS1_MOSI A4
//#define LSM9DS1_XGCS 6
//#define LSM9DS1_MCS 5

float degree = 57.295;
// You can also use software SPI
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_SCK, LSM9DS1_MISO, LSM9DS1_MOSI, LSM9DS1_XGCS, LSM9DS1_MCS);
// Or hardware SPI! In this case, only CS pins are passed in
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);

void setupSensor()
{
  // 1.) Set the accelerometer range
  // lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  //
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}

// flag to indicate that a packet was sent
volatile bool transmittedFlag = false;

// disable interrupt when it's not needed
volatile bool enableInterrupt = true;

// this function is called when a complete packet
// is transmitted by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
void setFlag(void)
{
  // check if the interrupt is enabled
  if (!enableInterrupt)
  {
    return;
  }

  // we sent a packet, set the flag
  transmittedFlag = true;
}

// save transmission state between loops
int transmissionState = ERR_NONE;

RFM95 radio = new Module(FEATHER_RFM95_CS, FEATHER_RFM95_INT, FEATHER_RFM95_RST, FEATHER_RFM95_DUMMY);

STATE state = INIT;

float initial_altitude;

bool led = false;

void setup()
{
  delay(200);
  pinMode(13, OUTPUT);
  digitalWrite(13, led);

  Serial.begin(115200);

  // initialize SX1278 with default settings
  Serial.print(F("[SX1278] Initializing ... "));

  int radio_state = radio.begin(PAYLOAD_RF95_FREQ, RF95_BW, RF95_SF, RF95_CR, RF95_SYNC_WORD, RF95_POWER, RF95_PL, RF95_GAIN);
  if (radio_state == ERR_NONE)
  {
    Serial.println(F("success!"));
  }
  else
  {
    Serial.print(F("failed, code "));
    Serial.println(radio_state);
    while (true)
      ;
  }

  Serial.println("Adafruit BMP388 / BMP390 test");
  Serial.println("LSM9DS1 data read demo");
  Serial.println("Adafruit GPS library basic parsing test!");

  GPS.begin(115200);
  GPS.sendCommand(PMTK_SET_BAUD_38400);
  delay(200);

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_BAUD_38400);
  delay(200);

  GPS.begin(38400);
  GPS.sendCommand(PMTK_SET_BAUD_38400);
  delay(200);

  // GPSSerial.end();
  // GPSSerial.begin(115200);
  GPSSerial.begin(38400);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  GPSSerial.println(PMTK_Q_RELEASE);

  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1)
      ;
  }
  Serial.println("Found LSM9DS1 9DOF");

  // helper to just set the default scaling we want, see above!
  setupSensor();

  // if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
  if (!bmp.begin_SPI(BMP_CS))
  { // hardware SPI mode
    // if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1)
      ;
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  for (int i = 0; i < 10; i++)
  {
    bmp.performReading();
  }

  float tmp = 0;
  for (int i = 0; i < 10; i++)
  {
    tmp += bmp.readAltitude(SEALEVELPRESSURE_HPA);
  }
  initial_altitude = tmp / 10;

  // set the function that will be called
  // when packet transmission is finished
  radio.setDio0Action(setFlag);

  radio.startTransmit((byte *)&p1, sizeof p1);

  // last_speed_time = millis();
  timer = millis();
}

float latLong(float a)
{
  int degree = a / 100;
  a -= degree * 100;
  float res = a / 60;
  return degree + res;
}

void loop()
{

  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c)
      Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived())
  {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    // Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return;                       // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats
  if (true && millis() - timer > 200)
  {

    // Serial.print("\nTime: ");
    // if (GPS.hour < 10)
    // {
    //   Serial.print('0');
    // }
    // Serial.print(GPS.hour, DEC);
    // Serial.print(':');
    // if (GPS.minute < 10)
    // {
    //   Serial.print('0');
    // }
    // Serial.print(GPS.minute, DEC);
    // Serial.print(':');
    // if (GPS.seconds < 10)
    // {
    //   Serial.print('0');
    // }
    // Serial.print(GPS.seconds, DEC);
    // Serial.print('.');
    // if (GPS.milliseconds < 10)
    // {
    //   Serial.print("00");
    // }
    // else if (GPS.milliseconds > 9 && GPS.milliseconds < 100)
    // {
    //   Serial.print("0");
    // }
    // Serial.println(GPS.milliseconds);
    // Serial.print("Date: ");
    // Serial.print(GPS.day, DEC);
    // Serial.print('/');
    // Serial.print(GPS.month, DEC);
    // Serial.print("/20");
    // Serial.println(GPS.year, DEC);
    // Serial.print("Fix: ");
    // Serial.print((int)GPS.fix);
    // Serial.print(" quality: ");
    // Serial.println((int)GPS.fixquality);
    if (GPS.fix)
    {
      // Serial.print("Location: ");
      // Serial.print(GPS.latitude, 4);
      // Serial.print(GPS.lat);
      // Serial.print(", ");
      // Serial.print(GPS.longitude, 4);
      // Serial.println(GPS.lon);
      // Serial.print("Speed (knots): ");
      // Serial.println(GPS.speed);
      // Serial.print("Angle: ");
      // Serial.println(GPS.angle);
      // Serial.print("Altitude: ");
      // Serial.println(GPS.altitude);
      // Serial.print("Satellites: ");
      // Serial.println((int)GPS.satellites);
    }

    // Serial.print("Accel X: ");
    // Serial.print(a.acceleration.x);
    // Serial.print(" m/s^2");

    // Serial.print("\tY: ");
    // Serial.print(a.acceleration.y);
    // Serial.print(" m/s^2 ");
    // Serial.print("\tZ: ");
    // Serial.print(a.acceleration.z);
    // Serial.println(" m/s^2 ");

    // Serial.print("Mag X: ");
    // Serial.print(m.magnetic.x);
    // Serial.print(" uT");
    // Serial.print("\tY: ");
    // Serial.print(m.magnetic.y);
    // Serial.print(" uT");
    // Serial.print("\tZ: ");
    // Serial.print(m.magnetic.z);
    // Serial.println(" uT");

    // Serial.print("Gyro X: ");
    // Serial.print(g.gyro.x * degree);
    // Serial.print(" deg");
    // Serial.print("\tY: ");
    // Serial.print(g.gyro.y * degree);
    // Serial.print(" deg");
    // Serial.print("\tZ: ");
    // Serial.print(g.gyro.z * degree);
    // Serial.println(" deg");

    if (!bmp.performReading())
    {
      Serial.println("Failed to perform reading :(");
      // return;
    }
    // Serial.print("Temperature = ");
    // Serial.print(bmp.temperature);
    // Serial.println(" *C");

    // Serial.print("Pressure = ");
    // Serial.print(bmp.pressure / 100.0);
    // Serial.println(" hPa");

    // Serial.print("Approx. Altitude = ");
    // Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    // Serial.println(" m");

    // Serial.println();

    // check if the previous transmission finished
    if (transmittedFlag)
    {
      // disable the interrupt service routine while
      // processing the data
      enableInterrupt = false;

      // reset flag
      transmittedFlag = false;

      timer = millis(); // reset the timer
      led ^= 1;
      digitalWrite(13, led);

      if (GPS.fix)
      {
        p1.gps_latitude = latLong(GPS.latitude);
        p1.gps_longtitude = latLong(GPS.longitude);
      }

      lsm.read(); /* ask it to read in the data */

      /* Get a new sensor event */
      sensors_event_t a, m, g, temp;

      lsm.getEvent(&a, &m, &g, &temp);

      p1.acc_x = a.acceleration.x;
      p1.acc_y = a.acceleration.y;
      p1.acc_z = a.acceleration.z;

      if (!bmp.performReading())
      {
        Serial.println("Failed to perform reading :(");
        // return;
      }
      else
      {
        p1.pressure = bmp.pressure / 100;
        p1.temp = bmp.temperature;
        p1.altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA) - initial_altitude;
      }

      if (transmissionState == ERR_NONE)
      {
        // packet was successfully sent

        Serial.println(F("transmission finished!**********************************"));

        // Serial.print("Time: ");
        // Serial.println(millis() - timer);
        // timer = millis();

        // NOTE: when using interrupt-driven transmit method,
        //       it is not possible to automatically measure
        //       transmission data rate using getDataRate()
      }
      else
      {
        Serial.print(F("failed, code "));
        Serial.println(transmissionState);
      }

      // for (int i = 0 ; i < sizeof p1; i++){
      //   char * tmp = (char *) &p1;
      //   tmp[i] = i;
      // }

      fill_checksum((char *)&p1, sizeof p1);

      p_send = p1;

      if (check_checksum((char *)&p_send, sizeof p_send))
      {
        Serial.print("Size: ");
        Serial.println(sizeof p_send);
        Serial.println("fill checksum OK");
      }
      else
      {
        Serial.println("fill checksum FAIL");
      }
      // Serial.printf("press: %f\n accx: %f\n", p_send.pressure, p_send.acc_x);
      int state = radio.startTransmit((byte *)&p_send, sizeof p_send);
      // int state = radio.startTransmit((byte*) &p_send, 10);
      // p1.package_number++;

      // we're ready to send more packets,
      // enable interrupt service routine
      enableInterrupt = true;
    }
  }
}
