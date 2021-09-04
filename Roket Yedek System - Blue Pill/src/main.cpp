#include <Arduino.h>
#include "gps.h"
#include <Wire.h>
#include <SPI.h>

#include <Adafruit_Sensor.h> // not used in this demo but required!

#include "bme.h"
#include "lsm.h"
#include "shared.h"

#include "minmea.h"

#include "rf.h"

// #define SIMULATION
// #define PRINT

HardwareSerial GPSSerial(USART2);

#define BMP_CS PB15

Adafruit_BMP3XX bmp;
Adafruit_BME280 bme(BMP_CS);

bool MAIN_LIVE = true;
// or using RadioShield
// https://github.com/jgromes/RadioShield
//SX1278 radio = RadioShield.ModuleA;

packet p1_receive;

// flag to indicate that a packet was received
volatile bool receivedFlag_1 = false;

// disable interrupt when it's not needed
volatile bool enableInterrupt_1 = true;

// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
void setFlag_1(void)
{
  // check if the interrupt is enabled
  if (!enableInterrupt_1)
  {
    return;
  }

  // we got a packet, set the flag
  receivedFlag_1 = true;
}

packet p1;
packet p_send;

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

void buzzer(int dl)
{
  digitalWrite(PA12, HIGH);
  delay(dl);
  digitalWrite(PA12, LOW);
  delay(dl);
}

static float sim_alt = 0;
static float sim_mul = 75;

float simulation_altitude()
{
  sim_alt += sim_mul;
  if (sim_alt > 3000)
  {
    sim_mul = -10;
  }
  if (sim_alt < 0)
    sim_alt = 0;
  return sim_alt;
}

// save transmission state between loops
int transmissionState = ERR_NONE;

RFM95 radio = new Module(RFM95_CS, RFM95_INT, RFM95_RST, RFM95_DUMMY);

float initial_altitude = 0;
uint32_t timer = millis();
uint32_t last_speed_time = millis();

STATE state = INIT;

int last_received_time = millis();

int pins[3] = {MAIN_YAY, MAIN_PAYLOAD, MAIN_ANA};
int stages_times[3] = {0, 0, 0};

void setup()
{

  for (int i = 0; i < 3; i++)
  {
    pinMode(pins[i], OUTPUT);
  }

  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_GREEN, HIGH);

  Serial.begin(38400);
  Serial.println("Booting");

  delay(500);

  pinMode(PA12, OUTPUT);

  buzzer(500);

  srand(analogRead(1));

  // p1.package_number = 0;
  // p1.acc_x = 0;
  // p1.acc_y = 0;
  // p1.acc_z = 0;

  p1.altitude = 0;
  p1.gps_fix = 0;
  p1.gps_latitude = 0;
  p1.gps_longtitude = 0;

  p1.pressure = 0;
  p1.speed = 0;
  // p1.temperature = 0;

  // initialize SX1278 with default settings
  Serial.print(F("[SX1278] Initializing ... "));

  int radio_state = radio.begin(RF95_FREQ, RF95_BW, RF95_SF, RF95_CR, RF95_SYNC_WORD, RF95_POWER, RF95_PL, RF95_GAIN);
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

  if (!lsm.begin())
  {
    while (1)
    {
      delay(250);
      Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    }
  }
  else
  {
    Serial.println("Found LSM9DS1 9DOF");
  }

  setupLsmSensor();

  gps_setup();
  BMEsetup();
  for (int i = 0; i < 10; i++)
  {
    bme.readPressure();
  }

  float tmp = 0;
  for (int i = 0; i < 10; i++)
  {
    tmp += bme.readAltitude(SEALEVELPRESSURE_HPA);
  }
  initial_altitude = tmp / 10;

  state = STATE::WAIT;

  for (int i = 0; i < 2; i++)
  {
    buzzer(500);
  }

  // set the function that will be called
  // when packet transmission is finished

  /*
  radio.setDio0Action(setFlag);

  radio.startTransmit((byte*) &p1, sizeof p1);
  */

  radio.setDio0Action(setFlag_1);

  // start listening for LoRa packets
  Serial.print(F("[SX1278] Starting to listen ... "));
  radio_state = radio.startReceive();
  if (radio_state == ERR_NONE)
  {
    Serial.println(F("success 1!"));
  }
  else
  {
    Serial.print(F("failed 1, code "));
    Serial.println(radio_state);
    while (true)
      ;
  }

  last_speed_time = millis();
  timer = millis();
  last_received_time = millis();
}

void check_stages()
{
  for (int i = 0; i < 3; i++)
  {
    if (millis() - stages_times[i] > 3000)
    {
      digitalWrite(pins[i], LOW);
    }
  }
}
void fire_STAGE(int i)
{
  digitalWrite(pins[i], HIGH);
  stages_times[i] = millis();
}

bool led = false;

int counter = 0;

void loop()
{

  lsm.read(); /* ask it to read in the data */

  /* Get a new sensor event */
  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp);

  p1.abs_vert_acc = abs(a.acceleration.x);
  // p1.acc_y = a.acceleration.y;
  // p1.acc_z = a.acceleration.z;

  float old_alt = p1.altitude;
  p1.altitude = bme.readAltitude(SEALEVELPRESSURE_HPA) - initial_altitude;

#ifdef SIMULATION
  p1.altitude = simulation_altitude();
#endif

  p1.speed = (p1.altitude - old_alt) / ((millis() - last_speed_time) / 1000.0);
  last_speed_time = millis();

  p1.pressure = bme.readPressure() / 100.0;
  // p1.temperature = bmp.temperature;

  switch (state)
  {
  case STATE::INIT:
    break;
  case STATE::WAIT:
    if (p1.altitude > 200)
    {
      state = STATE::RISING_STAGE;
    }
    break;
  case STATE::RISING_STAGE:
    // if( p1.speed <= 20){ // instead of speed, use accelarion data

    if (p1.abs_vert_acc <= 1)
    {
      counter++;
    }
    else
    {
      counter = 0;
    }

    if (counter == 5)
    {
      state = STATE::FALLING_1;
      if (!MAIN_LIVE)
        fire_STAGE(0);
      buzzer(75);
    }

    break;
  case STATE::FALLING_1:
    if (p1.altitude < 1600)
    {
      state = STATE::FALLING_2;
      if (!MAIN_LIVE)
        fire_STAGE(1);
      buzzer(75);
      buzzer(75);
    }
    check_stages();
    break;
  case STATE::FALLING_2:
    if (p1.altitude < 700)
    {
      state = STATE::FALLING_3;
      if (!MAIN_LIVE)
        fire_STAGE(2);
      buzzer(75);
      buzzer(75);
      buzzer(75);
    }
    check_stages();
    break;
  case STATE::FALLING_3:
    if (p1.altitude < 20)
    {
      state = STATE::END_STAGE;
    }
    check_stages();
    break;
  case STATE::END_STAGE:
    digitalWrite(PA12, HIGH);
    check_stages();
    break;
  default:
    break;
  }

  String gps_data = gps_read();

  const char *line = gps_data.c_str();

  int r = minmea_sentence_id(line, false);

  switch (r)
  {
  case MINMEA_SENTENCE_RMC:
  {
    struct minmea_sentence_rmc frame;
    if (minmea_parse_rmc(&frame, line))
    {

      auto fix_check = minmea_tocoord(&frame.latitude);
      p1.gps_fix = (fix_check == fix_check);
      if (p1.gps_fix)
      {
        p1.gps_latitude = minmea_tocoord(&frame.latitude);
        p1.gps_longtitude = minmea_tocoord(&frame.longitude);
      }
    }
    else
    {
      p1.gps_fix = 0;
    }
  }
  break;
  default:;
  }

#ifdef PRINT
  Serial.print("GPS: ");
  Serial.println(gps_data);

  Serial.print("GPS_FIX: ");
  Serial.println((int)p1.gps_fix);

  Serial.print("GPS lat: ");
  Serial.println(p1.gps_latitude, 5);

  Serial.print("GPS lon: ");
  Serial.println(p1.gps_longtitude, 5);

  Serial.print("Pressure: ");
  Serial.println(p1.pressure, 5);

  Serial.print("Altitude: ");
  Serial.println(p1.altitude);

  Serial.print("Speed: ");
  Serial.println(p1.speed);

  Serial.print("Temp: ");
  Serial.println(bme.readTemperature());

  Serial.print("Acceleration x: ");
  Serial.println(a.acceleration.x);

  Serial.print("Acceleration y: ");
  Serial.println(a.acceleration.y);

  Serial.print("Acceleration z: ");
  Serial.println(a.acceleration.z);

  Serial.print("Pitch: ");
  Serial.println(getPitch(a.acceleration.z, a.acceleration.y, a.acceleration.x));

  Serial.print("Roll: ");
  Serial.println(getRoll(a.acceleration.z, a.acceleration.y, a.acceleration.x));

  Serial.print("Heading: ");
  Serial.println(getHeading(m.magnetic.z, m.magnetic.y));

  Serial.print("Mag x: ");
  Serial.println(m.magnetic.x);

  Serial.print("Mag y: ");
  Serial.println(m.magnetic.y);

  Serial.print("Mag z: ");
  Serial.println(m.magnetic.z);
  Serial.println();
#endif

  // check if the previous transmission finished
  if (transmittedFlag && millis() - timer > 500)
  {

    // disable the interrupt service routine while
    // processing the data
    enableInterrupt = false;

    // reset flag
    transmittedFlag = false;

    if (transmissionState == ERR_NONE)
    {
      // packet was successfully sent
      led ^= 1;
      digitalWrite(LED_GREEN, led);

      Serial.println(F("transmission finished!**********************************"));

      Serial.print("Time: ");
      Serial.println(millis() - timer);
      timer = millis();

      // NOTE: when using interrupt-driven transmit method,
      //       it is not possible to automatically measure
      //       transmission data rate using getDataRate()
    }
    else
    {
      Serial.print(F("failed, code "));
      Serial.println(transmissionState);
    }
    p1.state = state + 10;
    fill_checksum((char *)&p1, sizeof p1);
    p_send = p1;
    // Serial.printf("press: %f\n accx: %f\n", p_send.pressure, p_send.acc_x);
    radio.startTransmit((byte *)&p_send, sizeof p_send);
    // int state = radio.startTransmit((byte*) &p_send, 10);
    // p1.package_number++;

    // we're ready to send more packets,
    // enable interrupt service routine
    enableInterrupt = true;
  }

  if (receivedFlag_1)
  {
    // disable the interrupt service routine while
    // processing the data
    enableInterrupt_1 = false;

    // reset flag
    receivedFlag_1 = false;

    int radio_state = radio.readData((byte *)&p1_receive, sizeof p1_receive);

    if (radio_state == ERR_NONE)
    {

      if (check_checksum((char *)&p1_receive, sizeof p1_receive))
      {
        last_received_time = millis();
        if (p1_receive.state == STATE::END_STAGE)
        {
          state = STATE::END_STAGE;
          MAIN_LIVE = false;
        }
        Serial.println("RECEIVED");
      }
      else
      {
        Serial.println(F("[SX1278] Cheksum error!"));
      }
    }
    else if (radio_state == ERR_CRC_MISMATCH)
    {
      // packet was received, but is malformed
      Serial.println(F("[SX1278] CRC error!"));
    }
    else
    {
      // some other error occurred
      Serial.print(F("[SX1278] Failed, code "));
      Serial.println(radio_state);
    }

    // put module back to listen mode
    radio.startReceive();

    // we're ready to receive more packets,
    // enable interrupt service routine
    enableInterrupt_1 = true;
  }

  if (MAIN_LIVE && millis() - last_received_time > 1000)
  {
    radio.setDio0Action(setFlag);
    MAIN_LIVE = false;
    receivedFlag_1 = false;
    transmittedFlag = true;
    digitalWrite(LED_GREEN, LOW);
    buzzer(150);
  }
}
