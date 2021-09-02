#include <Arduino.h>
#include "gps.h"
#include <Wire.h>
#include <SPI.h>

#include <Adafruit_Sensor.h>  // not used in this demo but required!

#include "bme.h"
#include "lsm.h"
#include "shared.h"

#include "minmea.h"

#include "rf.h"

// #define SIMULATION
// #define PRINT

HardwareSerial GPSSerial(USART2);

Adafruit_BMP3XX bmp;

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
void setFlag(void) {
  // check if the interrupt is enabled
  if(!enableInterrupt) {
    return;
  }

  // we sent a packet, set the flag
  transmittedFlag = true;
}

void buzzer(int dl){
  digitalWrite(PA12, HIGH);
  delay(dl);
  digitalWrite(PA12, LOW);
  delay(dl);
}

static float sim_alt = 0;
static float sim_mul = 75;

float simulation_altitude(){
  sim_alt += sim_mul;
  if(sim_alt > 3000){
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

void setup() {

  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_GREEN, HIGH);

  // digitalWrite(LED_GREEN, 0);


  Serial.begin(38400);
  Serial.println("Booting");

  delay(1000);

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

  int radio_state = radio.begin(RF95_FREQ,RF95_BW,RF95_SF,RF95_CR, RF95_SYNC_WORD, RF95_POWER, RF95_PL, RF95_GAIN);
  if (radio_state == ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(radio_state);
    while (true);
  }

  if (!lsm_0.begin())
  {
    while (1){
        delay(250);
        Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    }
  } else {
    Serial.println("Found LSM9DS1 9DOF");
  }
  
  setupLsm_0_Sensor();

  gps_setup();
  BMPsetup();
  for(int i =0 ; i < 10 ; i++){
    BMPloop();
  }

  float tmp = 0;
  for(int i =0 ; i < 10 ; i++){
    tmp += bmp.readAltitude(SEALEVELPRESSURE_HPA);
  }
  initial_altitude = tmp / 10;

  state = STATE::WAIT;


  for(int i = 0;i<2;i++){
    buzzer(500);
  }


  // set the function that will be called
  // when packet transmission is finished
  radio.setDio0Action(setFlag);

  radio.startTransmit((byte*) &p1, sizeof p1);
  
  last_speed_time = millis();
  timer = millis();
}

int pins[3] = {MAIN_YAY, MAIN_PAYLOAD, MAIN_ANA};

int stages_times[3] = {0,0,0};


void check_stages(){
  for(int i =0;i < 3;i++){
    if( millis() - stages_times[i] > 2000){
      digitalWrite(pins[i], LOW);
    }
  }
}
void fire_STAGE(int i){
  pinMode(pins[i], OUTPUT);
  digitalWrite(pins[i], HIGH);   
  stages_times[i] = millis();
}

bool led = false;

void loop() {
  
  lsm_0.read();  /* ask it to read in the data */ 

  /* Get a new sensor event */ 
  sensors_event_t a, m, g, temp;

  lsm_0.getEvent(&a, &m, &g, &temp); 

  p1.abs_vert_acc = abs(a.acceleration.x);
  // p1.acc_y = a.acceleration.y;
  // p1.acc_z = a.acceleration.z;

  float old_alt = p1.altitude;
  p1.altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA) - initial_altitude;

  #ifdef SIMULATION
  p1.altitude = simulation_altitude();
  
  #endif

  p1.speed = (p1.altitude - old_alt) / ( (millis() - last_speed_time) / 1000.0 );
  last_speed_time = millis();

  p1.pressure = bmp.pressure / 100.0;
  // p1.temperature = bmp.temperature;

  switch (state)
  {
  case STATE::INIT:
    break;
  case STATE::WAIT:
    if(p1.altitude > 200){
      state = STATE::RISING_STAGE;
    }
    break;
  case STATE::RISING_STAGE:
    if( p1.speed <= 3){
      state = STATE::FALLING_1;
      fire_STAGE(0);
      buzzer(75);
    }
    break;
  case STATE::FALLING_1:
    if( p1.altitude < 1600){
      state = STATE::FALLING_2;
      fire_STAGE(1);
      buzzer(75);
      buzzer(75);
    }
    check_stages();
    break;
  case STATE::FALLING_2:
    if( p1.altitude < 700){
      state = STATE::FALLING_3;
      fire_STAGE(2);
      buzzer(75);
      buzzer(75);
      buzzer(75);
    }
  check_stages();
    break;
  case STATE::FALLING_3:
    if( p1.altitude < 20){
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

  const char* line = gps_data.c_str();

  int r = minmea_sentence_id(line, false);

  switch (r) {
              case MINMEA_SENTENCE_RMC: {
                  struct minmea_sentence_rmc frame;
                  if (minmea_parse_rmc(&frame, line)) {

                      auto fix_check = minmea_tocoord(&frame.latitude);
                      p1.gps_fix = (fix_check == fix_check);
                      if(p1.gps_fix) {
                        p1.gps_latitude = minmea_tocoord(&frame.latitude);
                        p1.gps_longtitude = minmea_tocoord(&frame.longitude);
                      }
                  }
                  else {
                    p1.gps_fix = 0;
                  }
              } break;
  default:
      ;
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
  Serial.println(bmp.temperature);

  Serial.print("Acceleration x: ");
  Serial.println(a.acceleration.x);

  Serial.print("Acceleration y: ");
  Serial.println(a.acceleration.y);

  Serial.print("Acceleration z: ");
  Serial.println(a.acceleration.z);

#endif
  

  // check if the previous transmission finished
  if(transmittedFlag && millis() - timer > 200) {
    // disable the interrupt service routine while
    // processing the data
    enableInterrupt = false;

    // reset flag
    transmittedFlag = false;

    if (transmissionState == ERR_NONE) {
      // packet was successfully sent
      led ^= 1;
      digitalWrite(LED_GREEN, led);
      
      // analogWrite(LED_GREEN, 10000);

      Serial.println(F("transmission finished!**********************************"));

      Serial.print("Time: ");
      Serial.println(millis() - timer);
      timer = millis();
      
      // NOTE: when using interrupt-driven transmit method,
      //       it is not possible to automatically measure
      //       transmission data rate using getDataRate()

    } else {
      Serial.print(F("failed, code "));
      Serial.println(transmissionState);

    }
    p1.state = state;
    fill_checksum((char * )&p1, sizeof p1);
    p_send = p1;
    // Serial.printf("press: %f\n accx: %f\n", p_send.pressure, p_send.acc_x);
    int state = radio.startTransmit((byte*) &p_send, sizeof p_send);
    // int state = radio.startTransmit((byte*) &p_send, 10);
    // p1.package_number++;

    // we're ready to send more packets,
    // enable interrupt service routine
    enableInterrupt = true;
  }
}
