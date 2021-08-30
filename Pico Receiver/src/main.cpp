/*
   RadioLib SX127x Receive Example

   This example listens for LoRa transmissions using SX127x Lora modules.
   To successfully receive data, the following settings have to be the same
   on both transmitter and receiver:
    - carrier frequency
    - bandwidth
    - spreading factor
    - coding rate
    - sync word
    - preamble length

   Other modules from SX127x/RFM9x family can also be used.

   For default module settings, see the wiki page
   https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem

   For full API reference, see the GitHub Pages
   https://jgromes.github.io/RadioLib/
*/

#include <SPI.h>
#include <RadioLib.h>

#include "shared.h"

// #define SIMULATION

RFM95 radio = new Module(PICO_1_RFM95_CS, PICO_1_RFM95_INT, PICO_1_RFM95_RST, PICO_1_RFM95_DUMMY);

// or using RadioShield
// https://github.com/jgromes/RadioShield
//SX1278 radio = RadioShield.ModuleA;

packet pac1;

// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// disable interrupt when it's not needed
volatile bool enableInterrupt = true;

// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
void setFlag(void) {
  // check if the interrupt is enabled
  if(!enableInterrupt) {
    return;
  }

  // we got a packet, set the flag
  receivedFlag = true;
}

void setup() {
  // ser();
  Serial.begin(38400);

  // initialize SX1278 with default settings
  Serial.print(F("[SX1278] Initializing ... "));
  int state = radio.begin(RF95_FREQ,RF95_BW,RF95_SF,RF95_CR, RF95_SYNC_WORD, RF95_POWER, RF95_PL, RF95_GAIN);
  if (state == ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  // set the function that will be called
  // when new packet is received
  radio.setDio0Action(setFlag);

  // start listening for LoRa packets
  Serial.print(F("[SX1278] Starting to listen ... "));
  state = radio.startReceive();
  if (state == ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  // if needed, 'listen' mode can be disabled by calling
  // any of the following methods:
  //
  // radio.standby()
  // radio.sleep()
  // radio.transmit();
  // radio.receive();
  // radio.readData();
  // radio.scanChannel();
}

int last = 0;
int total_received = 0;
int loss = 0;

void loop() {
  // check if the flag is set
  packet pac1;
  if(receivedFlag) {
    // disable the interrupt service routine while
    // processing the data
    enableInterrupt = false;

    // reset flag
    receivedFlag = false;

    // you can read received data as an Arduino String
    /*
    String str;
    int state = radio.readData(str);
*/
    // you can also read received data as byte array
    /*
      byte byteArr[8];
      int state = radio.readData(byteArr, 8);
    */
   int state = radio.readData((byte*) &pac1, sizeof pac1);

    if (state == ERR_NONE) {


      if(check_checksum((char *)&pac1, sizeof pac1)){

      
      // packet was successfully received
      Serial.println("[SX1278] Received packet!");



      if (pac1.package_number > 2){
        if( last == 0 ){
          last = pac1.package_number - 1;
        }
        total_received++;
        loss += (pac1.package_number - last) - 1;
        last = pac1.package_number;
      }

      // print data of the packet
      Serial.println("[SX1278] Data:");

      Serial.print("${");
      Serial.print("\"package_number\":");
      Serial.println(pac1.package_number);

      Serial.print(",\"total_received\":");
      Serial.println(total_received);

      Serial.print(",\"loss\":");
      Serial.println(loss);

      Serial.print(",\"loss_ratio\":");
      Serial.println( 100 * (float) loss / (float) (total_received + loss));

      Serial.print(",\"pressure\": ");
      Serial.println(pac1.pressure);

      Serial.print(",\"altitude\": ");
      Serial.println(pac1.altitude);

      // Serial.print(",\"temp\": ");
      // Serial.println(pac1.temperature);

      Serial.print(",\"gps_fix\": ");
      Serial.println((int)pac1.gps_fix ? "true" : "false");

      Serial.print(",\"gps_latitude\": ");
      Serial.println(pac1.gps_latitude, 5);

      Serial.print(",\"gps_longtitude\": ");
      Serial.println(pac1.gps_longtitude, 5);

      // Serial.print(",\"acc_x\": ");
      // Serial.println(pac1.acc_x);

      // Serial.print(",\"acc_y\": ");
      // Serial.println(pac1.acc_y);

      // Serial.print(",\"acc_z\": ");
      // Serial.println(pac1.acc_z);

      // print RSSI (Received Signal Strength Indicator)
      Serial.print(F(",\"rssi\":"));
      Serial.print(radio.getRSSI());
      // Serial.println(F(" dBm"));

      // print SNR (Signal-to-Noise Ratio)
      Serial.print(F(",\"snr\":"));
      Serial.print(radio.getSNR());
      // Serial.println(F(" dB"));

      // print frequency error
      Serial.print(F(",\"frequency_error\":"));
      Serial.print(radio.getFrequencyError());
      // Serial.println(F(" Hz"));
      // Serial.println("-----------------------------------");
      Serial.println("}#");
      }
      else {
        Serial.println(F("[SX1278] Cheksum error!"));
      }

    } else if (state == ERR_CRC_MISMATCH) {
      // packet was received, but is malformed
      Serial.println(F("[SX1278] CRC error!"));

    } else {
      // some other error occurred
      Serial.print(F("[SX1278] Failed, code "));
      Serial.println(state);

    }

    // put module back to listen mode
    radio.startReceive();

    // we're ready to receive more packets,
    // enable interrupt service routine
    enableInterrupt = true;
  }

}


// GPS: $GPRMC,181955.500,A,3837.0524,N,03443.1108,E,0.49,51.74,260521,,,A*56