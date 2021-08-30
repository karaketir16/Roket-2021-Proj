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

RFM95 radio_1 = new Module(NANO_1_RFM95_CS, NANO_1_RFM95_INT, NANO_1_RFM95_RST, NANO_1_RFM95_DUMMY);
RFM95 radio_2 = new Module(NANO_2_RFM95_CS, NANO_2_RFM95_INT, NANO_2_RFM95_RST, NANO_2_RFM95_DUMMY);

// or using RadioShield
// https://github.com/jgromes/RadioShield
//SX1278 radio = RadioShield.ModuleA;

packet p1;

packet_payload p_payload;

// flag to indicate that a packet was received
volatile bool receivedFlag_1 = false;
volatile bool receivedFlag_2 = false;

// disable interrupt when it's not needed
volatile bool enableInterrupt_1 = true;
volatile bool enableInterrupt_2 = true;

// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
void setFlag_1(void) {
  // check if the interrupt is enabled
  if(!enableInterrupt_1) {
    return;
  }

  // we got a packet, set the flag
  receivedFlag_1 = true;
}
void setFlag_2(void) {
  // check if the interrupt is enabled
  if(!enableInterrupt_2) {
    return;
  }

  // we got a packet, set the flag
  receivedFlag_2 = true;
}

int state;

void setup() {

  Serial.begin(38400);
  

  // initialize SX1278 with default settings
  Serial.print(F("[RADIO1] Initializing ... "));
  state = radio_1.begin(PAYLOAD_RF95_FREQ,RF95_BW,RF95_SF,RF95_CR, RF95_SYNC_WORD, RF95_POWER, RF95_PL, RF95_GAIN);
  if (state == ERR_NONE) {
    Serial.println(F("success 1!"));
  } else {
    Serial.print(F("failed 1, code "));
    Serial.println(state);
    while (true);
  }

  Serial.print(F("[RADIO2] Initializing ... "));
  state = radio_2.begin(PAYLOAD_RF95_FREQ,RF95_BW,RF95_SF,RF95_CR, RF95_SYNC_WORD, RF95_POWER, RF95_PL, RF95_GAIN);
  if (state == ERR_NONE) {
    Serial.println(F("success 2!"));
  } else {
    Serial.print(F("failed 2, code "));
    Serial.println(state);
    while (true);
  }

  // set the function that will be called
  // when new packet is received
  radio_1.setDio0Action(setFlag_1);
  radio_2.setDio0Action(setFlag_2);

  // start listening for LoRa packets
  Serial.print(F("[SX1278] Starting to listen ... "));
  state = radio_1.startReceive();
  if (state == ERR_NONE) {
    Serial.println(F("success 1!"));
  } else {
    Serial.print(F("failed 1, code "));
    Serial.println(state);
    while (true);
  }
    // start listening for LoRa packets
  Serial.print(F("[SX1278] Starting to listen ... "));
  state = radio_2.startReceive();
  if (state == ERR_NONE) {
    Serial.println(F("success 2!"));
  } else {
    Serial.print(F("failed 2, code "));
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

int last_1 = 0;
int total_received_1 = 0;
int loss_1 = 0;

int last_2 = 0;
int total_received_2 = 0;
int loss_2 = 0;

void loop() {
  // check if the flag is set

  if(receivedFlag_1) {
    // disable the interrupt service routine while
    // processing the data
    enableInterrupt_1 = false;

    // reset flag
    receivedFlag_1 = false;

    //test
    {
          // put module back to listen mode
      radio_1.readData((byte*) &p_payload, sizeof p_payload);

      Serial.println("[Radio 1] Received packet: ");
      Serial.print("rssi: ");
      Serial.println(radio_1.getRSSI()); //dbm
      // print SNR (Signal-to-Noise Ratio)
      Serial.print("snr: ");
      Serial.println(radio_1.getSNR()); //db
      // Serial.println(F(" dB"));

      // print frequency error
      Serial.print("frequency_error: ");
      Serial.println(radio_1.getFrequencyError());

      Serial.println();
      Serial.println();
      Serial.println();

      radio_1.startReceive();

      enableInterrupt_1 = true;

      return;
    }

   int state = radio_1.readData((byte*) &p1, sizeof p1);

    if (state == ERR_NONE) {

      if(check_checksum((char *)&p1, sizeof p1)){
      
      // packet was successfully received
      Serial.println("[SX1278] Received packet!");

      if (p1.package_number > 2){
        if( last_1 == 0 ){
          last_1 = p1.package_number - 1;
        }
        total_received_1++;
        loss_1 += (p1.package_number - last_1) - 1;
        last_1 = p1.package_number;
      }

      // print data of the packet
      Serial.println("[SX1278] Data:");

      Serial.print("${");

      Serial.print("\"type\":");
      Serial.println("\"rocket\"");

      Serial.print("\"package_number\":");
      Serial.println(p1.package_number);

      Serial.print(",\"total_received\":");
      Serial.println(total_received_1);

      Serial.print(",\"loss\":");
      Serial.println(loss_1);

      Serial.print(",\"loss_ratio\":");
      Serial.println( 100 * (float) loss_1 / (float) (total_received_1 + loss_1));

      Serial.print(",\"pressure\": ");
      Serial.println(p1.pressure);

      Serial.print(",\"altitude\": ");
      Serial.println(p1.altitude);

      // Serial.print(",\"temp\": ");
      // Serial.println(p1.temperature);

      Serial.print(",\"gps_fix\": ");
      Serial.println((int)p1.gps_fix ? "true" : "false");

      Serial.print(",\"gps_latitude\": ");
      Serial.println(p1.gps_latitude, 5);

      Serial.print(",\"gps_longtitude\": ");
      Serial.println(p1.gps_longtitude, 5);

      // Serial.print(",\"acc_x\": ");
      // Serial.println(p1.acc_x);

      // Serial.print(",\"acc_y\": ");
      // Serial.println(p1.acc_y);

      // Serial.print(",\"acc_z\": ");
      // Serial.println(p1.acc_z);

      // print RSSI (Received Signal Strength Indicator)
      Serial.print(F(",\"rssi\":"));
      Serial.print(radio_1.getRSSI());
      // Serial.println(F(" dBm"));

      // print SNR (Signal-to-Noise Ratio)
      Serial.print(F(",\"snr\":"));
      Serial.print(radio_1.getSNR());
      // Serial.println(F(" dB"));

      // print frequency error
      Serial.print(F(",\"frequency_error\":"));
      Serial.print(radio_1.getFrequencyError());
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
    radio_1.startReceive();

    // we're ready to receive more packets,
    // enable interrupt service routine
    enableInterrupt_1 = true;
  }



  if(receivedFlag_2) {
    // disable the interrupt service routine while
    // processing the data
    enableInterrupt_2 = false;

    // reset flag
    receivedFlag_2 = false;

    //test
    {
          // put module back to listen mode
      radio_2.readData((byte*) &p_payload, sizeof p_payload);

      Serial.println("[Radio 2] Received packet: ");
      Serial.print("rssi: ");
      Serial.println(radio_2.getRSSI()); //dbm
      // print SNR (Signal-to-Noise Ratio)
      Serial.print("snr: ");
      Serial.println(radio_2.getSNR()); //db
      // Serial.println(F(" dB"));

      // print frequency error
      Serial.print("frequency_error: ");
      Serial.println(radio_2.getFrequencyError());

      Serial.println();
      Serial.println();
      Serial.println();

      radio_2.startReceive();

      enableInterrupt_2 = true;

      return;
    }

   int state = radio_2.readData((byte*) &p_payload, sizeof p_payload);

    if (state == ERR_NONE) {

      if(check_checksum((char *)&p_payload, sizeof p_payload)){
      
      // packet was successfully received
      Serial.println("[SX1278] Received packet!");

      if (p_payload.package_number > 2){
        if( last_2 == 0 ){
          last_2 = p_payload.package_number - 1;
        }
        total_received_2++;
        loss_2 += (p_payload.package_number - last_2) - 1;
        last_2 = p_payload.package_number;
      }

      // print data of the packet
      Serial.println("[SX1278] Data:");

      Serial.print("${");

      Serial.print("\"type\":");
      Serial.println("\"payload\"");

      Serial.print("\"package_number\":");
      Serial.println(p_payload.package_number);

      Serial.print(",\"total_received\":");
      Serial.println(total_received_2);

      Serial.print(",\"loss\":");
      Serial.println(loss_2);

      Serial.print(",\"loss_ratio\":");
      Serial.println( 100 * (float) loss_2 / (float) (total_received_2 + loss_2));

      Serial.print(",\"pressure\": ");
      Serial.println(p_payload.pressure);

      Serial.print(",\"altitude\": ");
      Serial.println(p_payload.altitude);

      Serial.print(",\"temp\": ");
      Serial.println(p_payload.temp);

      Serial.print(",\"gps_fix\": ");
      Serial.println((int)p_payload.gps_fix ? "true" : "false");

      Serial.print(",\"gps_latitude\": ");
      Serial.println(p_payload.gps_latitude, 5);

      Serial.print(",\"gps_longtitude\": ");
      Serial.println(p_payload.gps_longtitude, 5);

      Serial.print(",\"acc_x\": ");
      Serial.println(p_payload.acc_x);

      Serial.print(",\"acc_y\": ");
      Serial.println(p_payload.acc_y);

      Serial.print(",\"acc_z\": ");
      Serial.println(p_payload.acc_z);

      // print RSSI (Received Signal Strength Indicator)
      Serial.print(F(",\"rssi\":"));
      Serial.print(radio_2.getRSSI());
      // Serial.println(F(" dBm"));

      // print SNR (Signal-to-Noise Ratio)
      Serial.print(F(",\"snr\":"));
      Serial.print(radio_2.getSNR());
      // Serial.println(F(" dB"));

      // print frequency error
      Serial.print(F(",\"frequency_error\":"));
      Serial.print(radio_2.getFrequencyError());
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
    radio_2.startReceive();

    // we're ready to receive more packets,
    // enable interrupt service routine
    enableInterrupt_2 = true;
  }

}


// GPS: $GPRMC,181955.500,A,3837.0524,N,03443.1108,E,0.49,51.74,260521,,,A*56