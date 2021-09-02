#ifndef _SHARED_
#define _SHARED_


#define MAIN_PAYLOAD PB5 //M3
#define MAIN_YAY     PB4 //M1
#define MAIN_ANA     PB3 //M2


#define RFM95_CS PB12
#define RFM95_RST PB13
#define RFM95_INT PB14
#define RFM95_DUMMY PB1

// #define SWTCH 

#ifdef SWTCH
  #define NANO_1_RFM95_CS 8
  #define NANO_1_RFM95_RST 7
  #define NANO_1_RFM95_INT 3
  #define NANO_1_RFM95_DUMMY 0

  #define NANO_2_RFM95_CS 10
  #define NANO_2_RFM95_RST 9
  #define NANO_2_RFM95_INT 2
  #define NANO_2_RFM95_DUMMY 0
#else
  #define NANO_1_RFM95_CS 10
  #define NANO_1_RFM95_RST 9
  #define NANO_1_RFM95_INT 2
  #define NANO_1_RFM95_DUMMY 0

  #define NANO_2_RFM95_CS 8
  #define NANO_2_RFM95_RST 7
  #define NANO_2_RFM95_INT 3
  #define NANO_2_RFM95_DUMMY 0
#endif

#define RECV1_RFM95_CS PB12
#define RECV1_RFM95_RST PB13
#define RECV1_RFM95_INT PB14
#define RECV1_RFM95_DUMMY PB1

#define RECV2_RFM95_CS PB3
#define RECV2_RFM95_RST PB4
#define RECV2_RFM95_INT PB5
#define RECV2_RFM95_DUMMY PB1

#define FEATHER_RFM95_CS 8
#define FEATHER_RFM95_RST 4
#define FEATHER_RFM95_INT 3
#define FEATHER_RFM95_DUMMY 0

//Carrier frequency in MHz. Allowed values range from 868.0 MHz to 915.0 MHz.
#define RF95_FREQ 911.2
#define PAYLOAD_RF95_FREQ 879.2
//%LoRa link bandwidth in kHz. Allowed values are 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250 and 500 kHz.
#define RF95_BW 500
//%LoRa link spreading factor. Allowed values range from 6 to 12.
#define RF95_SF 9
//%LoRa link coding rate denominator. Allowed values range from 5 to 8.
#define RF95_CR 7
//%LoRa sync word. Can be used to distinguish different networks. Note that value 0x34 is reserved for LoRaWAN networks.
#define RF95_SYNC_WORD 0x15
//Transmission output power in dBm. Allowed values range from 2 to 17 dBm.
#define RF95_POWER 17
//Length of %LoRa transmission preamble in symbols. The actual preamble length is 4.25 symbols longer than the set number.
  //Allowed values range from 6 to 65535.
#define RF95_PL 8
//Gain of receiver LNA (low-noise amplifier). Can be set to any integer in range 1 to 6 where 1 is the highest gain.
  //Set to 0 to enable automatic gain control (recommended).
#define RF95_GAIN 0

// #pragma pack(1)

#include <stdint.h>

typedef struct __attribute__((packed)) packet_t
{
    // uint32_t package_number;
    float altitude;
    float pressure;
    float speed;
    float gps_latitude;
    float gps_longtitude;
    float abs_vert_acc;
    char  gps_fix;
    char state;
    char checksum_1;
    char checksum_2;
} packet;


typedef struct __attribute__((packed)) packet_payload_t
{
    // uint32_t package_number;
    float altitude;
    float pressure;
    float speed;
    float gps_latitude;
    float gps_longtitude;
    float temp;
    float acc_x;
    float acc_y;
    float acc_z;
    char  gps_fix;
    char checksum_1;
    char checksum_2;
} packet_payload;


enum STATE {
  INIT,
  WAIT,
  RISING_STAGE,
  FALLING_1,
  FALLING_2,
  FALLING_3,
  END_STAGE
};

void fill_checksum(char *p, int sz);

int check_checksum(char *p, int sz);

#endif //_SHARED_
