#ifndef _SHARED_
#define _SHARED_


#define RFM95_CS PB12
#define RFM95_RST PB13
#define RFM95_INT PB14
#define RFM95_DUMMY PB1

#define FEATHER_RFM95_CS 8
#define FEATHER_RFM95_RST 4
#define FEATHER_RFM95_INT 3
#define FEATHER_RFM95_DUMMY 0

//Carrier frequency in MHz. Allowed values range from 868.0 MHz to 915.0 MHz.
#define RF95_FREQ 877.2
#define RF95_FREQ_PAYLOAD 879.2
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

typedef struct __attribute__((packed)) packet_t
{
    int package_number;
    float altitude;
    float pressure;
    float speed;
    float gps_latitude;
    float gps_longtitude;
    char  gps_fix;
    char checksum;
} packet;


enum STATE {
  INIT,
  WAIT,
  RISING_STAGE,
  FALLING_1,
  FALLING_2,
  FALLING_3,
  END_STAGE
};

void fill_checksum(packet *p){
  p->checksum = 0;
  char *x = (char*)p;
  char tmp = 0;
  for(int i =0; i < (sizeof (packet)); i++){
    tmp += *x;
    x++;
  }
  p->checksum = tmp;
}

int check_checksum(packet *p){
  char check = p->checksum;
  p->checksum = 0;
  char *x = (char*)p;
  char tmp = 0;
  for(int i =0; i < (sizeof (packet)); i++){
    tmp += *x;
    x++;
  }
  return tmp == check;
}

#endif //_SHARED_
