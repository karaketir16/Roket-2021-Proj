#include <Arduino.h>
#include "gps.h"

extern HardwareSerial GPSSerial;

//$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
String gps_read(){

    char c = 0;

    String str = "$";

    while(GPSSerial.read() >= 0 );

    while ( true ) {
        if( '$' == GPSSerial.read()){
            break;
        }
    } 

    while ( true ){
        if (GPSSerial.available() > 0){
            c = GPSSerial.read();
            str += c;
            if (c == '\n'){
                break;
            }
        }
    }

    return str;
}

void gps_setup(){
        // 
    GPSSerial.begin(115200);
    GPSSerial.print(PMTK_SET_BAUD_38400 END); 
    delay(200);
    GPSSerial.end();
    
    GPSSerial.begin(9600);
    GPSSerial.print(PMTK_SET_BAUD_38400 END);
    delay(200);
    GPSSerial.end();
    
    GPSSerial.begin(38400);
    GPSSerial.print(PMTK_SET_BAUD_38400 END); 
    delay(200);
    GPSSerial.end();
    
    
    // GPSSerial.end();
    // GPSSerial.begin(115200);
    GPSSerial.begin(38400);
    delay(200);
    GPSSerial.print(PMTK_SET_NMEA_OUTPUT_RMCONLY END);  //(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    delay(200);
    GPSSerial.print(PMTK_SET_NMEA_UPDATE_10HZ END); // 10 Hz update rate
    delay(200);
    GPSSerial.print(PGCMD_NOANTENNA END);
    delay(200);
}
