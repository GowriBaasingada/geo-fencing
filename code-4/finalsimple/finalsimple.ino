#include <NMEAGPS.h>
#include "Adafruit_FONA.h"
#include <SoftwareSerial.h>
#define FONA_RX 5
#define FONA_TX 4
#define FONA_RST 11
int buzzerpin=13;
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

#define gpsPort Serial
#define GPS_PORT_NAME "Serial"
#define DEBUG_PORT Serial

#if !defined( NMEAGPS_PARSE_RMC ) &  \
    !defined( NMEAGPS_PARSE_GGA ) &  \
    !defined( NMEAGPS_PARSE_GLL )
  #error You must uncomment at least one of NMEAGPS_PARSE_RMC, NMEAGPS_PARSE_GGA or NMEAGPS_PARSE_GLL in NMEAGPS_cfg.h!
#endif

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

#if !defined( GPS_FIX_LOCATION )
  #error You must uncomment GPS_FIX_LOCATION in GPSfix_cfg.h!
#endif

NMEAGPS gps;

// The base location, in degrees * 10,000,000
NeoGPS::Location_t base( -253448688L, 1310324914L ); // Ayers Rock, AU

void setup()
{
  DEBUG_PORT.begin(9600);
  DEBUG_PORT.println( F("NMEAdistance.ino started.") );
  DEBUG_PORT.println( F("Looking for GPS device on " GPS_PORT_NAME) );
  pinMode(13,INPUT);
  gpsPort.begin(9600);
   fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while(1);
  }
  Serial.println(F("FONA is OK"));

  // Print SIM card IMEI number.
  char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("SIM card IMEI: "); Serial.println(imei);
  }

  fonaSerial->print("AT+CNMI=2,1\r\n");  //set up the FONA to send a +CMTI notification when an SMS is received

  Serial.println("FONA Ready");

} // setup

void loop()
{
  
  while (gps.available( gpsPort )) {
    gps_fix fix = gps.read(); // save the latest

    // When we have a location, calculate how far away we are from the base location.
 
      float range = fix.location.DistanceMiles( base );

      DEBUG_PORT.print( F("Range: ") );
      DEBUG_PORT.print( range );
      DEBUG_PORT.println( "range in miles" );
   if (range>=8732.82){
    Serial.println("grater");
    digitalWrite(13,HIGH);
     if (!fona.sendSMS("7979952235", "Hey, This man gone out of range please catch")) {
        Serial.println(F("Failed"));
      } else {
        Serial.println(F("Sent!"));
        delay(4000);
      }
  }else{
    
  digitalWrite(13,LOW);
  delay(300);
}
  }
  
} // loop
