/*
google maps link
https://www.google.com/maps/?q=27.988139,86.924974

SMS reading from storage
https://mechatrofice.com/arduino/gsm-send-sms

GPGLL
https://docs.novatel.com/OEM7/Content/Logs/GPGLL.htm


NMEA parser
http://swairlearn.bluecover.pt/nmea_analyser


1.1V internal reference

*/


#include "TinyGPS++.h"          //http://arduiniana.org/libraries/tinygpsplus/
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif
#include <OneWire.h>
#include <SoftwareSerial.h>

//PC_RX                         0
//PC_TX                         1
#define BUZZER_PIN              A3
#define DS18B20_PIN             A4
#define NEOPIXEL_POWER_PIN      A2
#define NEOPIXEL_PIN            A5
//ACC_MISO                      12
//ACC_MOSI                      11
//ACC_SCK                       13
#define ACC_CS_PIN              4 //?
#define ACC_INT_PIN             2
#define GSM_RX_PIN              8
#define GSM_TX_PIN              7 //?
#define GSM_POWER_PIN           5 //?
#define GPS_RX_PIN              9
#define GPS_TX_PIN              6 //?
#define GPS_POWER_PIN           A0
#define BUTTON_PIN              3
#define BATTERY_EN_PIN          A1
#define BATTERY_PIN             A6
#define CHARGER_PIN             A7



OneWire  ds(DS18B20_PIN);  // on pin 10 (a 4.7K resistor is necessary)
#define NUMPIXELS      16   // How many NeoPixels are attached to the Arduino?
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
SoftwareSerial portGPS(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  portGPS.begin(9600);
  pixels.begin(); // This initializes the NeoPixel library.


  pixels.setPixelColor(0, pixels.Color(0,10,0)); // Moderately bright green color.
  pixels.setPixelColor(1, pixels.Color(10,0,0)); // Moderately bright green color.
  pixels.setPixelColor(2, pixels.Color(0,0,10)); // Moderately bright green color.
  pixels.show(); // This sends the updated pixel color to the hardware.
}

enum modes{
  sleepMode,    // Gets up once a day, to measure batt voltage, and send SMS if it is too low. Wait for external IT
  guardMode,    // Get out from sleep if acc sensor trigger, in this mode a longer acceleration triggers a warn SMS. GPS and GSM active
  disarmedMode, // After disarmig it gets into this mode. You can ride the bike. If you stop, and no move occures, it will fall back to guard mode.
  armedMode,    // It is similar to guard mode, but almost every acc generates a notification sound and police light.
  bikeLightMode // it is like disarmMode, but with blinking lights.
  //charging??
};

void readOneWireKey()
{
    Serial.println("START");
  byte addr[8];
  int i = 0;
    if ( !ds.search(addr)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(250);
    return;
  }
  
  Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }

  ds.reset_search();
  Serial.println("END");
}

void handleGpsSerial()
{
  if(portGPS.isListening() == false){
      portGPS.listen();
  }
  unsigned long startTime = millis();
  while(millis() < startTime + 1000){
    while (portGPS.available() > 0){
      gps.encode(portGPS.read());
    }
  }

 /* 
    portGPS.listen();
 // Serial.println("Data from port one:");
  // while there is data coming in, read it
  // and send to the hardware serial port:
  while (portGPS.available() > 0) {
    char inByte = portGPS.read();
    Serial.write(inByte);
  }*/
}


void loop(void) {

  readOneWireKey();
  handleGpsSerial();
 // if (gps.altitude.isUpdated())
  //Serial.println(gps.altitude.meters());
  Serial.print("Age: ");
  Serial.println(gps.location.age());
  Serial.print("Pos: ");
  Serial.print(gps.location.lat(), 6);
  Serial.print(" ");
  Serial.println(gps.location.lng(), 6);
   Serial.print(" ");
  //delay(500);
}


