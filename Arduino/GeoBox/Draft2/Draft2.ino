


/* Engagement Box   by Kenton Harris 11/12/2012
Reverse Geocache Engagement Ring Box
This program unlocks a box that has reached a certain location.
 
The device uses the following products from Adafruit Industries:
Arduino Uno: https://www.adafruit.com/products/50
Ultimate GPS (version1): http://www.adafruit.com/products/746
16x2 Character LCD: https://www.adafruit.com/products/181
TPro Micro Servo SG90: https://www.adafruit.com/products/169
Half Sized Perma proto: https://www.adafruit.com/products/571
 
Tutorials for these products found on learn.adafruit.com helped with much of this.
Copyright (c) 2012, Adafruit Industries
All rights reserved.
 
Thanks to bnordlund9 for much of the code. This is  simplified verison of his Geobox found here:
http://www.youtube.com/watch?v=g0060tcuofg
Credit to Mikal Hart of http://arduiniana.org/ for the original idea of Reverse Geocache.
 
*/
#include <SPI.h> 
#include <math.h>
#include <LiquidCrystal.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
SoftwareSerial mySerial(3,2);
Adafruit_GPS GPS(&mySerial);
#define GPSECHO false												     //make true to debug GPS
boolean usingInterrupt = false;
void useInterrupt(boolean);


//Declarations
const float deg2rad = 0.01745329251994;
const float rEarth = 3958.750;                                          //can replace with 3958.75 mi, 6370.0 km, or 3440.06 NM
float range = 3000;                                                      // distance from HERE to THERE
String here;                                                             // read from GPS
 
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(7,8,9,10,11,12);
 
int gpsWasFixed = HIGH;                                                  // did the GPS have a fix?
int solenoidPin = 4;                                                     // pin for the locking solenoid
int switchOffPin = 5; 


 
 // pin to turn off power switch 
//digitalWrite(switchOffPin,LOW);                                         // making sure the power off pin is not enabled. 
String there = "N47 40.543, W122 12.436";                                //Desired Location goes here. Make sure you use the same syntax and number of characters
 
void setup()
{

  pinMode(solenoidPin, OUTPUT);
  pinMode(switchOffPin, OUTPUT);
  
  delay(50);
  lcd.begin(16, 2);
  Serial.begin(115200);
  Serial.println("Debug GPS Test:");
 
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);                          // RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);                             // 1 Hz update rate
  useInterrupt(true);                                                    // reads the steaming data in a background
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Welcome");
  delay(1000);
  
 
}
 
void loop(){
  // Parse GPS and recalculate RANGE
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))                                      // also sets the newNMEAreceived() flag to false
      return;                                                            // We can fail to parse a sentence in which case we should just wait for another
  }
  if (GPS.fix) {
    gpsWasFixed = HIGH;
    digitalWrite(solenoidPin,HIGH);
 
  
    here = gps2string ((String) GPS.lat, GPS.latitude, (String) GPS.lon, GPS.longitude);
    range = haversine(string2lat(here), string2lon(here), string2lat(there), string2lon(there));
   /*
    Serial.print("Here: ");                                        //for GPS debug
    Serial.print(here);
    Serial.print("There: ");
    Serial.println(there);
    Serial.print("Range: ");
    Serial.print(range);
    Serial.println("m");
    */
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(range);
    lcd.print(" miles");
    lcd.setCursor(0,1);
    lcd.print("50 tries left");
    delay(10000);
        //digitalWrite(switchOffPin,HIGH);
  }
  else {                                                              //No GPS fix- take box outside
    lcd.clear();
    delay(500);
    lcd.setCursor(0,0);
    lcd.print("Searching...");
    lcd.setCursor(0,1);
    lcd.print("Take me outside!");
    delay(10000);
    
    if(!GPS.fix){
      lcd.clear();
      delay(500);
      lcd.setCursor(0,0);
      lcd.print("Searching...");
      lcd.setCursor(0,1);
      lcd.print("Take me outside!");
      delay(10000);
    }
    
    if(!GPS.fix){
      lcd.clear();
      delay(500);
      lcd.setCursor(0,0);
      lcd.print("This might take");
      lcd.setCursor(0,1);
      lcd.print("a few minutes");
      delay(10000);
    }
    
    if(!GPS.fix){
      lcd.clear();
      delay(500);
      lcd.setCursor(0,0);
      lcd.print("Searching...");
      lcd.setCursor(0,1);
      lcd.print("Take me outside!");
      delay(10000);
    }
    
    if(!GPS.fix){
      lcd.clear();
      delay(500);
      lcd.setCursor(0,0);
      lcd.print("This might take");
      lcd.setCursor(0,1);
      lcd.print("a few minutes");
      delay(5000);
    }
    
    if(!GPS.fix){
      lcd.clear();
      delay(500);
      lcd.setCursor(0,0);
      lcd.print("How about a joke");
      lcd.setCursor(0,1);
      lcd.print("while we wait");
      delay(10000);
      
      lcd.clear();
      delay(500);
      lcd.setCursor(0,0);
      lcd.print("what's the");
      lcd.setCursor(0,1);
      lcd.print("difference");
      delay(3000);
  
      lcd.clear();
      delay(500);
      lcd.setCursor(0,0);
      lcd.print("between a hippo");
      lcd.setCursor(0,1);
      lcd.print("and a zippo?");
      delay(5000);
  
      lcd.clear();
      delay(500);
      lcd.setCursor(0,0);
      lcd.print("what's the");
      lcd.setCursor(0,1);
      lcd.print("difference");
      delay(3000);
  
      lcd.clear();
      delay(500);
      lcd.setCursor(0,0);
      lcd.print("between a hippo");
      lcd.setCursor(0,1);
      lcd.print("and a zippo?");
      delay(10000);
  
      lcd.clear();
      delay(500);
      lcd.setCursor(0,0);
      lcd.print("one is a little");
      lcd.setCursor(0,1);
      lcd.print("lighter!!!");
      delay(10000); 
   
      lcd.clear();
      delay(500);
      lcd.setCursor(0,0);
      lcd.print("Ha Ha!!");
      delay(10000);    
    }
    
    
    
  }
  
  if (range < 5){
    digitalWrite(solenoidPin,HIGH);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Found it!");
    lcd.setCursor(1,0);
    lcd.print("Open Box");
    digitalWrite(solenoidPin,HIGH);
    delay(10000);
  }
}
 
 
SIGNAL(TIMER0_COMPA_vect) {
  // Interrupt is called once a millisecond, looks for any new GPS data, and stores it
  char c = GPS.read();
  if (GPSECHO)
    if (c) UDR0 = c;  
}
 
void useInterrupt(boolean v) {
  if (v) {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
 
String int2fw (int x, int n) {
  // returns a string of length n (fixed-width)
  String s = (String) x;
  while (s.length() < n) {
    s = "0" + s;
  }
  return s;
}
 
String gps2string (String lat, float latitude, String lon, float longitude) {
  // returns "Ndd mm.mmm, Wddd mm.mmm";
  int dd = (int) latitude/100;
  int mm = (int) latitude % 100;
  int mmm = (int) round(1000 * (latitude - floor(latitude)));
  String gps2lat = lat + int2fw(dd, 2) + " " + int2fw(mm, 2) + "." + int2fw(mmm, 3);
  dd = (int) longitude/100;
  mm = (int) longitude % 100;
  mmm = (int) round(1000 * (longitude - floor(longitude)));
  String gps2lon = lon + int2fw(dd, 3) + " " + int2fw(mm, 2) + "." + int2fw(mmm, 3);
  String myString = gps2lat + ", " + gps2lon;
  return myString;
};
/*
float string2radius (String myString) {
  // returns a floating-point number: e.g. String myString = "Radius: 005.1 NM";
  float r = ((myString.charAt(8) - '0') * 100.0) + ((myString.charAt(9) - '0') * 10.0) + ((myString.charAt(10) - '0') * 1.0) + ((myString.charAt(12) - '0') * 0.10);
  return r;
};*/
 
float string2lat (String myString) {
  // returns radians: e.g. String myString = "N38 58.892, W076 29.177";
  float lat = ((myString.charAt(1) - '0') * 10.0) + (myString.charAt(2) - '0') * 1.0 + ((myString.charAt(4) - '0') / 6.0) + ((myString.charAt(5) - '0') / 60.0) + ((myString.charAt(7) - '0') / 600.0) + ((myString.charAt(8) - '0') / 6000.0) + ((myString.charAt(9) - '0') / 60000.0);
  //Serial.print("float lat: ");
  //Serial.println(lat);
  lat *= deg2rad;
  if (myString.charAt(0) == 'S')
    lat *= -1;                                                           // Correct for hemisphere
  return lat;
};
 
float string2lon (String myString) {
  // returns radians: e.g. String myString = "N38 58.892, W076 29.177";
  float lon = ((myString.charAt(13) - '0') * 100.0) + ((myString.charAt(14) - '0') * 10.0) + (myString.charAt(15) - '0') * 1.0 + ((myString.charAt(17) - '0') / 6.0) + ((myString.charAt(18) - '0') / 60.0) + ((myString.charAt(20) - '0') / 600.0) + ((myString.charAt(21) - '0') / 6000.0) + ((myString.charAt(22) - '0') / 60000.0);
  //Serial.print("float lon: ");
  //Serial.println(lon);
  lon *= deg2rad;
  if (myString.charAt(12) == 'W')
    lon *= -1;                                                           // Correct for hemisphere
  return lon;
};
 
 
float haversine (float lat1, float lon1, float lat2, float lon2) {
  // returns the great-circle distance between two points (radians) on a sphere
  float h = sq((sin((lat1 - lat2) / 2.0))) + (cos(lat1) * cos(lat2) * sq((sin((lon1 - lon2) / 2.0))));
  float d = 2.0 * rEarth * asin (sqrt(h)); 
  //Serial.println(d);
  return d;
  
};
