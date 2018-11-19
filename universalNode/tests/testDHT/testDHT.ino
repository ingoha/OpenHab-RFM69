/*
  Author: Ingo Haschler
  Test program for breadboard arduino
  Components covered:
   voltage regulator
   ATmega
   LED
   DHT
  Expected result: LED blinks and then stays on
*/
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

//
// Arduino
// Arduino pro mini has LED on D13 (https://www.arduino.cc/en/Main/ArduinoBoardProMini)
//
#define LED           8    // breadboard arduino (low power) has LED on pin 8
#define SERIAL_BAUD   9600  //must be 9600 for GPS, use whatever if no GPS

//
// DHT
//
#define DHTPIN 7           // digital pin we're connected to
// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)
// Initialize DHT sensor for 8mhz Arduino
DHT_Unified dht(DHTPIN, DHTTYPE);

void setup()
{
  dht.begin();
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);              // wait for a second
  digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);              // wait for a second
}

void loop()
{
  // Get temperature event and print its value.
  sensors_event_t event;  
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature))
  {
    digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
  }
  else
  {
    digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  }
}//end loop







