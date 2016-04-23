/*
Original Author: Eric Tsai
Updated by Ingo Haschler
License: CC-BY-SA, https://creativecommons.org/licenses/by-sa/2.0/
Date: 9-1-2014
Original File: UberSensor.ino
This sketch is for a wired Arduino w/ RFM69 wireless transceiver
Sends sensor data (gas/smoke, flame, PIR, noise, temp/humidity) back
to gateway.  See OpenHAB configuration file.
*/


#include <SPI.h>

//
// Arduino
// Arduino pro mini has LED on D13 (https://www.arduino.cc/en/Main/ArduinoBoardProMini)
//
#define LED           8    // breadboard arduino (low power) has LED on pin 8
#define SERIAL_BAUD   9600  //must be 9600 for GPS, use whatever if no GPS

void setup()
{
  pinMode(LED, OUTPUT);
}

void loop()
{
  digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);              // wait for a second
  digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);              // wait for a second
}//end loop







