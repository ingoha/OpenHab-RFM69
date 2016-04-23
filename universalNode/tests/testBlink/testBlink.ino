/*
Author: Ingo Haschler
Test program for breadboard arduino
Components covered:
 * voltage regulator
 * ATmega
 * LED
Expected result: LED blinks 
*/

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







