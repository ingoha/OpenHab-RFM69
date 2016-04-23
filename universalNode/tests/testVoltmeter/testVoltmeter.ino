/*
  Author: Ingo Haschler
  Test program for breadboard arduino
  Components covered:
   voltage regulator
   ATmega
   LED
   voltage divider
  Expected result: LED blinks once and then blinks in a frequency proportional to voltage readout
*/

//
// Arduino
// Arduino pro mini has LED on D13 (https://www.arduino.cc/en/Main/ArduinoBoardProMini)
//
#define LED           8    // breadboard arduino (low power) has LED on pin 8

// battery monitor
// TODO extract constants and move to config
float R1 = 1000000.0; // resistance of R1 (1M)
float R2 = 1000000.0; // resistance of R2 (1M)
float R1R2 = R1 / (R1 + R2); // proportion of voltage divider
double VOLTAGE = 3.3; // Arduino operating voltage
int BATTERY_PIN = 3;

void setup()
{
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);              // wait for a second
  digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);              // wait for a second
}

void loop()
{
  // read the value at analog input
  int value = analogRead(BATTERY_PIN);
  float vout = (value * VOLTAGE) / 1024.0;
  float vin = vout / R1R2;
  if (vin < 0.09) {
    vin = 0.0;//statement to quash undesired reading !
  }
  int frequency = 500 * vin;
  digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(frequency);              // wait for a second
  digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
  delay(frequency);              // wait for a second

}//end loop







