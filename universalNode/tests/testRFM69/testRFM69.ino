/*
  Author: Ingo Haschler
  Test program for breadboard arduino
  Components covered:
   voltage regulator
   ATmega
   LED
   RFM69
  Expected result: LED blinks twice (slowly)
  Possible error states: LED blinks once and the blinks faster: failed to init RFM
*/


//
// Arduino
// Arduino pro mini has LED on D13 (https://www.arduino.cc/en/Main/ArduinoBoardProMini)
//
#define LED           8    // breadboard arduino (low power) has LED on pin 8
#define SERIAL_BAUD   9600  //must be 9600 for GPS, use whatever if no GPS


//
// RFM
//
#include <RFM69.h>  // from https://github.com/ingoha/RFM69-Arduino

#define NODEID        99    //unique for each node on same network
#define NETWORKID     27  //the same on all nodes that talk to each other
#define GATEWAYID     1
//Match frequency to the hardware version of the radio (uncomment one):
//#define FREQUENCY   RF69_433MHZ
#define FREQUENCY   RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "test" //exactly the same 16 characters/bytes on all nodes!
//#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define ACK_TIME      30 // max # of ms to wait for an ack

//struct for wireless data transmission
typedef struct {
    int       nodeID;     //node ID (1xx, 2xx, 3xx);  1xx = basement, 2xx = main floor, 3xx = outside
    int       deviceID;   //sensor ID (2, 3, 4, 5)
    unsigned long   uptime_ms;    //uptime in ms
    float     sensordata;     //sensor data?
    float     battery_volts;    //battery condition?
} Payload;

RFM69 radio;

void setup()
{
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);              // wait for a second
  digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);              // wait for a second
  if (!radio.initialize(FREQUENCY, NODEID, NETWORKID))
  {
    while (true)
    {
      digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(100);              // wait for 1/10 second
      digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
      delay(100);              // wait for 1/10 second
    }
  }
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
#endif
  radio.encrypt(ENCRYPTKEY);
  radio.setPowerLevel(31);
  digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);              // wait for a second
  digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);              // wait for a second

}

void loop()
{
  Payload msg;
  msg.nodeID = NODEID;
  msg.deviceID = 999;
  msg.uptime_ms = millis();
  msg.sensordata = 1.23;
  msg.battery_volts = 1.23;
  radio.sendWithRetry(GATEWAYID, (const void*)(&msg), sizeof(msg));
  radio.sleep();
  digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);              // wait for a second
  digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);              // wait for a second
  while (true);
}//end loop







