/*
Original Author: Eric Tsai
Updated by Ingo Haschler
License: CC-BY-SA, https://creativecommons.org/licenses/by-sa/2.0/
Date: 9-1-2014
Original File: UberSensor.ino
This sketch is for a wired Arduino w/ RFM69 wireless transceiver
Sends sensor data (gas/smoke, flame, PIR, noise, temp/humidity) back
to gateway.  See OpenHAB configuration file.
1) Update encryption string "ENCRYPTKEY" and network id, node id and frequency
2) pick sensors to enable
*/


/* sensor
node = 12
device ID
2 = 1222 = smoke or not
3 = 1232 = flame detected or not
4 = 1242 = human motion present or not
5 = 1252 = barking or not
6 = 1262, 1263 = temperature, humidity

*/

// Sensors (device ids)
// uncomment to enable

//#define SENSOR_GAS 2
//#define SENSOR_FLAME 3
//#define SENSOR_PIR 4
//#define SENSOR_SOUND 5
//#define SENSOR_TEMP_HUM 6
//#define SENSOR_LIGHT 7

// 2 = 1222 = smoke or not
// 3 = 1232 = flame detected or not
// 4 = 1242 = human motion present or not
// 5 = 1252 = barking or not
// IH: OMG...
// 6 = 1262, 1263 = temperature, humidity


//RFM69  --------------------------------------------------------------------------------------------------
#include <RFM69.h>
#include <SPI.h>
#define NODEID        12    //unique for each node on same network
#define NETWORKID     27  //the same on all nodes that talk to each other
#define GATEWAYID     1
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
//#define FREQUENCY   RF69_433MHZ
#define FREQUENCY   RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "ENCRYPTKEY" //exactly the same 16 characters/bytes on all nodes!
//#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define ACK_TIME      30 // max # of ms to wait for an ack
#define LED           13  // Arduino pro mini has LED on D13 (https://www.arduino.cc/en/Main/ArduinoBoardProMini)
#define SERIAL_BAUD   9600  //must be 9600 for GPS, use whatever if no GPS

boolean debug = 0;

//struct for wireless data transmission
typedef struct {		
  int       nodeID; 		//node ID (1xx, 2xx, 3xx);  1xx = basement, 2xx = main floor, 3xx = outside
  int       deviceID;		//sensor ID (2, 3, 4, 5)
  unsigned long   var1_usl; 		//uptime in ms
  float     var2_float;   	//sensor data?
  float     var3_float;		//battery condition?
} Payload;
Payload theData;

char buff[20];
byte sendSize=0;
boolean requestACK = false;
RFM69 radio;

//end RFM69 ------------------------------------------

// gas sensor================================================
#ifdef SENSOR_GAS
int GasSmokeAnalogPin = 0;      // potentiometer wiper (middle terminal) connected to analog pin 
int gas_sensor = -500;           // gas sensor value, current
int gas_sensor_previous = -500;  //sensor value previously sent via RFM
#endif

//temperature / humidity  =====================================
#ifdef SENSOR_TEMP_HUM
#include "dht11.h"
#define DHTPIN 7     			// digital pin we're connected to
// Initialize DHT sensor for normal 16mhz Arduino
dht11 dht;
#endif

// flame sensor ==============================================
#ifdef SENSOR_FLAME
int flameAnalogInput = A1;
int flame_status = 0;
int flameValue = -50;			      //analog value of current flame sensor
int flameValue_previous = -50;  //value previously sent via RFM
#endif

// Light sensor ===============================================
#ifdef SENSOR_LIGHT
int lightAnalogInput = A2;    //analog input for photo resistor
int lightValue = -50;
int lightValue_previous = -50;
#endif

// PIR sensor ================================================
#ifdef SENSOR_PIR
int PirInput = 5;
int PIR_status = 0;
int PIR_reading = 0;
int PIR_reading_previous = 0;
#endif


// sound sensor ==============================================
#ifdef SENSOR_SOUND
//sound sensor digital input pin
int soundInput = 6;
int sound_status = 0;
int sound_reading = 0;  //reading =1 mean no noise, 0=noise
int sound_reading_previous = 0;
#endif


// timings
unsigned long gas_time;			//sensor read time
unsigned long gas_time_send;	//sensor value transmission time
unsigned long flame_time;
unsigned long flame_time_send;
unsigned long pir_time;
//unsigned long pir_time_send;
unsigned long sound_time;
//unsigned long sound_time_Send;
unsigned long temperature_time;
unsigned long light_time;
unsigned long light_time_send;


void setup()
{
  Serial.begin(9600);          //  setup serial

  //RFM69-------------------------------------------
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  #ifdef IS_RFM69HW
    radio.setHighPower(); //uncomment only for RFM69HW!
  #endif
  radio.encrypt(ENCRYPTKEY);
  char buff[50];
  sprintf(buff, "\nTransmitting at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(buff);
  theData.nodeID = NODEID;  //this node id should be the same for all devices in this node
  //end RFM--------------------------------------------
  
  
  //temperature / humidity sensor
#ifdef SENSOR_TEMP_HUM
  dht.attach(DHTPIN);
#endif
  
  //sound/noise
#ifdef SENSOR_SOUND
  pinMode(soundInput, INPUT);
#endif

  //initialize times
  gas_time = millis();
  flame_time = millis();
  pir_time = millis();
  sound_time = millis();
  temperature_time = millis();
 
  //PIR sensor
#ifdef SENSOR_PIR
  pinMode(PirInput, INPUT);
#endif
}

void loop()
{
  
  unsigned long time_passed = 0;
  
  //===================================================================
  //device #2
  //read gas sensor
  // don't read analog pins too often (<1Hz), else caps never get to charge.
  //112 to 120 = normal, 400 = high
 #ifdef SENSOR_GAS
  time_passed = millis() - gas_time;
  //take care of millis roll over.  In case of roll over
  //if roll over, send next value again 
  if (time_passed < 0)
  {
	gas_time = millis();
	gas_time_send = -700000;
  }
  
  //Serial.print("gas time passed = ");
  //Serial.println(time_passed);
  if (time_passed > 5000)		//read gas sensor analog input every X seconds
  {
    gas_time = millis();		//update gas_time w/ when sensor last read
    gas_sensor = analogRead(GasSmokeAnalogPin);    // read the input pin
    if (debug){
      Serial.print("Gas = ");
      Serial.println(gas_sensor);
    }
	
    //send data if gas detected, or if big changes relative to value last sent, or if it's been a while
    if ((gas_sensor < (gas_sensor_previous - 70)) || ((gas_sensor > (gas_sensor_previous + 70)) || (700000 < (millis() - gas_time_send))))
    {
      gas_time_send = millis();  //update gas_time_send with when sensor value last transmitted
      
      theData.deviceID = 2;
      theData.var1_usl = millis();
      theData.var2_float = gas_sensor;
      theData.var3_float = gas_sensor + 100;		//null value;
      radio.sendWithRetry(GATEWAYID, (const void*)(&theData), sizeof(theData));
      gas_sensor_previous = gas_sensor;
      Serial.print("gas rfm = ");
      Serial.println(gas_sensor);
    }//end if send RFM
  }//end if time_passed >
#endif

  //===================================================================
  
  //delay(100);
  
  //===================================================================
  //device #3
  //flame
#ifdef SENSOR_FLAME

  time_passed = millis() - flame_time;
  if (time_passed < 0)
  {
    flame_time = millis();
    flame_time_send = -70000;
  }
  if (time_passed > 2000)  //how often to examine the flame sensor analog value
  {
    flame_time = millis();		//update time when sensor value last read
    flame_status = 0;
    flameValue = 0;

    //analog value:  usually 1023 for no fire, lower for fire.
    flameValue = analogRead(flameAnalogInput);
    if ((flameValue < (flameValue_previous - 20)) || ((flameValue > (flameValue_previous + 20)) || (705000 < (millis() - flame_time_send))) )
    {
      flame_time_send = millis();  //update gas_time_send with when sensor value last transmitted
      theData.deviceID = 3;
      theData.var1_usl = millis();
      theData.var2_float = flameValue;
      theData.var3_float = flameValue + 100;
      radio.sendWithRetry(GATEWAYID, (const void*)(&theData), sizeof(theData));
      flameValue_previous = flameValue;
      
      Serial.print("flame detected rfm");
      Serial.println(flameValue);
      delay(2000);
    }	
	
	
    //start debug code
    if (debug){   
    Serial.print("flame analog = ");
    Serial.print(flameValue);
    
    //analog value:  usually 1023 for no fire, lower for fire.
    if (flameValue > 1000)
    {
      flame_status = 0;
      Serial.println("   no fire");
    }
    else
    {
      flame_status = 1;
      Serial.println("    fire!!!");
    }
    }//end debug text
  }// end if millis time_passed >
#endif

 //===================================================================
  //device #4
  //PIR
#ifdef SENSOR_PIR

  //1 mean presence detected?
  PIR_reading = digitalRead(PirInput);
  //if (PIR_reading == 1)
  //Serial.println("PIR = 1");
  //else
  //Serial.println("PIR = 0");
  //send PIR sensor value only if presence is detected and the last time
  //presence was detected is over x miniutes ago.  Avoid excessive RFM sends
	if ((PIR_reading == 1) && ( ((millis() - pir_time)>60000)||( (millis() - pir_time)< 0)) ) //meaning there was sound
	{
		pir_time = millis();  //update gas_time_send with when sensor value last transmitted
		theData.deviceID = 4;
		theData.var1_usl = millis();
		theData.var2_float = 1111;
		theData.var3_float = 1112;		//null value;
		radio.sendWithRetry(GATEWAYID, (const void*)(&theData), sizeof(theData));
                Serial.println("PIR detectedEDED RFM");
                delay(2000);
    }
#endif  
  
 //===================================================================
  //device #5
  //sound
 
#ifdef SENSOR_SOUND
  //soundValue = analogRead(soundAnalogInput);
  //Serial.print("sound analog = ");
  //Serial.print(soundValue);
  
  // 1 = no noise, 0 = noise!!
  sound_reading = digitalRead(soundInput);
  //Serial.print("sound value = ");
  //Serial.println(sound_reading);
	if ((sound_reading == 0) && ( ((millis() - sound_time)>20000)||( (millis() - sound_time)< 0)) ) //meaning there was sound
	{
		sound_time = millis();  //update gas_time_send with when sensor value last transmitted
		
		theData.deviceID = 5;
		theData.var1_usl = millis();
		theData.var2_float = 2222;
		theData.var3_float = 2223;		//null value;
		radio.sendWithRetry(GATEWAYID, (const void*)(&theData), sizeof(theData));
                Serial.print("sound noise detected RFM ");
		sound_reading_previous = sound_reading;
	}


  /*
 if (sound_reading == 1)
   sound_status = 0;
    else
   sound_status = 1;
  if (sound_status == 1)  //noise!
  {
    Serial.print("sound noise = ");
    Serial.println(sound_reading);
  }
  */
  
  
  /*
  // analog value lower = louder
  if (soundValue < 200)
  {
    sound_status = 1;
    Serial.println("   loud");
  }
  else
  {
    sound_status = 0;
    Serial.println("    not loud!!!");
  }
  */

  //delay(100);
#endif

  //===================================================================
  //device #6
  //temperature / humidity
#ifdef SENSOR_TEMP_HUM
  time_passed = millis() - temperature_time;
  if (time_passed < 0)
  {
	temperature_time = millis();
  }
  
  if (time_passed > 360000)
  {
    int chk = dht.read();
    // Check if any reads failed and exit early (to try again).
    if (chk != 0) {
      Serial.println("Failed to read from DHT sensor!");
      return;
    }
    float h = (float)dht.humidity;
    // Read temperature as Celsius
    float t = (float)dht.temperature;
    // Read temperature as Fahrenheit
    float f = dht.fahrenheit();
    
    Serial.print("Humidity=");
    Serial.print(h);
    Serial.print("   Temp=");
    Serial.println(f);
    temperature_time = millis();

	//send data
	theData.deviceID = 6;
	theData.var1_usl = millis();
	theData.var2_float = f;
	theData.var3_float = h;
	radio.sendWithRetry(GATEWAYID, (const void*)(&theData), sizeof(theData));
        delay(1000);
	
  }

#endif
  //===================================================================
  //===================================================================
  //device #7
  //light
#ifdef SENSOR_LIGHT
  time_passed = millis() - light_time;
  if (time_passed < 0)
  {
    light_time = millis();
    light_time_send = -70000;
  }
  if (time_passed > 2000)  //how often to examine the sensor analog value
  {
    light_time = millis();		//update time when sensor value last read
    lightValue = 0;

    //analog value:  Less than 100 is dark.  greater than 500 is room lighting
    lightValue = analogRead(lightAnalogInput);
    if ((lightValue < (lightValue_previous - 50)) || ((lightValue > (lightValue_previous + 100)) || (705000 < (millis() - light_time_send))) )
    {
      light_time_send = millis();  //update gas_time_send with when sensor value last transmitted
      theData.deviceID = 7;
      theData.var1_usl = millis();
      theData.var2_float = lightValue;
      theData.var3_float = lightValue + 20;
      radio.sendWithRetry(GATEWAYID, (const void*)(&theData), sizeof(theData));
      lightValue_previous = lightValue;
      Serial.print("light RFM =");
      Serial.println(lightValue);
    }	
	
    //start debug code
    if (debug)
    {
      Serial.print("light analog = ");
      Serial.println(lightValue);
    }
    //analog value:  usually 1023 for no fire, lower for fire.
 
  }// end if millis time_passed >
  #endif
}//end loop






