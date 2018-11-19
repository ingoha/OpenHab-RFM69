/*
Original Author: Eric Tsai
Updated by Ingo Haschler
License: CC-BY-SA, https://creativecommons.org/licenses/by-sa/2.0/
Date: 9-1-2014
Original File: UberSensor.ino
This sketch is for a wired Arduino w/ RFM69 wireless transceiver
Sends sensor data (gas/smoke, flame, PIR, noise, temp/humidity) back
to gateway.  See OpenHAB configuration file.

Blink codes on startup: 1+2+3: everything OK, continous blinking: failed to init RFM module
*/


#include "config.h"

#include <LowPower.h> // from https://github.com/LowPowerLab/LowPower
#include <RFM69.h>  // from https://github.com/LowPowerLab/RFM69
#include <SPI.h>

//struct for wireless data transmission
typedef struct {
    int       nodeID; 		//node ID (1xx, 2xx, 3xx);  1xx = basement, 2xx = main floor, 3xx = outside
    int       deviceID;		//sensor ID (2, 3, 4, 5)
    unsigned long   uptime_ms; 		//uptime in ms
    float     sensordata;   	//sensor data?
    float     battery_volts;		//battery condition?
} Payload;

/*
 * RFM69 Pinout to arduino:
 *   MOSI = 11
 *   MISO = 12
 *   SCK = 13
 *   SS = 10
 */
RFM69 radio;

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

// new: "turn-based" system, 1 turn=8S
unsigned int temperature_skipped_turns;
// every 10 minutes = 600 seconds = 75 turns
#define TEMPERATURE_TURNS 75

// battery monitor
// TODO extract constants and move to config
float R1 = 1000000.0; // resistance of R1 (1M)
float R2 = 1000000.0; // resistance of R2 (1M)
float R1R2 = R1 / (R1 + R2); // proportion of voltage divider
double VOLTAGE = 3.3; // Arduino operating voltage
int BATTERY_PIN = 3;

// gas sensor================================================
#ifdef SENSOR_GAS
int GasSmokeAnalogPin = 0;      // potentiometer wiper (middle terminal) connected to analog pin
int gas_sensor = -500;           // gas sensor value, current
int gas_sensor_previous = -500;  //sensor value previously sent via RFM
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

#ifdef SENSOR_TEMP_HUM
#include "DHT.h"
// Initialize DHT sensor for 8mhz Arduino
DHT dht(DHTPIN, DHTTYPE, 2);
// NOTE: For working with a faster chip, like an Arduino Due or Teensy, you
// might need to increase the threshold for cycle counts considered a 1 or 0.
// You can do this by passing a 3rd parameter for this threshold.  It's a bit
// of fiddling to find the right value, but in general the faster the CPU the
// higher the value.  The default for a 16mhz AVR is a value of 6.  For an
// Arduino Due that runs at 84mhz a value of 30 works.
// Example to initialize DHT sensor for Arduino Due:
//DHT dht(DHTPIN, DHTTYPE, 30);

//
// Handles DHT sensor
//
void sensorTempHum() {
    float h = dht.readHumidity();
    // Read temperature as Celsius
    float t = dht.readTemperature();

    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t)) {
        debugPrintln("Failed to read from DHT sensor!");
        return;
    }

    debugPrintf("Humidity=", h);
    debugPrintf("   Temp=", t);

    temperature_time = millis();

    float voltage = batteryVoltage();

    //send data
    Payload msg;
    msg.nodeID = NODEID;
    msg.deviceID = SENSOR_TEMP_HUM;
    msg.uptime_ms = millis();
    msg.sensordata = t;
    msg.battery_volts = voltage;
    radio.send(GATEWAYID, (const void*)(&msg), sizeof(msg));

    blinkNTimes(1);

    msg.deviceID = SENSOR_HUMIDITY;
    msg.uptime_ms = millis();
    msg.sensordata = h;
    msg.battery_volts = voltage;
    radio.send(GATEWAYID, (const void*)(&msg), sizeof(msg));

    blinkNTimes(1);

    radio.sleep();
}
#endif

#ifdef SENSOR_GAS
//===================================================================
//device #2
//read gas sensor
// don't read analog pins too often (<1Hz), else caps never get to charge.
//112 to 120 = normal, 400 = high
void sensorGas()
{
    gas_sensor = analogRead(GasSmokeAnalogPin);    // read the input pin

    debugPrint("Gas = ");
    debugPrint(gas_sensor);

    //send data if gas detected, or if big changes relative to value last sent, or if it's been a while
    if ((gas_sensor < (gas_sensor_previous - 70)) || ((gas_sensor > (gas_sensor_previous + 70)) || (700000 < (millis() - gas_time_send))))
    {
        gas_time_send = millis();  //update gas_time_send with when sensor value last transmitted

        Payload msg;
        msg.nodeID = NODEID;
        msg.deviceID = SENSOR_GAS;
        msg.uptime_ms = millis();
        msg.sensordata = gas_sensor;
        msg.battery_volts = batteryVoltage();
        radio.sendWithRetry(GATEWAYID, (const void*)(&msg), sizeof(msg));

        ledFlash();

        radio.sleep();
        gas_sensor_previous = gas_sensor;
        debugPrint("gas rfm = ");
        debugPrint(gas_sensor);

    }//end if send RFM
}
#endif

#ifdef SENSOR_FLAME
//===================================================================
//device #3
//flame
void sensorFlame
{
    //analog value:  usually 1023 for no fire, lower for fire.
    flameValue = analogRead(flameAnalogInput);
    if ((flameValue < (flameValue_previous - 20)) || ((flameValue > (flameValue_previous + 20)) || (705000 < (millis() - flame_time_send))) )
    {
        flame_time_send = millis();  //update gas_time_send with when sensor value last transmitted

        Payload msg;
        msg.nodeID = NODEID;
        msg.deviceID = SENSOR_FLAME;
        msg.uptime_ms = millis();
        msg.sensordata = flameValue;
        msg.battery_volts = batteryVoltage();

        radio.sendWithRetry(GATEWAYID, (const void*)(&msg), sizeof(msg));
        radio.sleep();

        flameValue_previous = flameValue;

        debugPrint("flame detected rfm");
        debugPrint(flameValue);
        delay(2000);
    }


    //start debug code
    if (debug) {
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
}
#endif

#ifdef SENSOR_PIR
//===================================================================
//device #4
//PIR
void sensorPir()
{
    //1 mean presence detected?
    PIR_reading = digitalRead(PirInput);
    //if (PIR_reading == 1)
    //Serial.println("PIR = 1");
    //else
    //Serial.println("PIR = 0");
    //send PIR sensor value only if presence is detected and the last time
    //presence was detected is over x miniutes ago.  Avoid excessive RFM sends
    if ((PIR_reading == 1) && ( ((millis() - pir_time) > 60000) || ( (millis() - pir_time) < 0)) ) //meaning there was sound
    {
        pir_time = millis();
        Payload msg;
        msg.nodeID = NODEID;
        msg.deviceID = SENSOR_PIR;
        msg.uptime_ms = millis();
        msg.sensordata = 1111;
        msg.battery_volts = batteryVoltage();		//null value;
        radio.sendWithRetry(GATEWAYID, (const void*)(&msg), sizeof(msg));
        radio.sleep();
        debugPrintln("PIR detectedEDED RFM");
        delay(2000);
    }
}
#endif

#ifdef SENSOR_SOUND
//===================================================================
//device #5
//sound
void sensorSound()
{
    //soundValue = analogRead(soundAnalogInput);
    //Serial.print("sound analog = ");
    //Serial.print(soundValue);

    // 1 = no noise, 0 = noise!!
    sound_reading = digitalRead(soundInput);
    //Serial.print("sound value = ");
    //Serial.println(sound_reading);
    if ((sound_reading == 0) && ( ((millis() - sound_time) > 20000) || ( (millis() - sound_time) < 0)) ) //meaning there was sound
    {
        sound_time = millis();  //update gas_time_send with when sensor value last transmitted

        Payload msg;
        msg.nodeID = NODEID;
        msg.deviceID = SENSOR_SOUND;
        msg.uptime_ms = millis();
        msg.sensordata = 2222;
        msg.battery_volts = batteryVoltage();
        radio.sendWithRetry(GATEWAYID, (const void*)(&msg), sizeof(msg));
        radio.sleep();
        debugPrint("sound noise detected RFM ");
        sound_reading_previous = sound_reading;
    }
}
#endif

#ifdef SENSOR_LIGHT
//===================================================================
//device #8
//light
void sensorLight()
{
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
            Payload msg;
            msg.nodeID = NODEID;
            msg.deviceID = SENSOR_LIGHT;
            msg.uptime_ms = millis();
            msg.sensordata = lightValue;
            msg.battery_volts = batteryVoltage();
            radio.sendWithRetry(GATEWAYID, (const void*)(&msg), sizeof(msg));
            radio.sleep();
            lightValue_previous = lightValue;
            debugPrintf("light RFM =", lightValue);
        }

        debugPrintf("light analog = ", lightValue);

    }// end if millis time_passed >
}
#endif

// Blinks the led n times (freqency 1s)
void blinkNTimes(int n)
{
  for(int i = 0; i < n; i += 1)
  {
    digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);              // wait for 1/10 second
    digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
    delay(1000);             // wait for 1/10 second
  }
}


//
// Print to serial in debug mode only
//
void debugPrintln(const char* s)
{
    if (debug)
    {
        Serial.println(s);
    }
}

void debugPrintf(const char* s, double x)
{
    if (debug)
    {
        char buff[256];
        sprintf(buff, s, x);
        Serial.println(buff);
    }
}

//
// Reads and calculates current battery voltage
// Returns: Current voltage in Volts.
//
float batteryVoltage()
{
    // read the value at analog input
    int value = analogRead(BATTERY_PIN);
    float vout = (value * VOLTAGE) / 1024.0;
    float vin = vout / R1R2;
    if (vin < 0.09) {
        vin = 0.0;//statement to quash undesired reading !
    }
    return vin;
}

void setup()
{
    pinMode(LED, OUTPUT);

    // first signal: started :-)
    blinkNTimes(1);

    if (debug)
        Serial.begin(SERIAL_BAUD);          //  setup serial

    if (!radio.initialize(FREQUENCY, NODEID, NETWORKID))
    {
      while(true)
      {
        blinkNTimes(5);
      }
    }
#ifdef IS_RFM69HW
    radio.setHighPower(); //uncomment only for RFM69HW!
#endif
    radio.encrypt(ENCRYPTKEY);
    radio.setPowerLevel(31);
    radio.sleep();

    if (debug)
    {
        char buff[50];
        sprintf(buff, "\nTransmitting at %d Mhz...", FREQUENCY == RF69_433MHZ ? 433 : FREQUENCY == RF69_868MHZ ? 868 : 915);
        Serial.println(buff);
    }
    // second signal: radio initialized
    blinkNTimes(2);

    // battery monitor
    pinMode(BATTERY_PIN, INPUT);

    //temperature / humidity sensor
#ifdef SENSOR_TEMP_HUM
    dht.begin();
    debugPrintln("DHT sensor enabled.");
    // third signal: DHT initialized
    blinkNTimes(3);
    // send initial reading
    sensorTempHum();
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

    temperature_skipped_turns = 0;

    //PIR sensor
#ifdef SENSOR_PIR
    pinMode(PirInput, INPUT);
#endif
}

void loop()
{
#ifdef SENSOR_TEMP_HUM
    debugPrintf("Turns skipped: ", temperature_skipped_turns);
    if(temperature_skipped_turns >= TEMPERATURE_TURNS)
    {
        temperature_skipped_turns = 0;
        sensorTempHum();
    }
    else
    {
        temperature_skipped_turns += 1;
    }
#endif

    // Enter power down state for 8 s with ADC and BOD module disabled
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
}//end loop
