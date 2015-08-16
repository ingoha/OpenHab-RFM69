//
// Configuration section 
// 1) Update encryption string "ENCRYPTKEY" and network id, node id and frequency
// 2) pick sensors to enable
//


//
// Sensors (device ids)
// uncomment to enable
//

//#define SENSOR_GAS 2
//#define SENSOR_FLAME 3
//#define SENSOR_PIR 4
//#define SENSOR_SOUND 5
//#define SENSOR_TEMP_HUM 6
#ifdef SENSOR_TEMP_HUM
  #define SENSOR_HUMIDITY 7
#endif
//#define SENSOR_LIGHT 8


//
// Radio
// TBD: This is only temporary; configuration will be stored in arduino eeprom in a future release.
//

#define NODEID        10    //unique for each node on same network
#define NETWORKID     100  //the same on all nodes that talk to each other
#define GATEWAYID     1
//Match frequency to the hardware version of the radio (uncomment one):
//#define FREQUENCY   RF69_433MHZ
#define FREQUENCY   RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "ENCRYPTKEY" //exactly the same 16 characters/bytes on all nodes!
//#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define ACK_TIME      30 // max # of ms to wait for an ack

//
// Arduino
//
#define LED           13  // Arduino pro mini has LED on D13 (https://www.arduino.cc/en/Main/ArduinoBoardProMini)
#define SERIAL_BAUD   9600  //must be 9600 for GPS, use whatever if no GPS

//temperature / humidity  =====================================
#ifdef SENSOR_TEMP_HUM

#define DHTPIN 7     			// digital pin we're connected to

// Uncomment whatever type you're using!
#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// time interval between sensor data is read and sent
#define TEMPERATURE_INTERVAL 360000

#endif


/*
 * General
 */
bool debug = 0;
 
/*
 * End configuration section
 */
