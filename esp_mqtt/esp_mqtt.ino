/***************************************************
  Adafruit MQTT Library ESP8266 Example

  Must use ESP8266 Arduino from:
    https://github.com/esp8266/Arduino

  Works great with Adafruit's Huzzah ESP board & Feather
  ----> https://www.adafruit.com/product/2471
  ----> https://www.adafruit.com/products/2821

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Tony DiCola for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <time.h>


/************************* WiFi Access Point *********************************/

#define WLAN_SSID       "S20_FE9EC7"
#define WLAN_PASS       "bbzh9747"

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "ian_d"
#define AIO_KEY         "aio_rSlK12C3nPmEGO2ZITENGIIQPlFP"

/************ Global State (you don't need to change this!) ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiClientSecure for SSL
//WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/

// Setup a feed called 'pressure' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish pressureMQTT = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pressure");

// Setup a feed called 'terraintypebutton' for subscribing to changes.
Adafruit_MQTT_Subscribe terraintypebutton = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/terrain-type");

/*************************** Sketch Code ************************************/

// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void MQTT_connect();


/*************************** Can Include ************************************/

#include <mcp_can.h>
#define CAN0_INT 4
MCP_CAN CAN0(15);

long unsigned int rxId;
uint8_t len = 0;
uint8_t rxBuf[8];
float wheelPressure = 0;

int terrainType=0;


typedef struct {
  uint8_t priority;
  uint16_t pdu;
  uint8_t sourceAddr;
} SaeID;
double setpoint = 0;

uint32_t generateSaeId(const SaeID *sae_id) {
  uint32_t id = (0b111 & sae_id->priority) << 26;
  id |= sae_id->pdu << 8;
  id |= sae_id->sourceAddr;
  return id;
}

void parseSaeId(uint32_t id, SaeID *sae_id) {
  sae_id->priority = (id >> 26) & 0b111;
  sae_id->pdu = (id >> 8) & 0xffff;
  sae_id->sourceAddr = id & 0xff;
}


void setup() {
  Serial.begin(115200);
  delay(10);

  Serial.println(F("Adafruit MQTT demo"));

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());

  // Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&terraintypebutton);

  CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ);
  CAN0.setMode(MCP_NORMAL);
  pinMode(CAN0_INT, INPUT);
  
}

uint32_t x=0;

void loop() {
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();

  // this is our 'wait for incoming subscription packets' busy subloop
  // try to spend your time here

  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &terraintypebutton) {
      Serial.print(F("Terrain Type Selected: "));
      Serial.println((char *)terraintypebutton.lastread);
      terrainType = *terraintypebutton.lastread;
    }
  }

  if(!digitalRead(CAN0_INT)){
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    SaeID id;
    parseSaeId(rxId, &id);
    Serial.println(id.pdu);
    if(id.pdu==0x00FEEE){
      wheelPressure = (float) (rxBuf[0] / 255.0)*100;
      Serial.print(F("\nSending pressure data "));
      Serial.print(x);
      Serial.print("...");
 
      if (! pressureMQTT.publish(wheelPressure)) {
        Serial.println(F("Failed"));
      } else {
        Serial.println(F("OK!"));
      }  
    }
    delay(1);   
  }

  
  // Now we can publish stuff!
  

  // ping the server to keep the mqtt connection alive
  // NOT required if you are publishing once every KEEPALIVE seconds
  /*
  if(! mqtt.ping()) {
    mqtt.disconnect();
  }
  */
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}
