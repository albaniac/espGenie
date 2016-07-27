#include <Arduino.h>

#include <ESP8266WiFi.h>          //ESP8266 Core WiFi Library
// #include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <PubSubClient.h>
#include <DallasTemperature.h>
#include <OneWire.h>

const char* espGenieFirmwareVersion     = "espGenie Firmware Version - 1.4";

// TODO
// Params https://gist.github.com/knnniggett/7dec273b7b634e955284
//  mqtt dest, userdevicedeviceName, password, port
// Allow local toggle without MQTT connection
// wifi auto reconnect
// mqtt auto reconnect
// GPIO - reset wifi..use 1st switch at startup?  https://github.com/tzapu/WiFiManager/blob/master/examples/OnDemandConfigPortal/OnDemandConfigPortal.ino
// Look at previous range of values and only send if beyond threshold
// Minimum Send Interval
// Configure from HG and save to EEPROM.
// RSSI as percentage, see http://www.speedguide.net/faq/how-does-rssi-dbm-relate-to-signal-quality-percent-439
// MQTT Get Version
// OTA Support
// Multiple Relays & Switches
// Swap MAC Addr to DeviceID

// Testing outside of HG - TODO Move to README.md
// use moquette as a standalone mqtt broker on windows or other platforms (java based) https://github.com/andsel/moquette
// use mqtt.fx as a standalone mqtt client on windows or other platforms (java based) http://mqttfx.jfx4ee.org/
  
// Define pin mappings
#define ONE_WIRE_BUS 2
#define RELAY_PIN 5
#define SWITCH_PIN 13


// Set Default Poll Interval for checking sensor
#define POLL_INTERVAL_SECS 10

// MQTT Settings
#define mqtt_server "192.168.0.176"
#define mqtt_server_port 1883
//#define mqtt_user ""
//#define mqtt_password ""
#define base_topic "home/"
#define temperature_topic "/sensor/temp"
#define wifi_topic "/sensor/signal"

float previousTemperature[20];
float diff = 1.0;

int reconnectCount = 0;
int numberOfDevices;
String ssid;
long previousrssi;
unsigned long previousMillis = 0;

// Flash Settings
int relayPreviousState = 0;
boolean flashEnabled;
int flashCount = 15;
int flashCounter = 0;
int flashIntervalms = 750;  
unsigned long previousFlashMillis = 0;  

// Button
int current;         // Current state of the button
long millis_held;    // How long the button was held (milliseconds)
long secs_held;      // How long the button was held (seconds)
long prev_secs_held; // How long the button was held in the previous check
byte previous = HIGH;
unsigned long firstTime; // how long since the button was first pressed
int triggered = 0;

WiFiClient espClient;
PubSubClient mqttClient(espClient); // TODO Check client name..

// Setup a oneWire instance
OneWire oneWire(ONE_WIRE_BUS);

// Pass oneWire reference to DallasTemperature.
DallasTemperature sensors(&oneWire);
  

void loop() { 

  unsigned long currentMillis = millis();
  
  if (flashEnabled) {
    if (currentMillis - previousFlashMillis >= flashIntervalms) {
      previousFlashMillis = currentMillis;
      if (flashCounter < flashCount) {
        digitalWrite(RELAY_PIN, !digitalRead(RELAY_PIN));
        flashCounter++;
      }
      else
      {
        disableFlash();
        // Put back to previous state
        digitalWrite(RELAY_PIN, relayPreviousState);
      }
    }
  }
  if (currentMillis - previousMillis >= (POLL_INTERVAL_SECS  * 1000)) { // Timed Loop for sensors
    previousMillis = currentMillis;       // Save last time we ran

    sensors.requestTemperatures();
    sendSignalStrength();

    // Step through each sensor
    for(int i=0;i<numberOfDevices; i++) {
      float temperature = sensors.getTempCByIndex(i);
      if (checkBound(temperature, previousTemperature[i], diff)) {
        Serial.print("Sensor ");
        Serial.print(i);
        Serial.print(", Temp: ");
        Serial.println(temperature);

        sendTemperature(temperature,i);
        previousTemperature[i] = temperature;
      }
    }
  } // End Sensor Timed Loop
  
 
  current = digitalRead(SWITCH_PIN);

  // if the button state changes to pressed, remember the start time 
  if (current == LOW && previous == HIGH && (millis() - firstTime) > 200) { firstTime = millis(); }

  millis_held = (millis() - firstTime);
  secs_held = millis_held / 1000;

  if (millis_held > 50) { //debounce 50 ms

    if (current == LOW && secs_held >=2 && triggered == 0) {
      triggered = 1;
      // Toggle flash state
      flashEnabled = !flashEnabled;
    }

    // check if the button was released since last checked
    if (current == HIGH && previous == LOW) {

      if (secs_held <= 0) {
        // Button press
        disableFlash();
        digitalWrite(RELAY_PIN, !digitalRead(RELAY_PIN));
        sendStatus(String(digitalRead(RELAY_PIN)));
      }
              
      // button released so clear flag to stop it running multiple times
      triggered = 0;
    }
  }

  previous = current;
  prev_secs_held = secs_held;   
  
  mqttClient.loop();
  yield();
}

void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  // print the ssid that we should connect to to configure the ESP8266
  Serial.print("Created config portal with access point name: ");
  Serial.println(myWiFiManager->getConfigPortalSSID());
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // create character buffer with ending null terminator (string)
  char message_buff[100];
  int i;
  for(i=0; i<length; i++) {
    message_buff[i] = payload[i];
  }
  message_buff[i] = '\0';
  String msgString = String(message_buff);
  
  if (msgString.equals("ON")) {
    disableFlash();
    digitalWrite(RELAY_PIN, HIGH);
    sendStatus(String(digitalRead(RELAY_PIN)));
  }else if  (msgString.equals("OFF")) {
    disableFlash();
    digitalWrite(RELAY_PIN, LOW);
    sendStatus(String(digitalRead(RELAY_PIN)));
  }else if  (msgString.equals("STATUS")) {
    // TODO: Display Flash Status
    if (flashEnabled) {
      Serial.print("Status: Flashing");
      sendStatus("Flash_TurnedOn");
    }
    else
    {
      Serial.print("Status: ");
      Serial.println(digitalRead(RELAY_PIN)); 
      sendStatus(String(digitalRead(RELAY_PIN)));   
    }
    // TODO: Send Status
  }else if  (msgString.equals("TOGGLE")) {    
    disableFlash(); 
    digitalWrite(RELAY_PIN, !digitalRead(RELAY_PIN));
    sendStatus("Toggled_NOW_" + String(digitalRead(RELAY_PIN)));
  }else if  (msgString.equals("FLASH_ON")) {
    relayPreviousState = digitalRead(RELAY_PIN); 
    Serial.println ("Flash ON");
    flashEnabled = true;
    sendStatus("Flash_TurnedOn");
  }else if  (msgString.equals("FLASH_OFF")) {
    Serial.println ("Flash Off");
    disableFlash();
    digitalWrite(RELAY_PIN, relayPreviousState);
    sendStatus("Flash_TurnedOff");
  }else{
    Serial.print("Unrecognised Command: ");
    Serial.println(msgString);
  }
}

void printAddress(DeviceAddress deviceAddress) { 
   Serial.print(" address: { "); 
   for (uint8_t i = 0; i < 8; i++) 
   { 
     // zero pad the address if necessary 
     Serial.print("0x"); 
     if (deviceAddress[i] < 16) Serial.print("0"); 
     Serial.print(deviceAddress[i], HEX); 
     if (i<7) Serial.print(", ");   
   } 
   Serial.print(" }"); 
 } 

void mqttReconnect() {
    // reconnect code from PubSubClient example
}

void sendSignalStrength() {
    // if (WiFi.status() != WL_CONNECTED) { wifiConnect(); } //TODO: Do reconnect better...

    long rssi = WiFi.RSSI();
    
    // Changed, so send the signal strength.
    if (rssi != previousrssi)
    {
      String SignalTopic;
      SignalTopic = base_topic + String(ssid) + String(wifi_topic);
  
      if (! mqttClient.publish(String(SignalTopic).c_str(), String(rssi).c_str(), true)) { 
        Serial.println("Publish failed"); 
      }    

      // Store Previous Signal Strength
      previousrssi = rssi;
    }
}

void sendTemperature(float temperature,int sensorIndex) {  
  // Construct topic containing sensor number
  String Topic;
  Topic = base_topic + String(ssid) + String(temperature_topic) + sensorIndex;
  MQTTconnect();  
   
  if (! mqttClient.publish(String(Topic).c_str(), String(temperature).c_str(), true)) { Serial.println("Publish failed"); }    
}

void sendStatus(String message) {
  String StatusTopic;
  StatusTopic = base_topic + String(ssid) + "/output/status";
  if (!mqttClient.publish(String(StatusTopic).c_str(), String(message).c_str(), true)) { 
     Serial.println("Publish status message failed"); 
   }  
 }

void disableFlash() {
  flashEnabled = false;
  flashCounter = 0;
 }

void MQTTconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.println("Attempting to establish MQTT connection");
    // Attempt to connect 
    #ifndef mqtt_user
        if (mqttClient.connect(ssid.c_str())) {          
            Serial.println("MQTT Connected");
        } else {
            Serial.print("MQTT Connect Failed.");
            Serial.print("Return code= ");
            Serial.println(mqttClient.state());
            Serial.println("Trying again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    #else
        if (mqttClient.connect(ssid.c_str(), mqtt_user, mqtt_password)) {
            Serial.println("MQTT Connected");
        } else {
            Serial.print("MQTT Connect Failed.");
            Serial.print("Return code= ");
            Serial.println(mqttClient.state());
            Serial.println("Trying again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    #endif
    
    // Publish Device Startup
    String startupTopic = String(base_topic) + "automation/startup";
    mqttClient.publish(String(startupTopic).c_str(), String(ssid).c_str(), true); 
   
    String commandTopic = base_topic + String(ssid) + "/command";
    Serial.print("Subscribing to command topic ");
    Serial.println(commandTopic);
    mqttClient.subscribe(String(commandTopic).c_str());
  }
}

bool checkBound(float newValue, float prevValue, float maxDiff) {
  return !isnan(newValue) &&
         (newValue < prevValue - maxDiff || newValue > prevValue + maxDiff);
}


