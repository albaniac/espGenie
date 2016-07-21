#include <ESP8266WiFi.h>          //ESP8266 Core WiFi Library
// #include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <PubSubClient.h>
#include <DallasTemperature.h>
#include <OneWire.h>

const char* espGenieFirmwareVersion     = "espGenie Firmware Version - 1.3";

WiFiClient espClient;
PubSubClient mqttClient(espClient); // TODO Check client name..

// TODO
// Params https://gist.github.com/knnniggett/7dec273b7b634e955284
// wifi auto reconnect
// mqtt auto reconnect
// GPIO - reset wifi..use 1st switch at startup?  https://github.com/tzapu/WiFiManager/blob/master/examples/OnDemandConfigPortal/OnDemandConfigPortal.ino
// Look at previous range of values and only send if beyond threshold
// Minimum Send Interval
// Configure from HG and save to EEPROM.
// RSSI as percentage, see http://www.speedguide.net/faq/how-does-rssi-dbm-relate-to-signal-quality-percent-439
  
// Define pin mappings
#define ONE_WIRE_BUS 2
#define RELAY_PIN 6
#define SWITCH_PIN 13
#define RECEIVER_PIN 12

// Set Default Poll Interval for checking sensor
#define POLL_INTERVAL_SECS 10

// MQTT Settings
#define mqtt_server "192.168.0.161"
#define mqtt_server_port 1883
#define mqtt_user ""
#define mqtt_password ""
#define base_topic "home/"
#define temperature_topic "/sensor/temp"
#define wifi_topic "/sensor/signal"

float previousTemperature[20];
int reconnectCount = 0;
int numberOfDevices;
String clientMac = "";
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

// Setup a oneWire instance
OneWire oneWire(ONE_WIRE_BUS);

// Pass oneWire reference to DallasTemperature.
DallasTemperature sensors(&oneWire);
  
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  WiFiManager wifiManager;

  //reset settings - for testing  
  wifiManager.resetSettings();

  wifiManager.setDebugOutput(true);
  
  wifiManager.setTimeout(180);

  char versionBuffer[50];    // this needs to be large enough for the version text and tags
  strcpy(versionBuffer, "<p>");
  strcat(versionBuffer, espGenieFirmwareVersion);
  strcat(versionBuffer, "</p>");

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  WiFiManagerParameter customVersionText(versionBuffer);

  //set config save notify callback
  //wifiManager.setSaveConfigCallback(saveConfigCallback);
  
  // callback for when device enters configuration mode on failed WiFi connection attempt. 
  wifiManager.setAPCallback(configModeCallback);

  //add all your parameters here
  wifiManager.addParameter(&customVersionText);
  
  //Dynamically create the SSID from the ChipId
  String ssid = "espGenie_" + String(ESP.getChipId());

  // Loop if unable to connect to configured Wifi
  if(!wifiManager.autoConnect(ssid.c_str(),NULL)) {
    Serial.println("Failed to connect and hit timeout");
    delay(3000);
    ESP.reset();
    delay(5000);
  } 

  Serial.println("espGenie WiFi Connected.");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Set hostname - TODO, make this configurable
  WiFi.hostname(ssid);

  // Get Mac Address
  unsigned char mac[6];
  WiFi.macAddress(mac);
  clientMac += macToStr(mac); 

  Serial.print("Device MAC Address: ");
  Serial.println(clientMac);
    
  // MQTT Configuration
  mqttClient.setClient(espClient);
  mqttClient.setServer(mqtt_server, mqtt_server_port);
  mqttClient.setCallback(mqttCallback);  
  MQTTconnect();

  // Get sensor details
  sensors.begin();
  DeviceAddress tmp_address; 
  numberOfDevices = sensors.getDeviceCount(); 
  Serial.print(numberOfDevices);
  Serial.println(" temperature sensor(s) found"); 
  for(int i=0;i<numberOfDevices; i++) 
  {
   Serial.print("Sensor ");
   Serial.print(i); 
   sensors.getAddress(tmp_address, i);
   
   printAddress(tmp_address); 
   Serial.println(); 
    
   Serial.print("Setting resolution to 9 bit for Sensor ");
   Serial.println(i); 
   sensors.setResolution(tmp_address, 9); // LOWER IS FASTER, 12,10,9
  }
  
  // Configure toggle button
  pinMode(SWITCH_PIN, INPUT);
}

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
  }else{
  if (currentMillis - previousMillis >= (POLL_INTERVAL_SECS  * 1000)) {     // Timed Loop for sensors
      previousMillis = currentMillis;       // Save last time we ran

      float temperature;
      sensors.requestTemperatures();
      sendSignalStrength();

      // Step through each sensor
      for(int i=0;i<numberOfDevices; i++)
      {
        // Serial.print("Processing Sensor ");
        // Serial.println(i);
        float temperature;
        temperature = sensors.getTempCByIndex(i);
        if (temperature != previousTemperature[i])
        {
          //Serial.print("Sensor ");
          //Serial.print(i);
          //Serial.print(" Changed, Temp: ");
          //Serial.println(temperature);

          sendTemperature(temperature,i);
          previousTemperature[i] = temperature;
        }
      }
    } // End Sensor Timed Loop
  }
 
  current = digitalRead(SWITCH_PIN);

  // if the button state changes to pressed, remember the start time 
  if (current == LOW && previous == HIGH && (millis() - firstTime) > 200) { firstTime = millis(); }

  millis_held = (millis() - firstTime);
  secs_held = millis_held / 1000;

  // This if statement is a basic debouncing tool, the button must be pushed for at least
  // 50 milliseconds in a row for it to be considered as a push.
  if (millis_held > 50) {

    //if (current == LOW && secs_held > prev_secs_held) {
      // Serial.println("Each second the button is held blink the indicator led");
      // Serial.println(millis_held);
    //}

    if (current == LOW && secs_held >=2 && triggered == 0) {
      triggered = 1;
      // Toggle flash state
      flashEnabled = !flashEnabled;
    }

    // check if the button was released since we last checked
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

String macToStr(const uint8_t* mac) {
  String result;
  for (int i = 0; i < 6; ++i) {
    result += String(mac[i], 16);
  }
  return result;
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
      SignalTopic = base_topic + String(clientMac) + String(wifi_topic);
  
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
  Topic = base_topic + String(clientMac) + String(temperature_topic) + sensorIndex;
  MQTTconnect();  
   
  if (! mqttClient.publish(String(Topic).c_str(), String(temperature).c_str(), true)) { Serial.println("Publish failed"); }    
}

void sendStatus(String message) {
  String StatusTopic;
  StatusTopic = base_topic + String(clientMac) + "/output/status";
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
        if (MQTTclient.connect("clientID")) {          
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
        if (mqttClient.connect("clientID", mqtt_user, mqtt_password)) {
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
    mqttClient.publish(String(startupTopic).c_str(), String(clientMac).c_str(), true); 
   
    String commandTopic = base_topic + String(clientMac) + "/command";
    Serial.print("Subscribing to command topic ");
    Serial.println(commandTopic);
    mqttClient.subscribe(String(commandTopic).c_str());
  }

}


