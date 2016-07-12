 /*
 External libraries:
 - https://github.com/milesburton/Arduino-temp-Control-Library
 - http://pubsubclient.knolleary.net/ - pubsubclient
 
 - Links:
 - https://home-assistant.io/blog/2015/10/11/measure-temperature-with-esp8266-and-report-to-mqtt/
 - http://www.hobbytronics.co.uk/ds18b20-arduino
 - https://datasheets.maximintegrated.com/en/ds/DS18B20.pdf
 - http://blog.hekkers.net/2012/09/18/mqtt-about-dumb-sensors-topics-and-clean-code/
 - http://playground.arduino.cc/Code/HoldButton Button  

  I’ve got a table which contains a list of all the devices with their ID, description, where it is located (Building, Floor, Room, Location) and lots of other attributes.

 // TODO: Look at previous range of values and only send if beyond threshold
 // TODO: Minimum Send Interval
 // TODO: Configure from HG and save to EPROM.
 */
 
#include <ESP8266WiFi.h>
#include <OneWire.h>
#include <PubSubClient.h>
#include <RCSwitch.h>
#include <DallasTemperature.h>

// Wifi Parameters
#define AP_SSID "dw-2g"
#define AP_PASSWORD ""

// Set Default Poll Interval for checking sensor
#define POLL_INTERVAL_SECS 10

// MQTT Settings
#define mqtt_server "192.168.0.161"
#define mqtt_server_port 1883
// #define mqtt_user "your_username" // Uncomment to use user / password based Auth
// #define mqtt_password "your_password" // Uncomment to use user / password based Auth

#define base_topic "home/"
#define temperature_topic "/sensor/temp"
#define wifi_topic "/sensor/signal"

// define pin mappings
#define ONE_WIRE_BUS 2
//#define RELAY_PIN 16
#define RELAY_PIN 6
#define SWITCH_PIN 13
#define RECEIVER_PIN 12
// Setup a oneWire instance
OneWire oneWire(ONE_WIRE_BUS);

// Pass oneWire reference to DallasTemperature.
DallasTemperature sensors(&oneWire);

WiFiClient espClient;
PubSubClient MQTTclient(espClient);

// RC Switch
RCSwitch mySwitch = RCSwitch();


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

void setup() {
  Serial.begin(9600);
  pinMode(RELAY_PIN, OUTPUT); 
  pinMode(RELAY_PIN, HIGH);
  
  wifiConnect();

  // Get Mac Address
  unsigned char mac[6];
  WiFi.macAddress(mac);
  clientMac += macToStr(mac); 

  Serial.print("Device MAC Address: ");
  Serial.println(clientMac);

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

  // MQTT Configuration
  MQTTclient.setClient(espClient);
  MQTTclient.setServer(mqtt_server, mqtt_server_port);
  MQTTclient.setCallback(MQTTcallback);  
  MQTTconnect();
  
  // Configure toggle button
  pinMode(SWITCH_PIN, INPUT);

  // RC Switch
  mySwitch.enableReceive(RECEIVER_PIN);  // Receiver on pin #12
  
}

void MQTTcallback(char* topic, byte* payload, unsigned int length) {
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

 void disableFlash() {
  flashEnabled = false;
  flashCounter = 0;
 }

 void sendStatus(String message) {
  // TODO: Refine!

  String StatusTopic;
  StatusTopic = base_topic + String(clientMac) + "/output/status";
  
  if (!MQTTclient.publish(String(StatusTopic).c_str(), String(message).c_str(), true)) { 
     Serial.println("Publish status message failed"); 
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

String macToStr(const uint8_t* mac) {
  String result;
  for (int i = 0; i < 6; ++i) {
    result += String(mac[i], 16);
  }
  return result;
}

void sendTemperature(float temperature,int sensorIndex) {  

  // Construct topic containing sensor number
  String Topic;
  Topic = base_topic + String(clientMac) + String(temperature_topic) + sensorIndex;
  MQTTconnect();  
   
  if (! MQTTclient.publish(String(Topic).c_str(), String(temperature).c_str(), true)) { 
   Serial.println("Publish failed"); 
  }    
}

void sendSignalStrength() {
    if (WiFi.status() != WL_CONNECTED)
        wifiConnect();

    long rssi = WiFi.RSSI();
    
    // Changed, so send the signal strength.
    if (rssi != previousrssi)
    {
      String SignalTopic;
      SignalTopic = base_topic + String(clientMac) + String(wifi_topic);
  
      if (! MQTTclient.publish(String(SignalTopic).c_str(), String(rssi).c_str(), true)) { 
        Serial.println("Publish failed"); 
      }    

      // Store Previous Signal Strength
      previousrssi = rssi;
    }
}

void loop() {
  yield() ; // Allow background tasks to run, such as WiFi
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
  if (current == LOW && previous == HIGH && (millis() - firstTime) > 200) {
    firstTime = millis();
  }

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

  //RC SWITCH
  if (mySwitch.available()) {
    //Serial.println(mySwitch.getReceivedValue(), BIN); // Binary value
    // Serial.println(mySwitch.getReceivedValue()); // Decimal

    if (mySwitch.getReceivedValue() == 1144067) {
      disableFlash();
      digitalWrite(RELAY_PIN, HIGH);
      sendStatus("On");     
    }else if (mySwitch.getReceivedValue() == 1144076) {
      disableFlash();
      digitalWrite(RELAY_PIN, LOW);
      sendStatus("Off");  
    }
    mySwitch.resetAvailable();
  }
  //END RC SWITCH
 
  // Run this outside of the 'timed' loops
  MQTTclient.loop();  
}

void wifiConnect() {
    Serial.println();
    Serial.print("Attempting to establish WiFi connection with: ");
    Serial.print(AP_SSID);
    WiFi.begin(AP_SSID, AP_PASSWORD);
    
    int retryCount = 0;
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
      if (retryCount == 100)
        {
          Serial.println("Unable to connect to WiFi");
          return;
        }
        retryCount++;
    }
  
    Serial.println("WiFi Connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

void MQTTconnect() {
  // Loop until we're reconnected
  while (!MQTTclient.connected()) {
    Serial.println("Attempting to establish MQTT connection");
    // Attempt to connect 
    #ifndef mqtt_user
        if (MQTTclient.connect("clientID")) {          
            Serial.println("MQTT Connected");
        } else {
            Serial.print("MQTT Connect Failed.");
            Serial.print("Return code= ");
            Serial.println(MQTTclient.state());
            Serial.println("Trying again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    #else
        if (MQTTclient.connect("clientID", mqtt_user, mqtt_password)) {
            Serial.println("MQTT Connected");
        } else {
            Serial.print("MQTT Connect Failed.");
            Serial.print("Return code= ");
            Serial.println(MQTTclient.state());
            Serial.println("Trying again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    #endif
    
    // Publish Device Startup
    String startupTopic = String(base_topic) + "automation/startup";
    MQTTclient.publish(String(startupTopic).c_str(), String(clientMac).c_str(), true); 
   
    String commandTopic = base_topic + String(clientMac) + "/command";
    Serial.print("Subscribing to command topic ");
    Serial.println(commandTopic);
    MQTTclient.subscribe(String(commandTopic).c_str());
  }

}


