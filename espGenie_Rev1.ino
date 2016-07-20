
#include <ESP8266WiFi.h>          //ESP8266 Core WiFi Library
// #include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <PubSubClient.h>

const char* espGenieFirmwareVersion     = "espGenie Firmware Version - 1.3";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// TODO
// Params https://gist.github.com/knnniggett/7dec273b7b634e955284
// wifi reconnect
// GPIO - reset wifi..use 1st switch at startup?  https://github.com/tzapu/WiFiManager/blob/master/examples/OnDemandConfigPortal/OnDemandConfigPortal.ino


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
    
  mqttClient.setServer("192.168.0.1", 1883);
  mqttClient.setCallback(mqttCallback);

 
  
}

void loop() {
  if (!mqttClient.connected()) {
    mqttReconnect();
  }
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
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void mqttReconnect() {
    // reconnect code from PubSubClient example
}




