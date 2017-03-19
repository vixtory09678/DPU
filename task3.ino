#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

//needed for library
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager

#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "EEPROMControl.h"

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3F,20,4);

#define DEBUG

#define TYPE "door"

WiFiClient client;
PubSubClient mqtt(client);
#define MQTT_HOST_SERVER      "service.anto.io" //assign
#define MQTT_PORT             1883              //assign
#define MQTT_USER             "dpu"
#define MQTT_PASS             "z8qJnVSGXafIaCoiPhmN2gs250VkXZvZXKImPYTp"  //assign
#define SUB_TOPIC             "channel/dpu/DPU/DPU"

String passCode = "";

typedef struct Time
{
  unsigned long lastReconnectAttempt;
  unsigned long timeDelaySendMqtt;
};
Time time_;

typedef struct Door{
  bool state;
};
Door door_;

typedef struct Sensor
{
  int tempSensor;
  int pulseSensor;
};

void onWiFiRead(String ssid , String password){
  #ifndef DEBUG
    Serial.print(F("SSID save "));
    Serial.println(ssid);
    Serial.print(F("PASSWORD save "));
    Serial.println(password);
  #endif

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("SAVE DATA");
  lcd.setCursor(0,1);
  lcd.print(ssid);
  lcd.setCursor(0,2);
  lcd.print(password);

  EEPROMControl::getInstance()->save(SSID,(char*)ssid.c_str(),ssid.length());
  EEPROMControl::getInstance()->save(PASSWORD,(char*)password.c_str(),password.length());
}

void onParameterRead(const char* id,String value){
  Serial.print(F("Id -> "));
  Serial.println(id);
  Serial.print(F("value -> "));
  Serial.println(value);

  lcd.setCursor(0,3);
  lcd.print(value);


  passCode = value;
  EEPROMControl::getInstance()->save(CODE,(char*)passCode.c_str(),passCode.length());
  delay(1000);
  lcd.clear();
}

void onMqttDataReceive(char* topic , uint8_t *payload , unsigned int length){
  char jsonString[length];

  for(int i = 0;i < length; i++){
    jsonString[i] = (char)payload[i];
  }
  jsonString[length] = '\0';

  #ifdef DEBUG
    Serial.println(jsonString);
  #endif

  StaticJsonBuffer<500> jsonBufferInput;
  JsonObject& root = jsonBufferInput.parseObject(jsonString);

  //command
  String type = root["type"];
  String code = root["passcode"];
  String command = root["cmd"];

  //"cmd": "saveconfig","s11":[10.00,32.00],"s12":[30.00,49.00]
//  int minHeartRate = root["s11"][0]; int maxHeartRate = root["s11"][1];
//  int minTemp  = root["s12"][0]; int maxTemp = root["s12"][1];

  if(type.equals(TYPE)){
    if(code.equals(passCode)){
      if(command.equals("doorOpen")){
        door_.state = true;
      }else if(command.equals("doorClose")){
        door_.state = false;
      }
    }
  }
}

void configModeCallback (WiFiManager *myWiFiManager){
  lcd.init();
  lcd.backlight();

  lcd.setCursor(0,0);
  lcd.print("Config mode AP");
  lcd.setCursor(0,1);
  lcd.print(myWiFiManager->getConfigPortalSSID());
  lcd.setCursor(0,2);
  lcd.print("IP Address");
  lcd.setCursor(0,3);
  lcd.print(WiFi.softAPIP());
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  EEPROM.begin(512);

//  pinMode(CONFIG_PIN,INPUT);

  mqtt.setServer(MQTT_HOST_SERVER,MQTT_PORT);
  mqtt.setCallback(onMqttDataReceive);

  time_.lastReconnectAttempt = millis();
  time_.timeDelaySendMqtt = millis();
  door_.state = false;

  
  setupWiFi();

  lcd.init();
  lcd.backlight();

  lcd.setCursor(3,0);
  lcd.print("Healthcare IoT");
  lcd.setCursor(6,1);
  lcd.print("project");
  lcd.setCursor(4,3);
  lcd.print("by HONEYLab");
  delay(2000);
  lcd.clear();
}

int countLimitConnect = 0;

void loop() {
  // put your main code here, to run repeatedly:
  //check wifi connect
  if(WiFi.status() != WL_CONNECTED){
    setupWiFi();
  }

  //check mqtt connect
  if (!mqtt.connected()) {
    Serial.println(F("mqtt disconnect"));
    long now = millis();
    if (now - time_.lastReconnectAttempt > 5000) {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Server disconnect");
      lcd.setCursor(0,1);
      lcd.print("connecting..");
      time_.lastReconnectAttempt = now;
      // Attempt to reconnect
      if (reconnect()) {
        lcd.clear();
        time_.lastReconnectAttempt = 0;
      }else{
        countLimitConnect += 1;
        if(countLimitConnect == 2){
          countLimitConnect = 0;
          ESP.reset();
        }
      }
    }
  } else {
    // Client connected
    if(millis() - time_.timeDelaySendMqtt > 1000){
    //TODO
      StaticJsonBuffer<200> jsonBufferOutput;
      JsonObject& displaySensor = jsonBufferOutput.createObject();
      displaySensor["type"] = TYPE;
      displaySensor["passcode"] = passCode;

      if(door_.state){
        displaySensor["msg"] = "Open";
      }else{
        displaySensor["msg"] = "Close";
      }

//      displaySensor["cout"] = ++counter;
      String out = "";
      displaySensor.printTo(out);
     mqtt.publish(SUB_TOPIC,out.c_str());
      time_.timeDelaySendMqtt = millis();
    }
    mqtt.loop();
  }
}

void setupWiFi(){
  if(WiFi.status() != WL_CONNECTED){

    String ssid = EEPROMControl::getInstance()->read(SSID);
    String password = EEPROMControl::getInstance()->read(PASSWORD);
    String code = EEPROMControl::getInstance()->read(CODE);

    #ifdef DEBUG
      Serial.println(F("SSID is : "));
      Serial.println(ssid);
      Serial.println(ssid.length());
      Serial.println(F("PASSWORD is : "));
      Serial.println(password);
      Serial.println(password.length());
      Serial.println(F("CODE is : "));
      Serial.println(code);
      Serial.println(code.length());
    #endif

    if(code.length()>0){
      passCode = code;
    }else{
      passCode = "";
    }

    if(ssid.length() > 0){
      WiFi.begin(ssid.c_str(),password.c_str());
      if(WiFi.waitForConnectResult() == WL_CONNECTED){
        Serial.println(F("Connected.."));
        Serial.println(WiFi.localIP());

        reconnect();
      }else{
        configWiFi();
      }
    }else{
      configWiFi();
    }
  }
}

void configWiFi(){
  WiFiManager wifiManager;
  wifiManager.setConfigPortalTimeout(240);
  wifiManager.setAPCallback(configModeCallback);
  wifiManager.setParameterCallback(onParameterRead);
  wifiManager.setWiFiCallback(onWiFiRead);
  WiFiManagerParameter parameter("test","ID Code","",4);
  wifiManager.addParameter(&parameter); 

  if (!wifiManager.startConfigPortal("ConfigWiFi")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }
  lcd.clear();
  Serial.println("connected... :)");
}

boolean reconnect() {
 
  if (mqtt.connect("project3",MQTT_USER,MQTT_PASS)) {
    // lcd.clear();
    mqtt.subscribe(SUB_TOPIC);
  }
  return mqtt.connected();
}
