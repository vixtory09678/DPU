#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

//needed for library
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "EEPROMControl.h"
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <Adafruit_NeoPixel.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,20,4);

//test DEBUG
#define DEBUG
#define WIFI_ONLY
#define HARDWARE


#define TYPE "tree"

#define DHTTYPE DHT11
#define DHTPIN D4               //assign
 
#define WATER_BUTTON_PIN    A0    //assign
#define LIGHT_BUTTON_PIN    D7    //assign

#define LIGHT_PIN           D3    //assign
#define NUMPIXELS           24

#define WATER_PIN           D6    //assign

    
#define TRIGGER_PIN 0
DHT dht(DHTPIN, DHTTYPE);

Adafruit_ADS1015 ads; 

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, LIGHT_PIN, NEO_GRB + NEO_KHZ800);

WiFiClient client;
PubSubClient mqtt(client);
#define MQTT_HOST_SERVER      "service.anto.io" //assign
#define MQTT_PORT             1883              //assign
#define MQTT_USER             "dpu"
#define MQTT_PASS             "z8qJnVSGXafIaCoiPhmN2gs250VkXZvZXKImPYTp"  //assign
#define SUB_TOPIC             "channel/dpu/DPU/DPU"                       //assign

typedef struct {
  int humi;
  int light;
  int temp;
  int soil;
}Sensor;
Sensor sensor;

typedef struct
{
  int water_pump;
  int light;
}SwitchState;
SwitchState state;

typedef struct {
  int humi[2];
  int light[2];
  int temp[2];
  int soil[2];
} TrickerPoint;
TrickerPoint waterTrick;
TrickerPoint lightTrick;

String passCode = "";       //id code from user setting

void onMqttDataReceive(char* topic , uint8_t *payload , unsigned int length);
void onParameterRead(const char* id,String value);
void onWiFiRead(String ssid,String password);
void configPortal();
void readConfig();
void setupWiFi();
boolean reconnect();


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  EEPROM.begin(512);
//  EEPROMControl::getInstance()->clear();
  Serial.println("\n Starting");
    
  pinMode(WATER_PIN,OUTPUT);
  pinMode(WATER_BUTTON_PIN,INPUT);
  pinMode(LIGHT_BUTTON_PIN,INPUT);
  //set default sensor
  
  sensor.humi = 0;
  sensor.light = 0;
  sensor.temp = 0;
  sensor.soil = 0;
  
  //set default tricker
  waterTrick.humi[0] = 0;   waterTrick.humi[1] = 1;
  waterTrick.light[0] = 0;  waterTrick.light[1] = 1;
  waterTrick.temp[0] = 0;   waterTrick.temp[1] = 1;
  waterTrick.soil[0] = 0;   waterTrick.soil[1] = 1;

  lightTrick.humi[0] = 0;   lightTrick.humi[1] = 1;
  lightTrick.light[0] = 0;  lightTrick.light[1] = 1;
  lightTrick.temp[0] = 0;   lightTrick.temp[1] = 1;
  lightTrick.soil[0] = 0;   lightTrick.soil[1] = 1;

  //set default switch
  state.water_pump = 0;
  state.light = 0;
  
  
  
  
  
  mqtt.setServer(MQTT_HOST_SERVER,MQTT_PORT);
  mqtt.setCallback(onMqttDataReceive);
  
  setupWiFi();
  pixels.begin();

  lcd.init();
  lcd.backlight();

  dht.begin();
  ads.begin();

  lcd.setCursor(3,0);
  lcd.print("Agriculture IoT");
  lcd.setCursor(6,1);
  lcd.print("project");
  lcd.setCursor(4,3);
  lcd.print("by HONEYLab");
  delay(2000);
  lcd.clear();
}

unsigned long timeSent = 0;
unsigned long timeDelay = 0;
bool manualTrick = false;
bool stateTrickWaterPump = false;
bool stateTrickLight = false;

unsigned long counter = 0;
unsigned long timeConfig = 0;
unsigned long timeDHT = 0;
unsigned long timeSensor = 0;

long lastReconnectAttempt = 0;


uint8_t countLimitConnect = 0;

void loop() {

  //------------change to config mode ---------
  if ( digitalRead(WATER_BUTTON_PIN) == LOW && analogRead(LIGHT_BUTTON_PIN) < 500) {
    if(millis() - timeConfig > 3000){
      configPortal(); 
    }
  }else{
    timeConfig = millis();
  }

  if(millis() - timeDHT > 2000){
    if(!isnan(dht.readHumidity()) && !isnan(dht.readTemperature())){
      sensor.humi = dht.readHumidity();
      sensor.temp = dht.readTemperature();
    }
    timeDHT = millis();
  }

  if(millis() - timeSensor >1000){
     //------------read sensor--------------------
      sensor.light = map(ads.readADC_SingleEnded(2),0,2000,100,0);  //A2 I2C pin
      // sensor.light = ads.readADC_SingleEnded(2);
      sensor.soil = map(ads.readADC_SingleEnded(3),0,2000,100,0); //A3 I2C pin
      
      lcd.setCursor(0,0);
      lcd.print("TEMP : ");
      lcd.setCursor(7,0);
      lcd.print(sensor.temp);

      lcd.setCursor(11,0);
      lcd.print("PASS CODE");

      lcd.setCursor(0,1);
      lcd.print("HUMI : ");
      lcd.setCursor(7,1);
      lcd.print(sensor.humi);

      lcd.setCursor(13,1);
      lcd.print(passCode);

      lcd.setCursor(0,2);
      lcd.print("SOIL : ");
      lcd.setCursor(7,2);
      lcd.print(sensor.soil);

      lcd.setCursor(0,3);
      lcd.print("LIGHT : ");
      lcd.setCursor(8,3);
      lcd.print(sensor.light);
      timeSensor = millis();
  }

  if(WiFi.status() != WL_CONNECTED){
    setupWiFi();
  }

  if (!mqtt.connected()) {
    Serial.println(F("mqtt disconnect"));
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Server disconnect");
      lcd.setCursor(0,1);
      lcd.print("connecting..");
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }else{
        countLimitConnect += 1;
        if(countLimitConnect == 2){
          ESP.reset();
        }
      }
    }
  } else {
    // Client connected
    if(millis() - timeSent > 1000){
      StaticJsonBuffer<200> jsonBufferOutput;
      JsonObject& displaySensor = jsonBufferOutput.createObject();
      displaySensor["type"] = TYPE;
      displaySensor["passcode"] = passCode;
      displaySensor["g1"] = sensor.soil;
      displaySensor["g2"] = sensor.temp;
      displaySensor["g3"] = sensor.humi;
      displaySensor["g4"] = sensor.light;
      displaySensor["sw1"] = state.water_pump;
      displaySensor["sw2"] = state.light;
      displaySensor["cout"] = ++counter;
      String out = "";
      displaySensor.printTo(out);
      mqtt.publish(SUB_TOPIC,out.c_str());
      timeSent = millis();
    }
    mqtt.loop();
  }

  if(manualTrick){
    if(millis()-timeDelay > 3000){
      manualTrick = false;
    }
  }else{
    //-------------- water trick ------------------
    if(sensor.light > waterTrick.light[0] && sensor.light < waterTrick.light[1]
      &&sensor.humi > waterTrick.humi[0] && sensor.humi < waterTrick.humi[1]
      &&sensor.temp > waterTrick.temp[0] && sensor.temp < waterTrick.temp[1]
      &&sensor.soil > waterTrick.soil[0] && sensor.soil < waterTrick.soil[1]){
      state.water_pump = 1;
     // do something for delay
    }else{
      state.water_pump = 0;
    }

   //--------------- light trick ------------------
    if(sensor.light > lightTrick.light[0] && sensor.light < lightTrick.light[1]
      &&sensor.humi > lightTrick.humi[0] && sensor.humi < lightTrick.humi[1]
      &&sensor.temp > lightTrick.temp[0] && sensor.temp < lightTrick.temp[1]
      &&sensor.soil > lightTrick.soil[0] && sensor.soil < lightTrick.soil[1]){
      state.light = 1;
    }else{
      state.light = 0;
    }
  }

  if(state.water_pump){
    digitalWrite(WATER_PIN,HIGH);
  }else{
    digitalWrite(WATER_PIN,LOW);
  }

  if(state.light){
    for(int i=0;i<NUMPIXELS;i++){
      pixels.setPixelColor(i, pixels.Color(255,0,255));
      pixels.show();
    }
  }else{
    for(int i=0;i<NUMPIXELS;i++){
      pixels.setPixelColor(i, pixels.Color(0,0,0));
      pixels.show();
    }
  }

  
  //manual trick
  if(analogRead(WATER_BUTTON_PIN) < 500){    // PULL UP
    if(!stateTrickWaterPump){
      stateTrickWaterPump = true;
    }
  }else{
    if(stateTrickWaterPump){
      manualTrick = true;
      state.water_pump = !state.water_pump;
      timeDelay = millis();
      stateTrickWaterPump = false;
    }
  }

  if(digitalRead(LIGHT_BUTTON_PIN) == LOW){    // PULL UP
    if(!stateTrickLight){
      stateTrickLight = true;
    }
  }else{
    if(stateTrickLight){
      manualTrick = true;
      state.light = !state.light;
      timeDelay = millis();
      stateTrickLight = false;
    }
  }
}

void configModeCallback (WiFiManager *myWiFiManager) {
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

void setupWiFi(){
  pinMode(TRIGGER_PIN, INPUT);
  
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
      }else{
        configPortal();
      }
    }else{
      configPortal();
    }
  }
}

void configPortal(){
  WiFiManager wifiManager;
  wifiManager.setConfigPortalTimeout(120);
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
 
  if (mqtt.connect("project1",MQTT_USER,MQTT_PASS)) {
    lcd.clear();
    mqtt.subscribe(SUB_TOPIC);
  }
  return mqtt.connected();
}


//callback
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

  //set max min sensor
  int minWater1 = root["s11"][0]; int maxWater1 = root["s11"][1];
  int minWater2 = root["s12"][0]; int maxWater2 = root["s12"][1];
  int minWater3 = root["s13"][0]; int maxWater3 = root["s13"][1];
  int minWater4 = root["s14"][0]; int maxWater4 = root["s14"][1];

  int minLight1 = root["s21"][0]; int maxLight1 = root["s21"][1];
  int minLight2 = root["s22"][0]; int maxLight2 = root["s22"][1];
  int minLight3 = root["s23"][0]; int maxLight3 = root["s23"][1];
  int minLight4 = root["s24"][0]; int maxLight4 = root["s24"][1];

  //manual
  int waterSwitch = root["sw1"];
  int lightSwitch = root["sw2"];

  if(type.equals(TYPE)){
    if(code.equals(passCode)){
      if(command.equals("saveconfig")){
        waterTrick.soil[0] = minWater1; waterTrick.soil[1] = maxWater1;
        waterTrick.temp[0] = minWater2; waterTrick.temp[1] = maxWater2;
        waterTrick.humi[0] = minWater3; waterTrick.humi[1] = maxWater3;
        waterTrick.light[0] = minWater4; waterTrick.light[1] = maxWater4;

        lightTrick.soil[0] = minLight1; lightTrick.soil[1] = maxLight1;
        lightTrick.temp[0] = minLight2; lightTrick.temp[1] = maxLight2;
        lightTrick.humi[0] = minLight3; lightTrick.humi[1] = maxLight3;
        lightTrick.light[0] = minLight4; lightTrick.light[1] = maxLight4;

      }else if(command.equals("cmd")){
        //do something
        state.water_pump = waterSwitch;
        state.light = lightSwitch;
        manualTrick = true;
        timeDelay = millis();

      }else if(command.equals("readconfig")){
        //do something
        readConfig();
      }
    }
  }
}

void readConfig(){
  StaticJsonBuffer<500> jsonBufferOutput;
  JsonObject& jsonReadConfig = jsonBufferOutput.createObject();

  jsonReadConfig["type"] = TYPE;
  jsonReadConfig["passcode"] = passCode;

  //------------------ create waterTrick json-----------------------

  JsonArray& s11 = jsonReadConfig.createNestedArray("s11");
  s11.add(waterTrick.soil[0]);
  s11.add(waterTrick.soil[1]);

  JsonArray& s12 = jsonReadConfig.createNestedArray("s12");
  s12.add(waterTrick.temp[0]);
  s12.add(waterTrick.temp[1]);

  JsonArray& s13 = jsonReadConfig.createNestedArray("s13");
  s13.add(waterTrick.humi[0]);
  s13.add(waterTrick.humi[1]);

  JsonArray& s14 = jsonReadConfig.createNestedArray("s14");
  s14.add(waterTrick.light[0]);
  s14.add(waterTrick.light[1]);

  //-------------------- create lightTrick json---------------------

  JsonArray& s21 = jsonReadConfig.createNestedArray("s21");
  s21.add(lightTrick.soil[0]);
  s21.add(lightTrick.soil[1]);

  JsonArray& s22 = jsonReadConfig.createNestedArray("s22");
  s22.add(lightTrick.temp[0]);
  s22.add(lightTrick.temp[1]);

  JsonArray& s23 = jsonReadConfig.createNestedArray("s23");
  s23.add(lightTrick.humi[0]);
  s23.add(lightTrick.humi[1]);

  JsonArray& s24 = jsonReadConfig.createNestedArray("s24");
  s24.add(lightTrick.light[0]);
  s24.add(lightTrick.light[1]);

  #ifdef DEBUG
//    jsonReadConfig.printTo(Serial);
  #endif
  String out = "";
  jsonReadConfig.printTo(out);
  mqtt.publish(SUB_TOPIC,out.c_str());

}

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


