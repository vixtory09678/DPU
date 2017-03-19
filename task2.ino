#include <Adafruit_MLX90614.h>
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

#define PULSE_SENSOR_PIN A0
#define CONFIG_PIN D5

#define TYPE "heartRate"

#define BUZZER_PIN D3

#define MSG_ALARM "หัวใจช้านนเจ็บเหลือเกิน"
#define MSG_TEMP_ALARM "อุณหภูมิสูง"

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

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
  unsigned long timeDelayReadPulseSensor;
  unsigned long timeDelayReadTempSensor;
  unsigned long timeDelaySendMqtt;
  unsigned long timeDelayConfig;
  unsigned long lastReconnectAttempt;
  unsigned long timeToSendAlert;
  unsigned long timeToSendAlertTemp;
  unsigned long toneAlarm;
  unsigned long toneAlarmHealth;
};

typedef struct Sensor
{
  int tempSensor;
  int pulseSensor;
};

typedef struct PulseSensorValue
{
  bool beatCheck;
  unsigned long timeOutHeartrate;
  unsigned long timeBeat;
  bool beat;
  int useTtime;
  bool oneTime;
  int bpm;
};

typedef struct Tricker
{
  int heartrate[2];
  int temp[2];
};

PulseSensorValue pulseSensor_;
Tricker trick_;
Sensor sensor_;
Time time_;

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
  #ifdef DEBUG
    Serial.print(F("Id -> "));
    Serial.println(id);
    Serial.print(F("value -> "));
    Serial.println(value);
  #endif

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
  int minHeartRate = root["s11"][0]; int maxHeartRate = root["s11"][1];
  int minTemp  = root["s12"][0]; int maxTemp = root["s12"][1];

  if(type.equals(TYPE)){
    if(code.equals(passCode)){
      if(command.equals("saveconfig")){
        trick_.temp[0] = minTemp;             trick_.temp[1] = maxTemp;
        trick_.heartrate[0] = minHeartRate;   trick_.heartrate[1] = maxHeartRate;

      }else if(command.equals("readconfig")){
        //do something
        readConfig();
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

//  Serial.println("Adafruit MLX90614 test");  

  pinMode(CONFIG_PIN,INPUT);

  mqtt.setServer(MQTT_HOST_SERVER,MQTT_PORT);
  mqtt.setCallback(onMqttDataReceive);

  setupWiFi();

  mlx.begin();  
  time_.timeDelayReadTempSensor = millis();
  time_.timeDelayReadPulseSensor = millis();
  time_.timeDelaySendMqtt = millis();
  time_.timeDelayConfig = millis();
  time_.lastReconnectAttempt = millis();
  time_.timeToSendAlert = millis();
  time_.timeToSendAlertTemp = millis();
  time_.toneAlarm = millis();
  time_.toneAlarmHealth = millis();

  sensor_.pulseSensor = 0;
  sensor_.tempSensor = 0;

  trick_.heartrate[0] = 0; trick_.heartrate[1] = 0;
  trick_.temp[0] = 0; trick_.temp[1] = 1;

  pulseSensor_.beat = false;
  pulseSensor_.timeOutHeartrate = millis();
  pulseSensor_.timeBeat = millis();
  pulseSensor_.beatCheck = false;
  pulseSensor_.oneTime = false;
  pulseSensor_.bpm = 0;

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

uint8_t countLimitConnect = 0;

void loop() {

  if(digitalRead(CONFIG_PIN) == LOW){
    if(millis() - time_.timeDelayConfig > 2000){
      //TO DO
      configWiFi();
    }
  }else{
    time_.timeDelayConfig = millis();
  }

  if(millis() - time_.timeDelayReadPulseSensor > 20){
    //TO DO
    sensor_.pulseSensor = analogRead(PULSE_SENSOR_PIN);
    #ifdef DEBUG_SENSOR
      Serial.print(F("SENSOR IS "));
      Serial.println(sensor_.pulseSensor);
    #endif
    if(sensor_.pulseSensor > 1000){
      pulseSensor_.timeOutHeartrate = millis();
      pulseSensor_.beatCheck = true;
      pulseSensor_.beat = true;
    }else{
      pulseSensor_.beat = false;
      pulseSensor_.oneTime = false;
    }
    time_.timeDelayReadPulseSensor = millis();
  }

  if(pulseSensor_.beatCheck == true){
    if(millis() - pulseSensor_.timeOutHeartrate < 1500){
      if(pulseSensor_.beat == true){
        if(!pulseSensor_.oneTime){
          pulseSensor_.useTtime = millis() - pulseSensor_.timeBeat;
          #ifdef DEBUG
            Serial.print(" yeahhhhhhhh ");
            Serial.println(pulseSensor_.useTtime);
          #endif
          pulseSensor_.bpm = (int)(60.0/(pulseSensor_.useTtime / 1000.0));
          pulseSensor_.timeBeat = millis();
          pulseSensor_.oneTime = true;
        }
      }
    }else{
      Serial.print(F("out\n"));
      pulseSensor_.timeBeat = millis();
      pulseSensor_.beatCheck = false;
    }
  }

  if((pulseSensor_.bpm > trick_.heartrate[0] && pulseSensor_.bpm < trick_.heartrate[1])){
    //TO DO
    //Condition
    //TO DO TONE
    if(millis() - time_.toneAlarmHealth > 5000){
      tone(BUZZER_PIN,830,2);
      time_.toneAlarmHealth = millis();
    }else{
      noTone(BUZZER_PIN);
    }
    
    if(millis() - time_.timeToSendAlert > 10000){
      StaticJsonBuffer<200> jsonBufferOutput;
      JsonObject& jsonReadConfig = jsonBufferOutput.createObject();

      jsonReadConfig["type"] = TYPE;
      jsonReadConfig["passcode"] = passCode;

      jsonReadConfig["msg"] = MSG_ALARM;

      String out = "";
      jsonReadConfig.printTo(out);
      mqtt.publish(SUB_TOPIC,out.c_str());

      time_.timeToSendAlert = millis();
    }
  }

  if((sensor_.tempSensor > trick_.temp[0] && sensor_.tempSensor < trick_.temp[1])){

    //TO DO TONE
    if(millis() - time_.toneAlarm > 5000){
      tone(BUZZER_PIN,830,2);
      time_.toneAlarm = millis();
    }else{
      noTone(BUZZER_PIN);
    }
    
    if(millis() - time_.timeToSendAlertTemp > 10000){
      StaticJsonBuffer<200> jsonBufferOutput;
      JsonObject& jsonSend = jsonBufferOutput.createObject();
      
      jsonSend["type"] = TYPE;
      jsonSend["passcode"] = passCode;
  
      jsonSend["msg"] = MSG_TEMP_ALARM;
      String out = "";
      jsonSend.printTo(out);
      mqtt.publish(SUB_TOPIC,out.c_str());
      time_.timeToSendAlertTemp = millis();
    } 
  }


  if(millis() - time_.timeDelayReadTempSensor > 500){
    //TODO
    sensor_.tempSensor = mlx.readObjectTempC();
    time_.timeDelayReadTempSensor = millis();

    lcd.setCursor(0,2);
    lcd.print("BPM : ");
    lcd.setCursor(7,2);
    String bpmLCD = String(pulseSensor_.bpm)+"   ";
    lcd.print(bpmLCD.c_str());

    lcd.setCursor(11,0);
    lcd.print("PASS CODE");

    lcd.setCursor(0,3);
    lcd.print("TEMP : ");
    lcd.setCursor(7,3);
    String tempPrint = String(sensor_.tempSensor)+"   ";
    lcd.print(tempPrint);
    

    lcd.setCursor(13,1);
    lcd.print(passCode);

  }

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

      displaySensor["g1"] = pulseSensor_.bpm;
      displaySensor["g2"] = sensor_.tempSensor;

      // displaySensor["cout"] = ++counter;
      String out = "";
      displaySensor.printTo(out);
     mqtt.publish(SUB_TOPIC,out.c_str());
      time_.timeDelaySendMqtt = millis();
    }
    mqtt.loop();
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
 
  if (mqtt.connect("project2",MQTT_USER,MQTT_PASS)) {
    // lcd.clear();
    mqtt.subscribe(SUB_TOPIC);
  }
  return mqtt.connected();
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

void readConfig(){
 StaticJsonBuffer<500> jsonBufferOutput;
 JsonObject& jsonReadConfig = jsonBufferOutput.createObject();
//
 jsonReadConfig["type"] = TYPE;
 jsonReadConfig["passcode"] = passCode;
//
//  //------------------ create waterTrick json-----------------------
//
 JsonArray& s11 = jsonReadConfig.createNestedArray("s11");
 s11.add(trick_.heartrate[0]);
 s11.add(trick_.heartrate[1]);

 JsonArray& s12 = jsonReadConfig.createNestedArray("s12");
 s12.add(trick_.temp[0]);
 s12.add(trick_.temp[1]);

 String out = "";
 jsonReadConfig.printTo(out);
 mqtt.publish(SUB_TOPIC,out.c_str());
}






