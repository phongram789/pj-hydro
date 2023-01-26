#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <BlynkSimpleEsp32.h>
#include <TimeLib.h>
#include <EEPROM.h>

#include <Wire.h>
#include "DS3231.h"
#include <LiquidCrystal_I2C.h>

#include "tds.h"
#include "DHT.h"
#include "pin.h"
#include "SerialCommand.h"
#include "Input_pullup.h"

LiquidCrystal_I2C lcd(0x27, 20, 4);
RTClib RTC;

WiFiClient espClient;
PubSubClient mqtt(espClient);
BlynkTimer timer;
#define BLYNK_PRINT Serial

#define DHTTYPE DHT22
TDS tds(ECPIN);
DHT dht(DHTPIN, DHTTYPE);

#define EEPROM_SIZE 64
const char* ssid = "ooy";
const char* password = "0863447295";
const char* mqtt_server = "broker.netpie.io";
const int mqtt_port = 1883;
const char* mqtt_client = "aa47cd89-19f0-4db3-a3df-b823fb50b939";
const char* mqtt_username = "iWrjeyzumGdQGZM8pSEobjA3cPCUpciE";
const char* mqtt_password = "n6htDnjn7rLq8_epUM)N-M076iMWy4t4";
#define BLYNK_TEMPLATE_ID "TMPLLaOYk4zr"
#define BLYNK_DEVICE_NAME "Smart hydroponic for urban"
#define BLYNK_AUTH_TOKEN "1aE2xcwuAkfH4FYhRq36xLlexDVPPvu4"
char auth[] = BLYNK_AUTH_TOKEN;

const char* subscribe_topic = "@msg/temp";

unsigned long currentMillis = 0;

float t = 30.2,h = 100;


//-------------pH-----------------
float voltagePH,phValue;
float acidVoltage = 1810;
float neutralVoltage = 1370;
const int smoothFactor = 10;
float lastPH = 0;

//--------------------------------
float ecValue,TdsValue,lastEC;
//float ecValue = 1.5 ,lastPH = 7.5;



int nowHour,nowMinute,nowSecond;       //RTC HH:MM:SS


int startHour,startMinute,startSecond; //ON HH:MM:SS 
int stopHour,stoptMinute,stopSecond;   //OFF HH:MM:SS
bool growLigh1,growLigh2,growLigh3,growLigh4;



float phLow,phHigh,ecLow,ecHigh; //set up from user with 

bool statusBlynk,statusMqtt,statusWifi;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RX2, TX2);
  //closeRTU();
  dht.begin();
  initEEPROM();
  lcd.init();
  lcd.backlight();
  Wire.begin();//เริ่มการสื่อสารแบบ I2C
  initWiFi();
  mqtt.setServer(mqtt_server, mqtt_port);
  mqtt.setCallback(callback);
  Blynk.config(auth,"blynk.cloud", 8080);
  timer.setInterval(2000L, sendSensor);
  delay(500);
}

void loop() {
  // put your main code here, to run repeatedly:
  currentMillis = millis();
  RTCfunction();
  Mqttreconnect();
  timer.run();
  reconnectBlynk();
  SerialstatusConnecting();
  functionLcd();
  EEPROMfunction();
  readDHT();
  PH();
  Ec();

}

void sendSensor(){
  float h = random(0,30);
  float t = random(0,100);
  Blynk.virtualWrite(V3, h);
  Blynk.virtualWrite(V4, t);
}

void Mqttreconnect(){
  static unsigned long timepoint = 0;
  if(currentMillis - timepoint >= 5000U){
    timepoint = currentMillis;
    if (mqtt.connected() == false) {
      statusMqtt = mqtt.connected();
      Serial.print("MQTT connection... ");
      if (mqtt.connect(mqtt_client, mqtt_username, mqtt_password)){
        mqtt.subscribe(subscribe_topic);// DONT FORGET
        Serial.println("connected");
        statusMqtt = mqtt.connected();
      } 
      else{
        statusMqtt = mqtt.connected();
        Serial.println("failed");
        delay(1000); //fix to millis 
      }
    }
    else {
      mqtt.loop();
      String dataJS = "{\"temp\":" + String(t) + ",\"hum\":" +String(h) + ",\"ec\":" +String(ecValue) + ",\"ph\":" +String(lastPH) + "}";
      char json[100];
      dataJS.toCharArray(json,dataJS.length()+1);
      mqtt.publish("@msg/v1/devices/me/telemetry", json);
    }
  }
};

void initWiFi() { 
  WiFi.begin(ssid, password);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Connecting to WiFi..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  lcd.clear();
  lcd.setCursor(3,0);
  lcd.print("WiFi Connected");
  //lcd.setCursor(0,1);
  //lcd.print(String(WiFi.localIP()));
  Serial.println(WiFi.localIP());
  statusWifi = WiFi.status();
  delay(2000);

};
void initEEPROM() { 
  if (!EEPROM.begin(EEPROM_SIZE))
  {
    Serial.println("failed to initialise EEPROM");
  }
  Serial.println(" bytes read from Flash . Values are:");
  for (int i = 0; i < EEPROM_SIZE; i++)
  {
    Serial.print(byte(EEPROM.read(i))); Serial.print(" ");
  }
  /*for (int i = 0; i < EEPROM_SIZE; i++) //reset value from eeprom
  {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();*/

  startHour = EEPROM.read(0);
  startMinute = EEPROM.read(1);
  startSecond = EEPROM.read(2);
  stopHour = EEPROM.read(3);
  stoptMinute = EEPROM.read(4);
  stopSecond = EEPROM.read(5);
  EEPROM.get(6,phLow);
  EEPROM.get(10,phHigh);
  EEPROM.get(15,ecLow);
  EEPROM.get(20,ecHigh);
  /*int startHour,startMinute,startSecond; //ON HH:MM:SS 
  int stopHour,stoptMinute,stopSecond;   //OFF HH:MM:SS */
  


};

void callback(char* topic,byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  String msg;
  for (int i = 0; i < length; i++) {
    msg = msg + (char)payload[i];
  }
   Serial.println(msg);
  if (String(topic) == subscribe_topic) { 
    if (msg == "1"){
      Serial.println("Turn on LED");
    } else {
      Serial.println("Turn off LED");
    }
  }
};

void reconnectBlynk(){
  static unsigned long timepoint = 0;
  if(currentMillis - timepoint >= 1000U){
    timepoint = currentMillis;
    if (!Blynk.connected() ) {
      statusBlynk = Blynk.connected();
      
      //Serial.println("Lost Blynk server connection");
      Blynk.connect();
    } else {
      // return something
      statusBlynk = Blynk.connected();
    }
  }
}


long timer_start_set[1] = {0xFFFF};
long timer_stop_set[1] = {0xFFFF};
unsigned char weekday_set[2];

BLYNK_WRITE(V0) 
{
  unsigned char week_day;
 
  TimeInputParam t(param);

  if (t.hasStartTime() && t.hasStopTime() ) 
  {
    timer_start_set[0] = (t.getStartHour() * 60 * 60) + (t.getStartMinute() * 60) + t.getStartSecond();
    timer_stop_set[0] = (t.getStopHour() * 60 * 60) + (t.getStopMinute() * 60) + t.getStopSecond();
    
    Serial.println(String("Start Time: ") +t.getStartHour() + ":" + t.getStartMinute() + ":" + t.getStartSecond());

    startHour = t.getStartHour();
    startMinute = t.getStartMinute();
    startSecond = t.getStartSecond();

    stopHour = t.getStopHour();
    stoptMinute = t.getStopMinute();
    stopSecond = t.getStopSecond();

    Serial.println(String("Stop Time: ") + t.getStopHour() + ":" + t.getStopMinute() + ":" + t.getStopSecond());
                   
    for (int i = 1; i <= 7; i++) 
    {
      if (t.isWeekdaySelected(i)) 
      {
        week_day |= (0x01 << (i-1));
        Serial.println(String("Day ") + i + " is selected");
      }
      else
      {
        week_day &= (~(0x01 << (i-1)));
      }
    } 

    weekday_set[0] = week_day;
  }

  else
  {
    timer_start_set[0] = 0xFFFF;
    timer_stop_set[0] = 0xFFFF;
  }
}

void SerialstatusConnecting(){
  static unsigned long timepoint = 0;
  if(currentMillis - timepoint >= 10000U){
    timepoint = currentMillis;
    Serial.print("status of WiFi :");
    Serial.print(statusWifi);
    Serial.print("\t");
    Serial.print("status of Blynk :");
    Serial.print(statusBlynk);
    Serial.print("\t");
    Serial.print("status of Mqtt :");
    Serial.println(statusMqtt);


    Serial.print("ph low :");
    Serial.print(phLow);
    Serial.print("\t");
    Serial.print("ph High :");
    Serial.print(phHigh);
    Serial.print("\t");
    Serial.print("ec Low :");
    Serial.print(ecLow);
    Serial.print("\t");
    Serial.print("ec High :");
    Serial.println(ecHigh);

    //Blynk.sendInternal("rtc", "sync"); //request current local time for device
    //Serial.println(String("Start Time: ") + startHour + ":" + startMinute + ":" + startSecond);
    //Serial.println(String("Stop Time: ") + stopHour + ":" + stoptMinute + ":" + stopSecond);
  }
};

void functionLcd(){
    static unsigned long lastSaveTime = 0;
    static unsigned long currentPageTime = 0;
    static int currentPage = 0;
    if (currentMillis - currentPageTime >= 5000U) {
      currentPageTime = currentMillis;
      currentPage++;
      currentPage = currentPage % 3;

    }
    if (currentMillis - lastSaveTime >= 1000U) {
      lastSaveTime = currentMillis;
      // clear the screen
      switch(currentPage) {
        case 0:
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Temp  :" + String(t) + " c"); //"/n"
          lcd.setCursor(0,1);
          lcd.print("humid :" + String(h) + " %");
          lcd.setCursor(0,2);
          lcd.print("EC    :" + String(ecValue) + " mS/cm");
          lcd.setCursor(0,3);
          lcd.print("pH    :" + String(lastPH));
          break;
        case 1:
          lcd.clear();
          if(statusMqtt == 1 ){
            lcd.setCursor(0,0);
            lcd.print("Mqtt: connected");
          }
          else{
            lcd.setCursor(0,0);
            lcd.print("Mqtt: not connected");
          }
          if(statusBlynk == 1 ){
            lcd.setCursor(0,1);
            lcd.print("Blynk: connected");
          }
          else{
            lcd.setCursor(0,1);
            lcd.print("Blynk: not connected");
          }
          if(statusWifi == 1 ){
            lcd.setCursor(0,2);
            lcd.print("Wifi: connected");
          }
          else{
            lcd.setCursor(0,2);
            lcd.print("Wifi: not connected");
          }
          /*if(statusWifi == 1){
            lcd.setCursor(0,3);
            lcd.print(String(WiFi.localIP()));
          }   */
          break;
        case 2:
          lcd.clear();
          lcd.setCursor(0,0);
          //int nowHour,nowMinute,nowSecond;
          lcd.print(String("RTC:      ") + nowHour + ":" + nowMinute + ":" + nowSecond);
          lcd.setCursor(0,1);
          lcd.print(String("On Time:  ") + startHour + ":" + startMinute + ":" + startSecond);
          lcd.setCursor(0,2);
          lcd.print(String("Off Time: ") + stopHour + ":" + stoptMinute + ":" + stopSecond);

          
          // timer
          // rtc
          // ec set
          // pH set
          break;
      }
    }
};
void RTCfunction(){
  static unsigned long lastSaveTime = 0;
  if (currentMillis - lastSaveTime >= 1000U) {
    lastSaveTime = currentMillis;
    DateTime now = RTC.now();
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(' ');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();
    nowHour=now.hour();
    nowMinute=now.minute();
    nowSecond=now.second();
  }
};

void EEPROMfunction(){
  static unsigned long lastSaveTime = 0;
  if (currentMillis - lastSaveTime >= 10000U) {
    lastSaveTime = currentMillis;
    float phLow1 = 5.5;
    float phHigh1 = 5.9;
    float ecLow1 = 1.2;
    float ecHigh1 = 1.5;

    EEPROM.write(0, startHour);
    EEPROM.write(1, startMinute); //startMinute = byte(startMinute); optional
    EEPROM.write(2, startSecond);
    EEPROM.write(3, stopHour);
    EEPROM.write(4, stoptMinute);
    EEPROM.write(5, stopSecond);
    EEPROM.put(6, phLow1);
    EEPROM.put(10, phHigh1);
    EEPROM.put(15, ecLow1);
    EEPROM.put(20, ecHigh1);
    EEPROM.commit();
    Serial.println("EEPROM Done");


    //int startHour,startMinute,startSecond; //ON HH:MM:SS 
    //int stopHour,stoptMinute,stopSecond;   //OFF HH:MM:SS
  }
};
void readDHT(){
  static unsigned long timepoint = 0;
  if(currentMillis - timepoint >= 2000U){
    timepoint = currentMillis;
    h = dht.readHumidity();
    t = dht.readTemperature();
    if (isnan(h) || isnan(t)) {
      Serial.println(F("Failed to read from DHT sensor!"));
      return;
    }
    Serial.print(F("Humidity: "));
    Serial.print(h);
    Serial.print(F("%  Temperature: "));
    Serial.print(t);
    Serial.println(F(" C "));
   }
};
void PH(){
  static int sum = 0; // variable to store the sum of the readings
  static int count = 0; // variable to store the number of readings
  static unsigned long timepoint = 0;
  if(currentMillis - timepoint >= 100U){
    timepoint = currentMillis;
    float analogValue = analogRead(PHPIN);
    voltagePH = analogValue/4095.0*3300; //read the voltage  
    float slope = (7.0-4.0)/((neutralVoltage-1500)/3.0 - (acidVoltage-1500)/3.0); //two point: (_NautralVoltage,7.0),(_acidVoltage,4.0)
    float intercept = 7.0 - slope*(neutralVoltage-1500)/3.0;
    phValue = slope*(voltagePH-1500)/3.0+intercept; //y = k*x +b
    //----------------------AFTER CALCULATE---------------------------
    sum += phValue;// add the current reading to the sum
    count++;// increment the count of readings
    if (count == smoothFactor) {
      // calculate the average of the readings
      lastPH = sum / smoothFactor;
      Serial.print("pH: ");
      Serial.println(lastPH);
      // reset the sum and count for the next set of readings
      sum = 0;
      count = 0;
    }
  }
};
void Ec(){
  static int sum = 0; // variable to store the sum of the readings
  static int count = 0; // variable to store the number of readings
  static unsigned long timepoint = 0;
  if(currentMillis - timepoint >= 200U){
     timepoint = currentMillis;
     tds.calTDS();
     ecValue = tds.getEC()*0.001;
     sum += ecValue;// add the current reading to the sum
     count++;// increment the count of readings
     if (count == smoothFactor) {
      // calculate the average of the readings
      lastEC = sum / smoothFactor;
      // reset the sum and count for the next set of readings
      sum = 0;
      count = 0;
      Serial.print("EC: ");
      Serial.print(lastEC);
      Serial.println(" ms/cm");
     }
  }
}
