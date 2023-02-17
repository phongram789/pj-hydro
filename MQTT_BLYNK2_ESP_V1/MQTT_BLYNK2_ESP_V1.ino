// ขา 0 , 2 ห้ามเป็น low
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <BlynkSimpleEsp32.h>
#include <TimeLib.h>
#include <EEPROM.h>
#include <PZEM004Tv30.h>
#include <Wire.h>
#include "DS3231.h"
#include <LiquidCrystal_I2C.h>
#include <Bounce2.h>
#include "rtu.h"
#include "INPUT_PULLUP.h"
LiquidCrystal_I2C lcd(0x27, 20, 4);
RTClib RTC;
DS3231 Clock;

PZEM004Tv30 pzem(Serial2, 16, 17,0x07);

WiFiClient espClient;
PubSubClient mqtt(espClient);
BlynkTimer timer;
#define BLYNK_PRINT Serial

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
float t = 30.2,h = 100,ecValue = 1.3 ,phValue = 7.5;
unsigned long currentMillis = 0;



bool Rtcmodule; // 0 = error, 1 = good 
int nowHour,nowMinute,nowSecond;       //RTC HH:MM:SS
int startHour,startMinute,startSecond; //ON HH:MM:SS 
int stopHour,stopMinute,stopSecond;   //OFF HH:MM:SS
int startTimeInSeconds;
int stopTimeInSeconds;
int currentTimeInSeconds;

bool growLigh1,growLigh2,growLigh3,growLigh4; //status on working of grow light

//bool mainWaterPump; //status on working of water pump to line plant zone

bool timeonGL; //0 == off , 1 == on //value of on/off in time clock

bool controlPumpEc,controlPumpPH; //void Control 

float phLow,phHigh,ecLow,ecHigh; // first read input from eeprom and then already read from blynk n mqtt and then save in eeprom
bool relayStatePH;

bool statusBlynk,statusMqtt,statusWifi; 

float voltage ,current ,power ,energy , frequency, pf; // values from energy module

bool ecAuto, ecMan, phAuto, phMan ,GLAuto ,GLMan; //switch mode 

bool swEcModeOff,swEcModeMan,swEcModeAuto;  //switch mode

bool GrowLight1Control1,GrowLight1Control2,GrowLight1Control3,GrowLight1Control4; // read input from blynk and mtqq 0 = off, 1 = on --> save eefrom repeate
bool firstAuto = 0;
bool flowwing = 1;

#define pinSwitchEcAuto 5
#define pinSwitchEcMan 18

#define pinSwitchPHAuto 4
#define pinSwitchPHMan 15

#define pinSwitchGrowlightAuto 19
#define pinSwitchGrowlightMan 23

#define pinSwitchtestman 3

bool buttonPinMainWaterPump = 0; // 15 สลับกับขา 0 เพราะ connected to on-board LED, must be left floating or LOW to enter flashing mode

bool mainWaterPump = HIGH;
Bounce debouncer = Bounce(); // create obj

swInput swAutoEC(pinSwitchEcAuto);
swInput swManEC(pinSwitchEcMan);
swInput swAutoPH(pinSwitchPHAuto);
swInput swManPH(pinSwitchPHMan);
swInput swAutoGL(pinSwitchGrowlightAuto);
swInput swManGL(pinSwitchGrowlightMan);
swInput swManWaterIn(pinSwitchtestman);

/*#define RXD2 16
#define TXD2 17
float fVoltage;
float fCurrent;
float fPower;
float fEnergy;
float fFrequency;
uint8_t bufferModbus[25];*/

WidgetLCD lcdBlynk(V16);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  //while (!Serial2)
  //  ;
  initEEPROM();
  lcd.init();
  lcd.backlight();
  Wire.begin();//เริ่มการสื่อสารแบบ I2C
  initWiFi();
  mqtt.setServer(mqtt_server, mqtt_port);
  mqtt.setCallback(callback);
  Blynk.config(auth,"blynk.cloud", 8080);
  timer.setInterval(2000L, sendSensor);
  timer.setInterval(10000L, lcdBlynkPrint);

  //--------------push button-----------------
  debouncer.attach(buttonPinMainWaterPump, INPUT_PULLUP);
  debouncer.interval(20); // กำหนดเวลาการเปลี่ยนสถานะให้กับ debouncer object ที่ 10 มิลลิวินาที

  delay(500);
}

void loop() {
  // put your main code here, to run repeatedly:
  currentMillis = millis();
  RTCfunction();
  Mqttreconnect();
  timer.run();
  reconnectBlynk();
  controlWaterPump();//
  SerialstatusConnecting();
  functionLcd();
  EEPROMfunction();
  pzemRead();
  controlEC();
  controlPH();
  controlGrowLight();
}

void sendSensor(){
  //float voltage ,current ,power ,energy , frequency, pf;
  float h = random(0,30);
  float t = random(0,100);
  Blynk.virtualWrite(V3, h);
  Blynk.virtualWrite(V4, t);
  Blynk.virtualWrite(V5, voltage);
  Blynk.virtualWrite(V6, current);
  Blynk.virtualWrite(V7, power);
  Blynk.virtualWrite(V8, energy);
  Blynk.virtualWrite(V9, frequency);
  Blynk.virtualWrite(V15, pf);

  //You can change button labels from hardware with
  Blynk.virtualWrite(V10, growLigh1);
  Blynk.virtualWrite(V11, growLigh2);
  Blynk.virtualWrite(V12, growLigh3);
  Blynk.virtualWrite(V13, growLigh4);
  
  
};

void Mqttreconnect(){
  static unsigned long timepoint = 0;
  if(currentMillis - timepoint >= 5000U){
    timepoint = currentMillis;
    if (mqtt.connected() == false) {
      statusMqtt = mqtt.connected();
      Serial.print("MQTT connection... ");
      if (mqtt.connect(mqtt_client, mqtt_username, mqtt_password)){
        mqtt.subscribe(subscribe_topic);// DONT FORGET subscribe_topic = "@msg/temp"
        //------------------range of ec ph values---
        mqtt.subscribe("@msg/eclow"); 
        mqtt.subscribe("@msg/echigh");
        mqtt.subscribe("@msg/phlow");
        mqtt.subscribe("@msg/phhigh");
        //------------------get values to control growlight ---
        mqtt.subscribe("@msg/kmutnb/cs/smarthydroponic1/gl1"); 
        mqtt.subscribe("@msg/kmutnb/cs/smarthydroponic1/gl2");
        mqtt.subscribe("@msg/kmutnb/cs/smarthydroponic1/gl3");
        mqtt.subscribe("@msg/kmutnb/cs/smarthydroponic1/gl4");
        //------------------get values to control main water pump ---
        mqtt.subscribe("@msg/kmutnb/cs/smarthydroponic1/mainwater");

        


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
      
      String dataJS = "{\"temp\":" + String(t) + ",\"hum\":" + String(h) + ",\"ec\":" + String(ecValue) + ",\"ph\":" + String(phValue) + ",\"onhh\":" + String(startHour) + ",\"onmm\":" + String(startMinute) + ",\"onss\":" + String(startSecond) + "}";
      char json[150];
      dataJS.toCharArray(json,dataJS.length()+1);
      mqtt.publish("@msg/v1/devices/me/telemetry", json);

      //idea name of topics @msg/kmutnb/cs/smarthydroponic1/power
      //idea name of topics @msg/kmutnb/cs/smarthydroponic1/status
      //bool growLigh1,growLigh2,growLigh3,growLigh4;

      // status of growlight , all pump , power meter , 

      // startHour,startMinute,startSecond; //ON HH:MM:SS 
      /*String date = String(startHour);
      Serial.println(date);
      String dataJS2 = "{\"temp\":" + date + "}";
      char json2[100];
      dataJS2.toCharArray(json2,dataJS2.length()+1);
      mqtt.publish("@msg/v1/devices/me/telemetry2", json2);*/

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
  Serial.println("Reading...");
  int address = 0;
  startHour = EEPROM.read(address);
  address += sizeof(startHour);
  startMinute = EEPROM.read(address);
  address += sizeof(startMinute);
  startSecond = EEPROM.read(address);
  address += sizeof(startSecond);
  stopHour = EEPROM.read(address);
  address += sizeof(stopHour);
  stopMinute = EEPROM.read(address);
  address += sizeof(stopMinute);
  stopSecond = EEPROM.read(address);
  address += sizeof(stopSecond);
  EEPROM.get(address, phLow);
  address += sizeof(phLow);
  EEPROM.get(address, phHigh);
  address += sizeof(phHigh);
  EEPROM.get(address, ecLow);
  address += sizeof(ecLow);
  EEPROM.get(address, ecHigh);
  address += sizeof(ecHigh);
  GrowLight1Control1 = EEPROM.read(address);
  address += sizeof(GrowLight1Control1);
  GrowLight1Control2 = EEPROM.read(address);
  address += sizeof(GrowLight1Control2);
  GrowLight1Control3 = EEPROM.read(address);
  address += sizeof(GrowLight1Control3);
  GrowLight1Control4 = EEPROM.read(address);

  /*Serial.print("startHour: ");
  Serial.println(startHour);
  Serial.print("startMinute: ");
  Serial.println(startMinute);
  Serial.print("startSecond: ");
  Serial.println(startSecond);
  Serial.print("stopHour: ");
  Serial.println(stopHour);
  Serial.print("stopMinute: ");
  Serial.println(stopMinute);
  Serial.print("stopSecond: ");
  Serial.println(stopSecond);
  Serial.print("phLow: ");
  Serial.println(phLow);
  Serial.print("phHigh: ");
  Serial.println(phHigh);
  Serial.print("ecLow: ");
  Serial.println(ecLow);
  Serial.print("ecHigh: ");
  Serial.println(ecHigh);
  Serial.print("growLigh1: ");
  Serial.println(GrowLight1Control1);
  Serial.print("growLigh2: ");
  Serial.println(GrowLight1Control2);
  Serial.print("growLigh3: ");
  Serial.println(GrowLight1Control3);
  Serial.print("growLigh4: ");
  Serial.println(GrowLight1Control4);*/

  //work list hear
  //GrowLight1Control1,GrowLight1Control2,GrowLight1Control3,GrowLight1Control4;
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
  if (String(topic) == subscribe_topic) { //subscribe_topic = "@msg/temp"
    if (msg == "1"){
      Serial.println("Turn on LED");
    } else {
      Serial.println("Turn off LED");
    }
  }
  //float phLow,phHigh,ecLow,ecHigh;
  if (String(topic) == "@msg/eclow") { 
    ecLow = msg.toFloat();
    //Serial.print("eclow");
    //Serial.println(msg);
  }
  if (String(topic) == "@msg/echigh") { 
    ecHigh = msg.toFloat();
    //Serial.print("echigh");
    //Serial.println(msg);
  }
  if (String(topic) == "@msg/phlow") { 
    phLow = msg.toFloat();
    //Serial.print("phlow");
    //Serial.println(msg);
  }
  if (String(topic) == "@msg/phhigh") { 
    phHigh = msg.toFloat();
    //Serial.print("phhigh");
    //Serial.println(msg);
  }
  
  //-------------------------- control growlight-----------------
  //GrowLight1Control1, GrowLight1Control2, GrowLight1Control3, GrowLight1Control4
  if (String(topic) == "@msg/kmutnb/cs/smarthydroponic1/gl1") { 
    if (msg == "1"){
      GrowLight1Control1 = 1;
      firstAuto = 0;
    } else {
      GrowLight1Control1 = 0;
      firstAuto = 1;
    }
    
  }
  if (String(topic) == "@msg/kmutnb/cs/smarthydroponic1/gl2") { 
    if (msg == "1"){
      GrowLight1Control2 = 1;
      firstAuto = 0;
    } else {
      GrowLight1Control2 = 0;
      firstAuto = 1;
    }
  }
  if (String(topic) == "@msg/kmutnb/cs/smarthydroponic1/gl3") { 
    if (msg == "1"){
      GrowLight1Control3 = 1;
      firstAuto = 0;
    } else {
      GrowLight1Control3 = 0;
      firstAuto = 1;
    }
    
  }
  if (String(topic) == "@msg/kmutnb/cs/smarthydroponic1/gl4") { 
    if (msg == "1"){
      GrowLight1Control4 = 1;
      firstAuto = 0;
    } else {
      GrowLight1Control4 = 0;
      firstAuto = 1;
    }
  }
  
  //-------------------------main water pump pumping to line plant
  if (String(topic) == "@msg/kmutnb/cs/smarthydroponic1/mainwater") { 
    if (msg == "1"){
      mainWaterPump = HIGH; 
    } else {
      mainWaterPump = LOW;
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

BLYNK_WRITE(V0){
  unsigned char week_day;
  TimeInputParam t(param);
  if (t.hasStartTime() && t.hasStopTime() ){
    startHour = t.getStartHour();
    startMinute = t.getStartMinute();
    startSecond = t.getStartSecond();

    stopHour = t.getStopHour();
    stopMinute = t.getStopMinute();
    stopSecond = t.getStopSecond();

    Serial.println(String("Stop Time: ") + t.getStopHour() + ":" + t.getStopMinute() + ":" + t.getStopSecond());
  }else{

  }
}

void SerialstatusConnecting(){
  static unsigned long timepoint = 0;
  if(currentMillis - timepoint >= 10000U){
    timepoint = currentMillis;
    /*Serial.print("status of WiFi :");
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
    Serial.println(ecHigh);*/

    //Blynk.sendInternal("rtc", "sync"); //request current local time for device
    //Serial.println(String("Start Time: ") + startHour + ":" + startMinute + ":" + startSecond);
    //Serial.println(String("Stop Time: ") + stopHour + ":" + stoptMinute + ":" + stopSecond);
    //GrowLight1Control1,GrowLight1Control2,GrowLight1Control3,GrowLight1Control4
  /*Serial.print("relayState growLigh1 mode off: ");
  Serial.println(String(growLigh1) + " " + String(GrowLight1Control1) );
  Serial.print("relayState growLigh2 mode off: ");
  Serial.println(String(growLigh2) + " " + String(GrowLight1Control2) );
  Serial.print("relayState growLigh3 mode off: ");
  Serial.println(String(growLigh3) + " " + String(GrowLight1Control3) );
  Serial.print("relayState growLigh4 mode off: ");
  Serial.println(String(growLigh4) + " " + String(GrowLight1Control4) );
  Serial.print("relayState GLMan: ");
  Serial.println(GLMan);
  Serial.print("relayState GLAuto: ");
  Serial.println(GLAuto);
  Serial.print("time on: ");
  Serial.println(timeonGL);  
  Serial.println(currentTimeInSeconds);
  Serial.println(startTimeInSeconds);
  Serial.println(stopTimeInSeconds);*/
  /* int startTimeInSeconds;
  int stopTimeInSeconds;
  int currentTimeInSeconds;*/
  /*Serial.print("mode controlPumpEc :");
  Serial.println(controlPumpEc);
  Serial.print("mode controlPumpPh :");
  Serial.println(controlPumpPH);*/
  //Serial.println("swManWaterIn : " + String(swManWaterIn.get_status()));
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
          lcd.print("pH    :" + String(phValue));
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
          lcd.print(String("Off Time: ") + stopHour + ":" + stopMinute + ":" + stopSecond);

          
          // timer
          // rtc
          // ec set
          // pH set
          break;
      }
    }
};

void clearSerial2Buffer() {
  Serial2.setTimeout(1); // set a small timeout to discard incoming data
  while (Serial2.available()) {
    Serial2.read(); // read any incoming data (which will be discarded)
  }
  Serial2.setTimeout(1000); // set the timeout back to a normal value 
}

void pzemRead(){
  static unsigned long lastSaveTime = 0;
  static int countError;
    if (currentMillis - lastSaveTime >= 8000U) {
      lastSaveTime = currentMillis;
      clearSerial2Buffer();

        //Serial.print("Custom Address:");
        //Serial.println(pzem.readAddress(), HEX);

        // Read the data from the sensor
         voltage = pzem.voltage();
         current = pzem.current();
         power = pzem.power();
         energy = pzem.energy();
         frequency = pzem.frequency();
         pf = pzem.pf();

        // Check if the data is valid
        if(isnan(voltage)){
          countError++;
          //Serial.println("Error reading voltage: "+ String(countError));
        } else if (isnan(current)) {
            Serial.println("Error reading current");
        } else if (isnan(power)) {
            Serial.println("Error reading power");
        } else if (isnan(energy)) {
            Serial.println("Error reading energy");
        } else if (isnan(frequency)) {
            Serial.println("Error reading frequency");
        } else if (isnan(pf)) {
            Serial.println("Error reading power factor");
        } else {
            countError=0;

            // Print the values to the Serial console
            /*Serial.print("Voltage: ");      Serial.print(voltage);      Serial.println("V");
            Serial.print("Current: ");      Serial.print(current);      Serial.println("A");
            Serial.print("Power: ");        Serial.print(power);        Serial.println("W");
            Serial.print("Energy: ");       Serial.print(energy,3);     Serial.println("kWh");
            Serial.print("Frequency: ");    Serial.print(frequency, 1); Serial.println("Hz");
            Serial.print("PF: ");           Serial.println(pf);*/

        }
        //Serial.println();
      
    }
}

void RTCfunction(){
  static unsigned long lastSaveTime = 0;
  static unsigned long lastSaveTimeError;
  if (currentMillis - lastSaveTime >= 1000U) {
    lastSaveTime = currentMillis;
    DateTime now = RTC.now();
    /*Serial.print(now.year(), DEC);
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
    Serial.println();*/
    int year = now.year();
    //Serial.println("This year: " + String(year));
    if(year  < 2023 || year  > 2100){
      Rtcmodule = 0;
      if (currentMillis - lastSaveTimeError >= 10000U) {
        //Serial.println("RTC ERROR");
        requestTime();
        lastSaveTimeError = currentMillis;
      }
    }
    else{
      Rtcmodule = 1;
      nowHour=now.hour();
      nowMinute=now.minute();
      nowSecond=now.second();
    }
  }
};

void EEPROMfunction(){
  static unsigned long lastSaveTime = 0;
  if (currentMillis - lastSaveTime >= 10000U) {
    int address = 0;
    lastSaveTime = currentMillis;
    EEPROM.write(address, startHour);
    address += sizeof(startHour);
    EEPROM.write(address, startMinute);
    address += sizeof(startMinute);
    EEPROM.write(address, startSecond);
    address += sizeof(startSecond);
    EEPROM.write(address, stopHour);
    address += sizeof(stopHour);
    EEPROM.write(address, stopMinute);
    address += sizeof(stopMinute);
    EEPROM.write(address, stopSecond);
    address += sizeof(stopSecond);
    EEPROM.put(address, phLow);
    address += sizeof(phLow);
    EEPROM.put(address, phHigh);
    address += sizeof(phHigh);
    EEPROM.put(address, ecLow);
    address += sizeof(ecLow);
    EEPROM.put(address, ecHigh);
    address += sizeof(ecHigh);
    EEPROM.write(address, GrowLight1Control1);
    address += sizeof(GrowLight1Control1);
    EEPROM.write(address, GrowLight1Control2);
    address += sizeof(GrowLight1Control2);
    EEPROM.write(address, GrowLight1Control3);
    address += sizeof(GrowLight1Control3);
    EEPROM.write(address, GrowLight1Control4);
    EEPROM.commit();
    //work list hear
    //
    //Serial.println("EEPROM Done");
    //work list is done
    //startHour,startMinute,startSecond ON HH:MM:SS 
    //stopHour,stoptMinute,stopSecond   OFF HH:MM:SS
  }
};

void controlEC(){ //switch mode --> codition of lv EC value --> Return state of relay
  static unsigned long lastSaveTimeAuto;
  static unsigned long lastSaveTimeMan;
  static unsigned long lastSaveTimeoOff;
  static unsigned long lastSaveTimeOffAuto;
  static bool relayStateEC;
  const int relayOnTime = 5000;     // Relay on time in milliseconds
  const int relayOffTime = 60000;   // Relay off time in milliseconds
  
  ecAuto = swAutoEC.get_status();
  ecMan = swManEC.get_status();
  /*
  Serial.print("ecAuto :");Serial.println(ecAuto);
  Serial.print("ecMan :");Serial.println(ecMan);
  */
  
  //-------------------- can add this condition in auto mode 
  if(ecValue < ecLow){ // pump AB solution to up ec value in water
    controlPumpEc = 1;
  }
  else if((ecValue >= ecLow)&&(ecValue <= ecHigh)){ // good Ec Value
    controlPumpEc = 0;
  }
  else{ // high ec value in water
    controlPumpEc = 0;
  }

  // Select switch
  if((ecAuto == 1)&&(ecMan == 1)){ // swEcModeOff
    // Rtu relay off case 0
    if(relayStateEC != LOW){
      Serial.print("relayState mode off: ");
      Serial.println(relayStateEC);
      relayRtu(2);
      relayStateEC = LOW;
    }
    
    if (currentMillis - lastSaveTimeoOff >= 60000U) {
      Serial.print("relayState mode off: ");
      Serial.println(relayStateEC);
      relayRtu(2);
      relayStateEC = LOW;
      lastSaveTimeoOff = currentMillis;
      //Serial.println("sw man on");
    }    
  }
  else if ((ecAuto == 0)&&(ecMan == 1)){ //swEcModeAuto
    if(controlPumpEc == 1){
      if (currentMillis - lastSaveTimeAuto >= (relayStateEC == HIGH ? relayOnTime : relayOffTime)) {
        // Toggle the state of the relay
        relayStateEC = !relayStateEC;
        if(relayStateEC == HIGH){
          Serial.print("relayStateEC: ");
          Serial.println(relayStateEC);  
          relayRtu(1);   
        }
        else{
          Serial.print("relayStateEC else: ");
          Serial.println(relayStateEC);
          relayRtu(2);
        }
        // Store the current time
        lastSaveTimeAuto = currentMillis;
      }
    }
    else{
      // Relay RTU Off
      if(relayStateEC != LOW){
        Serial.print("relayState mode off: ");
        Serial.println(relayStateEC);
        relayRtu(2);
        relayStateEC = LOW;
      }
      
      if (currentMillis - lastSaveTimeOffAuto >= 60000U) {
        Serial.print("relayState mode off: ");
        Serial.println(relayStateEC);
        relayRtu(2);
        relayStateEC = LOW;
        lastSaveTimeOffAuto = currentMillis;
        //Serial.println("sw man on");
      }    
      //Serial.println("relayOffTime with good ec value");
    }
  }
  else if ((ecAuto == 1)&&(ecMan == 0)){ //swEcModeMan
    // Rtu relay on
    if(relayStateEC != HIGH){
      Serial.print("relayStateEC mode MAN: ");
      Serial.println(relayStateEC);
      relayRtu(1);
      relayStateEC = HIGH;
    }
    
    if (currentMillis - lastSaveTimeMan >= 60000U) {
      relayStateEC = HIGH;
      Serial.print("relayStateEC mode MAN: ");
      Serial.println(relayStateEC);
      relayRtu(1);
      lastSaveTimeMan = currentMillis;
      //Serial.println("sw man on");
    }
  }
  else{
    // serial error
    // Rtu relay off
    Serial.println("sw error");
  }



}
void controlPH(){ //switch mode --> codition of lv pH value --> Return state of relay 
  static unsigned long lastSaveTimeAuto;
  static unsigned long lastSaveTimeMan;
  static unsigned long lastSaveTimeoOff;
  static unsigned long lastSaveTimeOffElseAuto;
  const int relayOnTime = 5000;     // Relay on time in milliseconds
  const int relayOffTime = 60000;   // Relay off time in milliseconds

  phAuto = swAutoPH.get_status();
  phMan = swManPH.get_status();
  
  // Select switch
  if((phAuto == 1)&&(phMan == 1)){ // swphModeOff
    // Rtu relay off case 0
    if(relayStatePH != LOW){
      Serial.print("relayState ph mode off: ");
      Serial.println(relayStatePH);
      relayRtu(4);
      relayStatePH = LOW;
    }
    
    if (currentMillis - lastSaveTimeoOff >= 60000U) {
      Serial.print("relayState ph mode off: ");
      Serial.println(relayStatePH);
      relayRtu(4);
      relayStatePH = LOW;
      lastSaveTimeoOff = currentMillis;
      //Serial.println("sw man on");
    }    
  }
  else if ((phAuto == 0)&&(phMan == 1)){ //swphModeAuto

    //-------------------- can add this condition in auto mode 
    if((phValue > phHigh)&&(controlPumpEc != 1)){ // pump pH solution to up pH value in water //anything else not auto ph with ec man
      controlPumpPH = 1;
    }
    else if((phValue >= phLow)&&(phValue <= phHigh)){ // good pH Value
      controlPumpPH = 0;
    }
    else{ // low pH value in water
      controlPumpPH = 0;
    }

    /*if( relayStatePH != LOW && relayStateEC == HIGH ){ //else if 
      relayRtu(4);
      relayStatePH = LOW;
    }*/
    
    if(controlPumpPH == 1){
      if (currentMillis - lastSaveTimeAuto >= (relayStatePH == HIGH ? relayOnTime : relayOffTime)) {
        // Toggle the state of the relay
        relayStatePH = !relayStatePH;
        if(relayStatePH == HIGH){
          Serial.print("relayState ph: ");
          Serial.println(relayStatePH);  
          relayRtu(3);
          relayStatePH = HIGH;        
        }
        else{
          Serial.print("relayState ph else: ");
          Serial.println(relayStatePH);
          relayRtu(4);
          relayStatePH = LOW;
        }
        // Store the current time
        lastSaveTimeAuto = currentMillis;
      }
    }
    else{
      // Relay RTU Off 
      if(relayStatePH != LOW){
        Serial.print("relayState ph mode auto else: ");
        Serial.println(relayStatePH);
        relayRtu(4);
        relayStatePH = LOW;
      }
      
      if (currentMillis - lastSaveTimeOffElseAuto >= 120000U) {
        Serial.print("relayState ph mode auto else: ");
        Serial.println(relayStatePH);
        relayRtu(4);
        relayStatePH = LOW;
        lastSaveTimeOffElseAuto = currentMillis;
        //Serial.println("sw man on");
      }  
    }
  }
  else if ((phAuto == 1)&&(phMan == 0)){ //swphModeMan
    // Rtu relay on
    if(relayStatePH != HIGH){
      Serial.print("relayState mode MAN: ");
      Serial.println(relayStatePH);
      relayRtu(3);
      relayStatePH = HIGH;
    }
    
    if (currentMillis - lastSaveTimeMan >= 60000U) {
      relayStatePH = HIGH;
      Serial.print("relayState mode MAN: ");
      Serial.println(relayStatePH);
      relayRtu(3);
      lastSaveTimeMan = currentMillis;
      //Serial.println("sw man on");
    }
  }
  else{
    // serial error
    // Rtu relay off
    Serial.println("sw error");
  }

  
  
}
void controlGrowLight(){ //switch mode --> timer --> Return state of relay
  static unsigned long lastSaveTimeoOff;
  static unsigned long lastSaveTimeoOn;
  static unsigned long lastSaveTimeoAuto;
  GLAuto = swAutoGL.get_status();
  GLMan = swManGL.get_status();
  /*
   nowHour,nowMinute,nowSecond;       //RTC HH:MM:SS
   startHour,startMinute,startSecond; //ON HH:MM:SS 
   stopHour,stoptMinute,stopSecond;   //OFF HH:MM:SS
   growLigh1,growLigh2,growLigh3,growLigh4;
  */

  // Select switch
  if((GLAuto == 1)&&(GLMan == 1)){ // sw grow light Mode off
    firstAuto = 0;
    if(growLigh1 != LOW){
      relayRtu(8);
      growLigh1 = LOW;
    }
    if(growLigh2 != LOW){
      relayRtu(10);
      growLigh2 = LOW;
    }
    if(growLigh3 != LOW){
      relayRtu(12);
      growLigh3 = LOW;
    }
    if(growLigh4 != LOW){
      relayRtu(14);
      growLigh4 = LOW;
    }

    if (currentMillis - lastSaveTimeoOff >= 120000U) {
      relayRtu(8);// off command growLigh1
      growLigh1 = LOW;
      relayRtu(10);// off command growLigh2
      growLigh2 = LOW;
      relayRtu(12);// off command growLigh3
      growLigh3 = LOW;
      relayRtu(14);// off command growLigh4
      growLigh4 = LOW;
      lastSaveTimeoOff = currentMillis;
      //Serial.println("sw man on");
    }
  }
  else if ((GLAuto == 0)&&(GLMan == 1)){ //sw grow light Mode Auto *timer*
  //note GrowLight1Control1,GrowLight1Control2,GrowLight1Control3,GrowLight1Control4;
    startTimeInSeconds = abs((startHour * 60 * 60) + (startMinute * 60) + startSecond);
    stopTimeInSeconds = abs((stopHour * 60 * 60) + (stopMinute * 60) + stopSecond);
    currentTimeInSeconds = abs((nowHour * 60 * 60) + (nowMinute * 60) + nowSecond);

    if(currentTimeInSeconds >= startTimeInSeconds && currentTimeInSeconds < stopTimeInSeconds){
      timeonGL = 1;
    }
    else{
      timeonGL = 0;
    }

  if (currentMillis - lastSaveTimeoAuto >= 120000U || firstAuto == 0) {
    lastSaveTimeoAuto = currentMillis;
    firstAuto = 1;
    if(timeonGL == 1 && GrowLight1Control1 == 1){
      relayRtu(7);
      growLigh1 = HIGH;
    }else{
      relayRtu(8);
      growLigh1 = LOW;
    }
    if(timeonGL == 1 && GrowLight1Control2 == 1){
      relayRtu(9);
      growLigh2 = HIGH;
    }else{
      relayRtu(10);
      growLigh2 = LOW;
    }
    if(timeonGL == 1 && GrowLight1Control3 == 1){
      relayRtu(11);
      growLigh3 = HIGH;
    }else{
      relayRtu(12);
      growLigh3 = LOW;
    }
    if(timeonGL == 1 && GrowLight1Control4 == 1){
      relayRtu(13);
      growLigh4 = HIGH;
    }else{
      relayRtu(14);
      growLigh4 = LOW;
    }
  }


 

  }
  else if ((GLAuto == 1)&&(GLMan == 0)){ //sw grow light Mode Man
    firstAuto = 0;
    if(growLigh1 != HIGH){
      relayRtu(7);
      growLigh1 = HIGH;
    }
    if(growLigh2 != HIGH){
      relayRtu(9);
      growLigh2 = HIGH;
    }
    if(growLigh3 != HIGH){
      relayRtu(11);
      growLigh3 = HIGH;
    }
    if(growLigh4 != HIGH){
      relayRtu(13);
      growLigh4 = HIGH;
    }

    if (currentMillis - lastSaveTimeoOn >= 120000U) {
      relayRtu(7);// on command growLigh1
      growLigh1 = HIGH;
      relayRtu(9);// on command growLigh2
      growLigh2 = HIGH;
      relayRtu(11);// on command growLigh3
      growLigh3 = HIGH;
      relayRtu(13);// on command growLigh4
      growLigh4 = HIGH;
      lastSaveTimeoOn = currentMillis;
      //Serial.println("sw man on");
    }

  }
  else{
    // serial error
    // Rtu relay off
    Serial.println("sw GrowLight error");
  }

  
}
void controlWaterPump(){ //switch mode --> Return state of relay
  static bool firstpump;
  static unsigned long lastSaveTimeHigh;
  static unsigned long lastSaveTimeLow;
  debouncer.update();

  // กำหนดเงื่อนไขให้โค้ดโปรแกรมในวงเล็บปีกกาทำงานเมื่อสถานะปุ่มกดเปลี่ยนจาก HIGH เป็น LOW โดยเช็คจากฟังก์ชั่น fell()
  // หากต้องการเช็คสถานะจาก LOW เป็น HIGH ให้แทนที่ฟังก์ชั่น fell() ด้วยฟังก์ชั่น rose()
  if ( debouncer.fell() ) { 
      mainWaterPump  = !mainWaterPump ; // สลับสถานะ
  }
  
    if (currentMillis - lastSaveTimeHigh >= 60000U  && mainWaterPump == HIGH && flowwing == 1) {
      //firstpump = 1;
      relayRtu(15);
      mainWaterPump = HIGH;
      lastSaveTimeHigh = currentMillis;
      
    }
    //RTU off
    else if (currentMillis - lastSaveTimeLow >= 60000U  && mainWaterPump == LOW) {
      //firstpump = 0;
      relayRtu(16);
      mainWaterPump = LOW;
      lastSaveTimeLow = currentMillis;
    }else{

    }
    
  

}
void controlWaterLevel(){ //switch mode --> lv of water in tank --> Return state of relay
  
}
void relayRtu(int condition){ // manage state of relay on/off ch.
  switch(condition) {
    case 1:
      //sending conmand relay on ec ch
      //relay 50-100
      Serial2.write(ON_RTU1, 8);
      Serial.println("case 1:");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
    case 2:
      //sending conmand relay off ec ch
      //relay 50-100
      Serial2.write(OFF_RTU1, 8);
      Serial.println("case 1:");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
    case 3:
      //sending conmand relay on ph ch
      //relay 50-100
      Serial2.write(ON_RTU2, 8);
      Serial.println("case 3:");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
    case 4:
      //sending conmand relay off ph ch
      //relay 50-100
      Serial2.write(OFF_RTU2, 8);
      Serial.println("case 4:");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;

    case 5:
      //sending conmand relay off pumpกวน
      //relay 50-100
      Serial2.write(ON_RTU3, 8);
      Serial.println("case 5:");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
    case 6:
      //sending conmand relay off  pumpกวน
      //relay 50-100
      Serial2.write(OFF_RTU3, 8);
      delay(50);
      Serial.println("case 6:");
      Serial2.flush();
      //Serial2.flushReceive();
      break;

    case 7:
      //sending conmand relay on gl1
      //relay 50-100
      Serial2.write(ON_RTU16, 8);
      Serial.println("case 7:");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
    case 8:
      //sending conmand relay off  gl1
      //relay 50-100
      Serial2.write(OFF_RTU16, 8);
      Serial.println("case 8:");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
    case 9:
      //sending conmand relay on gl2
      //relay 50-100
      Serial2.write(ON_RTU15, 8);
      Serial.println("case 9:");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
    case 10:
      //sending conmand relay off  gl2
      //relay 50-100
      Serial2.write(OFF_RTU15, 8);
      Serial.println("case 10:");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
    case 11:
      //sending conmand relay on gl3
      //relay 50-100
      Serial2.write(ON_RTU14, 8);
      Serial.println("case 11:");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
    case 12:
      //sending conmand relay off  gl3
      //relay 50-100
      Serial2.write(OFF_RTU14, 8);
      Serial.println("case 12:");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
    case 13:
      //sending conmand relay on gl4
      //relay 50-100
      Serial2.write(ON_RTU13, 8);
      Serial.println("case 13:");
      delay(50);
      Serial2.flush();
     // Serial2.flushReceive();
      break;
    case 14:
      //sending conmand relay off  g14
      //relay 50-100
      Serial2.write(OFF_RTU13, 8);
      Serial.println("case 14:");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
    
    case 15:
      //sending conmand relay off pumpกวน
      //relay 50-100
      Serial2.write(ON_RTU12, 8);
      Serial.println("case 15:");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
    case 16:
      //sending conmand relay off  pumpกวน
      //relay 50-100
      Serial2.write(OFF_RTU12, 8);
      Serial.println("case 16:");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
  }
}
void requestTime(){
  Blynk.sendInternal("rtc","sync"); //using when want to get time clock
  //Serial.println("requestTime()");
}

BLYNK_WRITE(InternalPinRTC){
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013
  unsigned long blynkTime = param.asLong();
  int day1,month1,year1,weekday1,nowSecond1,nowMinute1,nowHour1; //18 02 2023 6
  
  if (blynkTime >= DEFAULT_TIME) 
  {
    setTime(blynkTime);
    //Serial.println(blynkTime);
    //Serial.println(String("RTC Server: ") + hour() + ":" + minute() + ":" + second());
    //String currentDate = String(day()) + " " + month() + " " + year();
    //Serial.println(currentDate);
    //Serial.println(String("Day of Week: ") + weekday()); 
    int 
    day1 = day();
    month1 = month();
    year1 = year();
    weekday1 = weekday();
    nowHour1 = hour();
    nowMinute1 = minute();
    nowSecond1 = second();

    nowHour = nowHour1;
    nowMinute = nowMinute1;
    nowSecond = nowSecond1;
    
    settime(year1,month1,day1,weekday1,nowHour,nowMinute,nowSecond);
    
  }
}
void settime(byte Year,byte Month,byte Date,byte DoW,byte Hour,byte Minute,byte Second){ 
  static int countSetError = 0 ;
  if(countSetError < 1){
    Clock.setYear(Year);
    Clock.setMonth(Month);
    Clock.setDate(Date);
    Clock.setDoW(DoW);
    Clock.setHour(Hour);
    Clock.setMinute(Minute);
    Clock.setSecond(Second);
    countSetError++;
  }
}

//GrowLight1Control1,GrowLight1Control2,GrowLight1Control3,GrowLight1Control4; 
BLYNK_WRITE(V10){
  GrowLight1Control1 = param.asInt();
  Serial.println(GrowLight1Control1);

}
BLYNK_WRITE(V11){
  GrowLight1Control2 = param.asInt();
  Serial.println(GrowLight1Control2);
}
BLYNK_WRITE(V12){
  GrowLight1Control3 = param.asInt();
  Serial.println(GrowLight1Control3);
}
BLYNK_WRITE(V13){
  GrowLight1Control4 = param.asInt();
  Serial.println(GrowLight1Control4);
}
BLYNK_WRITE(V14){
  buttonPinMainWaterPump = param.asInt();
  //BLYNK_LOG("buttonPinMainWaterPump: %d",buttonPinMainWaterPump);
  Serial.println(buttonPinMainWaterPump);

}

BLYNK_WRITE(V17){
  phLow = param.asFloat();
  //BLYNK_LOG("phLow: %d",phLow);
  Serial.println(phLow);

}
BLYNK_WRITE(V18){
  phHigh = param.asFloat();
  //BLYNK_LOG("phHigh: %d",phHigh);
  Serial.println(phHigh);
}
BLYNK_WRITE(V19){
  ecLow = param.asFloat();
  //BLYNK_LOG("phHigh: %d",phHigh);
  Serial.println(ecLow);
}
BLYNK_WRITE(V20){
  ecHigh = param.asFloat();
  //BLYNK_LOG("phHigh: %d",phHigh);
  Serial.println(ecHigh);
}

void lcdBlynkPrint(){
  //x = position symbol 0 -15 , y = line 0,1
  lcdBlynk.clear();
  const String text = String("RTC:") + nowHour + ":" + nowMinute + ":" + nowSecond;
  const String text2 =  String("On Time : ") + startHour + ":" + startMinute + ":" + startSecond;
  const String text3 =  String("Off Time: ") + stopHour + ":" + stopMinute + ":" + stopSecond;
  //Serial.println(text);

  lcdBlynk.clear(); //Use it to clear the LCD Widget
  lcdBlynk.print(0, 0, text2); // use: (position X: 0-15, position Y: 0-1, "Message you want to print")
  lcdBlynk.print(0, 1, text3); // use: (position X: 0-15, position Y: 0-1, "Message you want to print")
  lcdBlynk.print(0, 2, text);
  
 
}