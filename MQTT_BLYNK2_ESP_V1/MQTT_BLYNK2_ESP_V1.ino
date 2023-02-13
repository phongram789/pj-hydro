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
#include "rtu.h"
#include "INPUT_PULLUP.h"
LiquidCrystal_I2C lcd(0x27, 20, 4);
RTClib RTC;

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
int stopHour,stoptMinute,stopSecond;   //OFF HH:MM:SS

bool growLigh1,growLigh2,growLigh3,growLigh4; //status on working of grow light

bool mainWaterPump; //status on working of water pump to line plant zone

bool timeonGL; //0 == off , 1 == on //value of on/off in time clock

bool controlPumpEc,controlPumpPH; //void Control 

float phLow,phHigh,ecLow,ecHigh; // first read input from eeprom and then already read from blynk n mqtt and then save in eeprom

bool statusBlynk,statusMqtt,statusWifi; 

float voltage ,current ,power ,energy , frequency, pf; // values from energy module

bool ecAuto, ecMan, phAuto, phMan ,GLAuto ,GLMan; //switch mode 

bool swEcModeOff,swEcModeMan,swEcModeAuto;  //switch mode

bool GrowLight1Control1,GrowLight1Control2,GrowLight1Control3,GrowLight1Control4; // read input from blynk and mtqq 0 = off, 1 = on --> save eefrom repeate

#define pinSwitchEcAuto 5
#define pinSwitchEcMan 18

#define pinSwitchPHAuto 4
#define pinSwitchPHMan 0

#define pinSwitchGrowlightAuto 19
#define pinSwitchGrowlightMan 23

const int buttonPinMainWaterPump = 15;
int buttonState = HIGH;

swInput swAutoEC(pinSwitchEcAuto);
swInput swManEC(pinSwitchEcMan);
swInput swAutoPH(pinSwitchPHAuto);
swInput swManPH(pinSwitchPHMan);
swInput swAutoGL(pinSwitchGrowlightAuto);
swInput swManGL(pinSwitchGrowlightMan);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  initEEPROM();
  lcd.init();
  lcd.backlight();
  Wire.begin();//เริ่มการสื่อสารแบบ I2C
  initWiFi();
  mqtt.setServer(mqtt_server, mqtt_port);
  mqtt.setCallback(callback);
  Blynk.config(auth,"blynk.cloud", 8080);
  timer.setInterval(2000L, sendSensor);

  pinMode(buttonPin, INPUT);//
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

  Serial.print("relayState growLigh1 mode off: ");
  Serial.println(growLigh1);
  Serial.print("relayState growLigh2 mode off: ");
  Serial.println(growLigh2);
  Serial.print("relayState growLigh3 mode off: ");
  Serial.println(growLigh3);
  Serial.print("relayState growLigh4 mode off: ");
  Serial.println(growLigh4);


  Serial.print("relayState GLMan: ");
  Serial.println(GLMan);
  Serial.print("relayState GLAuto: ");
  Serial.println(GLAuto);
}

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

  //work list hear
  //GrowLight1Control1,GrowLight1Control2,GrowLight1Control3,GrowLight1Control4;

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
    } else {
      GrowLight1Control1 = 0;
    }
    
  }
  if (String(topic) == "@msg/kmutnb/cs/smarthydroponic1/gl2") { 
    if (msg == "1"){
      GrowLight1Control2 = 1;
    } else {
      GrowLight1Control2 = 0;
    }
  }
  if (String(topic) == "@msg/kmutnb/cs/smarthydroponic1/gl3") { 
    if (msg == "1"){
      GrowLight1Control3 = 1;
    } else {
      GrowLight1Control3 = 0;
    }
    
  }
  if (String(topic) == "@msg/kmutnb/cs/smarthydroponic1/gl4") { 
    if (msg == "1"){
      GrowLight1Control4 = 1;
    } else {
      GrowLight1Control4 = 0;
    }
  }
  
  //-------------------------main water pump pumping to line plant
  if (String(topic) == "@msg/kmutnb/cs/smarthydroponic1/mainwater") { 
    if (msg == "1"){
      mainWaterPump = 1;
    } else {
      mainWaterPump = 0;
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
          lcd.print(String("Off Time: ") + stopHour + ":" + stoptMinute + ":" + stopSecond);

          
          // timer
          // rtc
          // ec set
          // pH set
          break;
      }
    }
};
void pzemRead(){
  static unsigned long lastSaveTime = 0;
    if (currentMillis - lastSaveTime >= 2000U) {
      lastSaveTime = currentMillis;
      for(int i = 1 ; i <= 2 ; i++){
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
          //Serial.println("Error reading voltage");
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

            // Print the values to the Serial console
            /*Serial.print("Voltage: ");      Serial.print(voltage);      Serial.println("V");
            Serial.print("Current: ");      Serial.print(current);      Serial.println("A");
            Serial.print("Power: ");        Serial.print(power);        Serial.println("W");
            Serial.print("Energy: ");       Serial.print(energy,3);     Serial.println("kWh");
            Serial.print("Frequency: ");    Serial.print(frequency, 1); Serial.println("Hz");
            Serial.print("PF: ");           Serial.println(pf);*/

        }
        //Serial.println();
        delay(100);
      }
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
    if(now.year() < 2023){
      Rtcmodule = 0;
      if (currentMillis - lastSaveTimeError >= 10000U) {
        Serial.println("RTC ERROR");
        requestTime();
        lastSaveTimeError = currentMillis;
      }
    }
    else{
      Rtcmodule = 1;
    }
    nowHour=now.hour();
    nowMinute=now.minute();
    nowSecond=now.second();
  }
};

void EEPROMfunction(){
  static unsigned long lastSaveTime = 0;
  if (currentMillis - lastSaveTime >= 10000U) {
    lastSaveTime = currentMillis;


    //work list hear
    //GrowLight1Control1,GrowLight1Control2,GrowLight1Control3,GrowLight1Control4;

    EEPROM.write(0, startHour);
    EEPROM.write(1, startMinute); //startMinute = byte(startMinute); optional
    EEPROM.write(2, startSecond);
    EEPROM.write(3, stopHour);
    EEPROM.write(4, stoptMinute);
    EEPROM.write(5, stopSecond);
    EEPROM.put(6, phLow);
    EEPROM.put(10, phHigh);
    EEPROM.put(15, ecLow);
    EEPROM.put(20, ecHigh);
    EEPROM.commit();
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
  static bool relayState;
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
    if(relayState != LOW){
      Serial.print("relayState mode off: ");
      Serial.println(relayState);
      relayRtu(2);
      relayState = LOW;
    }
    
    if (currentMillis - lastSaveTimeoOff >= 60000U) {
      Serial.print("relayState mode off: ");
      Serial.println(relayState);
      relayRtu(2);
      relayState = LOW;
      lastSaveTimeoOff = currentMillis;
      //Serial.println("sw man on");
    }    
  }
  else if ((ecAuto == 0)&&(ecMan == 1)){ //swEcModeAuto
    if(controlPumpEc == 1){
      if (currentMillis - lastSaveTimeAuto >= (relayState == HIGH ? relayOnTime : relayOffTime)) {
        // Toggle the state of the relay
        relayState = !relayState;
        if(relayState == HIGH){
          Serial.print("relayState: ");
          Serial.println(relayState);  
          relayRtu(1);          
        }
        else{
          Serial.print("relayState else: ");
          Serial.println(relayState);
          relayRtu(2);
        }
        // Store the current time
        lastSaveTimeAuto = currentMillis;
      }
    }
    else{
      // Relay RTU Off
      relayState = LOW;
      //Serial.println("relayOffTime with good ec value");
    }
  }
  else if ((ecAuto == 1)&&(ecMan == 0)){ //swEcModeMan
    // Rtu relay on
    if(relayState != HIGH){
      Serial.print("relayState mode MAN: ");
      Serial.println(relayState);
      relayRtu(1);
      relayState = HIGH;
    }
    
    if (currentMillis - lastSaveTimeMan >= 60000U) {
      relayState = HIGH;
      Serial.print("relayState mode MAN: ");
      Serial.println(relayState);
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
  static bool relayStatePH;
  const int relayOnTime = 5000;     // Relay on time in milliseconds
  const int relayOffTime = 60000;   // Relay off time in milliseconds

  phAuto = swAutoPH.get_status();
  phMan = swManPH.get_status();
  
  /*Serial.print("phAuto :");Serial.println(phAuto);
  Serial.print("phMan :");Serial.println(phMan);*/
  
  
  //-------------------- can add this condition in auto mode 
  if((phValue > phHigh)&&(controlPumpEc != 1)){ // pump pH solution to up pH value in water
    controlPumpPH = 1;
  }
  else if((phValue >= phLow)&&(phValue <= phHigh)){ // good pH Value
    controlPumpPH = 0;
  }
  else{ // low pH value in water
    controlPumpPH = 0;
  }
  
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
    if(controlPumpPH == 1){
      if (currentMillis - lastSaveTimeAuto >= (relayStatePH == HIGH ? relayOnTime : relayOffTime)) {
        // Toggle the state of the relay
        relayStatePH = !relayStatePH;
        if(relayStatePH == HIGH){
          Serial.print("relayState ph: ");
          Serial.println(relayStatePH);  
          relayRtu(3);          
        }
        else{
          Serial.print("relayState ph else: ");
          Serial.println(relayStatePH);
          relayRtu(4);
        }
        // Store the current time
        lastSaveTimeAuto = currentMillis;
      }
    }
    else{
      // Relay RTU Off 
      relayStatePH = LOW;
      //Serial.println("relayOffTime with good ph value");
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
  static bool firstAuto = 0;
  GLAuto = swAutoGL.get_status();
  GLMan = swManGL.get_status();
  /*
   nowHour,nowMinute,nowSecond;       //RTC HH:MM:SS
   startHour,startMinute,startSecond; //ON HH:MM:SS 
   stopHour,stoptMinute,stopSecond;   //OFF HH:MM:SS
   growLigh1,growLigh2,growLigh3,growLigh4;
  */
  static int startTimeInSeconds;
  static int stopTimeInSeconds;
  static int currentTimeInSeconds;
  if(currentTimeInSeconds >= startTimeInSeconds && currentTimeInSeconds < stopTimeInSeconds){
   timeonGL = 1;
  }
  else{
    timeonGL = 0;
  }

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
  stopTimeInSeconds = abs((stopHour * 60 * 60) + (stoptMinute * 60) + stopSecond);
  currentTimeInSeconds = abs((nowHour * 60 * 60) + (nowMinute * 60) + nowSecond);

  if(currentTimeInSeconds >= startTimeInSeconds && currentTimeInSeconds < stopTimeInSeconds){
   timeonGL = 1;
  }
  else{
    timeonGL = 0;
  }

  if (currentMillis - lastSaveTimeoAuto >= 120000U || firstAuto == 0) {
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
  
  
}
void controlWaterLevel(){ //switch mode --> lv of water in tank --> Return state of relay
  
}
void relayRtu(int condition){ // manage state of relay on/off ch.
  switch(condition) {
    case 1:
      //sending conmand relay on ec ch
      //relay 50-100
      Serial2.write(ON_RTU1, 8);
      delay(50);
      break;
    case 2:
      //sending conmand relay off ec ch
      //relay 50-100
      Serial2.write(OFF_RTU1, 8);
      delay(50);
      break;
    case 3:
      //sending conmand relay on ph ch
      //relay 50-100
      Serial2.write(ON_RTU2, 8);
      delay(50);
      break;
    case 4:
      //sending conmand relay off ph ch
      //relay 50-100
      Serial2.write(OFF_RTU2, 8);
      delay(50);
      break;

    case 5:
      //sending conmand relay off pump
      //relay 50-100
      Serial2.write(ON_RTU3, 8);
      delay(50);
      break;
    case 6:
      //sending conmand relay off  pump
      //relay 50-100
      Serial2.write(OFF_RTU3, 8);
      delay(50);
      break;

    case 7:
      //sending conmand relay on gl1
      //relay 50-100
      Serial2.write(ON_RTU16, 8);
      delay(50);
      break;
    case 8:
      //sending conmand relay off  gl1
      //relay 50-100
      Serial2.write(OFF_RTU16, 8);
      delay(50);
      break;
    case 9:
      //sending conmand relay on gl2
      //relay 50-100
      Serial2.write(ON_RTU15, 8);
      delay(50);
      break;
    case 10:
      //sending conmand relay off  gl2
      //relay 50-100
      Serial2.write(OFF_RTU15, 8);
      delay(50);
      break;
    case 11:
      //sending conmand relay on gl3
      //relay 50-100
      Serial2.write(ON_RTU14, 8);
      delay(50);
      break;
    case 12:
      //sending conmand relay off  gl3
      //relay 50-100
      Serial2.write(OFF_RTU14, 8);
      delay(50);
      break;
    case 13:
      //sending conmand relay on gl4
      //relay 50-100
      Serial2.write(ON_RTU13, 8);
      delay(50);
      break;
    case 14:
      //sending conmand relay off  g14
      //relay 50-100
      Serial2.write(OFF_RTU13, 8);
      delay(50);
      break;
  }
}
void requestTime(){
  Blynk.sendInternal("rtc","sync"); //using when want to get time clock
}
BLYNK_WRITE(InternalPinRTC){
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013
  unsigned long blynkTime = param.asLong();
  if (blynkTime >= DEFAULT_TIME) 
  {
    setTime(blynkTime);
    Serial.println(blynkTime);
    Serial.println(String("RTC Server: ") + hour() + ":" + minute() + ":" + second());
    Serial.println(String("Day of Week: ") + weekday()); 
  }
}
