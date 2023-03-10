// ขา 0 , 2 ห้ามเป็น low
//pzemRead และ waterRs485 และ RTC จะทำให้ ปู่ม pin 0 debouce มีปัญหา
/*

          Work list to do 
          function to check flowing of A B n Acic
          function draning water and fill tank
          function pumping went AB AB n Acid solutions r pumpping 
          function count water wnet fill tank n calculate bill of water
          function updatre version of smooth to read ec and ph

          function save all value should be save in eeprom


*/
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <BlynkSimpleEsp32.h>
#include <TimeLib.h>
#include <EEPROM.h>
#include <PZEM004Tv30.h>
#include <Wire.h>
#include "ModbusMaster.h"
#include "DS3231.h"
#include <LiquidCrystal_I2C.h>
#include <Bounce2.h>
#include "rtu.h"
#include "INPUT_PULLUP.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include "DHT.h"
#include "tds.h"
#define PHPIN 39
#define ECPIN 36
#define DHTPIN 33
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
TDS tds(ECPIN);
ModbusMaster node;

//-------------pH-----------------
float voltagePH,phValue;
float acidVoltage = 1810;
float neutralVoltage = 1370;
const int smoothFactor = 10;
float lastPH;

//--------------------------------
float ecValue,TdsValue,lastEC;


#define ONE_WIRE_BUS 26 //กำหนดขาที่จะเชื่อมต่อ Sensor
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensorsWatertemp(&oneWire);

LiquidCrystal_I2C lcd(0x27, 20, 4);
RTClib RTC;
DS3231 Clock;

PZEM004Tv30 pzem(Serial2, 16, 17,0x07);

WiFiClient espClient;
PubSubClient mqtt(espClient);
BlynkTimer timer;
#define BLYNK_PRINT Serial

#define EEPROM_SIZE 64
const char* ssid = "ooy2G";
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
float temperatureC,h,t;


bool Rtcmodule; // 0 = error, 1 = good 
int nowHour,nowMinute,nowSecond;       //RTC HH:MM:SS
int startHour,startMinute,startSecond; //ON HH:MM:SS 
int stopHour,stopMinute,stopSecond;   //OFF HH:MM:SS
int startTimeInSeconds;
int stopTimeInSeconds;
int currentTimeInSeconds;

bool growLigh1,growLigh2,growLigh3,growLigh4; //status on working of grow light

bool timeonGL; //0 == off , 1 == on //value of on/off in time clock

bool controlPumpEc,controlPumpPH; //void Control 

float phLow,phHigh,ecLow,ecHigh; // first read input from eeprom and then already read from blynk n mqtt and then save in eeprom
bool relayStatePH;

bool statusBlynk,statusMqtt,statusWifi; 

float voltage ,current ,power ,energy , frequency, pf; // values from energy module
float ft = NULL;// input from blynk
float Unit,energyprice;

bool ecAuto, ecMan, phAuto, phMan ,GLAuto ,GLMan; //switch mode 

bool swEcModeOff,swEcModeMan,swEcModeAuto;  //switch mode

bool GrowLight1Control1,GrowLight1Control2,GrowLight1Control3,GrowLight1Control4; // read input from blynk and mtqq 0 = off, 1 = on --> save eefrom repeate

bool updateGl = 0;
bool updatePumpMain = 0;
bool flowwing = 1; // water in main line plant is flowing is good

#define pinSwitchEcAuto 5
#define pinSwitchEcMan 18

#define pinSwitchPHAuto 4
#define pinSwitchPHMan 15

#define pinSwitchGrowlightAuto 19
#define pinSwitchGrowlightMan 23

#define pinSwitchtestman 3

#define buttonPinMainWaterPump  0 // 15 สลับกับขา 0 เพราะ connected to on-board LED, must be left floating or LOW to enter flashing mode

bool mainWaterPump = HIGH;
bool mainWaterPump2 = HIGH;
Bounce debouncer = Bounce(); // create obj

swInput swAutoEC(pinSwitchEcAuto);
swInput swManEC(pinSwitchEcMan);
swInput swAutoPH(pinSwitchPHAuto);
swInput swManPH(pinSwitchPHMan);
swInput swAutoGL(pinSwitchGrowlightAuto);
swInput swManGL(pinSwitchGrowlightMan);
swInput swManWaterIn(pinSwitchtestman);

#define pintestlevel 35//25
swInput swtest(pintestlevel);
#define pintestlevel2 25
swInput swtest2(pintestlevel2);

WidgetLCD lcdBlynk(V16);

//---------------------test station-----------------------
#define WaterFowPinMainLine 27
int pulseCounterMainLine;
#define WaterFowPinMainLine2 14
int pulseCounterMainLine2;

#define WaterFowPinMainLine3 34
int pulseCounterMainLine3;

#define WaterFowPinMainLine4 32
int pulseCounterMainLine4;

#define WaterFowPinMainLine5 13
int pulseCounterMainLine5;

void IRAM_ATTR pulseCounter()
{
  pulseCounterMainLine++;
}

void IRAM_ATTR pulseCounter2()
{
  pulseCounterMainLine2++;
}

void IRAM_ATTR pulseCounter3()
{
  pulseCounterMainLine3++;
}
void IRAM_ATTR pulseCounter4()
{
  pulseCounterMainLine4++;
}
void IRAM_ATTR pulseCounter5()
{
  pulseCounterMainLine5++;
}

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
  timer.setInterval(10000L, lcdBlynkPrint);
  timer.setInterval(2000L, readWaterTemp);
  timer.setInterval(5000L, waterRs485);
  //timer.setInterval(10L, testprint);

  
  sensorsWatertemp.begin();

  // rs485
  //Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  while(!Serial2);
  node.begin(1, Serial2); // Id slave is 1 

  //--------------push button-----------------
  debouncer.attach(buttonPinMainWaterPump, INPUT_PULLUP);
  debouncer.interval(20); // กำหนดเวลาการเปลี่ยนสถานะให้กับ debouncer object ที่ 20 มิลลิวินาที

  //------------flowing-----------------------
  pinMode(WaterFowPinMainLine, INPUT_PULLUP);
  pinMode(WaterFowPinMainLine2, INPUT_PULLUP);
  pinMode(WaterFowPinMainLine3, INPUT_PULLUP);
  pinMode(WaterFowPinMainLine4, INPUT_PULLUP);
  pinMode(WaterFowPinMainLine5, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(WaterFowPinMainLine), pulseCounter, FALLING);
  attachInterrupt(digitalPinToInterrupt(WaterFowPinMainLine2), pulseCounter2, FALLING);
  attachInterrupt(digitalPinToInterrupt(WaterFowPinMainLine3), pulseCounter3, FALLING);
  attachInterrupt(digitalPinToInterrupt(WaterFowPinMainLine4), pulseCounter4, FALLING);
  attachInterrupt(digitalPinToInterrupt(WaterFowPinMainLine5), pulseCounter5, FALLING);
  dht.begin();
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
  readDHT();
  PH();
  Ec();
}

void testprint(){
    Serial.println("digitalRead(buttonPinMainWaterPump): " + String(digitalRead(buttonPinMainWaterPump)));
}

void sendSensor(){ //to blynk
  //float voltage ,current ,power ,energy , frequency, pf;
  //Blynk.virtualWrite(V15, pf);

  //test
  //readWaterTemp();
  

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
    updateGl = 0;
  }
  if (String(topic) == "@msg/kmutnb/cs/smarthydroponic1/gl2") { 
    if (msg == "1"){
      GrowLight1Control2 = 1;
    } else {
      GrowLight1Control2 = 0;
    }
    updateGl = 0;
  }
  if (String(topic) == "@msg/kmutnb/cs/smarthydroponic1/gl3") { 
    if (msg == "1"){
      GrowLight1Control3 = 1;
    } else {
      GrowLight1Control3 = 0;
    }
    updateGl = 0;
    
  }
  if (String(topic) == "@msg/kmutnb/cs/smarthydroponic1/gl4") { 
    if (msg == "1"){
      GrowLight1Control4 = 1;
    } else {
      GrowLight1Control4 = 0;
    }
    updateGl = 0;
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
  //pulseCounterMainLine
  //Serial.println( );
  /*Serial.println("digitalRead(buttonPinMainWaterPump): " + String(digitalRead(buttonPinMainWaterPump)));
  Serial.println("mainWaterPump: " + String(mainWaterPump));
  Serial.println("mainWaterPump2: " + String(mainWaterPump2));*/
  /*Serial.println( "pulseCounterMainLine " + String(pulseCounterMainLine));
  Serial.println( "pulseCounterMainLine2 " + String(pulseCounterMainLine2));
  Serial.println( "pulseCounterMainLine3 " + String(pulseCounterMainLine3));
  Serial.println( "pulseCounterMainLine4 " + String(pulseCounterMainLine4));
  Serial.println( "pulseCounterMainLine5 " + String(pulseCounterMainLine5));*/

 // Serial.println("level " + String(swtest.get_status()));
 // Serial.println("level2 " + String(swtest2.get_status()));

 /* Serial.print("EC: ");
  Serial.print(lastEC);
  Serial.println(" ms/cm");
  Serial.print("pH: ");
  Serial.println(lastPH);*/
  /*Serial.print("GLAuto: ");
      Serial.println(GLAuto);
      Serial.print("GLMan: ");
      Serial.println(GLMan);*/

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
    if (currentMillis - lastSaveTime >= 10000U) {
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
          Serial.println("Error reading voltage: "+ String(countError));
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
            
            Unit = (energy/1000);
            if(ft == NULL){
              ft = 93.49;
            }
            energyprice = calEnergyPrice(Unit,ft);
            Blynk.virtualWrite(V21, energyprice);
            Blynk.virtualWrite(V23, Unit);
            Blynk.virtualWrite(V5, voltage);
            Blynk.virtualWrite(V6, current);
            Blynk.virtualWrite(V7, power);
            Blynk.virtualWrite(V8, energy);
            Blynk.virtualWrite(V9, frequency);

            // Print the values to the Serial console
            /*Serial.print("Voltage: ");      Serial.print(voltage);      Serial.println("V");
            Serial.print("Current: ");      Serial.print(current);      Serial.println("A");
            Serial.print("Power: ");        Serial.print(power);        Serial.println("W");
            Serial.print("Energy: ");       Serial.print(energy,3);     Serial.println("kWh");
            Serial.print("Frequency: ");    Serial.print(frequency, 1); Serial.println("Hz");
            Serial.print("PF: ");           Serial.println(pf);
            Serial.println();*/
        }
        if (countError > 200){
          lcdBlynkPrintError("Module energy error");
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
    //state of grow light
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

  // Select switch
  if((GLAuto == 1)&&(GLMan == 1)){ // sw grow light Mode off
    updateGl = 0;
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

  if (currentMillis - lastSaveTimeoAuto >= 120000U || updateGl == 0) {
    lastSaveTimeoAuto = currentMillis;
    updateGl = 1;
    if(timeonGL == 1 && GrowLight1Control1 == 1){
      relayRtu(7);
      growLigh1 = HIGH;
      //You can change button labels from hardware with
      Blynk.virtualWrite(V10, growLigh1);
    }else{
      relayRtu(8);
      growLigh1 = LOW;
      Blynk.virtualWrite(V10, growLigh1);
    }
    if(timeonGL == 1 && GrowLight1Control2 == 1){
      relayRtu(9);
      growLigh2 = HIGH;
      Blynk.virtualWrite(V11, growLigh2);
    }else{
      relayRtu(10);
      growLigh2 = LOW;
      Blynk.virtualWrite(V11, growLigh2);
    }
    if(timeonGL == 1 && GrowLight1Control3 == 1){
      relayRtu(11);
      growLigh3 = HIGH;
      Blynk.virtualWrite(V12, growLigh3);
    }else{
      relayRtu(12);
      growLigh3 = LOW;
      Blynk.virtualWrite(V12, growLigh3);
    }
    if(timeonGL == 1 && GrowLight1Control4 == 1){
      relayRtu(13);
      growLigh4 = HIGH;
      Blynk.virtualWrite(V13, growLigh4);
    }else{
      relayRtu(14);
      growLigh4 = LOW;
      Blynk.virtualWrite(V13, growLigh4);
    }
  }


 

  }
  else if ((GLAuto == 1)&&(GLMan == 0)){ //sw grow light Mode Man
    updateGl = 0;
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
  static unsigned long lastSaveTimeHigh;
  static unsigned long lastSaveTimeLow;
  debouncer.update();

  // กำหนดเงื่อนไขให้โค้ดโปรแกรมในวงเล็บปีกกาทำงานเมื่อสถานะปุ่มกดเปลี่ยนจาก HIGH เป็น LOW โดยเช็คจากฟังก์ชั่น fell()
  // หากต้องการเช็คสถานะจาก LOW เป็น HIGH ให้แทนที่ฟังก์ชั่น fell() ด้วยฟังก์ชั่น rose()
  if ( debouncer.fell() ) { 
    mainWaterPump2  = !mainWaterPump2 ; // สลับสถานะ
    updatePumpMain = 0;
    Serial.println("----------------------");
  }
  
    if (currentMillis - lastSaveTimeHigh >= 60000U  && mainWaterPump == HIGH && flowwing == 1 || flowwing == 1 && updatePumpMain == 0) { // flowwing is status of health from water main flowing is good = 1 , bad = 0
      relayRtu(15);
      mainWaterPump = HIGH;
      lastSaveTimeHigh = currentMillis;
      updatePumpMain = 1;
      Serial.println("mainWaterPump: " + String(mainWaterPump));
    }
    //RTU off
    else if (currentMillis - lastSaveTimeLow >= 60000U  && mainWaterPump == LOW) {
      relayRtu(16);
      mainWaterPump = LOW;
      lastSaveTimeLow = currentMillis;
      updatePumpMain = 1;
    }else{
      //updatePumpMain = 1;
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
  static int countSetError;
  if(countSetError < 1 && Rtcmodule == 0){
    Clock.setYear(Year);
    Clock.setMonth(Month);
    Clock.setDate(Date);
    Clock.setDoW(DoW);
    Clock.setHour(Hour);
    Clock.setMinute(Minute);
    Clock.setSecond(Second);
    countSetError++;
    Serial.println("Time update to RTC module done" );
  }
  
}

//GrowLight1Control1,GrowLight1Control2,GrowLight1Control3,GrowLight1Control4; 
BLYNK_WRITE(V10){
  GrowLight1Control1 = param.asInt();
  Serial.println(GrowLight1Control1);
  updateGl = 0; // update status of auto mode to change to status from dashboard
}
BLYNK_WRITE(V11){
  GrowLight1Control2 = param.asInt();
  Serial.println(GrowLight1Control2);
  updateGl = 0;

}
BLYNK_WRITE(V12){
  GrowLight1Control3 = param.asInt();
  Serial.println(GrowLight1Control3);
  updateGl = 0;
}
BLYNK_WRITE(V13){
  GrowLight1Control4 = param.asInt();
  Serial.println(GrowLight1Control4);
  updateGl = 0;
}
BLYNK_WRITE(V14){
  mainWaterPump = param.asInt();
  //BLYNK_LOG("buttonPinMainWaterPump: %d",buttonPinMainWaterPump);
  Serial.println(mainWaterPump);
  updatePumpMain = 0;

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

BLYNK_WRITE(V22){
  ft = param.asFloat();
  //BLYNK_LOG("phHigh: %d",phHigh);
  Serial.println(ft);
}
void lcdBlynkPrint(){
  //x = position symbol 0 -15 , y = line 0,1
  lcdBlynk.clear();
  const String text = String("RTC:") + nowHour + ":" + nowMinute;
  const String text2 =  String("On Time : ") + startHour + ":" + startMinute;
  const String text3 =  String("Off Time: ") + stopHour + ":" + stopMinute;
  //Serial.println(text);

  lcdBlynk.clear(); //Use it to clear the LCD Widget
  lcdBlynk.print(0, 0, text2); // use: (position X: 0-15, position Y: 0-1, "Message you want to print")
  lcdBlynk.print(0, 1, text3); // use: (position X: 0-15, position Y: 0-1, "Message you want to print")
  //lcdBlynk.print(0, 2, text);
}
void lcdBlynkPrintError(String text){
  //x = position symbol 0 -15 , y = line 0,1
  lcdBlynk.clear();
  lcdBlynk.clear(); //Use it to clear the LCD Widget
  lcdBlynk.print(0, 0, text); // use: (position X: 0-15, position Y: 0-1, "Message you want to print")
}

float calEnergyPrice(float Unit , float ft) {
  float total = 0;
  if (Unit <= 150) {
    float Rate15 = 2.3488;
    float Rate25 = 2.9882;
    float Rate35 = 3.2405;
    float Rate100 = 3.6237;
    float Rate150 = 3.7171;
    if (Unit >= 6) total += _min(Unit, 15) * Rate15;
    if (Unit >= 16) total += _min(Unit - 15, 10) * Rate25;
    if (Unit >= 26) total += _min(Unit - 25, 10) * Rate35;
    if (Unit >= 36) total += _min(Unit - 35, 65) * Rate100;
    if (Unit >= 101) total += _min(Unit - 100, 50) * Rate150;
  } else {
    float Rate150 = 3.2484;
    float Rate400 = 4.2218;
    float RateMore400 = 4.4217;
    total += _min(Unit, 150) * Rate150;
    if (Unit >= 151) total += _min(Unit - 150, 250) * Rate400;
    if (Unit >= 401) total += (Unit - 400) * RateMore400;
  }
  total += Unit * (ft / 100);
  return total;
}
void readWaterTemp(){
  sensorsWatertemp.requestTemperatures();
  temperatureC = sensorsWatertemp.getTempCByIndex(0);
  Blynk.virtualWrite(V24, temperatureC);

}
void readDHT(){
  static unsigned long timepoint = 0;
  if(currentMillis - timepoint >= 2000U){
    timepoint = currentMillis;
    h = dht.readHumidity();
    t = dht.readTemperature();
    if (isnan(h) || isnan(t)) {
      t = 31;
      h = -1;
      //Serial.println(F("Failed to read from DHT sensor!"));
      return;
    }
    Blynk.virtualWrite(V3, h);
    Blynk.virtualWrite(V4, t);
    /*Serial.print(F("Humidity: "));
    Serial.print(h);
    Serial.print(F("%  Temperature: "));
    Serial.print(t);
    Serial.println(F(" C "));*/
   }
};
void PH(){
  static float sum; // variable to store the sum of the readings
  static int count; // variable to store the number of readings
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
      /*Serial.print("pH: ");
      Serial.println(lastPH);*/
      // reset the sum and count for the next set of readings
      sum = 0;
      count = 0;
    }
  }
};
void Ec(){
  static float sum; // variable to store the sum of the readings
  static int count; // variable to store the number of readings
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
      /*Serial.print("EC: ");
      Serial.print(lastEC);
      Serial.println(" ms/cm");*/
     }
  }
}
void waterRs485(){
  int value;
  uint8_t result = node.readHoldingRegisters(0,2); //function 03 
  delay(50);
  if (result == node.ku8MBSuccess) {
    Serial.println("success: ");
    /*value = node.getResponseBuffer(0);
    Serial.println(value);*/
    value = node.getResponseBuffer(1);
    Serial.println(value/100);
  } else {
    Serial.print("error code: ");
    Serial.println(result, HEX);
  }
}
void checkFlow() {
  static unsigned long lastCheckTime = 0;
  static unsigned int lastPulseCount = 0;
  
  // Check if 5 minutes have elapsed since the last check
  if (currentMillis - lastCheckTime >= 300000) {
    // Store the current pulse count
    unsigned int currentPulseCount = pulseCounterMainLine;
    
    // Check if the pulse count has not changed since the last check
    if (currentPulseCount == lastPulseCount && flowwing == 1 && mainWaterPump == 1) {
      // Do something here if there is no flow detected
      // For example, you could turn off a pump or send an alert
      flowwing = 0; 
      lcdBlynkPrintError("checkFlow: Error");
      // Reset the pulse count
      pulseCounterMainLine = 0;
    }
    
    // Store the current time and pulse count for the next check
    lastCheckTime = currentMillis;
    lastPulseCount = currentPulseCount;
  }
}
//---------------------------------------------------------------------------------------------------
void checkFlow2() {
  static unsigned long lastCheckTime = 0;
  static unsigned int lastPulseCount = 0;
  
  // Check if 5 minutes have elapsed since the last check
  if (currentMillis - lastCheckTime >= 300000) {
    // Store the current pulse count
    unsigned int currentPulseCount = pulseCounterMainLine;
    
    // Calculate the flow rate in pulses per second
    float flowRate = (float)(currentPulseCount - lastPulseCount) / ((float)(currentMillis - lastCheckTime) / 1000.0);
    
    // Calculate the maximum flow rate based on the sensor specifications
    float maxFlowRate = 10.0; // Replace with your sensor's maximum flow rate
    
    // Check if the flow rate is greater than 20% of the maximum flow rate
    if (flowRate >= maxFlowRate * 0.2) {
      // Do something here if the flow rate is good
      // For example, you could turn on a pump or reset a timer
      
      // Reset the pulse count
      pulseCounterMainLine = 0;
    }
    
    // Store the current time and pulse count for the next check
    lastCheckTime = currentMillis;
    lastPulseCount = currentPulseCount;
  }
}


