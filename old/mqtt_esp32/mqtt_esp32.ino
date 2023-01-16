#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <BlynkSimpleEsp32.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "sw.h"
#include "pinMange.h"
#include "status.h"
#include "tds.h"
#include "flow.h"

//#include <Wire.h>
#define MQTT_SERVER   "192.168.1.51"
#define MQTT_PORT     1883
#define MQTT_USERNAME "esp"      
#define MQTT_PASSWORD "esp"    
#define MQTT_NAME     "esp"

DHT dht(DHTPIN,DHT22);

FLOW sensorFlowA(waterFlowA);

WiFiClient client;
PubSubClient mqtt(client);
LiquidCrystal_I2C lcd(0x27, 20, 4);

const char* ssid ="ooy2G";
const char* passwd =  "0863447295";

OneWire oneWire(oneWireBus); // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature DS18B20(&oneWire); // Pass our oneWire reference to Dallas Temperature sensor 

float flowAValue,flowBValue,flowPHValue;

float humidity;
float temperature;

//Dallas
float temperatureC;

//pH
float voltagePH,phValue;
float acidVoltage = 1810;
float neutralVoltage = 1370;

//PZEM
int requesstCount = 0;
unsigned long timeOut_PZEM = millis();
uint8_t bufferModbus [25];
float fVoltage,fCurrent,fPower,fEnergy,fFrequency;

int statusWiFi = WL_IDLE_STATUS;

//flow
float flowrateA;

//EC
TDS tds(ECPIN);
float ecValue;
//------------------time interval------------------------------
unsigned long time1 = 0; //for sensor water flow to get flowtare 
#define INTERVAL1 1200

void IRAM_ATTR pulseSensorFlowA(){
  sensorFlowA.pulseCounter();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(9600,SERIAL_8N1,RXD0,TXD0);
  while (!Serial2);
  WiFi.begin(ssid, passwd);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("IP ad :");
  Serial.println(WiFi.localIP());
  
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  //mqtt.setCallback(callback); open when include Callback
  dht.begin();
  DS18B20.begin(); //Dallas Temperature IC Control Library
  lcd.init();
  lcd.backlight();
  attachInterrupt(digitalPinToInterrupt(sensorFlowA.SENSOR), pulseSensorFlowA, FALLING); //https://lastminuteengineers.com/handling-esp32-gpio-interrupts-tutorial/
}

void loop() {
  // put your main code here, to run repeatedly:

  PZEM();
  DHT();
  swTopfloat.get_status();      //get status of water level on top
  swButtomfloat.get_status();   //get status of water level on button

  tds.calTDS();
  ecValue = tds.getEC();
  Serial.print("EC = ");
  Serial.print(ecValue);
  Serial.println("\t");


  if(millis() - time1 > INTERVAL1){
    time1 = millis();
    flowrateA = sensorFlowA.read();
  }
  Serial.print("Flow rate1: ");
  Serial.print(flowrateA);
  Serial.println("L/min");
  
//--------------------------- switch Light --------------------------------------

  if(swAutoLight.get_status()== 1 && swManLight.get_status()== 0){
      statusSwLight = 'A';
  }
  else if(swAutoLight.get_status()== 0 && swManLight.get_status()== 1){
      statusSwLight = 'M';
  }
  else if(swAutoLight.get_status()== 0 && swManLight.get_status()== 0){
      statusSwLight = 'O';
  }
  else{
    Serial.println("Switch Light Error");
    statusSwLight = 'E';
  }
//--------------------------- switch Pump --------------------------------------
  if(swAutoPump.get_status()== 1 && swManPump.get_status()== 0){
      statusSwPump = 'A';
  }
  else if(swAutoPump.get_status()== 0 && swManPump.get_status()== 1){
      statusSwPump = 'M';
  }
  else if(swAutoPump.get_status()== 0 && swManPump.get_status()== 0){
      statusSwPump = 'O';
  }
  else{
    Serial.println("Switch Pump Error");
    statusSwPump = 'E';
  }
  
//--------------------------- switch AB --------------------------------------

  if(swAutoAB.get_status()== 1 && swManAB.get_status()== 0){
      statusSwAB = 'A';
  }
  else if(swAutoAB.get_status()== 0 && swManAB.get_status()== 1){
      statusSwAB = 'M';
  }
  else if(swAutoAB.get_status()== 0 && swManAB.get_status()== 0){
      statusSwAB = 'O';
  }
  else{
    Serial.println("Switch AB Error");
    statusSwAB = 'E';
  }
//--------------------------- switch PH --------------------------------------

  if(swAutoPH.get_status()== 1 && swManPH.get_status()== 0){
      statusSwPH = 'A';
  }
  else if(swAutoPH.get_status()== 0 && swManPH.get_status()== 1){
      statusSwPH = 'M';
  }
  else if(swAutoPH.get_status()== 0 && swManPH.get_status()== 0){
      statusSwPH = 'O';
  }
  else{
    Serial.println("Switch PH Error");
    statusSwPH = 'E';
  }
  
//--------------------------- END --------------------------------------
  
  float volt = random (220.0, 240.0);
  float current = random (1.0, 5.0);

  Serial.print("volt = ");
  Serial.print(volt);
  Serial.print("\tCurrent = ");
  Serial.println(current);

  //------------------IF WIFI LOST CONNECTED---------------
  if(WiFi.status() != WL_CONNECTED){
    statusWiFi = WiFi.status();
    Serial.println("Reconnecting WiFi");
    if(statusWiFi != WL_CONNECTED){
      WiFi.begin(ssid,passwd);
      while(WiFi.status() != WL_CONNECTED){
        delay(500);
        Serial.print(".");
      }
      Serial.println("Connected to AP");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
    }
  }

  if (mqtt.connected() == false) {
    Serial.println(WiFi.status());
    Serial.print("MQTT connection... ");
    if (mqtt.connect(MQTT_NAME, MQTT_USERNAME, MQTT_PASSWORD)) {
      mqttConnected = true;
      Serial.println("connected");
      //mqtt.subscribe("/ESP32_1/LED");
    } else {
      mqttConnected = false;
      Serial.println("failed");
      delay(5000);
    }
  } else {
    mqtt.loop();
    String dataJS = "{\"Voltage\":" + String(volt,3) + ",\"current\":" +String(current, 3) + "}";
    char json[100];
    dataJS.toCharArray(json,dataJS.length()+1);
    mqtt.publish("v1/devices/me/telemetry", json);
  }
  delay(1000);
}

void DHT()
{
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();
  //Blynk.virtualWrite(V2, humidity);
  //Blynk.virtualWrite(V3, temperature);
  if (isnan(humidity) || isnan(temperature) ) {
    Serial.println(F("Failed to read from DHT sensor!"));
    humidity = -1;
    return;
  }
  delay(2000);
};

void PH(){
  static unsigned long timepoint = millis();
  if(millis()-timepoint>1000U){
    voltagePH = analogRead(PHPIN)/4095.0*3300; //read the voltage  
    float slope = (7.0-4.0)/((neutralVoltage-1500)/3.0 - (acidVoltage-1500)/3.0); //two point: (_NautralVoltage,7.0),(_acidVoltage,4.0)
    float intercept = 7.0 - slope*(neutralVoltage-1500)/3.0;
    phValue = slope*(voltagePH-1500)/3.0+intercept; //y = k*x +b
  }
  
};
void DallasTemp(){
  static unsigned long timepoint = millis();
  if(millis()-timepoint>2000){
    DS18B20.requestTemperatures();
    temperatureC = DS18B20.getTempCByIndex(0);
  }
};

void PZEM(){
  if(millis() - timeOut_PZEM > 2222){
    //Request 
    Serial2.write(0xF8);
    Serial2.write(0x04);
    Serial2.write(0x00);
    Serial2.write(0x00);
    Serial2.write(0x0A);
    Serial2.write(0x64);
    Serial2.write(0x64);
    timeOut_PZEM = millis();
  }
  if(Serial2.available()){
    for(int i = 0 ; i < 25 ; i++){
      if(!Serial2.available()){
        Serial.println("data not match");
        break;
      }
      bufferModbus[i] = Serial2.read();
      delay(20);
    }
  }
  uint32_t voltage = (uint32_t)bufferModbus[3] << 8 | (uint32_t)bufferModbus[4];
  uint32_t current = (uint32_t)bufferModbus[5] << 8 | (uint32_t)bufferModbus[6] | (uint32_t)bufferModbus[7] << 24 | (uint32_t)bufferModbus[8] << 16;
  uint32_t power = (uint32_t)bufferModbus[9] << 8 | (uint32_t)bufferModbus[10] | (uint32_t)bufferModbus[11] << 24 | (uint32_t)bufferModbus[12] << 16;
  uint32_t energy = (uint32_t)bufferModbus[13] << 8 | (uint32_t)bufferModbus[14] | (uint32_t)bufferModbus[15] << 24 | (uint32_t)bufferModbus[16] << 16;
  uint32_t frequecy = (uint32_t)bufferModbus[17] << 8 | (uint32_t)bufferModbus[18];

  fVoltage = voltage * 0.1;
  fCurrent = current * 0.001;
  fPower = power * 0.1;
  fEnergy = energy * 0.001;
  fFrequency = frequecy * 0.1;

  Serial.println("------------------------------");
  Serial.println("Voltage = " + String(fVoltage));
  Serial.println("Current = " + String(fCurrent));
  Serial.println("Power   = " + String(fPower));
  Serial.println("Energy  = " + String(fEnergy));
  Serial.println("Freq    = " + String(fFrequency));

  while (Serial2.available())
    {
      Serial2.read();
    }
}
