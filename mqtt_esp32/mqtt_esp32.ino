#include<WiFi.h>
#include<PubSubClient.h>
//#include <Wire.h>
#include "sw.h"
#include "pinMange.h"
#include "status.h"
#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <BlynkSimpleEsp32.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#define MQTT_SERVER   "192.168.1.51"
#define MQTT_PORT     1883
#define MQTT_USERNAME "esp"      
#define MQTT_PASSWORD "esp"    
#define MQTT_NAME     "esp"

DHT dht(DHTPIN,DHT22);
WiFiClient client;
PubSubClient mqtt(client);
LiquidCrystal_I2C lcd(0x27, 20, 4);

const char* ssid ="ooy2G";
const char* password =  "0863447295";


OneWire oneWire(oneWireBus); // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature DS18B20(&oneWire); // Pass our oneWire reference to Dallas Temperature sensor 


float humidity;
float temperature;
float temperatureC;//Dallas

float voltagePH,phValue;
float acidVoltage = 1810;
float neutralVoltage = 1370;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  WiFi.begin(ssid, password);
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
}

void loop() {
  // put your main code here, to run repeatedly:
  
  DHT();
  swTopfloat.get_status();      //get status of water level on top
  swButtomfloat.get_status();   //get status of water level on button
  
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
  
  if (mqtt.connected() == false) {
    Serial.print("MQTT connection... ");
    if (mqtt.connect(MQTT_NAME, MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.println("connected");
      //mqtt.subscribe("/ESP32_1/LED");
    } else {
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
