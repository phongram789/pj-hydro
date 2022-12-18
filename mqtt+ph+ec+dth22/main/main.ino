#include <WiFi.h>
#include <PubSubClient.h>
#include "tds.h"
#include "DHT.h"
#include "pin.h"
#include "flow.h"
#define DHTTYPE DHT22 
TDS tds(ECPIN);
FLOW flowA(FLOW_P);
WiFiClient client;
PubSubClient mqtt(client);
DHT dht(DHTPIN, DHTTYPE);
const char* ssid = "ooy2G";
const char* password = "0863447295";
const char* mqtt_server = "broker.netpie.io";
const int mqtt_port = 1883;
const char* mqtt_client = "aa47cd89-19f0-4db3-a3df-b823fb50b939";
const char* mqtt_username = "iWrjeyzumGdQGZM8pSEobjA3cPCUpciE";
const char* mqtt_password = "n6htDnjn7rLq8_epUM)N-M076iMWy4t4";

//-----------Time current-------------
unsigned long currentMillis = 0; //for function millis() main current time for board
unsigned long TIME_FLOW = 0;
const long interval = 1000;

float h,t; //humidity and temperature
//-------------pH-----------------
float voltagePH,phValue;
float acidVoltage = 1810;
float neutralVoltage = 1370;
//--------------------------------
float ecValue,TdsValue;

void IRAM_ATTR pulseCounterFlowA(){
  flowA.pulseCounter();
}

void setup() {
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(flowA.t_pin), pulseCounterFlowA, FALLING);
  initWiFi(); // function connect WiFi
  mqtt.setServer(mqtt_server, mqtt_port);
  dht.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  currentMillis = millis();
  flow();
  readDHT();
  PH();
  Ec();
  Mqttreconnect();
}
void flow(){
  if (currentMillis - TIME_FLOW > interval) { // currentMillis - TIME_FLOW > interval
    TIME_FLOW = flowA.previousMillis;
    flowA.readFlowrate();
  }
}
void Ec(){
  static unsigned long timepoint = millis();
  if(currentMillis - timepoint >= 2000U){
     timepoint = currentMillis;
     tds.calTDS();
     ecValue = tds.getEC()*0.001;
   }
   Serial.print("EC: ");
   Serial.print(ecValue);
   Serial.println(" ms/cm");
}
void Mqttreconnect(){
  static unsigned long timepoint = millis();
  if(currentMillis - timepoint >= 2000U){
    timepoint = currentMillis;
    if (mqtt.connected() == false) {
      Serial.print("MQTT connection... ");
      if (mqtt.connect(mqtt_client, mqtt_username, mqtt_password)){
        Serial.println("connected");
      } 
      else{
        Serial.println("failed");
        delay(5000);
      }
    }
    else {
      mqtt.loop();
      String dataJS = "{\"temp\":" + String(t) + ",\"hum\":" +String(h) + ",\"ec\":" +String(ecValue) + ",\"ph\":" +String(phValue) + "}";
      char json[100];
      dataJS.toCharArray(json,dataJS.length()+1);
      mqtt.publish("@msg/v1/devices/me/telemetry", json);
    }
  }
};
void initWiFi() { 
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
};
void readDHT(){
  static unsigned long timepoint = millis();
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
  static unsigned long timepoint = millis();
  if(currentMillis - timepoint >= 1000U){
    timepoint = currentMillis;
    float analogValue = analogRead(PHPIN);
    /*Serial.print("analogValue from pH");
    Serial.println(analogValue);*/
    voltagePH = analogValue/4095.0*3300; //read the voltage  
    /*Serial.print("voltage from pH");
    Serial.println(voltagePH);*/
    float slope = (7.0-4.0)/((neutralVoltage-1500)/3.0 - (acidVoltage-1500)/3.0); //two point: (_NautralVoltage,7.0),(_acidVoltage,4.0)
    float intercept = 7.0 - slope*(neutralVoltage-1500)/3.0;
    phValue = slope*(voltagePH-1500)/3.0+intercept; //y = k*x +b
  }
  Serial.print("pH: ");
  Serial.println(phValue);
};
