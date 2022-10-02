#include <WiFi.h>
#include <WiFiClient.h>
#include <Wire.h>
#include "sw.h"
#include "pinManage.h"
#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <BlynkSimpleEsp32.h>
#include <PubSubClient.h>
#define BLYNK_PRINT Serial

DHT dht(DHTPIN,DHT22);
BlynkTimer timer;
WiFiClient espClient;
PubSubClient client(espClient);
LiquidCrystal_I2C lcd(0x27, 20, 4);// Set the LCD address to 0x27 or 0x3F for a 16 chars and 2 line display


char ssid[] = "ooy2G";
char pass[] = "0863447295";
char auth[] = "tsLOdm66zqk7XRhY8RGD3Xmx-vZOiC8d";
//char auth[] = "EBA5sMz-RAWzDwaRsFprgVxE8KKzARtA"; //mega

//const char* mqtt_server = "broker.netpie.io";
//const int mqtt_port = 1883;
//const char* mqtt_client = "aa47cd89-19f0-4db3-a3df-b823fb50b939";
//const char* mqtt_username = "iWrjeyzumGdQGZM8pSEobjA3cPCUpciE";
//const char* mqtt_password = "n6htDnjn7rLq8_epUM)N-M076iMWy4t4";

const char* mqtt_server = "192.168.1.51";
const int mqtt_port = 1883;
const char* mqtt_client = "test69";
const char* mqtt_username = "test69";
const char* mqtt_password = "test69";


float humidity;
float temperature;
String dataPublish;
char msg[100];



void setup() {
  Serial.begin(115200);
  Blynk.begin(auth, ssid, pass, "blynk2.iot-cm.com", 8080); 
  //WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  delay(500);
  pinMode(LIGHT1, OUTPUT);
  digitalWrite(LIGHT1, HIGH);
  pinMode(LIGHT2, OUTPUT);
  digitalWrite(LIGHT2, HIGH);
  pinMode(LIGHT3, OUTPUT);
  digitalWrite(LIGHT3, HIGH);
  pinMode(LIGHT4, OUTPUT);
  digitalWrite(LIGHT4, HIGH);
  pinMode(PUMP,OUTPUT);
  digitalWrite(PUMP,HIGH);
  pinMode(AB,OUTPUT);
  digitalWrite(AB,HIGH);
  pinMode(PH,OUTPUT);
  digitalWrite(PH,HIGH);
  pinMode(VALVE,OUTPUT);
  digitalWrite(VALVE,HIGH);

  dht.begin();
  lcd.init();
  lcd.backlight();

} 

void loop() {
  // put your main code here, to run repeatedly:
  if (Blynk.connected()) {
    Blynk.run();
  }
  timer.run();
  int test = random(300);
  Blynk.virtualWrite(V1, test);
  DHT();
  sentDataMqtt();
  mqttloop();
  
  lcd.setCursor(0, 0);
  lcd.print("Hello, World!");
  delay(1000);
  lcd.clear();
  
};

void DHT()
{
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();
  Blynk.virtualWrite(V2, humidity);
  Blynk.virtualWrite(V3, temperature);
  if (isnan(humidity) || isnan(temperature) ) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  delay(2000);
};

void sentDataMqtt(){
  dataPublish = "{\"Humidity\":"+String(humidity)+",\"Temperature\":"+String(temperature)+"}";
  Serial.println(dataPublish);                       
  dataPublish.toCharArray(msg , (dataPublish.length() + 1)); 
  client.publish("@msg/sensors", msg);
 }
