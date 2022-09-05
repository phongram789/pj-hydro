#include <WiFi.h>
#include <WiFiClient.h>
#include "sw.h"
#include "pinManage.h"
#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <BlynkSimpleEsp32.h>
#include <PubSubClient.h>
#define BLYNK_PRINT Serial

DHT dht;
BlynkTimer timer;
WiFiClient espClient;
PubSubClient client(espClient);

char ssid[] = "ooy";
char pass[] = "0863447295";
char auth[] = "tsLOdm66zqk7XRhY8RGD3Xmx-vZOiC8d";
//char auth[] = "EBA5sMz-RAWzDwaRsFprgVxE8KKzARtA"; //mega

const char* mqtt_server = "broker.netpie.io";
const int mqtt_port = 1883;
const char* mqtt_client = "aa47cd89-19f0-4db3-a3df-b823fb50b939";
const char* mqtt_username = "iWrjeyzumGdQGZM8pSEobjA3cPCUpciE";
const char* mqtt_password = "n6htDnjn7rLq8_epUM)N-M076iMWy4t4";


float humidity;
float temperature;

LiquidCrystal_I2C lcd(0x27, 20, 4);// Set the LCD address to 0x27 or 0x3F for a 16 chars and 2 line display

void IRAM_ATTR pulseCounter()
{
  pulseCount++;
}

void setup() {
  // put your setup code here, to run once:
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
  //dht.setup(DHTPIN);
  //lcd.begin();
  //lcd.backlight();
  pinMode(WATERFLOWIN, INPUT_PULLUP);
  pulseCount = 0;
  flowRate = 0.0;
  flowMilliLitres = 0;
  totalMilliLitres = 0;
  previousMillis = 0;

  attachInterrupt(digitalPinToInterrupt(WATERFLOWIN), pulseCounter, FALLING);

  

} 

void loop() {
  // put your main code here, to run repeatedly:
  //DHT();
  if (Blynk.connected()) {
    Blynk.run();
  }
  timer.run();
  int test = random(300);
  Blynk.virtualWrite(V0, test);
  mqttloop();
  delay(200);
  waterflow();
  
};

void DHT()
{
  delay(dht.getMinimumSamplingPeriod());
  humidity = dht.getHumidity();
  temperature = dht.getTemperature();
  Blynk.virtualWrite(V0, humidity);
  Blynk.virtualWrite(V0, temperature);
  //Serial.println(dht.getStatusString()); //status is OK
  //Serial.println(dht.toFahrenheit(temperature), 1);
  delay(1000);//1-2sec
};
