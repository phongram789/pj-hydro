#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <BlynkSimpleEsp32.h>
#include <TimeLib.h>
WiFiClient espClient;
PubSubClient mqtt(espClient);
BlynkTimer timer;
#define BLYNK_PRINT Serial

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
float t = 30.2,h = 100,ecValue = 1.5 ,lastPH = 7.5;
unsigned long currentMillis = 0;

bool statusBlynk,statusMqtt,statusWifi;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
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
  Mqttreconnect();
  timer.run();
  reconnectBlynk();
  statusConnecting();

}

void sendSensor(){
  float h = random(0,30);
  float t = random(0,100);
  Blynk.virtualWrite(V3, h);
  Blynk.virtualWrite(V4, t);
}

void Mqttreconnect(){
  static unsigned long timepoint = 0;
  if(currentMillis - timepoint >= 2000U){
    timepoint = currentMillis;
    if (mqtt.connected() == false) {
      Serial.print("MQTT connection... ");
      if (mqtt.connect(mqtt_client, mqtt_username, mqtt_password)){
        mqtt.subscribe(subscribe_topic);// DONT FORGET
        Serial.println("connected");
      } 
      else{
        Serial.println("failed");
        delay(5000);
      }
    }
    else {
      mqtt.loop();
      statusMqtt = mqtt.connected();
      String dataJS = "{\"temp\":" + String(t) + ",\"hum\":" +String(h) + ",\"ec\":" +String(ecValue) + ",\"ph\":" +String(lastPH) + "}";
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
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(WiFi.localIP());
  statusWifi = WiFi.status();

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
void statusConnecting(){
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
    //Blynk.sendInternal("rtc", "sync"); //request current local time for device
  }
};

long timer_start_set[2] = {0xFFFF, 0xFFFF};
long timer_stop_set[2] = {0xFFFF, 0xFFFF};
unsigned char weekday_set[2];
BLYNK_WRITE(V0) 
{
  unsigned char week_day;
 
  TimeInputParam t(param);

  if (t.hasStartTime() && t.hasStopTime() ) 
  {
    timer_start_set[0] = (t.getStartHour() * 60 * 60) + (t.getStartMinute() * 60) + t.getStartSecond();
    timer_stop_set[0] = (t.getStopHour() * 60 * 60) + (t.getStopMinute() * 60) + t.getStopSecond();
    
    Serial.println(String("Start Time: ") +
                   t.getStartHour() + ":" +
                   t.getStartMinute() + ":" +
                   t.getStartSecond());
                   
    Serial.println(String("Stop Time: ") +
                   t.getStopHour() + ":" +
                   t.getStopMinute() + ":" +
                   t.getStopSecond());
                   
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


