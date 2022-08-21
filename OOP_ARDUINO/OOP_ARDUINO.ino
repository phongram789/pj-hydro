#include "sw.h"
#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <PubSubClient.h>

BlynkTimer timer;
/*--------------------------------------------------*/
int ledPin = D4;  // GPIO2 = D4
const char* subscribe_topic = "@msg/temp";

const char* mqtt_server = "broker.netpie.io";
const int mqtt_port = 1883;
const char* mqtt_client = "aa47cd89-19f0-4db3-a3df-b823fb50b939";
const char* mqtt_username = "iWrjeyzumGdQGZM8pSEobjA3cPCUpciE";
const char* mqtt_password = "n6htDnjn7rLq8_epUM)N-M076iMWy4t4";
/*--------------------------------------------------*/
char ssid[] = "ooy";
char pass[] = "0863447295";
char auth[] = "Yzir-DlYEFG7DKrYAlfz-7bgzmIO_P02";
swInput swAuto(D0);

/*--------------------------------------------------*/
WiFiClient espClient;
PubSubClient client(espClient);
/*--------------------------------------------------*/
void setup() {
  Serial.begin(115200);
  Blynk.begin(auth, ssid, pass, "blynk2.iot-cm.com", 8080); 
  
  Serial.print("Connecting to mqtt broker wifi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
}

void loop() {

  if (Blynk.connected()) {
    Blynk.run();
  }
  timer.run();
  
  if (!client.connected()) {
    reconnect();
  }
  int val = swAuto.get_status();
  Blynk.virtualWrite(V0, val);
  client.loop();
  delay(100);

}
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
      digitalWrite(ledPin, LOW);
      Serial.println("Turn on LED");
      Blynk.virtualWrite(V1, msg);
    } else {
      digitalWrite(ledPin, HIGH);
      Serial.println("Turn off LED");
      Blynk.virtualWrite(V1, msg);
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection..");
    if (client.connect(mqtt_client, mqtt_username, mqtt_password)) {
      Serial.println("Connected");
      client.subscribe(subscribe_topic);
    }
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println("Try again in 5 seconds...");
      delay(5000);
    }
  }
}
