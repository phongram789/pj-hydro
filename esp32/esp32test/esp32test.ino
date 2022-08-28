#include "sw.h"
#define BLYNK_PRINT Serial
#include <BlynkSimpleEsp32.h>
#include <PubSubClient.h>

BlynkTimer timer;
WiFiClient espClient;
PubSubClient client(espClient);

char ssid[] = "ooy";
char pass[] = "0863447295";
//char auth[] = "tsLOdm66zqk7XRhY8RGD3Xmx-vZOiC8d";

char auth[] = "EBA5sMz-RAWzDwaRsFprgVxE8KKzARtA";

const char* mqtt_server = "broker.netpie.io";
const int mqtt_port = 1883;
const char* mqtt_client = "aa47cd89-19f0-4db3-a3df-b823fb50b939";
const char* mqtt_username = "iWrjeyzumGdQGZM8pSEobjA3cPCUpciE";
const char* mqtt_password = "n6htDnjn7rLq8_epUM)N-M076iMWy4t4";
const char* subscribe_topic = "@msg/temp";

swInput swAuto(25);



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
  //pinMode(TdsSensorPin,INPUT);

  

}

void loop() {
  // put your main code here, to run repeatedly:
  if (Blynk.connected()) {
    Blynk.run();
  }
  timer.run();
  int test = random(300);
  Blynk.virtualWrite(V0, test);
  int val = swAuto.get_status();
  //Serial.print(switch: ");
  //Serial.println(val);
  Blynk.virtualWrite(V1, val);
  mqttloop();
  //tdsloop();
  delay(200);
  
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
      //digitalWrite(ledPin, LOW);
      Serial.println("Turn on LED");
    } else {
      //digitalWrite(ledPin, HIGH);
      Serial.println("Turn off LED");
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
