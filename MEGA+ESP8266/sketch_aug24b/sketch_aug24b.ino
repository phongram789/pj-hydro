#include "sw.h"

//  LIBRARY-Blynk
//====================================================
#define BLYNK_PRINT Serial
#include <ESP8266_Lib.h> // insert this library 
#include <BlynkSimpleShieldEsp8266.h>
BlynkTimer timer;
//====================================================

//Pubsub
//====================================================
#include <PubSubClient.h>
const char* subscribe_topic = "@msg/temp";
const char* mqtt_server = "broker.netpie.io";
const int mqtt_port = 1883;
const char* mqtt_client = "1ce9b0500726";
const char* mqtt_username = "C9uFEmJ64biA";
const char* mqtt_password = "KOsDZz926mi8";

//====================================================

//  Pin
//====================================================
swInput swAuto(53);
//====================================================
//  Initialitation
//====================================================
char ssid[] = "ooy";
char pass[] = "0863447295";
char auth[] = "EBA5sMz-RAWzDwaRsFprgVxE8KKzARtA";
#define EspSerial Serial3
#define ESP8266_BAUD 115200
ESP8266 wifi(&EspSerial);

//====================================================


void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);
  delay(10);
  EspSerial.begin(ESP8266_BAUD);
  delay(10);
  Blynk.begin(auth, wifi, ssid, pass,"blynk2.iot-cm.com", 8080);  //Local server

  
}

void loop() {
  if (Blynk.connected()) {
    Blynk.run();
  }
  timer.run();



  
  int val = swAuto.get_status();
  Blynk.virtualWrite(V0, val);
  

  
  espSerial(); //last line

}
