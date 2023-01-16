#include<ArduinoJson.h>
#include<SoftwareSerial.h>
SoftwareSerial s(5,6);

void setup() {
  s.begin(9600);
  Serial.begin(115200);
}

void loop() {
  StaticJsonBuffer<1000> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["data1"] = 100;
  root["data2"] = 200;

  if(s.available()>0){
    root.printTo(s);
    }
  //Serial.println(root.prettyPrintTo(Serial));
}
