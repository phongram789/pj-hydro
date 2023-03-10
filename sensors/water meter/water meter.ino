#include "ModbusMaster.h"

#define RX_PIN 16
#define TX_PIN 17

ModbusMaster node;

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  while(!Serial2);
  node.begin(1, Serial2);
}

void loop() {
  int value;
  uint8_t result = node.readHoldingRegisters(0,2); //function 03 
  if (result == node.ku8MBSuccess) {
    Serial.println("success: ");
    /*value = node.getResponseBuffer(0);
    Serial.println(value);*/
    value = node.getResponseBuffer(1);
    Serial.println(value/100);
  } else {
    Serial.print("error code: ");
    Serial.println(result, HEX);
  }
  delay(1000);
}
