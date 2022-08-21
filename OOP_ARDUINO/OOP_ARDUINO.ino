#include "sw.h"

swInput swAuto(14);

void setup() {
  Serial.begin(9600);
  
}

void loop() {
  int val = swAuto.get_status();
  Serial.println(val);
  delay(100);

}
