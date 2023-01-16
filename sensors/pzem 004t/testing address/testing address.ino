#include <PZEM004Tv30.h>

PZEM004Tv30 pzem1(16,17,);
PZEM004Tv30 pzem2(16,17,0x08);
PZEM004Tv30 pzem3(16,17,0x09);

void setup() {
  Serial.begin(115200);
}

void loop() {

    Serial.println(pzem1.voltage());
    Serial.println(pzem2.voltage());
    Serial.println(pzem3.voltage());
    Serial.println();
    delay(1000);
}