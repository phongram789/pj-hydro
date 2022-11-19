#include "tds.h"
#define ECPIN 36
TDS tds(ECPIN);
float ecValue,TdsValue;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
  tds.calTDS();
  //tds.getkkValue();
  ecValue = tds.getEC();
  TdsValue = tds.getTds();
  Serial.print("EC IS:");
  Serial.print(ecValue);
  Serial.println("us/cm");
  Serial.print("TDS IS: ");
  Serial.print(TdsValue);
  Serial.println(" ppm");
  delay(1000);
  
}
