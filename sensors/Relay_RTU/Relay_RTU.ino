#include <ModbusMaster.h>
ModbusMaster node;  //ModbusRTU Relay Slave ID1
#define RX2 16        //RX
#define TX2 17        //TX
uint16_t valueSW1;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RX2, TX2);  //RX2=16,RO ,TX2=17, DI
  node.begin(1, Serial2);
}

void loop() {
  valueSW1 = random(1,3);
  test1();
  delay(500);
  Serial.println(valueSW1);
  Serial.println("end");

}
void test1(){
  if(valueSW1 == 1){
    node.writeSingleRegister(1, valueSW1);
  } else {
    node.writeSingleRegister(1, valueSW1);
  }
}
