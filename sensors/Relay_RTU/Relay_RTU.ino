#include <ModbusMaster.h>
ModbusMaster node1;   //ModbusRTU Relay Slave ID1
#define RX2 16        //RX
#define TX2 17        //TX
int8_t pool_size1;
void setup() {
  // put your setup code here, to run once:
  Serial2.begin(9600, SERIAL_8N1, RX2, TX2);  //RX2=16,RO ,TX2=17, DI
  node1.begin(1, Serial2);
}

void loop() {
  // put your main code here, to run repeatedly: 
  pool_size1 = node1.writeSingleRegister(0x01, 0x0100); //Delay 0x01 on 0x0100
  delay(1000);
  pool_size1 = node1.writeSingleRegister(0x01, 0x0200); //Delay 0x01 close 0x0200
  delay(1000);

}
