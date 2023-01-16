#include <ModbusMaster.h>
ModbusMaster node;
uint16_t const REG_EC = 2 ; // 0001H EC/TDS value
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  double EC = Read_RTUDevice(1, REG_EC);
  Serial.println("EC: ");
  Serial.println(EC);
  delay(3000);

}

uint32_t Read_RTUDevice(char addr, uint16_t REG){
  uint32_t i = 0;
  uint8_t j, result;
  uint16_t data[2];
  uint32_t Value = 0;

  node.begin(addr,Serial);
  result = node.readHoldingRegisters(REG,1); // Modbus func 0x03 read Holding Registers
  Serial.print("result ");  
  Serial.println(result);  
  delay(20);
  if(result == node.ku8MBSuccess){
    data[j] = node.getResponseBuffer(j);
    Value = data[0];
    i = Value;
    return i;
  }
  else{
    Serial.print("ID = ");
    Serial.println(addr);
    Serial.print("Modbus fail. REG >>> "); 
    Serial.println(REG, DEC);
    delay(10);
    return 0;
  }
  
};