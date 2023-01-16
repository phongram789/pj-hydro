#include <ModbusMaster.h>

#define MODBUS_SERIAL_PORT Serial1  // Modbus RTU relay is connected to Serial1 port on the master
#define MODBUS_ADDRESS 0            // Modbus RTU address of the relay
#define RELAY_REGISTER_ADDRESS 1    // Address of the relay's control register

ModbusMaster node;  // Create a ModbusMaster object

void setup() {
  MODBUS_SERIAL_PORT.begin(9600, SERIAL_8N1, 16, 17);  // Initialize the serial port for Modbus communication
  node.begin(MODBUS_ADDRESS, MODBUS_SERIAL_PORT);  // Start the Modbus communication
}

void loop() {
  // Turn the relay on
  uint16_t value = 1;
  node.writeSingleRegister(RELAY_REGISTER_ADDRESS, value);
  node.writeSingleCoil(0, 8);

  // Wait for 1 second
  delay(1000);

  // Turn the relay off
  value = 2;
  node.writeSingleRegister(RELAY_REGISTER_ADDRESS, value);

  // Wait for 1 second
  delay(1000);
}
