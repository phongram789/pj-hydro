
#include <PZEM004Tv30.h>
#include <ModbusMaster.h>
ModbusMaster node;
uint16_t valueSW1;
/* Hardware Serial2 is only available on certain boards.
 * For example the Arduino MEGA 2560
*/

PZEM004Tv30 pzem(Serial2, 16, 17);

void setup() {
    Serial.begin(115200);
    Serial1.begin(9600, SERIAL_8N1, 16, 17);
    
    node.begin(1, Serial1);

    // Uncomment in order to reset the internal energy counter
    // pzem.resetEnergy()
}

void loop() {

  valueSW1 = random(1,3);
  test1();
        
    Serial.print("Custom Address:");
    Serial.println(pzem.readAddress(), HEX);

    // Read the data from the sensor
    float voltage = pzem.voltage();
    float current = pzem.current();
    float power = pzem.power();
    float energy = pzem.energy();
    float frequency = pzem.frequency();
    float pf = pzem.pf();

    // Check if the data is valid
    if(isnan(voltage)){
        Serial.println("Error reading voltage");
    } else if (isnan(current)) {
        Serial.println("Error reading current");
    } else if (isnan(power)) {
        Serial.println("Error reading power");
    } else if (isnan(energy)) {
        Serial.println("Error reading energy");
    } else if (isnan(frequency)) {
        Serial.println("Error reading frequency");
    } else if (isnan(pf)) {
        Serial.println("Error reading power factor");
    } else {

        // Print the values to the Serial console
        Serial.print("Voltage: ");      Serial.print(voltage);      Serial.println("V");
        Serial.print("Current: ");      Serial.print(current);      Serial.println("A");
        Serial.print("Power: ");        Serial.print(power);        Serial.println("W");
        Serial.print("Energy: ");       Serial.print(energy,3);     Serial.println("kWh");
        Serial.print("Frequency: ");    Serial.print(frequency, 1); Serial.println("Hz");
        Serial.print("PF: ");           Serial.println(pf);

    }

    Serial.println();
    delay(2000);
}

void test1(){
  if(valueSW1 == 1){
    node.writeSingleRegister(1, valueSW1);
  } else {
    node.writeSingleRegister(1, valueSW1);
  }
}