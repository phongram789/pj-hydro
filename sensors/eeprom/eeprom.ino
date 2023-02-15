#include <EEPROM.h>

int startHour =23;
int startMinute =50;
int startSecond =40;
int stopHour =20;
int stopMinute = 20;
int stopSecond = 50;
float phLow = 5.5,phHigh = 7.5 ,ecLow = 1.3,ecHigh = 1.7;

bool growLigh1 =1;
bool growLigh2 =0;
bool growLigh3 =1;
bool growLigh4 = 1;
#define EEPROM_SIZE 64

int startHour1;
int startMinute1;
int startSecond1;
int stopHour1;
int stopMinute1;
int stopSecond1;
float phLow1,phHigh1,ecLow1,ecHigh1;
bool growLigh11;
bool growLigh21;
bool growLigh31;
bool growLigh41;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
   if (!EEPROM.begin(EEPROM_SIZE))
  {
    Serial.println("failed to initialise EEPROM");
  }


  Serial.println("Writing...");
  delay(500);
  int address = 0;

  /*EEPROM.write(0, startHour);  
  EEPROM.write(sizeof(startHour), startMinute);
  EEPROM.write(sizeof(startHour)+sizeof(startMinute), startSecond);*/
  EEPROM.write(address, startHour);
  address += sizeof(startHour);
  EEPROM.write(address, startMinute);
  address += sizeof(startMinute);
  EEPROM.write(address, startSecond);
  address += sizeof(startSecond);
  EEPROM.write(address, stopHour);
  address += sizeof(stopHour);
  EEPROM.write(address, stopMinute);
  address += sizeof(stopMinute);
  EEPROM.write(address, stopSecond);
  address += sizeof(stopSecond);
  EEPROM.put(address, phLow);
  address += sizeof(phLow);
  EEPROM.put(address, phHigh);
  address += sizeof(phHigh);
  EEPROM.put(address, ecLow);
  address += sizeof(ecLow);
  EEPROM.put(address, ecHigh);
  address += sizeof(ecHigh);
  EEPROM.write(address, growLigh1);
  address += sizeof(growLigh1);
  EEPROM.write(address, growLigh2);
  address += sizeof(growLigh2);
  EEPROM.write(address, growLigh3);
  address += sizeof(growLigh3);
  EEPROM.write(address, growLigh4);
  EEPROM.commit();
  //EEPROM.end();



  Serial.println("Reading...");
  address = 0;
  startHour1 = EEPROM.read(address);
  address += sizeof(startHour);
  startMinute1 = EEPROM.read(address);
  address += sizeof(startMinute);
  startSecond1 = EEPROM.read(address);
  address += sizeof(startSecond);
  stopHour1 = EEPROM.read(address);
  address += sizeof(stopHour);
  stopMinute1 = EEPROM.read(address);
  address += sizeof(stopMinute);
  stopSecond1 = EEPROM.read(address);
  address += sizeof(stopSecond);
  EEPROM.get(address, phLow1);
  address += sizeof(phLow);
  EEPROM.get(address, phHigh1);
  address += sizeof(phHigh);
  EEPROM.get(address, ecLow1);
  address += sizeof(ecLow);
  EEPROM.get(address, ecHigh1);
  address += sizeof(ecHigh);
  growLigh11 = EEPROM.read(address);
  address += sizeof(growLigh1);
  growLigh21 = EEPROM.read(address);
  address += sizeof(growLigh2);
  growLigh31 = EEPROM.read(address);
  address += sizeof(growLigh3);
  growLigh41 = EEPROM.read(address);
  EEPROM.end();


Serial.print("startHour: ");
Serial.println(startHour1);
Serial.print("startMinute: ");
Serial.println(startMinute1);
Serial.print("startSecond: ");
Serial.println(startSecond1);
Serial.print("stopHour: ");
Serial.println(stopHour1);
Serial.print("stopMinute: ");
Serial.println(stopMinute1);
Serial.print("stopSecond: ");
Serial.println(stopSecond1);
Serial.print("phLow: ");
Serial.println(phLow1);
Serial.print("phHigh: ");
Serial.println(phHigh1);
Serial.print("ecLow: ");
Serial.println(ecLow1);
Serial.print("ecHigh: ");
Serial.println(ecHigh1);
Serial.print("growLigh1: ");
Serial.println(growLigh11);
Serial.print("growLigh2: ");
Serial.println(growLigh21);
Serial.print("growLigh3: ");
Serial.println(growLigh31);
Serial.print("growLigh4: ");
Serial.println(growLigh41);





}

void loop() {
  // put your main code here, to run repeatedly:

}
