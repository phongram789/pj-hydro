// PZEM-004T
#define RXD2 16
#define TXD2 17
int requestCount = 0;
unsigned long timeOut_PZEM = millis();
uint8_t bufferModbus[25];
float fVoltage;
float fCurrent;
float fPower;
float fEnergy;
float fFrequency;

byte ON_RTU1[8] = {0x00, 0x06, 0x00, 0x01, 0x01, 0x00, 0xD8, 0x4B};
byte OFF_RTU1[8] = {0x00, 0x06, 0x00, 0x01, 0x02, 0x00, 0xD8, 0xBB};

byte ON_RTU2[8] = {0x00, 0x06, 0x00, 0x02, 0x01, 0x00, 0x28, 0x4B};
byte OFF_RTU2[8] = {0x00, 0x06, 0x00, 0x02, 0x02, 0x00, 0x28, 0xBB};

byte ON_RTU3[8] = {0x00, 0x06, 0x00, 0x03, 0x01, 0x00, 0x79, 0x8B};
byte OFF_RTU3[8] = {0x00, 0x06, 0x00, 0x03, 0x02, 0x00, 0x79, 0x7B};

byte ON_RTU4[8] = {0x00, 0x06, 0x00, 0x04, 0x01, 0x00, 0xC8, 0x4A};
byte OFF_RTU4[8] = {0x00, 0x06, 0x00, 0x04, 0x02, 0x00, 0xC8, 0xBA};

byte ON_RTU5[8] = {0x00, 0x06, 0x00, 0x05, 0x01, 0x00, 0x99, 0x8A};
byte OFF_RTU5[8] = {0x00, 0x06, 0x00, 0x05, 0x02, 0x00, 0x99, 0x7A};

byte ON_RTU6[8] = {0x00, 0x06, 0x00, 0x06, 0x01, 0x00, 0x69, 0x8A};
byte OFF_RTU6[8] = {0x00, 0x06, 0x00, 0x06, 0x02, 0x00, 0x69, 0x7A};

byte ON_RTU7[8] = {0x00, 0x06, 0x00, 0x07, 0x01, 0x00, 0x38, 0x4A};
byte OFF_RTU7[8] = {0x00, 0x06, 0x00, 0x07, 0x02, 0x00, 0x38, 0xBA};

byte ON_RTU8[8] = {0x00, 0x06, 0x00, 0x08, 0x01, 0x00, 0x08, 0x49};
byte OFF_RTU8[8] = {0x00, 0x06, 0x00, 0x08, 0x02, 0x00, 0x08, 0xB9};

byte ON_RTU9[8] = {0x00, 0x06, 0x00, 0x09, 0x01, 0x00, 0x59, 0x89};
byte OFF_RTU9[8] = {0x00, 0x06, 0x00, 0x09, 0x02, 0x00, 0x59, 0x79};

byte ON_RTU10[8] = {0x00, 0x06, 0x00, 0x0A, 0x01, 0x00, 0xA9, 0x89};
byte OFF_RTU10[8] = {0x00, 0x06, 0x00, 0x0A, 0x02, 0x00, 0xA9, 0x79};

byte ON_RTU11[8] = {0x00, 0x06, 0x00, 0x0B, 0x01, 0x00, 0xF8, 0x49};
byte OFF_RTU11[8] = {0x00, 0x06, 0x00, 0x0B, 0x02, 0x00, 0xF8, 0xB9};

byte ON_RTU12[8] = {0x00, 0x06, 0x00, 0x0C, 0x01, 0x00, 0x49, 0x88};
byte OFF_RTU12[8] = {0x00, 0x06, 0x00, 0x0C, 0x02, 0x00, 0x49, 0x78};

byte ON_RTU13[8] = {0x00, 0x06, 0x00, 0x0D, 0x01, 0x00, 0x18, 0x48};
byte OFF_RTU13[8] = {0x00, 0x06, 0x00, 0x0D, 0x02, 0x00, 0x18, 0xB8};

byte ON_RTU14[8] = {0x00, 0x06, 0x00, 0x0E, 0x01, 0x00, 0xE8, 0x48};
byte OFF_RTU14[8] = {0x00, 0x06, 0x00, 0x0E, 0x02, 0x00, 0xE8, 0xB8};

byte ON_RTU15[8] = {0x00, 0x06, 0x00, 0x0F, 0x01, 0x00, 0xB9, 0x88};
byte OFF_RTU15[8] = {0x00, 0x06, 0x00, 0x0F, 0x02, 0x00, 0xB9, 0x78};

byte ON_RTU16[8] = {0x00, 0x06, 0x00, 0x10, 0x01, 0x00, 0x88, 0x4E};
byte OFF_RTU16[8] = {0x00, 0x06, 0x00, 0x10, 0x02, 0x00, 0x88, 0xBE};

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  while (!Serial2)
    ;
}

void loop()
{
  if (millis() - timeOut_PZEM > 2222){
    Serial2.write(0xF8);
    Serial2.write(0x04);
    Serial2.write(0x00);
    Serial2.write(0x00);
    Serial2.write(0x00);
    Serial2.write(0x0A);
    Serial2.write(0x64);
    Serial2.write(0x64);
    timeOut_PZEM = millis();
  }
  if (Serial2.available()){
    for (int i = 0; i < 25; i++){
      if (!Serial2.available()){
        Serial.println("data not match");
        break;
      }
      bufferModbus[i] = Serial2.read();
      delay(20);
    }
    uint32_t voltage = (uint32_t)bufferModbus[3] << 8 | (uint32_t)bufferModbus[4];
    uint32_t current = (uint32_t)bufferModbus[5] << 8 | (uint32_t)bufferModbus[6] | (uint32_t)bufferModbus[7] << 24 | (uint32_t)bufferModbus[8] << 16;
    uint32_t power = (uint32_t)bufferModbus[9] << 8 | (uint32_t)bufferModbus[10] | (uint32_t)bufferModbus[11] << 24 | (uint32_t)bufferModbus[12] << 16;
    uint32_t energy = (uint32_t)bufferModbus[13] << 8 | (uint32_t)bufferModbus[14] | (uint32_t)bufferModbus[15] << 24 | (uint32_t)bufferModbus[16] << 16;
    uint32_t frequecy = (uint32_t)bufferModbus[17] << 8 | (uint32_t)bufferModbus[18];

    fVoltage = voltage * 0.1;
    fCurrent = current * 0.001;
    fPower = power * 0.1;
    fEnergy = energy * 0.001;
    fFrequency = frequecy * 0.1;

    Serial.println("------------------------------");
    Serial.println("Voltage = " + String(fVoltage));
    Serial.println("Current = " + String(fCurrent));
    Serial.println("Power   = " + String(fPower));
    Serial.println("Energy  = " + String(fEnergy));
    Serial.println("Freq    = " + String(fFrequency));

    while (Serial2.available())
    {
      Serial2.read();
    }
  }
  rtu();

}

void rtu() {
  // put your main code here, to run repeatedly:
  Serial2.write(ON_RTU1, 8);
  delay(500);
  Serial2.write(ON_RTU2, 8);
  delay(500);
  Serial2.write(ON_RTU3, 8);
  delay(500);
  Serial2.write(ON_RTU4, 8);
  delay(500);
  Serial2.write(ON_RTU5, 8);
  delay(500);
  Serial2.write(ON_RTU6, 8);
  delay(500);
  Serial2.write(ON_RTU7, 8);
  delay(500);
  Serial2.write(ON_RTU8, 8);
  delay(500);
  Serial2.write(ON_RTU9, 8);
  delay(500);
  Serial2.write(ON_RTU10, 8);
  delay(500);
  Serial2.write(ON_RTU11, 8);
  delay(500);
  Serial2.write(ON_RTU12, 8);
  delay(500);
  Serial2.write(ON_RTU13, 8);
  delay(500);
  Serial2.write(ON_RTU14, 8);
  delay(500);
  Serial2.write(ON_RTU15, 8);
  delay(500);
  Serial2.write(ON_RTU16, 8);
  delay(500);


  Serial2.write(OFF_RTU1, 8);
  delay(500);
  Serial2.write(OFF_RTU2, 8);
  delay(500);
  Serial2.write(OFF_RTU3, 8);
  delay(500);
  Serial2.write(OFF_RTU4, 8);
  delay(500);
  Serial2.write(OFF_RTU5, 8);
  delay(500);
  Serial2.write(OFF_RTU6, 8);
  delay(500);
  Serial2.write(OFF_RTU7, 8);
  delay(500);
  Serial2.write(OFF_RTU8, 8);
  delay(500);
  Serial2.write(OFF_RTU9, 8);
  delay(500);
  Serial2.write(OFF_RTU10, 8);
  delay(500);
  Serial2.write(OFF_RTU11, 8);
  delay(500);
  Serial2.write(OFF_RTU12, 8);
  delay(500);
  Serial2.write(OFF_RTU13, 8);
  delay(500);
  Serial2.write(OFF_RTU14, 8);
  delay(500);
  Serial2.write(OFF_RTU15, 8);
  delay(500);
  Serial2.write(OFF_RTU16, 8);
  delay(500);

}