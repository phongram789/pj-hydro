//                     ESP32      TTL TO RS485
#define RX2 16       //IO16------>RXD
#define TX2 17       //IO17------>TXD

#define RS485Transmit HIGH
#define RS485Receive LOW
/*void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
    Serial2.begin(9600, SERIAL_8N1, RX2, TX2);  //RX2=16,RO ,TX2=17, DI
    closeRTU();


}*/

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


/*void loop() {
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

}*/
/*
011-Tx:00 06 00 01 01 00 D8 4B
012-Rx:00 06 00 01 01 00 D8 4B
013-Tx:00 06 00 01 02 00 D8 BB
014-Rx:00 06 00 01 02 00 D8 BB
000-Tx:00 06 00 02 01 00 28 4B
001-Tx:00 06 00 02 02 02 00 BA 7E
000-Tx:00 06 00 03 01 00 79 8B*
001-Tx:00 06 00 03 02 00 79 7B*
002-Tx:00 06 00 04 01 00 C8 4A*
003-Tx:00 06 00 04 02 00 C8 BA*
004-Tx:00 06 00 05 01 00 99 8A*
005-Tx:00 06 00 05 02 00 99 7A*
006-Tx:00 06 00 06 01 00 69 8A*
007-Tx:00 06 00 06 02 00 69 7A*
008-Tx:00 06 00 07 01 00 38 4A*
009-Tx:00 06 00 07 02 00 38 BA*
012-Tx:00 06 00 08 01 00 08 49*
013-Tx:00 06 00 08 02 00 08 B9*
014-Tx:00 06 00 09 01 00 59 89*
015-Tx:00 06 00 09 02 00 59 79*
016-Tx:00 06 00 0A 01 00 A9 89*
017-Tx:00 06 00 0A 02 00 A9 79*

004-Tx:00 06 00 0B 01 00 F8 49*
005-Tx:00 06 00 0B 02 00 F8 B9*
006-Tx:00 06 00 0C 01 00 49 88*
007-Tx:00 06 00 0C 02 00 49 78*
008-Tx:00 06 00 0D 01 00 18 48*
009-Tx:00 06 00 0D 02 00 18 B8*
010-Tx:00 06 00 0E 01 00 E8 48*
011-Tx:00 06 00 0E 02 00 E8 B8*
012-Tx:00 06 00 0F 01 00 B9 88*
013-Tx:00 06 00 0F 02 00 B9 78*
000-Tx:00 06 00 10 01 00 88 4E
001-Tx:00 06 00 10 02 00 88 BE


*/

void closeRTU(){
  int relay = 100;
  Serial2.write(OFF_RTU1, 8);
  delay(relay);
  Serial2.write(OFF_RTU2, 8);
  delay(relay);
  Serial2.write(OFF_RTU3, 8);
  delay(relay);
  Serial2.write(OFF_RTU4, 8);
  delay(relay);
  Serial2.write(OFF_RTU5, 8);
  delay(relay);
  Serial2.write(OFF_RTU6, 8);
  delay(relay);
  Serial2.write(OFF_RTU7, 8);
  delay(relay);
  Serial2.write(OFF_RTU8, 8);
  delay(relay);
  Serial2.write(OFF_RTU9, 8);
  delay(relay);
  Serial2.write(OFF_RTU10, 8);
  delay(relay);
  Serial2.write(OFF_RTU11, 8);
  delay(relay);
  Serial2.write(OFF_RTU12, 8);
  delay(relay);
  Serial2.write(OFF_RTU13, 8);
  delay(relay);
  Serial2.write(OFF_RTU14, 8);
  delay(relay);
  Serial2.write(OFF_RTU15, 8);
  delay(relay);
  Serial2.write(OFF_RTU16, 8);
  delay(relay);
}