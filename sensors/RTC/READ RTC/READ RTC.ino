/**
 * www.arduinona.com
 * ตัวอย่างการเซ็ตค่าเวลา ให้ DS3231
 * ใช้คู่กับ Library DS3231 จาก https://github.com/PilleStat/PilleStat/tree/master/arduino%20code/libraries/DS3231
 * วิธีการต่อ 
 * Arduino UNO -> DS3231
 * 5V -> VCC
 * GND -> GND
 * SDA -> A4
 * SCL -> A5
 */


#include <Wire.h>
#include "DS3231.h"
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);
RTClib RTC;
unsigned long currentMillis = 0;
void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  Wire.begin();//เริ่มการสื่อสารแบบ I2C
}

void loop () {
  currentMillis = millis();
  RTCfunction();
  functionLcd();


}

void RTCfunction(){
  static unsigned long lastSaveTime = 0;
  if (currentMillis - lastSaveTime >= 10000U) {
    lastSaveTime = currentMillis;
    DateTime now = RTC.now();
    int day;
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    day = now.day();
    Serial.print(' ');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();
    //Serial.println(day);
  }
};
void functionLcd(){
    static unsigned long lastSaveTime = 0;
    if (currentMillis - lastSaveTime >= 1000U) {
      lastSaveTime = currentMillis;
      // clear the screen
      lcd.clear();
      lcd.setCursor(3,0);
      lcd.print("Hello, world!");
      lcd.setCursor(2,1);
      lcd.print("Ywrobot Arduino!");
      lcd.setCursor(0,2);
      lcd.print("Arduino LCM IIC 2004");
      lcd.setCursor(2,3);
      lcd.print("Power By Ec-yuan!");
    }
};