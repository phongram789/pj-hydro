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

#include <DS3231.h>
#include <Wire.h>

DS3231 Clock;

byte Year;
byte Month;
byte Date;
byte DoW;
byte Hour;
byte Minute;
byte Second;

/** 
 *  ฟังก์ชันนี้ใช้รับค่าจาก Serial monitor เพื่อตั้งเวลา โดยมีรูปแบบคือ
 *  YYMMDDwHHMMSS และ x ลงท้าย เช่น 
 *  (2001315105300x คือ ปี 20 เดือน 01 วัน 31 วันศุกร์(5) 10 นาฬิกา 53 นาที 00 วินาที)
 */
void GetDateStuff(byte& Year, byte& Month, byte& Day, byte& DoW, 
		byte& Hour, byte& Minute, byte& Second) {


  /**
   * Call this if you notice something coming in on 
   * the serial port. The stuff coming in should be in 
   * the order YYMMDDwHHMMSS, with an 'x' at the end.
   */
    	 
    	boolean GotString = false;
    	char InChar;
    	byte Temp1, Temp2;
    	char InString[20];
    
    	byte j=0;
    	while (!GotString) {
    		if (Serial.available()) {
    			InChar = Serial.read();
    			InString[j] = InChar;
    			j += 1;
    			if (InChar == 'x') {
    				GotString = true;
    			}
    		}
    	}
    	Serial.println(InString);
    	// Read Year first
    	Temp1 = (byte)InString[0] -48;
    	Temp2 = (byte)InString[1] -48;
    	Year = Temp1*10 + Temp2;
    	// now month
    	Temp1 = (byte)InString[2] -48;
    	Temp2 = (byte)InString[3] -48;
    	Month = Temp1*10 + Temp2;
    	// now date
    	Temp1 = (byte)InString[4] -48;
    	Temp2 = (byte)InString[5] -48;
    	Day = Temp1*10 + Temp2;
    	// now Day of Week
    	DoW = (byte)InString[6] - 48;		
    	// now Hour
    	Temp1 = (byte)InString[7] -48;
    	Temp2 = (byte)InString[8] -48;
    	Hour = Temp1*10 + Temp2;
    	// now Minute
    	Temp1 = (byte)InString[9] -48;
    	Temp2 = (byte)InString[10] -48;
    	Minute = Temp1*10 + Temp2;
    	// now Second
    	Temp1 = (byte)InString[11] -48;
    	Temp2 = (byte)InString[12] -48;
    	Second = Temp1*10 + Temp2;
}

void setup() {
	/**
  * เริ่มการสื่อสารกับ Serial monitor
	 */
	    Serial.begin(115200);

	/**
  * เริ่มการสื่อสารแบบ I2C
	 */
	    Wire.begin();
}

void loop() {

	/**
  * เช็คว่ามีค่าเข้ามาทาง Serial monitor หรือไม่ 
  * ถ้าใช่ให้นำค่านั้นไปคิดในฟังก์ชัน  GetDateStuff
	*/
    	if (Serial.available()) {
    		GetDateStuff(Year, Month, Date, DoW, Hour, Minute, Second);
    
    		Clock.setClockMode(false);	// ตั้งเวลาแบบ 24h
    		//setClockMode(true);	// ตั้งเวลาแบบ 12h
    
    		Clock.setYear(Year);
    		Clock.setMonth(Month);
    		Clock.setDate(Date);
    		Clock.setDoW(DoW);
    		Clock.setHour(Hour);
    		Clock.setMinute(Minute);
    		Clock.setSecond(Second);
    
    
        /**
         * ทดสอบฟังก์ชันเซ็ตนาฬิกาปลุก โดยตั้งเวลานาฬิกาปลุก A1 ให้ล่วงหน้าค่าที่ตั้งไว้ 1 นาที และ A2 ล่วงหน้าไว้ 2 นาที
         * หากมีการปลุกเมื่อไรให้แจ้งออกมาทาง external interrupt
         */
    		Clock.setA1Time(DoW, Hour, Minute+1, Second, 0x0, true, 
    			false, false);
    		// set A2 to two minutes past, on current day of month.
    		Clock.setA2Time(Date, Hour, Minute+2, 0x0, false, false, 
    			false);
    		// Turn on both alarms, with external interrupt
    		Clock.turnOnAlarm(1);
    		Clock.turnOnAlarm(2);
    
    	}


  /**   
   *    อัพเดททุก 1 วินาที
   */
	    delay(1000);
}
/*
จากนั้นพิมพ์ ปี เดือน วัน วันที่ในสัปดาห์(Day of week) ชั่วโมง นาที วินาที ตามด้วยตัว x ปิดท้าย เช่น

ปี 2020  ใส่ 20
เดือน 1  ใส่ 01
วันที่ 30 ใส่ 30
วันที่ในสัปดาห์ (วันศุกร์ =5) ใส่ 5
ชั่วโมง 11 
นาที 46
วินาที 30 

จึงรวมกันได้เป็น 
2001305114630x เป็นต้น

Serial port
ให้เราใส่เลขข้างต้นในตำแหน่งที่ 1 ตามรูปด้านล่าง แล้วกด send เพื่อเซ็ตเวลา ควรจะมีหมายเลข 3 ตอบกลับมาดังรูป แสดงให้ทราบว่าเวลาได้ถูกเซ็ตแล้ว


*/