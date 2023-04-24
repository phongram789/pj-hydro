// ขา 0 , 2 ห้ามเป็น low
//pzemRead และ waterRs485 และ RTC จะทำให้ ปู่ม pin 0 debouce มีปัญหา

          /*
          Work list to do 
          -- esp32
          function to check flowing of A B n Acic
          function draning water and fill tank
          function pumping went AB AB n Acid solutions r pumpping 
          function count water wnet fill tank n calculate bill of water
          function updatre version of smooth to read ec and ph
          function save all value should be save in eeprom
          checkflow_ รับค่าจาก blynk --> eeprom
          -- node-red
          --reset อัตโนมัติเมื่อ เชื่อมไวไฟนานเกิน

          C:\Users\goodd\Documents\Arduino\libraries\blynk-library-master\src\Blynk\BlynkConfig.h
          */

#include <WiFi.h>
#include <WiFiClient.h>
#include <ESP32Ping.h>
#include <PubSubClient.h>
#include <BlynkSimpleEsp32.h>
#include <TimeLib.h>
#include <EEPROM.h>
#include <PZEM004Tv30.h>
#include <Wire.h>
#include "ModbusMaster.h"
#include "DS3231.h"
#include <LiquidCrystal_I2C.h>
#include <Bounce2.h>
#include "rtu.h"
#include "INPUT_PULLUP.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include "DHT.h"
#include "tds.h"


#define PHPIN 39 // การใช้งาน macro ชื่อ PHPIN โดยมีค่าเท่ากับ 39 ใช้สำหรับกำหนด Pin input เซ็นเซอร์ pH
#define ECPIN 36 // กำหนด ECPIN ให้เท่ากับ 36 ใช้สำหรับกำหนด Pin input เซ็นเซอร์ EC
#define DHTPIN 33 // กำหนด DHTPIN ให้เท่ากับ 33 ใช้สำหรับกำหนด Pin input เซ็นเซอร์ DHT
#define DHTTYPE DHT22 //กำหนด DHTTYPE ให้เท่ากับ DHT22 ใช้สำหรับกำหนดชนิดเซ็นเซอร์ให้กับ lib DHT
DHT dht(DHTPIN, DHTTYPE); //สร้างอ็อบเจกต์ของคลาส DHT โดยกำหนดค่าพารามิเตอร์ DHTPIN และ DHTTYPE 
TDS tds(ECPIN); //สร้างอ็อบเจกต์ของคลาส TDS โดยกำหนดค่าพารามิเตอร์ ECPIN เป็น pin input เซ็นเซอร์
ModbusMaster node; //สร้างอ็อบเจกต์ ModbusMaster ชื่อ node เพื่อเชื่อมต่อและสื่อสารกับอุปกรณ์ที่ใช้โปรโตคอล Modbus
int water485; // ตัวแปรสำหรับเก็บค่าน้ำที่ไหลผ่านจาก Water metor rs485

//-------------pH-----------------
bool  StatusOfPHsensor = 0; //ประกาศตัวแปรสำหรับบอกสถานะการอ่านค่า pH โดยค่า 0 หมายถึงไม่พร้อมอ่าน 1 หมายถึงพร้อมอ่าน
float lastPH ,phValue;; //ประกาศตัวแปรสำหรับเก็บค่า pH ที่อ่านได้จากเซ็นเซอร์

bool pH_calibrat_state = false; //สั่งใช้ฟังก์ชันนี้โดย เปลี่ยนสถานะเป็น 1 

int acidVoltage = 1810; // ค่าแรงดันไฟฟ้าที่เป็นค่าคงที่สำหรับสารเป็นกรด
int neutralVoltage = 1370; // ค่าแรงดันไฟฟ้าที่เป็นค่าคงที่สำหรับสารเป็นกลาง


//-------------EC-----------------
float StatusOfECsensor; //ประกาศตัวแปรสำหรับบอกสถานะการอ่านค่า EC โดยค่า 0 หมายถึงไม่พร้อมอ่าน 1 หมายถึงพร้อมอ่าน
float ecValue,lastEC; //ประกาศตัวแปรสำหรับเก็บค่า EC ที่อ่านได้จากเซ็นเซอร์
bool EC_calibrat_state = false;
float kValue = 1.38; //ยังไม่ได้จำใน eeprom


//----------------water in tank-----------------
long waterAmount =1000; //ประกาศปริมาณน้ำที่วัดได้จาก Water flow sensor มีหน่วยเป็นลิตร

bool changeWater_state = 0; //ประกาศตัวแปรสำหรับเก็บสถานะฟังก์ชันการเปลี่ยนน้ำ 0 หมายถึงไม่ได้เปลี่ยนน้ำอยู่ 1 หมายถึงเปลี่ยนน้ำอยู่

#define ONE_WIRE_BUS 26 //กำหนดขาที่จะเชื่อมต่อ Sensor วัดอุณหภูมิน้ำด้วยชื่อ ONE_WIRE_BUS โดยใช้ขา 26
OneWire oneWire(ONE_WIRE_BUS); // การสร้างอ็อบเจกต์ oneWire ของคลาส OneWire โดยกำหนดขาเชื่อมต่อของอุปกรณ์แบบ One-Wire ที่ชื่อ ONE_WIRE_BUS คือชื่อ pin
DallasTemperature sensorsWatertemp(&oneWire); //สร้างอ็อบเจกต์ sensorsWatertemp ของคลาส DallasTemperature โดยกำหนดพารามิเตอร์ว่า &oneWire ที่เป็นอ็อบเจกต์ของคลาส OneWire ซึ่งใช้สำหรับเชื่อมต่อและสื่อสารกับอุปกรณ์เซนเซอร์อุณหภูมิของ Dallas Semiconductor เพื่ออ่านค่าอุณหภูมิในน้ำ เซ็นเซอร์ DS18B20

LiquidCrystal_I2C lcd(0x27, 20, 4); //การสร้างอ็อบเจกต์ lcd ของคลาส LiquidCrystal_I2C โดยกำหนดพารามิเตอร์ต่างๆ ได้แก่ 0x27 ที่เป็นที่อยู่ของชิปแปลง I2C address ที่ใช้ในการสื่อสารระหว่าง Arduino กับจอ LCD, 20 และ 4 คือ จำนวนตัวอักษรและบรรทัดของจอ LCD ที่ใช้งาน โดย LiquidCrystal_I2C เป็นไลบรารี่ที่ใช้ในการควบคุมการแสดงผลบนจอ LCD ด้วยวิธีการสื่อสารแบบ I2C ซึ่งช่วยลดจำนวนขาเชื่อมต่อที่ใช้ในการเชื่อมต่อกับจอ LCD

RTClib RTC; //การสร้างอ็อบเจกต์ RTC ของคลาส RTClib และอ็อบเจกต์ Clock ของคลาส DS3231 โดยไม่ได้กำหนดพารามิเตอร์ใดๆ ซึ่งคลาส RTClib เป็นไลบรารี่ที่ใช้ในการจัดการเวลา (RTC)
DS3231 Clock; //ส่วนคลาส DS3231 เป็นไลบรารี่ที่ใช้ในการอ่านค่าเวลาจากชิป RTC DS3231 และจัดการเกี่ยวกับการตั้งค่าเวลา ซึ่งใช้สัญญาณ I2C ในการสื่อสารกับไมโครคอนโทรลเลอร์

PZEM004Tv30 pzem(Serial2, 16, 17,0x07); //การสร้างอ็อบเจกต์ pzem ของคลาส PZEM004Tv30 โดยกำหนดพารามิเตอร์ต่างๆ ได้แก่ Serial2 เป็นพอร์ตซีเรียลที่ใช้สำหรับการสื่อสารกับ PZEM004Tv30, 16 และ 17 คือ พินที่ใช้สื่อสาร และ 0x07 คือค่า address ของ เซ็อเซอร์ pzem004j

WiFiClient espClient; //การสร้างอ็อบเจกต์ espClient ของคลาส WiFiClient สำหรับใช้งาน WiFi ของ ESP32
PubSubClient mqtt(espClient); //อ็อบเจกต์ mqtt ของคลาส PubSubClient สำหรับการเชื่อมต่อและส่งข้อมูลผ่าน MQTT โดยใช้ espClient เป็นพารามิเตอร์
BlynkTimer timer; //สร้างอ็อบเจกต์ timer ของคลาส BlynkTimer เพื่อใช้ในการจัดการสั่งงานรอบการทำงานฟังก์ชันต่างๆ

#define BLYNK_PRINT Serial //กำหนดค่าคงที่ BLYNK_PRINT ให้มีค่าเป็น Serial เพื่อใช้ในการพิมพ์ข้อความผ่านทาง Serial Monitor
#define EEPROM_SIZE 100 // กำหนดค่าคงที่ EEPROM_SIZE ให้มีค่าเป็น 64

/*const char* ssid = "ooy";
const char* password = "0863447295";
const char* mqtt_server = "broker.netpie.io";
const int mqtt_port = 1883;
const char* mqtt_client = "aa47cd89-19f0-4db3-a3df-b823fb50b939";
const char* mqtt_username = "iWrjeyzumGdQGZM8pSEobjA3cPCUpciE";
const char* mqtt_password = "n6htDnjn7rLq8_epUM)N-M076iMWy4t4";*/

const char* ssid = "Smart_hydro_2G"; //การกำหนดค่าคงที่ ssid ของ WiFi 
const char* password = "123456789"; //การกำหนดค่าคงที่ password ของ WiFi 
const char* mqtt_server = "192.168.2.35"; // กำหนดค่าคงที่ mqtt_server เพื่อกำหนดที่อยู่ IP
const int mqtt_port = 1883; // พอร์ตของ MQTT Broker
//กำหนดค่าคงที่ mqtt_client, mqtt_username, และ mqtt_password เพื่อกำหนดชื่อของ MQTT client, ชื่อผู้ใช้งาน MQTT broker และรหัสผ่านของผู้ใช้งาน MQTT broker ตามลำดับ
const char* mqtt_client = "esp32"; 
const char* mqtt_username = "esp";
const char* mqtt_password = "esp";

//กำหนดค่าคงที่ BLYNK_TEMPLATE_ID และ BLYNK_DEVICE_NAME เพื่อกำหนด ID และชื่อของ Blynk Template ที่ใช้ในการเชื่อมต่อกับ Blynk Server และกำหนดค่าคงที่ BLYNK_AUTH_TOKEN เพื่อกำหนดค่า Token สำหรับการเชื่อมต่อกับ Blynk Server
#define BLYNK_TEMPLATE_ID "TMPLLaOYk4zr"
#define BLYNK_DEVICE_NAME "Smart hydroponic for urban"
#define BLYNK_AUTH_TOKEN "1aE2xcwuAkfH4FYhRq36xLlexDVPPvu4"
char auth[] = BLYNK_AUTH_TOKEN; //กำหนดค่า auth[] ให้มีค่าเท่ากับ BLYNK_AUTH_TOKEN เพื่อใช้ในการเชื่อมต่อ Blynk

unsigned long currentMillis = 0; //กำหนดตัวแปร currentMillis เพื่อเก็บค่าเวลาปัจจุบัน (ใน millisecond) ที่ได้จากฟังก์ชัน millis() ซึ่งจะใช้ในการควบคุมเวลาในการทำงานของโปรแกรม Arduino ต่อไป
float temperatureC,hum_room,temp_room;// temperatureC สำหรับเก็บค่าอุณหภูมิน้ำ, hum_room และ temp_room เก็บค่าความชื้น (humidity) อุณหภูมิ (temperature) ในห้องหรือสภาพแวดล้อม


bool RtcState = false; // สถานะอ่าน Real time clock 0 หมายถึงยังอ่านไม่ได้ 1 หมายถึงอ่านได้สำเร็จ
int nowHour,nowMinute,nowSecond;       //ตัวแปรสำหรับเก็บค่าเวลา RTC ชม. น. ว. ตามลำดับ HH:MM:SS
int startHour,startMinute,startSecond; //ตัวแปรสำหรับเก็บค่าเวลาเปิดไฟปลูก startHour คือ ชั่วโมงเปิด,startMinute นาทีที่เปิด,startSecond วินาทีที่เปิด  ON HH:MM:SS 
int stopHour,stopMinute,stopSecond;   //ตัวแปรสำหรับเก็บค่าเวลาเปิดไฟปลูก stopHour คือ ชั่วโมงปิด,stopMinute นาทีที่ปิด,stopSecond วินาทีที่ปิด OFF HH:MM:SS
int startTimeInSeconds; // ตัวแปรสำหรับเก็บค่าเวลาเปิดไฟปลูกเป็นวินาทีที่รวมทั้งชั่วโมง,นาที และวินาที
int stopTimeInSeconds; // ตัวแปรสำหรับเก็บค่าเวลาปิดไฟปลูกเป็นวินาทีที่รวมทั้งชั่วโมง,นาที และวินาที
int currentTimeInSeconds; // ตัวแปรสำหรับเก็บค่าเวลาปัจจุบัน RTC ไฟปลูกเป็นวินาทีที่รวมทั้งชั่วโมง,นาที และวินาที

unsigned long milleHour; //ตัวแปรเก็บมิลิเซคของ 1 ชั่วโมง
int HourUpdateRTC; // ตัวแปรสำหรับเก็บชั่วโมง ใช้นับถอยหนังอัพเดทเวลา RTC กับ server 

bool growLigh1,growLigh2,growLigh3,growLigh4; //ตัวแปรสถานะของไฟปลูกดวงที่ 1 - 4 HIGH หมายถึงไฟเปิด LOW หมายถึงไฟปิด

bool timeonGL; //ตัวแปรสำหรับเก็บค่าเปิด-ปิด เมื่อไฟปลูกอยู่ชโหมด Auto  0 หมายถึงเวลานี้ไฟปิด, 1 หมายถึงเวลานี้ไฟเปิด //value of on/off in time clock

bool AdjustEcState,AdjustPHState; // ประกาศตัวแปลสำหรับเก็บค่าสถานะการปรับค่า EC ,pH

float phLow,phHigh,ecLow,ecHigh; // phLow สำหรับเก็บค่า pH ขั้นต่ำ , phHigh สำหรับเก็บค่า pH ขั้นสูง , ecLow สำหรับเก็บค่า EC ขั้นต่ำ , ecHigh สำหรับเก็บค่า EC ขั้นสูง 

bool statusBlynk,statusMqtt,statusWifi;  //ตัวแปรสำหรับเก็บค่าสถานะการเชื่อมต่อ 0 คือยังไม่เชื่อมต่อ 1 คือเชื่อมต่อสำเร็จ

float voltage ,current ,power ,energy , frequency, pf; // ประกาศตัวแปรสำหรับเก็บค่าไฟฟ้าต่างๆ  voltage เก็บค่าโวลต์ , current เก็บค่ากระแสที่ใช้ , power คือกำลังวัตต์ที่ใช้งาน ,energy พลังงานที่ใช้ไปหน่วย kwh หรือ unit , frequency เก็บความถี่ไฟฟ้า
float ft = NULL;// ประกาศตัวแปรนี้สำหรับเก็บค่าไฟฟ้าผันแปร
float Unit,energyprice; //Unit สำหรับเก็บค่าจำนวนหน่อยไฟฟ้าที่ใช้ไป energyprice สำหรับเก็บค่า ราคาไฟฟ้าที่คำนวณได้จากหน่วยไฟที่ใช้ไป

bool ecAuto, ecMan, phAuto, phMan ,GLAuto ,GLMan; //ตัวแปรสำหรับเก็บค่าสถานะสวิตช์ที่ตู้คอนโทรล ecAuto คือ input ที่จากสวิตช์ EC โหมด Auto , ecMan คือ input ที่จากสวิตช์ EC โหมด Manual
// phAuto คือ input ที่จากสวิตช์ pH โหมด Auto , phMan คือ input ที่จากสวิตช์ pH โหมด Manual ,GLAuto คือ input ที่จากสวิตช์ไฟปลูก โหมด Auto , GLMan คือ input ที่จากสวิตช์ไฟปลูก โหมด Manual

bool GrowLightControl1,GrowLightControl2,GrowLightControl3,GrowLightControl4; // read input from blynk and mtqq 0 = off, 1 = on --> save eefrom repeate

bool flowwing; // water in main line plant is flowing is good

bool stirPumpAB = LOW;
bool stirPumpPh = LOW;

bool checkflow_ = false ; //สถานะเปิดปิดฟังก์ชั่น
bool drain_state = false; //สถานะการเปลี่ยนน้ำ
bool empty_tank = true; //สถานะน้ำในถัง
//bool empty_tank = false; // PUMP pH  N EC

#define pinSwitchEcAuto 5
#define pinSwitchEcMan 18

#define pinSwitchPHAuto 4
#define pinSwitchPHMan 15

#define pinSwitchGrowlightAuto 19
#define pinSwitchGrowlightMan 23

#define pinSwitchtestman 3

#define buttonPinMainWaterPump  0 // 15 สลับกับขา 0 เพราะ connected to on-board LED, must be left floating or LOW to enter flashing mode

bool mainWaterPump = HIGH;
Bounce debouncer = Bounce(); // create obj

swInput swAutoEC(pinSwitchEcAuto);
swInput swManEC(pinSwitchEcMan);
swInput swAutoPH(pinSwitchPHAuto);
swInput swManPH(pinSwitchPHMan);
swInput swAutoGL(pinSwitchGrowlightAuto);
swInput swManGL(pinSwitchGrowlightMan);
swInput swManWaterIn(pinSwitchtestman);

#define pinWaterLevel_Top 35//25
swInput WaterLevel_Top(pinWaterLevel_Top);
#define pinWaterLevel_Bottom 25
swInput WaterLevel_Bottom(pinWaterLevel_Bottom);

bool changeWaterState = false;

bool release_valve_changewater = LOW;
bool release_valve_drain = LOW;
bool refill_valve_changeWater = LOW ;
bool refill_valve_fill = LOW;

WidgetLCD lcdBlynk(V16);

//---------------------test station-----------------------
#define flow_plantingTrough_pin 27
int pulse_plantingTrough;

#define flow_WaterTank_pin 14
int pulse_WaterTank;

#define flow_A_Solution_pin 34
int pulse_A_Solution;

#define flow_B_Solution_pin 32
int pulse_B_Solution;

#define flow_phDownSolution_pin 13 //กำหนดค่า 13 ชื่อ flow_phDownSolution_pin
int pulse_phDownSolution; //ตัวแปรสำหรับเก็บจำนวนพัลส์


void IRAM_ATTR pulseCounter_plantingTrough(){
  pulse_plantingTrough++;
}
void IRAM_ATTR pulseCounter_WaterTank(){
  pulse_WaterTank++;
}
void IRAM_ATTR pulseCounter_A_Solution(){
  pulse_A_Solution++;
}

void IRAM_ATTR pulseCounter_phDownSolution(){
  pulse_phDownSolution++;
}

void IRAM_ATTR pulseCounter_B_Solution(){
  pulse_B_Solution++;
}


// Set your Static IP address
IPAddress local_IP(192, 168, 2, 184);
// Set your Gateway IP address
IPAddress gateway(192, 168, 2, 1);
IPAddress subnet(255, 255, 0, 0);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  initEEPROM();
  lcd.init();
  lcd.backlight();
  Wire.begin();//เริ่มการสื่อสารแบบ I2C
  initWiFi();
  mqtt.setServer(mqtt_server, mqtt_port);
  mqtt.setCallback(callback);
  Blynk.config(auth,"blynk.cloud", 8080);
  timer.setInterval(2000L, sendSensor);
  timer.setInterval(10000L, lcdBlynkPrint);
  timer.setInterval(6000L, readWaterTemp);
  timer.setInterval(100000L, waterRs485);
  timer.setInterval(1000L, StirPump);
  timer.setInterval(20000L, statusMqttMsg);
  timer.setInterval(1000L, PRINT);
  timer.setInterval(10000L, release_valve);
  timer.setInterval(10000L, refill_valve);
  timer.setInterval(1000L, empty_check_Water);


  sensorsWatertemp.begin();

  // rs485
  //Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  while(!Serial2);
  node.begin(1, Serial2); // Id slave is 1 

  //--------------push button-----------------
  debouncer.attach(buttonPinMainWaterPump, INPUT_PULLUP);
  debouncer.interval(20); // กำหนดเวลาการเปลี่ยนสถานะให้กับ debouncer object ที่ 20 มิลลิวินาที

  //------------flowing-----------------------
  pinMode(flow_plantingTrough_pin, INPUT_PULLUP);
  pinMode(flow_WaterTank_pin, INPUT_PULLUP);
  pinMode(flow_A_Solution_pin, INPUT_PULLUP);
  pinMode(flow_phDownSolution_pin, INPUT_PULLUP);
  pinMode(flow_B_Solution_pin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(flow_plantingTrough_pin), pulseCounter_plantingTrough, FALLING);
  attachInterrupt(digitalPinToInterrupt(flow_WaterTank_pin), pulseCounter_WaterTank, FALLING);
  attachInterrupt(digitalPinToInterrupt(flow_A_Solution_pin), pulseCounter_A_Solution, FALLING);
  attachInterrupt(digitalPinToInterrupt(flow_phDownSolution_pin), pulseCounter_phDownSolution, FALLING);
  attachInterrupt(digitalPinToInterrupt(flow_B_Solution_pin), pulseCounter_B_Solution, FALLING);

  dht.begin();
  tds.kValue = kValue;
  delay(500);
  closeRTU();
}

void loop() {
  // put your main code here, to run repeatedly:
  currentMillis = millis();
  wifiReconnect();
  reconnectBlynk();
  timer.run();
  RTCfunction();
  controlWaterPump();
  functionLcd();
  EEPROMfunction();
  pzemRead();
  controlEC();
  controlPH();
  controlGrowLight();
  readDHT();
  PH();
  Ec();
  Mqttreconnect();
  checkFlow();
  fillWater();
  changeWater(changeWaterState);
  autoUpdateRTC();
  drainWater();
  calibration_pH_sensor();
  calibration_EC_sensor();
}

void PRINT(){
  /*Serial.println("ecAuto : " + String(ecAuto));
  Serial.println("ecMan : " + String(ecMan));*/ // read switch is good
  /*Serial.println("phAuto : " + String(phAuto));
  Serial.println("phMan : " + String(phMan));*/ //read switch is good
  /*Serial.println("GLAuto : " + String(GLAuto));
  Serial.println("GLMan : " + String(GLMan));*/ //switch is good
   /*static int count;
   count++;
   Serial.println("count : " + String(count));*/
  /*Serial.println("changeWater: " + String(changeWaterState));
  Serial.println("drain_state : " + String(drain_state));
  Serial.println("empty_tank : " + String(empty_tank));
  Serial.println("AdjustEcState : " + String(AdjustEcState));
  Serial.println("lastEC : " + String(lastEC));
  Serial.println("ecLow : " + String(ecLow));*/
  

  //Serial.println("mainWaterPump : " + String(mainWaterPump));

  /*bool switch_fillwaterAuto = swManWaterIn.get_status();
  Serial.println("switch_fillwaterAuto : " + String(switch_fillwaterAuto));*/ // read switch is good

  /*bool switch_WaterLevel_Top = WaterLevel_Top.get_status(); 
  bool switch_WaterLevel_Bottom = WaterLevel_Bottom.get_status();
  Serial.println("switch_WaterLevel_Top : " + String(switch_WaterLevel_Top));
  Serial.println("switch_WaterLevel_Bottom : " + String(switch_WaterLevel_Bottom));*/ //is good

 /* Serial.println("pulse_phDownSolution : " + String(pulse_phDownSolution));
  Serial.println("pulse_A_Solution : " + String(pulse_A_Solution));
  Serial.println("pulse_B_Solution : " + String(pulse_B_Solution));
  Serial.println("pulse_plantingTrough : " + String(pulse_plantingTrough));
  Serial.println("pulse_WaterTank : " + String(pulse_WaterTank));*/ //is good
}
void sendSensor(){ //to blynk
  //float voltage ,current ,power ,energy , frequency, pf;
  //Blynk.virtualWrite(V15, pf);
  Blynk.virtualWrite(V1, phValue);
  Blynk.virtualWrite(V2, ecValue);
  Blynk.virtualWrite(V32, flowwing);
  
  //Blynk.virtualWrite(V, WaterLevel_Top.get_status());

  //test
  //readWaterTemp();
  
};

void statusMqttMsg(){ //ฟังก์ชั่นส่งสถานะต่างๆของระบบกระกอบด้วยเวลาเปิด-ปิดไฟ ค่าสำหรับกำหนดช่วง pH EC สถานะเปิดปิดไฟปลูก
  if (mqtt.connected() == true) {
      mqtt.loop();
      String dataJS = "{\"timeOnHH\":" + String(startHour) + ",\"timeOnMM\":" + String(startMinute) + 
                      ",\"timeOffHH\":" + String(stopHour) + ",\"timeOffMM\":" + String(stopMinute) + 
                      ",\"phlow\":" + String(phLow) + ",\"phHigh\":" + String(phHigh) + 
                      ",\"ecLow\":" + String(ecLow) + ",\"ecHigh\":" + String(ecHigh) + 
                      ",\"gl1\":" + String(growLigh1) + ",\"gl2\":" + String(growLigh2) + 
                      ",\"gl3\":" + String(growLigh3) + ",\"gl4\":" + String(growLigh4) + "}";
      char json[150];
      dataJS.toCharArray(json,dataJS.length()+1);
      mqtt.publish("@msg/kmutnb/cs/smart-hydro1/status", json);
  }
}

void notifyingPubMqtt(String dataJS){ //ฟังก์ชันส่งการแจ้งเตือนไปยัง mqtt
  if (mqtt.connected() == true) {
      mqtt.loop();
      char json[150];
      dataJS.toCharArray(json,dataJS.length()+1);
      mqtt.publish("@msg/kmutnb/cs/smart-hydro1/notifying", json);
  }
}

void Mqttreconnect(){ //ฟังก์ชันตรวจสอบสถานะการเชื่อมต่อ MQTT และทำการเชื่อมต่ออีกครั้งหากการเชื่อมต่อหลุดหรือไม่สำเร็จ
  static unsigned long timepoint = 0;
  static int error_con_time = 0;
  if(currentMillis - timepoint >= 5000U){
    timepoint = currentMillis;
    if (mqtt.connected() == false) {
      error_con_time++;
      if(error_con_time > 10){
        //Serial.print("Err Mqtt : " + String(error_con_time)+" time" );
      }
      statusMqtt = mqtt.connected(); // return status of mqtt now
      Serial.print("MQTT connection... ");
      if (mqtt.connect(mqtt_client, mqtt_username, mqtt_password)){ //ทำการ subscribe topic ที่กำหนดไว้เพื่อรับข้อมูลเมื่อมีการ publish มา
        //------------------range of ec ph values---
        mqtt.subscribe("@msg/eclow"); 
        mqtt.subscribe("@msg/echigh");
        mqtt.subscribe("@msg/phlow");
        mqtt.subscribe("@msg/phhigh");
        //------------------get values to control growlight ---
        mqtt.subscribe("@msg/kmutnb/cs/smart-hydro1/gl1"); 
        mqtt.subscribe("@msg/kmutnb/cs/smart-hydro1/gl2");
        mqtt.subscribe("@msg/kmutnb/cs/smart-hydro1/gl3");
        mqtt.subscribe("@msg/kmutnb/cs/smart-hydro1/gl4");
        //------------------get values to control main water pump ---
        mqtt.subscribe("@msg/kmutnb/cs/smart-hydro1/mainwater");
        //-----------------time no-off
        mqtt.subscribe("@msg/kmutnb/cs/smart-hydro1/hour-on");
        mqtt.subscribe("@msg/kmutnb/cs/smart-hydro1/minute-on");
        mqtt.subscribe("@msg/kmutnb/cs/smart-hydro1/hour-off");
        mqtt.subscribe("@msg/kmutnb/cs/smart-hydro1/minute-off");
        //-----------------reset
        mqtt.subscribe("@msg/kmutnb/cs/smart-hydro1/reset-power");
        mqtt.subscribe("@msg/kmutnb/cs/smart-hydro1/reset-water");
        mqtt.subscribe("@msg/kmutnb/cs/smart-hydro1/flow-check");
        
        //----------------- change water n drain water
        mqtt.subscribe("@msg/kmutnb/cs/smart-hydro1/drain");
        mqtt.subscribe("@msg/kmutnb/cs/smart-hydro1/change-water");

        //----------------- calibration sensors state
        mqtt.subscribe("@msg/kmutnb/cs/smart-hydro1/calibrat-ph");
        mqtt.subscribe("@msg/kmutnb/cs/smart-hydro1/calibrat-ec");



        Serial.println("connected");
        statusMqtt = mqtt.connected();
      } 
      else{
        statusMqtt = mqtt.connected();
        //Serial.println("failed");
        //delay(1000); //fix to millis 
      }
    }
    //หลังจากที่ตรวจสอบว่า MQTT มีการเชื่อมต่ออยู่ ฟังก์ชันนี้จะทำการเชื่อมต่อและ publish ข้อมูลไปยัง MQTT broker โดยใช้ข้อมูลจากตัวแปร temp_room, hum_room, ecValue, phValue, temperatureC และ waterAmount ซึ่งเป็นข้อมูลที่เกี่ยวข้องกับเซนเซอร์ต่างๆ
    else {
      error_con_time = 0; 
      mqtt.loop();
      
      String dataJS = "{\"temp\":" + String(temp_room) + ",\"hum\":" + String(hum_room) + ",\"ec\":" + String(ecValue) + ",\"ph\":" + String(phValue) + ",\"waterTemp\":" + String(temperatureC) + ",\"waterFlow\":" + String(waterAmount) + "}" ;
      char json[150];
      dataJS.toCharArray(json,dataJS.length()+1);
      mqtt.publish("@msg/kmutnb/cs/smart-hydro1/sensors", json);
    }
  }
};

void initWiFi() { //ฟังก์ชันนี้จะทำการเชื่อมต่อกับ WiFi network และรอจนกว่าจะเชื่อมต่อสำเร็จ หากไม่สำเร็จภายใน 30 วินาที จะมีการ restart ESP32 ทำให้เริ่มต้นการเชื่อมต่อ WiFi ใหม่อีกครั้ง
  //192.127.2.34
  static int count_time = 0;
  WiFi.begin(ssid, password);
  WiFi.mode(WIFI_STA);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Connecting to WiFi..");
  while (WiFi.status() != WL_CONNECTED) {
    count_time++;
    if(count_time > 30){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("ESP WLL RESTART");
      delay(10000);
      ESP.restart();
    }
    Serial.print('.');
    delay(1000);
  }
  lcd.clear();
  lcd.setCursor(3,0);
  lcd.print("WiFi Connected");
  Serial.println(WiFi.localIP());
  statusWifi = WiFi.status();
  delay(2000);

};

void wifiReconnect(){//ฟังก์ชันจะทำการตรวจสอบว่าการเชื่อมต่อ WiFi ได้เชื่อมต่อสำเร็จหรือไม่ และหากไม่สำเร็จในระยะเวลา 30 วินาที จะทำการ reconnect อัตโนมัติ
  static unsigned long previousMillis = 0;
  unsigned long interval = 30000;
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >=interval)) {
    Serial.print(millis());
    //Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    previousMillis = currentMillis;
  }
}

void initEEPROM() {  //ฟังก์ชัน initEEPROM จะทำการเริ่มต้นการใช้งาน EEPROM และทำการดึงข้อที่อยู่ภายใน EEPROM ออกมาเก็บที่ตัวแปร
  if (!EEPROM.begin(EEPROM_SIZE))
  {
    Serial.println("failed to initialise EEPROM");
  }
  Serial.println(" bytes read from Flash . Values are:");
  for (int i = 0; i < EEPROM_SIZE; i++)
  {
    Serial.print(byte(EEPROM.read(i))); Serial.print(" ");
  }
  /*for (int i = 0; i < EEPROM_SIZE; i++) //reset value from eeprom
  {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();*/
  Serial.println("Reading...");
  int address = 0;
  startHour = EEPROM.read(address);
  address += sizeof(startHour);
  startMinute = EEPROM.read(address);
  address += sizeof(startMinute);
  startSecond = EEPROM.read(address);
  address += sizeof(startSecond);
  stopHour = EEPROM.read(address);
  address += sizeof(stopHour);
  stopMinute = EEPROM.read(address);
  address += sizeof(stopMinute);
  stopSecond = EEPROM.read(address);
  address += sizeof(stopSecond);
  EEPROM.get(address, phLow);
  address += sizeof(phLow);
  EEPROM.get(address, phHigh);
  address += sizeof(phHigh);
  EEPROM.get(address, ecLow);
  address += sizeof(ecLow);
  EEPROM.get(address, ecHigh);
  address += sizeof(ecHigh);
  GrowLightControl1 = EEPROM.read(address);
  address += sizeof(GrowLightControl1);
  GrowLightControl2 = EEPROM.read(address);
  address += sizeof(GrowLightControl2);
  GrowLightControl3 = EEPROM.read(address);
  address += sizeof(GrowLightControl3);
  GrowLightControl4 = EEPROM.read(address); 
  address += sizeof(milleHour);
  milleHour = EEPROM.read(address);
  address += sizeof(HourUpdateRTC);
  HourUpdateRTC = EEPROM.read(address);
  
  //---
  address += sizeof(waterAmount);
  EEPROM.get(address, waterAmount);
  address += sizeof(acidVoltage);
  EEPROM.get(address, acidVoltage);
  address += sizeof(neutralVoltage);
  EEPROM.get(address, neutralVoltage);
  //---
  address += sizeof(kValue);
  EEPROM.get(address, kValue);

  /*address += sizeof(ft);
  EEPROM.get(address, ft);*/


  Serial.println("HourUpdateRTC: " + String(HourUpdateRTC));
  Serial.println("waterAmount: " + String(waterAmount));
  Serial.println("acidVoltage: " + String(acidVoltage));
  Serial.println("neutralVoltage: " + String(neutralVoltage));
  Serial.println("kValue: " + String(kValue));
  Serial.println("ft: " + String(ft));
};

void callback(char* topic,byte* payload, unsigned int length) { //ฟังก์ชัน callback เป็นฟังก์ชันที่ถูกเรียกใช้เมื่อมีการรับข้อความจากโบรกเกอร์ MQTT 
  /*Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");*/
  String msg;
  for (int i = 0; i < length; i++) { //แปลง payload จากข้อมูลที่รับเข้ามาเป็นข้อความ String โดยที่ payload เป็นอาร์เรย์ของ byte และ length คือความยาวของ payload ซึ่งเป็นพารามิเตอร์ที่ส่งมาพร้อมกับ topic
    msg = msg + (char)payload[i];
  }
  if (String(topic) == "@msg/eclow") {  //หากชื่อหัวข้อ (topic) ของข้อความที่ส่งมาเท่ากับ "@msg/eclow" จะเก็บค่าที่อยู่ใน payload ไว้ในตัวแปร msg แล้วแปลงค่าใน msg เป็นตัวเลขที่มีทศนิยม (float) แล้วเก็บลงในตัวแปร ecLow
    ecLow = msg.toFloat();
  }
  if (String(topic) == "@msg/echigh") { //หากชื่อหัวข้อ (topic) ของข้อความที่ส่งมาเท่ากับ "@msg/echigh" จะเก็บค่าที่อยู่ใน payload ไว้ในตัวแปร msg แล้วแปลงค่าใน msg เป็นตัวเลขที่มีทศนิยม (float) แล้วเก็บลงในตัวแปร ecHigh
    ecHigh = msg.toFloat();
  }
  if (String(topic) == "@msg/phlow") { // หากชื่อหัวข้อ (topic) ของข้อความที่ส่งมาเท่ากับ "@msg/phlow" จะเก็บค่าที่อยู่ใน payload ไว้ในตัวแปร msg แล้วแปลงค่าใน msg เป็นตัวเลขที่มีทศนิยม (float) แล้วเก็บลงในตัวแปร phLow
    phLow = msg.toFloat();
  }
  if (String(topic) == "@msg/phhigh") { // เมื่อได้รับข้อความจาก topic "@msg/phhigh" ให้กำหนดค่า phHigh ในโปรแกรมเป็นค่าที่ได้รับมาในรูปแบบของตัวเลขทศนิยม (float) จากข้อความที่ได้รับมา (msg)
    phHigh = msg.toFloat();
  }
  
  //-------------------------- control growlight-----------------

  if (String(topic) == "@msg/kmutnb/cs/smart-hydro1/gl1") { // หากมีการส่งข้อความมาที่ topic "@msg/kmutnb/cs/smart-hydro1/gl1" จะทำการกำหนดเพื่อกำหนดสถานะให้ไฟปลูก
    if (msg == "1"){
      GrowLightControl1 = 1;
    } else {
      GrowLightControl1 = 0;
    }
  }
  if (String(topic) == "@msg/kmutnb/cs/smart-hydro1/gl2") { 
    if (msg == "1"){
      GrowLightControl2 = 1;
    } else {
      GrowLightControl2 = 0;
    }
  }
  if (String(topic) == "@msg/kmutnb/cs/smart-hydro1/gl3") { 
    if (msg == "1"){
      GrowLightControl3 = 1;
    } else {
      GrowLightControl3 = 0;
    }
    
  }
  if (String(topic) == "@msg/kmutnb/cs/smart-hydro1/gl4") { 
    if (msg == "1"){
      GrowLightControl4 = 1;
    } else {
      GrowLightControl4 = 0;
    }
  }

  //-------------------------time 
  if (String(topic) == "@msg/kmutnb/cs/smart-hydro1/hour-on") { 
    startHour = msg.toInt();
  }
  if (String(topic) == "@msg/kmutnb/cs/smart-hydro1/minute-on") { 
    startMinute = msg.toInt();
  }
  if (String(topic) == "@msg/kmutnb/cs/smart-hydro1/hour-off") { 
    stopHour = msg.toInt();
  }
  if (String(topic) == "@msg/kmutnb/cs/smart-hydro1/minute-off") { 
    stopMinute = msg.toInt();
  }

  //-------------------------main water pump pumping to line plant
  if (String(topic) == "@msg/kmutnb/cs/smart-hydro1/mainwater") {  // หากมีการส่งข้อความมาที่ topic "@msg/kmutnb/cs/smart-hydro1/mainwater" จะทำการกำหนดเพื่อกำหนดสถานะปั๊มขึ้นรางปลูก
    if (msg == "1" && mainWaterPump == LOW){
      mainWaterPump = HIGH;
    } else if(msg == "0" && mainWaterPump == HIGH){
      mainWaterPump = LOW;
    }
  }
  //-------------------------flow checking
  if (String(topic) == "@msg/kmutnb/cs/smart-hydro1/flow-check") { // หากมีการส่งข้อความมาที่ topic "@msg/kmutnb/cs/smart-hydro1/flow-check" จะทำการกำหนดสถานะการใช้ฟังก์ชันตรวจสอบการไหล และทำการตอบสถานะฟังก์ชันกลับไปยัง MQTT
    checkflow_ = !checkflow_ ;
    Blynk.virtualWrite(V26,checkflow_);
    String text  = "{\"funcflow\":" + String(checkflow_) + "}";
    notifyingPubMqtt(text);
  }
  if (String(topic) == "@msg/kmutnb/cs/smart-hydro1/reset-power") { //เมื่อมีการส่งข้อความมาที่ topic "@msg/kmutnb/cs/smart-hydro1/reset-power" และข้อความที่ส่งมามีค่าเป็น "reset" จะทำการเรียกใช้ฟังก์ชั่น resetEnergy() เพื่อล้างค่าการใช้พลังงานทั้งหมดใน PZEM-004T
    if (msg == "reset"){
      clearSerial2Buffer(); //เรียกใช้ฟังก์ชั่น clearSerial2Buffer() เพื่อล้างข้อมูลใน Serial buffer
      pzem.resetEnergy();
      String text  = "{\"reset_energy\":" + String(1) + "}";
      notifyingPubMqtt(text);
    }
  }
  if (String(topic) == "@msg/kmutnb/cs/smart-hydro1/reset-water") {  // ถ้าหัวข้อ (topic) ของข้อความที่มาถึงเป็น "@msg/kmutnb/cs/smart-hydro1/reset-water" และข้อความ (msg) เป็น "1" จะทำการรีเซ็ตค่าปริมาณน้ำ (waterAmount) เป็น 0.0 และทำการตอบสถานะฟังก์ชันกลับไปยัง MQTT
    if (msg == "1"){
      waterAmount = 0.0;
      String text  = "{\"reset_water\":" + String(1) + "}";
      notifyingPubMqtt(text);
    }
  }

  if (String(topic) == "@msg/kmutnb/cs/smart-hydro1/drain") {  // หากมีการส่งข้อความมาที่ topic "@msg/kmutnb/cs/smart-hydro1/drain" จะดำเนินการตรวจสอบข้อความว่าเป็น "1" หรือไม่ ถ้าเป็นจะทำการสลับสถานะสำหรับควบคุมฟังก์ชันถ่ายน้ำออกจากถัง และตอบสถานะฟังก์ชันกลับไปยัง MQTT
    if (msg == "1"){
      Serial.println(drain_state);
      drain_state = !drain_state;
      if(drain_state == true){
        changeWaterState = false;
      }
      Serial.println(drain_state);
      String text  = "{\"drain\":" + String(drain_state) + "}";
      notifyingPubMqtt(text);
    }
  }

  if (String(topic) == "@msg/kmutnb/cs/smart-hydro1/change-water") { // หากมีการส่งข้อความมาที่ topic "@msg/kmutnb/cs/smart-hydro1/change-water" จะดำเนินการตรวจสอบข้อความว่าเป็น "1" หรือไม่ ถ้าเป็นจะทำการสลับสถานะตัวแปร changeWaterState สำหรับควบคุมฟังก์เปลี่ยนน้ำในถัง และตอบสถานะฟังก์ชันกลับไปยัง MQTT
    if (msg == "1"){
      Serial.println(changeWaterState);
      changeWaterState = !changeWaterState;
      if(changeWaterState == true){
        drain_state = false;
      }
      Serial.println(changeWaterState);
      String text  = "{\"change\":" + String(changeWaterState) + "}";
      notifyingPubMqtt(text);
    }
  }

  if (String(topic) == "@msg/kmutnb/cs/smart-hydro1/calibrat-ph") { 
    bool toggle = pH_calibrat_state;
    if(msg.toInt() == 1){
      toggle = !toggle;
    }else{
      return;
    }
      //pH_calibrat_state = msg.toInt();
    if(toggle == 1){
      if(EC_calibrat_state == true){
        EC_calibrat_state = false;
        pH_calibrat_state = msg.toInt();
      }else{
        pH_calibrat_state = msg.toInt();
      }
    }else{
      pH_calibrat_state = msg.toInt();
    }
    String text  = "{\"calibrationPH\":" + String(pH_calibrat_state) + "}";
    notifyingPubMqtt(text);
    
  }

  if (String(topic) == "@msg/kmutnb/cs/smart-hydro1/calibrat-ec") { 
    bool toggle = EC_calibrat_state;
    if(msg.toInt() == 1){
      toggle = !toggle;
    }else{
      return;
    }

    if(toggle == 1){
      if(pH_calibrat_state == true){
        pH_calibrat_state = false;
        EC_calibrat_state = msg.toInt();
      }else{
        EC_calibrat_state = msg.toInt();
      }
    }else{
      EC_calibrat_state = msg.toInt();
    }
    String text  = "{\"calibratEC\":" + String(EC_calibrat_state) + "}";
    notifyingPubMqtt(text);
  }
};

void reconnectBlynk(){ // ฟังก์ชันตรวจสอบสถานะการเชื่อมต่อของ Blynk ในทุกๆ 1 วินาที
  static int errorTime = 0;
  static unsigned long timepoint = 0;
  bool ret;
  if(currentMillis - timepoint >= 25000U){
    timepoint = currentMillis;
    ret = Ping.ping("www.google.com",1);        //ping ESP32 using google
    if (!Blynk.connected()) {
      if(!ret){
        //Serial.println("Ping failed");
        statusBlynk = Blynk.connected();
        errorTime++;
      }else{
        statusBlynk = Blynk.connect(3000UL);
        errorTime++;
        //Serial.println("Pinged");
      }
    // connect to blynk server successful
    } else {
      if(ret){
        statusBlynk = Blynk.connected();
      }else{
        statusBlynk = 0;
        statusBlynk = Blynk.connect(1000UL);
      }
      //Serial.println("Blynk: "+String(Blynk.connected()));
      errorTime = 0;
    }
  }
}


BLYNK_WRITE(V0){ // ฟังก์ชันสำหรับรับเวลาเปิดและเวลาปิดสำหรับใช้ควบคุมไฟปลูกจาก Blynk server โดย datastream คือ V0
  unsigned char week_day;
  TimeInputParam t(param);
  if (t.hasStartTime() && t.hasStopTime() ){
    startHour = t.getStartHour();
    startMinute = t.getStartMinute();
    startSecond = t.getStartSecond();

    stopHour = t.getStopHour();
    stopMinute = t.getStopMinute();
    stopSecond = t.getStopSecond();

    Serial.println(String("Stop Time: ") + t.getStopHour() + ":" + t.getStopMinute() + ":" + t.getStopSecond());
  }else{

  }
}

void functionLcd(){ // เป็นฟังก์ชั่นที่ใช้สำหรับแสดงข้อมูลบนจอ LCD 
    static unsigned long lastSaveTime = 0;
    static unsigned long currentPageTime = 0;
    static int currentPage = 0;
    if (currentMillis - currentPageTime >= 10000U) { //เปลี่ยนหน้าจอ โดยจะนับหน้าจอขึ้นไปอีก 1 แล้วหารเอาเศษด้วย 3 เพื่อทำให้หน้าจอเปลี่ยนไปสลับกัน 3 หน้าจอตามลำดับทุก 10 วินาที
      currentPageTime = currentMillis;
      currentPage++;
      currentPage = currentPage % 3;
    }
    if (currentMillis - lastSaveTime >= 1000U && drain_state == false && changeWaterState == false && pH_calibrat_state == false && EC_calibrat_state == false) {  // โดยกำหนดให้แสดงผลทุกๆ 1 วินาที (1000 มิลลิวินาที) และยังตรวจสอบเงื่อนไข drain_state และ changeWaterState ว่ามีค่าเป็น false หรือไม่ ถ้าเงื่อนไขทุกอย่างเป็นจริง ก็จะแสดงผลบนหน้าจอ LCD ตามคำสั่งที่กำหนดใน switch case ตาม currentPage ที่กำหนดไว้ 
      lastSaveTime = currentMillis;
      // clear the screen
      switch(currentPage) {
        case 0:
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Temp  :" + String(temp_room) + " c");
          lcd.setCursor(0,1);
          lcd.print("humid :" + String(hum_room) + " %");
          lcd.setCursor(0,2);
          lcd.print("EC    :" + String(lastEC) + " mS/cm");
          lcd.setCursor(0,3);
          lcd.print("pH    :" + String(lastPH));
          break;
        case 1:
          lcd.clear();
          if(statusMqtt == 1 ){
            lcd.setCursor(0,0);
            lcd.print("Mqtt: connected");
          }
          else{
            lcd.setCursor(0,0);
            lcd.print("Mqtt: not connected");
          }
          if(statusBlynk == 1 ){
            lcd.setCursor(0,1);
            lcd.print("Blynk: connected");
          }
          else{
            lcd.setCursor(0,1);
            lcd.print("Blynk: not connected");
          }
          if(statusWifi == 1 ){
            lcd.setCursor(0,2);
            lcd.print("Wifi: connected");
          }
          else{
            lcd.setCursor(0,2);
            lcd.print("Wifi: not connected");
          }
          /*if(statusWifi == 1){
            lcd.setCursor(0,3);
            lcd.print(String(WiFi.localIP()));
          }   */
          break;
        case 2:
          lcd.clear();
          lcd.setCursor(0,0);
          //int nowHour,nowMinute,nowSecond;
          lcd.print(String("RTC:      ") + nowHour + ":" + nowMinute + ":" + nowSecond);
          lcd.setCursor(0,1);
          lcd.print(String("On Time:  ") + startHour + ":" + startMinute + ":" + startSecond);
          lcd.setCursor(0,2);
          lcd.print(String("Off Time: ") + stopHour + ":" + stopMinute + ":" + stopSecond);

          break;
      }
    }
};

void clearSerial2Buffer() { //ฟังก์ชัน clearSerial2Buffer() นี้จะเคลียร์ข้อมูลที่มีอยู่ใน Serial Buffer ของ Serial2 โดยการตั้งค่า timeout ให้เป็นเวลาสั้น ๆ เพื่อที่จะลบข้อมูลขาเข้าที่ถูกส่งมา จากนั้นใช้วงลูป while เพื่ออ่านข้อมูลใน buffer ที่ยังคงมีอยู่และทำการลบทิ้งไป จากนั้นก็ตั้งค่า timeout ให้กลับเป็นเวลาปกติอีกครั้ง
  Serial2.setTimeout(1); // set a small timeout to discard incoming data
  while (Serial2.available()) {
    Serial2.read(); // read any incoming data (which will be discarded)
  }
  Serial2.setTimeout(1000); // set the timeout back to a normal value 
}

void pzemRead(){ //ฟังก์ชันอ่านค่าไฟฟ้าจาก pzem module
  static unsigned long lastSaveTime = 0;
  static int countError;
    if (currentMillis - lastSaveTime >= 10000U) {
      lastSaveTime = currentMillis;
      clearSerial2Buffer(); //ฟังก์ชั่น clearSerial2Buffer() มีหน้าที่ล้างข้อมูลที่เก็บอยู่ใน Serial Buffer

        //Serial.print("Custom Address:");
        //Serial.println(pzem.readAddress(), HEX);

        // Read the data from the sensor
        //อ่านค่าแรงดันไฟฟ้า (voltage) กระแสไฟฟ้า (current) กำลังไฟฟ้า (power) พลังงานไฟฟ้า (energy) ความถี่ (frequency) และแรงดันประสิทธิภาพ (power factor) จากเครื่องวัดพลังงานไฟฟ้า (PZEM-004t) ซึ่งตัวแปรทั้งหมดนี้จะเก็บค่าที่ได้จากการวัดไว้
         voltage = pzem.voltage();
         current = pzem.current();
         power = pzem.power();
         energy = pzem.energy();
         frequency = pzem.frequency();
         pf = pzem.pf();

        // Check if the data is valid
        if(isnan(voltage)){ //ถ้าค่า voltage เป็น NaN (Not a Number) คือไม่สามารถอ่านค่าได้ จะทำการเพิ่มค่า countError และแสดงข้อความ "Error reading voltage" พร้อมกับแสดงจำนวน countError ที่เพิ่มขึ้น
          countError++;
          Serial.println("Error reading voltage: "+ String(countError));
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
        } else { //หากไม่มีข้อผิดพลาดในการอ่านค่าไฟฟ้า
            countError=0;
            
            Unit = energy ;//หาค่าค่าพลังงานโดยกำหนดค่าตัวแปร Unit เท่ากับ energy และคำนวณค่า energyprice โดยใช้ฟังก์ชัน calEnergyPrice()
            if(ft == 0.00){
              ft = 93.49;
            }
            energyprice = calEnergyPrice(Unit,ft);

            //ส่งข้อมูลไปยังแพลตฟอร์ม Blynk โดยใช้ฟังก์ชัน virtualWrite() เพื่อแสดงค่า voltage, current, power, energy, และ frequency บนแอปพลิเคชัน Blynk
            Blynk.virtualWrite(V21, energyprice);
            Blynk.virtualWrite(V23, Unit);
            Blynk.virtualWrite(V5, voltage);
            Blynk.virtualWrite(V6, current);
            Blynk.virtualWrite(V7, power);
            Blynk.virtualWrite(V8, energy);
            Blynk.virtualWrite(V9, frequency);

            //ส่งข้อมูลไปยังแพลตฟอร์ม IoT โดยใช้โมดูล MQTT ด้วยฟังก์ชัน publish() โดยจะเก็บข้อมูลเป็น JSON ซึ่งประกอบไปด้วยค่า voltage, current, power, energy, frequency, Unit และ energyprice
            if (mqtt.connected() == true) {
              mqtt.loop();
              String dataJS = "{\"voltage\":" + String(voltage) + ",\"current\":" + String(current) + ",\"power\":" + String(power) + ",\"energy\":" + String(energy) + ",\"frequency\":" + String(frequency) + ",\"Unit\":" + String(Unit) + ",\"energyprice\":" + String(energyprice) +"}";
              char json[150];
              dataJS.toCharArray(json,dataJS.length()+1);
              mqtt.publish("@msg/kmutnb/cs/smart-hydro1/powerenergy", json);
            } 
            // Print the values to the Serial console
            /*Serial.print("Voltage: ");      Serial.print(voltage);      Serial.println("V");
            Serial.print("Current: ");      Serial.print(current);      Serial.println("A");
            Serial.print("Power: ");        Serial.print(power);        Serial.println("W");
            Serial.print("Energy: ");       Serial.print(energy,3);     Serial.println("kWh");
            Serial.print("Frequency: ");    Serial.print(frequency, 1); Serial.println("Hz");
            Serial.print("PF: ");           Serial.println(pf);
            Serial.println();*/
        }
        if (countError > 200){
          lcdBlynkPrintError("Module energy error");
        }
    }
}

void RTCfunction(){ // ฟังก์ชันอ่านเวลา โดยทุก 1 วินาทีจะทำการอ่านค่าเวลาจาก RTC Module
  static int errorReadRTC;
  static unsigned long lastSaveTime = 0;
  static unsigned long lastSaveTimeError;
  if (currentMillis - lastSaveTime >= 1000U) {
    lastSaveTime = currentMillis;
    DateTime now = RTC.now();
    /*Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(' ');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();*/
    int year = now.year();
    //Serial.println("This year: " + String(year));
    if(year  < 2023 || year  > 2100){ //ถ้าเวลาที่ได้อ่านมาไม่ตรงกับเงื่อนไขที่กำหนด (อยู่ระหว่าง 2023-2100) จะทำการนับจำนวนครั้งที่เกิดข้อผิดพลาดในการอ่าน RTC Module และ ใช้ฟังก์ชัน requestTime(); เพื่อขอเวลาปัจจุบันมาจาก Blynk Server 
      if (currentMillis - lastSaveTimeError >= 10000U ) {
        errorReadRTC++;
        Serial.println("error RTC Module: " + String(errorReadRTC) + " time");
        requestTime();
        lastSaveTimeError = currentMillis;
      }else{
        return;
      }
    }
    else{ //ในกรณีที่เวลาที่อ่านมาจาก RTC อยู่ในช่วงเวลาที่ถูกต้องจะทำการกำหนดค่า RtcState เป็น true และกำหนดค่า nowHour, nowMinute, และ nowSecond จากข้อมูลเวลาที่อ่านได้จาก RTC
      RtcState = true;
      nowHour=now.hour();
      nowMinute=now.minute();
      nowSecond=now.second();
    }
  }
};

void EEPROMfunction(){ //ฟังก์ชัน EEPROMfunction() จะทำการบันทึกข้อมูลทุก ๆ 10 วินาที โดยตำแหน่งของแต่ละตัวแปรจะถูกกำหนดโดยตัวแปร address และถูกเพิ่มขึ้นตามขนาดของแต่ละตัวแปรที่บันทึก
  static unsigned long lastSaveTime = 0;
  if (currentMillis - lastSaveTime >= 10000U) {
    int address = 0;
    lastSaveTime = currentMillis;
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
    EEPROM.write(address, GrowLightControl1);
    address += sizeof(GrowLightControl1);
    EEPROM.write(address, GrowLightControl2);
    address += sizeof(GrowLightControl2);
    EEPROM.write(address, GrowLightControl3);
    address += sizeof(GrowLightControl3);
    EEPROM.write(address, GrowLightControl4);
    address += sizeof(milleHour);
    EEPROM.write(address, milleHour);
    address += sizeof(HourUpdateRTC);
    EEPROM.write(address, HourUpdateRTC); 
    //-----ยังไม่ได้ทดสอบ
    address += sizeof(waterAmount);
    EEPROM.put(address, waterAmount);
    //-----
    address += sizeof(acidVoltage);
    EEPROM.put(address, acidVoltage);
    address += sizeof(neutralVoltage);
    EEPROM.put(address, neutralVoltage);

    //kValue
    address += sizeof(kValue);
    EEPROM.put(address, kValue);

    //ft ไฟฟ้า
    address += sizeof(ft);
    EEPROM.put(address, ft);

    EEPROM.commit();

    /*Serial.println("waterAmount: " + String(waterAmount));
    Serial.println("acidVoltage: " + String(acidVoltage));
    Serial.println("neutralVoltage: " + String(neutralVoltage));
    Serial.println("kValue: " + String(kValue));*/
    //Serial.println("EEPROM Done");
    //work list is done

  }
};

void controlEC(){ //ฟังก์ชันควบคุมปั๊มปุ๋ย AB
  static unsigned long lastSaveTimeMan;
  static unsigned long lastSaveTimeoOff;
  static unsigned long lastSaveTimeOffAuto;
  static bool StateRelayOfEC = false;
  static const unsigned long PUMP_ON_TIME = 3000; // 5 seconds
  static const unsigned long PUMP_OFF_TIME = 120000; // 60 seconds
  static unsigned long last_pump_time = 0; 
  static unsigned long lastPulseTime_A_Solution; //ตัวแปรสำหรับเก็บค่า Pulse ที่ได้จากเซ็นเซอร์ flow ปุ๋ย A ของครั้งลาสุด
  static int empty_A_count = 0; //ตัวแปรสำหรับเก็บค่าเมื่อเซ็นเซอร์ไม่สามารถตรวจจับการไหลของปุ๋ย A ได้
	static unsigned long lastPulseTime_B_Solution; //ตัวแปรสำหรับเก็บค่า Pulse ที่ได้จากเซ็นเซอร์ flow ปุ๋ย B ของครั้งลาสุด
  static int empty_B_count = 0; //ตัวแปรสำหรับเก็บค่าเมื่อเซ็นเซอร์ไม่สามารถตรวจจับการไหลของปุ๋ย B ได้

  static bool re_off_pump = false;
  static int count_re_off = 0;

  //ค่า ecAuto และ ecMan ถูกกำหนดโดยการเรียกใช้เมธอด get_status() เพื่ออ่านค่าสวิตช์จากตู้ควบคุม
  ecAuto = swAutoEC.get_status();
  ecMan = swManEC.get_status();

  // ส่วนนี้เป็นการตรวจสอบเงื่อนไขว่าสวิตช์ควบคุม EC ที่ตู้บิดเปิดโหมด off หรือไม่ ถ้าใช้จะปั๊ม EC และกำหนดตัวแปร stirPumpAB ให้เท่ากับ LOW เพื่อไว้ใช้ควบปั๊มกวน และรีเซ็ทค่า empty_A_count , empty_B_count
  if((ecAuto == 1)&&(ecMan == 1)){ // swEcModeOff
    // Rtu relay off case 0
    AdjustEcState = 0;
    stirPumpAB = LOW;
    last_pump_time = 0;
    empty_A_count = 0;
    empty_B_count = 0;
    count_re_off = 0;
    re_off_pump = true;
    if(StateRelayOfEC != false){
      relayRtu(2);
      StateRelayOfEC = false;
    }
    if (currentMillis - lastSaveTimeoOff >= 120000U) {
      relayRtu(2);
      StateRelayOfEC = false;
      lastSaveTimeoOff = currentMillis;
    }
  }
  // ส่วนนี้เป็นการตรวจสอบเงื่อนไขว่าสวิตช์ควบคุม EC ที่ตู้บิดเปิดโหมด Auto หรือไม่ 
  else if ((ecAuto == 0)&&(ecMan == 1)){ 

    // หากค่า EC ปัจจุบันน้อยกว่าค่า EC ต่ำสุดที่ตั้งเอาไว้ (ecLow) และสถานะของเซ็นเซอร์ EC เป็น 1 และไม่มีการเปลี่ยนสถานะของการใช้น้ำ (changeWaterState) และถังไม่ว่าง (empty_tank) และไม่มีการระบายน้ำ (drain_state) จะมีการปรับแต่งสถานะของการปรับค่า EC (AdjustEcState) เป็น 1 เพื่อนำไปใช้สั่งปั๊ม
    if(lastEC < ecLow && StatusOfECsensor == 1 && changeWaterState == false && empty_tank == false && drain_state == false){ // pump AB solution to up ec value in water
      AdjustEcState = 1;
    }
    //หากค่า EC อยู่ระหว่างค่า ต่ำสุดที่ตั้งเอาไว้ ecLow และค่า สูงสุดที่ตั้งเอาไว้ ecHigh และสถานะของเซ็นเซอร์ EC เป็น 1 จะไม่มีการปรับแต่งสถานะของการปรับค่า EC (AdjustEcState) เพราะค่า EC อยู่ในช่วงที่เหมาะสม
    else if((lastEC >= ecLow)&&(lastEC <= ecHigh) && StatusOfECsensor == 1 ){ // good Ec Value
      AdjustEcState = 0;
    }
    else{ // สถานะของเซ็นเซอร์ EC เป็น 0 จะไม่มีการปรับแต่งสถานะของการปรับค่า EC (AdjustEcState) 
      AdjustEcState = 0;
    }

    //ถ้า AdjustEcState เป็น 1 จะทำการปั๊มปุ๋ย AB สลับปั๊มกับหยุดปั๊ม โดยเวลาที่ปั๊มจะเท่ากับค่า PUMP_ON_TIME และ ช่วงที่หยุดปั๊มจะเท่ากับ PUMP_OFF_TIME
    if(AdjustEcState == 1){
      stirPumpAB = HIGH; //กำหนดให้สถานะปั๊มกวนปุ๋ยภายในถัง
      if (StateRelayOfEC && (currentMillis - last_pump_time >= PUMP_ON_TIME)) {
        // turn off pump
        relayRtu(2);
        StateRelayOfEC = false;
        re_off_pump = false;
        count_re_off = 0;
        if(lastPulseTime_A_Solution+100 >= pulse_A_Solution){ //หากค่าพัลส์ที่ได้จากเซ็นเซอร์ตรวจสอบการนไหลครั้งก่อน lastPulseTime_A_Solution+100 มากกว่าค่าพัลส์ล่าสุด pulse_A_Solution จะบวกค่าให้กับตัวแปร empty_A_count
          Serial.println("A sulution is not flow: " +String(lastPulseTime_A_Solution));
          empty_A_count++;
        }
        if(lastPulseTime_B_Solution+100 >= pulse_B_Solution){ //หากค่าพัลส์ที่ได้จากเซ็นเซอร์ตรวจสอบการนไหลครั้งก่อน lastPulseTime_B_Solution+100 มากกว่าค่าพัลส์ล่าสุด pulse_B_Solution จะบวกค่าให้กับตัวแปร empty_B_count
          Serial.println("B sulution is not flow:" + String(lastPulseTime_B_Solution));
          empty_B_count++;
        }else{ // ถ้าเซ็นเซอร์สามารถอ่านค่าได้จะรีเซ็ตค่าที่นับมา
          empty_A_count = 0;
          empty_B_count = 0;
        }
        // หากตรวจพบว่าไม่มีการไหลของสาร A และ B Solution
        if( empty_A_count >= 2 && empty_A_count >= 2){ // ถ้าไม่สามารถตรวจสอบการไหลได้มากกว่าหรือเท่ากับ 2 จะทำการส่งข้อมูลผ่าน MQTT ที่ระบุว่า AB Solution ไม่ไหล
          String text  = "{\"abSolutionEmptyTime\":" + String(empty_B_count) + "}";
          notifyingPubMqtt(text);
        }
        last_pump_time = currentMillis;

      } else if (!StateRelayOfEC && (currentMillis - last_pump_time >= PUMP_OFF_TIME)) {
        // turn on pump
        lastPulseTime_A_Solution = pulse_A_Solution;
        lastPulseTime_B_Solution = pulse_B_Solution;
        relayRtu(1);
        StateRelayOfEC = true;
        re_off_pump = true;
        count_re_off = 0;
        last_pump_time = currentMillis;
      }else{

        if (currentMillis - lastSaveTimeOffAuto >= 1000U && re_off_pump == false && count_re_off <= 3) {
          count_re_off++;
          relayRtu(2);
          StateRelayOfEC = false;
          lastSaveTimeOffAuto = currentMillis;
        }  

      }
    }
    else{ //ถ้า AdjustEcState เป็น 0 จะทำการปิดปั๊มถ้าปั๊มเปิดอยู่
      
      // Relay RTU Off
      last_pump_time = 0;
      empty_A_count = 0;
      empty_B_count = 0;
      stirPumpAB = LOW; //ปั๊มกวนปิด
      count_re_off = 0;
      re_off_pump = true;

      if(StateRelayOfEC != false){
        relayRtu(2);
        StateRelayOfEC = false;
      }
      
      if (currentMillis - lastSaveTimeOffAuto >= 60000U) {
        relayRtu(2);
        StateRelayOfEC = false;
        lastSaveTimeOffAuto = currentMillis;
      }    
    }

  }
  // ส่วนนี้เป็นการตรวจสอบเงื่อนไขว่าสวิตช์ควบคุม EC ที่ตู้บิดเปิดโหมด Man หรือไม่ 
  //ถ้าใช้จะสั่งให้ปั๊มกวนทำงาน stirPumpAB = HIGH และ สั่งให้ปั๊มดูดปุ๋ยทำงาน
  else if ((ecAuto == 1)&&(ecMan == 0)){ //swEcModeMan
  AdjustEcState = 0;
    // Rtu relay on
    stirPumpAB = HIGH;
    last_pump_time = 0;
    empty_A_count = 0;
    empty_B_count = 0;
    count_re_off = 0;
    re_off_pump = true;
    if(StateRelayOfEC != true){
      relayRtu(1);
      StateRelayOfEC = true;
    }
    
    if (currentMillis - lastSaveTimeMan >= 120000U) {
      StateRelayOfEC= true;
      relayRtu(1);
      lastSaveTimeMan = currentMillis;
      //Serial.println("sw man on");
    }
  }
  else{
    // serial error
    // Rtu relay off
    Serial.println("sw error");
  }
}

void controlPH(){ //ฟังก์ชันควบคุมปั๊ม pH Down
  static unsigned long lastSaveTimeMan;
  static unsigned long lastSaveTimeoOff;
  static unsigned long lastSaveTimeOffElseAuto;
  static unsigned long last_pump_time = 0; 
  const int PUMP_ON_TIME = 3000;      // Relay on time in milliseconds
  const int PUMP_OFF_TIME = 120000;   // Relay off time in milliseconds
  static bool relayStatePH = false; //ตัวแปรสำหรับเก็บค่าสถานะปัจจุบันของ Relay
  static unsigned long lastPulseTime_pH_Solution; //ตัวแปรสำหรับเก็บค่า Pulse ที่ได้จากเซ็นเซอร์ flow สารลด pH ของครั้งล่าสุด
  static int empty_ph_count = 0; //ตัวแปรสำหรับเก็บค่าเมื่อเซ็นเซอร์ตรวจการไหลไม่สามารถตรวจจับการไหลของสารลด pH ได้

  static bool re_off_pump = false;
  static int count_re_off = 0;

  //ค่า phAuto และ phMan ถูกกำหนดโดยการเรียกใช้เมธอด get_status() เพื่ออ่านค่าสวิตช์จากตู้ควบคุม 
  phAuto = swAutoPH.get_status();
  phMan = swManPH.get_status();
  
  // ส่วนนี้เป็นการตรวจสอบเงื่อนไขว่าสวิตช์ควบคุม pH ที่ตู้บิดเปิดโหมด off หรือไม่ ถ้าใช่จะปั๊ม pH และกำหนดตัวแปร stirPumpPh ให้เท่ากับ LOW เพื่อไว้ใช้ควบปั๊มกวน และรีเซ็ทค่าต่างๆ
  // Select switch
  if((phAuto == 1)&&(phMan == 1)){ // swphModeOff
    stirPumpPh = LOW;
    last_pump_time = 0;
    empty_ph_count = 0;
    count_re_off = 0;
    re_off_pump = true;

    // Rtu relay off case 0
    if(relayStatePH != LOW){
      relayRtu(4);
      relayStatePH = LOW;
      
    }
    
    if (currentMillis - lastSaveTimeoOff >= 60000U) {
      relayRtu(4);
      relayStatePH = LOW;

      lastSaveTimeoOff = currentMillis;
      //Serial.println("sw man on");
    }
  }

  // ส่วนนี้เป็นการตรวจสอบเงื่อนไขว่าสวิตช์ควบคุม pH ที่ตู้บิดเปิดโหมด Auto หรือไม่ 
  else if ((phAuto == 0)&&(phMan == 1)){ //swphModeAuto
    //คำนวณ
    // หากค่า pH ปัจจุบันมากกว่าค่า pH ต่ำสุดที่ตั้งเอาไว้ (phHigh) และสถานะของเซ็นเซอร์ pH เป็น 1 และไม่มีการเปลี่ยนสถานะของการใช้น้ำ (changeWaterState) และถังไม่ว่าง (empty_tank) และไม่มีการระบายน้ำ (drain_state) จะมีการปรับแต่งสถานะของการปรับค่า pH (AdjustPHState) เป็น 1 เพื่อนำไปใช้สั่งปั๊ม
    if((lastPH > phHigh)&&(AdjustEcState != 1) && StatusOfPHsensor == 1 && changeWaterState == false && empty_tank == false && drain_state == false){ // pump pH solution to up pH value in water //anything else not auto ph with ec man
      AdjustPHState = 1;
    }
    //หากค่า pH อยู่ระหว่างค่า ต่ำสุดที่ตั้งเอาไว้ pHLow และค่า สูงสุดที่ตั้งเอาไว้ phHigh และสถานะของเซ็นเซอร์ pH เป็น 1 จะไม่มีการปรับแต่งสถานะของการปรับค่า pH (AdjustpHState) เพราะค่า pH อยู่ในช่วงที่เหมาะสม
    else if((lastPH >= phLow)&&(lastPH <= phHigh) && StatusOfPHsensor == 1){ // good pH Value
      AdjustPHState = 0;
    }
    else{ // สถานะของเซ็นเซอร์ pH เป็น 0 จะไม่มีการปรับแต่งสถานะของการปรับค่า pH (AdjustPHState) 
      AdjustPHState = 0;
    }

    //ถ้า AdjustPHState เป็น 1 จะทำการปั๊มลดค่า pH สลับปั๊มกับหยุดปั๊ม โดยเวลาที่ปั๊มจะเท่ากับค่า PUMP_ON_TIME และ ช่วงที่หยุดปั๊มจะเท่ากับ PUMP_OFF_TIME
    if(AdjustPHState == 1){
      stirPumpPh = HIGH; //pump กวน
      if (relayStatePH && (currentMillis - last_pump_time >= PUMP_ON_TIME)) {  
        // turn off pump
        relayRtu(4); 
        relayStatePH = LOW;
        re_off_pump = false;
        count_re_off = 0;
        last_pump_time = currentMillis;

        if(lastPulseTime_pH_Solution+100 >= pulse_phDownSolution){ //หากค่าพัลส์ที่ได้จากเซ็นเซอร์ตรวจสอบการนไหลครั้งก่อน lastPulseTime_pH_Solution+100 มากกว่าค่าพัลส์ล่าสุด pulse_pHDownSolution จะบวกค่าให้กับตัวแปร empty_ph_count
          Serial.println("pH sulution is not flow");
          empty_ph_count++;
        }else{
          empty_ph_count = 0;
        }
        
        // หากตรวจพบว่าไม่มีการไหลของสาร pH down มากกว่า 2 ครั้ง
        if(empty_ph_count > 2){
          String text  = "{\"pHSolutionEmptyTime\":" + String(empty_ph_count) + "}";
          notifyingPubMqtt(text);
          //Blynk

        }
      
      }else if(!relayStatePH && (currentMillis - last_pump_time >= PUMP_OFF_TIME)){ 
        // turn on pump
        relayRtu(3); 
        relayStatePH = HIGH;
        count_re_off = 0;
        re_off_pump = true;
        last_pump_time = currentMillis;
        lastPulseTime_pH_Solution = pulse_phDownSolution;
      }else{


        if (currentMillis - lastSaveTimeOffElseAuto >= 1000U && re_off_pump == false && count_re_off <= 3) {
        count_re_off++;
        relayRtu(4);
        relayStatePH = LOW;
        lastSaveTimeOffElseAuto = currentMillis;
      }

      }
    }
    else{// Relay RTU Off 
      last_pump_time = 0;
      empty_ph_count = 0;
      stirPumpPh = LOW;
      count_re_off = 0;
      re_off_pump = true;
      if(relayStatePH != LOW){
        relayRtu(4);
        relayStatePH = LOW;
      }
      
      if (currentMillis - lastSaveTimeOffElseAuto >= 120000U) {
        relayRtu(4);
        relayStatePH = LOW;
        lastSaveTimeOffElseAuto = currentMillis;
      }
    }
  }
  // ส่วนนี้เป็นการตรวจสอบเงื่อนไขว่าสวิตช์ควบคุม pH ที่ตู้บิดเปิดโหมด Man หรือไม่ 
  //ถ้าใช่จะสั่งให้ปั๊มกวนทำงาน stirPumpPh = HIGH และ สั่งให้ปั๊มสารลด pH
  else if ((phAuto == 1)&&(phMan == 0)){ //swphModeMan
    stirPumpPh = HIGH;
    last_pump_time = 0;
    empty_ph_count = 0;
    count_re_off = 0;
    re_off_pump = true;
    // Rtu relay on
    if(relayStatePH != HIGH){
      Serial.print("relayState mode MAN: ");
      Serial.println(relayStatePH);
      relayRtu(3);
      relayStatePH = HIGH;
    }
    
    if (currentMillis - lastSaveTimeMan >= 60000U) {
      relayStatePH = HIGH;
      Serial.print("relayState mode MAN: ");
      Serial.println(relayStatePH);
      relayRtu(3);
      lastSaveTimeMan = currentMillis;
      //Serial.println("sw man on");
    }
  }
  else{
    // serial error
    // Rtu relay off
    Serial.println("sw error");
  }
}

void StirPump(){ 
  static bool stateStirPump = LOW; //ตัวแปรสำหรับเก็บสถานะปั๊มกวน

  if(stirPumpPh == LOW && stirPumpAB == LOW){ //ถ้าปั๊ม pH ไม่ได้เติม หรือ EC ไม่ได้เติมจะสั่งปิดปั๊มกวน
    if(stateStirPump != LOW){ //ถ้าปั๊มกวนไม่ได้มีสถานะเท่ากับ low จะสั่งให้ปิด และ เปลี่ยนสถานะเป็น low
      stateStirPump = LOW;
      //Serial.println("stir pump off TRICK");
      relayRtu(6); //สั่งปิดปั๊มกวน
    }
    //Serial.println("stir pump off");
  }
  else if (stirPumpPh == HIGH && stirPumpAB == LOW){ //ถ้าปั๊ม pH ไม่ได้เติมแต่ EC กำลังเติมจะสั่งเปิดปั๊มกวน
    if(stateStirPump != HIGH){ //ถ้าปั๊มกวนไม่ได้มีสถานะเท่ากับ high จะสั่งให้เปิด และ เปลี่ยนสถานะเป็น high
      stateStirPump = HIGH;
      //Serial.println("stir pump ON TRICK");
      relayRtu(5); //สั่งเปิดปั๊มกวน
    }
    //Serial.println("stir pump ON");

  }
  else if (stirPumpPh == LOW && stirPumpAB == HIGH){
    if(stateStirPump != HIGH){
      stateStirPump = HIGH;
      //Serial.println("stir pump ON TRICK");
      relayRtu(5);
    }
    //Serial.println("stir pump ON");

  }
  else if(stirPumpPh == HIGH && stirPumpAB == HIGH){
    if(stateStirPump != HIGH){
      stateStirPump = HIGH;
      //Serial.println("stir pump ON TRICK");
      relayRtu(5);
    }
    //Serial.println("stir pump ON");
  }
  else{
    Serial.println("stir pump error");
  }


}

void controlGrowLight(){ //switch mode --> timer --> Return state of relay
  static unsigned long lastSaveTimeoOff;
  static unsigned long lastSaveTimeoOn;
  static unsigned long lastSaveTimeoAuto;
  GLAuto = swAutoGL.get_status();
  GLMan = swManGL.get_status();

  // Select switch
  if((GLAuto == 1)&&(GLMan == 1)){ // sw grow light Mode off
    if(growLigh1 != LOW){
      relayRtu(8);
      growLigh1 = LOW;
    }
    if(growLigh2 != LOW){
      relayRtu(10);
      growLigh2 = LOW;
    }
    if(growLigh3 != LOW){
      relayRtu(12);
      growLigh3 = LOW;
    }
    if(growLigh4 != LOW){
      relayRtu(14);
      growLigh4 = LOW;
    }
    
    if (currentMillis - lastSaveTimeoOff >= 400000U) {
      relayRtu(8);// off command growLigh1
      growLigh1 = LOW;
      relayRtu(10);// off command growLigh2
      growLigh2 = LOW;
      relayRtu(12);// off command growLigh3
      growLigh3 = LOW;
      relayRtu(14);// off command growLigh4
      growLigh4 = LOW;
      lastSaveTimeoOff = currentMillis;
      //Serial.println("sw man on");
    }
  }
  //AUTO MODE
  else if ((GLAuto == 0)&&(GLMan == 1)){ 

    startTimeInSeconds = abs((startHour * 60 * 60) + (startMinute * 60) + startSecond);
    stopTimeInSeconds = abs((stopHour * 60 * 60) + (stopMinute * 60) + stopSecond);
    currentTimeInSeconds = abs((nowHour * 60 * 60) + (nowMinute * 60) + nowSecond);

    if(currentTimeInSeconds >= startTimeInSeconds && currentTimeInSeconds < stopTimeInSeconds || RtcState == false){
      timeonGL = 1;
    }
    else{
      timeonGL = 0;
    }

    
    if(timeonGL == 1 && GrowLightControl1 == 1 && growLigh1 == LOW){
      relayRtu(7);
      growLigh1 = HIGH;
      //You can change button labels from hardware with
      Blynk.virtualWrite(V10, growLigh1);
    }
    else if(timeonGL == 1 && GrowLightControl1 == 0 && growLigh1 == HIGH){
      relayRtu(8);
      growLigh1 = LOW;
      Blynk.virtualWrite(V10, growLigh1);
    }else if(timeonGL == 0){
      if(growLigh1 == HIGH){
        relayRtu(8);
        growLigh1 = LOW;
        Blynk.virtualWrite(V10, growLigh1);
      }
    }

    //------------------------
    if(timeonGL == 1 && GrowLightControl2 == 1 && growLigh2 == LOW){
      relayRtu(9);
      growLigh2 = HIGH;
      //You can change button labels from hardware with
      Blynk.virtualWrite(V11, growLigh2);
    }
    else if(timeonGL == 1 && GrowLightControl2 == 0 && growLigh2 == HIGH){
      relayRtu(10);
      growLigh2 = LOW;
      Blynk.virtualWrite(V11, growLigh2);
    }else if(timeonGL == 0){
      if(growLigh2 == HIGH){
        relayRtu(10);
        growLigh2 = LOW;
        Blynk.virtualWrite(V11, growLigh2);
      }
    }

    if(timeonGL == 1 && GrowLightControl3 == 1 && growLigh3 == LOW){
      relayRtu(11);
      growLigh3 = HIGH;
      Blynk.virtualWrite(V12, growLigh3);
    }else if(timeonGL == 1 && GrowLightControl3 == 0 && growLigh3 == HIGH){
      relayRtu(12);
      growLigh3 = LOW;
      Blynk.virtualWrite(V12, growLigh3);
    }else if(timeonGL == 0){
      if(growLigh3 == HIGH){
        relayRtu(12);
        growLigh3 = LOW;
        Blynk.virtualWrite(V12, growLigh3);
      }
    }
    
    if(timeonGL == 1 && GrowLightControl4 == 1 && growLigh4 == LOW){
      relayRtu(13);
      growLigh4 = HIGH;
      Blynk.virtualWrite(V13, growLigh4);
    }else if(timeonGL == 1 && GrowLightControl4 == 0 && growLigh4 == HIGH){
      relayRtu(14);
      growLigh4 = LOW;
      Blynk.virtualWrite(V13, growLigh4);
    }else if(timeonGL == 0){
      if(growLigh4 == HIGH){
        relayRtu(14);
        growLigh4 = LOW;
        Blynk.virtualWrite(V13, growLigh4);
      }

    }
    
  

  }
  else if ((GLAuto == 1)&&(GLMan == 0)){ //sw grow light Mode Man
    if(growLigh1 != HIGH){
      relayRtu(7);
      growLigh1 = HIGH;
    }
    if(growLigh2 != HIGH){
      relayRtu(9);
      growLigh2 = HIGH;
    }
    if(growLigh3 != HIGH){
      relayRtu(11);
      growLigh3 = HIGH;
    }
    if(growLigh4 != HIGH){
      relayRtu(13);
      growLigh4 = HIGH;
    }

    if (currentMillis - lastSaveTimeoOn >= 400000U) {
      relayRtu(7);// on command growLigh1
      growLigh1 = HIGH;
      relayRtu(9);// on command growLigh2
      growLigh2 = HIGH;
      relayRtu(11);// on command growLigh3
      growLigh3 = HIGH;
      relayRtu(13);// on command growLigh4
      growLigh4 = HIGH;
      lastSaveTimeoOn = currentMillis;
      //Serial.println("sw man on");
    }

  }
  else{
    // serial error
    // Rtu relay off
    Serial.println("sw GrowLight error");
  }

  
}

void controlWaterPump(){ //switch mode --> Return state of relay
  static unsigned long lastSaveTimeHigh;
  static unsigned long lastSaveTimeLow;
  static bool pump_state = LOW;
  debouncer.update();
  
  // กำหนดเงื่อนไขให้โค้ดโปรแกรมในวงเล็บปีกกาทำงานเมื่อสถานะปุ่มกดเปลี่ยนจาก HIGH เป็น LOW โดยเช็คจากฟังก์ชั่น fell()
  // หากต้องการเช็คสถานะจาก LOW เป็น HIGH ให้แทนที่ฟังก์ชั่น fell() ด้วยฟังก์ชั่น rose()
  if ( debouncer.fell() ) { 
    mainWaterPump  = !mainWaterPump ; // สลับสถานะ
    //Serial.println("-----------switch pump con box-----------");
  }
  //on
  if (mainWaterPump == 1 && changeWaterState == false && empty_tank == false && drain_state == false ) { // flowwing is status of health from water main flowing is good = 1 , bad = 0
      if(pump_state == LOW){
        relayRtu(15);
        pump_state = HIGH;
        Blynk.virtualWrite(V14,mainWaterPump);
      }
      //Serial.println("mainWaterPump: " + String(mainWaterPump));
  }
  //off
  else{
    if(pump_state == HIGH){
      relayRtu(16);
      pump_state = LOW;
      Blynk.virtualWrite(V14,mainWaterPump);
    }
  }
}



//-----------------relay 15 เสีย เปลี่ยนเป็น 11
void relayRtu(int condition){ // manage state of relay on/off ch.
  switch(condition) {
    
    case 1: //sending conmand relay on ec ch
      //relay 50-100
      Serial2.write(ON_RTU1, 8);
      Serial.println("pump ec on");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
    case 2: //sending conmand relay off ec ch
      
      //relay 50-100
      Serial2.write(OFF_RTU1, 8);
      Serial.println("pump ec off");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
    case 3: //sending conmand relay on ph ch
      
      //relay 50-100
      Serial2.write(ON_RTU2, 8);
      Serial.println("pump pH on");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
    case 4: //sending conmand relay off ph ch
      
      //relay 50-100
      Serial2.write(OFF_RTU2, 8);
      Serial.println("pump pH off");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;

    case 5: //sending conmand relay on pumpกวน
      
      //relay 50-100
      Serial2.write(ON_RTU3, 8);
      Serial.println("stir pump on");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
    case 6: //sending conmand relay off  pumpกวน
      
      //relay 50-100
      Serial2.write(OFF_RTU3, 8);
      delay(50);
      Serial.println("stir pump off");
      Serial2.flush();
      //Serial2.flushReceive();
      break;

    case 7: //sending conmand relay on gl1
      
      //relay 50-100
      Serial2.write(ON_RTU16, 8);
      Serial.println("gl 1 on");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
    case 8: //sending conmand relay off  gl1
      
      //relay 50-100
      Serial2.write(OFF_RTU16, 8);
      Serial.println("gl 1 off");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
    case 9: //sending conmand relay on gl2
      
      //relay 50-100
      Serial2.write(ON_RTU11, 8);
      Serial.println("gl 2 on");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
    case 10: //sending conmand relay off  gl2
      
      //relay 50-100
      Serial2.write(OFF_RTU11, 8);
      Serial.println("gl 2 off");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
    case 11: //sending conmand relay on gl3
      
      //relay 50-100
      Serial2.write(ON_RTU14, 8);
      Serial.println("gl 3 on");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
    case 12: //sending conmand relay off  gl3
      
      //relay 50-100
      Serial2.write(OFF_RTU14, 8);
      Serial.println("gl 3 off");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
    case 13: //sending conmand relay on gl4
      
      //relay 50-100
      Serial2.write(ON_RTU13, 8);
      Serial.println("gl 4 on");
      delay(50);
      Serial2.flush();
     // Serial2.flushReceive();
      break;
    case 14: //sending conmand relay off  g14
      
      //relay 50-100
      Serial2.write(OFF_RTU13, 8);
      Serial.println("gl 4 off");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
    
    case 15: //sending conmand relay on คำสั่งปิดปั๊มน้ำขึ้นรางปลูก
      //relay 50-100
      Serial2.write(ON_RTU12, 8);
      Serial.println("water pump on");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
    case 16: //sending conmand relay off  คำสั่งปิดปั๊มน้ำขึ้นรางปลูก
      //relay 50-100
      Serial2.write(OFF_RTU12, 8);
      Serial.println("water pump off");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
    case 17: //คำสั่งเปิดวาล์วน้ำเข้า ch 4 on
      Serial2.write(ON_RTU4, 8);
      Serial.println("refill valve on");
      delay(50);
      Serial2.flush();
      break;
    case 18: //คำสั่งปิดวาล์วน้ำเข้า ch 4 off
      Serial2.write(OFF_RTU4, 8);
      Serial.println("refill valve off");
      delay(50);
      Serial2.flush();
      break;
    case 19: //คำสั่งเปิดวาล์วน้ำออก ch 5 on
      Serial2.write(ON_RTU5, 8);
      Serial.println("releast valve on");
      delay(50);
      Serial2.flush();
      break;
    case 20: //คำสั่งปิดวาล์วน้ำออก ch 5 off
      Serial2.write(OFF_RTU5, 8);
      Serial.println("releast valve off");
      Serial2.flush();
      delay(50);
      break;



    //---------------------------------------------------------------------
    case 21: //คำสั่งเปิดวาล์วน้ำกั้นทางออก ch 6 on
      Serial2.write(ON_RTU6, 8);
      Serial.println("cut water line on");
      delay(50);
      Serial2.flush();
      break;
    case 22: //คำสั่งปิดวาล์วน้ำกั้นทางออก ch 6 off
      Serial2.write(OFF_RTU6, 8);
      Serial.println("cut water line off");
      delay(50);
      Serial2.flush();
      break;
  }
}

void requestTime(){
  Blynk.sendInternal("rtc","sync"); //using when want to get time clock
}

BLYNK_WRITE(InternalPinRTC){
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013
  unsigned long blynkTime = param.asLong();
  int dayBlynk,monthBlynk,yearBlynk,weekdayBlynk,nowSecondBlynk,nowMinuteBlynk,nowHourBlynk; //18 02 2023 6
  if (blynkTime >= DEFAULT_TIME) 
  {
    RtcState = true;
    setTime(blynkTime);
    dayBlynk = day();
    monthBlynk = month();
    yearBlynk = year();
    weekdayBlynk = weekday();
    nowHourBlynk = hour();
    nowMinuteBlynk = minute();
    nowSecondBlynk = second();
    nowHour = nowHourBlynk; //กำหนดเวลานี้ไปใช้คำนวณเวลาเปิด-ปิดไฟปลูก
    nowMinute = nowMinuteBlynk; //กำหนดเวลานี้ไปใช้คำนวณเวลาเปิด-ปิดไฟปลูก
    nowSecond = nowSecondBlynk; //กำหนดเวลานี้ไปใช้คำนวณเวลาเปิด-ปิดไฟปลูก
    Serial.println("year from blynk: " + String(yearBlynk));
    settime(yearBlynk,monthBlynk,dayBlynk,weekdayBlynk,nowHourBlynk,nowMinuteBlynk,nowSecondBlynk);
  }
}

void settime(byte Year,byte Month,byte Date,byte DoW,byte Hour,byte Minute,byte Second){ 
  static int countSetError;
  Clock.setYear(Year);
  Clock.setMonth(Month);
  Clock.setDate(Date);
  Clock.setDoW(DoW);
  Clock.setHour(Hour);
  Clock.setMinute(Minute);
  Clock.setSecond(Second);
  countSetError++;
  Serial.println("Time update to RTC module done with " + String(countSetError) + " time.");
}

BLYNK_WRITE(V10){//รับค่าเปิด-ปิดไฟ1
  GrowLightControl1 = param.asInt();
  Serial.println(GrowLightControl1);
}
BLYNK_WRITE(V11){//รับค่าเปิด-ปิดไฟ2
  GrowLightControl2 = param.asInt();
  Serial.println(GrowLightControl2);

}
BLYNK_WRITE(V12){//รับค่าเปิด-ปิดไฟ3
  GrowLightControl3 = param.asInt();
  Serial.println(GrowLightControl3);
}
BLYNK_WRITE(V13){//รับค่าเปิด-ปิดไฟ4
  GrowLightControl4 = param.asInt();
  Serial.println(GrowLightControl4);
}
BLYNK_WRITE(V14){ //main pump
  mainWaterPump = param.asInt();
  Serial.println("pump blynk :" + String(mainWaterPump));
}
BLYNK_WRITE(V17){//รับค่า pH ขั้นต่ำ
  phLow = param.asFloat();

}
BLYNK_WRITE(V18){//รับค่า pH ขั้นสูง
  phHigh = param.asFloat();
}
BLYNK_WRITE(V19){//รับค่า EC ขั้นต่ำ
  ecLow = param.asFloat();
}
BLYNK_WRITE(V20){//รับค่า EC ขั้นสูง
  ecHigh = param.asFloat();
}
BLYNK_WRITE(V22){//รับค่า ft ไฟฟ้า
  ft = param.asFloat();
}
BLYNK_WRITE(V26){//รับค่าการเปิดฟังก์ชัน check flow
  checkflow_ = param.asInt();
}
BLYNK_WRITE(V27){//รับค่าการเปิดฟังก์ชันเปลี่ยนน้ำ
  changeWaterState = param.asInt();
  if(changeWaterState == true){
        drain_state = false;
  }
}
BLYNK_WRITE(V31){//รับค่าการเปิดฟังก์ชันถ่ายน้ำ
  if(param.asInt() == 1){
    drain_state = true;
    //off other function about water
    changeWaterState = false;
  }
  else{
    drain_state = false;
  }
}
BLYNK_WRITE(V28){//รับค่าการรีเซ็ทค่าไฟฟ้า
  int reset = param.asInt();
  if(reset == 1){
    //clearSerial2Buffer();
    pzem.resetEnergy();
    Blynk.virtualWrite(V28,1);
  }
}

BLYNK_WRITE(V33){//รับค่าการเปิดฟังก์ชัน calibrat pH
  if(param.asInt() == 1){
    if(EC_calibrat_state == false){
      pH_calibrat_state = param.asInt();
    }else{
      EC_calibrat_state == false;
      pH_calibrat_state = param.asInt();
    }
  }else{
    pH_calibrat_state = param.asInt();
  }
}
BLYNK_WRITE(V34){//รับค่าการเปิดฟังก์ชัน calibrat EC
  //EC_calibrat_state = param.asInt();
  if(param.asInt() == 1){
    if(pH_calibrat_state == true){
      pH_calibrat_state = false;
      EC_calibrat_state = param.asInt();
    }else{
      EC_calibrat_state = param.asInt();
    }
  }else{
    EC_calibrat_state = param.asInt();
  }
}

void lcdBlynkPrint(){
  //x = position symbol 0 -15 , y = line 0,1
  if(drain_state == false && changeWaterState == false && pH_calibrat_state == false && EC_calibrat_state == false){
    lcdBlynk.clear();
    const String text = String("RTC:") + nowHour + ":" + nowMinute;
    const String text2 =  String("On Time : ") + startHour + ":" + startMinute;
    const String text3 =  String("Off Time: ") + stopHour + ":" + stopMinute;
    //Serial.println(text);

    lcdBlynk.clear(); //Use it to clear the LCD Widget
    lcdBlynk.print(0, 0, text2); // use: (position X: 0-15, position Y: 0-1, "Message you want to print")
    lcdBlynk.print(0, 1, text3); // use: (position X: 0-15, position Y: 0-1, "Message you want to print")
    //lcdBlynk.print(0, 2, text);
  }
}

void lcdBlynkPrintError(String text){
  //x = position symbol 0 -15 , y = line 0,1
  lcdBlynk.clear(); //Use it to clear the LCD Widget
  lcdBlynk.print(0, 0, text); // use: (position X: 0-15, position Y: 0-1, "Message you want to print")
}

float calEnergyPrice(float Unit , float ft) {
  float total = 0;
  if (Unit <= 150) {
    float Rate15 = 2.3488; //แรก (หน่วยที่ 1-15) หน่วยละ 2.3488 บาท
    float Rate25 = 2.9882; //10 หน่วยต่อไป (หน่วยที่ 16-25) หน่วยละ 2.9882 บาท
    float Rate35 = 3.2405; //10 หน่วยต่อไป (หน่วยที่ 26-35) หน่วยละ 3.2405 บาท
    float Rate100 = 3.6237; //65 หน่วยต่อไป (หน่วยที่ 36-100) หน่วยละ 3.6237 บาท
    float Rate150 = 3.7171; //50 หน่วยต่อไป (หน่วยที่ 101-150) หน่วยละ 3.7171 บาท

    if (Unit >= 0.01) total += _min(Unit, 15) * Rate15; //ตรวจสอบว่าจำนวนหน่วยไฟฟ้า (Unit) ที่รับเข้ามามีค่ามากกว่าหรือเท่ากับ 6 หรือไม่ ถ้ามีจะทำการคำนวณราคาไฟฟ้าในช่วงหน่วยไฟฟ้า 6-15 โดยใช้เรทและบวกผลลัพธ์ลงในตัวแปร total
    if (Unit >= 16) total += _min(Unit - 15, 10) * Rate25; //ตรวจสอบว่าจำนวนหน่วยไฟฟ้า (Unit) ที่รับเข้ามามีค่ามากกว่าหรือเท่ากับ 16 หรือไม่ ถ้าใช่จะทำการคำนวณราคาไฟฟ้าในช่วงหน่วยไฟฟ้า 16-25 ราคารเรท Rate25 และบวกผลลัพธ์ลงในตัวแปร total
    if (Unit >= 26) total += _min(Unit - 25, 10) * Rate35; //ตรวจสอบว่าจำนวนหน่วยไฟฟ้า (Unit) ที่รับเข้ามามีค่ามากกว่าหรือเท่ากับ 26 หรือไม่ ถ้าใช่จะทำการคำนวณราคาไฟฟ้าในช่วงหน่วยไฟฟ้า 26-25 ราคารเรท Rate25 และบวกผลลัพธ์ลงในตัวแปร total
    if (Unit >= 36) total += _min(Unit - 35, 65) * Rate100; //ตรวจสอบว่าจำนวนหน่วยไฟฟ้า (Unit) ที่รับเข้ามามีค่ามากกว่าหรือเท่ากับ 36 หรือไม่ ถ้าใช่จะทำการคำนวณราคาไฟฟ้าในช่วงหน่วยไฟฟ้า 36-65 ราคารเรท Rate65 และบวกผลลัพธ์ลงในตัวแปร total
    if (Unit >= 101) total += _min(Unit - 100, 50) * Rate150; //ตรวจสอบว่าจำนวนหน่วยไฟฟ้า (Unit) ที่รับเข้ามามีค่ามากกว่าหรือเท่ากับ 101 หรือไม่ ถ้าใช่จะทำการคำนวณราคาไฟฟ้าในช่วงหน่วยไฟฟ้า 101-150 ราคารเรท Rate150 และบวกผลลัพธ์ลงในตัวแปร total
  } else {
    float Rate150 = 3.2484; //unit เกิน 150 หน่วย หน่วยละ 3.7171 บาท
    float Rate400 = 4.2218; //250 หน่วยต่อไป (หน่วยที่ 151-400) หน่วยละ 4.2218 บาท
    float RateMore400 = 4.4217;//เกินกว่า 400 หน่วย (หน่วยที่ 401 เป็นต้นไป) หน่วยละ 4.4217 บาท
    total += _min(Unit, 150) * Rate150; //เพิ่มค่าตัวแปร total ด้วยผลคูณของค่าไฟฟ้าต่อหน่วยด้วย Rate150 โดยถ้าหน่วยไฟฟ้าน้อยกว่า 150 หน่วย
    if (Unit >= 151) total += _min(Unit - 150, 250) * Rate400; //ตรวจสอบว่าจำนวนหน่วยไฟฟ้า (Unit) ที่รับเข้ามามีค่ามากกว่าหรือเท่ากับ 151 หรือไม่ ถ้าใช่จะทำการคำนวณราคาไฟฟ้าในช่วงหน่วยไฟฟ้า 151-400 ราคารเรท Rate400 และบวกผลลัพธ์ลงในตัวแปร total
    if (Unit >= 401) total += (Unit - 400) * RateMore400; ////ตรวจสอบว่าจำนวนหน่วยไฟฟ้า (Unit) ที่รับเข้ามามีค่ามากกว่าหรือเท่ากับ 400 หรือไม่ ถ้าใช่จะทำการคำนวณราคาไฟฟ้าเกิน 400 หน่วยขึ้นไป ราคารเรท RateMore400 และบวกผลลัพธ์ลงในตัวแปร total
  }
  total += Unit * (ft / 100); //คำนวณค่าไฟฟ้าผันแปร คิดจากจำนวนพลังงานไฟฟ้า x ค่า Ft
  total += total * 0.07; //รวมภาษีมูลค่าเพิ่มอีก 7%
  return total; //ส่งค่า total กลับ
}

void readWaterTemp(){
  sensorsWatertemp.requestTemperatures(); //คำสั่งนี้ใช้สำหรับอ่านค่าอุณหภูมิของเซนเซอร์วัดอุณหภูมิของน้ำโดยจะเรียกใช้ฟังก์ชั่น requestTemperatures() เพื่อขออ่านค่าอุณหภูมิ
  temperatureC = sensorsWatertemp.getTempCByIndex(0); //โดยจะเรียกใช้ฟังก์ชั่น requestTemperatures() เพื่อขออ่านค่าอุณหภูมิ
  Blynk.virtualWrite(V24, temperatureC); //นำค่าที่ได้ส่งไปยัง Blynk เพื่อแสดงผลบนแอพพลิเคชัน Blynk
}

void readDHT(){
  static unsigned long timepoint = 0; //ตัวแปรสำหรับเก็บเวลารอบการทำงาน
  if(currentMillis - timepoint >= 2000U){ // ตรวจสอบว่าเวลาที่ผ่านมาห่างจากเวลาล่าสุดที่อ่านค่าอินพุตมากกว่าหรือเท่ากับ 2000 มิลลิวินาที
    timepoint = currentMillis; // กำหนดเวลาล่าสุดที่อ่านค่าอินพุตเป็นเวลาปัจจุบัน
    float hum,temp; //ตัวแปรเก็บค่าควมชื้นและอุณหภูมิตามลำดับ
    hum = dht.readHumidity();  // อ่านค่าความชื้นจากเซ็นเซอร์ DHT เก็บค่าลงในตัวแปร hum_room
    temp = dht.readTemperature(); // อ่านค่าอุณหภูมิจากเซ็นเซอร์ DHT เก็บค่าลงในตัวแปร temp_room
    if (isnan(hum) || isnan(temp)) {//ถ้าค่าใดค่าหนึ่งไม่ถูกต้อง (NaN) จะไม่ทำอะไรและสั่ง return
      //Serial.println(F("Failed to read from DHT sensor!"));
      return;
    }
    else{
      hum_room = hum; //hum_room ให้มีค่าเท่ากับ hum
      temp_room = temp; //temp_room ให้มีค่าเท่ากับ temp
      Blynk.virtualWrite(V4, hum_room);
      Blynk.virtualWrite(V3, temp_room);
    }
    /*Serial.print(F("Humidity: "));
    Serial.print(h);
    Serial.print(F("%  Temperature: "));
    Serial.print(t);
    Serial.println(F(" C "));*/
   }
};

void PH(){
  int smoothFactor = 10; // ตัวแปร smoothFactor ใช้เก็บจำนวนการอ่านอินพุตที่จะใช้ในการหาค่าเฉลี่ย
  static float sum; // ตัวแปร sum ใช้เก็บผลรวมของอินพุตทั้งหมด
  static int count; // ตัวแปร count ใช้เก็บจำนวนการอ่านอินพุตทั้งหมด
  float voltagePH; // ตัวแปร voltagePH ใช้เก็บค่าแรงดันไฟฟ้าของอินพุต

  static unsigned long timepoint = 0; // ตัวแปร timepoint เก็บเวลาล่าสุดที่อ่านค่าอินพุต
  if(currentMillis - timepoint >= 100U && pH_calibrat_state == false){ // ตรวจสอบว่าเวลาที่ผ่านมาห่างจากเวลาล่าสุดที่อ่านค่าอินพุตมากกว่าหรือเท่ากับ 100 มิลลิวินาที
    timepoint = currentMillis; // กำหนดเวลาล่าสุดที่อ่านค่าอินพุตเป็นเวลาปัจจุบัน
    float analogValue = analogRead(PHPIN); // อ่านค่าอินพุตจากพอร์ต analog และเก็บค่าไว้ในตัวแปร analogValue
    voltagePH = analogValue/4095.0*3300; // คำนวณค่าแรงดันไฟฟ้าโดยอ่านค่าแรงดันไฟฟ้าและแปลงค่าให้อยู่ในช่วง 0-3300 โวลต์
    float slope = (7.0-4.0)/((neutralVoltage-1500)/3.0 - (acidVoltage-1500)/3.0); // คำนวณค่าความชันของเส้นตรงระหว่างจุดสองจุด (_NautralVoltage,7.0),(_acidVoltage,4.0)
    float intercept = 7.0 - slope*(neutralVoltage-1500)/3.0; //คำนวณค่า intercept โดยใช้สูตร y = kx + b โดยที่ k เป็นค่า slope ที่คำนวณไว้ก่อนหน้านี้ และ x เป็น voltagePH ที่ได้จากการอ่านแรงดันไฟฟ้า  โดยจุดที่ใช้คำนวณ intercept คือ neutralVoltage (ค่าแรงดันไฟฟ้าที่เป็นกลาง) โดยที่เลข 1500 คือค่า offset ของค่าแรงดันไฟฟ้า
    phValue = slope*(voltagePH-1500)/3.0+intercept; //y = k*x +b ใช้สมการเชิงเส้นคำนวณเป็นค่า pH
    
    sum += phValue;// เพิ่มค่า pH ที่อ่านได้และเก็บไว้ในตัวแปร sum 
    count++;// เพิ่มค่า count ที่เป็นจำนวนการอ่านค่า pH ไปทีละหนึ่งในตัวแปร
    if (count == smoothFactor) { // ถ้าอ่านค่าครบตามจำนวนที่ตั้งไว้
      // คำนวณค่าเฉลี่ยของค่า pH ทั้งหมดและส่งค่าออกมาใน lastPH
      lastPH = sum / smoothFactor;
      if(lastPH > 16 || lastPH < 0){ //ถ้าค่า lastPH มีค่ามากกว่า 16 จะกำหนด StatusOfPHsensor เท่ากับ 0 หมายถึงค่าที่อ่านมีความผิดพลาด
        StatusOfPHsensor = 0;
      }
      StatusOfPHsensor = 1; // return sensors pH is good
      // reset the sum and count for the next set of readings
      sum = 0;
      count = 0;
    }
    else{
    }
  }else{
  }
};

void Ec(){
  int smoothFactor = 10; // ตั้งค่าจำนวนค่าที่ใช้ในการคำนวณเฉลี่ย
  static float storeOfEcValue; // ตัวแปรสำหรับเก็บผลรวมของค่า EC
  static int count; // ตัวแปรสำหรับเก็บจำนวนค่าที่อ่านได้
  static unsigned long timepoint = 0; // ตัวแปรสำหรับเก็บจำนวนค่าที่อ่านได้
  // เมื่อผ่านไปเวลา 100 มิลลิวินาที จะทำการอ่านค่า EC จาก TDS sensor
  if(currentMillis - timepoint >= 100U){
     timepoint = currentMillis;
     tds.calTDS();  //เรียกใช้ฟังก์ชัน calTDS() ของออบเจ็กต์ tds เพื่อทำการคำนวณค่า TDS
     ecValue = tds.getEC()*0.001; // รับค่า EC จากฟังก์ชัน getEC() และแปลงค่าให้เป็นหน่วย mS/cm

     // เพิ่มค่า EC เข้าไปยัง storeOfEcValue และเพิ่มจำนวนค่าที่อ่านได้
     storeOfEcValue += ecValue; // add the current reading to the sum
     count++; // increment the count of readings

     if (count == smoothFactor) { // ถ้าอ่านค่าครบตามจำนวนที่ตั้งไว้
      // คำนวณค่าเฉลี่ยของค่า EC ทั้งหมดและส่งค่าออกมาใน lastEC
      lastEC = storeOfEcValue / smoothFactor;
      StatusOfECsensor = 1; // ตั้งค่าสถานะของเซ็นเซอร์ EC เป็น 1 หมายความว่าอ่านค่าได้ถูกต้อง

      // reset the sum and count for the next set of readings
      storeOfEcValue = 0; // เริ่มค่า storeOfEcValue ใหม่
      count = 0; // เริ่มค่า count ใหม่
     }else{
      // ยังไม่ได้อ่านค่าครบตามจำนวนที่ตั้งไว้ ไม่ต้องทำอะไร
     }
  }
}

void waterRs485(){
  int value;      //ตัวแปรสำหรับเก็บค่าน้ำที่อ่านได้จากเซ็นเซอร์ Water metor RS485
  String dataJS;  // ตัวแปรเก็บข้อความสำหรับส่งให้ฟังก์ชันแจ้งเตือนไปยังโปโทคอล Mqtt
  float waterAmountPrice = calBillWater(waterAmount/1000); // คำนวณค่าน้ำโดย waterAmount คือค่าน้ำที่ Sensort Water flow วัดได้ ส่งไปคำนวณด้วยฟังก์ชัน calBillWater(flaot Unit)
  uint8_t result = node.readHoldingRegisters(0,2); //เป็นตัวแปรสำหรับเก็บค่าสถานะการรับส่งข้อมูลแบบ Modbus เช่น ถ้าส่งไม่สำเร็จ result จะมีค่าเท่ากับ 0 โดยฟังก์ชัน readHoldingRegisters จะอ่านค่าจาก Holding Register ที่อยู่ในตำแหน่งที่ 0 จนถึง 1 คืนค่าการอ่านข้อมูลผ่านตัวแปร result ที่มีค่าเป็นศูนย์ถ้าการอ่านไม่สำเร็จและเป็นค่าอื่น ๆ ถ้าการอ่านสำเร็จและข้อมูลถูกอ่านเข้ามาได้ โดยข้อมูลที่ถูกอ่านเข้ามาจะเก็บไว้ใน buffer ซึ่งสามารถเข้าถึงได้โดยใช้เมธอด getResponseBuffer() ของไลบรารี ModbusMaster
  if (result == node.ku8MBSuccess) { //ถ้า result มีค่าเท่ากับ node.ku8MBSuccess ซึ่งหมายถึงการอ่านค่าเป็นไปตามปกติ
    //Serial.println("success: ");  
    value = node.getResponseBuffer(1); // จะมีการอ่านค่าของ Holding Register ตำแหน่งที่ 1 ซึ่งจะได้เป็นค่าน้ำที่ได้จากเซนเซอร์ที่ติดตั้งไว้บนระบบ โดยค่าน้ำที่ได้จะถูกเก็บไว้ในตัวแปรชื่อ value
    Blynk.virtualWrite(V25,value/100); //ส่งค่าน้ำที่ได้จากเซนเซอร์ไปที่ Server Blynk โดย datastream ชื่อ V25 
    water485 = value/100; //แปลงค่าที่อ่านได้จาก Water metor rs485 เป็นหน่วยที่กูกต้อง เช่น อ่านที่ตัวเซ็นเซอร์จะอ่านได้ 2 ลบม เวลาอ่านมาเก็บไว้บนไมโครคอนโทลเลอร์ จะอ่านได้ 200 จึงจำเป็นต้องหาร 100 เพื่อให้เท่ากับ 2 ลบม
    float waterprice485 = calBillWater(water485); //ตัวแปรสำหรับเก็บ ราคาน้ำที่อ่านได้จาก Water metor rs485 โดยฟังก์ชัน calBillWater จะส่งค่าน้ำและค่าบริการกลับมา
    if (mqtt.connected() == true) { //ถ้าบอร์ดเชื่อมต่อกับโพโตคอล Mtqq จะทำการส่งข้อมูลน้ำต่างไปยัง Topic @msg/kmutnb/cs/smart-hydro1/rs485
      mqtt.loop(); // คอลเล็กชัน loop() ของไลบรารี่ MQTT ใช้สำหรับการรับ-ส่งข้อมูลผ่าน MQTT
      dataJS = "{\"water485\":" + String(water485) + ",\"waterprice\":" + String(waterprice485) + ",\"waterFlow\":" + String(waterAmount) + ",\"waterFlowPrice\":" + String(waterAmountPrice) +"}"; // สร้าง JSON ในรูปแบบของ String
      char json[150]; // ประกาศตัวแปร json เป็น char array ขนาด 150 ไบต์
      dataJS.toCharArray(json,dataJS.length()+1); // แปลง String ให้กลายเป็น char array
      mqtt.publish("@msg/kmutnb/cs/smart-hydro1/rs485", json); // ทำการ publish ข้อมูลผ่านทาง MQTT broker ด้วยชื่อ topic "@msg/kmutnb/cs/smart-hydro1/rs485"
      //BLYNK WATER AND WATER PRICE
    }

    Blynk.virtualWrite(V29,waterprice485);
    Blynk.virtualWrite(V30,waterAmount);    
  } else { //ถ้า result มีค่าไม่สมบูรณ์ เช่นการส่งขอข้อมูลผิดพลาด
    Serial.print("error code: ");
    Serial.println(result, HEX); 
    Blynk.virtualWrite(V30,waterAmount);    
    
    
  }
}

void checkFlow() {
  static unsigned long lastCheckTime1 = 0;
  static unsigned int lastPulseCount = 0;
  static int count_no_flow = 0;
  // ตรวจสอบว่าเวลาผ่านไปห้านาทีหลังการตรวจสอบครั้งล่าสุดหรือไม่โดยที่ checkflow_ เป็นจริง และ changeWaterState การเปลี่ยนน้ำเป็นเท็จ
  if (currentMillis - lastCheckTime1 >= 300000 && checkflow_ == true && changeWaterState == false && empty_tank == false && drain_state == false && mainWaterPump == HIGH) {
    unsigned int currentPulseCount = pulse_plantingTrough; // เก็บจำนวนพัลส์ปัจจุบัน

    if (currentPulseCount == lastPulseCount && mainWaterPump == 1) { // ตรวจสอบว่าจำนวนพัลส์ไม่ได้เปลี่ยนแปลงตั้งแต่การตรวจสอบครั้งล่าสุดหรือไม่
      Serial.println("currentPulseCount-lastPulseCount = : " +String(currentPulseCount-lastPulseCount));
      flowwing = 0; //อัพเดทสถานะว่าไม่มีการไหลว่าเท่ากับ 0 หรือไม่มีการไหลเ
      mainWaterPump = 0; //สั่งปิดปั๊มน้ำรางปลูก
      lcdBlynkPrintError("Water not flowing");
      Serial.println("checkFlow: Error");

      String text  = "{\"flowingWater\":" + String(flowwing) + "}";
      notifyingPubMqtt(text);

      // Reset the pulse count
      lastCheckTime1 = 0; // reset time

    }
    //เมื่อสถานะการไหลเป็นปกติ flowwing เป็นจริง
    else{
      
      flowwing = 1;
      String text  = "{\"flowingWater\":" + String(flowwing) + "}";
      notifyingPubMqtt(text);

      //Serial.println("checkFlow good --------------------------------------");
      //Serial.println("currentPulseCount-lastPulseCount = : " +String(currentPulseCount-lastPulseCount));
    }

    // เก็บเวลาและจำนวนพัลส์ปัจจุบันเพื่อใช้ตรวจสอบครั้งต่อไป
    lastCheckTime1 = currentMillis;
    lastPulseCount = pulse_plantingTrough;
  }else{
    //ไม่ทำอะไรเมื่อยังไม่ถึงเวลา
    return;
  }
}

void fillWater(){
  bool switch_fillwaterAuto = swManWaterIn.get_status(); //ตัวแปรสำหรับตรวจสอบว่าฟังก์ชันเติมน้ำอัตโนมัติเปิดอยู่หรือไหม โดยเรียกใช้ฟังก์ชัน get_status() ของออบเจกต์ swManWaterIn เมื่อเปิดอยู่ค่าที่อ่านได้จะเท่ากับ 0 หรือปิดอยู่จะเท่ากับ 1  โดยควบคุมจากสวิชต์ที่ตู้ควบคุมเป็นตัวเปลี่ยนแปลงค่า 
  bool switch_WaterLevel_Top = WaterLevel_Top.get_status(); //เป็นการอ่านสถานะของเซ็นเซอร์ระดับน้ำที่อยู่ในถังน้ำด้านบน โดยค่าที่ได้จะเป็นจริง (true) เมื่อระดับน้ำอยู่ต่ำกว่าเซ็นเซอร์ และเป็นเท็จ (false) เมื่อระดับน้ำอยู่เหนือเซ็นเซอร์
  bool switch_WaterLevel_Bottom = WaterLevel_Bottom.get_status(); //เป็นการอ่านสถานะของเซ็นเซอร์ระดับน้ำที่อยู่ในถังน้ำด้านล่าง โดยค่าที่ได้จะเป็นจริง (true) เมื่อระดับน้ำอยู่ต่ำกว่าเซ็นเซอร์ และเป็นเท็จ (false) เมื่อระดับน้ำอยู่เหนือเซ็นเซอร์
  static unsigned long timepoint_count = 0; //ตัวแแปรสำหรับเก็บเวลาที่จะนับเมื่อเซ็นเซอร์ลูกอยู่ต่ำกว่าเซ็นเซอร์ และ สวิตช์เติมน้ำอัตโนมัติเปิดอยู่
  static unsigned long timepoint_Store_water = 0; //ตัวแปรเก็บเวลาสำหรับใช้คำนวณน้ำที่ไหลผ่าน Water flow sensor 
  static int countTime = 0; //ตัวแแปรสำหรับเก็บเวลา 
  static bool state_of_valve = LOW; //ตัวแปลเก็บสถานะของวาล์วที่ใช้สำหรับเปิดให้น้ำไหลผ่านเข้าถังน้ำ

  
  if(switch_fillwaterAuto == 0 && changeWaterState == false && drain_state == false ){ //ถ้า switch_fillwaterAuto เท่ากับ 0 คือสวิตช์เติมน้ำอัตโนมัติเปิดอยู่ และ changeWaterState เท่ากับเท็จ หรือไม่ได้มีการเปลี่ยนน้ำอยู่ จะเข้าเงื่อนไข
    if(currentMillis - timepoint_count >= 1000U && switch_WaterLevel_Top == 1){  //นับเวลาทุกๆ 10 วิ และ ค่าที่ได้จะเป็น 1 หรือระดับน้ำอยู่ต่ำกว่าเซ็นเซอร์
      timepoint_count = currentMillis; //เป็นการเก็บเวลาที่ระบบทำงานไปแล้ว
      countTime++; //นับเวลาที่ระดับน้ำอยู่ต่ำกว่าเซ็นเซอร์
      //Serial.println("time fill :"+String(countTime));
    }else{
      
    }

    if(countTime >= 6 && switch_WaterLevel_Top == 1 ){ // ถ้าเวลาที่นับว่าระดับน้ำอยู่ต่ำกว่าเซ็นเซอร์มากกว่าหรือเท่ากับ 6 หรือ 60 วิ จะทำการเปิดวาล์วเติมน้ำ
      //turn on valve to fill water in tank
      if(state_of_valve == LOW){ //ถ้า state_of_valve เป็นเท็จ หรือวาลว์ยังไม่ได้เปิด จะทำการเปิดให้น้ำไหลเติมถังน้ำ
        pulse_WaterTank = 0;  // รีเซ็ตค่าที่อ่าจได้จาก Water flow เท่ากับ 0
        //rtu on  //สั่งการทำงานให้วาล์วเปิด
        //relayRtu(17);
        state_of_valve = HIGH;  //เปลี่ยนสถานะของวาล์วเป็นจริง
        
      }
      refill_valve_fill = HIGH;
      if(currentMillis - timepoint_Store_water >= 5000U ){ // ทุกๆ 5 วิ จะทำการจัดเก็บจำนวนน้ำที่ไหลผ่านเข้าถัง
        float Factor = 7.5; //calibration factor
        float flowRate = pulse_WaterTank/Factor; // คำนวณจำนวนพลัลส์ที่นับได้กับค่าคาลิเบรท
        Serial.println("waterAmount: " + String(waterAmount));
        timepoint_Store_water = currentMillis; //จัดเก็บเวลาใช้สำหรับรอบต่อไป
        waterAmount += flowRate/60; // คำนวณจำนวนพลัลส์ที่นับได้ไปเป็นจำนวนลิตรต่อ 1 นาที
        pulse_WaterTank = 0; // รีเซ็ตค่าที่อ่าจได้จาก Water flow เท่ากับ 0
      }
    }
    else{
      //turn off valve to fill water in tank
      if(state_of_valve == HIGH){ //ถ้า state_of_valve เป็นจริง หรือวาลว์ยังเปิดอยู่ จะทำการปิดวาล์วเพื่อหยุดเติมน้ำ
        //rtu off //สั่งการทำงานให้วาล์วปิด
        //relayRtu(18);
        state_of_valve =  LOW; //เปลี่ยนสถานะของวาล์วเป็นเท็จ
        waterAmount += (pulse_WaterTank/7.5)/60; // คำนวณจำนวนพลัลส์ที่นับได้ไปเป็นจำนวนลิตรต่อนาที
        pulse_WaterTank = 0; // รีเซ็ตค่าที่อ่าจได้จาก Water flow เท่ากับ 0
        countTime = 0 ; //ถ้าไม่เข้าเงื่อนไขแรกจะรีเซ็นเวลาที่ระดับน้ำอยู่ต่ำกว่าเซ็นเซอร์ให้เท่ากับ 0
      }
      refill_valve_fill = LOW;
    }
  }else{ //ถ้าสวิตช์ไม่ได้เปิดให้เติมน้ำอัตโนมัตระบบจะตรวจสอบสถานะวาล์วถ้าเปิดอยู่จะสั่งให้ปิด

    if(state_of_valve == HIGH){
      //rtu off  //สั่งการทำงานให้วาล์วปิด
      //relayRtu(18);
      state_of_valve = LOW; //เปลี่ยนสถานะของวาล์วเป็นเท็จ
      
    }
    refill_valve_fill = LOW;
  }
}

void changeWater(bool changeWater_state){ // dashboard 1,0 ---> 1 ----> changeWater--State = 1 
  static unsigned long last_time = 0;
  const unsigned long interval = 1000; // 5 seconds
  static int countTime,countTimeEmpty;
  static unsigned long timepoint = 0;
  static unsigned long timepoint_empty,timepoint_waiting,timepoint_fill,timepoint_release,timepoint_empty_plues;
  bool switch_WaterLevel_Top = WaterLevel_Top.get_status();
  bool switch_WaterLevel_Bottom = WaterLevel_Bottom.get_status();
  static bool filling = false;

  static bool release_valveState = LOW;
  static bool refill_valveState = LOW;

  if(changeWater_state == 1){  //changing water and then fill water
    if (switch_WaterLevel_Top == 0 && switch_WaterLevel_Bottom == 0 || switch_WaterLevel_Top == 1 && switch_WaterLevel_Bottom == 0 && filling == false) { // check if water level is at the top and not at the bottom
      
      if(countTimeEmpty != 0){
        changeWaterState = 0;
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("changed");

        lcdBlynk.clear();
        lcdBlynk.print(0, 0, "changed");

        Blynk.virtualWrite(V27, changeWaterState);
        String text  = "{\"change\":\"done\"}";
        notifyingPubMqtt(text);
      }

      if(currentMillis - timepoint >= 1000U){
        timepoint = currentMillis; //store timepoint
        countTime++; //count 1 = 10 sec.
        countTimeEmpty = 0;
        //Serial.println("countTime : "+String(countTime));
        }
    }
    else if(switch_WaterLevel_Top == 1 && switch_WaterLevel_Bottom == 1){
      if(currentMillis - timepoint_empty >= 1000U){
        timepoint_empty = currentMillis; //store timepoint
        countTimeEmpty++; //count 1 = 10 sec.
        countTime = 0;
        //Serial.println("countTimeEmpty : "+String(countTimeEmpty));
      }
    }
    else{
      if(currentMillis - timepoint_empty_plues >= 1000U){
        timepoint_empty_plues = currentMillis; //store timepoint
        countTimeEmpty++;
      }
    }
    //switch_WaterLevel_Top == 0 เต็ม 1 คือไม่เต็ม
    if(countTime > 60 && switch_WaterLevel_Top == 0 && switch_WaterLevel_Bottom == 0 || countTime > 60 && switch_WaterLevel_Top == 1 && switch_WaterLevel_Bottom == 0){
      //release water ปล่อยน้ำ
      if(release_valveState == LOW){
        release_valveState = HIGH;
        //relayRtu(19);
        release_valve_changewater = HIGH;
        //RTU ON VALVE
      }
      filling = false;
      if(currentMillis - timepoint_release >= 10000U){
        timepoint_release = currentMillis; //store timepoint
        Serial.println("release water");
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Releasing water");

        lcdBlynk.clear();
        lcdBlynk.print(0, 0, "Releasing water");
      }

    }
    else if(countTimeEmpty > 60 && switch_WaterLevel_Top == 1 && switch_WaterLevel_Bottom == 1 || countTimeEmpty > 60 && switch_WaterLevel_Top == 1 && switch_WaterLevel_Bottom == 0){
      //fill water เติมน้ำ
      if(refill_valveState == LOW){
        refill_valveState = HIGH;
        //RTU ON VALVE FILL
        //relayRtu(17);
        
      }
      refill_valve_changeWater = HIGH;
      filling = true;
      if(currentMillis - timepoint_fill >= 10000U){
        timepoint_fill = currentMillis; //store timepoint
        //Serial.println("fill water");
        //Serial.println("countTimeEmpty : "+String(countTimeEmpty));
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("filling the tank");

        lcdBlynk.clear();
        lcdBlynk.print(0, 0, "filling the tank");
      }
    }
    else{
      //not release water
      if(refill_valveState == HIGH){
        refill_valveState = LOW;
        //RTU OFF VALVE FILL
        //relayRtu(18);
        
      }
      refill_valve_changeWater = LOW;
      if(release_valveState == HIGH){
        release_valveState = LOW;
        //RTU OFF VALVE
        //relayRtu(20);
        release_valve_changewater = LOW;
      }
      
      if(currentMillis - timepoint_waiting >= 10000U){
        timepoint_waiting = currentMillis; //store timepoint
        //Serial.println("not release water n not fill water");
        //Serial.println("countTimeEmpty : "+String(countTimeEmpty));
        //Serial.println("countTime : "+String(countTime));
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("waiting");

        lcdBlynk.clear();
        lcdBlynk.print(0, 0, "waiting");
      }
    }
  }else{
    // not change
    //สั่งปิดทั้งหมด
    if(refill_valveState == HIGH){
      refill_valveState = LOW;
      //RTU OFF VALVE refill
      //relayRtu(18);
      
    }
    refill_valve_changeWater = LOW;
    if(release_valveState == HIGH){
      release_valveState = LOW;
      
      //RTU OFF VALVE out
      //relayRtu(20);
    }
    release_valve_changewater = LOW;
    changeWaterState = false;
    filling = false;
    countTimeEmpty = 0;
    countTime = 0;
    
    timepoint_empty = 0;
    timepoint = 0;
  }

}

void autoUpdateRTC(){
  milleHour = currentMillis;
  if(milleHour >= 3600000 && milleHour % 3600000 == 0){ // ตรวจสอบเมื่อเวลาผ่านไป 1 ชั่วโมง
    HourUpdateRTC++; // เพิ่มจำนวนชั่วโมงที่ผ่านไป
  }
  if(HourUpdateRTC >=24){ // ตรวจสอบเมื่อผ่านไป 24 ชั่วโมงหรือไม่
    requestTime(); // ขอเวลาจาก Blynk server
    HourUpdateRTC = 0; // รีเซ็ตจำนวนชั่วโมงที่ผ่านไปใหม่
  }else{
    return; // ไม่ต้องทำอะไรเมื่อยังไม่ครบ 24 ชั่วโมง
  }
}

float calBillWater(float unit){
  // กำหนดค่าคงที่สำหรับค่าน้ำแต่ละหน่วย
  float meterSizeFee = 25; //ค่าบริการขนาดมาตรวัดน้ำ
  float RawWaterStoragefee = unit*0.15; //ค่าบริการจัดเก็บจัดเก็บค่าน้ำดิบ 1000ลิตร ต่อ 0.15 สต.
  float pay = 0; // ใช้เก็บค่าค่าน้ำที่คำนวณได้
  // ถ้าหน่วยน้ำน้อยกว่าหรือเท่ากับ 30 จะคำนวณค่าน้ำตามอัตราค่าน้ำแบบปกติ
  if( unit <=30 ){
    pay = unit*8.50; // คำนวณค่าน้ำ
    pay += meterSizeFee; // ค่าบริการขนาดมาตรวัดน้ำ 1/2นิ้ว ขนาดมารฐาน
    pay += RawWaterStoragefee; // ค่าเก็บน้ำสด
  }
  // ถ้าหน่วยน้ำมากกว่า 30 จะคำนวณค่าน้ำตามอัตราค่าน้ำสูง
  else if ( unit >=31 ){
    pay = unit*10.03; // คำนวณค่าน้ำ
    pay += meterSizeFee; // ค่าบริการขนาดมาตรวัดน้ำ 1/2นิ้ว ขนาดมารฐาน
    pay += RawWaterStoragefee; // ค่าเก็บน้ำสด
  }  
  return pay; // ส่งค่าค่าน้ำที่คำนวณได้กลับไป
}



void empty_check_Water(){
    bool switch_WaterLevel_Top = WaterLevel_Top.get_status();
    bool switch_WaterLevel_Bottom = WaterLevel_Bottom.get_status();
    static int count = 0;
    if(switch_WaterLevel_Top == 1 && switch_WaterLevel_Bottom  == 1 ){
      count++;
      if(count > 6){
        empty_tank = true;
        lcdBlynk.clear();
        lcdBlynk.print(0, 0, "Tank is empty");
      }
    }else{
      count = 0;
      empty_tank = false;
    }
}

void drainWater(){
  static unsigned timePointLcd = 0;
  static bool release_valve = LOW;
  if(drain_state == 1){
    bool switch_WaterLevel_Top = WaterLevel_Top.get_status();
    bool switch_WaterLevel_Bottom = WaterLevel_Bottom.get_status();
    if(switch_WaterLevel_Top == 1 && switch_WaterLevel_Bottom  == 1 ){
      // off valve
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Drained");

      lcdBlynk.clear();
      lcdBlynk.print(0, 0, "Drained");
      
      if(release_valve == HIGH){
        //RTU OFF
        //relayRtu(20);
        release_valve = LOW;
        
      } 
      release_valve_drain = LOW;
      drain_state = 0;
      Blynk.virtualWrite(V31, drain_state);

      String text  = "{\"drain\":\"done\"}";
      notifyingPubMqtt(text);

    }else{
      // on valve
      if(currentMillis - timePointLcd >= 10000U){
        timePointLcd = currentMillis; //store timepoint
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Draining water...");
        lcdBlynk.clear();
        lcdBlynk.print(0, 0, "Draining water...");
       }
      
      if(release_valve == LOW){
        //RTU ON
        //relayRtu(19);
        release_valve = HIGH;
                
      }
      release_valve_drain = HIGH;
    }
  //ถ้าไม่ได้ถ่ายน้ำ
  }else{
    // off valve
    if(release_valve == HIGH){
        //RTU OFF
        //relayRtu(20);
      release_valve = LOW;
      
    } 
    release_valve_drain = LOW;
    return;
  }
}

void calibration_pH_sensor(){
  static unsigned long timePointLcd1 = 0 ;
  static unsigned long timePointLcd2 = 0 ;
  static bool save_pH_natural_Voltage = false , save_pH_acid_Voltage = false;
  static int countdownLcd1 = 10;
  static int countdownLcd2 = 60; // time to cancele

  static int count_Donw_save_acid = 10;
  static float sumAcidVoltage = 0;
  static int count_read_Acid = 0;


  static int count_Donw_save_Natural = 10;  
  static float sumNaturalVoltage = 0;
  static int count_read_Natural = 0;

  if(pH_calibrat_state == 1){
    

    if(currentMillis - timePointLcd1 >= 1000U && countdownLcd1 != 0 ){
        lcdBlynk.clear(); //Use it to clear the LCD Widget
        lcdBlynk.print(0, 0, "calibrating pH..");
        timePointLcd1 = currentMillis;
        countdownLcd1--;
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("pH Sensors - calibration");
        lcd.setCursor(0,1);
        lcd.print("Must have");
        lcd.setCursor(0,2);
        lcd.print("1. pH 6.86 Buffer");
        lcd.setCursor(0,3);
        lcd.print("2. pH 4.00 Buffer");
        //--------reset about Natural
        count_Donw_save_Natural = 10;
        count_read_Natural = 0;
        sumNaturalVoltage = 0;
        //--------reset about acid
        count_Donw_save_acid = 10;
        count_read_Acid = 0;
        sumAcidVoltage = 0;
        //------------------------
        countdownLcd2 = 60;
    }
    
    if(currentMillis - timePointLcd2 >= 1000U && countdownLcd1 == 0){
      lcdBlynk.clear(); //Use it to clear the LCD Widget
      lcdBlynk.print(0, 0, "calibrating pH..");
      float analogValue = analogRead(PHPIN);
      float voltagePH = analogValue/4095.0*3300;

      if(voltagePH >=1320 && voltagePH <= 1420 && save_pH_natural_Voltage == false){
        count_Donw_save_Natural--;
        //--------reset about acid
        count_Donw_save_acid = 10;
        count_read_Acid = 0;
        sumAcidVoltage = 0;
        //------------------------
        sumNaturalVoltage += voltagePH;
        count_read_Natural++;
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Stay Immerse the probe");
        lcd.setCursor(0,1);
        lcd.print("in pH 6.86 solution");
        lcd.setCursor(0,2);
        lcd.print("Saved in: " + String(count_Donw_save_Natural));
        if(count_read_Natural >= 10){
          neutralVoltage = sumNaturalVoltage/count_read_Natural;
          count_read_Natural = 0;
          sumNaturalVoltage = 0;
          countdownLcd2 = 60;
          count_Donw_save_Natural = 10;
          save_pH_natural_Voltage = true;
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Saved Natural Solution");
          Serial.println("neutralVoltage save:"+ String(neutralVoltage));
          String text  = "{\"calibrationPH\":\"Save-neutral\"}";
          notifyingPubMqtt(text);

        }
      }
      else if(voltagePH >= 1471 && voltagePH <= 1970 && save_pH_acid_Voltage == false){
        count_Donw_save_acid--;
        //--------reset about Natural
        count_Donw_save_Natural = 10;
        count_read_Natural = 0;
        sumNaturalVoltage = 0;
        //------------------------
        sumAcidVoltage += voltagePH;
        count_read_Acid++;
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Stay Immerse the probe");
        lcd.setCursor(0,1);
        lcd.print("in pH 4.00 solution");
        lcd.setCursor(0,2);
        lcd.print("Saved in: " + String(count_Donw_save_acid));
        if(count_read_Acid >= 10){
          acidVoltage = sumAcidVoltage/count_read_Acid;

          count_read_Acid = 0;
          sumAcidVoltage = 0;
          countdownLcd2 = 60;
          count_Donw_save_acid = 10;

          save_pH_acid_Voltage = true;
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Saved Acid Solution");
          Serial.println("acidVoltage save:"+ String(acidVoltage));
          String text  = "{\"calibrationPH\":\"Save-acid\"}";
          notifyingPubMqtt(text);
        }
      }
      else if( save_pH_acid_Voltage == true && save_pH_natural_Voltage == true){
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Saved All Solution");
        lcd.setCursor(0,1);
        lcd.print(":D");
        String text  = "{\"calibration-ph\":\"done\"}";
        notifyingPubMqtt(text);
        pH_calibrat_state = false;
        Blynk.virtualWrite(V33,pH_calibrat_state);
      }
      else{
        String next;
        if(save_pH_acid_Voltage == true){
          next = "6.86 ";
        }else if(save_pH_natural_Voltage == true){
          next = "4.00 ";
        }else{
          next = "";
        }
        countdownLcd2--;
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Immerse the probe into");
        lcd.setCursor(0,1);
        lcd.print(next+"buffer solution.");
        lcd.setCursor(0,2);
        lcd.print("canceled in: " + String(countdownLcd2));
        lcd.setCursor(0,3);
        lcd.print("voltage is: " + String(voltagePH));
        

        //--------reset about Natural
        count_Donw_save_Natural = 10;
        count_read_Natural = 0;
        sumNaturalVoltage = 0;
        //--------reset about acid
        count_Donw_save_acid = 10;
        count_read_Acid = 0;
        sumAcidVoltage = 0;
        //------------------------

      }
      if(countdownLcd2 == 0){
        pH_calibrat_state = false;
        lcdBlynk.clear(); //Use it to clear the LCD Widget
        lcdBlynk.print(0, 0, "canceled");
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("canceled....");
        String text  = "{\"calibrationPH\":" + String(pH_calibrat_state) + "}";
        notifyingPubMqtt(text);
        Blynk.virtualWrite(V33,pH_calibrat_state);
      }
      timePointLcd2 = currentMillis;
    }
  }else{
    countdownLcd1 = 10;
    save_pH_natural_Voltage = false;
    save_pH_acid_Voltage = false;
    return;

  }
}

//bool EC_calibrat_state = false;
//float kValue = 1.38; //ยังไม่ได้จำใน eeprom
void calibration_EC_sensor(){
  float static sumVoltage;
  float static count_time_read;
  float static countDown = 10;
  static int countdownLcd1 = 10;
  static int countdownLcd2 = 60;
  static unsigned long timePointLcd2 ,timePointLcd1;
  float TdsFactor = 0.5;  // tds = ec / 2
  float voltage;
  float rawECsolution,KValueTemp;
  float temperature = 25;
  if(EC_calibrat_state){
    if(currentMillis - timePointLcd1 >= 1000U && countdownLcd1 != 0 ){
        lcdBlynk.clear(); //Use it to clear the LCD Widget
        lcdBlynk.print(0, 0, "calibrating EC..");
        timePointLcd1 = currentMillis;
        countdownLcd1--;
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("EC Sensors - calibration");
        lcd.setCursor(0,1);
        lcd.print("Must have");
        lcd.setCursor(0,2);
        lcd.print("1. 1413us/cm buffer solution");
        countdownLcd2 = 60;
        countDown = 10;
        sumVoltage = 0;
        count_time_read = 0;
    }

    if(currentMillis - timePointLcd2 >= 1000U && countdownLcd1 == 0){   
      lcdBlynk.clear(); //Use it to clear the LCD Widget
      lcdBlynk.print(0, 0, "calibrating EC..");   
      voltage = tds.getVoltage();
      if( voltage > 1 ){
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Stay Immerse the probe");
        lcd.setCursor(0,1);
        lcd.print("in 1413us/cm solution");
        lcd.setCursor(0,2);
        lcd.print("Saved in: " + String(countDown));
        /*lcd.setCursor(0,3);
        lcd.print("voltage is: " + String(voltage));*/
        countDown--;
        sumVoltage += voltage;
        count_time_read++;
        if(count_time_read >= 10){
          //cal
          float lastVoltage;
          lastVoltage = sumVoltage/count_time_read;
          rawECsolution = 707/(float)(TdsFactor); //707 is tds 707ppm
          rawECsolution = rawECsolution*(1.0+0.02*(temperature-25.0));
          KValueTemp = rawECsolution/(133.42*lastVoltage*lastVoltage*lastVoltage - 255.86*lastVoltage*lastVoltage + 857.39*lastVoltage);  //calibrate in the  buffer solution, such as 707ppm(1413us/cm)@25^c
          kValue =  KValueTemp;
          tds.kValue = kValue;
          //return state is good
          EC_calibrat_state = false;
          Serial.println("kValue:" + String(kValue));
          Serial.println("success");
          Blynk.virtualWrite(V34,EC_calibrat_state);
          String text  = "{\"calibratEC\":\"done\"}";
          notifyingPubMqtt(text);
        }

      }else{
        countDown = 10;
        sumVoltage = 0;
        count_time_read = 0;
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Immerse the probe into");
        lcd.setCursor(0,1);
        lcd.print("buffer solution.");
        lcd.setCursor(0,2);
        lcd.print("canceled in: " + String(countdownLcd2));
        lcd.setCursor(0,3);
        lcd.print("voltage is: " + String(voltage));
             
      }
      if(countdownLcd2 == 0){
        lcdBlynk.clear(); //Use it to clear the LCD Widget
        lcdBlynk.print(0, 0, "canceled");
        Serial.println("canceled");
        EC_calibrat_state = false;
        countdownLcd1 = 10;
        Blynk.virtualWrite(V34,EC_calibrat_state);
        String text  = "{\"calibratEC\":" + String(EC_calibrat_state) + "}";
        notifyingPubMqtt(text);
      }
      
      timePointLcd2 = currentMillis;
      countdownLcd2--;
    }

  }else{
    return;
  }

}

void release_valve(){ //change function , drain function
  static bool relay_state = LOW;
  if(release_valve_changewater == LOW && release_valve_drain == LOW){
    // off valve
    if(relay_state == HIGH){
      relayRtu(20);
      relay_state = LOW;
    }
    // on valve
  }else{
    if(relay_state == LOW){
      relayRtu(19);
      relay_state = HIGH;
    }
  }
  //Serial.println("release_valve: " + String(relay_state));
}

void refill_valve(){ //fill function change function
  static bool relay_state = LOW;
  if(refill_valve_changeWater == LOW && refill_valve_fill == LOW){
    // on valve
    if(relay_state == HIGH){
      relayRtu(18);
      relay_state = LOW;
    }
    // on valve
  }else{
    if(relay_state == LOW){
      relayRtu(17);
      relay_state = HIGH;
    }
  }
  //Serial.println("refill_valve: " + String(relay_state));
}

