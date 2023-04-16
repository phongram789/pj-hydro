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
*/
#include <WiFi.h>
#include <WiFiClient.h>
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

//-------------EC-----------------
float StatusOfECsensor; //ประกาศตัวแปรสำหรับบอกสถานะการอ่านค่า EC โดยค่า 0 หมายถึงไม่พร้อมอ่าน 1 หมายถึงพร้อมอ่าน
float ecValue,lastEC; //ประกาศตัวแปรสำหรับเก็บค่า EC ที่อ่านได้จากเซ็นเซอร์



//----------------water in tank-----------------
float waterAmount; //ประกาศปริมาณน้ำที่วัดได้จาก Water flow sensor มีหน่วยเป็นลิตร

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
#define EEPROM_SIZE 64 // กำหนดค่าคงที่ EEPROM_SIZE ให้มีค่าเป็น 64

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

bool updateGl = 0; // 
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

bool release_valve_changewater = false;
bool release_valve_drain = false;
bool refill_valve_changeWater = false ;
bool refill_valve_fill = false;

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
  Serial.begin(115200);
  /*if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure");
  }*/
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
  timer.setInterval(9000L, PRINT);
  timer.setInterval(30000L, release_valve);
  timer.setInterval(30000L, refill_valve);
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

}

void PRINT(){
  /*Serial.println("ecAuto : " + String(ecAuto));
  Serial.println("ecMan : " + String(ecMan));*/ // read switch is good
  /*Serial.println("phAuto : " + String(phAuto));
  Serial.println("phMan : " + String(phMan));*/ //read switch is good
  /*Serial.println("GLAuto : " + String(GLAuto));
  Serial.println("GLMan : " + String(GLMan));*/ //switch is good

  Serial.println("changeWater: " + String(changeWaterState));
  Serial.println("drain_state : " + String(drain_state));
  Serial.println("empty_tank : " + String(empty_tank));
  Serial.println("AdjustEcState : " + String(AdjustEcState));
  Serial.println("lastEC : " + String(lastEC));
  Serial.println("ecLow : " + String(ecLow));
  

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

void statusMqttMsg(){
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

void notifyingPubMqtt(String dataJS){
  if (mqtt.connected() == true) {
      mqtt.loop();
      char json[150];
      dataJS.toCharArray(json,dataJS.length()+1);
      mqtt.publish("@msg/kmutnb/cs/smart-hydro1/notifying", json);
  }
}

void Mqttreconnect(){
  static unsigned long timepoint = 0;
  static int error_con_time = 0;
  if(currentMillis - timepoint >= 5000U){
    timepoint = currentMillis;
    if (mqtt.connected() == false) {
      error_con_time++;
      if(error_con_time > 10){
        Serial.print("Err Mqtt : " + String(error_con_time)+" time" );
        //lcd
      }
      statusMqtt = mqtt.connected(); // return status of mqtt now
      Serial.print("MQTT connection... ");
      if (mqtt.connect(mqtt_client, mqtt_username, mqtt_password)){
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


        Serial.println("connected");
        statusMqtt = mqtt.connected();
      } 
      else{
        statusMqtt = mqtt.connected();
        Serial.println("failed");
        //delay(1000); //fix to millis 
      }
    }
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

void initWiFi() { 
  //192.127.2.34
  //WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Connecting to WiFi..");
  while (WiFi.status() != WL_CONNECTED) {
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

void wifiReconnect(){
  static unsigned long previousMillis = 0;
  unsigned long interval = 30000;
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >=interval)) {
    Serial.print(millis());
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    previousMillis = currentMillis;
  }
}

void initEEPROM() { 
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
  address += sizeof(waterAmount);
  waterAmount = EEPROM.read(address);

};

void callback(char* topic,byte* payload, unsigned int length) {
  /*Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");*/
  String msg;
  for (int i = 0; i < length; i++) {
    msg = msg + (char)payload[i];
  }
  if (String(topic) == "@msg/eclow") { 
    ecLow = msg.toFloat();
  }
  if (String(topic) == "@msg/echigh") { 
    ecHigh = msg.toFloat();
  }
  if (String(topic) == "@msg/phlow") { 
    phLow = msg.toFloat();
  }
  if (String(topic) == "@msg/phhigh") { 
    phHigh = msg.toFloat();
  }
  
  //-------------------------- control growlight-----------------
  //GrowLight1Control1, GrowLight1Control2, GrowLight1Control3, GrowLight1Control4
  if (String(topic) == "@msg/kmutnb/cs/smart-hydro1/gl1") { 
    if (msg == "1"){
      GrowLightControl1 = 1;
    } else {
      GrowLightControl1 = 0;
    }
    updateGl = 0;
  }
  if (String(topic) == "@msg/kmutnb/cs/smart-hydro1/gl2") { 
    if (msg == "1"){
      GrowLightControl2 = 1;
    } else {
      GrowLightControl2 = 0;
    }
    updateGl = 0;
  }
  if (String(topic) == "@msg/kmutnb/cs/smart-hydro1/gl3") { 
    if (msg == "1"){
      GrowLightControl3 = 1;
    } else {
      GrowLightControl3 = 0;
    }
    updateGl = 0;
    
  }
  if (String(topic) == "@msg/kmutnb/cs/smart-hydro1/gl4") { 
    if (msg == "1"){
      GrowLightControl4 = 1;
    } else {
      GrowLightControl4 = 0;
    }
    updateGl = 0;
  }
  
  //-------------------------main water pump pumping to line plant
  if (String(topic) == "@msg/kmutnb/cs/smart-hydro1/mainwater") { 
    if (msg == "1" && mainWaterPump == LOW){
      mainWaterPump = HIGH;
    } else if(msg == "0" && mainWaterPump == HIGH){
      mainWaterPump = LOW;
    }
  }
  //-------------------------flow checking
  if (String(topic) == "@msg/kmutnb/cs/smart-hydro1/flow-check") { 
    checkflow_ = !checkflow_ ;
    Blynk.virtualWrite(V26,checkflow_);
    String text  = "{\"funcflow\":" + String(checkflow_) + "}";
    notifyingPubMqtt(text);
  }
  if (String(topic) == "@msg/kmutnb/cs/smart-hydro1/reset-power") { 
    if (msg == "reset"){
      clearSerial2Buffer();
      pzem.resetEnergy();
      String text  = "{\"reset_energy\":" + String(1) + "}";
      notifyingPubMqtt(text);
    }
  }
  if (String(topic) == "@msg/kmutnb/cs/smart-hydro1/reset-water") { 
    if (msg == "1"){
      waterAmount = 0.0;
      String text  = "{\"reset_water\":" + String(1) + "}";
      notifyingPubMqtt(text);
    }
  }

  if (String(topic) == "@msg/kmutnb/cs/smart-hydro1/drain") { 
    if (msg == "1"){
      Serial.println(drain_state);
      drain_state = !drain_state;
      Serial.println(drain_state);
      String text  = "{\"drain\":" + String(drain_state) + "}";
      notifyingPubMqtt(text);
    }
  }

  if (String(topic) == "@msg/kmutnb/cs/smart-hydro1/change-water") { 
    if (msg == "1"){
      Serial.println(changeWaterState);
      changeWaterState = !changeWaterState;
      Serial.println(changeWaterState);
      String text  = "{\"change\":" + String(changeWaterState) + "}";
      notifyingPubMqtt(text);
    }
  }
};

void reconnectBlynk(){
  static int errorTime = 0;
  static unsigned long timepoint = 0;
  if(currentMillis - timepoint >= 1000U){
    timepoint = currentMillis;
    if (!Blynk.connected() ) {
      statusBlynk = Blynk.connected();
      errorTime++;
      //Serial.println("Lost Blynk server connection");
      Blynk.connect();
    } else {
      // return something
      //Serial.println("Blynk connected");
      statusBlynk = Blynk.connected();
      errorTime = 0;
    }
  }
}

BLYNK_WRITE(V0){
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

void functionLcd(){
    static unsigned long lastSaveTime = 0;
    static unsigned long currentPageTime = 0;
    static int currentPage = 0;
    if (currentMillis - currentPageTime >= 10000U) {
      currentPageTime = currentMillis;
      currentPage++;
      currentPage = currentPage % 3;
    }
    if (currentMillis - lastSaveTime >= 1000U && drain_state == false && changeWaterState == false) {
      lastSaveTime = currentMillis;
      // clear the screen
      switch(currentPage) {
        case 0:
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Temp  :" + String(temp_room) + " c"); //"/n"
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

          
          // timer
          // rtc
          // ec set
          // pH set
          break;
      }
    }
};

void clearSerial2Buffer() {
  Serial2.setTimeout(1); // set a small timeout to discard incoming data
  while (Serial2.available()) {
    Serial2.read(); // read any incoming data (which will be discarded)
  }
  Serial2.setTimeout(1000); // set the timeout back to a normal value 
}

void pzemRead(){
  static unsigned long lastSaveTime = 0;
  static int countError;
    if (currentMillis - lastSaveTime >= 10000U) {
      lastSaveTime = currentMillis;
      clearSerial2Buffer();

        //Serial.print("Custom Address:");
        //Serial.println(pzem.readAddress(), HEX);

        // Read the data from the sensor
         voltage = pzem.voltage();
         current = pzem.current();
         power = pzem.power();
         energy = pzem.energy();
         frequency = pzem.frequency();
         pf = pzem.pf();

        // Check if the data is valid
        if(isnan(voltage)){
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
        } else {
            countError=0;
            
            Unit = energy;
            if(ft == NULL){
              ft = 93.49;
            }
            energyprice = calEnergyPrice(Unit,ft);
            Blynk.virtualWrite(V21, energyprice);
            Blynk.virtualWrite(V23, Unit);
            Blynk.virtualWrite(V5, voltage);
            Blynk.virtualWrite(V6, current);
            Blynk.virtualWrite(V7, power);
            Blynk.virtualWrite(V8, energy);
            Blynk.virtualWrite(V9, frequency);
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

void RTCfunction(){
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
    if(year  < 2023 || year  > 2100){
      if (currentMillis - lastSaveTimeError >= 10000U ) {
        errorReadRTC++;
        Serial.println("error RTC Module: " + String(errorReadRTC) + " time");
        requestTime();
        lastSaveTimeError = currentMillis;
      }else{
        return;
      }
    }
    else{
      RtcState = true;
      nowHour=now.hour();
      nowMinute=now.minute();
      nowSecond=now.second();
    }
  }
};

void EEPROMfunction(){
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
    address += sizeof(waterAmount);
    EEPROM.write(address, waterAmount);

    //state of grow light
    EEPROM.commit();
    //work list hear
    //
    //Serial.println("EEPROM Done");
    //work list is done
    //startHour,startMinute,startSecond ON HH:MM:SS 
    //stopHour,stoptMinute,stopSecond   OFF HH:MM:SS
  }
};

void controlEC(){ //switch mode --> codition of lv EC value --> Return state of relay
  static unsigned long lastSaveTimeMan;
  static unsigned long lastSaveTimeoOff;
  static unsigned long lastSaveTimeOffAuto;
  static bool StateRelayOfEC = false;
  static const unsigned long PUMP_ON_TIME = 3000; // 5 seconds
  static const unsigned long PUMP_OFF_TIME = 120000; // 60 seconds
  static unsigned long last_pump_time = 0;
  static unsigned long lastPulseTime_A_Solution;
  static int empty_A_count = 0;
	static unsigned long lastPulseTime_B_Solution;
  static int empty_B_count = 0;


  ecAuto = swAutoEC.get_status();
  ecMan = swManEC.get_status();

  // Select switch
  if((ecAuto == 1)&&(ecMan == 1)){ // swEcModeOff
    // Rtu relay off case 0
    stirPumpAB = LOW;
    last_pump_time = 0;
    empty_A_count = 0;
    empty_B_count = 0;
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
  else if ((ecAuto == 0)&&(ecMan == 1)){ //swEcModeAuto
    if(lastEC < ecLow && StatusOfECsensor == 1 && changeWaterState == false && empty_tank == false && drain_state == false){ // pump AB solution to up ec value in water
      AdjustEcState = 1;
    }
    else if((lastEC >= ecLow)&&(lastEC <= ecHigh) && StatusOfECsensor == 1 ){ // good Ec Value
      AdjustEcState = 0;
    }
    else{ // high ec value in water
      AdjustEcState = 0;
    }

    if(AdjustEcState == 1){
      stirPumpAB = HIGH; //pump กวน
      if (StateRelayOfEC && (currentMillis - last_pump_time >= PUMP_ON_TIME)) {
        // turn off pump
        relayRtu(2);
        StateRelayOfEC = false;
        if(lastPulseTime_A_Solution+100 >= pulse_A_Solution){
          Serial.println("A sulution is not flow");
          empty_A_count++;
        }
        if(lastPulseTime_B_Solution+100 >= pulse_B_Solution){
          Serial.println("B sulution is not flow");
          empty_B_count++;
        }else{
          empty_A_count = 0;
          empty_B_count = 0;
        }

        if( empty_A_count >= 2 && empty_A_count >= 2){
          String text  = "{\"ABSolutionEmptyTime\":" + String(empty_B_count) + "}";
          notifyingPubMqtt(text);
        }
        
        last_pump_time = currentMillis;
      } else if (!StateRelayOfEC && (currentMillis - last_pump_time >= PUMP_OFF_TIME)) {
        // turn on pump
        lastPulseTime_A_Solution = pulse_A_Solution;
        lastPulseTime_B_Solution = pulse_B_Solution;
        relayRtu(1);
        StateRelayOfEC = true;
        last_pump_time = currentMillis;
      }
    }
    else{
      // Relay RTU Off
      last_pump_time = 0;
      empty_A_count = 0;
      empty_B_count = 0;
      stirPumpAB = LOW; //ปั๊มกวนปิด
      
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
  else if ((ecAuto == 1)&&(ecMan == 0)){ //swEcModeMan
    // Rtu relay on
    stirPumpAB = HIGH;
    last_pump_time = 0;
    empty_A_count = 0;
    empty_B_count = 0;
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

void controlPH(){ //switch mode --> codition of lv pH value --> Return state of relay 
  static unsigned long lastSaveTimeMan;
  static unsigned long lastSaveTimeoOff;
  static unsigned long lastSaveTimeOffElseAuto;
  static unsigned long last_pump_time = 0;
  const int PUMP_ON_TIME = 3000;     // Relay on time in milliseconds
  const int PUMP_OFF_TIME = 120000;   // Relay off time in milliseconds
  static bool relayStatePH = false;
  static unsigned long lastPulseTime_pH_Solution;
  static int empty_ph_count = 0;

  phAuto = swAutoPH.get_status();
  phMan = swManPH.get_status();
  
  // Select switch
  if((phAuto == 1)&&(phMan == 1)){ // swphModeOff
    stirPumpPh = LOW;
    last_pump_time = 0;
    empty_ph_count = 0;

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

  else if ((phAuto == 0)&&(phMan == 1)){ //swphModeAuto
    //คำนวณ
    if((lastPH > phHigh)&&(AdjustEcState != 1) && StatusOfPHsensor == 1 && changeWaterState == false && empty_tank == false && drain_state == false){ // pump pH solution to up pH value in water //anything else not auto ph with ec man
      AdjustPHState = 1;
    }
    else if((lastPH >= phLow)&&(lastPH <= phHigh) && StatusOfPHsensor == 1){ // good pH Value
      AdjustPHState = 0;
    }
    else{ // low pH value in water
      AdjustPHState = 0;
    }

    if(AdjustPHState == 1){
      stirPumpPh = HIGH; //pump กวน
      if (relayStatePH && (currentMillis - last_pump_time >= PUMP_ON_TIME)) {  // turn off pump
        relayRtu(4); 
        relayStatePH = LOW;
        last_pump_time = currentMillis;

        if(lastPulseTime_pH_Solution+100 >= pulse_A_Solution){
          Serial.println("pH sulution is not flow");
          empty_ph_count++;
        }else{
          empty_ph_count = 0;
        }
        
        if(empty_ph_count > 2){
          String text  = "{\"pHSolutionEmptyTime\":" + String(empty_ph_count) + "}";
          notifyingPubMqtt(text);
          //Blynk

        }
      }else if(!relayStatePH && (currentMillis - last_pump_time >= PUMP_OFF_TIME)){ // turn on pump
        relayRtu(3); 
        relayStatePH = HIGH;
        last_pump_time = currentMillis;
        lastPulseTime_pH_Solution = pulse_phDownSolution;
      }
    }
    else{// Relay RTU Off 
      last_pump_time = 0;
      empty_ph_count = 0;
      stirPumpPh = LOW;
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

  else if ((phAuto == 1)&&(phMan == 0)){ //swphModeMan
    stirPumpPh = HIGH;
    last_pump_time = 0;
    empty_ph_count = 0;
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
    updateGl = 0;
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
    updateGl = 0;
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
    Serial.println("-----------switch pump con box-----------");
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

/*//----------------new relay Rtu
void relayRtu(int condition){ // manage state of relay on/off ch.
  switch(condition) {

    case 1: //sending conmand relay on ec ch
      //relay 50-100
      Serial2.write(ON_RTU6, 8);
      Serial.println("pump ec on");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
    case 2: //sending conmand relay off ec ch
      
      //relay 50-100
      Serial2.write(OFF_RTU6, 8);
      Serial.println("pump ec off");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
    case 3: //sending conmand relay on ph ch
      
      //relay 50-100
      Serial2.write(ON_RTU7, 8);
      Serial.println("pump pH on");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
    case 4: //sending conmand relay off ph ch
      
      //relay 50-100
      Serial2.write(OFF_RTU7, 8);
      Serial.println("pump pH off");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;

    case 5: //sending conmand relay on pumpกวน
      
      //relay 50-100
      Serial2.write(ON_RTU8, 8);
      Serial.println("stir pump on");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
    case 6: //sending conmand relay off  pumpกวน
      
      //relay 50-100
      Serial2.write(OFF_RTU8, 8);
      delay(50);
      Serial.println("stir pump off");
      Serial2.flush();
      //Serial2.flushReceive();
      break;

    case 7: //sending conmand relay on gl1
      
      //relay 50-100
      Serial2.write(ON_RTU10, 8);
      Serial.println("gl 1 on");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
    case 8: //sending conmand relay off  gl1
      
      //relay 50-100
      Serial2.write(OFF_RTU10, 8);
      Serial.println("gl 1 off");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
    case 9: //sending conmand relay on gl2
      
      //relay 50-100
      Serial2.write(ON_RTU9, 8);
      Serial.println("gl 2 on");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
    case 10: //sending conmand relay off  gl2
      
      //relay 50-100
      Serial2.write(OFF_RTU9, 8);
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
}*/

//----------------ต้นฉบับ
/*void relayRtu(int condition){ // manage state of relay on/off ch.
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
      Serial2.write(ON_RTU15, 8);
      Serial.println("gl 2 on");
      delay(50);
      Serial2.flush();
      //Serial2.flushReceive();
      break;
    case 10: //sending conmand relay off  gl2
      
      //relay 50-100
      Serial2.write(OFF_RTU15, 8);
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
}*/

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
  //notifyingPubMqtt(String dataJS);
}

BLYNK_WRITE(V10){
  GrowLightControl1 = param.asInt();
  Serial.println(GrowLightControl1);
  updateGl = 0; // update status of auto mode to change to status from dashboard
}
BLYNK_WRITE(V11){
  GrowLightControl2 = param.asInt();
  Serial.println(GrowLightControl2);
  updateGl = 0;

}
BLYNK_WRITE(V12){
  GrowLightControl3 = param.asInt();
  Serial.println(GrowLightControl3);
  updateGl = 0;
}
BLYNK_WRITE(V13){
  GrowLightControl4 = param.asInt();
  Serial.println(GrowLightControl4);
  updateGl = 0;
}
BLYNK_WRITE(V14){ //main pump
  mainWaterPump = param.asInt();
  Serial.println("pump blynk :" + String(mainWaterPump));
}
BLYNK_WRITE(V17){
  phLow = param.asFloat();

}
BLYNK_WRITE(V18){
  phHigh = param.asFloat();
}
BLYNK_WRITE(V19){
  ecLow = param.asFloat();
}
BLYNK_WRITE(V20){
  ecHigh = param.asFloat();
}
BLYNK_WRITE(V22){
  ft = param.asFloat();
}
BLYNK_WRITE(V26){
  checkflow_ = param.asInt();
}
BLYNK_WRITE(V27){
  changeWaterState = param.asInt();
}
BLYNK_WRITE(V28){
  int reset = param.asInt();
  if(reset == 1){
    //clearSerial2Buffer();
    pzem.resetEnergy();
    Blynk.virtualWrite(V28,1);
  }
}

void lcdBlynkPrint(){
  //x = position symbol 0 -15 , y = line 0,1
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

void lcdBlynkPrintError(String text){
  //x = position symbol 0 -15 , y = line 0,1
  lcdBlynk.clear(); 
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

    if (Unit >= 6) total += _min(Unit, 15) * Rate15; //ตรวจสอบว่าจำนวนหน่วยไฟฟ้า (Unit) ที่รับเข้ามามีค่ามากกว่าหรือเท่ากับ 6 หรือไม่ ถ้ามีจะทำการคำนวณราคาไฟฟ้าในช่วงหน่วยไฟฟ้า 6-15 โดยใช้เรทและบวกผลลัพธ์ลงในตัวแปร total
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
  float acidVoltage = 1810; // ค่าแรงดันไฟฟ้าที่เป็นค่าคงที่สำหรับสารเป็นกรด
  float neutralVoltage = 1370; // ค่าแรงดันไฟฟ้าที่เป็นค่าคงที่สำหรับสารเป็นกลาง
  static unsigned long timepoint = 0; // ตัวแปร timepoint เก็บเวลาล่าสุดที่อ่านค่าอินพุต
  if(currentMillis - timepoint >= 100U){ // ตรวจสอบว่าเวลาที่ผ่านมาห่างจากเวลาล่าสุดที่อ่านค่าอินพุตมากกว่าหรือเท่ากับ 100 มิลลิวินาที
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

      Serial.println("checkFlow good --------------------------------------");
      Serial.println("currentPulseCount-lastPulseCount = : " +String(currentPulseCount-lastPulseCount));
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
      Serial.println("time fill :"+String(countTime));
    }else{
      
    }

    if(countTime >= 6 && switch_WaterLevel_Top == 1 ){ // ถ้าเวลาที่นับว่าระดับน้ำอยู่ต่ำกว่าเซ็นเซอร์มากกว่าหรือเท่ากับ 6 หรือ 60 วิ จะทำการเปิดวาล์วเติมน้ำ
      //turn on valve to fill water in tank
      if(state_of_valve == LOW){ //ถ้า state_of_valve เป็นเท็จ หรือวาลว์ยังไม่ได้เปิด จะทำการเปิดให้น้ำไหลเติมถังน้ำ
        pulse_WaterTank = 0;  // รีเซ็ตค่าที่อ่าจได้จาก Water flow เท่ากับ 0
        //rtu on  //สั่งการทำงานให้วาล์วเปิด
        relayRtu(17);
        state_of_valve = HIGH;  //เปลี่ยนสถานะของวาล์วเป็นจริง
        refill_valve_fill = HIGH;
      }
      if(currentMillis - timepoint_Store_water >= 5000U ){ // ทุกๆ 5 วิ จะทำการจัดเก็บจำนวนน้ำที่ไหลผ่านเข้าถัง
        Serial.println("waterAmount: " + String(waterAmount));
        timepoint_Store_water = currentMillis; //จัดเก็บเวลาใช้สำหรับรอบต่อไป
        waterAmount += pulse_WaterTank/7.5; // คำนวณจำนวนพลัลส์ที่นับได้ไปเป็นจำนวนลิตร
        pulse_WaterTank = 0; // รีเซ็ตค่าที่อ่าจได้จาก Water flow เท่ากับ 0
      }
    }
    else{
      //turn off valve to fill water in tank
      if(state_of_valve == HIGH){ //ถ้า state_of_valve เป็นจริง หรือวาลว์ยังเปิดอยู่ จะทำการปิดวาล์วเพื่อหยุดเติมน้ำ
        //rtu off //สั่งการทำงานให้วาล์วปิด
        relayRtu(18);
        refill_valve_fill = LOW;
        state_of_valve =  LOW; //เปลี่ยนสถานะของวาล์วเป็นเท็จ
        waterAmount += pulse_WaterTank/7.5; // คำนวณจำนวนพลัลส์ที่นับได้ไปเป็นจำนวนลิตร
        pulse_WaterTank = 0; // รีเซ็ตค่าที่อ่าจได้จาก Water flow เท่ากับ 0
        countTime = 0 ; //ถ้าไม่เข้าเงื่อนไขแรกจะรีเซ็นเวลาที่ระดับน้ำอยู่ต่ำกว่าเซ็นเซอร์ให้เท่ากับ 0
      }
    }
  }else{ //ถ้าสวิตช์ไม่ได้เปิดให้เติมน้ำอัตโนมัตระบบจะตรวจสอบสถานะวาล์วถ้าเปิดอยู่จะสั่งให้ปิด

    if(state_of_valve == HIGH){
      //rtu off  //สั่งการทำงานให้วาล์วปิด
      relayRtu(18);
      state_of_valve = LOW; //เปลี่ยนสถานะของวาล์วเป็นเท็จ
      refill_valve_fill = LOW;
    }
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
        Blynk.virtualWrite(V27, changeWaterState);

        String text  = "{\"change\":\"done\"}";
        notifyingPubMqtt(text);

        //mtqq
      }else{}
      if(currentMillis - timepoint >= 1000U){
        timepoint = currentMillis; //store timepoint
        countTime++; //count 1 = 10 sec.
        countTimeEmpty = 0;
        Serial.println("countTime : "+String(countTime));
        }
    }
    else if(switch_WaterLevel_Top == 1 && switch_WaterLevel_Bottom == 1){
      if(currentMillis - timepoint_empty >= 1000U){
        timepoint_empty = currentMillis; //store timepoint
        countTimeEmpty++; //count 1 = 10 sec.
        countTime = 0;
        Serial.println("countTimeEmpty : "+String(countTimeEmpty));
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
        relayRtu(19);
        release_valveState = HIGH;
        //RTU ON VALVE
      }
      filling = false;
      if(currentMillis - timepoint_release >= 1000U){
        timepoint_release = currentMillis; //store timepoint
        Serial.println("release water");
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("release water");
      }

    }
    else if(countTimeEmpty > 60 && switch_WaterLevel_Top == 1 && switch_WaterLevel_Bottom == 1 || countTimeEmpty > 60 && switch_WaterLevel_Top == 1 && switch_WaterLevel_Bottom == 0){
      //fill water เติมน้ำ
      if(refill_valveState == LOW){
        refill_valveState = HIGH;
        //RTU ON VALVE FILL
        relayRtu(17);
        refill_valve_changeWater = HIGH;
      }
      filling = true;
      if(currentMillis - timepoint_fill >= 1000U){
        timepoint_fill = currentMillis; //store timepoint
        Serial.println("fill water");
        Serial.println("countTimeEmpty : "+String(countTimeEmpty));
      }
    }
    else{
      //not release water
      if(refill_valveState == HIGH){
        refill_valveState = LOW;
        //RTU OFF VALVE FILL
        relayRtu(18);
        refill_valve_changeWater = LOW;
      }
      if(release_valveState == HIGH){
        release_valveState = LOW;
        //RTU OFF VALVE
        relayRtu(20);
        release_valve_changewater = LOW;
      }
      
      if(currentMillis - timepoint_waiting >= 1000U){
        timepoint_waiting = currentMillis; //store timepoint
        Serial.println("not release water n not fill water");
        Serial.println("countTimeEmpty : "+String(countTimeEmpty));
        Serial.println("countTime : "+String(countTime));
      }
    }
  }else{
    // not change
    //สั่งปิดทั้งหมด
    if(refill_valveState == HIGH){
      refill_valveState = LOW;
      //RTU OFF VALVE refill
      relayRtu(18);
      refill_valve_changeWater = LOW;
    }
    if(release_valveState == HIGH){
      release_valveState = LOW;
      release_valve_changewater = LOW;
      //RTU OFF VALVE out
      relayRtu(20);
    }
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
  if(milleHour >= 3600000){ // ตรวจสอบเมื่อเวลาผ่านไป 1 ชั่วโมง
    HourUpdateRTC++; // เพิ่มจำนวนชั่วโมงที่ผ่านไป
    milleHour = 0; // รีเซ็ตค่าเวลาที่ผ่านไปใหม่
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

BLYNK_WRITE(V31){
  if(param.asInt() == 1){
    drain_state = true;
  }
  else{
    drain_state = false;
  }
}

void empty_check_Water(){
    bool switch_WaterLevel_Top = WaterLevel_Top.get_status();
    bool switch_WaterLevel_Bottom = WaterLevel_Bottom.get_status();
    if(switch_WaterLevel_Top == 1 && switch_WaterLevel_Bottom  == 1 ){
      empty_tank = true;
    }else{
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
      
      if(release_valve == HIGH){
        //RTU OFF
        relayRtu(20);
        release_valve = LOW;
        release_valve_drain = LOW;
      } 
      drain_state = 0;
      Blynk.virtualWrite(V31, drain_state);

      String text  = "{\"drain\":\"done\"}";
      notifyingPubMqtt(text);

    }else{
      // on valve
      if(currentMillis - timePointLcd >= 1000U){
        timePointLcd = currentMillis; //store timepoint
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Draining water...");
       }
      
      if(release_valve == LOW){
        //RTU ON
        relayRtu(19);
        release_valve = HIGH;
        release_valve_drain = HIGH;        
      }
    }
  //ถ้าไม่ได้ถ่ายน้ำ
  }else{
    // off valve
    if(release_valve == HIGH){
        //RTU OFF
        relayRtu(19);
      release_valve = LOW;
      release_valve_drain = LOW;
    } 
    return;
  }
}

void release_valve(){
  static bool relay_state = LOW;
  if(release_valve_changewater == LOW && release_valve_drain == LOW){
    // off valve
    if(relay_state == HIGH){

      relay_state = LOW;
    }
    // on valve
  }else{
    if(relay_state == LOW){
      
      relay_state = HIGH;
    }
  }
  Serial.println("release_valve: " + String(relay_state));
}

void refill_valve(){
  static bool relay_state = LOW;
  if(refill_valve_changeWater == LOW && refill_valve_fill == LOW){
    // on valve
    if(relay_state == HIGH){

      relay_state = LOW;
    }
    // on valve
  }else{
    if(relay_state == LOW){
      
      relay_state = HIGH;
    }
  }
  Serial.println("refill_valve: " + String(relay_state));
}

