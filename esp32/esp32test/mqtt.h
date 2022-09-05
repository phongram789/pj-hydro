//=================อันที่ 1===========================https://www.youtube.com/watch?v=XwXcbWqbLVE&ab_channel=prasitHs3bbb
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHTStable.h>
//===============================================
   #include <LiquidCrystal_I2C.h>
   LiquidCrystal_I2C lcd(0x27, 20, 4);
//===============================================
#include "DHT.h"
DHT dht;
#define DHTTYPE DHT11 
//===============================================
int relay1 =  2 ; // D4
int relay2 = 14 ; // D5
int relay3 = 12 ; // D6 
int relay4 = 13 ; // D7 
String autoHum = "on"; 
String autoTem = "on"; 
String SW01 = "on"; 
String SW02 = "on"; 
//===============================================
const char* ssid = "xx WIFI ที่บ้าน xx";
const char* password = "xx รหัส WIFI xx";
const char* mqtt_server = "broker.netpie.io";
const int mqtt_port = 1883;
const char* mqtt_Client = "xxxxxxxxxxxxxxxxx";
const char* mqtt_username = "xxxxxxxxxxxxxxxxx";
const char* mqtt_password = "xxxxxxxxxxxxxxxxx";
WiFiClient espClient;                
PubSubClient client(espClient);      
char msg[100];
//===============================================
void reconnect() {                   
  while (!client.connected()) {                                        
    Serial.print("Attempting MQTT connection…");                      
    if (client.connect(mqtt_Client, mqtt_username, mqtt_password)) {    
      Serial.println("connected");     
      client.subscribe("@msg/#");    
      client.subscribe("@msg/relay3");
      client.subscribe("@msg/relay4");          
    } else {                         
      Serial.print("failed, rc=");                                  
      Serial.print(client.state());                                
      Serial.println("try again in 5 seconds");    
      delay(5000);
    }
  }
}
//=============================================== 
void callback(char* topic, byte* payload, unsigned int length) {     
  Serial.print("Message arrived [");  
  Serial.print(topic);                                               
  Serial.print("] ");                                               
  String message;                                                   
  String tpc;                                                        
  for (int i = 0; i < length; i++) {                                 
    message = message + (char)payload[i];  
  }                                                                  
  Serial.println(message);                                           
//===============เพิ่ม22================================  
  if (String(topic) == "@msg/relay3") {
    if(message == "on"){
      digitalWrite(relay3,LOW);       
      lcd.setCursor(7, 2);                 
      lcd.print(" ON ");    
      client.publish("shadow/data/updata","(\"data\":(\"relay3\":\"on\"))");
      Serial.println("relay3 ON");}
    else  if(message == "off"){
      digitalWrite(relay3,HIGH);
      lcd.setCursor(7, 2);                 
      lcd.print("Off"); 
      client.publish("shadow/data/updata","(\"data\":(\"relay3\":\"off\"))");
      Serial.println("relay3 OFF");}              
   }
//===============เพิ่ม33================================  
   if (String(topic) == "@msg/relay4") {
    if(message == "on"){
      digitalWrite(relay4,LOW);       
      lcd.setCursor(7, 3);                 
      lcd.print(" ON ");    
      client.publish("shadow/data/updata","(\"data\":(\"relay4\":\"on\"))");
      Serial.println("relay4 ON");}
     else  if(message == "off"){
      digitalWrite(relay4,HIGH);
      lcd.setCursor(7, 3);                 
      lcd.print("Off"); 
      client.publish("shadow/data/updata","(\"data\":(\"relay4\":\"off\"))");
      Serial.println("relay4 OFF");}              
   }
//===============the end 22 and 33===============    
}   //==== end void callback ======
โปรมแกรม arduino ตอนที่ 2
//=================อันที่ 2===========================
//===============================================
void setup() { 
  lcd.begin(); 
  dht.setup(16);  // dht11 ต่ออยู่ที่ D0
  pinMode(relay1, OUTPUT);
  digitalWrite(relay1, HIGH); // relay ปิด
  pinMode(relay2, OUTPUT);
  digitalWrite(relay2, HIGH); // relay2 ปิด
  pinMode(relay3, OUTPUT);
  digitalWrite(relay3, HIGH); // relay3 ปิด
  pinMode(relay4, OUTPUT);
  digitalWrite(relay4, HIGH); // relay4 ปิด
//===============================================  
  Serial.begin(115200);    
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
    {
        Serial.print(".");
          lcd.setCursor(6, 1);           
          lcd.print("CONNECTED");                 
    }
  Serial.println("");
  Serial.println("WiFi connected");
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
//=================== แสดงผลหน้า จอ LCD ( SHOW LCD )==============
          lcd.begin();
          lcd.setCursor(0, 0);           
          lcd.print("Hum:");             
          lcd.setCursor(12, 0);         
          lcd.print("%");               
          lcd.setCursor(0, 1);          
          lcd.print("Tem:");             
          lcd.setCursor(12, 1);          
          lcd.print("C");             
          lcd.setCursor(14, 0);          
          lcd.print("= ");         
          lcd.setCursor(14, 1);           
          lcd.print("= ");          
          lcd.setCursor(0, 2);           
          lcd.print("SW01 = "); 
          lcd.setCursor(7, 2);                 
          lcd.print("Off"); 
          lcd.setCursor(0, 3);           
          lcd.print("SW02 = ");
          lcd.setCursor(7, 3);                 
          lcd.print("Off");         
//================================================ 
  }  //======end void setup ========
//=================END Void setup==================
void loop() {
    delay(dht.getMinimumSamplingPeriod());      
    float h = dht.getHumidity(); 
    float t = dht.getTemperature();
    if (isnan(h) || isnan(t))              
    {          
        Serial.println(F("Failed to read from DHT sensor!"));  
        return;                   
    }
        lcd.setCursor(5, 0);                     
        lcd.print(h);
        lcd.setCursor(5, 1);                      
        lcd.print(t);                                           
    if (!client.connected()) {                     
      reconnect();  
    }
//================================================
  if (autoHum == "on") {                  
    if (h < 66) {                         
      digitalWrite(relay1, LOW);          
          lcd.setCursor(16, 0);           
          lcd.print("ON ");         
    } else {                         
      digitalWrite(relay1, HIGH);         
          lcd.setCursor(16, 0);            
          lcd.print("OFF");          
    }
  }
//================================================
  if (autoTem == "on") {                  
    if (t > 32) {                        
      digitalWrite(relay2, LOW);           
          lcd.setCursor(16, 1);           
          lcd.print("ON ");         
    } else {                         
      digitalWrite(relay2, HIGH);         
          lcd.setCursor(16, 1);            
          lcd.print("OFF");          
    }
  }
//================================================  
  int r1 = digitalRead(relay1); //เป็นการอ่าน relay1 เป็น 0 หริอ 1 เพื่อส่งค่านี้ไป netpie
  int r2 = digitalRead(relay2); //เป็นการอ่าน relay2 เป็น 0 หริอ 1 เพื่อส่งค่านี้ไป netpie
  int r3 = digitalRead(relay3); //เป็นการอ่าน relay3 เป็น 0 หริอ 1 เพื่อส่งค่านี้ไป netpie
  int r4 = digitalRead(relay4); //เป็นการอ่าน relay4 เป็น 0 หริอ 1 เพื่อส่งค่านี้ไป netpie
//==================เป็นการส่ง data goto netpie====================================== 
String data = "{\"data\":{\"Humidity\":"+String(h) + 
              ",\"Temperature\":"+String(t)+ 
              ",\"relay1\":"+String(r1)+
              ",\"relay2\":"+String(r2)+
              ",\"relay3\":"+String(r3)+
              ",\"relay4\":"+String(r4)+
              "}}"; 
//================================================================================= 
  Serial.println(data);                       
  data.toCharArray(msg , (data.length() + 1)); 
  client.publish("@shadow/data/update", msg);  
  client.loop();                               
  delay(1000);
//================================================
}    //==== end void loop ===============
//================================================  
String getMsg(String topic_, String message_) 
     {
   //================================================ 
    if (topic_ == "@msg/relay1") {       //ตรวจเช็ค ตรงกับเงือนไขที่ตั้งไว้ไหม
      if (message_ == "on") {        //netpie["???"].publish("@msg/relay1","on")
          digitalWrite(relay1, LOW);     
          lcd.setCursor(16, 0);                
          lcd.print("ON ");               
          autoHum = "off";                       
     } else if (message_ == "off") { //netpie["???"].publish("@msg/relay1","off")
           digitalWrite(relay1, HIGH);     
          lcd.setCursor(16, 0);            
          lcd.print("OFF");             
       autoHum = "on";                       
     }
     }
     return autoHum;
   //================================================ 
    if (topic_ == "@msg/relay2") {       //ตรวจเช็ค ตรงกับเงือนไขที่ตั้งไว้ไหม
      if (message_ == "on") {      //netpie["???"].publish("@msg/relay2","on")
          digitalWrite(relay2, LOW);      
          lcd.setCursor(16, 1);                
          lcd.print("ON ");              
          autoTem = "off";                       
     } else if (message_ == "off") {    //netpie["???"].publish("@msg/relay2","off")
           digitalWrite(relay2, HIGH);     
          lcd.setCursor(16, 1);            
          lcd.print("OFF");            
       autoTem = "on";                       
     }
     }
     return autoTem;
//================================================      
   }  //=========== end String getMsg ============
//================================================
