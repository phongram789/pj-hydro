#include <SoftwareSerial.h>
SoftwareSerial Com(1,3);

void setup() {
  Serial1.begin(115200);
  Com.begin(9600);  
  Serial1.println("helloo setup");
}

void loop() {
  Serial1.println("helloo");
  static int value1,value2;
  static char SerialBuffer[4];
  static char SerialData;
  static byte SerialState = 0;
  static byte SerialIndex = 0;
  static double Timeout = millis();
  
  if(Com.available()){
    SerialData = Com.read();  

    switch(SerialState){
      
    
      case 0: if(SerialData=='#'){
                SerialState = 1; 
                SerialIndex = 0;
              }
              break;
              
      case 1: if(SerialData==':'){
                SerialBuffer[SerialIndex] = '\0';
                value1 = atoi(SerialBuffer);
                SerialIndex = 0;
                SerialState = 2;  
                
              }else{
                SerialBuffer[SerialIndex] = SerialData;
                if(++SerialIndex==4){
                  SerialState = 0;
                }
              }
              break;
              
      case 2: if(SerialData==';'){
                SerialBuffer[SerialIndex] = '\0';
                value2 = atoi(SerialBuffer);
  
                Serial.print("Rx receive value1: ");
                Serial.print(value1);
                Serial.print(" value2: ");
                Serial.println(value2);
                Timeout = millis();
                SerialState = 0;
              }else{
                SerialBuffer[SerialIndex] = SerialData;
                if(++SerialIndex==4){
                  SerialState = 0;
                }  
              }
              break;
    }
  }

  if((millis()-Timeout)>200){
    Timeout = millis();
    Serial.println("การเชื่อมต่อมีปัญหา");
  }
}
