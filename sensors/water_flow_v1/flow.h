#include<Arduino.h>
//https://www.tinkercad.com/things/2SFCGiML7ln-exquisite-sango-gaaris/editel?sharecode=f7aYuvQvirKhQ4VUNplggjAJ1_eyCL-_qvRiTutcWfA
class FLOW{
  private:
    float calibrationFactor = 4.5;
    volatile byte pulseCount;
    byte pulse1Sec = 0;
    float flowRate;
    unsigned int flowMilliLitres;
    unsigned long totalMilliLitres;
  public:
    int t_pin;
    unsigned long previousMillis = 0;
    FLOW(int pin){
      t_pin = pin;
      pinMode(t_pin, INPUT_PULLUP);
      previousMillis = 0;
      pulseCount = 0;
      flowRate = 0.0;
      flowMilliLitres = 0;
      totalMilliLitres = 0;
    }
    void pulseCounter(){
      pulseCount++;
    }
    double readFlowrate(){
      pulse1Sec = pulseCount;
      pulseCount = 0;
      flowRate = ((1000.0 / (millis() - previousMillis)) * pulse1Sec) / calibrationFactor;
      previousMillis = millis();
      flowMilliLitres = (flowRate / 60) * 1000;
      totalMilliLitres += flowMilliLitres;
      /*Serial.print("Flow rate: ");
      Serial.print(int(flowRate));  // Print the integer part of the variable
      Serial.print("L/min");
      Serial.print("\t");       // Print tab space

      // Print the cumulative total of litres flowed since starting
      Serial.print("Output Liquid Quantity: ");
      Serial.print(totalMilliLitres);
      Serial.print("mL / ");
      Serial.print(totalMilliLitres / 1000);
      Serial.println("L");*/
      return flowRate;
    }
    float Totalwater(){
      return totalMilliLitres; //mL
    }


};

/*
// C++ code
//
const int ledPin = LED_BUILTIN;
int ledState = LOW;
unsigned long previousMillis2 = 0;

class test{
  private:
  public:
  	long previousMillis = 0;
    int cal(){
      previousMillis = millis();
      return previousMillis;
    }
};

long currentMillis = 0; //flow
long previousMill = 0; //flow
long currentMillis2 = 0;
long previousMill2 = 0;
int interval = 1000; // 1 Sec
test Objtest1;
void setup()
{
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
}

void loop()
{
  flow();
  onoff();
}

void flow(){
  currentMillis = millis();
  if (currentMillis - previousMill > interval) {
    Objtest1.previousMillis = previousMill;
    Serial.println(Objtest1.cal());
  }
}
void onoff(){
  unsigned long currentMillis2 = millis();
  if (currentMillis2 - previousMillis2 >= interval) {
    // save the last time you blinked the LED
    previousMillis2 = currentMillis2;
    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW){
      ledState = HIGH;
    } else{
      ledState = LOW;
    }
    digitalWrite(ledPin, ledState);
  }
}
*/
