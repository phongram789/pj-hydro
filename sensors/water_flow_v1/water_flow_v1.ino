#include "flow.h";
#define FLOW_P  26
FLOW flowA(FLOW_P,7.5);
unsigned long time_1 = 0; //
unsigned long time_2 = 0; //
unsigned long currentMillis = 0;
unsigned long previousMill = 0;
int interval = 1000; // 1 Sec

void IRAM_ATTR pulseCounterFlowA(){
  flowA.pulseCounter();
}

void setup()
{
  Serial.begin(115200);

  //pinMode(SENSOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(flowA.t_pin), pulseCounterFlowA, FALLING);
}

void loop(){
  currentMillis = millis();
  flow();
  someOtherFunction();
  PH();
}
void flow(){
  if (currentMillis - previousMill > interval) {
    flowA.readFlowrate(currentMillis);
    previousMill = currentMillis;
  }
}
void someOtherFunction() {
  static unsigned long timepoint = 0;
  if (currentMillis - timepoint >= 1000U) {
    timepoint = currentMillis;
    //Serial.println(timepoint);
  }
  if (currentMillis - time_2 >= interval*2) {
    time_2 = currentMillis;
    //Serial.println(time_2);
  }
}
void PH(){
  static unsigned long timepoint = 0;
  if(currentMillis-timepoint >= 1000U){
    timepoint = currentMillis;
    /*Serial.print("pH: ");
  	Serial.println(timepoint);*/
  }
  
  
};
