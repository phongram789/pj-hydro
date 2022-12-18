#include"flow.h"
#define FLOW_P  27
FLOW flowA(FLOW_P);

unsigned long currentMillis = 0; // millis()
unsigned long TIME_FLOW = 0; // previousMillis for water flow sensors function flow()
const long interval = 1000;

void IRAM_ATTR pulseCounterFlowA(){
  flowA.pulseCounter();
}

void setup()
{
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(flowA.t_pin), pulseCounterFlowA, FALLING);
}

void loop(){
  currentMillis = millis(); // only one millis()
  flow();
  resetTimeMillis(); //last line in void loop to reset currentMillis;
}

void flow(){
  if (currentMillis - TIME_FLOW > interval) { // currentMillis - TIME_FLOW > interval
    TIME_FLOW = flowA.previousMillis;
    flowA.readFlowrate();
  }
}

void resetTimeMillis(){
  if (currentMillis >= 4294967294) {
    //maximum value it can hold (4,294,967,295 milliseconds, or approximately 49.7 days)
    // reset the value to 0
    currentMillis = 0;
  }
}
