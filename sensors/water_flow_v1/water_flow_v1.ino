#include"flow.h"
#define FLOW_P  27
FLOW flowA(FLOW_P);

long currentMillis = 0;
long previousMill = 0;
int interval = 1000; // 1 Sec

void IRAM_ATTR pulseCounterFlowA(){
  flowA.pulseCounter();
}

void setup()
{
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(flowA.t_pin), pulseCounterFlowA, FALLING);
}

void loop(){
  flow();
}
void flow(){
  currentMillis = millis();
  if (currentMillis - previousMill > interval) {
    previousMill = flowA.previousMillis;
    flowA.readFlowrate();
  }
}
