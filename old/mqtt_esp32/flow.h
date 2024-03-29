class FLOW{
  public:
    int SENSOR;
    long currentMillis = 0;
    long previousMillis = 0;
    int interval = 1000;
    float calibrationFactor = 4.5;
    volatile byte pulseCount;
    byte pulse1Sec = 0;
    float flowRate;
    unsigned int flowMilliLitres;
    unsigned long totalMilliLitres;
    FLOW(int pin){
      SENSOR = pin;
      pinMode(SENSOR, INPUT_PULLUP);
      pulseCount = 0;
      flowRate = 0.0;
      flowMilliLitres = 0;
      totalMilliLitres = 0;
      previousMillis = 0;
    }
    void pulseCounter(){
      pulseCount++;
    };
    float read(){
      currentMillis = millis();
      if (currentMillis - previousMillis > interval) {
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
        Serial.print("Output Liquid Quantity: ");
        Serial.print(totalMilliLitres);
        Serial.print("mL / ");
        Serial.print(totalMilliLitres / 1000);
        Serial.println("L");*/
        return flowRate;
      }
    };
    float Totalwater(){
      return totalMilliLitres; //mL
    }
};