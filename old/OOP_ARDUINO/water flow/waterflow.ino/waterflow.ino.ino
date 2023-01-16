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
    void pulseCounter(){ //void IRAM_ATTR pulseCounter()
      pulseCount++;
    };
    //void pulseCounter();
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
};
/*void FLOW::pulseCounter(){
  pulseCount++;
}*/

#define SENSOR1  27
#define SENSOR2  26
/*long currentMillis = 0;
long previousMillis = 0;
int interval = 1000;

float calibrationFactor = 4.5;
volatile byte pulseCount;
byte pulse1Sec = 0;
float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;*/

/*void IRAM_ATTR pulseCounter()
{
  pulseCount++;
}*/
FLOW myobj1(SENSOR1);
FLOW myobj2(SENSOR2);

/*void FLOW::pulseCounter(){
  pulseCount++;
}*/

void pulse_1(){
  myobj1.pulseCounter();
}
void pulse_2(){
  myobj2.pulseCounter();
}

float flowratevalue;
float flowratevalue2;
unsigned long time1 = 0;
#define INTERVAL1 1200

void setup()
{
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(myobj1.SENSOR), pulse_1, FALLING);
  attachInterrupt(digitalPinToInterrupt(myobj2.SENSOR), pulse_2, FALLING);
  /*pinMode(SENSOR, INPUT_PULLUP);
  pulseCount = 0;
  flowRate = 0.0;
  flowMilliLitres = 0;
  totalMilliLitres = 0;
  previousMillis = 0;
  attachInterrupt(digitalPinToInterrupt(SENSOR), pulseCounter, FALLING);*/
}

void loop()
{

  if(millis() - time1 > INTERVAL1){
    time1 = millis();
    flowratevalue = myobj1.read();
    flowratevalue2 = myobj2.read();
  }
  Serial.print("Flow rate1: ");
  Serial.print(flowratevalue);
  Serial.println("L/min");
  Serial.print("Flow rate2: ");
  Serial.print(flowratevalue2);
  Serial.println("L/min");



 
  /*currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    pulse1Sec = pulseCount;
    pulseCount = 0;
    flowRate = ((1000.0 / (millis() - previousMillis)) * pulse1Sec) / calibrationFactor;
    previousMillis = millis();
    flowMilliLitres = (flowRate / 60) * 1000;
    totalMilliLitres += flowMilliLitres;
    Serial.print("Flow rate: ");
    Serial.print(int(flowRate));  // Print the integer part of the variable
    Serial.print("L/min");
    Serial.print("\t");       // Print tab space
    Serial.print("Output Liquid Quantity: ");
    Serial.print(totalMilliLitres);
    Serial.print("mL / ");
    Serial.print(totalMilliLitres / 1000);
    Serial.println("L");
  }*/
}