swInput swAutoPump(25);
//swInput swManPump(25);

//swInput swAutoGrowlight(25);
//swInput swManGrowlight(25);

//swInput swAutoValve(25);
//swInput swManValve(25);

//swInput swAutopHdown(25);
//swInput swManpHdown(25);

//swInput swAutopHdown(25);

#define DHTPIN 2
#define WATERFLOWIN 27
#define WATERFLOWOUT 33

long currentMillis = 0;
long previousMillis = 0;
int interval = 1000;
boolean ledState = LOW;
float calibrationFactor = 4.5;
volatile byte pulseCount;
byte pulse1Sec = 0;
float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;
