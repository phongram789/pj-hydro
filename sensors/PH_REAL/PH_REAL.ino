//pH
#define PHPIN 39
float voltagePH,phValue;
float acidVoltage = 1810;
float neutralVoltage = 1370;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  

}

void loop() {
  // put your main code here, to run repeatedly:
  PH();
  Serial.print("pH: ");
  Serial.println(phValue);

}
void PH(){
  static unsigned long timepoint = millis();
  if(millis()-timepoint>1000U){
    voltagePH = analogRead(PHPIN)/4095.0*3300; //read the voltage  
    float slope = (7.0-4.0)/((neutralVoltage-1500)/3.0 - (acidVoltage-1500)/3.0); //two point: (_NautralVoltage,7.0),(_acidVoltage,4.0)
    float intercept = 7.0 - slope*(neutralVoltage-1500)/3.0;
    phValue = slope*(voltagePH-1500)/3.0+intercept; //y = k*x +b
  }
  
};
